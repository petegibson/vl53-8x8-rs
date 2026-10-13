#![no_std]
#![no_main]

//! On-target tests for the async (DMA) I2C path on the STM32F412RE board.
//!
//! These are deliberately **variant-agnostic**: every assertion holds for both
//! the VL53L5CX and the VL53L8CX, so the same suite passes whichever sensor
//! module is plugged in. The headline check is that `init()` — which streams
//! the ~84 KB firmware over async I2C DMA and reads back the verification —
//! completes, confirming the async backend works on this I2C v1 part.

use defmt_rtt as _;
// Panic handler is provided by embedded-test (do not also link panic-probe).

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, dma, i2c, peripherals};
use embassy_time::Delay;
use vl53_8x8::{PowerMode, RangingMode, Resolution, TargetOrder, Variant};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    DMA1_STREAM7 => dma::InterruptHandler<peripherals::DMA1_CH7>;
    DMA1_STREAM0 => dma::InterruptHandler<peripherals::DMA1_CH0>;
});

type I2cInst = I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>;
type Sensor = vl53_8x8::Sensor<I2cInst, Delay>;

const CYCLES_PER_MS: u32 = 96_000; // SYSCLK = 96 MHz

/// Bring up clocks, run the sensor power-on sequence, and build the async-I2C
/// sensor. Returns an uninitialised sensor; each test calls `init()` itself
/// (embedded-test resets the chip between tests, so this runs fresh per test).
fn board_init() -> Sensor {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV6,
            mul: PllMul::MUL144,
            divp: Some(PllPDiv::DIV2), // 96 MHz
            divq: Some(PllQDiv::DIV4), // 48 MHz
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
    }

    let p = embassy_stm32::init(config);

    // Power-on sequence (busy-wait delays; #[init] is synchronous).
    let mut pwr_en = Output::new(p.PD2, Level::High, Speed::Low); // active low
    let mut i2c_reset = Output::new(p.PB8, Level::High, Speed::Low); // active high
    let mut tof_lp = Output::new(p.PB9, Level::Low, Speed::Low); // gate 2nd sensor

    pwr_en.set_high();
    i2c_reset.set_high();
    cortex_m::asm::delay(CYCLES_PER_MS * 500);
    pwr_en.set_low();
    cortex_m::asm::delay(CYCLES_PER_MS * 100);
    i2c_reset.set_low();
    cortex_m::asm::delay(CYCLES_PER_MS * 1);
    tof_lp.set_low();

    // Keep the control pins driven for the duration of the test.
    core::mem::forget(pwr_en);
    core::mem::forget(i2c_reset);
    core::mem::forget(tof_lp);

    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = Hertz(1_000_000);
    i2c_config.scl_pullup = true;
    i2c_config.sda_pullup = true;
    let i2c = I2c::new(
        p.I2C1, p.PB6, p.PB7, p.DMA1_CH7, p.DMA1_CH0, Irqs, i2c_config,
    );

    vl53_8x8::Sensor::new(i2c, Delay)
}

#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Sensor {
        board_init()
    }

    /// Basic async-I2C read: the sensor answers on the bus.
    #[test]
    #[timeout(10)]
    async fn async_i2c_alive(mut sensor: Sensor) {
        match sensor.is_alive().await {
            Ok(alive) => defmt::assert!(alive, "no sensor detected on async I2C bus"),
            Err(e) => {
                defmt::error!("is_alive error: {}", defmt::Debug2Format(&e));
                panic!("is_alive failed on async I2C");
            }
        }
    }

    /// The strong async-DMA test: `init()` streams ~84 KB of firmware over
    /// async I2C and reads back the verification. Variant-agnostic.
    #[test]
    #[timeout(15)]
    async fn init_uploads_firmware_over_async_i2c(mut sensor: Sensor) {
        sensor.init().await.unwrap();
        let v = sensor.variant();
        defmt::info!("detected variant: {}", defmt::Debug2Format(&v));
        defmt::assert!(matches!(v, Variant::L5 | Variant::L8));
    }

    #[test]
    #[timeout(15)]
    async fn set_resolution(mut sensor: Sensor) {
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();
        assert_eq!(sensor.get_resolution().await.unwrap(), Resolution::Res4x4);
        sensor.set_resolution(Resolution::Res8x8).await.unwrap();
        assert_eq!(sensor.get_resolution().await.unwrap(), Resolution::Res8x8);
    }

    /// Async multi-byte DMA reads of a full 4x4 frame. Distances are scene-
    /// dependent, so only magnitude is sanity-checked (no fixed target needed).
    #[test]
    #[timeout(15)]
    async fn ranging_4x4_async(mut sensor: Sensor) {
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();
        sensor.set_ranging_frequency_hz(5).await.unwrap();
        sensor.start_ranging().await.unwrap();
        sensor.poll_data_ready().await.unwrap();

        let data = sensor.get_ranging_data().await.unwrap();
        for (i, &d) in data.distance_mm[..16].iter().enumerate() {
            defmt::assert!(d < 8000, "zone {} distance {} mm implausible (bad read?)", i, d);
        }
        sensor.stop_ranging().await.unwrap();
    }

    /// Larger async DMA frame (64 zones), supported by both variants.
    #[test]
    #[timeout(15)]
    async fn ranging_8x8_async(mut sensor: Sensor) {
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res8x8).await.unwrap();
        sensor.set_ranging_frequency_hz(5).await.unwrap();
        sensor.start_ranging().await.unwrap();
        sensor.poll_data_ready().await.unwrap();

        let data = sensor.get_ranging_data().await.unwrap();
        for (i, &d) in data.distance_mm[..64].iter().enumerate() {
            defmt::assert!(d < 8000, "zone {} distance {} mm implausible (bad read?)", i, d);
        }
        sensor.stop_ranging().await.unwrap();
    }

    #[test]
    #[timeout(15)]
    async fn ambient_light(mut sensor: Sensor) {
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();
        sensor.set_ranging_frequency_hz(5).await.unwrap();
        sensor.start_ranging().await.unwrap();
        sensor.poll_data_ready().await.unwrap();

        let data = sensor.get_ranging_data().await.unwrap();
        // Per-zone ambient magnitude is lighting/variant dependent (and rounds
        // to 0 after scaling in low light), so only require that the ambient
        // block transferred and at least one zone reports light.
        let nonzero = data.ambient_per_spad[..16].iter().filter(|&&a| a > 0).count();
        defmt::info!("ambient: {}/16 zones non-zero", nonzero);
        defmt::assert!(nonzero > 0, "no zone reported ambient light (sensor in the dark?)");
        sensor.stop_ranging().await.unwrap();
    }

    #[test]
    #[timeout(15)]
    async fn ranging_frequency(mut sensor: Sensor) {
        sensor.init().await.unwrap();
        sensor.set_ranging_frequency_hz(5).await.unwrap();
        defmt::assert_eq!(sensor.get_ranging_frequency_hz().await.unwrap(), 5);
        sensor.set_ranging_frequency_hz(15).await.unwrap();
        defmt::assert_eq!(sensor.get_ranging_frequency_hz().await.unwrap(), 15);
    }

    #[test]
    #[timeout(15)]
    async fn integration_time(mut sensor: Sensor) {
        sensor.init().await.unwrap();
        sensor.set_integration_time_ms(20).await.unwrap();
        defmt::assert_eq!(sensor.get_integration_time_ms().await.unwrap(), 20);
        sensor.set_integration_time_ms(50).await.unwrap();
        defmt::assert_eq!(sensor.get_integration_time_ms().await.unwrap(), 50);
    }

    #[test]
    #[timeout(15)]
    async fn target_order(mut sensor: Sensor) {
        sensor.init().await.unwrap();
        sensor.set_target_order(TargetOrder::Closest).await.unwrap();
        assert_eq!(sensor.get_target_order().await.unwrap(), TargetOrder::Closest);
        sensor.set_target_order(TargetOrder::Strongest).await.unwrap();
        assert_eq!(sensor.get_target_order().await.unwrap(), TargetOrder::Strongest);
    }

    #[test]
    #[timeout(15)]
    async fn ranging_mode(mut sensor: Sensor) {
        sensor.init().await.unwrap();
        sensor.set_ranging_mode(RangingMode::Continuous).await.unwrap();
        assert_eq!(sensor.get_ranging_mode().await.unwrap(), RangingMode::Continuous);
        sensor.set_ranging_mode(RangingMode::Autonomous).await.unwrap();
        assert_eq!(sensor.get_ranging_mode().await.unwrap(), RangingMode::Autonomous);
    }

    #[test]
    #[timeout(15)]
    async fn sharpener_percent(mut sensor: Sensor) {
        sensor.init().await.unwrap();
        sensor.set_sharpener_percent(25).await.unwrap();
        let v = sensor.get_sharpener_percent().await.unwrap();
        defmt::assert!((24..=26).contains(&v), "expected ~25, got {}", v);
        sensor.set_sharpener_percent(75).await.unwrap();
        let v = sensor.get_sharpener_percent().await.unwrap();
        defmt::assert!((74..=76).contains(&v), "expected ~75, got {}", v);
    }

    /// Sleep/Wakeup are common to both variants (deep-sleep is L8-only and is
    /// intentionally not exercised, to keep the suite variant-agnostic).
    #[test]
    #[timeout(15)]
    async fn power_mode_sleep_wakeup(mut sensor: Sensor) {
        sensor.init().await.unwrap();
        assert_eq!(sensor.get_power_mode().await.unwrap(), PowerMode::Wakeup);
        sensor.set_power_mode(PowerMode::Sleep).await.unwrap();
        assert_eq!(sensor.get_power_mode().await.unwrap(), PowerMode::Sleep);
        sensor.set_power_mode(PowerMode::Wakeup).await.unwrap();
        assert_eq!(sensor.get_power_mode().await.unwrap(), PowerMode::Wakeup);
    }
}
