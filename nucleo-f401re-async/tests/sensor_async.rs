#![no_std]
#![no_main]

use defmt_rtt as _;

use embassy_stm32::i2c;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_time::Delay;
use vl53l8cx_async::{PowerMode, RangingMode, Resolution, TargetOrder, Vl53l8cx};

// I2C1 interrupt bindings for async operation
embassy_stm32::bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
});

type AsyncSensor<'d> = Vl53l8cx<
    embassy_stm32::i2c::I2c<'d, embassy_stm32::mode::Async>,
    Delay,
    Output<'d>,
    embassy_stm32::gpio::Input<'d>,
>;

fn board_init_async() -> AsyncSensor<'static> {
    // Clock config: HSI 16 MHz -> PLL -> 84 MHz SYSCLK
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = true;
        config.rcc.pll_src = PllSource::HSI;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV16,   // 16 MHz / 16 = 1 MHz
            mul: PllMul::MUL336,        // 1 MHz * 336 = 336 MHz VCO
            divp: Some(PllPDiv::DIV4),  // 336 / 4 = 84 MHz SYSCLK
            divq: Some(PllQDiv::DIV7),  // 336 / 7 = 48 MHz (USB)
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;   // 84 MHz
        config.rcc.apb1_pre = APBPrescaler::DIV2;   // 42 MHz
        config.rcc.apb2_pre = APBPrescaler::DIV1;   // 84 MHz
    }

    let p = embassy_stm32::init(config);

    // Power-up sequence
    let mut pwr_en_c = Output::new(p.PA7, Level::Low, Speed::Low);
    cortex_m::asm::delay(84_000 * 50);
    pwr_en_c.set_high();
    cortex_m::asm::delay(84_000 * 50);
    // Keep pwr_en_c alive (leak it so it doesn't get dropped)
    core::mem::forget(pwr_en_c);

    // Reset pin (LPn center) — active-high reset: assert high, delay, deassert low
    let mut lp_c = Output::new(p.PB0, Level::Low, Speed::Low);
    lp_c.set_high(); // assert reset
    cortex_m::asm::delay(84_000 * 10);
    lp_c.set_low(); // deassert reset
    cortex_m::asm::delay(84_000 * 100);

    let int_c = embassy_stm32::gpio::Input::new(p.PA4, Pull::Up);

    // Async I2C1: PB8 = SCL, PB9 = SDA @ 400 kHz
    // Read PCLK1 from RCC registers directly for diagnostics
    let cfgr = unsafe { core::ptr::read_volatile(0x4002_3808 as *const u32) };
    let ppre1 = (cfgr >> 10) & 0x7;
    defmt::info!("RCC_CFGR: {:#010x}, PPRE1 bits: {}", cfgr, ppre1);
    let i2c = i2c::I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH0,
        embassy_stm32::time::Hertz(400_000),
        i2c::Config::default(),
    );

    let sensor = Vl53l8cx::new(i2c, Delay)
        .with_reset_pin_active_high(lp_c)
        .with_interrupt_pin_active_low(int_c);

    sensor
}

#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> AsyncSensor<'static> {
        board_init_async()
    }

    #[test]
    #[timeout(5)]
    async fn test_sensor_alive(mut sensor: AsyncSensor<'static>) {
        defmt::assert!(
            sensor.is_alive().await.unwrap(),
            "sensor.is_alive failed, check X-NUCLEO-53L8A1 is connected and I2C jumpers are set"
        );
    }

    #[test]
    #[timeout(10)]
    async fn test_init(mut sensor: AsyncSensor<'static>) {
        sensor.init().await.unwrap();
    }

    #[test]
    #[timeout(10)]
    async fn test_set_resolution(mut sensor: AsyncSensor<'static>) {
        sensor.init().await.unwrap();

        sensor.set_resolution(Resolution::Res4x4).await.unwrap();
        assert_eq!(sensor.get_resolution().await.unwrap(), Resolution::Res4x4);

        sensor.set_resolution(Resolution::Res8x8).await.unwrap();
        assert_eq!(sensor.get_resolution().await.unwrap(), Resolution::Res8x8);
    }

    #[test]
    #[timeout(10)]
    async fn test_ranging_4x4(mut sensor: AsyncSensor<'static>) {
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();
        sensor.set_ranging_frequency_hz(5).await.unwrap();
        sensor.start_ranging().await.unwrap();

        sensor.poll_data_ready().await.unwrap();

        let data = sensor.get_ranging_data().await.unwrap();
        for (i, &d) in data.distance_mm[..16].iter().enumerate() {
            defmt::assert!(
                d > 10 && d < 4000,
                "zone {} distance {} mm out of range",
                i,
                d
            );
        }

        sensor.stop_ranging().await.unwrap();
    }

    #[test]
    #[timeout(10)]
    async fn test_ranging_8x8(mut sensor: AsyncSensor<'static>) {
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res8x8).await.unwrap();
        sensor.set_ranging_frequency_hz(5).await.unwrap();
        sensor.start_ranging().await.unwrap();

        sensor.poll_data_ready().await.unwrap();

        let data = sensor.get_ranging_data().await.unwrap();
        for (i, &d) in data.distance_mm[..64].iter().enumerate() {
            defmt::assert!(
                d > 10 && d < 4000,
                "zone {} distance {} mm out of range",
                i,
                d
            );
        }

        sensor.stop_ranging().await.unwrap();
    }

    #[test]
    #[timeout(10)]
    async fn test_ambient_light(mut sensor: AsyncSensor<'static>) {
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();
        sensor.set_ranging_frequency_hz(5).await.unwrap();
        sensor.start_ranging().await.unwrap();

        sensor.poll_data_ready().await.unwrap();

        let data = sensor.get_ranging_data().await.unwrap();
        for (i, &a) in data.ambient_per_spad[..16].iter().enumerate() {
            defmt::assert!(a > 0, "zone {} ambient_per_spad is zero", i);
        }

        sensor.stop_ranging().await.unwrap();
    }

    #[test]
    #[timeout(10)]
    async fn test_ranging_frequency(mut sensor: AsyncSensor<'static>) {
        sensor.init().await.unwrap();

        sensor.set_ranging_frequency_hz(5).await.unwrap();
        defmt::assert_eq!(sensor.get_ranging_frequency_hz().await.unwrap(), 5);

        sensor.set_ranging_frequency_hz(15).await.unwrap();
        defmt::assert_eq!(sensor.get_ranging_frequency_hz().await.unwrap(), 15);

        sensor.set_ranging_frequency_hz(30).await.unwrap();
        defmt::assert_eq!(sensor.get_ranging_frequency_hz().await.unwrap(), 30);
    }

    #[test]
    #[timeout(10)]
    async fn test_target_order(mut sensor: AsyncSensor<'static>) {
        sensor.init().await.unwrap();

        sensor.set_target_order(TargetOrder::Closest).await.unwrap();
        assert_eq!(sensor.get_target_order().await.unwrap(), TargetOrder::Closest);

        sensor.set_target_order(TargetOrder::Strongest).await.unwrap();
        assert_eq!(sensor.get_target_order().await.unwrap(), TargetOrder::Strongest);
    }

    #[test]
    #[timeout(10)]
    async fn test_ranging_mode(mut sensor: AsyncSensor<'static>) {
        sensor.init().await.unwrap();

        sensor.set_ranging_mode(RangingMode::Continuous).await.unwrap();
        assert_eq!(sensor.get_ranging_mode().await.unwrap(), RangingMode::Continuous);

        sensor.set_ranging_mode(RangingMode::Autonomous).await.unwrap();
        assert_eq!(sensor.get_ranging_mode().await.unwrap(), RangingMode::Autonomous);
    }

    #[test]
    #[timeout(10)]
    async fn test_integration_time(mut sensor: AsyncSensor<'static>) {
        sensor.init().await.unwrap();

        sensor.set_integration_time_ms(20).await.unwrap();
        defmt::assert_eq!(sensor.get_integration_time_ms().await.unwrap(), 20);

        sensor.set_integration_time_ms(50).await.unwrap();
        defmt::assert_eq!(sensor.get_integration_time_ms().await.unwrap(), 50);
    }

    #[test]
    #[timeout(10)]
    async fn test_sharpener_percent(mut sensor: AsyncSensor<'static>) {
        sensor.init().await.unwrap();

        sensor.set_sharpener_percent(25).await.unwrap();
        let v = sensor.get_sharpener_percent().await.unwrap();
        defmt::assert!(v >= 24 && v <= 26, "expected ~25, got {}", v);

        sensor.set_sharpener_percent(75).await.unwrap();
        let v = sensor.get_sharpener_percent().await.unwrap();
        defmt::assert!(v >= 74 && v <= 76, "expected ~75, got {}", v);
    }

    #[test]
    #[timeout(10)]
    async fn test_power_mode(mut sensor: AsyncSensor<'static>) {
        sensor.init().await.unwrap();

        assert_eq!(sensor.get_power_mode().await.unwrap(), PowerMode::Wakeup);

        sensor.set_power_mode(PowerMode::Sleep).await.unwrap();
        assert_eq!(sensor.get_power_mode().await.unwrap(), PowerMode::Sleep);

        sensor.set_power_mode(PowerMode::Wakeup).await.unwrap();
        assert_eq!(sensor.get_power_mode().await.unwrap(), PowerMode::Wakeup);
    }
}
