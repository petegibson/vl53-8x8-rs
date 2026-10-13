#![no_std]
#![no_main]

use defmt_rtt as _;

use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_time::Delay;
use vl53_8x8::{PowerMode, RangingMode, Resolution, TargetOrder};

// --- I2C mode (default) ---

#[cfg(not(feature = "spi-bus"))]
mod bus_setup {
    use super::*;
    use embassy_stm32::i2c;

    embassy_stm32::bind_interrupts!(struct Irqs {
        I2C1_EV => i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
        I2C1_ER => i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
    });

    type I2cInst = i2c::I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>;

    #[cfg(feature = "blocking-i2c")]
    pub type Sensor = vl53_8x8::Sensor<
        vl53_8x8::BlockingI2c<I2cInst>,
        Delay,
        Output<'static>,
        embassy_stm32::gpio::Input<'static>,
    >;

    #[cfg(not(feature = "blocking-i2c"))]
    pub type Sensor = vl53_8x8::Sensor<
        I2cInst,
        Delay,
        Output<'static>,
        embassy_stm32::gpio::Input<'static>,
    >;

    pub fn init_bus(p: &mut embassy_stm32::Peripherals) -> Sensor {
        let _i2c_rst_c = Output::new(
            unsafe { core::ptr::read(&raw const p.PB3) },
            Level::Low,
            Speed::Low,
        );
        core::mem::forget(_i2c_rst_c);

        let int_c = embassy_stm32::gpio::Input::new(
            unsafe { core::ptr::read(&raw const p.PA4) },
            Pull::Up,
        );

        let mut i2c_config = i2c::Config::default();
        i2c_config.frequency = embassy_stm32::time::Hertz(400_000);
        let i2c = i2c::I2c::new(
            unsafe { core::ptr::read(&raw const p.I2C1) },
            unsafe { core::ptr::read(&raw const p.PB8) },
            unsafe { core::ptr::read(&raw const p.PB9) },
            Irqs,
            unsafe { core::ptr::read(&raw const p.DMA1_CH6) },
            unsafe { core::ptr::read(&raw const p.DMA1_CH0) },
            i2c_config,
        );

        let mut lp_c = Output::new(
            unsafe { core::ptr::read(&raw const p.PB0) },
            Level::Low,
            Speed::Low,
        );
        // LPn LOW = sensor in reset, HIGH = sensor enabled (datasheet).
        // Pulse reset then leave HIGH so sensor is ready for init().
        lp_c.set_low();
        cortex_m::asm::delay(84_000 * 10);
        lp_c.set_high();
        cortex_m::asm::delay(84_000 * 100);

        #[cfg(feature = "blocking-i2c")]
        let sensor = vl53_8x8::Sensor::new(vl53_8x8::BlockingI2c(i2c), Delay);
        #[cfg(not(feature = "blocking-i2c"))]
        let sensor = vl53_8x8::Sensor::new(i2c, Delay);

        sensor
            .with_reset_pin_active_low(lp_c)
            .with_interrupt_pin_active_low(int_c)
    }
}

// --- SPI mode ---

#[cfg(feature = "spi-bus")]
mod bus_setup {
    use super::*;
    use embassy_stm32::spi;
    use embedded_hal_bus::spi::ExclusiveDevice;

    type SpiDev = ExclusiveDevice<
        spi::Spi<'static, embassy_stm32::mode::Async, embassy_stm32::spi::mode::Master>,
        Output<'static>,
        embedded_hal_bus::spi::NoDelay,
    >;

    pub type Sensor = vl53_8x8::Sensor<
        vl53_8x8::Spi<SpiDev>,
        Delay,
        Output<'static>,
        embassy_stm32::gpio::Input<'static>,
    >;

    pub fn init_bus(p: &mut embassy_stm32::Peripherals) -> Sensor {
        let int_c = embassy_stm32::gpio::Input::new(
            unsafe { core::ptr::read(&raw const p.PA4) },
            Pull::Up,
        );

        let mut spi_config = spi::Config::default();
        spi_config.mode = spi::Mode {
            polarity: spi::Polarity::IdleHigh,
            phase: spi::Phase::CaptureOnSecondTransition,
        };
        spi_config.frequency = embassy_stm32::time::Hertz(1_000_000);

        let spi = spi::Spi::new(
            unsafe { core::ptr::read(&raw const p.SPI1) },
            unsafe { core::ptr::read(&raw const p.PB3) },  // SCK
            unsafe { core::ptr::read(&raw const p.PB5) },  // MOSI
            unsafe { core::ptr::read(&raw const p.PB4) },  // MISO
            unsafe { core::ptr::read(&raw const p.DMA2_CH3) }, // TX
            unsafe { core::ptr::read(&raw const p.DMA2_CH0) }, // RX
            spi_config,
        );

        let cs = Output::new(
            unsafe { core::ptr::read(&raw const p.PB6) },
            Level::High,
            Speed::VeryHigh,
        );
        let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

        let mut lp_c = Output::new(
            unsafe { core::ptr::read(&raw const p.PB0) },
            Level::Low,
            Speed::Low,
        );
        // LPn LOW = sensor in reset, HIGH = sensor enabled (datasheet).
        // Pulse reset then leave HIGH so sensor is ready for init().
        lp_c.set_low();
        cortex_m::asm::delay(84_000 * 10);
        lp_c.set_high();
        cortex_m::asm::delay(84_000 * 100);

        vl53_8x8::Sensor::new_spi(spi_device, Delay)
            .with_reset_pin_active_low(lp_c)
            .with_interrupt_pin_active_low(int_c)
    }
}

use bus_setup::Sensor;

fn board_init_async() -> Sensor {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = true;
        config.rcc.pll_src = PllSource::HSI;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV16,
            mul: PllMul::MUL336,
            divp: Some(PllPDiv::DIV4),
            divq: Some(PllQDiv::DIV7),
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
    }

    let mut p = embassy_stm32::init(config);

    // Power-up sequence
    let mut pwr_en_c = Output::new(
        unsafe { core::ptr::read(&raw const p.PA7) },
        Level::Low,
        Speed::Low,
    );
    pwr_en_c.set_low();
    cortex_m::asm::delay(84_000 * 50);
    pwr_en_c.set_high();
    cortex_m::asm::delay(84_000 * 50);
    core::mem::forget(pwr_en_c);

    bus_setup::init_bus(&mut p)
}

#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Sensor {
        board_init_async()
    }

    #[test]
    #[timeout(5)]
    async fn test_sensor_alive(mut sensor: Sensor) {
        match sensor.is_alive().await {
            Ok(alive) => defmt::assert!(
                alive,
                "sensor.is_alive returned false, check X-NUCLEO-53L8A1 is connected"
            ),
            Err(e) => {
                defmt::error!("is_alive error: {}", defmt::Debug2Format(&e));
                panic!("is_alive failed");
            }
        }
    }

    #[test]
    #[timeout(10)]
    async fn test_init(mut sensor: Sensor) {
        sensor.init().await.unwrap();
    }

    #[test]
    #[timeout(10)]
    async fn test_set_resolution(mut sensor: Sensor) {
        sensor.init().await.unwrap();

        sensor.set_resolution(Resolution::Res4x4).await.unwrap();
        assert_eq!(sensor.get_resolution().await.unwrap(), Resolution::Res4x4);

        sensor.set_resolution(Resolution::Res8x8).await.unwrap();
        assert_eq!(sensor.get_resolution().await.unwrap(), Resolution::Res8x8);
    }

    #[test]
    #[timeout(10)]
    async fn test_ranging_4x4(mut sensor: Sensor) {
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
    async fn test_ranging_8x8(mut sensor: Sensor) {
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
    async fn test_ambient_light(mut sensor: Sensor) {
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
    async fn test_ranging_frequency(mut sensor: Sensor) {
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
    async fn test_target_order(mut sensor: Sensor) {
        sensor.init().await.unwrap();

        sensor.set_target_order(TargetOrder::Closest).await.unwrap();
        assert_eq!(sensor.get_target_order().await.unwrap(), TargetOrder::Closest);

        sensor.set_target_order(TargetOrder::Strongest).await.unwrap();
        assert_eq!(sensor.get_target_order().await.unwrap(), TargetOrder::Strongest);
    }

    #[test]
    #[timeout(10)]
    async fn test_ranging_mode(mut sensor: Sensor) {
        sensor.init().await.unwrap();

        sensor.set_ranging_mode(RangingMode::Continuous).await.unwrap();
        assert_eq!(sensor.get_ranging_mode().await.unwrap(), RangingMode::Continuous);

        sensor.set_ranging_mode(RangingMode::Autonomous).await.unwrap();
        assert_eq!(sensor.get_ranging_mode().await.unwrap(), RangingMode::Autonomous);
    }

    #[test]
    #[timeout(10)]
    async fn test_integration_time(mut sensor: Sensor) {
        sensor.init().await.unwrap();

        sensor.set_integration_time_ms(20).await.unwrap();
        defmt::assert_eq!(sensor.get_integration_time_ms().await.unwrap(), 20);

        sensor.set_integration_time_ms(50).await.unwrap();
        defmt::assert_eq!(sensor.get_integration_time_ms().await.unwrap(), 50);
    }

    #[test]
    #[timeout(10)]
    async fn test_sharpener_percent(mut sensor: Sensor) {
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
    async fn test_power_mode(mut sensor: Sensor) {
        sensor.init().await.unwrap();

        assert_eq!(sensor.get_power_mode().await.unwrap(), PowerMode::Wakeup);

        sensor.set_power_mode(PowerMode::Sleep).await.unwrap();
        assert_eq!(sensor.get_power_mode().await.unwrap(), PowerMode::Sleep);

        sensor.set_power_mode(PowerMode::Wakeup).await.unwrap();
        assert_eq!(sensor.get_power_mode().await.unwrap(), PowerMode::Wakeup);
    }
}
