#![no_std]
#![no_main]

use defmt_rtt as _;

use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_time::Delay;
use vl53_8x8::{Resolution, NB_TARGET_PER_ZONE};

#[cfg(feature = "thresholds")]
use vl53_8x8::{
    DetectionThreshold, MotionConfiguration, ThresholdMeasurement, ThresholdOperation,
    ThresholdType,
};

// --- I2C mode (default) ---

#[cfg(not(feature = "spi-bus"))]
mod bus_setup {
    use super::*;
    use embassy_stm32::i2c;

    embassy_stm32::bind_interrupts!(struct Irqs {
        I2C1_EV => i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
        I2C1_ER => i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
        DMA1_STREAM6 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH6>;
        DMA1_STREAM0 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH0>;
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
            unsafe { core::ptr::read(&raw const p.DMA1_CH6) },
            unsafe { core::ptr::read(&raw const p.DMA1_CH0) },
            Irqs,
            i2c_config,
        );

        let mut lp_c = Output::new(
            unsafe { core::ptr::read(&raw const p.PB0) },
            Level::Low,
            Speed::Low,
        );
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

    embassy_stm32::bind_interrupts!(struct Irqs {
        DMA2_STREAM3 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA2_CH3>;
        DMA2_STREAM0 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA2_CH0>;
    });

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
            Irqs,
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
    #[timeout(10)]
    async fn test_set_resolution_while_ranging(mut sensor: Sensor) {
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();
        sensor.set_ranging_frequency_hz(5).await.unwrap();
        sensor.start_ranging().await.unwrap();

        // Change resolution while ranging
        sensor.set_resolution(Resolution::Res8x8).await.unwrap();
        assert_eq!(sensor.get_resolution().await.unwrap(), Resolution::Res8x8);

        sensor.stop_ranging().await.unwrap();
    }

    #[test]
    #[timeout(15)]
    async fn test_reinit_after_reset(mut sensor: Sensor) {
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res8x8).await.unwrap();

        // Reset and re-init
        sensor.reset().await.unwrap();
        cortex_m::asm::delay(84_000 * 100);
        sensor.init().await.unwrap();

        // Verify sensor is functional after reinit
        defmt::assert!(sensor.is_alive().await.unwrap());
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();
        assert_eq!(sensor.get_resolution().await.unwrap(), Resolution::Res4x4);
    }

    #[test]
    #[timeout(10)]
    async fn test_vhv_repeat_count(mut sensor: Sensor) {
        sensor.init().await.unwrap();

        sensor.set_vhv_repeat_count(5).await.unwrap();
        defmt::assert_eq!(sensor.get_vhv_repeat_count().await.unwrap(), 5);

        sensor.set_vhv_repeat_count(10).await.unwrap();
        defmt::assert_eq!(sensor.get_vhv_repeat_count().await.unwrap(), 10);
    }

    #[test]
    #[timeout(10)]
    async fn test_external_sync_pin_enable(mut sensor: Sensor) {
        sensor.init().await.unwrap();

        sensor.set_external_sync_pin_enable(true).await.unwrap();
        defmt::assert!(sensor.get_external_sync_pin_enable().await.unwrap());

        sensor.set_external_sync_pin_enable(false).await.unwrap();
        defmt::assert!(!sensor.get_external_sync_pin_enable().await.unwrap());
    }

    #[cfg(not(feature = "spi-bus"))]
    #[test]
    #[timeout(15)]
    async fn test_set_address(mut sensor: Sensor) {
        let new_address: u8 = 0x35;
        sensor.set_i2c_address(new_address).await.unwrap();
        cortex_m::asm::delay(84_000 * 100);
        defmt::assert!(sensor.is_alive().await.unwrap());
    }

    #[test]
    #[timeout(10)]
    async fn test_wait_for_data_ready_blocking(mut sensor: Sensor) {
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();
        sensor.set_ranging_frequency_hz(5).await.unwrap();
        sensor.start_ranging().await.unwrap();

        // Use the interrupt pin to wait instead of polling check_data_ready
        sensor.wait_for_data_ready_blocking().unwrap();

        let data = sensor.get_ranging_data().await.unwrap();
        // First (closest/strongest) target of each zone: entry zone * N.
        for i in 0..16 {
            let d = data.distance_mm[i * NB_TARGET_PER_ZONE];
            defmt::assert!(
                d > 10 && d < 4000,
                "zone {} distance {} mm out of range",
                i,
                d
            );
        }

        sensor.stop_ranging().await.unwrap();
    }

    #[cfg(feature = "thresholds")]
    #[test]
    #[timeout(10)]
    async fn test_detection_thresholds_enable(mut sensor: Sensor) {
        sensor.init().await.unwrap();

        sensor.set_detection_thresholds_enable(true).await.unwrap();
        defmt::assert!(sensor.get_detection_thresholds_enable().await.unwrap());

        sensor.set_detection_thresholds_enable(false).await.unwrap();
        defmt::assert!(!sensor.get_detection_thresholds_enable().await.unwrap());
    }

    /// Auto-stop is a VL53L8CX-only plugin function (the L5 ULD lacks it); the
    /// Nucleo shield carries an L8, so the round trip must succeed.
    #[cfg(feature = "thresholds")]
    #[test]
    #[timeout(10)]
    async fn test_detection_thresholds_auto_stop(mut sensor: Sensor) {
        sensor.init().await.unwrap();

        // Default after init is off.
        defmt::assert!(!sensor.get_detection_thresholds_auto_stop().await.unwrap());

        sensor.set_detection_thresholds_auto_stop(true).await.unwrap();
        defmt::assert!(sensor.get_detection_thresholds_auto_stop().await.unwrap());

        sensor.set_detection_thresholds_auto_stop(false).await.unwrap();
        defmt::assert!(!sensor.get_detection_thresholds_auto_stop().await.unwrap());
    }

    #[cfg(feature = "thresholds")]
    #[test]
    #[timeout(10)]
    async fn test_detection_thresholds(mut sensor: Sensor) {
        sensor.init().await.unwrap();

        let mut thresholds: [DetectionThreshold; 64] = unsafe { core::mem::zeroed() };

        // Configure zone 0: trigger when distance < 500 mm
        thresholds[0] = DetectionThreshold {
            zone_num: 0,
            measurement: ThresholdMeasurement::DistanceMm,
            threshold_type: ThresholdType::LessThanEqualMin,
            low_threshold: 500,
            high_threshold: 0,
            mathematic_operation: ThresholdOperation::None,
        };

        sensor.set_detection_thresholds(thresholds).await.unwrap();
        let readback = sensor.get_detection_thresholds().await.unwrap();

        defmt::assert_eq!(readback[0].zone_num, 0);
        defmt::assert_eq!(readback[0].low_threshold, 500);
    }

    #[cfg(feature = "thresholds")]
    #[test]
    #[timeout(10)]
    async fn test_motion_indicator_init(mut sensor: Sensor) {
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();

        let mut motion_config: MotionConfiguration = unsafe { core::mem::zeroed() };
        sensor
            .motion_indicator_init(&mut motion_config, Resolution::Res4x4)
            .await
            .unwrap();
    }

    #[cfg(feature = "thresholds")]
    #[test]
    #[timeout(10)]
    async fn test_motion_indicator_set_distance(mut sensor: Sensor) {
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();

        let mut motion_config: MotionConfiguration = unsafe { core::mem::zeroed() };
        sensor
            .motion_indicator_init(&mut motion_config, Resolution::Res4x4)
            .await
            .unwrap();
        sensor
            .motion_indicator_set_distance_motion(&mut motion_config, 400, 1500)
            .await
            .unwrap();
    }

    #[cfg(feature = "thresholds")]
    #[test]
    #[timeout(10)]
    async fn test_motion_indicator_set_resolution(mut sensor: Sensor) {
        sensor.init().await.unwrap();

        let mut motion_config: MotionConfiguration = unsafe { core::mem::zeroed() };
        sensor
            .motion_indicator_init(&mut motion_config, Resolution::Res4x4)
            .await
            .unwrap();
        sensor
            .motion_indicator_set_resolution(&mut motion_config, Resolution::Res8x8)
            .await
            .unwrap();
    }

    #[cfg(feature = "xtalk")]
    #[test]
    #[timeout(10)]
    async fn test_xtalk_margin(mut sensor: Sensor) {
        sensor.init().await.unwrap();

        sensor.set_xtalk_margin(100).await.unwrap();
        defmt::assert_eq!(sensor.get_xtalk_margin().await.unwrap(), 100);

        sensor.set_xtalk_margin(50).await.unwrap();
        defmt::assert_eq!(sensor.get_xtalk_margin().await.unwrap(), 50);
    }

    #[cfg(feature = "xtalk")]
    #[test]
    #[timeout(10)]
    async fn test_caldata_xtalk(mut sensor: Sensor) {
        sensor.init().await.unwrap();

        let mut buffer = [0u8; vl53_8x8::XTALK_BUFFER_SIZE];
        sensor.get_caldata_xtalk(&mut buffer).await.unwrap();

        // Buffer should have been filled with calibration data
        let non_zero = buffer.iter().any(|&b| b != 0);
        defmt::assert!(non_zero, "xtalk caldata buffer is all zeros");

        // Round-trip: set it back and re-read
        let saved = buffer;
        sensor.set_caldata_xtalk(&mut buffer).await.unwrap();
        sensor.get_caldata_xtalk(&mut buffer).await.unwrap();
        defmt::assert!(buffer == saved, "xtalk caldata round-trip mismatch");
    }

    /// Crosstalk calibration against whatever is in front of the sensor (no
    /// calibration target on the bench): the command sequence must complete
    /// and leave the sensor usable, with its configuration restored. The
    /// firmware may accept the scene or report `XtalkFailed` ("coverglass too
    /// good"); both are fine here. Variant-agnostic.
    #[cfg(feature = "xtalk")]
    #[test]
    #[timeout(60)]
    async fn test_calibrate_xtalk_leaves_sensor_usable(mut sensor: Sensor) {
        use vl53_8x8::{Error, FirmwareError};
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();
        sensor.set_ranging_frequency_hz(5).await.unwrap();

        match sensor.calibrate_xtalk(5, 4, 600).await {
            Ok(()) => defmt::info!("xtalk calibration accepted the scene"),
            Err(Error::Firmware(FirmwareError::XtalkFailed)) => {
                defmt::info!("xtalk calibration: XtalkFailed (expected without a target)")
            }
            Err(e) => {
                defmt::error!("calibrate_xtalk: {}", defmt::Debug2Format(&e));
                panic!("calibrate_xtalk failed");
            }
        }

        // Configuration restored.
        assert_eq!(sensor.get_resolution().await.unwrap(), Resolution::Res4x4);
        defmt::assert_eq!(sensor.get_ranging_frequency_hz().await.unwrap(), 5);

        // Calibration data readable.
        let mut buffer = [0u8; vl53_8x8::XTALK_BUFFER_SIZE];
        sensor.get_caldata_xtalk(&mut buffer).await.unwrap();
        defmt::assert!(buffer.iter().any(|&b| b != 0), "xtalk caldata buffer is all zeros");

        // A ranging round-trip still works.
        sensor.start_ranging().await.unwrap();
        sensor.poll_data_ready().await.unwrap();
        let data = sensor.get_ranging_data().await.unwrap();
        for i in 0..16 {
            let d = data.distance_mm[i * NB_TARGET_PER_ZONE];
            defmt::assert!(d < 8000, "zone {} distance {} mm implausible", i, d);
        }
        sensor.stop_ranging().await.unwrap();
    }

    /// Out-of-range arguments are rejected with `InvalidParam` (the host-side
    /// unit test in `ops.rs` checks that this happens before any bus traffic).
    #[cfg(feature = "xtalk")]
    #[test]
    #[timeout(15)]
    async fn test_calibrate_xtalk_rejects_invalid_params(mut sensor: Sensor) {
        use vl53_8x8::{Error, FirmwareError};
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res8x8).await.unwrap();

        for (r, n, d) in [(0u8, 4u8, 600u16), (100, 4, 600), (5, 0, 600), (5, 17, 600), (5, 4, 599), (5, 4, 3001)] {
            let result = sensor.calibrate_xtalk(r, n, d).await;
            defmt::assert!(
                matches!(result, Err(Error::Firmware(FirmwareError::InvalidParam))),
                "calibrate_xtalk({}, {}, {}) was not rejected",
                r, n, d
            );
        }

        // Nothing was disturbed.
        assert_eq!(sensor.get_resolution().await.unwrap(), Resolution::Res8x8);
    }
}
