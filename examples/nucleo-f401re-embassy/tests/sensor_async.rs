#![no_std]
#![no_main]

use defmt_rtt as _;

use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_time::Delay;
use vl53_8x8::{PowerMode, RangingMode, Resolution, TargetOrder, NB_TARGET_PER_ZONE};

#[cfg(feature = "thresholds")]
use vl53_8x8::{
    DetectionThreshold, ThresholdMeasurement, ThresholdOperation, ThresholdType,
    VL53L8CX_LAST_THRESHOLD,
};
#[cfg(any(feature = "motion", feature = "thresholds"))]
use vl53_8x8::MotionConfiguration;

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
        Output<'static>,                    // LPn (comms enable)
        embassy_stm32::gpio::Input<'static>, // INT
        Output<'static>,                    // PWR_EN_C (active high)
    >;

    #[cfg(not(feature = "blocking-i2c"))]
    pub type Sensor = vl53_8x8::Sensor<
        I2cInst,
        Delay,
        Output<'static>,                    // LPn (comms enable)
        embassy_stm32::gpio::Input<'static>, // INT
        Output<'static>,                    // PWR_EN_C (active high)
    >;

    pub fn init_bus(p: &mut embassy_stm32::Peripherals, pwr_en_c: Output<'static>) -> Sensor {
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
        // LPn LOW = I2C disabled, HIGH = I2C enabled (it is a comms enable, not a
        // reset). Leave it HIGH so the sensor is ready for init(); the handle's
        // power_cycle() drives it with PWR_EN when a test needs a real reset.
        lp_c.set_low();
        cortex_m::asm::delay(84_000 * 10);
        lp_c.set_high();
        cortex_m::asm::delay(84_000 * 100);

        #[cfg(feature = "blocking-i2c")]
        let sensor = vl53_8x8::Sensor::new(vl53_8x8::BlockingI2c(i2c), Delay);
        #[cfg(not(feature = "blocking-i2c"))]
        let sensor = vl53_8x8::Sensor::new(i2c, Delay);

        sensor
            .with_lpn_pin(lp_c)
            .with_interrupt_pin_active_low(int_c)
            .with_power_enable_pin_active_high(pwr_en_c)
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
        Output<'static>,                    // LPn (comms enable)
        embassy_stm32::gpio::Input<'static>, // INT
        Output<'static>,                    // PWR_EN_C (active high)
    >;

    pub fn init_bus(p: &mut embassy_stm32::Peripherals, pwr_en_c: Output<'static>) -> Sensor {
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
        // LPn LOW = I2C disabled, HIGH = I2C enabled (it is a comms enable, not a
        // reset). Leave it HIGH so the sensor is ready for init(); the handle's
        // power_cycle() drives it with PWR_EN when a test needs a real reset.
        lp_c.set_low();
        cortex_m::asm::delay(84_000 * 10);
        lp_c.set_high();
        cortex_m::asm::delay(84_000 * 100);

        vl53_8x8::Sensor::new_spi(spi_device, Delay)
            .with_lpn_pin(lp_c)
            .with_interrupt_pin_active_low(int_c)
            .with_power_enable_pin_active_high(pwr_en_c)
    }
}

use bus_setup::Sensor;

/// `&'static mut Sensor` wrapped in a value type: embedded-test's `#[init]`
/// state must be a plain value, not a reference. Derefs to the sensor so test
/// bodies call methods on it directly.
pub struct SensorRef(pub &'static mut Sensor);

impl core::ops::Deref for SensorRef {
    type Target = Sensor;
    fn deref(&self) -> &Sensor {
        self.0
    }
}

impl core::ops::DerefMut for SensorRef {
    fn deref_mut(&mut self) -> &mut Sensor {
        self.0
    }
}

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

    // Power-up sequence (board bring-up; the sensor handle's power_cycle()
    // does the same, driving LPn along with PWR_EN, once it owns the pin).
    let mut pwr_en_c = Output::new(
        unsafe { core::ptr::read(&raw const p.PA7) },
        Level::Low,
        Speed::Low,
    );
    pwr_en_c.set_low();
    cortex_m::asm::delay(84_000 * 50);
    pwr_en_c.set_high();
    cortex_m::asm::delay(84_000 * 50);

    bus_setup::init_bus(&mut p, pwr_en_c)
}

#[embedded_test::tests]
mod tests {
    use super::*;

    /// The sensor lives in a static and tests receive a `&'static mut`:
    /// embedded-test allocates one static task pool per async test, sized
    /// by the test future, so passing the ~3 KB `Sensor` by value into every
    /// future costs 12x that in RAM and overflows the F401RE's 96 KB with the
    /// `targets-N` features enabled.
    #[init]
    fn init() -> SensorRef {
        static SENSOR: static_cell::StaticCell<Sensor> = static_cell::StaticCell::new();
        SensorRef(SENSOR.init(board_init_async()))
    }

    #[test]
    #[timeout(5)]
    async fn test_sensor_alive(mut sensor: SensorRef) {
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
    async fn test_init(mut sensor: SensorRef) {
        sensor.init().await.unwrap();
    }

    #[test]
    #[timeout(10)]
    async fn test_set_resolution(mut sensor: SensorRef) {
        sensor.init().await.unwrap();

        sensor.set_resolution(Resolution::Res4x4).await.unwrap();
        assert_eq!(sensor.get_resolution().await.unwrap(), Resolution::Res4x4);

        sensor.set_resolution(Resolution::Res8x8).await.unwrap();
        assert_eq!(sensor.get_resolution().await.unwrap(), Resolution::Res8x8);
    }

    #[test]
    #[timeout(10)]
    async fn test_ranging_4x4(mut sensor: SensorRef) {
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();
        sensor.set_ranging_frequency_hz(5).await.unwrap();
        sensor.start_ranging().await.unwrap();

        sensor.poll_data_ready().await.unwrap();

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

    /// Multi-target scene check (only built with a `targets-N` feature, N > 1).
    ///
    /// Requires a scene with two surfaces in the field of view, e.g. a clear
    /// plastic lid ~15-20 mm above the shield and the ceiling beyond it. The
    /// two surfaces must be at least 600 mm apart to be reported as separate
    /// targets (UM2884); closer pairs merge into one return. Logs,
    /// per zone, the firmware's detected-target count and the first two
    /// targets' distance/status, and warns if no zone in any frame reported
    /// two or more targets.
    ///
    /// **Needs that staged scene**, so it is `#[ignore]`d and skipped by
    /// default. Run it explicitly with
    /// `cargo test --features spi-bus,targets-2 -- --include-ignored test_multi_target_scene_report`.
    #[cfg(any(feature = "targets-2", feature = "targets-3", feature = "targets-4"))]
    #[test]
    #[ignore]
    #[timeout(20)]
    async fn test_multi_target_scene_report(mut sensor: SensorRef) {
        const FRAMES: usize = 5;
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();
        sensor.set_ranging_frequency_hz(5).await.unwrap();
        sensor.start_ranging().await.unwrap();

        let mut zones_with_two = 0u32;
        for frame in 0..FRAMES {
            sensor.poll_data_ready().await.unwrap();
            let data = sensor.get_ranging_data().await.unwrap();
            for zone in 0..16 {
                let n = data.nb_target_detected[zone];
                let base = zone * NB_TARGET_PER_ZONE;
                let (d0, s0) = (data.distance_mm[base], data.target_status[base]);
                let (d1, s1) = (data.distance_mm[base + 1], data.target_status[base + 1]);
                defmt::assert!(d0 < 8000 && d1 < 8000, "zone {} implausible distance", zone);
                if frame == FRAMES - 1 {
                    defmt::info!(
                        "zone {}: {} target(s)  t0 {} mm (status {})  t1 {} mm (status {})",
                        zone, n, d0, s0, d1, s1
                    );
                }
                if n >= 2 {
                    zones_with_two += 1;
                }
            }
        }
        sensor.stop_ranging().await.unwrap();
        defmt::info!(
            "{} zone-frames with >= 2 targets over {} frames (N = {})",
            zones_with_two, FRAMES, NB_TARGET_PER_ZONE
        );
        if zones_with_two == 0 {
            defmt::warn!("no zone reported a second target; is a two-surface scene in front of the sensor?");
        }
    }

    #[test]
    #[timeout(10)]
    async fn test_ranging_8x8(mut sensor: SensorRef) {
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res8x8).await.unwrap();
        sensor.set_ranging_frequency_hz(5).await.unwrap();
        sensor.start_ranging().await.unwrap();

        sensor.poll_data_ready().await.unwrap();

        let data = sensor.get_ranging_data().await.unwrap();
        // First (closest/strongest) target of each zone: entry zone * N.
        for i in 0..64 {
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

    #[test]
    #[timeout(10)]
    async fn test_ambient_light(mut sensor: SensorRef) {
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
    async fn test_ranging_frequency(mut sensor: SensorRef) {
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
    async fn test_target_order(mut sensor: SensorRef) {
        sensor.init().await.unwrap();

        sensor.set_target_order(TargetOrder::Closest).await.unwrap();
        assert_eq!(sensor.get_target_order().await.unwrap(), TargetOrder::Closest);

        sensor.set_target_order(TargetOrder::Strongest).await.unwrap();
        assert_eq!(sensor.get_target_order().await.unwrap(), TargetOrder::Strongest);
    }

    #[test]
    #[timeout(10)]
    async fn test_ranging_mode(mut sensor: SensorRef) {
        sensor.init().await.unwrap();

        sensor.set_ranging_mode(RangingMode::Continuous).await.unwrap();
        assert_eq!(sensor.get_ranging_mode().await.unwrap(), RangingMode::Continuous);

        sensor.set_ranging_mode(RangingMode::Autonomous).await.unwrap();
        assert_eq!(sensor.get_ranging_mode().await.unwrap(), RangingMode::Autonomous);
    }

    #[test]
    #[timeout(10)]
    async fn test_integration_time(mut sensor: SensorRef) {
        sensor.init().await.unwrap();

        sensor.set_integration_time_ms(20).await.unwrap();
        defmt::assert_eq!(sensor.get_integration_time_ms().await.unwrap(), 20);

        sensor.set_integration_time_ms(50).await.unwrap();
        defmt::assert_eq!(sensor.get_integration_time_ms().await.unwrap(), 50);
    }

    #[test]
    #[timeout(10)]
    async fn test_sharpener_percent(mut sensor: SensorRef) {
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
    async fn test_power_mode(mut sensor: SensorRef) {
        sensor.init().await.unwrap();

        assert_eq!(sensor.get_power_mode().await.unwrap(), PowerMode::Wakeup);

        sensor.set_power_mode(PowerMode::Sleep).await.unwrap();
        assert_eq!(sensor.get_power_mode().await.unwrap(), PowerMode::Sleep);

        sensor.set_power_mode(PowerMode::Wakeup).await.unwrap();
        assert_eq!(sensor.get_power_mode().await.unwrap(), PowerMode::Wakeup);
    }

    // --- formerly tests/sensor_async_extra.rs (merged once the Sensor moved into a
    //     static: per-test task pools are now small enough for one binary) ---

    #[test]
    #[timeout(10)]
    async fn test_set_resolution_while_ranging(mut sensor: SensorRef) {
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
    async fn test_reinit_after_power_cycle(mut sensor: SensorRef) {
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res8x8).await.unwrap();

        // Power-cycle (PWR_EN_C + LPn, UM2884 §4.2) and re-init
        sensor.power_cycle().await.unwrap();
        cortex_m::asm::delay(84_000 * 100);
        sensor.init().await.unwrap();

        // Verify sensor is functional after reinit
        defmt::assert!(sensor.is_alive().await.unwrap());
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();
        assert_eq!(sensor.get_resolution().await.unwrap(), Resolution::Res4x4);
    }

    #[test]
    #[timeout(10)]
    async fn test_vhv_repeat_count(mut sensor: SensorRef) {
        sensor.init().await.unwrap();

        sensor.set_vhv_repeat_count(5).await.unwrap();
        defmt::assert_eq!(sensor.get_vhv_repeat_count().await.unwrap(), 5);

        sensor.set_vhv_repeat_count(10).await.unwrap();
        defmt::assert_eq!(sensor.get_vhv_repeat_count().await.unwrap(), 10);
    }

    #[test]
    #[timeout(10)]
    async fn test_external_sync_pin_enable(mut sensor: SensorRef) {
        sensor.init().await.unwrap();

        sensor.set_external_sync_pin_enable(true).await.unwrap();
        defmt::assert!(sensor.get_external_sync_pin_enable().await.unwrap());

        sensor.set_external_sync_pin_enable(false).await.unwrap();
        defmt::assert!(!sensor.get_external_sync_pin_enable().await.unwrap());
    }

    #[cfg(not(feature = "spi-bus"))]
    #[test]
    #[timeout(15)]
    async fn test_set_address(mut sensor: SensorRef) {
        let new_address: u8 = 0x35;
        sensor.set_i2c_address(new_address).await.unwrap();
        cortex_m::asm::delay(84_000 * 100);
        defmt::assert!(sensor.is_alive().await.unwrap());
    }

    #[test]
    #[timeout(10)]
    async fn test_wait_for_data_ready_blocking(mut sensor: SensorRef) {
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
    async fn test_detection_thresholds_enable(mut sensor: SensorRef) {
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
    async fn test_detection_thresholds_auto_stop(mut sensor: SensorRef) {
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
    async fn test_detection_thresholds(mut sensor: SensorRef) {
        sensor.init().await.unwrap();

        let mut thresholds: [DetectionThreshold; 64] = unsafe { core::mem::zeroed() };

        // Configure zone 0: trigger when distance < 500 mm. It is the only
        // (hence last) entry, so ST's header requires VL53L8CX_LAST_THRESHOLD
        // OR'd into its zone_num.
        thresholds[0] = DetectionThreshold {
            zone_num: 0 | VL53L8CX_LAST_THRESHOLD,
            measurement: ThresholdMeasurement::DistanceMm,
            threshold_type: ThresholdType::LessThanEqualMin,
            low_threshold: 500,
            high_threshold: 0,
            mathematic_operation: ThresholdOperation::None,
        };

        sensor.set_detection_thresholds(thresholds).await.unwrap();
        let readback = sensor.get_detection_thresholds().await.unwrap();

        defmt::assert_eq!(readback[0].zone_num & !VL53L8CX_LAST_THRESHOLD, 0);
        defmt::assert_eq!(readback[0].low_threshold, 500);
    }

    #[cfg(any(feature = "motion", feature = "thresholds"))]
    #[test]
    #[timeout(10)]
    async fn test_motion_indicator_init(mut sensor: SensorRef) {
        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();

        let mut motion_config: MotionConfiguration = unsafe { core::mem::zeroed() };
        sensor
            .motion_indicator_init(&mut motion_config, Resolution::Res4x4)
            .await
            .unwrap();
    }

    #[cfg(any(feature = "motion", feature = "thresholds"))]
    #[test]
    #[timeout(10)]
    async fn test_motion_indicator_set_distance(mut sensor: SensorRef) {
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

    #[cfg(any(feature = "motion", feature = "thresholds"))]
    #[test]
    #[timeout(10)]
    async fn test_motion_indicator_set_resolution(mut sensor: SensorRef) {
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
    async fn test_xtalk_margin(mut sensor: SensorRef) {
        sensor.init().await.unwrap();

        sensor.set_xtalk_margin(100).await.unwrap();
        defmt::assert_eq!(sensor.get_xtalk_margin().await.unwrap(), 100);

        sensor.set_xtalk_margin(50).await.unwrap();
        defmt::assert_eq!(sensor.get_xtalk_margin().await.unwrap(), 50);
    }

    #[cfg(feature = "xtalk")]
    #[test]
    #[timeout(10)]
    async fn test_caldata_xtalk(mut sensor: SensorRef) {
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
    async fn test_calibrate_xtalk_leaves_sensor_usable(mut sensor: SensorRef) {
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

    /// Xtalk calibration report for a bench with cover glass fitted and a flat
    /// surface (e.g. the ceiling) in view. Ranges one frame, uses the mean
    /// valid distance as the calibration target distance, calibrates, then
    /// ranges again so the before/after frames and the result can be compared
    /// in the log. Informational: only the sensor staying usable is asserted.
    /// Motion indicator report: configures the motion plugin for 4x4 over
    /// 400..1900 mm, then logs the per-zone motion values for ~12 s while
    /// something moves in the field of view. ST's example treats a value of
    /// 44 or more as motion. If the `xtalk` feature is also on, xtalk is
    /// calibrated first so a cover glass does not swamp the measurement.
    /// Informational: asserts only that frames keep parsing.
    ///
    /// **Needs a person to move something in front of the sensor**, so it is
    /// `#[ignore]`d and skipped by default. Run it explicitly with
    /// `cargo test --features spi-bus,motion -- --include-ignored test_motion_indicator_report`.
    #[cfg(feature = "motion")]
    #[test]
    #[ignore]
    #[timeout(60)]
    async fn test_motion_indicator_report(mut sensor: SensorRef) {
        const MOTION_THRESHOLD: u32 = 44;
        const FREQ_HZ: u8 = 15;
        const FRAMES: usize = 180; // ~12 s at 15 Hz

        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();

        #[cfg(feature = "xtalk")]
        match sensor.calibrate_xtalk(50, 8, 1640).await {
            Ok(()) => defmt::info!("xtalk calibrated before motion test"),
            Err(e) => defmt::warn!("xtalk calibration skipped: {}", defmt::Debug2Format(&e)),
        }

        let mut motion_config: MotionConfiguration = unsafe { core::mem::zeroed() };
        sensor
            .motion_indicator_init(&mut motion_config, Resolution::Res4x4)
            .await
            .unwrap();
        sensor
            .motion_indicator_set_distance_motion(&mut motion_config, 400, 1900)
            .await
            .unwrap();
        sensor.set_ranging_frequency_hz(FREQ_HZ).await.unwrap();
        sensor.start_ranging().await.unwrap();

        // Discard the first frame (status 6, wrap-around not performed).
        sensor.poll_data_ready().await.unwrap();
        let _ = sensor.get_ranging_data().await.unwrap();

        let mut frames_with_motion = 0u32;
        let mut hits_per_zone = [0u32; 16];
        let mut max_motion = 0u32;
        defmt::info!("capturing {} frames at {} Hz: move something 30-80 cm above the sensor", FRAMES, FREQ_HZ);
        for frame in 0..FRAMES {
            sensor.poll_data_ready().await.unwrap();
            let data = sensor.get_ranging_data().await.unwrap();
            let m = &data.motion_indicator;
            let mut moving = 0u32;
            let mut frame_max = 0u32;
            for zone in 0..16 {
                let v = m.motion[zone];
                frame_max = frame_max.max(v);
                if v >= MOTION_THRESHOLD {
                    moving += 1;
                    hits_per_zone[zone] += 1;
                }
            }
            max_motion = max_motion.max(frame_max);
            if moving > 0 {
                frames_with_motion += 1;
                defmt::info!(
                    "frame {}: motion in {} zones, max {} (global {} / {}, aggregates {})",
                    frame, moving, frame_max, m.global_indicator_1, m.global_indicator_2,
                    m.nb_of_detected_aggregates
                );
            } else if frame % 30 == 0 {
                defmt::info!("frame {}: still (max {})", frame, frame_max);
            }
        }
        sensor.stop_ranging().await.unwrap();

        defmt::info!(
            "{} of {} frames had motion (>= {}); max motion value {}",
            frames_with_motion, FRAMES, MOTION_THRESHOLD, max_motion
        );
        defmt::info!("hits per zone (4x4, row-major): {}", hits_per_zone);
        if frames_with_motion == 0 {
            defmt::warn!("no motion observed: nothing moved in the field of view during the capture?");
        }
    }

    #[cfg(feature = "xtalk")]
    #[test]
    #[timeout(60)]
    async fn test_calibrate_xtalk_report(mut sensor: SensorRef) {
        use vl53_8x8::{Error, FirmwareError};
        const REFLECTANCE_PERCENT: u8 = 50;
        const NB_SAMPLES: u8 = 8;
        /// Known distance to the calibration surface, if the uncalibrated
        /// sensor cannot measure it (heavy crosstalk reports ~0 mm). `None`
        /// uses the mean valid distance of the "before" frame, falling back
        /// to the ULD minimum of 600 mm when no zone is valid.
        const KNOWN_DISTANCE_MM: Option<u16> = None;

        async fn frame(sensor: &mut SensorRef, label: &str) -> i32 {
            sensor.start_ranging().await.unwrap();
            // The first frame after start carries status 6 ("wrap-around not
            // performed"); sample the second one.
            sensor.poll_data_ready().await.unwrap();
            let _ = sensor.get_ranging_data().await.unwrap();
            sensor.poll_data_ready().await.unwrap();
            let data = sensor.get_ranging_data().await.unwrap();
            sensor.stop_ranging().await.unwrap();
            let (mut sum, mut n) = (0i32, 0i32);
            for zone in 0..16 {
                let i = zone * NB_TARGET_PER_ZONE;
                let (d, s) = (data.distance_mm[i], data.target_status[i]);
                defmt::info!("{} zone {}: {} mm (status {})", label, zone, d, s);
                if s == 5 || s == 9 {
                    sum += d as i32;
                    n += 1;
                }
            }
            if n > 0 { sum / n } else { 0 }
        }

        sensor.init().await.unwrap();
        sensor.set_resolution(Resolution::Res4x4).await.unwrap();
        sensor.set_ranging_frequency_hz(5).await.unwrap();

        let mean = frame(&mut sensor, "before").await;
        let distance_mm = match KNOWN_DISTANCE_MM {
            Some(d) => d,
            None if mean == 0 => {
                defmt::warn!("no valid zone before calibration; falling back to 600 mm");
                600
            }
            None => mean.clamp(600, 3000) as u16,
        };
        defmt::info!(
            "calibrate_xtalk(reflectance {}%, {} samples, {} mm) ...",
            REFLECTANCE_PERCENT, NB_SAMPLES, distance_mm
        );
        match sensor.calibrate_xtalk(REFLECTANCE_PERCENT, NB_SAMPLES, distance_mm).await {
            Ok(()) => defmt::info!("xtalk calibration: OK"),
            Err(Error::Firmware(FirmwareError::XtalkFailed)) => {
                defmt::info!("xtalk calibration: XtalkFailed (cover glass too good / no target)")
            }
            Err(e) => {
                defmt::error!("calibrate_xtalk: {}", defmt::Debug2Format(&e));
                panic!("calibrate_xtalk failed");
            }
        }

        let mut buffer = [0u8; vl53_8x8::XTALK_BUFFER_SIZE];
        sensor.get_caldata_xtalk(&mut buffer).await.unwrap();
        defmt::info!("xtalk caldata[0..32]: {=[u8]:#04x}", buffer[..32]);
        let nonzero = buffer.iter().filter(|&&b| b != 0).count();
        defmt::info!("xtalk caldata: {} of {} bytes non-zero", nonzero, buffer.len());

        let mean_after = frame(&mut sensor, "after").await;
        defmt::info!("mean valid distance before {} mm, after {} mm", mean, mean_after);
        defmt::assert!(mean_after < 8000);
    }

    /// Out-of-range arguments are rejected with `InvalidParam` (the host-side
    /// unit test in `ops.rs` checks that this happens before any bus traffic).
    #[cfg(feature = "xtalk")]
    #[test]
    #[timeout(15)]
    async fn test_calibrate_xtalk_rejects_invalid_params(mut sensor: SensorRef) {
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
