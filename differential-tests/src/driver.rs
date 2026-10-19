//! The `Driver` trait all drivers are driven through, the `vl53-8x8` adapter,
//! and the call sequences that re-enact the on-target tests.
//!
//! [`run_sequence`] follows the Nucleo C suite
//! (`examples/nucleo-f401re-uld/tests/sensor.rs`) call for call. Where the
//! Nucleo Rust suite (`examples/nucleo-f401re-embassy/tests/sensor_async.rs`)
//! differs it is noted on the sequence: the only differences are that the Rust
//! suite waits for data with `Sensor::poll_data_ready` (same bus traffic as a
//! manual `check_data_ready` loop, plus a 10 ms delay between polls) and that
//! it has extra xtalk-calibration tests the C suite lacks (not re-enacted).
//! [`run_evo_sequence`] follows the variant-agnostic evo-f412re suite
//! (`examples/evo-f412re-embassy/tests/sensor_async.rs`), which was recorded
//! on a VL53L5CX over I2C. Assertions on returned values are kept where the
//! on-target test had them, so a driver that decodes a register differently is
//! caught as well as one that writes different bytes.

use std::fmt::Debug;

use vl53_8x8::bus::VlBusAsync;
use vl53_8x8::{
    DetectionThreshold, FirmwareError, MotionConfiguration, PowerMode, RangingMode, Resolution,
    TargetOrder, ThresholdMeasurement, ThresholdOperation, ThresholdType, VL53L8CX_LAST_THRESHOLD,
    XTALK_BUFFER_SIZE,
};

use crate::mock::{block_on, MockDelay, MockI2c, MockPin, MockSpi};

/// Upper bound on data-ready polls before the sequence gives up; on the mock a
/// loop that never sees "ready" would otherwise spin forever.
pub const MAX_POLLS: usize = 10_000;

/// The common surface of `vl53l8cx_uld::Vl53l8cx`, `vl53l5cx_uld::Vl53l5cx`
/// and `vl53_8x8::Sensor`. Motion configuration is kept inside the adapter:
/// the crates use different (layout-identical) structs for it. The L5 ULD has
/// no sync pin and no threshold auto-stop; its adapter returns an error from
/// those four methods, which no L5 sequence calls.
pub trait Driver {
    type Error: Debug;

    fn is_alive(&mut self) -> Result<bool, Self::Error>;
    fn init(&mut self) -> Result<(), Self::Error>;
    /// `power_cycle()` on mock power-enable and LPn pins: no bus traffic.
    fn power_cycle(&mut self) -> Result<(), Self::Error>;

    fn set_resolution(&mut self, r: Resolution) -> Result<(), Self::Error>;
    fn get_resolution(&mut self) -> Result<Resolution, Self::Error>;
    fn set_ranging_frequency_hz(&mut self, hz: u8) -> Result<(), Self::Error>;
    fn get_ranging_frequency_hz(&mut self) -> Result<u8, Self::Error>;
    fn set_integration_time_ms(&mut self, ms: u32) -> Result<(), Self::Error>;
    fn get_integration_time_ms(&mut self) -> Result<u32, Self::Error>;
    fn set_target_order(&mut self, o: TargetOrder) -> Result<(), Self::Error>;
    fn get_target_order(&mut self) -> Result<TargetOrder, Self::Error>;
    fn set_ranging_mode(&mut self, m: RangingMode) -> Result<(), Self::Error>;
    fn get_ranging_mode(&mut self) -> Result<RangingMode, Self::Error>;
    fn set_sharpener_percent(&mut self, p: u8) -> Result<(), Self::Error>;
    fn get_sharpener_percent(&mut self) -> Result<u8, Self::Error>;
    fn set_power_mode(&mut self, m: PowerMode) -> Result<(), Self::Error>;
    fn get_power_mode(&mut self) -> Result<PowerMode, Self::Error>;
    fn set_vhv_repeat_count(&mut self, n: u32) -> Result<(), Self::Error>;
    fn get_vhv_repeat_count(&mut self) -> Result<u32, Self::Error>;
    fn set_external_sync_pin_enable(&mut self, on: bool) -> Result<(), Self::Error>;
    fn get_external_sync_pin_enable(&mut self) -> Result<bool, Self::Error>;

    fn start_ranging(&mut self) -> Result<(), Self::Error>;
    fn stop_ranging(&mut self) -> Result<(), Self::Error>;
    fn check_data_ready(&mut self) -> Result<bool, Self::Error>;
    fn wait_for_data_ready_blocking(&mut self) -> Result<(), Self::Error>;
    /// Read one frame; returns the first-target distance of each zone.
    fn get_ranging_data(&mut self) -> Result<Vec<i16>, Self::Error>;

    fn set_detection_thresholds_enable(&mut self, on: bool) -> Result<(), Self::Error>;
    fn get_detection_thresholds_enable(&mut self) -> Result<bool, Self::Error>;
    fn set_detection_thresholds_auto_stop(&mut self, on: bool) -> Result<(), Self::Error>;
    fn get_detection_thresholds_auto_stop(&mut self) -> Result<bool, Self::Error>;
    fn set_detection_thresholds(&mut self, t: [DetectionThreshold; 64]) -> Result<(), Self::Error>;
    fn get_detection_thresholds(&mut self) -> Result<[DetectionThreshold; 64], Self::Error>;

    fn motion_indicator_init(&mut self, r: Resolution) -> Result<(), Self::Error>;
    fn motion_indicator_set_distance_motion(&mut self, min_mm: u16, max_mm: u16) -> Result<(), Self::Error>;
    fn motion_indicator_set_resolution(&mut self, r: Resolution) -> Result<(), Self::Error>;

    fn set_xtalk_margin(&mut self, margin: u32) -> Result<(), Self::Error>;
    fn get_xtalk_margin(&mut self) -> Result<u32, Self::Error>;
    fn get_caldata_xtalk(&mut self, buf: &mut [u8]) -> Result<(), Self::Error>;
    fn set_caldata_xtalk(&mut self, buf: &mut [u8]) -> Result<(), Self::Error>;
    /// Crosstalk calibration (`xtalk` feature). The ULD wrappers take the
    /// reflectance as `u16`, `vl53-8x8` as `u8`; the valid range is 1..=99.
    fn calibrate_xtalk(&mut self, reflectance_percent: u8, nb_samples: u8, distance_mm: u16) -> Result<(), Self::Error>;

    /// The firmware status an error decodes to, if it is one. Every driver
    /// here uses `vl53_8x8_types::Error`, so this is a one-line match; it lets
    /// a sequence assert on `XtalkFailed`/`InvalidParam` without naming a
    /// concrete error type.
    fn firmware_error(e: &Self::Error) -> Option<FirmwareError>;
}

// --- vl53-8x8 adapter ---------------------------------------------------------

pub type RustSensor<BUS> = vl53_8x8::Sensor<BUS, MockDelay, MockPin, MockPin, MockPin>;
pub type RustSpiBus = vl53_8x8::BlockingSpi<MockSpi>;
pub type RustI2cBus = vl53_8x8::BlockingI2c<MockI2c>;

/// `vl53_8x8::Sensor` over one of the blocking bus adapters (`BlockingSpi` or
/// `BlockingI2c`), each call run to completion with [`block_on`]. The sensor
/// variant is whatever the model's device/revision id says: the driver
/// autodetects it in `init`, exactly as on hardware.
pub struct RustDriver<BUS: VlBusAsync> {
    pub sensor: RustSensor<BUS>,
    motion: MotionConfiguration,
}

impl RustDriver<RustSpiBus> {
    pub fn new(spi: MockSpi, delay: MockDelay) -> Self {
        RustDriver::over(vl53_8x8::BlockingSpi(spi), delay)
    }
}

impl RustDriver<RustI2cBus> {
    pub fn new_i2c(i2c: MockI2c, delay: MockDelay) -> Self {
        RustDriver::over(vl53_8x8::BlockingI2c(i2c), delay)
    }
}

impl<BUS: VlBusAsync> RustDriver<BUS> {
    fn over(bus: BUS, delay: MockDelay) -> Self {
        let sensor = vl53_8x8::Sensor::new(bus, delay)
            .with_lpn_pin(MockPin)
            .with_interrupt_pin_active_low(MockPin)
            .with_power_enable_pin_active_high(MockPin);
        RustDriver { sensor, motion: unsafe { core::mem::zeroed() } }
    }
}

type RustError = vl53_8x8::Error<core::convert::Infallible>;

impl<BUS: VlBusAsync<Error = core::convert::Infallible>> Driver for RustDriver<BUS> {
    type Error = RustError;

    fn is_alive(&mut self) -> Result<bool, RustError> {
        block_on(self.sensor.is_alive())
    }
    fn init(&mut self) -> Result<(), RustError> {
        block_on(self.sensor.init())
    }
    fn power_cycle(&mut self) -> Result<(), RustError> {
        block_on(self.sensor.power_cycle())
    }
    fn set_resolution(&mut self, r: Resolution) -> Result<(), RustError> {
        block_on(self.sensor.set_resolution(r))
    }
    fn get_resolution(&mut self) -> Result<Resolution, RustError> {
        block_on(self.sensor.get_resolution())
    }
    fn set_ranging_frequency_hz(&mut self, hz: u8) -> Result<(), RustError> {
        block_on(self.sensor.set_ranging_frequency_hz(hz))
    }
    fn get_ranging_frequency_hz(&mut self) -> Result<u8, RustError> {
        block_on(self.sensor.get_ranging_frequency_hz())
    }
    fn set_integration_time_ms(&mut self, ms: u32) -> Result<(), RustError> {
        block_on(self.sensor.set_integration_time_ms(ms))
    }
    fn get_integration_time_ms(&mut self) -> Result<u32, RustError> {
        block_on(self.sensor.get_integration_time_ms())
    }
    fn set_target_order(&mut self, o: TargetOrder) -> Result<(), RustError> {
        block_on(self.sensor.set_target_order(o))
    }
    fn get_target_order(&mut self) -> Result<TargetOrder, RustError> {
        block_on(self.sensor.get_target_order())
    }
    fn set_ranging_mode(&mut self, m: RangingMode) -> Result<(), RustError> {
        block_on(self.sensor.set_ranging_mode(m))
    }
    fn get_ranging_mode(&mut self) -> Result<RangingMode, RustError> {
        block_on(self.sensor.get_ranging_mode())
    }
    fn set_sharpener_percent(&mut self, p: u8) -> Result<(), RustError> {
        block_on(self.sensor.set_sharpener_percent(p))
    }
    fn get_sharpener_percent(&mut self) -> Result<u8, RustError> {
        block_on(self.sensor.get_sharpener_percent())
    }
    fn set_power_mode(&mut self, m: PowerMode) -> Result<(), RustError> {
        block_on(self.sensor.set_power_mode(m))
    }
    fn get_power_mode(&mut self) -> Result<PowerMode, RustError> {
        block_on(self.sensor.get_power_mode())
    }
    fn set_vhv_repeat_count(&mut self, n: u32) -> Result<(), RustError> {
        block_on(self.sensor.set_vhv_repeat_count(n))
    }
    fn get_vhv_repeat_count(&mut self) -> Result<u32, RustError> {
        block_on(self.sensor.get_vhv_repeat_count())
    }
    fn set_external_sync_pin_enable(&mut self, on: bool) -> Result<(), RustError> {
        block_on(self.sensor.set_external_sync_pin_enable(on))
    }
    fn get_external_sync_pin_enable(&mut self) -> Result<bool, RustError> {
        block_on(self.sensor.get_external_sync_pin_enable())
    }
    fn start_ranging(&mut self) -> Result<(), RustError> {
        block_on(self.sensor.start_ranging())
    }
    fn stop_ranging(&mut self) -> Result<(), RustError> {
        block_on(self.sensor.stop_ranging())
    }
    fn check_data_ready(&mut self) -> Result<bool, RustError> {
        block_on(self.sensor.check_data_ready())
    }
    fn wait_for_data_ready_blocking(&mut self) -> Result<(), RustError> {
        self.sensor.wait_for_data_ready_blocking()
    }
    fn get_ranging_data(&mut self) -> Result<Vec<i16>, RustError> {
        let d = block_on(self.sensor.get_ranging_data())?;
        Ok((0..64).map(|z| d.distance_mm[z * vl53_8x8::NB_TARGET_PER_ZONE]).collect())
    }
    fn set_detection_thresholds_enable(&mut self, on: bool) -> Result<(), RustError> {
        block_on(self.sensor.set_detection_thresholds_enable(on))
    }
    fn get_detection_thresholds_enable(&mut self) -> Result<bool, RustError> {
        block_on(self.sensor.get_detection_thresholds_enable())
    }
    fn set_detection_thresholds_auto_stop(&mut self, on: bool) -> Result<(), RustError> {
        block_on(self.sensor.set_detection_thresholds_auto_stop(on))
    }
    fn get_detection_thresholds_auto_stop(&mut self) -> Result<bool, RustError> {
        block_on(self.sensor.get_detection_thresholds_auto_stop())
    }
    fn set_detection_thresholds(&mut self, t: [DetectionThreshold; 64]) -> Result<(), RustError> {
        block_on(self.sensor.set_detection_thresholds(t))
    }
    fn get_detection_thresholds(&mut self) -> Result<[DetectionThreshold; 64], RustError> {
        block_on(self.sensor.get_detection_thresholds())
    }
    fn motion_indicator_init(&mut self, r: Resolution) -> Result<(), RustError> {
        block_on(self.sensor.motion_indicator_init(&mut self.motion, r))
    }
    fn motion_indicator_set_distance_motion(&mut self, min_mm: u16, max_mm: u16) -> Result<(), RustError> {
        block_on(self.sensor.motion_indicator_set_distance_motion(&mut self.motion, min_mm, max_mm))
    }
    fn motion_indicator_set_resolution(&mut self, r: Resolution) -> Result<(), RustError> {
        block_on(self.sensor.motion_indicator_set_resolution(&mut self.motion, r))
    }
    fn set_xtalk_margin(&mut self, margin: u32) -> Result<(), RustError> {
        block_on(self.sensor.set_xtalk_margin(margin))
    }
    fn get_xtalk_margin(&mut self) -> Result<u32, RustError> {
        block_on(self.sensor.get_xtalk_margin())
    }
    fn get_caldata_xtalk(&mut self, buf: &mut [u8]) -> Result<(), RustError> {
        block_on(self.sensor.get_caldata_xtalk(buf))
    }
    fn set_caldata_xtalk(&mut self, buf: &mut [u8]) -> Result<(), RustError> {
        block_on(self.sensor.set_caldata_xtalk(buf))
    }
    fn calibrate_xtalk(&mut self, reflectance_percent: u8, nb_samples: u8, distance_mm: u16) -> Result<(), RustError> {
        block_on(self.sensor.calibrate_xtalk(reflectance_percent, nb_samples, distance_mm))
    }
    fn firmware_error(e: &RustError) -> Option<FirmwareError> {
        match e {
            vl53_8x8::Error::Firmware(f) => Some(*f),
            _ => None,
        }
    }
}

// --- sequences ----------------------------------------------------------------

/// The on-target tests re-enacted here, by name (same names in both suites and
/// in the recordings). Not covered: `test_set_address` (I2C only, not in the
/// SPI recordings) and the Rust-only xtalk calibration tests.
pub const SEQUENCES: &[&str] = &[
    "test_sensor_alive",
    "test_init",
    "test_set_resolution",
    "test_ranging_4x4",
    "test_ranging_8x8",
    "test_ambient_light",
    "test_ranging_frequency",
    "test_target_order",
    "test_ranging_mode",
    "test_integration_time",
    "test_sharpener_percent",
    "test_power_mode",
    "test_set_resolution_while_ranging",
    "test_reinit_after_reset",
    "test_vhv_repeat_count",
    "test_external_sync_pin_enable",
    "test_wait_for_data_ready_blocking",
    "test_detection_thresholds_enable",
    "test_detection_thresholds_auto_stop",
    "test_detection_thresholds",
    "test_motion_indicator_init",
    "test_motion_indicator_set_distance",
    "test_motion_indicator_set_resolution",
    "test_xtalk_margin",
    "test_caldata_xtalk",
];

type R<D> = Result<(), <D as Driver>::Error>;

fn poll_until_ready<D: Driver>(d: &mut D) -> R<D> {
    for _ in 0..MAX_POLLS {
        if d.check_data_ready()? {
            return Ok(());
        }
    }
    panic!("data never became ready after {MAX_POLLS} polls (model exhausted?)");
}

/// The Nucleo suites' distance assertion: a real target within range.
fn check_distances(label: &str, distances: &[i16], zones: usize) {
    for (zone, &d) in distances[..zones].iter().enumerate() {
        assert!(d > 10 && d < 4000, "{label}: zone {zone} distance {d} mm out of range");
    }
}

/// The evo suite's distance assertion: no fixed target, only "not garbage".
fn check_distances_plausible(label: &str, distances: &[i16], zones: usize) {
    for (zone, &d) in distances[..zones].iter().enumerate() {
        assert!(d < 8000, "{label}: zone {zone} distance {d} mm implausible");
    }
}

/// Start ranging at 5 Hz in `res`, wait for a frame, read it, stop.
fn ranging<D: Driver>(d: &mut D, res: Resolution, zones: usize, check: fn(&str, &[i16], usize)) -> R<D> {
    d.init()?;
    d.set_resolution(res)?;
    d.set_ranging_frequency_hz(5)?;
    d.start_ranging()?;
    poll_until_ready(d)?;
    let data = d.get_ranging_data()?;
    check("ranging", &data, zones);
    d.stop_ranging()
}

/// Issue the API calls of on-target test `name` against `d`. Panics on an
/// unknown name or a failed assertion, returns the driver's error otherwise.
pub fn run_sequence<D: Driver>(name: &str, d: &mut D) -> R<D> {
    match name {
        "test_sensor_alive" => {
            assert!(d.is_alive()?, "is_alive");
            Ok(())
        }
        "test_init" => d.init(),
        "test_set_resolution" => {
            d.init()?;
            d.set_resolution(Resolution::Res4x4)?;
            assert_eq!(d.get_resolution()?, Resolution::Res4x4);
            d.set_resolution(Resolution::Res8x8)?;
            assert_eq!(d.get_resolution()?, Resolution::Res8x8);
            Ok(())
        }
        "test_ranging_4x4" => ranging(d, Resolution::Res4x4, 16, check_distances),
        "test_ranging_8x8" => ranging(d, Resolution::Res8x8, 64, check_distances),
        // Same calls as test_ranging_4x4; the on-target test checks ambient instead of distance.
        "test_ambient_light" => ranging(d, Resolution::Res4x4, 16, check_distances),
        "test_ranging_frequency" => {
            d.init()?;
            for hz in [5u8, 15, 30] {
                d.set_ranging_frequency_hz(hz)?;
                assert_eq!(d.get_ranging_frequency_hz()?, hz);
            }
            Ok(())
        }
        "test_target_order" => {
            d.init()?;
            d.set_target_order(TargetOrder::Closest)?;
            assert_eq!(d.get_target_order()?, TargetOrder::Closest);
            d.set_target_order(TargetOrder::Strongest)?;
            assert_eq!(d.get_target_order()?, TargetOrder::Strongest);
            Ok(())
        }
        "test_ranging_mode" => {
            d.init()?;
            d.set_ranging_mode(RangingMode::Continuous)?;
            assert_eq!(d.get_ranging_mode()?, RangingMode::Continuous);
            d.set_ranging_mode(RangingMode::Autonomous)?;
            assert_eq!(d.get_ranging_mode()?, RangingMode::Autonomous);
            Ok(())
        }
        "test_integration_time" => {
            d.init()?;
            for ms in [20u32, 50] {
                d.set_integration_time_ms(ms)?;
                assert_eq!(d.get_integration_time_ms()?, ms);
            }
            Ok(())
        }
        "test_sharpener_percent" => {
            d.init()?;
            for pct in [25u8, 75] {
                d.set_sharpener_percent(pct)?;
                let v = d.get_sharpener_percent()?;
                assert!(v >= pct - 1 && v <= pct + 1, "expected ~{pct}, got {v}");
            }
            Ok(())
        }
        "test_power_mode" => {
            d.init()?;
            assert_eq!(d.get_power_mode()?, PowerMode::Wakeup);
            d.set_power_mode(PowerMode::Sleep)?;
            assert_eq!(d.get_power_mode()?, PowerMode::Sleep);
            d.set_power_mode(PowerMode::Wakeup)?;
            assert_eq!(d.get_power_mode()?, PowerMode::Wakeup);
            Ok(())
        }
        "test_set_resolution_while_ranging" => {
            d.init()?;
            d.set_resolution(Resolution::Res4x4)?;
            d.set_ranging_frequency_hz(5)?;
            d.start_ranging()?;
            d.set_resolution(Resolution::Res8x8)?;
            assert_eq!(d.get_resolution()?, Resolution::Res8x8);
            d.stop_ranging()
        }
        // On target this is now `test_reinit_after_power_cycle` (0.2.0: `reset()`
        // only pulsed LPn, `power_cycle()` is the real reset); the recordings
        // predate the rename, so the sequence keeps the old name. Neither the
        // pin pulses nor the 100 ms wait produce bus traffic.
        "test_reinit_after_reset" => {
            d.init()?;
            d.set_resolution(Resolution::Res8x8)?;
            d.power_cycle()?;
            d.init()?;
            assert!(d.is_alive()?);
            d.set_resolution(Resolution::Res4x4)?;
            assert_eq!(d.get_resolution()?, Resolution::Res4x4);
            Ok(())
        }
        "test_vhv_repeat_count" => {
            d.init()?;
            for n in [5u32, 10] {
                d.set_vhv_repeat_count(n)?;
                assert_eq!(d.get_vhv_repeat_count()?, n);
            }
            Ok(())
        }
        "test_external_sync_pin_enable" => {
            d.init()?;
            d.set_external_sync_pin_enable(true)?;
            assert!(d.get_external_sync_pin_enable()?);
            d.set_external_sync_pin_enable(false)?;
            assert!(!d.get_external_sync_pin_enable()?);
            Ok(())
        }
        // The mock INT pin reads "asserted", so the wait returns at once with no bus traffic.
        "test_wait_for_data_ready_blocking" => {
            d.init()?;
            d.set_resolution(Resolution::Res4x4)?;
            d.set_ranging_frequency_hz(5)?;
            d.start_ranging()?;
            d.wait_for_data_ready_blocking()?;
            let data = d.get_ranging_data()?;
            check_distances("wait_for_data_ready_blocking", &data, 16);
            d.stop_ranging()
        }
        "test_detection_thresholds_enable" => {
            d.init()?;
            d.set_detection_thresholds_enable(true)?;
            assert!(d.get_detection_thresholds_enable()?);
            d.set_detection_thresholds_enable(false)?;
            assert!(!d.get_detection_thresholds_enable()?);
            Ok(())
        }
        "test_detection_thresholds_auto_stop" => {
            d.init()?;
            assert!(!d.get_detection_thresholds_auto_stop()?);
            d.set_detection_thresholds_auto_stop(true)?;
            assert!(d.get_detection_thresholds_auto_stop()?);
            d.set_detection_thresholds_auto_stop(false)?;
            assert!(!d.get_detection_thresholds_auto_stop()?);
            Ok(())
        }
        "test_detection_thresholds" => {
            d.init()?;
            let mut thresholds: [DetectionThreshold; 64] = unsafe { core::mem::zeroed() };
            // Single entry, so it is also the last one: ST's header requires
            // VL53L8CX_LAST_THRESHOLD OR'd into its zone_num.
            thresholds[0] = DetectionThreshold {
                zone_num: 0 | VL53L8CX_LAST_THRESHOLD,
                measurement: ThresholdMeasurement::DistanceMm,
                threshold_type: ThresholdType::LessThanEqualMin,
                low_threshold: 500,
                high_threshold: 0,
                mathematic_operation: ThresholdOperation::None,
            };
            d.set_detection_thresholds(thresholds)?;
            let readback = d.get_detection_thresholds()?;
            assert_eq!(readback[0].zone_num & !VL53L8CX_LAST_THRESHOLD, 0);
            assert_eq!(readback[0].low_threshold, 500);
            Ok(())
        }
        "test_motion_indicator_init" => {
            d.init()?;
            d.set_resolution(Resolution::Res4x4)?;
            d.motion_indicator_init(Resolution::Res4x4)
        }
        "test_motion_indicator_set_distance" => {
            d.init()?;
            d.set_resolution(Resolution::Res4x4)?;
            d.motion_indicator_init(Resolution::Res4x4)?;
            d.motion_indicator_set_distance_motion(400, 1500)
        }
        "test_motion_indicator_set_resolution" => {
            d.init()?;
            d.motion_indicator_init(Resolution::Res4x4)?;
            d.motion_indicator_set_resolution(Resolution::Res8x8)
        }
        "test_xtalk_margin" => {
            d.init()?;
            for m in [100u32, 50] {
                d.set_xtalk_margin(m)?;
                assert_eq!(d.get_xtalk_margin()?, m);
            }
            Ok(())
        }
        "test_caldata_xtalk" => {
            d.init()?;
            let mut buffer = [0u8; XTALK_BUFFER_SIZE];
            d.get_caldata_xtalk(&mut buffer)?;
            assert!(buffer.iter().any(|&b| b != 0), "xtalk caldata buffer is all zeros");
            let saved = buffer;
            d.set_caldata_xtalk(&mut buffer)?;
            d.get_caldata_xtalk(&mut buffer)?;
            assert!(buffer == saved, "xtalk caldata round-trip mismatch");
            Ok(())
        }
        other => panic!("no sequence named {other:?}"),
    }
}

// --- evo-f412re sequences --------------------------------------------------------

/// The on-target tests of `examples/evo-f412re-embassy/tests/sensor_async.rs`,
/// by name (same names in the `evo-l5-i2c-8x8` recording). The suite is
/// variant-agnostic; it was recorded on a VL53L5CX over I2C. Not covered:
/// `multi_target_scene_report` (needs `targets-N`, `#[ignore]`d on target too).
pub const EVO_SEQUENCES: &[&str] = &[
    "async_i2c_alive",
    "init_uploads_firmware_over_async_i2c",
    "set_resolution",
    "ranging_4x4_async",
    "ranging_8x8_async",
    "ambient_light",
    "ranging_frequency",
    "integration_time",
    "target_order",
    "ranging_mode",
    "sharpener_percent",
    "power_mode_sleep_wakeup",
    "calibrate_xtalk_leaves_sensor_usable",
    "calibrate_xtalk_rejects_invalid_params",
];

/// Arguments the evo suite feeds `calibrate_xtalk` expecting `InvalidParam`:
/// `(reflectance_percent, nb_samples, distance_mm)`.
pub const INVALID_XTALK_PARAMS: [(u8, u8, u16); 6] =
    [(0, 4, 600), (100, 4, 600), (5, 0, 600), (5, 17, 600), (5, 4, 599), (5, 4, 3001)];

/// Issue the API calls of evo-f412re on-target test `name` against `d`. Where
/// the calls equal a Nucleo test's, that sequence is reused.
pub fn run_evo_sequence<D: Driver>(name: &str, d: &mut D) -> R<D> {
    match name {
        "async_i2c_alive" => run_sequence("test_sensor_alive", d),
        // The on-target test also logs `sensor.variant()`; no bus traffic.
        "init_uploads_firmware_over_async_i2c" => d.init(),
        "set_resolution" => run_sequence("test_set_resolution", d),
        // `poll_data_ready` on target: same traffic as the `check_data_ready` loop here.
        "ranging_4x4_async" => ranging(d, Resolution::Res4x4, 16, check_distances_plausible),
        "ranging_8x8_async" => ranging(d, Resolution::Res8x8, 64, check_distances_plausible),
        // The on-target test asserts on ambient light, which `Driver` does not expose; same calls.
        "ambient_light" => ranging(d, Resolution::Res4x4, 16, check_distances_plausible),
        "ranging_frequency" => {
            d.init()?;
            for hz in [5u8, 15] {
                d.set_ranging_frequency_hz(hz)?;
                assert_eq!(d.get_ranging_frequency_hz()?, hz);
            }
            Ok(())
        }
        "integration_time" => run_sequence("test_integration_time", d),
        "target_order" => run_sequence("test_target_order", d),
        "ranging_mode" => run_sequence("test_ranging_mode", d),
        "sharpener_percent" => run_sequence("test_sharpener_percent", d),
        "power_mode_sleep_wakeup" => run_sequence("test_power_mode", d),
        "calibrate_xtalk_leaves_sensor_usable" => {
            d.init()?;
            d.set_resolution(Resolution::Res4x4)?;
            d.set_ranging_frequency_hz(5)?;
            match d.calibrate_xtalk(5, 4, 600) {
                Ok(()) => {}
                Err(e) if D::firmware_error(&e) == Some(FirmwareError::XtalkFailed) => {}
                Err(e) => return Err(e),
            }
            assert_eq!(d.get_resolution()?, Resolution::Res4x4);
            assert_eq!(d.get_ranging_frequency_hz()?, 5);
            let mut buffer = [0u8; XTALK_BUFFER_SIZE];
            d.get_caldata_xtalk(&mut buffer)?;
            assert!(buffer.iter().any(|&b| b != 0), "xtalk caldata buffer is all zeros");
            d.start_ranging()?;
            poll_until_ready(d)?;
            let data = d.get_ranging_data()?;
            check_distances_plausible("after xtalk calibration", &data, 16);
            d.stop_ranging()
        }
        "calibrate_xtalk_rejects_invalid_params" => {
            d.init()?;
            d.set_resolution(Resolution::Res8x8)?;
            for (r, n, dist) in INVALID_XTALK_PARAMS {
                let result = d.calibrate_xtalk(r, n, dist);
                let rejected = matches!(&result, Err(e) if D::firmware_error(e) == Some(FirmwareError::InvalidParam));
                assert!(rejected, "calibrate_xtalk({r}, {n}, {dist}) was not rejected with InvalidParam: {result:?}");
            }
            assert_eq!(d.get_resolution()?, Resolution::Res8x8);
            Ok(())
        }
        other => panic!("no evo sequence named {other:?}"),
    }
}
