//! The `Driver` trait both sensors are driven through, the `vl53-8x8` adapter,
//! and the call sequences that re-enact the on-target tests.
//!
//! The sequences follow the C suite (`examples/nucleo-f401re-uld/tests/sensor.rs`)
//! call for call. Where the Rust suite (`examples/nucleo-f401re-embassy/tests/
//! sensor_async.rs`) differs it is noted on the sequence: the only differences
//! are that the Rust suite waits for data with `Sensor::poll_data_ready`
//! (same bus traffic as a manual `check_data_ready` loop, plus a 10 ms delay
//! between polls) and that it has extra xtalk-calibration tests the C suite
//! lacks (not re-enacted). Assertions on returned values are kept where the
//! on-target test had them, so a driver that decodes a register differently is
//! caught as well as one that writes different bytes.

use std::fmt::Debug;

use vl53_8x8::{
    DetectionThreshold, MotionConfiguration, PowerMode, RangingMode, Resolution, TargetOrder,
    ThresholdMeasurement, ThresholdOperation, ThresholdType, XTALK_BUFFER_SIZE,
};

use crate::mock::{block_on, MockDelay, MockPin, MockSpi};

/// Upper bound on data-ready polls before the sequence gives up; on the mock a
/// loop that never sees "ready" would otherwise spin forever.
pub const MAX_POLLS: usize = 10_000;

/// The common surface of `vl53l8cx_uld::Vl53l8cx` and `vl53_8x8::Sensor`.
/// Motion configuration is kept inside the adapter: the two crates use
/// different (layout-identical) structs for it.
pub trait Driver {
    type Error: Debug;

    fn is_alive(&mut self) -> Result<bool, Self::Error>;
    fn init(&mut self) -> Result<(), Self::Error>;
    fn reset(&mut self) -> Result<(), Self::Error>;

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
}

// --- vl53-8x8 adapter ---------------------------------------------------------

pub type RustSensor = vl53_8x8::Sensor<vl53_8x8::BlockingSpi<MockSpi>, MockDelay, MockPin, MockPin>;

/// `vl53_8x8::Sensor` over the blocking SPI adapter, each call run to
/// completion with [`block_on`].
pub struct RustDriver {
    pub sensor: RustSensor,
    motion: MotionConfiguration,
}

impl RustDriver {
    pub fn new(spi: MockSpi, delay: MockDelay) -> Self {
        let sensor = vl53_8x8::Sensor::new(vl53_8x8::BlockingSpi(spi), delay)
            .with_reset_pin_active_low(MockPin)
            .with_interrupt_pin_active_low(MockPin);
        RustDriver { sensor, motion: unsafe { core::mem::zeroed() } }
    }
}

type RustError = vl53_8x8::Error<core::convert::Infallible>;

impl Driver for RustDriver {
    type Error = RustError;

    fn is_alive(&mut self) -> Result<bool, RustError> {
        block_on(self.sensor.is_alive())
    }
    fn init(&mut self) -> Result<(), RustError> {
        block_on(self.sensor.init())
    }
    fn reset(&mut self) -> Result<(), RustError> {
        block_on(self.sensor.reset())
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

fn check_distances(label: &str, distances: &[i16], zones: usize) {
    for (zone, &d) in distances[..zones].iter().enumerate() {
        assert!(d > 10 && d < 4000, "{label}: zone {zone} distance {d} mm out of range");
    }
}

/// Start ranging at 5 Hz in `res`, wait for a frame, read it, stop.
fn ranging<D: Driver>(d: &mut D, res: Resolution, zones: usize) -> R<D> {
    d.init()?;
    d.set_resolution(res)?;
    d.set_ranging_frequency_hz(5)?;
    d.start_ranging()?;
    poll_until_ready(d)?;
    let data = d.get_ranging_data()?;
    check_distances("ranging", &data, zones);
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
        "test_ranging_4x4" => ranging(d, Resolution::Res4x4, 16),
        "test_ranging_8x8" => ranging(d, Resolution::Res8x8, 64),
        // Same calls as test_ranging_4x4; the on-target test checks ambient instead of distance.
        "test_ambient_light" => ranging(d, Resolution::Res4x4, 16),
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
        // The on-target test busy-waits 100 ms after reset; no bus traffic.
        "test_reinit_after_reset" => {
            d.init()?;
            d.set_resolution(Resolution::Res8x8)?;
            d.reset()?;
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
            thresholds[0] = DetectionThreshold {
                zone_num: 0,
                measurement: ThresholdMeasurement::DistanceMm,
                threshold_type: ThresholdType::LessThanEqualMin,
                low_threshold: 500,
                high_threshold: 0,
                mathematic_operation: ThresholdOperation::None,
            };
            d.set_detection_thresholds(thresholds)?;
            let readback = d.get_detection_thresholds()?;
            assert_eq!(readback[0].zone_num, 0);
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
