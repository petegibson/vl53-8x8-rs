#![no_std]
#![no_main]

//! On-target tests for ST's VL53L5CX ULD (`vl53l5cx-uld`) over blocking
//! `stm32f4xx-hal` I2C on the evo STM32F412RE board.
//!
//! Ported from `examples/nucleo-f401re-uld/tests/sensor.rs` (the VL53L8CX
//! twin) with the L8-only tests removed: no external sync pin, no detection-
//! threshold auto-stop and no deep-sleep power mode on the VL53L5CX. The
//! L5-only `enable_internal_cp` / `disable_internal_cp` round-trip is added.
//!
//! The board must have a **VL53L5CX** fitted; the C wrapper cannot drive a
//! VL53L8CX. Scene: whatever is above the bench (ceiling-type, no fixed
//! target), so distances are only bounded, not matched.

use defmt_rtt as _;
// Panic handler is provided by embedded-test (do not also link panic-probe).

use vl53l5cx_uld::{
    PowerMode, RangingMode, Resolution, ResultsData, TargetOrder, NB_TARGET_PER_ZONE,
};

#[cfg(feature = "thresholds")]
use vl53l5cx_uld::{
    DetectionThreshold, MotionConfiguration, ThresholdMeasurement, ThresholdOperation,
    ThresholdType,
};

use evo_f412re_uld::{Sensor, SYSCLK_HZ};

const CYCLES_PER_MS: u32 = SYSCLK_HZ / 1000;

/// Assert that the first (closest/strongest) target of each of the first
/// `zones` zones has a plausible distance. Per-target arrays are laid out
/// `zone * NB_TARGET_PER_ZONE + target`, so this works for any `targets-N`.
fn assert_zone_distances(data: &ResultsData, zones: usize) {
    for zone in 0..zones {
        let d = data.distance_mm[zone * NB_TARGET_PER_ZONE];
        defmt::assert!(
            d > 0 && d < 4000,
            "zone {} distance {} mm out of range",
            zone,
            d
        );
    }
}

#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Sensor {
        evo_f412re_uld::board_init()
    }

    #[test]
    #[timeout(5)]
    fn test_sensor_alive(mut sensor: Sensor) {
        defmt::assert!(
            sensor.is_alive().unwrap(),
            "sensor.is_alive failed, check the board is powered and a VL53L5CX is fitted"
        );
    }

    #[test]
    #[timeout(10)]
    fn test_init(mut sensor: Sensor) {
        sensor.init().unwrap();
    }

    #[test]
    #[timeout(10)]
    fn test_set_resolution(mut sensor: Sensor) {
        sensor.init().unwrap();

        sensor.set_resolution(Resolution::Res4x4).unwrap();
        assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res4x4);

        sensor.set_resolution(Resolution::Res8x8).unwrap();
        assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res8x8);
    }

    #[test]
    #[timeout(10)]
    fn test_ranging_4x4(mut sensor: Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res4x4).unwrap();
        sensor.set_ranging_frequency_hz(5).unwrap();
        sensor.start_ranging().unwrap();

        while !sensor.check_data_ready().unwrap() {}

        let data = sensor.get_ranging_data().unwrap();
        assert_zone_distances(&data, 16);

        sensor.stop_ranging().unwrap();
    }

    #[test]
    #[timeout(10)]
    fn test_ranging_8x8(mut sensor: Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res8x8).unwrap();
        sensor.set_ranging_frequency_hz(5).unwrap();
        sensor.start_ranging().unwrap();

        while !sensor.check_data_ready().unwrap() {}

        let data = sensor.get_ranging_data().unwrap();
        assert_zone_distances(&data, 64);

        sensor.stop_ranging().unwrap();
    }

    #[test]
    #[timeout(10)]
    fn test_ambient_light(mut sensor: Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res4x4).unwrap();
        sensor.set_ranging_frequency_hz(5).unwrap();
        sensor.start_ranging().unwrap();

        while !sensor.check_data_ready().unwrap() {}

        let data = sensor.get_ranging_data().unwrap();
        // Per-zone ambient rounds to 0 in low light on the L5, so only require
        // that the block transferred and at least one zone reports light.
        let nonzero = data.ambient_per_spad[..16].iter().filter(|&&a| a > 0).count();
        defmt::info!("ambient: {}/16 zones non-zero", nonzero);
        defmt::assert!(nonzero > 0, "no zone reported ambient light (sensor in the dark?)");

        sensor.stop_ranging().unwrap();
    }

    #[test]
    #[timeout(10)]
    fn test_ranging_frequency(mut sensor: Sensor) {
        sensor.init().unwrap();

        sensor.set_ranging_frequency_hz(5).unwrap();
        defmt::assert_eq!(sensor.get_ranging_frequency_hz().unwrap(), 5);

        sensor.set_ranging_frequency_hz(15).unwrap();
        defmt::assert_eq!(sensor.get_ranging_frequency_hz().unwrap(), 15);

        sensor.set_ranging_frequency_hz(30).unwrap();
        defmt::assert_eq!(sensor.get_ranging_frequency_hz().unwrap(), 30);
    }

    #[test]
    #[timeout(10)]
    fn test_target_order(mut sensor: Sensor) {
        sensor.init().unwrap();

        sensor.set_target_order(TargetOrder::Closest).unwrap();
        assert_eq!(sensor.get_target_order().unwrap(), TargetOrder::Closest);

        sensor.set_target_order(TargetOrder::Strongest).unwrap();
        assert_eq!(sensor.get_target_order().unwrap(), TargetOrder::Strongest);
    }

    #[test]
    #[timeout(10)]
    fn test_ranging_mode(mut sensor: Sensor) {
        sensor.init().unwrap();

        sensor.set_ranging_mode(RangingMode::Continuous).unwrap();
        assert_eq!(sensor.get_ranging_mode().unwrap(), RangingMode::Continuous);

        sensor.set_ranging_mode(RangingMode::Autonomous).unwrap();
        assert_eq!(sensor.get_ranging_mode().unwrap(), RangingMode::Autonomous);
    }

    #[test]
    #[timeout(10)]
    fn test_integration_time(mut sensor: Sensor) {
        sensor.init().unwrap();

        sensor.set_integration_time_ms(20).unwrap();
        defmt::assert_eq!(sensor.get_integration_time_ms().unwrap(), 20);

        sensor.set_integration_time_ms(50).unwrap();
        defmt::assert_eq!(sensor.get_integration_time_ms().unwrap(), 50);
    }

    #[test]
    #[timeout(10)]
    fn test_sharpener_percent(mut sensor: Sensor) {
        sensor.init().unwrap();

        // The sensor quantises via u8: set=(pct*255)/100, get=(raw*100)/255,
        // so read-back may differ by +-1 from the written value.
        sensor.set_sharpener_percent(25).unwrap();
        let v = sensor.get_sharpener_percent().unwrap();
        defmt::assert!((24..=26).contains(&v), "expected ~25, got {}", v);

        sensor.set_sharpener_percent(75).unwrap();
        let v = sensor.get_sharpener_percent().unwrap();
        defmt::assert!((74..=76).contains(&v), "expected ~75, got {}", v);
    }

    /// Sleep/Wakeup only: the VL53L5CX ULD has no deep-sleep mode, and the
    /// wrapper rejects it without touching the bus.
    #[test]
    #[timeout(10)]
    fn test_power_mode(mut sensor: Sensor) {
        use vl53l5cx_uld::{Error, FirmwareError};
        sensor.init().unwrap();

        assert_eq!(sensor.get_power_mode().unwrap(), PowerMode::Wakeup);

        sensor.set_power_mode(PowerMode::Sleep).unwrap();
        assert_eq!(sensor.get_power_mode().unwrap(), PowerMode::Sleep);

        sensor.set_power_mode(PowerMode::Wakeup).unwrap();
        assert_eq!(sensor.get_power_mode().unwrap(), PowerMode::Wakeup);

        defmt::assert!(
            matches!(
                sensor.set_power_mode(PowerMode::DeepSleep),
                Err(Error::Firmware(FirmwareError::InvalidParam))
            ),
            "DeepSleep must be rejected with InvalidParam on the VL53L5CX"
        );
        assert_eq!(sensor.get_power_mode().unwrap(), PowerMode::Wakeup);
    }

    /// L5-only: the internal charge pump can be toggled; the sensor must stay
    /// responsive and keep ranging afterwards.
    #[test]
    #[timeout(15)]
    fn test_internal_cp_round_trip(mut sensor: Sensor) {
        sensor.init().unwrap();

        sensor.disable_internal_cp().unwrap();
        defmt::assert!(sensor.is_alive().unwrap());

        sensor.enable_internal_cp().unwrap();
        defmt::assert!(sensor.is_alive().unwrap());

        sensor.set_resolution(Resolution::Res4x4).unwrap();
        sensor.set_ranging_frequency_hz(5).unwrap();
        sensor.start_ranging().unwrap();
        while !sensor.check_data_ready().unwrap() {}
        let data = sensor.get_ranging_data().unwrap();
        assert_zone_distances(&data, 16);
        sensor.stop_ranging().unwrap();
    }

    #[test]
    #[timeout(10)]
    fn test_set_resolution_while_ranging(mut sensor: Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res4x4).unwrap();
        sensor.set_ranging_frequency_hz(5).unwrap();
        sensor.start_ranging().unwrap();

        // Change resolution while ranging
        sensor.set_resolution(Resolution::Res8x8).unwrap();
        assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res8x8);

        sensor.stop_ranging().unwrap();
    }

    /// `reset()` pulses I2C_RESET (PB8) for 10 ms; the sensor must come back
    /// at the default address and accept a fresh firmware upload.
    #[test]
    #[timeout(20)]
    fn test_reinit_after_reset(mut sensor: Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res8x8).unwrap();

        sensor.reset().unwrap();
        cortex_m::asm::delay(CYCLES_PER_MS * 100);
        sensor.init().unwrap();

        defmt::assert!(sensor.is_alive().unwrap());
        sensor.set_resolution(Resolution::Res4x4).unwrap();
        assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res4x4);
    }

    #[test]
    #[timeout(10)]
    fn test_vhv_repeat_count(mut sensor: Sensor) {
        sensor.init().unwrap();

        sensor.set_vhv_repeat_count(5).unwrap();
        defmt::assert_eq!(sensor.get_vhv_repeat_count().unwrap(), 5);

        sensor.set_vhv_repeat_count(10).unwrap();
        defmt::assert_eq!(sensor.get_vhv_repeat_count().unwrap(), 10);
    }

    /// The sensor is power-cycled by `board_init` before every test, so the
    /// new address does not leak into the next one.
    #[test]
    #[timeout(15)]
    fn test_set_address(mut sensor: Sensor) {
        let new_address: u8 = 0x35;
        sensor.set_i2c_address(new_address).unwrap();
        cortex_m::asm::delay(CYCLES_PER_MS * 100);
        defmt::assert!(sensor.is_alive().unwrap());
        sensor.init().unwrap();
    }

    #[cfg(feature = "thresholds")]
    #[test]
    #[timeout(10)]
    fn test_detection_thresholds_enable(mut sensor: Sensor) {
        sensor.init().unwrap();

        sensor.set_detection_thresholds_enable(true).unwrap();
        defmt::assert!(sensor.get_detection_thresholds_enable().unwrap());

        sensor.set_detection_thresholds_enable(false).unwrap();
        defmt::assert!(!sensor.get_detection_thresholds_enable().unwrap());
    }

    #[cfg(feature = "thresholds")]
    #[test]
    #[timeout(10)]
    fn test_detection_thresholds(mut sensor: Sensor) {
        sensor.init().unwrap();

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

        sensor.set_detection_thresholds(thresholds).unwrap();
        let readback = sensor.get_detection_thresholds().unwrap();

        defmt::assert_eq!(readback[0].zone_num, 0);
        defmt::assert_eq!(readback[0].low_threshold, 500);
    }

    #[cfg(feature = "thresholds")]
    #[test]
    #[timeout(10)]
    fn test_motion_indicator_init(mut sensor: Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res4x4).unwrap();

        let mut motion_config: MotionConfiguration = unsafe { core::mem::zeroed() };
        sensor
            .motion_indicator_init(&mut motion_config, Resolution::Res4x4)
            .unwrap();
    }

    #[cfg(feature = "thresholds")]
    #[test]
    #[timeout(10)]
    fn test_motion_indicator_set_distance(mut sensor: Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res4x4).unwrap();

        let mut motion_config: MotionConfiguration = unsafe { core::mem::zeroed() };
        sensor
            .motion_indicator_init(&mut motion_config, Resolution::Res4x4)
            .unwrap();
        sensor
            .motion_indicator_set_distance_motion(&mut motion_config, 400, 1500)
            .unwrap();
    }

    #[cfg(feature = "thresholds")]
    #[test]
    #[timeout(10)]
    fn test_motion_indicator_set_resolution(mut sensor: Sensor) {
        sensor.init().unwrap();

        let mut motion_config: MotionConfiguration = unsafe { core::mem::zeroed() };
        sensor
            .motion_indicator_init(&mut motion_config, Resolution::Res4x4)
            .unwrap();
        sensor
            .motion_indicator_set_resolution(&mut motion_config, Resolution::Res8x8)
            .unwrap();
    }

    #[cfg(feature = "xtalk")]
    #[test]
    #[timeout(10)]
    fn test_xtalk_margin(mut sensor: Sensor) {
        sensor.init().unwrap();

        sensor.set_xtalk_margin(100).unwrap();
        defmt::assert_eq!(sensor.get_xtalk_margin().unwrap(), 100);

        sensor.set_xtalk_margin(50).unwrap();
        defmt::assert_eq!(sensor.get_xtalk_margin().unwrap(), 50);
    }

    #[cfg(feature = "xtalk")]
    #[test]
    #[timeout(10)]
    fn test_caldata_xtalk(mut sensor: Sensor) {
        sensor.init().unwrap();

        let mut buffer = [0u8; vl53l5cx_uld::XTALK_BUFFER_SIZE];
        sensor.get_caldata_xtalk(&mut buffer).unwrap();

        // Buffer should have been filled with calibration data
        let non_zero = buffer.iter().any(|&b| b != 0);
        defmt::assert!(non_zero, "xtalk caldata buffer is all zeros");

        // Round-trip: set it back and re-read
        let saved = buffer;
        sensor.set_caldata_xtalk(&mut buffer).unwrap();
        sensor.get_caldata_xtalk(&mut buffer).unwrap();
        defmt::assert!(buffer == saved, "xtalk caldata round-trip mismatch");
    }
}
