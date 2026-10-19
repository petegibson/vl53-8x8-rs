#![no_std]
#![no_main]

use defmt_rtt as _;

use vl53l8cx_uld::{
    PowerMode, RangingMode, Resolution, ResultsData, TargetOrder, NB_TARGET_PER_ZONE,
};

#[cfg(feature = "thresholds")]
use vl53l8cx_uld::{
    DetectionThreshold, MotionConfiguration, ThresholdMeasurement, ThresholdOperation,
    ThresholdType, VL53L8CX_LAST_THRESHOLD,
};

/// Assert that the first (closest/strongest) target of each of the first
/// `zones` zones has a plausible distance. Per-target arrays are laid out
/// `zone * NB_TARGET_PER_ZONE + target`, so this works for any `targets-N`.
fn assert_zone_distances(data: &ResultsData, zones: usize) {
    for zone in 0..zones {
        let d = data.distance_mm[zone * NB_TARGET_PER_ZONE];
        defmt::assert!(
            d > 10 && d < 4000,
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
    fn init() -> nucleo_f401re_uld::Sensor {
        nucleo_f401re_uld::board_init()
    }

    #[test]
    #[timeout(5)]
    fn test_sensor_alive(mut sensor: nucleo_f401re_uld::Sensor) {
        #[cfg(not(feature = "spi-bus"))]
        defmt::assert!(
            sensor.is_alive().unwrap(),
            "sensor.is_alive failed, check X-NUCLEO-53L8A1 is connected and I2C jumpers are set"
        );
        #[cfg(feature = "spi-bus")]
        defmt::assert!(
            sensor.is_alive().unwrap(),
            "sensor.is_alive failed, check X-NUCLEO-53L8A1 SPI jumpers are set (I2C_RST high)"
        );
    }

    #[test]
    #[timeout(10)]
    fn test_init(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();
    }

    #[test]
    #[timeout(10)]
    fn test_set_resolution(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();

        sensor.set_resolution(Resolution::Res4x4).unwrap();
        assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res4x4);

        sensor.set_resolution(Resolution::Res8x8).unwrap();
        assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res8x8);
    }

    #[test]
    #[timeout(10)]
    fn test_ranging_4x4(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res4x4).unwrap();
        sensor.set_ranging_frequency_hz(5).unwrap();
        sensor.start_ranging().unwrap();

        // Poll until data is ready
        loop {
            if sensor.check_data_ready().unwrap() {
                break;
            }
        }

        let data = sensor.get_ranging_data().unwrap();
        // All 4x4 zones should read between 10 mm and 4000 mm
        assert_zone_distances(&data, 16);

        sensor.stop_ranging().unwrap();
    }

    #[test]
    #[timeout(10)]
    fn test_ranging_8x8(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res8x8).unwrap();
        sensor.set_ranging_frequency_hz(5).unwrap();
        sensor.start_ranging().unwrap();

        // Poll until data is ready
        loop {
            if sensor.check_data_ready().unwrap() {
                break;
            }
        }

        let data = sensor.get_ranging_data().unwrap();
        // All 8x8 zones should read between 10 mm and 4000 mm
        assert_zone_distances(&data, 64);

        sensor.stop_ranging().unwrap();
    }

    #[test]
    #[timeout(10)]
    fn test_ambient_light(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res4x4).unwrap();
        sensor.set_ranging_frequency_hz(5).unwrap();
        sensor.start_ranging().unwrap();

        loop {
            if sensor.check_data_ready().unwrap() {
                break;
            }
        }

        let data = sensor.get_ranging_data().unwrap();
        // All 4x4 zones should report non-zero ambient (kcps/spad)
        for (i, &a) in data.ambient_per_spad[..16].iter().enumerate() {
            defmt::assert!(a > 0, "zone {} ambient_per_spad is zero", i);
        }

        sensor.stop_ranging().unwrap();
    }

    #[test]
    #[timeout(10)]
    fn test_ranging_frequency(mut sensor: nucleo_f401re_uld::Sensor) {
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
    fn test_target_order(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();

        sensor.set_target_order(TargetOrder::Closest).unwrap();
        assert_eq!(sensor.get_target_order().unwrap(), TargetOrder::Closest);

        sensor.set_target_order(TargetOrder::Strongest).unwrap();
        assert_eq!(sensor.get_target_order().unwrap(), TargetOrder::Strongest);
    }

    #[test]
    #[timeout(10)]
    fn test_ranging_mode(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();

        sensor.set_ranging_mode(RangingMode::Continuous).unwrap();
        assert_eq!(sensor.get_ranging_mode().unwrap(), RangingMode::Continuous);

        sensor.set_ranging_mode(RangingMode::Autonomous).unwrap();
        assert_eq!(sensor.get_ranging_mode().unwrap(), RangingMode::Autonomous);
    }

    #[test]
    #[timeout(10)]
    fn test_integration_time(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();

        sensor.set_integration_time_ms(20).unwrap();
        defmt::assert_eq!(sensor.get_integration_time_ms().unwrap(), 20);

        sensor.set_integration_time_ms(50).unwrap();
        defmt::assert_eq!(sensor.get_integration_time_ms().unwrap(), 50);
    }

    #[test]
    #[timeout(10)]
    fn test_sharpener_percent(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();

        // The sensor quantises via u8: set=(pct*255)/100, get=(raw*100)/255,
        // so read-back may differ by ±1 from the written value.
        sensor.set_sharpener_percent(25).unwrap();
        let v = sensor.get_sharpener_percent().unwrap();
        defmt::assert!(v >= 24 && v <= 26, "expected ~25, got {}", v);

        sensor.set_sharpener_percent(75).unwrap();
        let v = sensor.get_sharpener_percent().unwrap();
        defmt::assert!(v >= 74 && v <= 76, "expected ~75, got {}", v);
    }

    #[test]
    #[timeout(10)]
    fn test_power_mode(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();

        assert_eq!(sensor.get_power_mode().unwrap(), PowerMode::Wakeup);

        sensor.set_power_mode(PowerMode::Sleep).unwrap();
        assert_eq!(sensor.get_power_mode().unwrap(), PowerMode::Sleep);

        sensor.set_power_mode(PowerMode::Wakeup).unwrap();
        assert_eq!(sensor.get_power_mode().unwrap(), PowerMode::Wakeup);
    }

    /// Resolution is a stopped-sensor setting: the ULD sets it before
    /// `start_ranging`, and changing it mid-stream occasionally leaves the
    /// firmware reporting GO2 status 0xc5 at the following stop (seen once on
    /// the Nucleo under bus tracing). Stop, change, restart.
    #[test]
    #[timeout(10)]
    fn test_set_resolution_between_ranging_sessions(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res4x4).unwrap();
        sensor.set_ranging_frequency_hz(5).unwrap();
        sensor.start_ranging().unwrap();
        sensor.stop_ranging().unwrap();

        // Change resolution between two ranging sessions
        sensor.set_resolution(Resolution::Res8x8).unwrap();
        assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res8x8);

        sensor.start_ranging().unwrap();
        sensor.stop_ranging().unwrap();
    }

    #[test]
    #[timeout(15)]
    fn test_reinit_after_power_cycle(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res8x8).unwrap();

        // Power-cycle (PWR_EN_C + LPn, UM2884 §4.2) and re-init
        sensor.power_cycle().unwrap();
        cortex_m::asm::delay(84_000 * 100);
        sensor.init().unwrap();

        // Verify sensor is functional after reinit
        defmt::assert!(sensor.is_alive().unwrap());
        sensor.set_resolution(Resolution::Res4x4).unwrap();
        assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res4x4);
    }

    #[test]
    #[timeout(10)]
    fn test_vhv_repeat_count(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();

        sensor.set_vhv_repeat_count(5).unwrap();
        defmt::assert_eq!(sensor.get_vhv_repeat_count().unwrap(), 5);

        sensor.set_vhv_repeat_count(10).unwrap();
        defmt::assert_eq!(sensor.get_vhv_repeat_count().unwrap(), 10);
    }

    #[test]
    #[timeout(10)]
    fn test_external_sync_pin_enable(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();

        sensor.set_external_sync_pin_enable(true).unwrap();
        defmt::assert!(sensor.get_external_sync_pin_enable().unwrap());

        sensor.set_external_sync_pin_enable(false).unwrap();
        defmt::assert!(!sensor.get_external_sync_pin_enable().unwrap());
    }

    #[cfg(not(feature = "spi-bus"))]
    #[test]
    #[timeout(15)]
    fn test_set_address(mut sensor: nucleo_f401re_uld::Sensor) {
        let new_address: u8 = 0x35;
        sensor.set_i2c_address(new_address).unwrap();
        cortex_m::asm::delay(84_000 * 100);
        defmt::assert!(sensor.is_alive().unwrap());
    }

    #[test]
    #[timeout(10)]
    fn test_wait_for_data_ready_blocking(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res4x4).unwrap();
        sensor.set_ranging_frequency_hz(5).unwrap();
        sensor.start_ranging().unwrap();

        // Use the interrupt pin to wait instead of polling check_data_ready
        sensor.wait_for_data_ready_blocking().unwrap();

        let data = sensor.get_ranging_data().unwrap();
        assert_zone_distances(&data, 16);

        sensor.stop_ranging().unwrap();
    }

    #[cfg(feature = "thresholds")]
    #[test]
    #[timeout(10)]
    fn test_detection_thresholds_enable(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();

        sensor.set_detection_thresholds_enable(true).unwrap();
        defmt::assert!(sensor.get_detection_thresholds_enable().unwrap());

        sensor.set_detection_thresholds_enable(false).unwrap();
        defmt::assert!(!sensor.get_detection_thresholds_enable().unwrap());
    }

    #[cfg(feature = "thresholds")]
    #[test]
    #[timeout(10)]
    fn test_detection_thresholds_auto_stop(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();

        // Default after init is off.
        defmt::assert!(!sensor.get_detection_thresholds_auto_stop().unwrap());

        sensor.set_detection_thresholds_auto_stop(true).unwrap();
        defmt::assert!(sensor.get_detection_thresholds_auto_stop().unwrap());

        sensor.set_detection_thresholds_auto_stop(false).unwrap();
        defmt::assert!(!sensor.get_detection_thresholds_auto_stop().unwrap());
    }

    #[cfg(feature = "thresholds")]
    #[test]
    #[timeout(10)]
    fn test_detection_thresholds(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();

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

        sensor.set_detection_thresholds(thresholds).unwrap();
        let readback = sensor.get_detection_thresholds().unwrap();

        defmt::assert_eq!(readback[0].zone_num & !VL53L8CX_LAST_THRESHOLD, 0);
        defmt::assert_eq!(readback[0].low_threshold, 500);
    }

    #[cfg(feature = "thresholds")]
    #[test]
    #[timeout(10)]
    fn test_motion_indicator_init(mut sensor: nucleo_f401re_uld::Sensor) {
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
    fn test_motion_indicator_set_distance(mut sensor: nucleo_f401re_uld::Sensor) {
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
    fn test_motion_indicator_set_resolution(mut sensor: nucleo_f401re_uld::Sensor) {
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
    fn test_xtalk_margin(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();

        sensor.set_xtalk_margin(100).unwrap();
        defmt::assert_eq!(sensor.get_xtalk_margin().unwrap(), 100);

        sensor.set_xtalk_margin(50).unwrap();
        defmt::assert_eq!(sensor.get_xtalk_margin().unwrap(), 50);
    }

    #[cfg(feature = "xtalk")]
    #[test]
    #[timeout(10)]
    fn test_caldata_xtalk(mut sensor: nucleo_f401re_uld::Sensor) {
        sensor.init().unwrap();

        let mut buffer = [0u8; vl53l8cx_uld::XTALK_BUFFER_SIZE];
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
