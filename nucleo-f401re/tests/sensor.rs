#![no_std]
#![no_main]

use defmt_rtt as _;

use vl53l8cx_driver::{PowerMode, RangingMode, Resolution, TargetOrder};

#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> nucleo_f401re::Sensor {
        nucleo_f401re::board_init()
    }

    #[test]
    #[timeout(5)]
    fn test_sensor_alive(mut sensor: nucleo_f401re::Sensor) {
        defmt::assert!(
            sensor.is_alive().unwrap(),
            "sensor.is_alive failed, check X-NUCLEO-53L8A1 is connected"
            );
    }

    #[test]
    #[timeout(10)]
    fn test_init(mut sensor: nucleo_f401re::Sensor) {
        sensor.init().unwrap();
    }

    #[test]
    #[timeout(10)]
    fn test_set_resolution(mut sensor: nucleo_f401re::Sensor) {
        sensor.init().unwrap();

        sensor.set_resolution(Resolution::Res4x4).unwrap();
        assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res4x4);

        sensor.set_resolution(Resolution::Res8x8).unwrap();
        assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res8x8);
    }

    #[test]
    #[timeout(10)]
    fn test_ranging_4x4(mut sensor: nucleo_f401re::Sensor) {
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
        // All 4x4 zones should read between 10 mm and 2000 mm
        for (i, &d) in data.distance_mm[..16].iter().enumerate() {
            defmt::assert!(
                d > 10 && d < 4000,
                "zone {} distance {} mm out of range",
                i,
                d
            );
        }

        sensor.stop_ranging().unwrap();
    }

    #[test]
    #[timeout(10)]
    fn test_ranging_8x8(mut sensor: nucleo_f401re::Sensor) {
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
        // All 4x4 zones should read between 10 mm and 2000 mm
        for (i, &d) in data.distance_mm[..64].iter().enumerate() {
            defmt::assert!(
                d > 10 && d < 4000,
                "zone {} distance {} mm out of range",
                i,
                d
            );
        }

        sensor.stop_ranging().unwrap();
    }

    #[test]
    #[timeout(10)]
    fn test_ambient_light(mut sensor: nucleo_f401re::Sensor) {
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
    fn test_ranging_frequency(mut sensor: nucleo_f401re::Sensor) {
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
    fn test_target_order(mut sensor: nucleo_f401re::Sensor) {
        sensor.init().unwrap();

        sensor.set_target_order(TargetOrder::Closest).unwrap();
        assert_eq!(sensor.get_target_order().unwrap(), TargetOrder::Closest);

        sensor.set_target_order(TargetOrder::Strongest).unwrap();
        assert_eq!(sensor.get_target_order().unwrap(), TargetOrder::Strongest);
    }

    #[test]
    #[timeout(10)]
    fn test_ranging_mode(mut sensor: nucleo_f401re::Sensor) {
        sensor.init().unwrap();

        sensor.set_ranging_mode(RangingMode::Continuous).unwrap();
        assert_eq!(sensor.get_ranging_mode().unwrap(), RangingMode::Continuous);

        sensor.set_ranging_mode(RangingMode::Autonomous).unwrap();
        assert_eq!(sensor.get_ranging_mode().unwrap(), RangingMode::Autonomous);
    }

    #[test]
    #[timeout(10)]
    fn test_integration_time(mut sensor: nucleo_f401re::Sensor) {
        sensor.init().unwrap();

        sensor.set_integration_time_ms(20).unwrap();
        defmt::assert_eq!(sensor.get_integration_time_ms().unwrap(), 20);

        sensor.set_integration_time_ms(50).unwrap();
        defmt::assert_eq!(sensor.get_integration_time_ms().unwrap(), 50);
    }

    #[test]
    #[timeout(10)]
    fn test_sharpener_percent(mut sensor: nucleo_f401re::Sensor) {
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
    fn test_power_mode(mut sensor: nucleo_f401re::Sensor) {
        sensor.init().unwrap();

        assert_eq!(sensor.get_power_mode().unwrap(), PowerMode::Wakeup);

        sensor.set_power_mode(PowerMode::Sleep).unwrap();
        assert_eq!(sensor.get_power_mode().unwrap(), PowerMode::Sleep);

        sensor.set_power_mode(PowerMode::Wakeup).unwrap();
        assert_eq!(sensor.get_power_mode().unwrap(), PowerMode::Wakeup);
    }

    #[test]
    #[timeout(10)]
    fn test_set_resolution_while_ranging(mut sensor: nucleo_f401re::Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res4x4).unwrap();
        sensor.set_ranging_frequency_hz(5).unwrap();
        sensor.start_ranging().unwrap();

        // Change resolution while ranging
        sensor.set_resolution(Resolution::Res8x8).unwrap();
        assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res8x8);

        sensor.stop_ranging().unwrap();
    }

    #[test]
    #[timeout(15)]
    fn test_reinit_after_reset(mut sensor: nucleo_f401re::Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res8x8).unwrap();

        // Reset and re-init
        sensor.reset().unwrap();
        cortex_m::asm::delay(84_000 * 100);
        sensor.init().unwrap();

        // Verify sensor is functional after reinit
        defmt::assert!(sensor.is_alive().unwrap());
        sensor.set_resolution(Resolution::Res4x4).unwrap();
        assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res4x4);
    }
}
