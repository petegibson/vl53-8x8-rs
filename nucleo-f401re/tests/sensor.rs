#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use vl53l8cx_driver::Resolution;

#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> nucleo_f401re::Sensor {
        nucleo_f401re::board_init()
    }

    #[test]
    fn test_sensor_alive(mut sensor: nucleo_f401re::Sensor) {
        assert!(sensor.is_alive().unwrap());
    }

    #[test]
    #[timeout(30)]
    fn test_init(mut sensor: nucleo_f401re::Sensor) {
        sensor.init().unwrap();
    }

    #[test]
    #[timeout(60)]
    fn test_set_resolution(mut sensor: nucleo_f401re::Sensor) {
        sensor.init().unwrap();

        sensor.set_resolution(Resolution::Res4x4).unwrap();
        assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res4x4);

        sensor.set_resolution(Resolution::Res8x8).unwrap();
        assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res8x8);
    }

    #[test]
    #[timeout(60)]
    fn test_ranging(mut sensor: nucleo_f401re::Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res4x4).unwrap();
        sensor.set_ranging_frequency_hz(10).unwrap();
        sensor.start_ranging().unwrap();

        // Poll until data is ready
        loop {
            if sensor.check_data_ready().unwrap() {
                break;
            }
        }

        let data = sensor.get_ranging_data().unwrap();
        // At least one zone should have a non-zero distance
        assert!(data.distance_mm.iter().any(|&d| d > 0));

        sensor.stop_ranging().unwrap();
    }
}
