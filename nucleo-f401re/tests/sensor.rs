#![no_std]
#![no_main]

use defmt_rtt as _;

use vl53l8cx_driver::Resolution;

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
}
