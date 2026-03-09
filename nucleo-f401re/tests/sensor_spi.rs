#![no_std]
#![no_main]

use defmt_rtt as _;

use vl53l8cx_driver::Resolution;

#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> nucleo_f401re::spi::Sensor {
        nucleo_f401re::spi::board_init_spi()
    }

    #[test]
    #[timeout(5)]
    fn test_sensor_alive_spi(mut sensor: nucleo_f401re::spi::Sensor) {
        defmt::assert!(
            sensor.is_alive().unwrap(),
            "sensor.is_alive failed over SPI, check X-NUCLEO-53L8A1 SPI jumpers"
        );
    }

    #[test]
    #[timeout(10)]
    fn test_init_spi(mut sensor: nucleo_f401re::spi::Sensor) {
        sensor.init().unwrap();
    }

    #[test]
    #[timeout(10)]
    fn test_ranging_4x4_spi(mut sensor: nucleo_f401re::spi::Sensor) {
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
    fn test_ranging_8x8_spi(mut sensor: nucleo_f401re::spi::Sensor) {
        sensor.init().unwrap();
        sensor.set_resolution(Resolution::Res8x8).unwrap();
        sensor.set_ranging_frequency_hz(5).unwrap();
        sensor.start_ranging().unwrap();

        loop {
            if sensor.check_data_ready().unwrap() {
                break;
            }
        }

        let data = sensor.get_ranging_data().unwrap();
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
