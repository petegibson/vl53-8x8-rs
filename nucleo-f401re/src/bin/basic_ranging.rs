#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[cfg(not(feature = "spi-bus"))]
mod app {
    use cortex_m_rt::entry;
    use vl53l8cx_driver::Resolution;

    // --- Configuration ---
    const RESOLUTION: Resolution = Resolution::Res4x4;
    const RANGING_FREQ_HZ: u8 = 10;

    #[entry]
    fn main() -> ! {
        defmt::info!("VL53L8CX firmware starting");

        let mut sensor = nucleo_f401re::board_init();

        defmt::info!("Initialising sensor...");
        sensor.init().unwrap();

        let grid = match RESOLUTION {
            Resolution::Res4x4 => 4usize,
            Resolution::Res8x8 => 8,
        };
        defmt::info!("Configuring: {}x{} @ {} Hz", grid, grid, RANGING_FREQ_HZ);
        sensor.set_resolution(RESOLUTION).unwrap();
        sensor.set_ranging_frequency_hz(RANGING_FREQ_HZ).unwrap();

        sensor.start_ranging().unwrap();

        defmt::info!("Entering measurement loop");
        loop {
            match sensor.check_data_ready() {
                Ok(true) => match sensor.get_ranging_data() {
                    Ok(data) => {
                        defmt::info!("--- Distance (mm) ---");
                        for row in 0..grid {
                            let b = row * grid;
                            match grid {
                                4 => defmt::info!(
                                    "  {} {} {} {}",
                                    data.distance_mm[b],
                                    data.distance_mm[b + 1],
                                    data.distance_mm[b + 2],
                                    data.distance_mm[b + 3],
                                ),
                                _ => defmt::info!(
                                    "  {} {} {} {} {} {} {} {}",
                                    data.distance_mm[b],
                                    data.distance_mm[b + 1],
                                    data.distance_mm[b + 2],
                                    data.distance_mm[b + 3],
                                    data.distance_mm[b + 4],
                                    data.distance_mm[b + 5],
                                    data.distance_mm[b + 6],
                                    data.distance_mm[b + 7],
                                ),
                            }
                        }
                    }
                    Err(e) => {
                        defmt::error!("Ranging read error: {}", defmt::Debug2Format(&e));
                    }
                },
                Ok(false) => {}
                Err(e) => {
                    defmt::error!("Data ready check error: {}", defmt::Debug2Format(&e));
                }
            }
        }
    }
}

#[cfg(feature = "spi-bus")]
mod app {
    use cortex_m_rt::entry;
    use stm32f4xx_hal as _; // force-link device interrupt vectors

    #[entry]
    fn main() -> ! {
        defmt::error!("basic_ranging binary is I2C-only; use sensor_spi test for SPI");
        loop { cortex_m::asm::wfi() }
    }
}
