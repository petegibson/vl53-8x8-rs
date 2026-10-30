//! Blocking ST-ULD example for a Pololu breakout on a Raspberry Pi Zero 2W:
//! VL53L8CX over SPI0 by default, VL53L5CX over I2C1 with `--features i2c-bus`
//! (see `lib.rs` for the wiring).
//!
//! Run on the Pi (or after copying the cross-compiled binary):
//!   cargo run --bin blocking
//!   cargo run --bin blocking --features i2c-bus
//!   sudo ./target/debug/blocking      # if the user isn't in the spi/i2c/gpio groups

use std::time::{Duration, Instant};

use anyhow::Context;
use rpi_zero_2w::uld::{self, Resolution};
use rpi_zero_2w::{bus_description, lpn_gpio, print_distance_grid, FIRMWARE_SIZE_KB, SENSOR_LABEL};

const RESOLUTION: Resolution = Resolution::Res4x4;
const RANGING_FREQ_HZ: u8 = 10;

fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    log::info!("{SENSOR_LABEL}: opening {} with LPn on BCM GPIO{}...", bus_description(), lpn_gpio());
    let mut sensor = uld::connect()?;

    anyhow::ensure!(
        sensor.is_alive().context("probe the sensor")?,
        "no sensor answered at the default address (wiring? bus mode selector? LPn high?)"
    );

    log::info!("Uploading firmware (~{FIRMWARE_SIZE_KB} KB)...");
    let started = Instant::now();
    sensor.init().context("sensor init / firmware upload")?;
    log::info!("Sensor ready ({:.1} s).", started.elapsed().as_secs_f32());

    let grid = match RESOLUTION {
        Resolution::Res4x4 => 4usize,
        Resolution::Res8x8 => 8,
    };
    sensor.set_resolution(RESOLUTION)?;
    sensor.set_ranging_frequency_hz(RANGING_FREQ_HZ)?;
    sensor.start_ranging()?;

    log::info!("Ranging {grid}x{grid} @ {RANGING_FREQ_HZ} Hz. Ctrl-C to stop.");
    loop {
        if sensor.check_data_ready()? {
            let data = sensor.get_ranging_data()?;
            print_distance_grid(&data.distance_mm, grid);
        } else {
            std::thread::sleep(Duration::from_millis(5));
        }
    }
}
