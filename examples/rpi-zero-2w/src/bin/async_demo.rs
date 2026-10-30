//! Async pure-Rust (`vl53-8x8`) example for a Pololu breakout on a Raspberry
//! Pi Zero 2W: VL53L8CX over SPI0 by default, VL53L5CX over I2C1 with
//! `--features i2c-bus` (see `lib.rs` for the wiring). The driver autodetects
//! the part; the bin logs which one it found.
//!
//! The bus is *not* truly async: `linux-embedded-hal` only implements the
//! blocking `embedded-hal` traits, so transfers go through the driver's
//! `BlockingSpi` / `BlockingI2c` adapters and block the Tokio task for their
//! duration. Only the driver's `DelayNs` waits yield. That is fine on the
//! `current_thread` runtime used here; on a multi-threaded runtime wrap the
//! driver calls in `tokio::task::spawn_blocking`.

use std::time::Instant;

use anyhow::Context;
use rpi_zero_2w::native::{self, Resolution};
use rpi_zero_2w::{bus_description, lpn_gpio, print_distance_grid, FIRMWARE_SIZE_KB, SENSOR_LABEL};

const RESOLUTION: Resolution = Resolution::Res4x4;
const RANGING_FREQ_HZ: u8 = 10;

#[tokio::main(flavor = "current_thread")]
async fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    log::info!("{SENSOR_LABEL}: opening {} with LPn on BCM GPIO{}...", bus_description(), lpn_gpio());
    let mut sensor = native::connect()?;

    anyhow::ensure!(
        sensor.is_alive().await.context("probe the sensor")?,
        "no sensor answered at the default address (wiring? bus mode selector? LPn high?)"
    );

    log::info!("Uploading firmware (~{FIRMWARE_SIZE_KB} KB)...");
    let started = Instant::now();
    sensor.init().await.context("sensor init / firmware upload")?;
    log::info!(
        "Sensor ready ({:.1} s), detected variant: {:?} (module type {:?}; expected one of {:?}).",
        started.elapsed().as_secs_f32(),
        sensor.variant(),
        sensor.module_type(),
        native::EXPECTED_VARIANTS
    );

    let grid = match RESOLUTION {
        Resolution::Res4x4 => 4usize,
        Resolution::Res8x8 => 8,
    };
    sensor.set_resolution(RESOLUTION).await?;
    sensor.set_ranging_frequency_hz(RANGING_FREQ_HZ).await?;
    sensor.start_ranging().await?;

    log::info!("Ranging {grid}x{grid} @ {RANGING_FREQ_HZ} Hz. Ctrl-C to stop.");
    loop {
        sensor.poll_data_ready().await?;
        let data = sensor.get_ranging_data().await?;
        print_distance_grid(&data.distance_mm, grid);
    }
}
