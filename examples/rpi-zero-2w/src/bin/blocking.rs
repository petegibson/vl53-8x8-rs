//! Blocking VL53L8CX example for a Pololu breakout on a Raspberry Pi Zero 2W.
//!
//! Run on the Pi (or after copying the cross-compiled binary):
//!   cargo run --bin blocking
//!   sudo ./target/debug/blocking      # if the user isn't in the spi/gpio groups

use std::time::Duration;

use anyhow::Context;
use rpi_zero_2w::{
    firmware_upload_secs, format_freq, open_lpn, open_spi, print_distance_grid, LpnPin, SensorSpi,
    StdDelay, FIRMWARE_SIZE_KB, LPN_GPIO, SPI_DEVICE, SPI_FREQ_HZ, SPI_MODE_LABEL,
};
use vl53l8cx_uld::{Resolution, Vl53l8cx};

// Generate the ST-ULD platform callbacks bound to our chosen bus + delay type.
vl53l8cx_uld::bind_platform_driver_spi!(SensorSpi, StdDelay);

const RESOLUTION: Resolution = Resolution::Res4x4;
const RANGING_FREQ_HZ: u8 = 10;

fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let freq = format_freq(SPI_FREQ_HZ);
    log::info!("Opening {SPI_DEVICE} in {SPI_MODE_LABEL} at {freq}...");
    let spi: SensorSpi = open_spi().context("open SPI device — is `dtparam=spi=on` set in /boot/firmware/config.txt?")?;

    log::info!("Acquiring LPn on BCM GPIO{LPN_GPIO}...");
    let lpn: LpnPin = open_lpn().context("open LPn — is the user in the gpio group, or run with sudo?")?;

    let mut sensor = Vl53l8cx::new_spi(spi, StdDelay).with_reset_pin_active_low(lpn);

    sensor.reset().context("sensor reset (LPn toggle)")?;
    std::thread::sleep(Duration::from_millis(10));

    log::info!(
        "Uploading firmware (~{FIRMWARE_SIZE_KB} KB, ~{:.1} s at {freq})...",
        firmware_upload_secs()
    );
    sensor.init().context("sensor init / firmware upload")?;
    log::info!("Sensor ready.");

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
