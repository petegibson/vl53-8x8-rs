//! Async VL53L8CX example for a Pololu breakout on a Raspberry Pi Zero 2W.
//!
//! Warning: **The SPI bus is *not* truly async.** `linux-embedded-hal`'s
//! `SpidevDevice` only implements the *blocking* `embedded-hal` SPI traits
//! (no `embedded-hal-async` impl exists upstream), so we wrap it in the
//! `BlockingSpi` adapter from `lib.rs` to satisfy the async `SpiDevice`
//! trait that `vl53-8x8` requires. Each transfer therefore **blocks
//! the Tokio task** for its duration. Only the driver's internal
//! `DelayNs` calls actually yield to the runtime.
//!
//! This is fine on the `current_thread` runtime used here. If you move
//! the driver to a multi-threaded runtime and care about responsiveness
//! for other tasks during the ~100 ms firmware upload, run the whole
//! `sensor.init()` (and ideally each `get_ranging_data()` call) inside
//! `tokio::task::spawn_blocking`.


use anyhow::Context;
use rpi_zero_2w::{
    firmware_upload_secs, format_freq, open_lpn, open_spi, print_distance_grid, BlockingSpi,
    TokioDelay, FIRMWARE_SIZE_KB, LPN_GPIO, SPI_DEVICE, SPI_FREQ_HZ, SPI_MODE_LABEL,
};
use vl53_8x8::{Resolution, Sensor};

const RESOLUTION: Resolution = Resolution::Res4x4;
const RANGING_FREQ_HZ: u8 = 10;

#[tokio::main(flavor = "current_thread")]
async fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let freq = format_freq(SPI_FREQ_HZ);
    log::info!("Opening {SPI_DEVICE} in {SPI_MODE_LABEL} at {freq}...");
    let spi = open_spi().context("open SPI device — is `dtparam=spi=on` set in /boot/firmware/config.txt?")?;

    log::info!("Acquiring LPn on BCM GPIO{LPN_GPIO}...");
    let lpn = open_lpn().context("open LPn — is the user in the gpio group, or run with sudo?")?;

    let mut sensor = Sensor::new_spi(BlockingSpi(spi), TokioDelay).with_lpn_pin(lpn);

    // open_lpn() leaves the line low; drive it high (comms enable). The Pololu
    // board has no PWR_EN, so there is no power_cycle(); init() reboots the
    // sensor in software.
    sensor.lpn_enable().context("drive LPn high")?;

    log::info!(
        "Uploading firmware (~{FIRMWARE_SIZE_KB} KB, ~{:.1} s at {freq})...",
        firmware_upload_secs()
    );
    sensor.init().await.context("sensor init / firmware upload")?;
    log::info!("Sensor ready.");

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
