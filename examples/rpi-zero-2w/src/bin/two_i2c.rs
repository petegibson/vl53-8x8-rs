//! Two VL53L5CX-class boards on one I2C bus (`--features i2c-bus`): the
//! multi-sensor bring-up from UM2884 §4.1 with the pure-Rust driver.
//!
//! Both Pololu carriers sit on I2C1 and pull their LPn high, so at power-up
//! both would answer at the default address 0x29. The bin claims both LPn
//! lines low, enables the second board (GPIO6) alone and moves it to
//! `SECOND_ADDRESS` (0x2A), then enables the first board (GPIO5) at 0x29,
//! initialises both and streams a 4x4 grid from each. Each board gets its
//! own `/dev/i2c-1` handle; transfers are whole ioctls, so two handles on one
//! bus are safe on the single-threaded runtime used here.
//!
//! The second board keeps 0x2A until it is power-cycled; the test suites'
//! `connect()` moves it back automatically (see `STRAY_ADDRESSES`).
//!
//!   cargo run --bin two_i2c --features i2c-bus

use std::time::{Duration, Instant};

use anyhow::Context;
use rpi_zero_2w::native::{BlockingI2c, Resolution, Sensor, DEFAULT_ADDRESS};
use rpi_zero_2w::{
    open_all_lpn, open_i2c, print_distance_grid, TokioDelay, FIRMWARE_SIZE_KB, I2C_DEVICE, LPN_HOLD_LOW,
    SECOND_ADDRESS,
};

const RESOLUTION: Resolution = Resolution::Res4x4;
const RANGING_FREQ_HZ: u8 = 10;
const INTEGRATION_MS: u32 = 20;

#[tokio::main(flavor = "current_thread")]
async fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let mut lpns = open_all_lpn().context("claim the LPn lines")?;
    anyhow::ensure!(lpns.len() >= 2, "this bin needs two LPn lines, got {:?}", lpns.len());
    let (gpio_a, lpn_a) = lpns.remove(0);
    let (gpio_b, lpn_b) = lpns.remove(0);
    log::info!("Both LPn lines (GPIO{gpio_a}, GPIO{gpio_b}) held low; opening {I2C_DEVICE} twice...");
    std::thread::sleep(LPN_HOLD_LOW);

    // Second board alone on the bus: give it its own address.
    let mut b = Sensor::new(BlockingI2c(open_i2c()?), TokioDelay).with_lpn_pin(lpn_b);
    b.lpn_enable().context("enable second board")?;
    std::thread::sleep(LPN_HOLD_LOW);
    anyhow::ensure!(
        b.is_alive().await.context("probe second board")?,
        "no sensor answered on LPn GPIO{gpio_b} at the default address"
    );
    b.set_i2c_address(SECOND_ADDRESS).await.context("move second board")?;
    log::info!("Board on GPIO{gpio_b} moved to {SECOND_ADDRESS:#04x}.");

    // First board at the default address.
    let mut a = Sensor::new(BlockingI2c(open_i2c()?), TokioDelay).with_lpn_pin(lpn_a);
    a.lpn_enable().context("enable first board")?;
    std::thread::sleep(LPN_HOLD_LOW);
    anyhow::ensure!(
        a.is_alive().await.context("probe first board")?,
        "no sensor answered on LPn GPIO{gpio_a} at the default address"
    );

    for (name, s) in [("A", &mut a), ("B", &mut b)] {
        log::info!("{name}: uploading firmware (~{FIRMWARE_SIZE_KB} KB)...");
        let started = Instant::now();
        s.init().await.with_context(|| format!("init board {name}"))?;
        log::info!("{name}: ready ({:.1} s), detected variant {:?}.", started.elapsed().as_secs_f32(), s.variant());
        s.set_resolution(RESOLUTION).await?;
        s.set_ranging_frequency_hz(RANGING_FREQ_HZ).await?;
        s.set_integration_time_ms(INTEGRATION_MS).await?;
        s.start_ranging().await?;
    }

    let grid = match RESOLUTION {
        Resolution::Res4x4 => 4usize,
        Resolution::Res8x8 => 8,
    };
    log::info!(
        "Ranging both at {grid}x{grid} @ {RANGING_FREQ_HZ} Hz: A = GPIO{gpio_a} @ {DEFAULT_ADDRESS:#04x}, \
         B = GPIO{gpio_b} @ {SECOND_ADDRESS:#04x}. Ctrl-C to stop."
    );
    loop {
        let mut idle = true;
        for (name, s) in [("A", &mut a), ("B", &mut b)] {
            if s.check_data_ready().await? {
                let data = s.get_ranging_data().await?;
                println!("[{name}]");
                print_distance_grid(&data.distance_mm, grid);
                idle = false;
            }
        }
        if idle {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }
}
