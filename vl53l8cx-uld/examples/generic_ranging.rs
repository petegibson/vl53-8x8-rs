//! Generic ranging walk-through for the `vl53l8cx-uld` blocking driver.
//!
//! `run` is written against the `embedded-hal` 1.0 traits only, so it can be
//! called from any board crate with a concrete I2C bus and delay. A real
//! program must also invoke `vl53l8cx_uld::bind_platform_driver!(YourI2c,
//! YourDelay)` exactly once at crate level so the ST ULD can reach the bus.
//!
//! The `main` below does not touch hardware; it exists so the example compiles
//! as part of `cargo test --workspace`.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use vl53l8cx_uld::{Error, Resolution, Vl53l8cx};

/// Bring the sensor up, take one 4x4 frame and stop ranging.
pub fn run<I2C, D>(i2c: I2C, delay: D) -> Result<(), Error<I2C::Error>>
where
    I2C: I2c,
    D: DelayNs,
{
    let mut sensor = Vl53l8cx::new(i2c, delay);

    sensor.init()?;
    sensor.set_resolution(Resolution::Res4x4)?;
    sensor.start_ranging()?;

    // Poll until the first frame is available.
    while !sensor.check_data_ready()? {}
    let _frame = sensor.get_ranging_data()?;

    sensor.stop_ranging()?;
    Ok(())
}

fn main() {
    println!(
        "generic_ranging: call `run(i2c, delay)` from a board crate with a concrete \
         embedded-hal I2C bus and delay (plus `bind_platform_driver!`); see the demo crates."
    );
}
