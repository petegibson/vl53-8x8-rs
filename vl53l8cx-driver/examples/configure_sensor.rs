use anyhow::Result;
use embedded_hal::{delay::DelayNs, i2c::I2c};
use i2cdev::linux::LinuxI2cDev;
use vl53l5cx_driver::{Vl53l5cx, Resolution, Vl53l5cxComms};

/// A simple delay provider for `std` environments that uses `thread::sleep`.
struct StdDelay;

impl DelayNs for StdDelay {
    fn delay_ns(&mut self, ns: u32) {
        let duration = std::time::Duration::from_nanos(ns as u64);
        std::thread::sleep(duration);
    }
}

fn main() -> Result<()> {
    // 1. Define Hardware specifics
    const I2C_BUS_PATH: &str = "/dev/i2c-2";

    // 2. Initialize platform-specific peripherals
    println!("Opening I2C bus {}...", I2C_BUS_PATH);
    let i2c = LinuxI2cDev::new(I2C_BUS_PATH)
        .map_err(|e| anyhow::anyhow!("Failed to open bus: {}", e))?;
    let delay = StdDelay;

    // 3. Initialize Sensor
    // Note: This uploads ~90KB firmware via I2C
    println!("Initializing VL53L5CX...");
    let mut sensor = Vl53l5cx::new(i2c, delay);
    sensor.init()?;
    println!("Sensor Init Complete.");

    // 5. Verify
    if sensor.is_alive()? {
        println!("Sensor is Alive!");
    } else {
        anyhow::bail!("Sensor did not respond to ping!");
    }

    // 6. Configure for Conveyor (8x8, 15Hz)
    sensor.set_resolution(Resolution::Res8x8)?;
    sensor.set_ranging_frequency_hz(15)?;
    
    println!("Configuration applied. Ready for Volume Analysis.");
    Ok(())
}