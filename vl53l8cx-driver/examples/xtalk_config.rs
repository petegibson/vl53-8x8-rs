use anyhow::Result;
use embedded_hal::delay::DelayNs;
use vl53l5cx_driver::{Vl53l5cx};
use i2cdev::linux::LinuxI2cDev;

struct StdDelay;
impl DelayNs for StdDelay {
    fn delay_ns(&mut self, ns: u32) {
        let duration = std::time::Duration::from_nanos(ns as u64);
        std::thread::sluse anyhow::Result;
use embedded_hal::delay::DelayNs;
use vl53l5cx_driver::{Vl53l5cx};
use i2cdev::linux::LinuxI2cDev;

struct StdDelay;
impl DelayNs for StdDelay {
    fn delay_ns(&mut self, ns: u32) {
        let duration = std::time::Duration::from_nanos(ns as u64);
        std::thread::sleep(duration);
    }
}

fn main() -> Result<()> {
    // Requires initialization and the "xtalk" feature enabled.
    const I2C_BUS_PATH: &str = "/dev/i2c-2";

    // ... (Setup Peripherals boilerplate)
    let i2c = LinuxI2cDev::new(I2C_BUS_PATH)
        .map_err(|e| anyhow::anyhow!("Failed to open bus: {}", e))?;
    let delay = StdDelay;

    println!("Initializing VL53L5CX for Xtalk configuration...");
    let mut sensor = Vl53l5cx::new(i2c, delay);
    sensor.init()?;

    // --- Xtalk Margin Accessors ---
    let initial_margin = sensor.get_xtalk_margin()?;
    println!("Initial Xtalk Margin: {} kcps/spads", initial_margin);

    // Set a moderate margin (e.g., 75 kcps/spads)
    const NEW_MARGIN: u32 = 75; 
    sensor.set_xtalk_margin(NEW_MARGIN)?;
    println!("Set new Xtalk Margin to {} kcps/spads.", NEW_MARGIN);

    let final_margin = sensor.get_xtalk_margin()?;
    println!("Verified final Xtalk Margin: {} kcps/spads", final_margin);
    
    Ok(())
}eep(duration);
    }
}

fn main() -> Result<()> {
    // Requires initialization and the "xtalk" feature enabled.
    const I2C_BUS_PATH: &str = "/dev/i2c-2";

    // ... (Setup Peripherals boilerplate, assuming it's identical to Example 1)
    let i2c = LinuxI2cDev::new(I2C_BUS_PATH)
        .map_err(|e| anyhow::anyhow!("Failed to open bus: {}", e))?;
    let delay = StdDelay;

    println!("Initializing VL53L5CX for Xtalk configuration...");
    let mut sensor = Vl53l5cx::new(i2c, delay);
    sensor.init()?;

    // --- Xtalk Margin Accessors ---
    let initial_margin = sensor.get_xtalk_margin()?;
    println!("Initial Xtalk Margin: {} kcps/spads", initial_margin);

    // Set a moderate margin (e.g., 75 kcps/spads)
    const NEW_MARGIN: u32 = 75; 
    sensor.set_xtalk_margin(NEW_MARGIN)?;
    println!("Set new Xtalk Margin to {} kcps/spads.", NEW_MARGIN);

    let final_margin = sensor.get_xtalk_margin()?;
    println!("Verified final Xtalk Margin: {} kcps/spads", final_margin);

    // Note: vl53l5cx_calibrate_xtalk is more complex and would need a safe wrapper 
    // to handle the buffer pointer, but the direct margin set/get works here.
    
    Ok(())
}