use anyhow::Result;
use embedded_hal::delay::DelayNs;
use vl53l5cx_driver::{Vl53l5cx, Resolution, MotionConfiguration};
use i2cdev::linux::LinuxI2cDev;

struct StdDelay;
impl DelayNs for StdDelay {
    fn delay_ns(&mut self, ns: u32) {
        let duration = std::time::Duration::from_nanos(ns as u64);
        std::thread::sleep(duration);
    }
}

fn main() -> Result<()> {
    // Requires initialization and the "motion" feature enabled.
    const I2C_BUS_PATH: &str = "/dev/i2c-2";
    
    // The Motion_Configuration struct comes from the re-exported MotionConfiguration
    let mut motion_config: MotionConfiguration = unsafe { core::mem::zeroed() };
    
    // Define the desired detection range (e.g., watch for motion between 500mm and 2000mm)
    const MIN_RANGE_MM: u16 = 500;
    const MAX_RANGE_MM: u16 = 2000;

    // ... (Setup Peripherals boilerplate)
    let i2c = LinuxI2cDev::new(I2C_BUS_PATH)
        .map_err(|e| anyhow::anyhow!("Failed to open bus: {}", e))?;
    let delay = StdDelay;

    println!("Initializing VL53L5CX for Motion Indicator...");
    let mut sensor = Vl53l5cx::new(i2c, delay);
    sensor.init()?;

    // --- Motion Configuration ---
    // 1. Initialize the motion engine and set the map resolution (using 8x8 here)
    sensor.motion_indicator_init(&mut motion_config, Resolution::Res8x8)?;

    // 2. Configure the physical distance range for motion detection
    sensor.motion_indicator_set_distance_motion(&mut motion_config, MIN_RANGE_MM, MAX_RANGE_MM)?;
    
    // 3. (Optional) Set the sensor's current resolution to ensure the motion map is correctly configured
    sensor.motion_indicator_set_resolution(&mut motion_config, Resolution::Res8x8)?;

    println!(
        "Motion Indicator configured to detect movement between {}mm and {}mm.",
        MIN_RANGE_MM, MAX_RANGE_MM
    );
    println!("Ready to start ranging and poll the results.motion_indicator field for data.");

    Ok(())
}