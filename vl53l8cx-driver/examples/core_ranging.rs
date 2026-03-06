use anyhow::Result;
use embedded_hal::delay::DelayNs;
// Import driver types
use vl53l5cx_driver::{Vl53l5cx, Resolution, TargetOrder};
// These are external utilities used in the original example setup
use i2cdev::linux::LinuxI2cDev;

/// A simple delay provider for `std` environments that uses `thread::sleep`.
struct StdDelay;

impl DelayNs for StdDelay {
    fn delay_ns(&mut self, ns: u32) {
        let duration = std::time::Duration::from_nanos(ns as u64);
        std::thread::sleep(duration);
    }
}

fn main() -> Result<()> {
    const I2C_BUS_PATH: &str = "/dev/i2c-2";

    // --- Setup Peripherals ---
    let i2c = LinuxI2cDev::new(I2C_BUS_PATH)
        .map_err(|e| anyhow::anyhow!("Failed to open bus: {}", e))?;
    let mut delay = StdDelay;

    println!("Initializing VL53L5CX...");
    let mut sensor = Vl53l5cx::new(i2c, delay);
    sensor.init()?;
    println!("Sensor Initialized and Alive.");

    // --- Configuration ---
    // Use type-safe enums
    sensor.set_resolution(Resolution::Res8x8)?;
    sensor.set_ranging_frequency_hz(15)?;
    sensor.set_target_order(TargetOrder::Closest)?;
    sensor.set_integration_time_ms(10)?;
    
    // --- Start Ranging ---
    sensor.start_ranging()?;
    println!("Ranging started (8x8 @ 15Hz).");

    for frame in 0..10 {
        while !sensor.check_data_ready()? {
            // Wait for new data (non-blocking poll)
            delay.delay_ms(1);
        }

        let results = sensor.get_ranging_data()?;
        
        // Print the center zone's data (Zone 27 for 8x8)
        let center_zone = 27;
        let distance = results.distance_mm[center_zone];
        let status = results.target_status[center_zone];
        
        println!(
            "Frame {}: Center Zone ({}) Dist: {}mm, Status: {}",
            frame, center_zone, distance, status
        );
    }

    sensor.stop_ranging()?;
    Ok(())
}