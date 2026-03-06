use anyhow::Result;
use embedded_hal::delay::DelayNs;
// Import from driver crate directly, not sys
use vl53l5cx_driver::{Vl53l5cx, DetectionThreshold, ThresholdMeasurement, ThresholdType, ThresholdOperation};
use i2cdev::linux::LinuxI2cDev;

struct StdDelay;
impl DelayNs for StdDelay {
    fn delay_ns(&mut self, ns: u32) {
        let duration = std::time::Duration::from_nanos(ns as u64);
        std::thread::sleep(duration);
    }
}

fn main() -> Result<()> {
    // Requires initialization and the "thresholds" feature enabled.
    const I2C_BUS_PATH: &str = "/dev/i2c-2";
    const TARGET_ZONE: u8 = 0; // Top-left zone for 8x8 or 4x4
    const MIN_DISTANCE_MM: i32 = 100;

    // ... (Setup Peripherals boilerplate)
    let i2c = LinuxI2cDev::new(I2C_BUS_PATH)
        .map_err(|e| anyhow::anyhow!("Failed to open bus: {}", e))?;
    let delay = StdDelay;

    println!("Initializing VL53L5CX for Thresholds...");
    let mut sensor = Vl53l5cx::new(i2c, delay);
    sensor.init()?;

    // --- Threshold Configuration ---
    // 1. Create the default array (64 elements)
    // Note: In the C API, 128 marks the end of the list. 
    // In our Rust wrapper, we just create the array.
    let mut thresholds = [DetectionThreshold {
        low_threshold: 0,
        high_threshold: 0,
        measurement: ThresholdMeasurement::DistanceMm,
        threshold_type: ThresholdType::LessThanEqualMin,
        zone_num: 128, // Equivalent to VL53L5CX_LAST_THRESHOLD
        mathematic_operation: ThresholdOperation::None,
    }; 64];

    // 2. Define the first actual threshold (Threshold 0)
    // Goal: Trigger if Distance on TARGET_ZONE is LESS THAN OR EQUAL TO MIN_DISTANCE_MM (100mm)
    thresholds[0] = DetectionThreshold {
        low_threshold: MIN_DISTANCE_MM, // Value to check against
        high_threshold: 0, 
        measurement: ThresholdMeasurement::DistanceMm,
        threshold_type: ThresholdType::LessThanEqualMin,
        zone_num: TARGET_ZONE,
        mathematic_operation: ThresholdOperation::Or, // First threshold must be OR
    };

    // 3. Mark the end of the threshold list immediately after the active one
    // We use 128 (C logic) to mark the end.
    thresholds[1].zone_num = 128; 
    
    // 4. Apply the configuration
    sensor.set_detection_thresholds(thresholds)?;
    sensor.set_detection_thresholds_enable(true)?;

    println!(
        "Thresholds enabled: Trigger when Zone {} distance is <= {}mm.",
        TARGET_ZONE, MIN_DISTANCE_MM
    );
    
    let is_enabled = sensor.get_detection_thresholds_enable()?;
    println!("Verification: Thresholds are enabled: {}", is_enabled);

    Ok(())
}