#![no_std]
/*!
Shared types for the STMicroelectronics VL53L5CX/VL53L7CX/VL53L8CX 8x8
multizone Time-of-Flight sensor drivers.

This crate holds the hand-written Rust types and constants that are common to
the pure-Rust driver and the ST ULD wrapper: the error type, the configuration
enums, the plugin (xtalk / motion / detection-threshold) types and the ULD
status codes. It has no dependencies and no build script.
*/

// --- Status Codes ---
// The ULD defines these as macros with explicit casts (e.g. ((uint8_t)0U)),
// which bindgen skips, so they are defined by hand here.

pub const VL53L8CX_STATUS_OK: u8 = 0;
pub const VL53L8CX_STATUS_TIMEOUT_ERROR: u8 = 1;
pub const VL53L8CX_STATUS_CORRUPTED_FRAME: u8 = 2;
pub const VL53L8CX_STATUS_LASER_SAFETY: u8 = 3;
pub const VL53L8CX_STATUS_XTALK_FAILED: u8 = 4;
pub const VL53L8CX_STATUS_FW_CHECKSUM_FAIL: u8 = 5;
pub const VL53L8CX_MCU_ERROR: u8 = 66;
pub const VL53L8CX_STATUS_INVALID_PARAM: u8 = 127;
pub const VL53L8CX_STATUS_ERROR: u8 = 255;

// Resolution
pub const VL53L8CX_RESOLUTION_4X4: u8 = 16;
pub const VL53L8CX_RESOLUTION_8X8: u8 = 64;

// Target Order
pub const VL53L8CX_TARGET_ORDER_CLOSEST: u8 = 1;
pub const VL53L8CX_TARGET_ORDER_STRONGEST: u8 = 2;

// Ranging Mode
pub const VL53L8CX_RANGING_MODE_CONTINUOUS: u8 = 1;
pub const VL53L8CX_RANGING_MODE_AUTONOMOUS: u8 = 3;

// Power Mode
pub const VL53L8CX_POWER_MODE_SLEEP: u8 = 0;
pub const VL53L8CX_POWER_MODE_WAKEUP: u8 = 1;
pub const VL53L8CX_POWER_MODE_DEEP_SLEEP: u8 = 2;

/// Default I2C address as the ULD spells it (8-bit, write address).
pub const VL53L8CX_DEFAULT_I2C_ADDRESS: u16 = 0x52;

// Thresholds & Detection (Plugin Constants)
#[cfg(feature = "thresholds")]
pub mod thresholds {
    pub const VL53L8CX_NB_THRESHOLDS: u8 = 64;
    pub const VL53L8CX_LAST_THRESHOLD: u8 = 128;

    // Detection Threshold Types
    pub const VL53L8CX_IN_WINDOW: u8 = 0;
    pub const VL53L8CX_OUT_OF_WINDOW: u8 = 1;
    pub const VL53L8CX_LESS_THAN_EQUAL_MIN_CHECKER: u8 = 2;
    pub const VL53L8CX_GREATER_THAN_MAX_CHECKER: u8 = 3;
    pub const VL53L8CX_EQUAL_MIN_CHECKER: u8 = 4;
    pub const VL53L8CX_NOT_EQUAL_MIN_CHECKER: u8 = 5;

    // Measurements
    pub const VL53L8CX_DISTANCE_MM: u8 = 1;
    pub const VL53L8CX_SIGNAL_PER_SPAD_KCPS: u8 = 2;
    pub const VL53L8CX_RANGE_SIGMA_MM: u8 = 4;
    pub const VL53L8CX_AMBIENT_PER_SPAD_KCPS: u8 = 8;
    pub const VL53L8CX_NB_TARGET_DETECTED: u8 = 9;
    pub const VL53L8CX_NB_SPADS_ENABLED: u8 = 10;
    pub const VL53L8CX_MOTION_INDICATOR: u8 = 11;
    pub const VL53L8CX_TARGET_STATUS: u8 = 12;

    // Operations
    pub const VL53L8CX_OPERATION_NONE: u8 = 0;
    pub const VL53L8CX_OPERATION_OR: u8 = 0;
    pub const VL53L8CX_OPERATION_AND: u8 = 2;
}

// Re-export threshold constants to top level if feature is enabled
#[cfg(feature = "thresholds")]
pub use thresholds::*;

// --- Shared Types ---
// These types are used by both the blocking driver and the async driver.

/// The default I2C address of the sensor (7-bit).
pub const DEFAULT_ADDRESS: u8 = 0x29;

/// The required buffer size for crosstalk calibration data (776 bytes).
pub const XTALK_BUFFER_SIZE: usize = 776;

/// Command to retrieve xtalk calibration data from the sensor.
/// Mirrors the C `static const uint8_t VL53L8CX_GET_XTALK_CMD[72]`.
#[cfg(feature = "xtalk")]
pub const VL53L8CX_GET_XTALK_CMD: [u8; 72] = [
    0x54, 0x00, 0x00, 0x40, 0x9F, 0xD8, 0x00, 0xC0,
    0x9F, 0xE4, 0x01, 0x40, 0x9F, 0xF8, 0x00, 0x40,
    0x9F, 0xFC, 0x04, 0x04, 0xA0, 0xFC, 0x01, 0x00,
    0xA1, 0x0C, 0x01, 0x00, 0xA1, 0x1C, 0x00, 0xC0,
    0xA1, 0x28, 0x09, 0x02, 0xA2, 0x48, 0x00, 0x40,
    0xA2, 0x4C, 0x00, 0x81, 0xA2, 0x54, 0x00, 0x81,
    0xA2, 0x5C, 0x00, 0x81, 0xA2, 0x64, 0x00, 0x81,
    0xA2, 0x6C, 0x00, 0x84, 0xA2, 0x8C, 0x00, 0x82,
    0x00, 0x00, 0x00, 0x0F, 0x07, 0x02, 0x00, 0x44,
];

/// A placeholder type for an unconfigured hardware pin.
pub struct NoPin;

/// Zone grid resolution. Default after `init()` is [`Resolution::Res4x4`].
/// Set it before the ranging frequency, whose allowed range depends on it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Resolution {
    /// 16 zones; ranging frequency 1..=60 Hz. The default.
    Res4x4 = VL53L8CX_RESOLUTION_4X4,
    /// 64 zones; ranging frequency 1..=15 Hz; four times the bus traffic per
    /// frame.
    Res8x8 = VL53L8CX_RESOLUTION_8X8,
}

/// Order in which detected targets within a zone are reported. Default after
/// `init()` is [`TargetOrder::Strongest`]. With one target per zone this picks
/// the target reported; two targets in one zone are only told apart when at
/// least 600 mm apart.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TargetOrder {
    /// Nearest target first.
    Closest = VL53L8CX_TARGET_ORDER_CLOSEST,
    /// Target with the strongest return signal first. The default.
    Strongest = VL53L8CX_TARGET_ORDER_STRONGEST,
}

/// Ranging mode. Default after `init()` is [`RangingMode::Autonomous`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RangingMode {
    /// VCSEL on throughout the frame: best range and ambient immunity, for
    /// fast or high-performance ranging; the integration time is ignored.
    Continuous = VL53L8CX_RANGING_MODE_CONTINUOUS,
    /// VCSEL on only for the integration time of each frame: lower power, the
    /// more so at low ranging frequency. The default.
    Autonomous = VL53L8CX_RANGING_MODE_AUTONOMOUS,
}

/// Sensor power mode. Default after `init()` is [`PowerMode::Wakeup`]. Change
/// it only while not ranging.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PowerMode {
    /// LP idle: retains firmware and configuration but cannot range until
    /// woken with [`PowerMode::Wakeup`].
    Sleep = VL53L8CX_POWER_MODE_SLEEP,
    /// HP idle: ready to range. The default.
    Wakeup = VL53L8CX_POWER_MODE_WAKEUP,
    /// VL53L8CX only, lower than sleep: waking re-uploads the firmware and
    /// returns the configuration to the `init()` defaults. Rejected with
    /// `FirmwareError::InvalidParam` on a VL53L5CX.
    DeepSleep = VL53L8CX_POWER_MODE_DEEP_SLEEP,
}

/// Measurement a detection threshold is evaluated against. Thresholds are
/// given in the natural units listed here; the API converts them to the
/// firmware's fixed-point format.
#[cfg(feature = "thresholds")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ThresholdMeasurement {
    /// Target distance, mm.
    DistanceMm = VL53L8CX_DISTANCE_MM,
    /// Return signal rate, kcps/SPAD.
    SignalPerSpadKcps = VL53L8CX_SIGNAL_PER_SPAD_KCPS,
    /// Standard deviation of the distance, mm.
    RangeSigmaMm = VL53L8CX_RANGE_SIGMA_MM,
    /// Ambient light rate, kcps/SPAD.
    AmbientPerSpadKcps = VL53L8CX_AMBIENT_PER_SPAD_KCPS,
    /// Number of targets detected in the zone.
    NbTargetDetected = VL53L8CX_NB_TARGET_DETECTED,
    /// Target status code (5 and 9 are valid ranges).
    TargetStatus = VL53L8CX_TARGET_STATUS,
    /// Number of SPADs enabled in the zone.
    NbSpadsEnabled = VL53L8CX_NB_SPADS_ENABLED,
    /// Motion indicator of the zone's aggregate (`motion` plugin).
    MotionIndicator = VL53L8CX_MOTION_INDICATOR,
}

/// Comparison applied by a detection threshold, between the measured value
/// and the entry's `low_threshold` / `high_threshold`.
#[cfg(feature = "thresholds")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ThresholdType {
    /// Trigger when `low <= value <= high`.
    InWindow = VL53L8CX_IN_WINDOW,
    /// Trigger when the value is outside `low..=high`.
    OutOfWindow = VL53L8CX_OUT_OF_WINDOW,
    /// Trigger when `value <= low`.
    LessThanEqualMin = VL53L8CX_LESS_THAN_EQUAL_MIN_CHECKER,
    /// Trigger when `value > high`.
    GreaterThanMax = VL53L8CX_GREATER_THAN_MAX_CHECKER,
    /// Trigger when `value == low`.
    EqualMin = VL53L8CX_EQUAL_MIN_CHECKER,
    /// Trigger when `value != low`.
    NotEqualMin = VL53L8CX_NOT_EQUAL_MIN_CHECKER,
}

/// Logical operation combining a threshold with the next one in the list.
/// Only used by the 4x4 two-thresholds-per-zone configuration; ST's header
/// notes that the first threshold of a pair is always OR, and defines OR as
/// 0, the same value as [`ThresholdOperation::None`].
#[cfg(feature = "thresholds")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ThresholdOperation {
    /// No combination (also the ULD's `OR` value, 0).
    None = VL53L8CX_OPERATION_NONE,
    // Or: ULD defines VL53L8CX_OPERATION_OR as 0, same as NONE.
    // Awaiting clarification from ST on whether OR should be a distinct value.
    // Or = VL53L8CX_OPERATION_OR,
    /// Both this and the next threshold must be met.
    And = VL53L8CX_OPERATION_AND,
}

/// One detection threshold entry.
/// Layout matches the C struct `VL53L8CX_DetectionThresholds`.
///
/// `set_detection_thresholds` takes 64 of these. Three configurations are
/// supported: 4x4 with one threshold per zone (16 entries used), 4x4 with
/// two per zone (32 entries; `mathematic_operation` combines each pair), or
/// 8x8 with one per zone (64 entries). ST's header says to OR
/// [`VL53L8CX_LAST_THRESHOLD`] (128) into the `zone_num` of the last entry
/// used and to leave the rest zero.
#[cfg(feature = "thresholds")]
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct DetectionThreshold {
    /// Lower bound, in the natural units of `measurement` (mm, kcps/SPAD,
    /// count, ...); the API converts to the firmware's fixed-point format.
    pub low_threshold: i32,
    /// Upper bound, same units as `low_threshold`.
    pub high_threshold: i32,
    /// Which result value is compared.
    pub measurement: ThresholdMeasurement,
    /// How the value is compared with the bounds.
    pub threshold_type: ThresholdType,
    /// Zone this entry applies to (0..16 in 4x4, 0..64 in 8x8); OR in
    /// [`VL53L8CX_LAST_THRESHOLD`] on the last entry used.
    pub zone_num: u8,
    /// Combination with the next entry; only applies to the 4x4
    /// two-per-zone configuration.
    pub mathematic_operation: ThresholdOperation,
}

/// Pure Rust representation of the motion indicator configuration.
/// Layout matches the C struct `VL53L8CX_Motion_Configuration`.
#[cfg(any(feature = "motion", feature = "thresholds"))]
#[derive(Clone, Copy)]
#[repr(C)]
pub struct MotionConfiguration {
    pub ref_bin_offset: i32,
    pub detection_threshold: u32,
    pub extra_noise_sigma: u32,
    pub null_den_clip_value: u32,
    pub mem_update_mode: u8,
    pub mem_update_choice: u8,
    pub sum_span: u8,
    pub feature_length: u8,
    pub nb_of_aggregates: u8,
    pub nb_of_temporal_accumulations: u8,
    pub min_nb_for_global_detection: u8,
    pub global_indicator_format_1: u8,
    pub global_indicator_format_2: u8,
    pub spare_1: u8,
    pub spare_2: u8,
    pub spare_3: u8,
    pub map_id: [i8; 64],
    pub indicator_format_1: [u8; 32],
    pub indicator_format_2: [u8; 32],
}

/// Error reported by the sensor firmware, decoded from a ULD status code.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum FirmwareError {
    Timeout,
    CorruptedFrame,
    LaserSafety,
    XtalkFailed,
    FwChecksumFail,
    McuError,
    InvalidParam,
    Unknown(u8),
}

impl From<u8> for FirmwareError {
    fn from(code: u8) -> Self {
        match code {
            VL53L8CX_STATUS_TIMEOUT_ERROR => FirmwareError::Timeout,
            VL53L8CX_STATUS_CORRUPTED_FRAME => FirmwareError::CorruptedFrame,
            VL53L8CX_STATUS_LASER_SAFETY => FirmwareError::LaserSafety,
            VL53L8CX_STATUS_XTALK_FAILED => FirmwareError::XtalkFailed,
            VL53L8CX_STATUS_FW_CHECKSUM_FAIL => FirmwareError::FwChecksumFail,
            VL53L8CX_MCU_ERROR => FirmwareError::McuError,
            VL53L8CX_STATUS_INVALID_PARAM => FirmwareError::InvalidParam,
            c => FirmwareError::Unknown(c),
        }
    }
}

/// Represents errors that can occur in the driver.
#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    Bus(E),
    NotAlive,
    /// A VL53L sensor responded (device id 0xF0) but its silicon revision id
    /// is not one this driver recognises. The raw revision byte is included.
    UnknownRevision(u8),
    Firmware(FirmwareError),
    Pin,
}

impl<E: core::error::Error + 'static> core::error::Error for Error<E> {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Error::Bus(e) => Some(e),
            _ => None,
        }
    }
}

impl<E: core::fmt::Debug> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Bus(e) => write!(f, "Bus error: {:?}", e),
            Error::NotAlive => write!(f, "Sensor not found at address"),
            Error::UnknownRevision(r) => write!(f, "Unknown sensor revision id: {:#04x}", r),
            Error::Firmware(e) => write!(f, "Firmware error: {:?}", e),
            Error::Pin => write!(f, "Pin error"),
        }
    }
}
