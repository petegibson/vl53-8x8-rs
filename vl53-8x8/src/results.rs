//! Ranging results.
//!
//! [`ResultsData`] is a field-for-field Rust rendering of the ULD's
//! `VL53L8CX_ResultsData` (`vl53l8cx_api.h`). It is `#[repr(C)]` and gated by
//! the same build options, so under a given feature set it has the same size
//! and field offsets as the C struct compiled into `vl53l8cx-uld-sys`; the oracle
//! tests check this for the default feature set.
//!
//! Feature mapping (ULD macro -> crate feature):
//!
//! | ULD `#define`                           | feature                            |
//! |-----------------------------------------|------------------------------------|
//! | `VL53L8CX_DISABLE_AMBIENT_PER_SPAD`     | `disable-ambient`                  |
//! | `VL53L8CX_DISABLE_NB_TARGET_DETECTED`   | `disable-nb-targets`               |
//! | `VL53L8CX_DISABLE_NB_SPADS_ENABLED`     | `disable-spads`                    |
//! | `VL53L8CX_DISABLE_SIGNAL_PER_SPAD`      | `disable-signal`                   |
//! | `VL53L8CX_DISABLE_RANGE_SIGMA_MM`       | `disable-range-sigma`              |
//! | `VL53L8CX_DISABLE_DISTANCE_MM`          | `disable-distance`                 |
//! | `VL53L8CX_DISABLE_REFLECTANCE_PERCENT`  | `disable-reflectance`              |
//! | `VL53L8CX_DISABLE_TARGET_STATUS`        | `disable-target-status`            |
//! | `VL53L8CX_DISABLE_MOTION_INDICATOR`     | not `motion`, or `disable-motion-indicator-output` |
//! | `VL53L8CX_NB_TARGET_PER_ZONE`           | `targets-2` / `targets-3` / `targets-4` (default 1) |

use crate::consts::NB_TARGET_PER_ZONE;
use vl53_8x8_types::VL53L8CX_RESOLUTION_8X8;

/// Zones in the full 8x8 grid; per-zone arrays have this many entries.
const ZONES: usize = VL53L8CX_RESOLUTION_8X8 as usize;
/// Entries in the per-target arrays: one per zone per target.
const TARGETS: usize = ZONES * NB_TARGET_PER_ZONE as usize;

/// One frame of ranging results, mirroring `VL53L8CX_ResultsData`.
///
/// Arrays are sized for the 8x8 grid; in 4x4 mode only the first 16 zones
/// (16 x `NB_TARGET_PER_ZONE` per-target entries) are populated. Fields are
/// present or absent according to the `disable-*` and `motion` features; see
/// the module docs for the mapping.
///
/// # Indexing
///
/// Per-zone arrays (`ambient_per_spad`, `nb_target_detected`,
/// `nb_spads_enabled`) have one entry per zone, indexed by zone number.
///
/// Per-target arrays (`signal_per_spad`, `range_sigma_mm`, `distance_mm`,
/// `reflectance`, `target_status`) hold `64 * NB_TARGET_PER_ZONE` entries,
/// laid out as in the ULD: entry `zone * NB_TARGET_PER_ZONE + target`, with
/// `target` in `0..nb_target_detected[zone]` valid and the rest unspecified
/// (`target_status` is forced to 255 for zones with no target). With the default
/// single target per zone this reduces to indexing by zone. The crate-root
/// constant [`crate::NB_TARGET_PER_ZONE`] gives the stride.
///
/// Unless the `raw-format` feature is enabled, values are converted to the
/// units given on each field, as the ULD does.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ResultsData {
    /// Internal sensor silicon temperature, degrees C.
    pub silicon_temp_degc: i8,

    /// Ambient noise in kcps/spad, per zone.
    #[cfg(not(feature = "disable-ambient"))]
    pub ambient_per_spad: [u32; ZONES],

    /// Number of valid targets detected, per zone.
    #[cfg(not(feature = "disable-nb-targets"))]
    pub nb_target_detected: [u8; ZONES],

    /// Number of SPADs enabled for this ranging, per zone.
    #[cfg(not(feature = "disable-spads"))]
    pub nb_spads_enabled: [u32; ZONES],

    /// Signal returned to the sensor in kcps/spad, per target.
    #[cfg(not(feature = "disable-signal"))]
    pub signal_per_spad: [u32; TARGETS],

    /// Sigma of the measured distance in mm, per target.
    #[cfg(not(feature = "disable-range-sigma"))]
    pub range_sigma_mm: [u16; TARGETS],

    /// Measured distance in mm, per target.
    #[cfg(not(feature = "disable-distance"))]
    pub distance_mm: [i16; TARGETS],

    /// Estimated reflectance in percent, per target.
    #[cfg(not(feature = "disable-reflectance"))]
    pub reflectance: [u8; TARGETS],

    /// Measurement validity status, per target (5 and 9 mean ranging OK;
    /// 255 is written for zones with no target when `nb_target_detected` is
    /// available).
    #[cfg(not(feature = "disable-target-status"))]
    pub target_status: [u8; TARGETS],

    /// Motion detector results (`motion` plugin).
    #[cfg(all(feature = "motion", not(feature = "disable-motion-indicator-output")))]
    pub motion_indicator: MotionIndicator,
}

/// Motion detector results block, mirroring the anonymous `motion_indicator`
/// struct inside `VL53L8CX_ResultsData`.
#[cfg(all(feature = "motion", not(feature = "disable-motion-indicator-output")))]
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotionIndicator {
    pub global_indicator_1: u32,
    pub global_indicator_2: u32,
    pub status: u8,
    pub nb_of_detected_aggregates: u8,
    pub nb_of_aggregates: u8,
    pub spare: u8,
    /// Motion indicator per aggregate (divided by 65535 unless `raw-format`).
    pub motion: [u32; 32],
}

impl Default for ResultsData {
    /// All-zero results, equivalent to the zero-initialised C struct.
    fn default() -> Self {
        // SAFETY: every field is an integer or an array/struct of integers, for
        // which the all-zero bit pattern is a valid value.
        unsafe { core::mem::zeroed() }
    }
}
