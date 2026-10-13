//! Protocol constants transcribed from the ST VL53L8CX Ultra Lite Driver.
//!
//! Reference: ULD `VL53L8CX_2.0.0` (`VL53L8CX_API_REVISION`), headers under
//! `vl53l8cx-uld-sys/src/VL53L8CX_ULD_API/inc/`:
//!
//! - `vl53l8cx_api.h`: UI command registers, buffer sizes, DCI indices,
//!   result block-header indices, `VL53L8CX_TEMPORARY_BUFFER_SIZE` formula.
//! - `vl53l8cx_plugin_xtalk.h`: `VL53L8CX_DCI_CAL_CFG`, `VL53L8CX_DCI_XTALK_CFG`.
//! - `vl53l8cx_plugin_detection_thresholds.h`: `VL53L8CX_DCI_DET_THRESH_*`.
//! - `src/vl53l8cx_api.c` (`vl53l8cx_init`): the firmware CRC literal.
//!
//! The ULD spells these as macros with explicit casts (`((uint16_t)0x5450U)`),
//! which bindgen does not emit, so they are written out by hand. Every value is
//! checked against the `vl53l8cx-uld-sys` crate by the oracle tests in
//! `oracle_tests.rs`. Names keep the C spelling so the two can be diffed
//! by eye; status codes, resolution and target-order values live in
//! `vl53-8x8-types`.

#![allow(dead_code)]

// --- Build configuration ---------------------------------------------------

/// Number of targets reported per zone (`VL53L8CX_NB_TARGET_PER_ZONE`).
/// Selected by the `targets-N` features; 1 when none is set.
pub(crate) const NB_TARGET_PER_ZONE: u8 = {
    #[cfg(feature = "targets-4")]
    {
        4
    }
    #[cfg(all(feature = "targets-3", not(feature = "targets-4")))]
    {
        3
    }
    #[cfg(all(
        feature = "targets-2",
        not(feature = "targets-3"),
        not(feature = "targets-4")
    ))]
    {
        2
    }
    #[cfg(not(any(feature = "targets-2", feature = "targets-3", feature = "targets-4")))]
    {
        1
    }
};

// --- UI command registers ----------------------------------------------------

pub(crate) const VL53L8CX_UI_CMD_STATUS: u16 = 0x2C00;
pub(crate) const VL53L8CX_UI_CMD_START: u16 = 0x2C04;
pub(crate) const VL53L8CX_UI_CMD_END: u16 = 0x2FFF;

// --- Buffer sizes -------------------------------------------------------------

pub(crate) const VL53L8CX_NVM_DATA_SIZE: u16 = 492;
pub(crate) const VL53L8CX_CONFIGURATION_SIZE: u16 = 972;
pub(crate) const VL53L8CX_OFFSET_BUFFER_SIZE: u16 = 488;
// VL53L8CX_XTALK_BUFFER_SIZE (776) is `vl53_8x8_types::XTALK_BUFFER_SIZE`.

// --- Firmware -----------------------------------------------------------------

/// CRC the VL53L8CX firmware reports after a successful download
/// (`vl53l8cx_api.c`, `vl53l8cx_init`: `0xc0b6c9e`).
pub(crate) const VL53L8CX_FW_CHECKSUM: u32 = 0x0C0B6C9E;

// --- DCI indices ----------------------------------------------------------------

pub(crate) const VL53L8CX_DCI_ZONE_CONFIG: u16 = 0x5450;
pub(crate) const VL53L8CX_DCI_FREQ_HZ: u16 = 0x5458;
pub(crate) const VL53L8CX_DCI_INT_TIME: u16 = 0x545C;
pub(crate) const VL53L8CX_DCI_FW_NB_TARGET: u16 = 0x5478;
pub(crate) const VL53L8CX_DCI_RANGING_MODE: u16 = 0xAD30;
pub(crate) const VL53L8CX_DCI_DSS_CONFIG: u16 = 0xAD38;
pub(crate) const VL53L8CX_DCI_VHV_CONFIG: u16 = 0xAD60;
pub(crate) const VL53L8CX_DCI_TARGET_ORDER: u16 = 0xAE64;
pub(crate) const VL53L8CX_DCI_SHARPENER: u16 = 0xAED8;
pub(crate) const VL53L8CX_DCI_INTERNAL_CP: u16 = 0xB39C;
pub(crate) const VL53L8CX_DCI_SYNC_PIN: u16 = 0xB5F0;
pub(crate) const VL53L8CX_DCI_MOTION_DETECTOR_CFG: u16 = 0xBFAC;
pub(crate) const VL53L8CX_DCI_SINGLE_RANGE: u16 = 0xD964;
pub(crate) const VL53L8CX_DCI_OUTPUT_CONFIG: u16 = 0xD968;
pub(crate) const VL53L8CX_DCI_OUTPUT_ENABLES: u16 = 0xD970;
pub(crate) const VL53L8CX_DCI_OUTPUT_LIST: u16 = 0xD980;
pub(crate) const VL53L8CX_DCI_PIPE_CONTROL: u16 = 0xDB80;

// Plugin: xtalk (`vl53l8cx_plugin_xtalk.h`).
pub(crate) const VL53L8CX_DCI_CAL_CFG: u16 = 0x5470;
pub(crate) const VL53L8CX_DCI_XTALK_CFG: u16 = 0xAD94;

// Plugin: detection thresholds (`vl53l8cx_plugin_detection_thresholds.h`).
pub(crate) const VL53L8CX_DCI_DET_THRESH_CONFIG: u16 = 0x5488;
pub(crate) const VL53L8CX_DCI_DET_THRESH_GLOBAL_CONFIG: u16 = 0xB6E0;
pub(crate) const VL53L8CX_DCI_DET_THRESH_START: u16 = 0xB6E8;
pub(crate) const VL53L8CX_DCI_DET_THRESH_VALID_STATUS: u16 = 0xB9F0;

// --- Result block-header indices (`VL53L8CX_NB_TARGET_PER_ZONE == 1` set) -------
//
// The ULD has a second set of values for multi-target builds; `start_ranging`
// in `ops.rs` likewise programs the single-target block-header list.

pub(crate) const VL53L8CX_METADATA_IDX: u16 = 0x54B4;
pub(crate) const VL53L8CX_SPAD_COUNT_IDX: u16 = 0x55D0;
pub(crate) const VL53L8CX_AMBIENT_RATE_IDX: u16 = 0x54D0;
pub(crate) const VL53L8CX_NB_TARGET_DETECTED_IDX: u16 = 0xDB84;
pub(crate) const VL53L8CX_SIGNAL_RATE_IDX: u16 = 0xDBC4;
pub(crate) const VL53L8CX_RANGE_SIGMA_MM_IDX: u16 = 0xDEC4;
pub(crate) const VL53L8CX_DISTANCE_IDX: u16 = 0xDF44;
pub(crate) const VL53L8CX_REFLECTANCE_EST_PC_IDX: u16 = 0xE044;
pub(crate) const VL53L8CX_TARGET_STATUS_IDX: u16 = 0xE084;
pub(crate) const VL53L8CX_MOTION_DETEC_IDX: u16 = 0xD858;

// --- Temporary buffer size ------------------------------------------------------
//
// Mirrors the `L5CX_*_SIZE` / `VL53L8CX_MAX_RESULTS_SIZE` /
// `VL53L8CX_TEMPORARY_BUFFER_SIZE` macro chain in `vl53l8cx_api.h`. Each term
// is the on-wire size of one result block (4-byte header included) and drops
// to 0 when the matching `disable-*` feature removes that output.

const fn block(enabled: bool, size: usize) -> usize {
    if enabled {
        size
    } else {
        0
    }
}

const N: usize = NB_TARGET_PER_ZONE as usize;

const L5CX_AMB_SIZE: usize = block(!cfg!(feature = "disable-ambient"), 260);
const L5CX_SPAD_SIZE: usize = block(!cfg!(feature = "disable-spads"), 260);
const L5CX_NTAR_SIZE: usize = block(!cfg!(feature = "disable-nb-targets"), 68);
const L5CX_SPS_SIZE: usize = block(!cfg!(feature = "disable-signal"), 256 * N + 4);
const L5CX_SIGR_SIZE: usize = block(!cfg!(feature = "disable-range-sigma"), 128 * N + 4);
const L5CX_DIST_SIZE: usize = block(!cfg!(feature = "disable-distance"), 128 * N + 4);
const L5CX_RFLEST_SIZE: usize = block(!cfg!(feature = "disable-reflectance"), 64 * N + 4);
const L5CX_STA_SIZE: usize = block(!cfg!(feature = "disable-target-status"), 64 * N + 4);
const L5CX_MOT_SIZE: usize = block(MOTION_INDICATOR_OUTPUT, 144);

/// Whether the motion-indicator result block is compiled in: the ULD's
/// `VL53L8CX_DISABLE_MOTION_INDICATOR` is undefined only when the `motion`
/// plugin is enabled and its output has not been disabled.
pub(crate) const MOTION_INDICATOR_OUTPUT: bool =
    cfg!(feature = "motion") && !cfg!(feature = "disable-motion-indicator-output");

/// `VL53L8CX_MAX_RESULTS_SIZE`: 40 bytes of header + metadata + common data,
/// the enabled blocks, and a 20-byte footer.
pub(crate) const VL53L8CX_MAX_RESULTS_SIZE: usize = 40
    + L5CX_AMB_SIZE
    + L5CX_SPAD_SIZE
    + L5CX_NTAR_SIZE
    + L5CX_SPS_SIZE
    + L5CX_SIGR_SIZE
    + L5CX_DIST_SIZE
    + L5CX_RFLEST_SIZE
    + L5CX_STA_SIZE
    + L5CX_MOT_SIZE
    + 20;

/// `VL53L8CX_TEMPORARY_BUFFER_SIZE`: the driver scratch buffer, never smaller
/// than 1024 bytes.
pub(crate) const VL53L8CX_TEMPORARY_BUFFER_SIZE: usize = if VL53L8CX_MAX_RESULTS_SIZE < 1024 {
    1024
} else {
    VL53L8CX_MAX_RESULTS_SIZE
};
