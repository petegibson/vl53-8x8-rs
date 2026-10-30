//! Driver scratch state.
//!
//! Replaces the ULD's `VL53L8CX_Configuration`, keeping only the fields the
//! I/O routines in [`crate::ops`] actually read or write. The C struct's
//! platform handle, bus address and pointers to the default tables are not
//! needed: the address lives on the driver handle and the tables are the
//! `const` arrays in `uld_data`.

use crate::consts::{VL53L8CX_OFFSET_BUFFER_SIZE, VL53L8CX_TEMPORARY_BUFFER_SIZE};
use vl53_8x8_types::XTALK_BUFFER_SIZE;

/// Per-sensor driver state. Not part of the public API; it is only nameable
/// through the private `state` module and all fields are crate-visible.
#[derive(Debug, Clone)]
pub struct State {
    /// Streamcount of the last result frame read; the sensor increments it
    /// per frame and `check_data_ready` uses it to detect a new one.
    pub(crate) streamcount: u8,
    /// Size in bytes of one result frame for the current output configuration,
    /// computed by `start_ranging`.
    pub(crate) data_read_size: u32,
    /// Set when the sensor has been told to stop by itself (autonomous
    /// single-shot); `stop_ranging` then skips the MCU-stop handshake.
    pub(crate) is_auto_stop_enabled: bool,
    /// Offset calibration read from the sensor NVM at init
    /// (`VL53L8CX_OFFSET_BUFFER_SIZE` bytes).
    pub(crate) offset_data: [u8; VL53L8CX_OFFSET_BUFFER_SIZE as usize],
    /// Module type the firmware reported at DCI 0xE0C5 after the last `init`
    /// (`None` before init, or when the L8 path skipped the probe).
    pub(crate) module_type: Option<u8>,
    /// Crosstalk shape currently applied: the ULD default or a user
    /// calibration (`VL53L8CX_XTALK_BUFFER_SIZE` bytes).
    pub(crate) xtalk_data: [u8; XTALK_BUFFER_SIZE],
    /// Scratch buffer for DCI transfers and result frames
    /// (`VL53L8CX_TEMPORARY_BUFFER_SIZE` bytes).
    pub(crate) temp_buffer: [u8; VL53L8CX_TEMPORARY_BUFFER_SIZE],
}

impl State {
    /// All-zero state, equivalent to the zero-initialised C struct.
    pub(crate) const fn new() -> Self {
        Self {
            streamcount: 0,
            data_read_size: 0,
            is_auto_stop_enabled: false,
            offset_data: [0; VL53L8CX_OFFSET_BUFFER_SIZE as usize],
            module_type: None,
            xtalk_data: [0; XTALK_BUFFER_SIZE],
            temp_buffer: [0; VL53L8CX_TEMPORARY_BUFFER_SIZE],
        }
    }
}

impl Default for State {
    fn default() -> Self {
        Self::new()
    }
}
