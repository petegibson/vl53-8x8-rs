use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::delay::DelayNs;

#[cfg(not(feature = "std"))]
extern crate alloc;
use crate::Vl53l8cx;

#[derive(Debug)]
pub struct PlatformError;

impl embedded_hal::i2c::Error for PlatformError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

// The C-compatible platform struct definition MUST match the C binding.
// This struct is used by the underlying C driver to hold context, but
// in this Rust wrapper, we largely bypass the `p_com` pointer in favor
// of a global mutex approach handled by the `impl_vl53l8cx_comms!` macro.
#[repr(C)]
pub struct VL53L8CX_Platform {
    pub address: u16,
    pub p_com: *mut core::ffi::c_void,
}
