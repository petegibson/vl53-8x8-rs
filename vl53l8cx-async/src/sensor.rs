//! Sensor-variant detection and per-variant metadata.
//!
//! The VL53L5CX and VL53L8CX are near-identical silicon. They are told apart at
//! runtime — before any firmware is uploaded — by the revision id register, and
//! their firmware images differ in only 7 bytes. This module provides:
//!
//! - [`detect`]: read device/revision ids and resolve the connected [`Variant`].
//! - [`SensorInfo`]: the small set of per-variant constants the init routine
//!   needs (revision id, firmware-checksum expectation, firmware overlay).
//!
//! Both variants share the [`crate::firmware::BASE_FIRMWARE`] image; the L5
//! image is reproduced from it with [`L5_FW_PATCH`].

use crate::bus::VlBusAsync;
use crate::firmware::{PatchedFirmware, BASE_FIRMWARE};
use embedded_hal::i2c::SevenBitAddress;
use vl53l8cx_sys::{Error, VL53L8CX_FW_CHECKSUM};

/// Device id register (reg 0) value common to both supported parts.
const DEVICE_ID: u8 = 0xF0;

/// Patch overlaying [`crate::firmware::BASE_FIRMWARE`] (the L8 image) to produce
/// the VL53L5CX firmware image. Seven bytes within a single 15-byte window.
///
/// Sorted ascending by offset (required by [`PatchedFirmware`]). Generated and
/// verified byte-exact against ST ULD 2.0.1 `vl53l5cx_buffers.h`.
pub const L5_FW_PATCH: &[(usize, u8)] = &[
    (0x0CB41, 0x50),
    (0x0CB42, 0x03),
    (0x0CB43, 0xE6),
    (0x0CB48, 0x00),
    (0x0CB49, 0x06),
    (0x0CB4E, 0xE4),
    (0x0CB4F, 0x00),
];

/// Empty overlay: [`crate::firmware::BASE_FIRMWARE`] is already the L8 image.
pub const L8_FW_PATCH: &[(usize, u8)] = &[];

/// A supported sensor variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Variant {
    /// VL53L5CX.
    L5,
    /// VL53L8CX.
    L8,
}

/// Per-variant constants and behaviour flags needed to bring a sensor up. The
/// two parts share the bulk of the init sequence; these capture the documented
/// differences between the ST ULD `vl53l5cx_init` and `vl53l8cx_init`.
#[derive(Debug, Clone, Copy)]
pub struct SensorInfo {
    /// Which variant this describes.
    pub variant: Variant,
    /// Expected silicon revision id (reg 1).
    pub revision_id: u8,
    /// Expected firmware CRC, read back after download. `Some` for parts that
    /// expose a checksum (L8); `None` for parts that verify the download by
    /// polling instead (L5).
    pub fw_checksum: Option<u32>,
    /// Overlay applied to [`crate::firmware::BASE_FIRMWARE`] for this variant.
    pub fw_patch: &'static [(usize, u8)],
    /// Whether init programs the glare filter (L5 only).
    pub has_glare_filter: bool,
    /// Whether `get_ranging_data` clamps negative distances to 0 (L5 only).
    pub clamp_negative_distance: bool,
}

const L5_INFO: SensorInfo = SensorInfo {
    variant: Variant::L5,
    revision_id: 0x02,
    fw_checksum: None,
    fw_patch: L5_FW_PATCH,
    has_glare_filter: true,
    clamp_negative_distance: true,
};

const L8_INFO: SensorInfo = SensorInfo {
    variant: Variant::L8,
    revision_id: 0x0C,
    fw_checksum: Some(VL53L8CX_FW_CHECKSUM),
    fw_patch: L8_FW_PATCH,
    has_glare_filter: false,
    clamp_negative_distance: false,
};

impl Variant {
    /// The per-variant constants for this sensor.
    pub const fn info(self) -> &'static SensorInfo {
        match self {
            Variant::L5 => &L5_INFO,
            Variant::L8 => &L8_INFO,
        }
    }

    /// A compiled-in, patched firmware source for this variant (base image plus
    /// the variant overlay), ready to hand to the init routine.
    pub fn firmware(self) -> PatchedFirmware<'static> {
        PatchedFirmware::new(BASE_FIRMWARE, self.info().fw_patch)
    }
}

/// Detect the connected sensor variant by reading the device and revision id
/// registers. Requires no firmware to be loaded.
///
/// Returns [`Error::NotAlive`] if nothing answers with the expected device id,
/// or [`Error::UnknownRevision`] if the device id matches but the revision is
/// not one of the known variants (e.g. a future silicon stepping).
pub async fn detect<BUS: VlBusAsync>(
    bus: &mut BUS,
    addr: SevenBitAddress,
) -> Result<Variant, Error<BUS::Error>> {
    bus.write_byte(addr, 0x7FFF, 0x00).await.map_err(Error::Bus)?;
    let device_id = bus.read_byte(addr, 0x00).await.map_err(Error::Bus)?;
    let revision_id = bus.read_byte(addr, 0x01).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x7FFF, 0x02).await.map_err(Error::Bus)?;

    if device_id != DEVICE_ID {
        return Err(Error::NotAlive);
    }
    match revision_id {
        r if r == L8_INFO.revision_id => Ok(Variant::L8),
        r if r == L5_INFO.revision_id => Ok(Variant::L5),
        other => Err(Error::UnknownRevision(other)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    /// Minimal executor: every future here resolves without ever yielding.
    fn block_on<F: Future>(fut: F) -> F::Output {
        fn raw() -> RawWaker {
            RawWaker::new(core::ptr::null(), &VTABLE)
        }
        static VTABLE: RawWakerVTable =
            RawWakerVTable::new(|_| raw(), |_| {}, |_| {}, |_| {});
        let waker = unsafe { Waker::from_raw(raw()) };
        let mut cx = Context::from_waker(&waker);
        let mut fut = pin!(fut);
        loop {
            if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
                return v;
            }
        }
    }

    /// Bus stub answering reg 0 with `device_id` and reg 1 with `revision_id`.
    struct MockBus {
        device_id: u8,
        revision_id: u8,
    }

    impl VlBusAsync for MockBus {
        type Error = core::convert::Infallible;

        async fn write(&mut self, _: SevenBitAddress, _: u16, _: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        async fn read(
            &mut self,
            _: SevenBitAddress,
            reg: u16,
            buf: &mut [u8],
        ) -> Result<(), Self::Error> {
            if let Some(b) = buf.first_mut() {
                *b = match reg {
                    0 => self.device_id,
                    1 => self.revision_id,
                    _ => 0,
                };
            }
            Ok(())
        }
    }

    fn detect_with(device_id: u8, revision_id: u8) -> Result<Variant, Error<core::convert::Infallible>> {
        let mut bus = MockBus { device_id, revision_id };
        block_on(detect(&mut bus, 0x29))
    }

    #[test]
    fn detects_l8() {
        assert_eq!(detect_with(0xF0, 0x0C), Ok(Variant::L8));
    }

    #[test]
    fn detects_l5() {
        assert_eq!(detect_with(0xF0, 0x02), Ok(Variant::L5));
    }

    #[test]
    fn unknown_revision_is_reported() {
        assert!(matches!(detect_with(0xF0, 0x99), Err(Error::UnknownRevision(0x99))));
    }

    #[test]
    fn no_device_is_not_alive() {
        assert!(matches!(detect_with(0x00, 0x0C), Err(Error::NotAlive)));
    }

    #[test]
    fn variant_firmware_carries_right_patch() {
        assert!(Variant::L8.info().fw_patch.is_empty());
        assert_eq!(Variant::L5.info().fw_patch.len(), 7);
        assert_eq!(Variant::L8.info().fw_checksum, Some(VL53L8CX_FW_CHECKSUM));
        assert_eq!(Variant::L5.info().fw_checksum, None);
    }

    #[test]
    fn variant_quirk_flags() {
        let l5 = Variant::L5.info();
        let l8 = Variant::L8.info();
        assert_eq!(l5.variant, Variant::L5);
        assert_eq!(l8.variant, Variant::L8);
        // Glare filter and negative-distance clamp are L5-only.
        assert!(l5.has_glare_filter && l5.clamp_negative_distance);
        assert!(!l8.has_glare_filter && !l8.clamp_negative_distance);
    }
}
