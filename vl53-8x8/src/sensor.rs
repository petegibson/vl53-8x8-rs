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
use crate::consts::{NB_TARGET_PER_ZONE, VL53L8CX_FW_CHECKSUM};
use crate::uld_data;
use vl53_8x8_types::Error;

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

/// Patch overlaying the shared (L8) `DEFAULT_CONFIGURATION` to produce the
/// VL53L5CX default configuration. Five bytes in the 972-byte blob. Offsets are
/// in the fully-resolved array (the `FW_NBTAR_RANGING` macro at index 107 is the
/// same value on both parts and is handled separately by
/// [`NB_TARGET_CONFIG_PATCH`]). Verified against ST ULD 2.0.1.
pub const L5_CONFIG_PATCH: &[(usize, u8)] = &[
    (0x01EE, 0x0C),
    (0x01EF, 0xCD),
    (0x01F6, 0x26),
    (0x02B5, 0x01),
    (0x02B9, 0x00),
];

/// Empty overlay: the shared default configuration is already the L8 one.
pub const L8_CONFIG_PATCH: &[(usize, u8)] = &[];

/// Patch overlaying the shared (L8) xtalk-calibration command table
/// (`uld_data::CALIBRATE_XTALK`, ST's `VL53L8CX_CALIBRATE_XTALK`) to produce
/// the VL53L5CX `VL53L5CX_CALIBRATE_XTALK` table. Generated and verified
/// byte-exact against ST ULD 2.0.1 `vl53l5cx_plugin_xtalk.h` by
/// `tools/extract_uld_blobs.py`.
///
/// This is the *only* L5/L8 difference in the xtalk plugin: the two
/// `calibrate_xtalk` C functions are identical apart from whitespace, and the
/// `GET_XTALK_CMD` tables match. The two bytes sit in the 0xB39C
/// (`VL53L8CX_DCI_INTERNAL_CP`) block of the command table, at the same
/// offsets and with the same values as the last two entries of
/// [`L5_CONFIG_PATCH`]: the VL53L5CX enables its internal charge pump there,
/// the VL53L8CX leaves it off (cf. [`SensorInfo::has_internal_cp`]).
pub const L5_CALIBRATE_XTALK_PATCH: &[(usize, u8)] = uld_data::L5_CALIBRATE_XTALK_PATCH;

/// Empty overlay: the shared xtalk-calibration command table is already the L8 one.
pub const L8_CALIBRATE_XTALK_PATCH: &[(usize, u8)] = &[];

/// Patch applied to the default configuration, after the variant patch, when
/// the crate is built for more than one target per zone (`targets-2/3/4`).
///
/// Byte 107 of the 972-byte `DEFAULT_CONFIGURATION` is the only preprocessor
/// macro in ST's table: element 107 of `VL53L8CX_DEFAULT_CONFIGURATION[]`
/// (`vl53l8cx_buffers.h` line 21546 onwards) is `VL53L8CX_FW_NBTAR_RANGING`,
/// which lines 24-28 of that header define as `2` when
/// `VL53L8CX_NB_TARGET_PER_ZONE == 1` and as `VL53L8CX_NB_TARGET_PER_ZONE`
/// otherwise. `vl53l5cx_buffers.h` (lines 24-28, array element 107) has the
/// identical macro at the identical index, so one patch serves both parts.
/// The generated table in `uld_data.rs` carries the single-target value `2`;
/// this overlay is empty for one target and `[(107, N)]` for `N > 1`.
pub const NB_TARGET_CONFIG_PATCH: &[(usize, u8)] = if NB_TARGET_PER_ZONE == 1 {
    &[]
} else {
    &[(FW_NBTAR_RANGING_INDEX, NB_TARGET_PER_ZONE)]
};

/// Index of the `VL53L8CX_FW_NBTAR_RANGING` macro inside `DEFAULT_CONFIGURATION`.
pub const FW_NBTAR_RANGING_INDEX: usize = 107;

/// A supported sensor variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Variant {
    /// VL53L5CX, and the VL53L7CX. The L7 is an L5 die behind a wider lens
    /// (60° instead of 45° square field of view): ST's VL53L7CX ULD carries
    /// byte-identical firmware, configuration and xtalk tables and checks
    /// for the same revision id (0x02), so the two cannot be told apart on
    /// the bus and an L7 is detected and driven as `L5`. Reported distances
    /// are plane-perpendicular on both (the lens geometry is handled inside
    /// the sensor); only an application that maps zones to angles needs to
    /// know which lens it has.
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
    /// Overlay applied to the shared default configuration for this variant.
    pub config_patch: &'static [(usize, u8)],
    /// Overlay applied to the shared xtalk-calibration command table
    /// (`VL53L8CX_CALIBRATE_XTALK`) by `calibrate_xtalk` (`xtalk` feature).
    pub calibrate_xtalk_patch: &'static [(usize, u8)],
    /// Whether init programs the glare filter (L5 only).
    pub has_glare_filter: bool,
    /// Whether `get_ranging_data` clamps negative distances to 0 (L5 only).
    pub clamp_negative_distance: bool,
    /// Whether `start_ranging` performs the laser-safety check (L8 only).
    pub has_laser_safety_check: bool,
    /// Whether the part supports deep-sleep power mode (L8 only).
    pub has_deep_sleep: bool,
    /// Whether the part has the external sync pin (L8 only).
    pub has_sync_pin: bool,
    /// Whether the part has internal charge-pump control (L5 only).
    pub has_internal_cp: bool,
    /// Whether the detection-thresholds plugin offers the auto-stop flag
    /// (`vl53l8cx_{get,set}_detection_thresholds_auto_stop`; L8 only, the
    /// VL53L5CX ULD has no such function).
    pub has_threshold_auto_stop: bool,
}

const L5_INFO: SensorInfo = SensorInfo {
    variant: Variant::L5,
    revision_id: 0x02,
    fw_checksum: None,
    fw_patch: L5_FW_PATCH,
    config_patch: L5_CONFIG_PATCH,
    calibrate_xtalk_patch: L5_CALIBRATE_XTALK_PATCH,
    has_glare_filter: true,
    clamp_negative_distance: true,
    has_laser_safety_check: false,
    has_deep_sleep: false,
    has_sync_pin: false,
    has_internal_cp: true,
    has_threshold_auto_stop: false,
};

const L8_INFO: SensorInfo = SensorInfo {
    variant: Variant::L8,
    revision_id: 0x0C,
    fw_checksum: Some(VL53L8CX_FW_CHECKSUM),
    fw_patch: L8_FW_PATCH,
    config_patch: L8_CONFIG_PATCH,
    calibrate_xtalk_patch: L8_CALIBRATE_XTALK_PATCH,
    has_glare_filter: false,
    clamp_negative_distance: false,
    has_laser_safety_check: true,
    has_deep_sleep: true,
    has_sync_pin: true,
    has_internal_cp: false,
    has_threshold_auto_stop: true,
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
        // L5-only behaviours.
        assert!(l5.has_glare_filter && l5.clamp_negative_distance && l5.has_internal_cp);
        assert!(!l8.has_glare_filter && !l8.clamp_negative_distance && !l8.has_internal_cp);
        // L8-only behaviours.
        assert!(l8.has_laser_safety_check && l8.has_deep_sleep && l8.has_sync_pin);
        assert!(!l5.has_laser_safety_check && !l5.has_deep_sleep && !l5.has_sync_pin);
        assert!(l8.has_threshold_auto_stop && !l5.has_threshold_auto_stop);
    }

    #[test]
    fn config_patch_is_l5_only_and_in_bounds() {
        assert!(Variant::L8.info().config_patch.is_empty());
        let p = Variant::L5.info().config_patch;
        assert_eq!(p.len(), 5);
        // Every patch offset must lie inside the 972-byte config blob.
        assert!(p.iter().all(|&(o, _)| o < 972));
        // The L5 patch never touches the target-count byte, so the two
        // overlays commute and `NB_TARGET_CONFIG_PATCH` can be applied last.
        assert!(p.iter().all(|&(o, _)| o != FW_NBTAR_RANGING_INDEX));
    }

    #[test]
    fn calibrate_xtalk_patch_is_l5_only_and_in_bounds() {
        assert!(Variant::L8.info().calibrate_xtalk_patch.is_empty());
        let p = Variant::L5.info().calibrate_xtalk_patch;
        assert_eq!(p.len(), 2);
        assert!(p.iter().all(|&(o, _)| o < uld_data::CALIBRATE_XTALK.len()));
        assert!(p.windows(2).all(|w| w[0].0 < w[1].0), "sorted ascending");
        // Never overlaps the target-count byte, so the two overlays commute.
        assert!(p.iter().all(|&(o, _)| o != uld_data::FW_NBTAR_XTALK_INDEX));
        // Same two bytes the L5 flips in the default configuration (0xB39C block).
        for entry in p {
            assert!(L5_CONFIG_PATCH.contains(entry), "{entry:?} not in L5_CONFIG_PATCH");
        }
    }

    #[test]
    fn nb_target_config_patch_tracks_target_count() {
        if NB_TARGET_PER_ZONE == 1 {
            assert!(NB_TARGET_CONFIG_PATCH.is_empty());
        } else {
            assert_eq!(
                NB_TARGET_CONFIG_PATCH,
                &[(FW_NBTAR_RANGING_INDEX, NB_TARGET_PER_ZONE)]
            );
        }
    }
}
