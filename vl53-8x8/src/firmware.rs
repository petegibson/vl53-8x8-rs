//! Firmware image sourcing.
//!
//! The sensor needs an 86,016-byte firmware image uploaded over the bus at
//! init. This module decouples *where those bytes come from* from the upload
//! routine in [`crate::ops`], via the [`FirmwareSource`] trait.
//!
//! Today two compiled-in ("app flash") sources are provided:
//! - [`SliceFirmware`] — a plain `&[u8]` (e.g. `include_bytes!`).
//! - [`PatchedFirmware`] — a shared base image plus a tiny per-variant overlay,
//!   so a single 86 KB image in flash can serve both the VL53L5CX and VL53L8CX
//!   (their firmware differs in only 7 bytes — see [`crate::sensor`]).
//!
//! An external-flash (QSPI/XIP) source reading through `embedded-storage` is a
//! planned addition; it will slot in as another `FirmwareSource` impl without
//! touching the upload routine.

/// The shared base firmware image, compiled into application flash.
///
/// This is the **VL53L8CX** image; the VL53L5CX image is reproduced from it by
/// applying [`crate::sensor::L5_FW_PATCH`]. Verified byte-exact against the ST
/// ULD 2.0.1 `vl53l8cx_buffers.h` / `vl53l5cx_buffers.h` blobs.
pub static BASE_FIRMWARE: &[u8] = include_bytes!("vl53l8cx_fw_base.bin");

/// Expected length of a complete firmware image, in bytes.
pub const FIRMWARE_LEN: usize = 86_016;

/// Scratch window used by [`PatchedFirmware`] to materialise the (small,
/// clustered) patched region without buffering the whole image. Must be large
/// enough to span the widest patch cluster; the L5 overlay spans 15 bytes.
const PATCH_SCRATCH: usize = 64;

/// A source of firmware-image bytes, pulled by the upload routine in bank-sized
/// runs. Implementors serve the bytes from wherever they live (compiled-in
/// slice, patched base, or — later — external flash).
///
/// Reads are infallible for the compiled-in sources here. A fallible variant
/// will be introduced alongside the external-flash source.
pub trait FirmwareSource {
    /// Total length of the image, in bytes (must be [`FIRMWARE_LEN`]).
    fn len(&self) -> usize;

    /// Returns true if the image is empty (always false in practice; present to
    /// satisfy the `len`-without-`is_empty` lint).
    fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Borrow a contiguous run of up to `max` bytes starting at `offset`.
    ///
    /// The returned slice may be shorter than `max` (the caller loops until the
    /// bank is consumed). Slice-backed sources return a zero-copy view; sources
    /// that must transform bytes return a view into internal scratch. The slice
    /// is always non-empty for `offset < len()`.
    fn chunk(&mut self, offset: usize, max: usize) -> &[u8];
}

/// Firmware served verbatim from a byte slice (`include_bytes!`, or any
/// `&'static [u8]`). Zero-copy.
pub struct SliceFirmware<'a>(pub &'a [u8]);

impl FirmwareSource for SliceFirmware<'_> {
    fn len(&self) -> usize {
        self.0.len()
    }

    fn chunk(&mut self, offset: usize, max: usize) -> &[u8] {
        let end = (offset + max).min(self.0.len());
        &self.0[offset..end]
    }
}

/// Firmware reconstructed from a shared `base` image plus a sorted `patch` list
/// of `(offset, byte)` overrides. Clean spans are returned zero-copy; only the
/// run containing patched bytes is copied into a small scratch buffer and
/// overlaid, so the full image is never duplicated in RAM.
///
/// `patch` **must be sorted ascending by offset** and every offset must be
/// within `base`.
pub struct PatchedFirmware<'a> {
    base: &'a [u8],
    patch: &'a [(usize, u8)],
    scratch: [u8; PATCH_SCRATCH],
}

impl<'a> PatchedFirmware<'a> {
    pub fn new(base: &'a [u8], patch: &'a [(usize, u8)]) -> Self {
        Self {
            base,
            patch,
            scratch: [0; PATCH_SCRATCH],
        }
    }
}

impl FirmwareSource for PatchedFirmware<'_> {
    fn len(&self) -> usize {
        self.base.len()
    }

    fn chunk(&mut self, offset: usize, max: usize) -> &[u8] {
        let base = self.base;
        let patch = self.patch;
        let end = (offset + max).min(base.len());

        // First patched byte that falls within this requested run, if any.
        match patch.iter().copied().find(|&(o, _)| o >= offset && o < end) {
            // No patch in range: hand back the clean span zero-copy.
            None => &base[offset..end],
            // Patch starts later: hand back the clean prefix up to it, zero-copy.
            // The next call will start exactly at the patch offset.
            Some((first, _)) if first > offset => &base[offset..first],
            // offset sits on a patch: emit a scratch window covering the cluster.
            Some(_) => {
                let win = (offset + PATCH_SCRATCH).min(end);
                let n = win - offset;
                self.scratch[..n].copy_from_slice(&base[offset..win]);
                for (o, v) in patch.iter().copied() {
                    if o >= offset && o < win {
                        self.scratch[o - offset] = v;
                    }
                }
                &self.scratch[..n]
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensor::L5_FW_PATCH;

    /// Drain a source by pulling chunks no larger than `max`, mimicking the
    /// upload loop, and return the reconstructed image.
    fn collect(src: &mut impl FirmwareSource, max: usize) -> std::vec::Vec<u8> {
        let mut out = std::vec::Vec::new();
        let mut off = 0;
        while off < src.len() {
            let take = (src.len() - off).min(max);
            let chunk = src.chunk(off, take);
            assert!(!chunk.is_empty(), "chunk must make progress at {off}");
            assert!(chunk.len() <= take);
            out.extend_from_slice(chunk);
            off += chunk.len();
        }
        out
    }

    #[test]
    fn base_image_matches_uld_l8_firmware() {
        // Oracle: the C ULD firmware array compiled into vl53l8cx-uld-sys.
        let oracle = unsafe { &vl53l8cx_uld_sys::VL53L8CX_FIRMWARE };
        assert_eq!(BASE_FIRMWARE.len(), FIRMWARE_LEN);
        assert_eq!(BASE_FIRMWARE, &oracle[..]);
    }

    #[test]
    fn slice_firmware_is_verbatim_at_any_chunking() {
        for max in [FIRMWARE_LEN, 0x8000, 4096, 100, 1] {
            let mut src = SliceFirmware(BASE_FIRMWARE);
            assert_eq!(collect(&mut src, max), BASE_FIRMWARE, "max={max}");
        }
    }

    #[test]
    fn patched_firmware_applies_only_the_overlay() {
        // Reconstruction must be independent of chunk size, including chunkings
        // that split the patch window across calls.
        for max in [FIRMWARE_LEN, 0x8000, 4096, 100, 16, 1] {
            let mut src = PatchedFirmware::new(BASE_FIRMWARE, L5_FW_PATCH);
            let out = collect(&mut src, max);
            assert_eq!(out.len(), FIRMWARE_LEN, "max={max}");
            for (i, (&got, &base)) in out.iter().zip(BASE_FIRMWARE).enumerate() {
                match L5_FW_PATCH.iter().find(|&&(o, _)| o == i) {
                    Some(&(_, val)) => assert_eq!(got, val, "patched byte {i:#x} max={max}"),
                    None => assert_eq!(got, base, "unpatched byte {i:#x} max={max}"),
                }
            }
        }
    }

    #[test]
    fn empty_patch_equals_base() {
        let mut src = PatchedFirmware::new(BASE_FIRMWARE, &[]);
        assert_eq!(collect(&mut src, 4096), BASE_FIRMWARE);
    }
}
