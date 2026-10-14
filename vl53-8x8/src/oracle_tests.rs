//! Oracle tests: check the hand-transcribed constants, generated data tables
//! and `#[repr(C)]` result/state layouts against the ST ULD as compiled by
//! `vl53l8cx-uld-sys` (a dev-dependency only; the driver itself never links it).
//!
//! The dev-dependency is built with its default features, so the tests that
//! depend on feature-sensitive layout (`ResultsData`, `temp_buffer`) only run
//! when this crate is also built with its default feature set. Constants and
//! data tables are compared unconditionally.

use crate::consts;
use crate::uld_data;
use vl53l8cx_uld_sys as sys;

// --- Data tables -------------------------------------------------------------

#[test]
fn default_configuration_matches_uld() {
    let oracle = unsafe { &sys::VL53L8CX_DEFAULT_CONFIGURATION };
    assert_eq!(uld_data::DEFAULT_CONFIGURATION.len(), oracle.len());
    assert_eq!(&uld_data::DEFAULT_CONFIGURATION[..], &oracle[..]);
    assert_eq!(
        uld_data::DEFAULT_CONFIGURATION.len(),
        consts::VL53L8CX_CONFIGURATION_SIZE as usize
    );
}

#[test]
fn default_xtalk_matches_uld() {
    let oracle = unsafe { &sys::VL53L8CX_DEFAULT_XTALK };
    assert_eq!(uld_data::DEFAULT_XTALK.len(), oracle.len());
    assert_eq!(&uld_data::DEFAULT_XTALK[..], &oracle[..]);
    assert_eq!(uld_data::DEFAULT_XTALK.len(), vl53_8x8_types::XTALK_BUFFER_SIZE);
}

#[test]
fn get_nvm_cmd_matches_uld() {
    let oracle = unsafe { &sys::VL53L8CX_GET_NVM_CMD };
    assert_eq!(uld_data::GET_NVM_CMD.len(), oracle.len());
    assert_eq!(&uld_data::GET_NVM_CMD[..], &oracle[..]);
}

// --- Constants -----------------------------------------------------------------

macro_rules! assert_consts_match {
    ($($name:ident),+ $(,)?) => {
        $( assert_eq!(consts::$name, sys::$name, stringify!($name)); )+
    };
}

#[test]
fn transcribed_constants_match_sys() {
    assert_consts_match!(
        VL53L8CX_UI_CMD_STATUS,
        VL53L8CX_UI_CMD_START,
        VL53L8CX_UI_CMD_END,
        VL53L8CX_NVM_DATA_SIZE,
        VL53L8CX_CONFIGURATION_SIZE,
        VL53L8CX_OFFSET_BUFFER_SIZE,
        VL53L8CX_FW_CHECKSUM,
        VL53L8CX_DCI_CAL_CFG,
        VL53L8CX_DCI_ZONE_CONFIG,
        VL53L8CX_DCI_FREQ_HZ,
        VL53L8CX_DCI_INT_TIME,
        VL53L8CX_DCI_FW_NB_TARGET,
        VL53L8CX_DCI_DET_THRESH_CONFIG,
        VL53L8CX_DCI_RANGING_MODE,
        VL53L8CX_DCI_DSS_CONFIG,
        VL53L8CX_DCI_VHV_CONFIG,
        VL53L8CX_DCI_XTALK_CFG,
        VL53L8CX_DCI_TARGET_ORDER,
        VL53L8CX_DCI_SHARPENER,
        VL53L8CX_DCI_INTERNAL_CP,
        VL53L8CX_DCI_SYNC_PIN,
        VL53L8CX_DCI_DET_THRESH_GLOBAL_CONFIG,
        VL53L8CX_DCI_DET_THRESH_START,
        VL53L8CX_DCI_DET_THRESH_VALID_STATUS,
        VL53L8CX_DCI_MOTION_DETECTOR_CFG,
        VL53L8CX_DCI_SINGLE_RANGE,
        VL53L8CX_DCI_OUTPUT_CONFIG,
        VL53L8CX_DCI_OUTPUT_ENABLES,
        VL53L8CX_DCI_OUTPUT_LIST,
        VL53L8CX_DCI_PIPE_CONTROL,
    );
}

// --- Result block headers ---------------------------------------------------------
//
// bindgen does not emit the cast macros `((uint16_t)0x54B4U)` / `((uint32_t)...U)`,
// and the oracle crates are single-target builds anyway, so both sets are pinned
// to literals copied from `vl53l8cx_api.h` (lines cited per set). The VL53L5CX
// header (`vl53l5cx_api.h` lines 94-144) carries the same two sets.

/// `vl53l8cx_api.h` lines 103-125: the `VL53L8CX_NB_TARGET_PER_ZONE == 1` set.
#[cfg(not(any(feature = "targets-2", feature = "targets-3", feature = "targets-4")))]
#[test]
fn block_headers_match_single_target_header_set() {
    assert_eq!(consts::NB_TARGET_PER_ZONE, 1);
    assert_eq!(
        consts::OUTPUT_BLOCK_HEADERS,
        [
            0x0000000D, 0x54B400C0, 0x54C00040, 0x54D00104, 0x55D00404, 0xDB840401,
            0xDBC40404, 0xDEC40402, 0xDF440402, 0xE0440401, 0xE0840401, 0xD85808C0,
        ]
    );
    assert_eq!(consts::VL53L8CX_METADATA_IDX, 0x54B4);
    assert_eq!(consts::VL53L8CX_SPAD_COUNT_IDX, 0x55D0);
    assert_eq!(consts::VL53L8CX_AMBIENT_RATE_IDX, 0x54D0);
    assert_eq!(consts::VL53L8CX_NB_TARGET_DETECTED_IDX, 0xDB84);
    assert_eq!(consts::VL53L8CX_SIGNAL_RATE_IDX, 0xDBC4);
    assert_eq!(consts::VL53L8CX_RANGE_SIGMA_MM_IDX, 0xDEC4);
    assert_eq!(consts::VL53L8CX_DISTANCE_IDX, 0xDF44);
    assert_eq!(consts::VL53L8CX_REFLECTANCE_EST_PC_IDX, 0xE044);
    assert_eq!(consts::VL53L8CX_TARGET_STATUS_IDX, 0xE084);
    assert_eq!(consts::VL53L8CX_MOTION_DETEC_IDX, 0xD858);
}

/// `vl53l8cx_api.h` lines 128-150: the `#else` (multi-target) set.
#[cfg(any(feature = "targets-2", feature = "targets-3", feature = "targets-4"))]
#[test]
fn block_headers_match_multi_target_header_set() {
    assert!(consts::NB_TARGET_PER_ZONE > 1);
    assert_eq!(
        consts::OUTPUT_BLOCK_HEADERS,
        [
            0x0000000D, 0x54B400C0, 0x54C00040, 0x54D00104, 0x55D00404, 0x57D00401,
            0x58900404, 0x64900402, 0x66900402, 0x6A900401, 0x6B900401, 0xCC5008C0,
        ]
    );
    assert_eq!(consts::VL53L8CX_METADATA_IDX, 0x54B4);
    assert_eq!(consts::VL53L8CX_SPAD_COUNT_IDX, 0x55D0);
    assert_eq!(consts::VL53L8CX_AMBIENT_RATE_IDX, 0x54D0);
    assert_eq!(consts::VL53L8CX_NB_TARGET_DETECTED_IDX, 0x57D0);
    assert_eq!(consts::VL53L8CX_SIGNAL_RATE_IDX, 0x5890);
    assert_eq!(consts::VL53L8CX_RANGE_SIGMA_MM_IDX, 0x6490);
    assert_eq!(consts::VL53L8CX_DISTANCE_IDX, 0x6690);
    assert_eq!(consts::VL53L8CX_REFLECTANCE_EST_PC_IDX, 0x6A90);
    assert_eq!(consts::VL53L8CX_TARGET_STATUS_IDX, 0x6B90);
    assert_eq!(consts::VL53L8CX_MOTION_DETEC_IDX, 0xCC50);
}

/// Each output-list word carries its block index in the upper half, so the
/// list `start_ranging` programs and the indices `get_ranging_data` matches on
/// describe the same blocks. The ULD only guarantees this by eye.
#[test]
fn output_list_indices_match_result_parser_indices() {
    let o = &consts::OUTPUT_BLOCK_HEADERS;
    assert_eq!((o[1] >> 16) as u16, consts::VL53L8CX_METADATA_IDX);
    assert_eq!((o[3] >> 16) as u16, consts::VL53L8CX_AMBIENT_RATE_IDX);
    assert_eq!((o[4] >> 16) as u16, consts::VL53L8CX_SPAD_COUNT_IDX);
    assert_eq!((o[5] >> 16) as u16, consts::VL53L8CX_NB_TARGET_DETECTED_IDX);
    assert_eq!((o[6] >> 16) as u16, consts::VL53L8CX_SIGNAL_RATE_IDX);
    assert_eq!((o[7] >> 16) as u16, consts::VL53L8CX_RANGE_SIGMA_MM_IDX);
    assert_eq!((o[8] >> 16) as u16, consts::VL53L8CX_DISTANCE_IDX);
    assert_eq!((o[9] >> 16) as u16, consts::VL53L8CX_REFLECTANCE_EST_PC_IDX);
    assert_eq!((o[10] >> 16) as u16, consts::VL53L8CX_TARGET_STATUS_IDX);
    assert_eq!((o[11] >> 16) as u16, consts::VL53L8CX_MOTION_DETEC_IDX);
    // `start_ranging` sizes blocks with idx in [0x54D0, 0x54D0 + 960) per zone
    // and the rest per target; each block must fall on the right side. The
    // single-target set keeps `nb_target_detected` outside that window, which
    // is harmless there (per target == per zone) but would be wrong for N > 1.
    let per_zone = |idx: u16| (0x54D0..0x54D0 + 960).contains(&idx);
    assert!(per_zone(consts::VL53L8CX_AMBIENT_RATE_IDX));
    assert!(per_zone(consts::VL53L8CX_SPAD_COUNT_IDX));
    if consts::NB_TARGET_PER_ZONE > 1 {
        assert!(per_zone(consts::VL53L8CX_NB_TARGET_DETECTED_IDX));
    }
    assert!(!per_zone(consts::VL53L8CX_SIGNAL_RATE_IDX));
    assert!(!per_zone(consts::VL53L8CX_RANGE_SIGMA_MM_IDX));
    assert!(!per_zone(consts::VL53L8CX_DISTANCE_IDX));
    assert!(!per_zone(consts::VL53L8CX_REFLECTANCE_EST_PC_IDX));
    assert!(!per_zone(consts::VL53L8CX_TARGET_STATUS_IDX));
}

// --- Target count ------------------------------------------------------------------

/// Byte 107 of the default configuration is `VL53L8CX_FW_NBTAR_RANGING`
/// (`vl53l8cx_buffers.h` lines 24-28: 2 for one target per zone, else the target
/// count). The generated table and the C oracle both hold the single-target
/// value; `NB_TARGET_CONFIG_PATCH` must turn it into N for multi-target builds
/// and leave it alone otherwise.
#[test]
fn nb_target_config_patch_sets_byte_107() {
    use crate::sensor::{FW_NBTAR_RANGING_INDEX, L5_CONFIG_PATCH, NB_TARGET_CONFIG_PATCH};
    let oracle = unsafe { &sys::VL53L8CX_DEFAULT_CONFIGURATION };
    assert_eq!(FW_NBTAR_RANGING_INDEX, 107);
    assert_eq!(oracle[107], 2, "oracle built for one target per zone");
    assert_eq!(uld_data::DEFAULT_CONFIGURATION[107], 2);
    // The xtalk plugin's twin macro has the same definition and is emitted by
    // bindgen as a plain integer; it pins the "2 for one target" rule.
    assert_eq!(sys::VL53L8CX_FW_NBTAR_XTALK, 2);

    let n = consts::NB_TARGET_PER_ZONE;
    let mut cfg = uld_data::DEFAULT_CONFIGURATION;
    for &(o, v) in L5_CONFIG_PATCH.iter().chain(NB_TARGET_CONFIG_PATCH) {
        cfg[o] = v;
    }
    if n == 1 {
        assert!(NB_TARGET_CONFIG_PATCH.is_empty());
        assert_eq!(cfg[107], 2);
    } else {
        assert_eq!(NB_TARGET_CONFIG_PATCH, &[(107usize, n)]);
        assert_eq!(cfg[107], n);
        // Relative to ST's (single-target) L5 table only byte 107 can change,
        // and for `targets-2` it does not: the single-target value is also 2.
        let l5 = unsafe { &vl53l5cx_uld_sys::VL53L5CX_DEFAULT_CONFIGURATION };
        let diffs: std::vec::Vec<usize> = (0..972).filter(|&i| cfg[i] != l5[i]).collect();
        let expected: std::vec::Vec<usize> = if n == 2 { std::vec![] } else { std::vec![107] };
        assert_eq!(diffs, expected);
    }
}

/// `ResultsData` per-target arrays and the frame-size chain follow the target
/// count (`vl53l8cx_api.h` lines 207-233 and 244-247).
#[test]
fn result_sizes_follow_target_count() {
    use core::mem::size_of_val;
    let n = consts::NB_TARGET_PER_ZONE as usize;
    assert_eq!(crate::NB_TARGET_PER_ZONE, n);
    let r = crate::results::ResultsData::default();
    #[cfg(not(feature = "disable-distance"))]
    assert_eq!(size_of_val(&r.distance_mm), 64 * n * 2);
    #[cfg(not(feature = "disable-target-status"))]
    assert_eq!(size_of_val(&r.target_status), 64 * n);
    #[cfg(not(feature = "disable-signal"))]
    assert_eq!(size_of_val(&r.signal_per_spad), 64 * n * 4);
    #[cfg(not(feature = "disable-nb-targets"))]
    assert_eq!(size_of_val(&r.nb_target_detected), 64);
    let _ = (&r, n);

    #[cfg(not(any(
        feature = "motion",
        feature = "disable-ambient",
        feature = "disable-spads",
        feature = "disable-nb-targets",
        feature = "disable-signal",
        feature = "disable-range-sigma",
        feature = "disable-distance",
        feature = "disable-target-status",
        feature = "disable-reflectance",
    )))]
    assert_eq!(
        consts::VL53L8CX_MAX_RESULTS_SIZE,
        40 + 260 + 260 + 68 + (256 * n + 4) + (128 * n + 4) + (128 * n + 4) + (64 * n + 4) + (64 * n + 4) + 20
    );
}

#[test]
fn temporary_buffer_size_is_at_least_1024() {
    assert!(consts::VL53L8CX_TEMPORARY_BUFFER_SIZE >= 1024);
    assert_eq!(
        consts::VL53L8CX_TEMPORARY_BUFFER_SIZE,
        core::cmp::max(1024, consts::VL53L8CX_MAX_RESULTS_SIZE)
    );
}

// --- Layout (default feature set only) ------------------------------------------

#[cfg(not(any(
    feature = "motion",
    feature = "disable-ambient",
    feature = "disable-spads",
    feature = "disable-nb-targets",
    feature = "disable-signal",
    feature = "disable-range-sigma",
    feature = "disable-distance",
    feature = "disable-target-status",
    feature = "disable-reflectance",
    feature = "disable-motion-indicator-output",
    feature = "targets-2",
    feature = "targets-3",
    feature = "targets-4",
)))]
mod layout {
    use super::sys;
    use crate::consts;
    use crate::results::ResultsData;
    use crate::state::State;
    use core::mem::{offset_of, size_of, size_of_val};

    macro_rules! assert_fields_match {
        ($rust:ty, $c:ty, $($field:ident),+ $(,)?) => {
            $(
                assert_eq!(
                    offset_of!($rust, $field),
                    offset_of!($c, $field),
                    concat!("offset of ", stringify!($field))
                );
            )+
        };
    }

    #[test]
    fn results_data_layout_matches_bindgen() {
        assert_eq!(size_of::<ResultsData>(), size_of::<sys::VL53L8CX_ResultsData>());
        assert_eq!(
            core::mem::align_of::<ResultsData>(),
            core::mem::align_of::<sys::VL53L8CX_ResultsData>()
        );
        assert_fields_match!(
            ResultsData,
            sys::VL53L8CX_ResultsData,
            silicon_temp_degc,
            ambient_per_spad,
            nb_target_detected,
            nb_spads_enabled,
            signal_per_spad,
            range_sigma_mm,
            distance_mm,
            reflectance,
            target_status,
        );

        // Array lengths, via the size of each field on a value.
        let r = ResultsData::default();
        let c: sys::VL53L8CX_ResultsData = unsafe { core::mem::zeroed() };
        assert_eq!(size_of_val(&r.ambient_per_spad), size_of_val(&c.ambient_per_spad));
        assert_eq!(size_of_val(&r.nb_target_detected), size_of_val(&c.nb_target_detected));
        assert_eq!(size_of_val(&r.nb_spads_enabled), size_of_val(&c.nb_spads_enabled));
        assert_eq!(size_of_val(&r.signal_per_spad), size_of_val(&c.signal_per_spad));
        assert_eq!(size_of_val(&r.range_sigma_mm), size_of_val(&c.range_sigma_mm));
        assert_eq!(size_of_val(&r.distance_mm), size_of_val(&c.distance_mm));
        assert_eq!(size_of_val(&r.reflectance), size_of_val(&c.reflectance));
        assert_eq!(size_of_val(&r.target_status), size_of_val(&c.target_status));
    }

    #[test]
    fn state_buffers_match_uld_configuration() {
        let s = State::new();
        let c = sys::VL53L8CX_Configuration::default();
        assert_eq!(size_of_val(&s.temp_buffer), size_of_val(&c.temp_buffer));
        assert_eq!(consts::VL53L8CX_TEMPORARY_BUFFER_SIZE, size_of_val(&c.temp_buffer));
        assert_eq!(size_of_val(&s.offset_data), size_of_val(&c.offset_data));
        assert_eq!(size_of_val(&s.xtalk_data), size_of_val(&c.xtalk_data));
    }
}

// --- VL53L5CX oracle --------------------------------------------------------------
//
// `vl53-8x8` derives the VL53L5CX firmware image and default configuration from
// the VL53L8CX ones by applying small byte patches. `vl53l5cx-uld-sys` (an
// unpublished, path-only dev-dependency) compiles ST's actual VL53L5CX ULD, so
// these tests pin the patches to ST's arrays byte for byte and confirm that the
// register map and the remaining data tables really are shared between the parts.

mod l5 {
    use crate::consts;
    use crate::firmware::{FirmwareSource, PatchedFirmware, BASE_FIRMWARE, FIRMWARE_LEN};
    use crate::sensor::{L5_CONFIG_PATCH, L5_FW_PATCH};
    use crate::uld_data;
    use vl53l5cx_uld_sys as l5;
    use vl53l8cx_uld_sys as l8;

    /// Drain a firmware source the way the upload loop does.
    fn collect(src: &mut impl FirmwareSource, max: usize) -> std::vec::Vec<u8> {
        let mut out = std::vec::Vec::with_capacity(src.len());
        let mut off = 0;
        while off < src.len() {
            let chunk = src.chunk(off, (src.len() - off).min(max));
            assert!(!chunk.is_empty());
            out.extend_from_slice(chunk);
            off += chunk.len();
        }
        out
    }

    /// Offsets at which two equal-length byte slices differ.
    fn diff_offsets(a: &[u8], b: &[u8]) -> std::vec::Vec<usize> {
        assert_eq!(a.len(), b.len());
        (0..a.len()).filter(|&i| a[i] != b[i]).collect()
    }

    #[test]
    fn patched_firmware_matches_l5_uld() {
        let oracle = unsafe { &l5::VL53L5CX_FIRMWARE };
        assert_eq!(oracle.len(), FIRMWARE_LEN);
        for max in [FIRMWARE_LEN, 0x8000, 4096, 1] {
            let img = collect(&mut PatchedFirmware::new(BASE_FIRMWARE, L5_FW_PATCH), max);
            assert_eq!(img.len(), oracle.len());
            assert!(
                img[..] == oracle[..],
                "patched L5 firmware differs from ST's (chunk {max})"
            );
        }
    }

    #[test]
    fn l5_fw_patch_is_exactly_the_l5_l8_difference() {
        let l5fw = unsafe { &l5::VL53L5CX_FIRMWARE };
        let l8fw = unsafe { &l8::VL53L8CX_FIRMWARE };
        let diffs = diff_offsets(&l8fw[..], &l5fw[..]);
        let patched: std::vec::Vec<usize> = L5_FW_PATCH.iter().map(|&(o, _)| o).collect();
        assert_eq!(diffs, patched, "L5_FW_PATCH must cover exactly the bytes that differ");
        for &(o, v) in L5_FW_PATCH {
            assert_eq!(v, l5fw[o], "patch value at {o:#07X}");
            assert_ne!(v, l8fw[o], "patch at {o:#07X} is a no-op");
        }
    }

    #[test]
    fn patched_configuration_matches_l5_uld() {
        let oracle = unsafe { &l5::VL53L5CX_DEFAULT_CONFIGURATION };
        let mut cfg = uld_data::DEFAULT_CONFIGURATION;
        for &(o, v) in L5_CONFIG_PATCH {
            cfg[o] = v;
        }
        assert_eq!(cfg.len(), oracle.len());
        assert!(
            cfg[..] == oracle[..],
            "patched L5 default configuration differs from ST's"
        );
    }

    #[test]
    fn l5_config_patch_is_exactly_the_l5_l8_difference() {
        let l5cfg = unsafe { &l5::VL53L5CX_DEFAULT_CONFIGURATION };
        let diffs = diff_offsets(&uld_data::DEFAULT_CONFIGURATION[..], &l5cfg[..]);
        let patched: std::vec::Vec<usize> = L5_CONFIG_PATCH.iter().map(|&(o, _)| o).collect();
        assert_eq!(diffs, patched, "L5_CONFIG_PATCH must cover exactly the bytes that differ");
    }

    #[test]
    fn xtalk_and_nvm_tables_are_shared_between_variants() {
        let xtalk = unsafe { &l5::VL53L5CX_DEFAULT_XTALK };
        let nvm = unsafe { &l5::VL53L5CX_GET_NVM_CMD };
        assert!(uld_data::DEFAULT_XTALK[..] == xtalk[..], "L5 default xtalk differs");
        assert!(uld_data::GET_NVM_CMD[..] == nvm[..], "L5 NVM command differs");
    }

    macro_rules! assert_l5_consts_match {
        ($($l8:ident => $l5:ident),+ $(,)?) => {
            $( assert_eq!(consts::$l8, l5::$l5, stringify!($l5)); )+
        };
    }

    /// The driver uses one register map for both parts; confirm it against the L5 header.
    #[test]
    fn shared_register_map_matches_l5_header() {
        assert_l5_consts_match!(
            VL53L8CX_UI_CMD_STATUS => VL53L5CX_UI_CMD_STATUS,
            VL53L8CX_UI_CMD_START => VL53L5CX_UI_CMD_START,
            VL53L8CX_UI_CMD_END => VL53L5CX_UI_CMD_END,
            VL53L8CX_NVM_DATA_SIZE => VL53L5CX_NVM_DATA_SIZE,
            VL53L8CX_CONFIGURATION_SIZE => VL53L5CX_CONFIGURATION_SIZE,
            VL53L8CX_OFFSET_BUFFER_SIZE => VL53L5CX_OFFSET_BUFFER_SIZE,
            VL53L8CX_DCI_ZONE_CONFIG => VL53L5CX_DCI_ZONE_CONFIG,
            VL53L8CX_DCI_FREQ_HZ => VL53L5CX_DCI_FREQ_HZ,
            VL53L8CX_DCI_INT_TIME => VL53L5CX_DCI_INT_TIME,
            VL53L8CX_DCI_FW_NB_TARGET => VL53L5CX_DCI_FW_NB_TARGET,
            VL53L8CX_DCI_RANGING_MODE => VL53L5CX_DCI_RANGING_MODE,
            VL53L8CX_DCI_DSS_CONFIG => VL53L5CX_DCI_DSS_CONFIG,
            VL53L8CX_DCI_VHV_CONFIG => VL53L5CX_DCI_VHV_CONFIG,
            VL53L8CX_DCI_TARGET_ORDER => VL53L5CX_DCI_TARGET_ORDER,
            VL53L8CX_DCI_SHARPENER => VL53L5CX_DCI_SHARPENER,
            VL53L8CX_DCI_INTERNAL_CP => VL53L5CX_DCI_INTERNAL_CP,
            VL53L8CX_DCI_SYNC_PIN => VL53L5CX_DCI_SYNC_PIN,
            VL53L8CX_DCI_MOTION_DETECTOR_CFG => VL53L5CX_DCI_MOTION_DETECTOR_CFG,
            VL53L8CX_DCI_SINGLE_RANGE => VL53L5CX_DCI_SINGLE_RANGE,
            VL53L8CX_DCI_OUTPUT_CONFIG => VL53L5CX_DCI_OUTPUT_CONFIG,
            VL53L8CX_DCI_OUTPUT_ENABLES => VL53L5CX_DCI_OUTPUT_ENABLES,
            VL53L8CX_DCI_OUTPUT_LIST => VL53L5CX_DCI_OUTPUT_LIST,
            VL53L8CX_DCI_PIPE_CONTROL => VL53L5CX_DCI_PIPE_CONTROL,
        );
        assert_eq!(
            vl53_8x8_types::XTALK_BUFFER_SIZE,
            l5::VL53L5CX_XTALK_BUFFER_SIZE as usize
        );
    }
}
