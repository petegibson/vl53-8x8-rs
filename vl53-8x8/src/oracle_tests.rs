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
