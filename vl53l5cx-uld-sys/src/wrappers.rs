//! Thin safe wrappers over the raw `vl53l5cx_*` bindings. Each returns the ULD
//! status byte (`VL53L5CX_STATUS_OK` is 0) plus any out-parameter as a tuple.
//! Mirrors `vl53l8cx_uld_sys::wrappers`; the L8-only functions (external sync
//! pin, detection-threshold auto-stop) are absent and the L5-only internal
//! charge pump pair is added.

use crate::{VL53L5CX_Configuration, VL53L5CX_ResultsData};

#[cfg(any(feature = "motion", feature = "thresholds"))]
use crate::VL53L5CX_Motion_Configuration;

/// A type alias for the configuration struct for brevity.
type Cfg = VL53L5CX_Configuration;

pub fn init(config: &mut Cfg) -> u8 {
    unsafe { crate::vl53l5cx_init(config) }
}

pub fn is_alive(config: &mut Cfg) -> (u8, u8) {
    let mut is_alive: u8 = 0;
    let status = unsafe { crate::vl53l5cx_is_alive(config, &mut is_alive) };
    (status, is_alive)
}

pub fn set_i2c_address(config: &mut Cfg, address: u16) -> u8 {
    unsafe { crate::vl53l5cx_set_i2c_address(config, address) }
}

pub fn set_resolution(config: &mut Cfg, resolution: u8) -> u8 {
    unsafe { crate::vl53l5cx_set_resolution(config, resolution) }
}

pub fn set_ranging_frequency_hz(config: &mut Cfg, frequency: u8) -> u8 {
    unsafe { crate::vl53l5cx_set_ranging_frequency_hz(config, frequency) }
}

pub fn get_sharpener_percent(config: &mut Cfg) -> (u8, u8) {
    let mut sharpener_percent: u8 = 0;
    let status = unsafe { crate::vl53l5cx_get_sharpener_percent(config, &mut sharpener_percent) };
    (status, sharpener_percent)
}

pub fn set_sharpener_percent(config: &mut Cfg, percent: u8) -> u8 {
    unsafe { crate::vl53l5cx_set_sharpener_percent(config, percent) }
}

pub fn get_target_order(config: &mut Cfg) -> (u8, u8) {
    let mut target_order: u8 = 0;
    let status = unsafe { crate::vl53l5cx_get_target_order(config, &mut target_order) };
    (status, target_order)
}

pub fn set_target_order(config: &mut Cfg, target_order: u8) -> u8 {
    unsafe { crate::vl53l5cx_set_target_order(config, target_order) }
}

pub fn stop_ranging(config: &mut Cfg) -> u8 {
    unsafe { crate::vl53l5cx_stop_ranging(config) }
}

pub fn set_power_mode(config: &mut Cfg, power_mode: u8) -> u8 {
    unsafe { crate::vl53l5cx_set_power_mode(config, power_mode) }
}

pub fn get_power_mode(config: &mut Cfg) -> (u8, u8) {
    let mut power_mode: u8 = 0;
    let status = unsafe { crate::vl53l5cx_get_power_mode(config, &mut power_mode) };
    (status, power_mode)
}

pub fn set_integration_time_ms(config: &mut Cfg, integration_time_ms: u32) -> u8 {
    unsafe { crate::vl53l5cx_set_integration_time_ms(config, integration_time_ms) }
}

pub fn get_integration_time_ms(config: &mut Cfg) -> (u8, u32) {
    let mut integration_time_ms: u32 = 0;
    let status =
        unsafe { crate::vl53l5cx_get_integration_time_ms(config, &mut integration_time_ms) };
    (status, integration_time_ms)
}

pub fn get_ranging_mode(config: &mut Cfg) -> (u8, u8) {
    let mut ranging_mode: u8 = 0;
    let status = unsafe { crate::vl53l5cx_get_ranging_mode(config, &mut ranging_mode) };
    (status, ranging_mode)
}

pub fn set_ranging_mode(config: &mut Cfg, ranging_mode: u8) -> u8 {
    unsafe { crate::vl53l5cx_set_ranging_mode(config, ranging_mode) }
}

pub fn get_vhv_repeat_count(config: &mut Cfg) -> (u8, u32) {
    let mut repeat_count: u32 = 0;
    let status = unsafe { crate::vl53l5cx_get_VHV_repeat_count(config, &mut repeat_count) };
    (status, repeat_count)
}

pub fn set_vhv_repeat_count(config: &mut Cfg, repeat_count: u32) -> u8 {
    unsafe { crate::vl53l5cx_set_VHV_repeat_count(config, repeat_count) }
}

/// Enable the sensor's internal charge pump (VL53L5CX only; the VL53L8CX ULD
/// has no equivalent). ST advise against calling this unless told to by ST.
pub fn enable_internal_cp(config: &mut Cfg) -> u8 {
    unsafe { crate::vl53l5cx_enable_internal_cp(config) }
}

/// Disable the sensor's internal charge pump (VL53L5CX only).
pub fn disable_internal_cp(config: &mut Cfg) -> u8 {
    unsafe { crate::vl53l5cx_disable_internal_cp(config) }
}

pub fn start_ranging(config: &mut Cfg) -> u8 {
    unsafe { crate::vl53l5cx_start_ranging(config) }
}

pub fn check_data_ready(config: &mut Cfg) -> (u8, u8) {
    let mut is_ready: u8 = 0;
    let status = unsafe { crate::vl53l5cx_check_data_ready(config, &mut is_ready) };
    (status, is_ready)
}

pub fn get_ranging_data(config: &mut Cfg) -> (u8, VL53L5CX_ResultsData) {
    let mut results: VL53L5CX_ResultsData = unsafe { core::mem::zeroed() };
    let status = unsafe { crate::vl53l5cx_get_ranging_data(config, &mut results) };
    (status, results)
}

#[cfg(feature = "xtalk")]
pub fn calibrate_xtalk(config: &mut Cfg, reflectance_percent: u16, nb_samples: u8, distance_mm: u16) -> u8 {
    unsafe { crate::vl53l5cx_calibrate_xtalk(config, reflectance_percent, nb_samples, distance_mm) }
}

#[cfg(feature = "xtalk")]
pub fn get_caldata_xtalk(config: &mut Cfg, xtalk_data: *mut u8) -> u8 {
    unsafe { crate::vl53l5cx_get_caldata_xtalk(config, xtalk_data) }
}

#[cfg(feature = "xtalk")]
pub fn set_caldata_xtalk(config: &mut Cfg, xtalk_data: *mut u8) -> u8 {
    unsafe { crate::vl53l5cx_set_caldata_xtalk(config, xtalk_data) }
}

#[cfg(feature = "thresholds")]
pub fn get_detection_thresholds_enable(config: &mut Cfg) -> (u8, u8) {
    let mut enabled: u8 = 0;
    let status = unsafe { crate::vl53l5cx_get_detection_thresholds_enable(config, &mut enabled) };
    (status, enabled)
}

#[cfg(feature = "thresholds")]
pub fn set_detection_thresholds_enable(config: &mut Cfg, enabled: u8) -> u8 {
    unsafe { crate::vl53l5cx_set_detection_thresholds_enable(config, enabled) }
}

#[cfg(feature = "xtalk")]
pub fn get_xtalk_margin(config: &mut Cfg) -> (u8, u32) {
    let mut xtalk_margin: u32 = 0;
    let status = unsafe { crate::vl53l5cx_get_xtalk_margin(config, &mut xtalk_margin) };
    (status, xtalk_margin)
}

#[cfg(feature = "xtalk")]
pub fn set_xtalk_margin(config: &mut Cfg, margin: u32) -> u8 {
    unsafe { crate::vl53l5cx_set_xtalk_margin(config, margin) }
}

#[cfg(feature = "thresholds")]
pub fn get_detection_thresholds(config: &mut Cfg, thresholds: *mut crate::VL53L5CX_DetectionThresholds) -> u8 {
    unsafe { crate::vl53l5cx_get_detection_thresholds(config, thresholds) }
}

#[cfg(feature = "thresholds")]
pub fn set_detection_thresholds(config: &mut Cfg, thresholds: *mut crate::VL53L5CX_DetectionThresholds) -> u8 {
    unsafe { crate::vl53l5cx_set_detection_thresholds(config, thresholds) }
}

#[cfg(any(feature = "motion", feature = "thresholds"))]
pub fn motion_indicator_init(config: &mut Cfg, motion_config: &mut VL53L5CX_Motion_Configuration, resolution: u8) -> u8 {
    unsafe { crate::vl53l5cx_motion_indicator_init(config, motion_config, resolution) }
}

#[cfg(any(feature = "motion", feature = "thresholds"))]
pub fn motion_indicator_set_distance_motion(config: &mut Cfg, motion_config: &mut VL53L5CX_Motion_Configuration, distance_min_mm: u16, distance_max_mm: u16) -> u8 {
    unsafe { crate::vl53l5cx_motion_indicator_set_distance_motion(config, motion_config, distance_min_mm, distance_max_mm) }
}

#[cfg(any(feature = "motion", feature = "thresholds"))]
pub fn motion_indicator_set_resolution(config: &mut Cfg, motion_config: &mut VL53L5CX_Motion_Configuration, resolution: u8) -> u8 {
    unsafe { crate::vl53l5cx_motion_indicator_set_resolution(config, motion_config, resolution) }
}
