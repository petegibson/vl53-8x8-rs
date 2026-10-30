//! Differential test for the VL53L5CX: does the pure-Rust driver's L5 path
//! (`vl53-8x8`, which autodetects the part and patches the L8 firmware and
//! tables) put the same bytes on the bus as ST's own VL53L5CX ULD
//! (`vl53l5cx-uld`)?
//!
//! There is no C recording of an L5: the only hardware recording is of the
//! Rust driver running the evo-f412re suite over I2C
//! (`recordings/evo-l5-i2c-8x8.json`). The register model is built from it,
//! and per re-enacted on-target test `T` there are two checks:
//!
//! * `T::c_rust_driver_replays_rust_recording`: Rust driver on the host over
//!   `BlockingI2c`, transcript must equal the recording (sanity: the model and
//!   the I2C mock reproduce the hardware run);
//! * `T::d_c_l5_driver_matches_rust_recording`: ST's L5 ULD on the host, same
//!   model, transcript must equal the recording. **This is the differential
//!   check**: given the responses the real sensor gave the Rust driver, does
//!   ST's C code issue the same writes and reads? A read of a `(reg, len)` the
//!   recording never saw is answered with zeros and reported as unknown.
//!
//! Transcripts are compared after collapsing runs of identical consecutive
//! reads, exactly as the L8 checks do. The Rust driver uploads the firmware
//! bank that holds the L5 patch in three writes where ST's code uses one
//! (KNOWN_DIVERGENCES.md #2), which fails every strict (d) check of a sequence
//! with an `init` at the same op. So that this does not hide anything behind
//! it, each sequence has a third check:
//!
//! * `T::d2_c_l5_driver_matches_rust_recording_with_writes_coalesced`: as (d),
//!   but both transcripts additionally have register-contiguous writes merged
//!   (`coalesce_contiguous_writes`) before the comparison. Everything else is
//!   compared exactly as in (d).
//!
//! When a strict comparison fails, its message also reports the first
//! difference after coalescing. Known divergences are `#[ignore]`d with a
//! pointer to KNOWN_DIVERGENCES.md.
//!
//! The L5 ULD's platform callbacks are `#[no_mangle]` `vl53l5cx_platform_*`
//! symbols, so this binary is the only one that may link the L5 C driver (the
//! L8 one lives in `tests/differential.rs`).

use differential_tests::mock::shared;
use differential_tests::transcript::{blobs, FIRMWARE};
use differential_tests::{
    coalesce_contiguous_writes, compare, normalise, run_evo_sequence, strip_module_type_probe, Dir, Driver, MockDelay,
    MockI2c, MockPin, Op, MODULE_TYPE_PROBE_ANSWER_LEN, MODULE_TYPE_PROBE_ANSWER_REG,
    Recording, RegisterModel, RustDriver, Shared, Transcript, EVO_SEQUENCES,
};
use vl53_8x8::sensor::{L5_CALIBRATE_XTALK_PATCH, L5_CONFIG_PATCH, L5_FW_PATCH};
use vl53_8x8::{DetectionThreshold, FirmwareError, PowerMode, RangingMode, Resolution, TargetOrder};

vl53l5cx_uld::bind_platform_driver!(MockI2c, MockDelay);

const SENSOR_ADDRESS: u8 = 0x29;

// --- C L5 ULD wrapper adapter -----------------------------------------------------

type CSensor = vl53l5cx_uld::Vl53l5cx<MockI2c, MockDelay, MockPin, MockPin, MockPin>;
type CError = vl53l5cx_uld::Error<core::convert::Infallible>;

struct CDriver {
    sensor: CSensor,
    motion: vl53l5cx_uld::MotionConfiguration,
}

impl CDriver {
    fn new(i2c: MockI2c, delay: MockDelay) -> Self {
        let sensor = vl53l5cx_uld::Vl53l5cx::new(i2c, delay)
            .with_lpn_pin(MockPin)
            .with_interrupt_pin_active_low(MockPin)
            .with_power_enable_pin_active_high(MockPin);
        CDriver { sensor, motion: unsafe { core::mem::zeroed() } }
    }
}

/// The VL53L5CX ULD has no external sync pin and no detection-threshold
/// auto-stop; no L5 sequence calls these.
fn unsupported() -> CError {
    vl53l5cx_uld::Error::Firmware(FirmwareError::InvalidParam)
}

impl Driver for CDriver {
    type Error = CError;

    fn is_alive(&mut self) -> Result<bool, CError> {
        self.sensor.is_alive()
    }
    fn init(&mut self) -> Result<(), CError> {
        self.sensor.init()
    }
    fn power_cycle(&mut self) -> Result<(), CError> {
        self.sensor.power_cycle()
    }
    fn set_resolution(&mut self, r: Resolution) -> Result<(), CError> {
        self.sensor.set_resolution(r)
    }
    fn get_resolution(&mut self) -> Result<Resolution, CError> {
        self.sensor.get_resolution()
    }
    fn set_ranging_frequency_hz(&mut self, hz: u8) -> Result<(), CError> {
        self.sensor.set_ranging_frequency_hz(hz)
    }
    fn get_ranging_frequency_hz(&mut self) -> Result<u8, CError> {
        self.sensor.get_ranging_frequency_hz()
    }
    fn set_integration_time_ms(&mut self, ms: u32) -> Result<(), CError> {
        self.sensor.set_integration_time_ms(ms)
    }
    fn get_integration_time_ms(&mut self) -> Result<u32, CError> {
        self.sensor.get_integration_time_ms()
    }
    fn set_target_order(&mut self, o: TargetOrder) -> Result<(), CError> {
        self.sensor.set_target_order(o)
    }
    fn get_target_order(&mut self) -> Result<TargetOrder, CError> {
        self.sensor.get_target_order()
    }
    fn set_ranging_mode(&mut self, m: RangingMode) -> Result<(), CError> {
        self.sensor.set_ranging_mode(m)
    }
    fn get_ranging_mode(&mut self) -> Result<RangingMode, CError> {
        self.sensor.get_ranging_mode()
    }
    fn set_sharpener_percent(&mut self, p: u8) -> Result<(), CError> {
        self.sensor.set_sharpener_percent(p)
    }
    fn get_sharpener_percent(&mut self) -> Result<u8, CError> {
        self.sensor.get_sharpener_percent()
    }
    fn set_power_mode(&mut self, m: PowerMode) -> Result<(), CError> {
        self.sensor.set_power_mode(m)
    }
    fn get_power_mode(&mut self) -> Result<PowerMode, CError> {
        self.sensor.get_power_mode()
    }
    fn set_vhv_repeat_count(&mut self, n: u32) -> Result<(), CError> {
        self.sensor.set_vhv_repeat_count(n)
    }
    fn get_vhv_repeat_count(&mut self) -> Result<u32, CError> {
        self.sensor.get_vhv_repeat_count()
    }
    fn set_external_sync_pin_enable(&mut self, _on: bool) -> Result<(), CError> {
        Err(unsupported())
    }
    fn get_external_sync_pin_enable(&mut self) -> Result<bool, CError> {
        Err(unsupported())
    }
    fn start_ranging(&mut self) -> Result<(), CError> {
        self.sensor.start_ranging()
    }
    fn stop_ranging(&mut self) -> Result<(), CError> {
        self.sensor.stop_ranging()
    }
    fn check_data_ready(&mut self) -> Result<bool, CError> {
        self.sensor.check_data_ready()
    }
    fn wait_for_data_ready_blocking(&mut self) -> Result<(), CError> {
        self.sensor.wait_for_data_ready_blocking()
    }
    fn get_ranging_data(&mut self) -> Result<Vec<i16>, CError> {
        let d = self.sensor.get_ranging_data()?;
        Ok((0..64).map(|z| d.distance_mm[z * vl53l5cx_uld::NB_TARGET_PER_ZONE]).collect())
    }
    fn set_detection_thresholds_enable(&mut self, on: bool) -> Result<(), CError> {
        self.sensor.set_detection_thresholds_enable(on)
    }
    fn get_detection_thresholds_enable(&mut self) -> Result<bool, CError> {
        self.sensor.get_detection_thresholds_enable()
    }
    fn set_detection_thresholds_auto_stop(&mut self, _on: bool) -> Result<(), CError> {
        Err(unsupported())
    }
    fn get_detection_thresholds_auto_stop(&mut self) -> Result<bool, CError> {
        Err(unsupported())
    }
    fn set_detection_thresholds(&mut self, t: [DetectionThreshold; 64]) -> Result<(), CError> {
        self.sensor.set_detection_thresholds(t)
    }
    fn get_detection_thresholds(&mut self) -> Result<[DetectionThreshold; 64], CError> {
        self.sensor.get_detection_thresholds()
    }
    fn motion_indicator_init(&mut self, r: Resolution) -> Result<(), CError> {
        self.sensor.motion_indicator_init(&mut self.motion, r)
    }
    fn motion_indicator_set_distance_motion(&mut self, min_mm: u16, max_mm: u16) -> Result<(), CError> {
        self.sensor.motion_indicator_set_distance_motion(&mut self.motion, min_mm, max_mm)
    }
    fn motion_indicator_set_resolution(&mut self, r: Resolution) -> Result<(), CError> {
        self.sensor.motion_indicator_set_resolution(&mut self.motion, r)
    }
    fn set_xtalk_margin(&mut self, margin: u32) -> Result<(), CError> {
        self.sensor.set_xtalk_margin(margin)
    }
    fn get_xtalk_margin(&mut self) -> Result<u32, CError> {
        self.sensor.get_xtalk_margin()
    }
    fn get_caldata_xtalk(&mut self, buf: &mut [u8]) -> Result<(), CError> {
        self.sensor.get_caldata_xtalk(buf)
    }
    fn set_caldata_xtalk(&mut self, buf: &mut [u8]) -> Result<(), CError> {
        self.sensor.set_caldata_xtalk(buf)
    }
    fn calibrate_xtalk(&mut self, reflectance_percent: u8, nb_samples: u8, distance_mm: u16) -> Result<(), CError> {
        self.sensor.calibrate_xtalk(u16::from(reflectance_percent), nb_samples, distance_mm)
    }
    fn firmware_error(e: &CError) -> Option<FirmwareError> {
        match e {
            vl53l5cx_uld::Error::Firmware(f) => Some(*f),
            _ => None,
        }
    }
}

// --- harness ------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Which {
    C,
    Rust,
}

struct Replay {
    transcript: Transcript,
    failure: Option<String>,
    /// `(reg, len)` keys the driver read that the recording never read.
    unknown: Vec<(u16, usize)>,
}

/// Run evo test `test` with driver `which` against a model built from the L5
/// recording of the same test.
fn replay(which: Which, test: &str) -> Replay {
    let rec = Recording::rust_l5();
    let seq = rec.sequence(test).unwrap_or_else(|| panic!("{} has no sequence {test}", rec.source));
    replay_with(which, test, &seq.ops)
}

/// Run evo test `test` with driver `which` against a model built from `model_ops`.
fn replay_with(which: Which, test: &str, model_ops: &[Op]) -> Replay {
    let rec = Recording::rust_l5();
    let st: Shared = shared(RegisterModel::from_ops(&strip_module_type_probe(model_ops)).intercepting_module_type_probe());
    let (i2c, delay) = (MockI2c(st.clone()), MockDelay(st.clone()));
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| match which {
        Which::C => run_evo_sequence(test, &mut CDriver::new(i2c, delay)).map_err(|e| format!("{e:?}")),
        Which::Rust => run_evo_sequence(test, &mut RustDriver::new_i2c(i2c, delay)).map_err(|e| format!("{e:?}")),
    }));
    let st = st.borrow();
    println!(
        "{which:?} driver, {test}, model from {}: {} ops, {} ms of delay requested, {} sticky reads, {} unknown reads",
        rec.source,
        st.transcript.len(),
        st.delay_ns / 1_000_000,
        st.model.sticky_reads,
        st.model.unknown.len()
    );
    assert!(
        st.i2c_addresses.iter().all(|&a| a == SENSOR_ADDRESS),
        "{which:?} driver addressed something other than 0x{SENSOR_ADDRESS:02x}: {:?}",
        st.i2c_addresses
    );
    let failure = match result {
        Ok(Ok(())) => None,
        Ok(Err(e)) => Some(format!("{which:?} driver returned an error in {test}: {e}")),
        Err(payload) => {
            let msg = payload
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| payload.downcast_ref::<&str>().map(|s| s.to_string()))
                .unwrap_or_else(|| "non-string panic".into());
            Some(format!("{which:?} driver, {test}: sequence assertion failed: {msg}"))
        }
    };
    Replay { transcript: st.transcript.clone(), failure, unknown: st.model.unknown.clone() }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Mode {
    /// `normalise` only, like the L8 checks.
    Strict,
    /// `normalise`, then `coalesce_contiguous_writes`, on both sides.
    Coalesced,
}

fn check(which: Which, test: &str, mode: Mode) {
    let rec = Recording::rust_l5();
    let expected = &rec.sequence(test).expect("sequence").ops;
    let replayed = replay(which, test);
    check_against(which, test, mode, &format!("recording {}", rec.source), expected, replayed);
}

fn check_against(which: Which, test: &str, mode: Mode, expected_label: &str, expected: &[Op], replayed: Replay) {
    let Replay { transcript: actual, failure, unknown } = replayed;
    // The pure-Rust driver ends an L5-class `init` with the module-type probe
    // (KNOWN_DIVERGENCES.md #6): one DCI read ST's L5 code never issues and
    // the current recording predates. Removed from both sides before every
    // comparison, and its answer key dropped from the unknown-read list.
    let (mut exp_n, mut act_n) = (
        normalise(&strip_module_type_probe(expected)),
        normalise(&strip_module_type_probe(&actual)),
    );
    let unknown: Vec<(u16, usize)> = unknown
        .into_iter()
        .filter(|&(reg, len)| !(reg == MODULE_TYPE_PROBE_ANSWER_REG && len == MODULE_TYPE_PROBE_ANSWER_LEN))
        .collect();
    println!(
        "{test}: expected {} ops ({} normalised) vs {which:?} driver {} ops ({} normalised)",
        expected.len(),
        exp_n.len(),
        actual.len(),
        act_n.len()
    );
    let mut label = format!("{which:?} driver on host");
    let mut expected_label = expected_label.to_string();
    if mode == Mode::Coalesced {
        exp_n = coalesce_contiguous_writes(&exp_n);
        act_n = coalesce_contiguous_writes(&act_n);
        label.push_str(" (writes coalesced)");
        expected_label.push_str(" (writes coalesced)");
    }
    let mut problems: Vec<String> = Vec::new();
    if let Err(msg) = compare(&expected_label, &exp_n, &label, &act_n) {
        problems.push(msg);
        if mode == Mode::Strict {
            // Second look, diagnostic only: with register-contiguous writes merged.
            let (exp_c, act_c) = (coalesce_contiguous_writes(&exp_n), coalesce_contiguous_writes(&act_n));
            match compare(&format!("{expected_label} (writes coalesced)"), &exp_c, &format!("{label} (writes coalesced)"), &act_c) {
                Ok(()) => problems.push("after coalescing register-contiguous writes the transcripts are identical".into()),
                Err(msg) => problems.push(format!("after coalescing register-contiguous writes: {msg}")),
            }
        }
    }
    if !unknown.is_empty() {
        let list: Vec<String> = unknown.iter().map(|(r, l)| format!("(0x{r:04x}, len {l})")).collect();
        problems.push(format!("{which:?} driver read {} (reg, len) key(s) the recording never read (answered with zeros): {}", unknown.len(), list.join(", ")));
    }
    if let Some(f) = failure {
        problems.push(if problems.is_empty() { format!("bus transcripts match, but: {f}") } else { format!("and the sequence did not complete: {f}") });
    }
    if !problems.is_empty() {
        panic!("{}", problems.join("\n"));
    }
}

/// `evo_tests! { name [ignore = "reason"] [ignore_coalesced = "reason"], ... }`
/// makes one module per evo on-target test with the three checks. `ignore`
/// marks the strict differential check (d) as a known divergence,
/// `ignore_coalesced` the write-coalesced one (d2); see KNOWN_DIVERGENCES.md.
macro_rules! evo_tests {
    ($($name:ident $([ignore = $reason:literal])? $([ignore_coalesced = $reason2:literal])?),* $(,)?) => {
        $(
            mod $name {
                use super::*;
                const TEST: &str = stringify!($name);

                #[test]
                fn c_rust_driver_replays_rust_recording() {
                    check(Which::Rust, TEST, Mode::Strict);
                }

                #[test]
                $(#[ignore = $reason])?
                fn d_c_l5_driver_matches_rust_recording() {
                    check(Which::C, TEST, Mode::Strict);
                }

                #[test]
                $(#[ignore = $reason2])?
                fn d2_c_l5_driver_matches_rust_recording_with_writes_coalesced() {
                    check(Which::C, TEST, Mode::Coalesced);
                }
            }
        )*

        #[test]
        fn every_evo_sequence_has_tests() {
            let covered = [$(stringify!($name)),*];
            for s in EVO_SEQUENCES {
                assert!(covered.contains(s), "sequence {s} has no evo_tests! entry");
            }
            for s in covered {
                assert!(EVO_SEQUENCES.contains(&s), "{s} is not a known evo sequence");
            }
        }
    };
}


evo_tests! {
    async_i2c_alive,
    init_uploads_firmware_over_async_i2c [ignore = "KNOWN_DIVERGENCES.md #2: vl53-8x8 writes the firmware bank holding the L5 patch in 3 writes, ST's ULD in 1"],
    set_resolution [ignore = "KNOWN_DIVERGENCES.md #2: vl53-8x8 writes the firmware bank holding the L5 patch in 3 writes, ST's ULD in 1"],
    ranging_4x4_async [ignore = "KNOWN_DIVERGENCES.md #2: vl53-8x8 writes the firmware bank holding the L5 patch in 3 writes, ST's ULD in 1"],
    ranging_8x8_async [ignore = "KNOWN_DIVERGENCES.md #2: vl53-8x8 writes the firmware bank holding the L5 patch in 3 writes, ST's ULD in 1"],
    ambient_light [ignore = "KNOWN_DIVERGENCES.md #2: vl53-8x8 writes the firmware bank holding the L5 patch in 3 writes, ST's ULD in 1"],
    ranging_frequency [ignore = "KNOWN_DIVERGENCES.md #2: vl53-8x8 writes the firmware bank holding the L5 patch in 3 writes, ST's ULD in 1"],
    integration_time [ignore = "KNOWN_DIVERGENCES.md #2: vl53-8x8 writes the firmware bank holding the L5 patch in 3 writes, ST's ULD in 1"],
    target_order [ignore = "KNOWN_DIVERGENCES.md #2: vl53-8x8 writes the firmware bank holding the L5 patch in 3 writes, ST's ULD in 1"],
    ranging_mode [ignore = "KNOWN_DIVERGENCES.md #2: vl53-8x8 writes the firmware bank holding the L5 patch in 3 writes, ST's ULD in 1"],
    sharpener_percent [ignore = "KNOWN_DIVERGENCES.md #2: vl53-8x8 writes the firmware bank holding the L5 patch in 3 writes, ST's ULD in 1"],
    power_mode_sleep_wakeup [ignore = "KNOWN_DIVERGENCES.md #2: vl53-8x8 writes the firmware bank holding the L5 patch in 3 writes, ST's ULD in 1"],
    calibrate_xtalk_leaves_sensor_usable [ignore = "KNOWN_DIVERGENCES.md #2: vl53-8x8 writes the firmware bank holding the L5 patch in 3 writes, ST's ULD in 1"],
    calibrate_xtalk_rejects_invalid_params [ignore = "KNOWN_DIVERGENCES.md #2: vl53-8x8 writes the firmware bank holding the L5 patch in 3 writes, ST's ULD in 1"]
        [ignore_coalesced = "KNOWN_DIVERGENCES.md #5: ST's ULD reads the configuration before rejecting invalid xtalk parameters"],
}


// --- patch-table cross-checks -----------------------------------------------------------

/// Concatenate the firmware bank writes (pages 0x09, 0x0A, 0x0B) of an init
/// transcript into the image as the sensor received it, whatever the chunking.
fn uploaded_firmware(ops: &[Op]) -> Vec<u8> {
    let mut page = 0u8;
    let mut banks: [Vec<u8>; 3] = Default::default();
    for op in ops.iter().filter(|o| o.dir == Dir::W) {
        if op.reg == 0x7FFF {
            page = op.data[0];
        } else if (0x09..=0x0B).contains(&page) {
            let bank = &mut banks[usize::from(page - 0x09)];
            assert_eq!(bank.len(), usize::from(op.reg), "firmware bank write is not contiguous");
            bank.extend_from_slice(&op.data);
        }
    }
    banks.concat()
}

/// The first write of exactly `len` bytes to `reg` in `ops`.
fn table_write(ops: &[Op], reg: u16, len: usize) -> &[u8] {
    ops.iter()
        .find(|o| o.dir == Dir::W && o.reg == reg && o.data.len() == len)
        .map(|o| o.data.as_slice())
        .unwrap_or_else(|| panic!("no {len}-byte write to 0x{reg:04x}"))
}

/// Offsets at which two equal-length byte strings differ, with both values.
fn diff_offsets(a: &[u8], b: &[u8]) -> Vec<(usize, u8, u8)> {
    assert_eq!(a.len(), b.len());
    a.iter().zip(b).enumerate().filter(|(_, (x, y))| x != y).map(|(i, (x, y))| (i, *x, *y)).collect()
}

fn assert_patch_is_exactly(label: &str, l8: &[u8], l5: &[u8], patch: &[(usize, u8)]) {
    let diffs = diff_offsets(l8, l5);
    let got: Vec<(usize, u8)> = diffs.iter().map(|&(o, _, v)| (o, v)).collect();
    assert_eq!(got, patch, "{label}: L8/L5 difference is not exactly the patch table");
    for &(o, l8_byte, l5_byte) in &diffs {
        println!("{label}: offset 0x{o:05x}: L8 0x{l8_byte:02x} -> L5 0x{l5_byte:02x}");
    }
}

/// (e) The L5 firmware is the L8 one patched at exactly the 7 offsets of
/// `L5_FW_PATCH`, on the wire: the Rust L5 hardware recording, ST's L5 ULD on
/// the host and the C L8 hardware recording agree with the tables.
#[test]
fn l5_firmware_upload_differs_from_l8_at_exactly_the_patched_offsets() {
    let rust_l5 = uploaded_firmware(&Recording::rust_l5().sequence("init_uploads_firmware_over_async_i2c").unwrap().ops);
    let c_l8 = uploaded_firmware(&Recording::c_uld().sequence("test_init").unwrap().ops);
    let c_l5 = uploaded_firmware(&replay(Which::C, "init_uploads_firmware_over_async_i2c").transcript);
    assert_eq!(c_l8, FIRMWARE, "C L8 recording uploads the L8 base image");
    assert_eq!(c_l8.len(), rust_l5.len());
    assert_eq!(rust_l5, c_l5, "ST's L5 firmware array and the Rust patched image differ on the wire");
    assert_eq!(rust_l5, blobs()["firmware_l5"]);
    assert_eq!(L5_FW_PATCH.len(), 7);
    assert_patch_is_exactly("firmware", &c_l8, &rust_l5, L5_FW_PATCH);
}

/// (e) Same for the 972-byte default configuration (`L5_CONFIG_PATCH`, 5 bytes).
#[test]
fn l5_default_configuration_differs_from_l8_at_exactly_the_patched_offsets() {
    let cfg_len = blobs()["default_configuration"].len();
    let rust_l5 = table_write(&Recording::rust_l5().sequence("init_uploads_firmware_over_async_i2c").unwrap().ops, 0x2C34, cfg_len).to_vec();
    let c_l8 = table_write(&Recording::c_uld().sequence("test_init").unwrap().ops, 0x2C34, cfg_len).to_vec();
    let c_l5 = table_write(&replay(Which::C, "init_uploads_firmware_over_async_i2c").transcript, 0x2C34, cfg_len).to_vec();
    assert_eq!(c_l8, blobs()["default_configuration"]);
    assert_eq!(rust_l5, c_l5, "ST's L5 default configuration and the Rust patched table differ on the wire");
    assert_eq!(rust_l5, blobs()["default_configuration_l5"]);
    assert_patch_is_exactly("default_configuration", &c_l8, &rust_l5, L5_CONFIG_PATCH);
}

/// (e) Same for the 984-byte xtalk calibration command (`L5_CALIBRATE_XTALK_PATCH`,
/// 2 bytes). The C L8 suite has no xtalk calibration test, so the L8 side is
/// the `CALIBRATE_XTALK` table itself (checked against the L8 ULD in `vl53-8x8`).
#[test]
fn l5_calibrate_xtalk_command_differs_from_l8_at_exactly_the_patched_offsets() {
    let l8 = blobs()["calibrate_xtalk"].clone();
    let seq = &Recording::rust_l5().sequence("calibrate_xtalk_leaves_sensor_usable").unwrap().ops;
    let rust_l5 = seq
        .iter()
        .find(|o| o.dir == Dir::W && o.data.len() == l8.len())
        .map(|o| (o.reg, o.data.clone()))
        .expect("xtalk calibration command write in the L5 recording");
    let c_l5 = table_write(&replay(Which::C, "calibrate_xtalk_leaves_sensor_usable").transcript, rust_l5.0, l8.len()).to_vec();
    assert_eq!(rust_l5.1, c_l5, "ST's L5 xtalk command and the Rust patched table differ on the wire");
    assert_eq!(rust_l5.1, blobs()["calibrate_xtalk_l5"]);
    assert_patch_is_exactly("calibrate_xtalk", &l8, &rust_l5.1, L5_CALIBRATE_XTALK_PATCH);
}

/// Investigation aid, not a check: print the full transcript of one driver on
/// one sequence. `DIFFERENTIAL_DUMP=c:calibrate_xtalk_rejects_invalid_params
/// cargo test -p differential-tests --test differential_l5 dump -- --ignored --nocapture`
/// (`rust:` for the Rust driver).
#[test]
#[ignore = "investigation aid; set DIFFERENTIAL_DUMP=<c|rust>:<sequence>"]
fn dump_transcript() {
    let spec = std::env::var("DIFFERENTIAL_DUMP").expect("DIFFERENTIAL_DUMP=<c|rust>:<sequence>");
    let (which, test) = spec.split_once(':').expect("<c|rust>:<sequence>");
    let which = match which {
        "c" => Which::C,
        "rust" => Which::Rust,
        other => panic!("unknown driver {other:?}"),
    };
    let Replay { transcript, failure, unknown } = replay(which, test);
    for (i, op) in transcript.iter().enumerate() {
        println!("[{i:4}] {op:?}");
    }
    println!("unknown reads: {unknown:?}");
    if let Some(f) = failure {
        println!("{f}");
    }
}

// --- harness self-checks ----------------------------------------------------------------

#[test]
fn l5_recording_loads_and_describes_itself() {
    let r = Recording::rust_l5();
    assert_eq!(r.driver, "vl53-8x8");
    assert_eq!((r.variant.as_str(), r.bus.as_str()), ("L5", "i2c"));
    assert_eq!(r.sequences.len(), 14);
    for s in EVO_SEQUENCES {
        assert!(r.sequence(s).is_some(), "L5 recording lacks {s}");
    }
    // The device answered with the L5 revision id.
    let alive = r.sequence("async_i2c_alive").unwrap();
    assert_eq!(alive.ops, vec![Op::write(0x7FFF, &[0]), Op::read(0, &[0xF0]), Op::read(1, &[0x02]), Op::write(0x7FFF, &[2])]);
    // Firmware expands from the L5 blob: three banks, the middle one split
    // into three writes around the 64-byte patch window (see KNOWN_DIVERGENCES.md).
    let init = r.sequence("init_uploads_firmware_over_async_i2c").unwrap();
    assert_eq!(uploaded_firmware(&init.ops), blobs()["firmware_l5"]);
    let mut page = 0u8;
    let mut bank_writes: Vec<(u8, u16, usize)> = Vec::new();
    for op in init.ops.iter().filter(|o| o.dir == Dir::W) {
        if op.reg == 0x7FFF {
            page = op.data[0];
        } else if (0x09..=0x0B).contains(&page) {
            bank_writes.push((page, op.reg, op.data.len()));
        }
    }
    assert_eq!(
        bank_writes,
        [(0x09, 0, 32768), (0x0A, 0, 19265), (0x0A, 19265, 64), (0x0A, 19329, 13439), (0x0B, 0, 20480)]
    );
    assert_eq!(L5_FW_PATCH[0].0, 0x8000 + 19265, "the split starts at the first patched byte");
}

#[test]
fn coalesce_merges_only_register_contiguous_writes() {
    let ops = vec![
        Op::write(0x7FFF, &[9]),
        Op::write(0, &[1, 2, 3]),
        Op::write(3, &[4]),
        Op::write(4, &[5, 6]),
        Op::read(6, &[0]),
        Op::write(6, &[7]),
        Op::write(8, &[8]),
    ];
    let c = coalesce_contiguous_writes(&ops);
    assert_eq!(c, vec![Op::write(0x7FFF, &[9]), Op::write(0, &[1, 2, 3, 4, 5, 6]), Op::read(6, &[0]), Op::write(6, &[7]), Op::write(8, &[8])]);
}
