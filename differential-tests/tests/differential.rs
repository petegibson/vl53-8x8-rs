//! Differential test: the C ULD wrapper (`vl53l8cx-uld`) and the pure-Rust
//! driver (`vl53-8x8`) must put the same bytes on the bus for the same API
//! calls. Reads are answered by a register model built from a recording of the
//! real VL53L8CX taken while the C suite ran on a NUCLEO-F401RE over SPI.
//!
//! Per re-enacted on-target test `T` there are three checks:
//!
//! * `T::a_c_driver_replays_c_recording`: C driver on the host, model from the
//!   C recording of `T`, transcript must equal that recording (validates the
//!   model and the harness);
//! * `T::b_rust_driver_matches_c_recording`: Rust driver, same model, transcript
//!   must equal the C recording. **This is the differential check.**
//! * `T::c_rust_driver_replays_rust_recording`: Rust driver, model from the Rust
//!   recording of `T`, transcript must equal that recording (sanity).
//!
//! Transcripts are compared after collapsing runs of identical consecutive
//! reads (poll loops ran a timing-dependent number of times on hardware).
//! Delay totals are printed (`cargo test -- --nocapture`), not asserted.
//!
//! The C ULD's platform callbacks are `#[no_mangle]` symbols, so this is the
//! only test binary that may link the C driver.

use differential_tests::mock::shared;
use differential_tests::{
    compare, normalise, run_sequence, Driver, MockDelay, MockPin, MockSpi, Op, Recording, RegisterModel,
    RustDriver, Shared, Transcript,
};
use vl53_8x8::{DetectionThreshold, PowerMode, RangingMode, Resolution, TargetOrder};

vl53l8cx_uld::bind_platform_driver_spi!(MockSpi, MockDelay);

// --- C ULD wrapper adapter ----------------------------------------------------

type CSensor = vl53l8cx_uld::Vl53l8cx<vl53l8cx_uld::Vl53l8cxSpiDevice<MockSpi>, MockDelay, MockPin, MockPin>;
type CError = vl53l8cx_uld::Error<core::convert::Infallible>;

struct CDriver {
    sensor: CSensor,
    motion: vl53l8cx_uld::MotionConfiguration,
}

impl CDriver {
    fn new(spi: MockSpi, delay: MockDelay) -> Self {
        let sensor = vl53l8cx_uld::Vl53l8cx::new_spi(spi, delay)
            .with_reset_pin_active_low(MockPin)
            .with_interrupt_pin_active_low(MockPin);
        CDriver { sensor, motion: unsafe { core::mem::zeroed() } }
    }
}

impl Driver for CDriver {
    type Error = CError;

    fn is_alive(&mut self) -> Result<bool, CError> {
        self.sensor.is_alive()
    }
    fn init(&mut self) -> Result<(), CError> {
        self.sensor.init()
    }
    fn reset(&mut self) -> Result<(), CError> {
        self.sensor.reset()
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
    fn set_external_sync_pin_enable(&mut self, on: bool) -> Result<(), CError> {
        self.sensor.set_external_sync_pin_enable(on)
    }
    fn get_external_sync_pin_enable(&mut self) -> Result<bool, CError> {
        self.sensor.get_external_sync_pin_enable()
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
        Ok((0..64).map(|z| d.distance_mm[z * vl53l8cx_uld::NB_TARGET_PER_ZONE]).collect())
    }
    fn set_detection_thresholds_enable(&mut self, on: bool) -> Result<(), CError> {
        self.sensor.set_detection_thresholds_enable(on)
    }
    fn get_detection_thresholds_enable(&mut self) -> Result<bool, CError> {
        self.sensor.get_detection_thresholds_enable()
    }
    fn set_detection_thresholds_auto_stop(&mut self, on: bool) -> Result<(), CError> {
        self.sensor.set_detection_thresholds_auto_stop(on)
    }
    fn get_detection_thresholds_auto_stop(&mut self) -> Result<bool, CError> {
        self.sensor.get_detection_thresholds_auto_stop()
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
}

// --- harness ------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Which {
    C,
    Rust,
}

/// Run on-target test `test` with driver `which` against a model built from
/// `model_from`'s recording of the same test. Returns the raw transcript and
/// whether the sequence completed.
fn replay(which: Which, test: &str, model_from: &Recording) -> Replay {
    let seq = model_from
        .sequence(test)
        .unwrap_or_else(|| panic!("{} has no sequence {test}", model_from.source));
    let st: Shared = shared(RegisterModel::from_ops(&seq.ops));
    let (spi, delay) = (MockSpi(st.clone()), MockDelay(st.clone()));
    // Value assertions inside the sequence may panic (e.g. a read-back that
    // decodes differently); the bus transcript up to that point is still the
    // primary evidence, so catch the panic and let `check` compare bytes first.
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| match which {
        Which::C => run_sequence(test, &mut CDriver::new(spi, delay)).map_err(|e| format!("{e:?}")),
        Which::Rust => run_sequence(test, &mut RustDriver::new(spi, delay)).map_err(|e| format!("{e:?}")),
    }));
    let st = st.borrow();
    println!(
        "{which:?} driver, {test}, model from {}: {} ops, {} ms of delay requested, {} sticky reads, {} unknown reads",
        model_from.source,
        st.transcript.len(),
        st.delay_ns / 1_000_000,
        st.model.sticky_reads,
        st.model.unknown.len()
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
    Replay { transcript: st.transcript.clone(), failure }
}

struct Replay {
    transcript: Transcript,
    failure: Option<String>,
}

fn check(which: Which, test: &str, model_from: &Recording, expected_from: &Recording) {
    let Replay { transcript: actual, failure } = replay(which, test, model_from);
    let expected = &expected_from.sequence(test).expect("sequence").ops;
    let (exp_n, act_n) = (normalise(expected), normalise(&actual));
    println!(
        "{test}: recorded {} ops ({} normalised) vs {which:?} driver {} ops ({} normalised)",
        expected.len(),
        exp_n.len(),
        actual.len(),
        act_n.len()
    );
    let bus = compare(&format!("recording {}", expected_from.source), &exp_n, &format!("{which:?} driver on host"), &act_n);
    match (bus, failure) {
        (Ok(()), None) => {}
        (Err(msg), None) => panic!("{msg}"),
        (Ok(()), Some(f)) => panic!("bus transcripts match, but: {f}"),
        (Err(msg), Some(f)) => panic!("{msg}\n  and the sequence did not complete: {f}"),
    }
}

/// `seq_tests! { name, name2 [ignore = "reason"], ... }` makes one module per
/// on-target test with the three checks. An `ignore` marks the differential
/// check (b) as a known divergence; see KNOWN_DIVERGENCES.md.
macro_rules! seq_tests {
    ($($name:ident $([ignore = $reason:literal])?),* $(,)?) => {
        $(
            mod $name {
                use super::*;
                const TEST: &str = stringify!($name);

                #[test]
                fn a_c_driver_replays_c_recording() {
                    check(Which::C, TEST, Recording::c_uld(), Recording::c_uld());
                }

                #[test]
                $(#[ignore = $reason])?
                fn b_rust_driver_matches_c_recording() {
                    check(Which::Rust, TEST, Recording::c_uld(), Recording::c_uld());
                }

                #[test]
                fn c_rust_driver_replays_rust_recording() {
                    check(Which::Rust, TEST, Recording::rust(), Recording::rust());
                }
            }
        )*

        #[test]
        fn every_sequence_has_tests() {
            let covered = [$(stringify!($name)),*];
            for s in differential_tests::SEQUENCES {
                assert!(covered.contains(s), "sequence {s} has no seq_tests! entry");
            }
            for s in covered {
                assert!(differential_tests::SEQUENCES.contains(&s), "{s} is not a known sequence");
            }
        }
    };
}

seq_tests! {
    test_sensor_alive,
    test_init,
    test_set_resolution,
    test_ranging_4x4,
    test_ranging_8x8,
    test_ambient_light,
    test_ranging_frequency,
    test_target_order,
    test_ranging_mode,
    test_integration_time,
    test_sharpener_percent,
    test_power_mode,
    test_set_resolution_while_ranging,
    test_reinit_after_reset,
    test_vhv_repeat_count,
    test_external_sync_pin_enable,
    test_wait_for_data_ready_blocking,
    test_detection_thresholds_enable,
    test_detection_thresholds_auto_stop,
    test_detection_thresholds [ignore = "KNOWN_DIVERGENCES.md #1: vl53l8cx-uld scales thresholds twice (x16 on the wire), vl53-8x8 once (x4)"],
    test_motion_indicator_init,
    test_motion_indicator_set_distance,
    test_motion_indicator_set_resolution,
    test_xtalk_margin,
    test_caldata_xtalk,
}

// --- harness self-checks -------------------------------------------------------

#[test]
fn recordings_load_and_describe_themselves() {
    let c = Recording::c_uld();
    let r = Recording::rust();
    assert_eq!(c.driver, "vl53l8cx-uld");
    assert_eq!(r.driver, "vl53-8x8");
    assert_eq!((c.variant.as_str(), c.bus.as_str()), ("L8", "spi"));
    assert_eq!((r.variant.as_str(), r.bus.as_str()), ("L8", "spi"));
    assert_eq!(c.sequences.len(), 25);
    assert_eq!(r.sequences.len(), 28);
    for s in differential_tests::SEQUENCES {
        assert!(c.sequence(s).is_some(), "C recording lacks {s}");
        assert!(r.sequence(s).is_some(), "Rust recording lacks {s}");
    }
    // The firmware upload is referenced through the blob, so it expands to the real image.
    let init = c.sequence("test_init").unwrap();
    let fw_writes: Vec<&Op> = init.ops.iter().filter(|o| o.data.len() >= 20480).collect();
    assert_eq!(fw_writes.iter().map(|o| o.data.len()).collect::<Vec<_>>(), [32768, 32768, 20480]);
    let uploaded: Vec<u8> = fw_writes.iter().flat_map(|o| o.data.iter().copied()).collect();
    assert_eq!(uploaded, differential_tests::transcript::FIRMWARE);
}

#[test]
fn normalise_collapses_only_identical_consecutive_reads() {
    let ops = vec![
        Op::read(6, &[0]),
        Op::read(6, &[0]),
        Op::read(6, &[1]),
        Op::read(6, &[1]),
        Op::write(6, &[1]),
        Op::write(6, &[1]),
        Op::read(6, &[1]),
    ];
    let n = normalise(&ops);
    assert_eq!(n.len(), 5);
    assert_eq!(n[0], Op::read(6, &[0]));
    assert_eq!(n[1], Op::read(6, &[1]));
    assert_eq!(n[2], Op::write(6, &[1]));
    assert_eq!(n[3], Op::write(6, &[1]));
    assert_eq!(n[4], Op::read(6, &[1]));
}

#[test]
fn compare_reports_first_difference_with_context() {
    let a: Vec<Op> = (0..10u16).map(|i| Op::write(i, &[i as u8])).collect();
    let mut b = a.clone();
    b[5].data[0] = 0xFF;
    let err = compare("a", &a, "b", &b).unwrap_err();
    assert!(err.contains("differ at op 5"), "{err}");
    assert!(err.contains(">> [   5] W 0x0005 len=1 [ff]"), "{err}");
    assert!(compare("a", &a, "a", &a).is_ok());
}

#[test]
fn model_replays_runs_in_order_and_sticks_on_the_last_value() {
    let ops = vec![Op::read(6, &[0]), Op::read(6, &[0]), Op::read(6, &[1]), Op::read(7, &[9])];
    let mut m = RegisterModel::from_ops(&ops);
    let mut b = [0u8; 1];
    let mut got = Vec::new();
    for _ in 0..5 {
        m.read(6, &mut b);
        got.push(b[0]);
    }
    assert_eq!(got, [0, 0, 1, 1, 1]);
    assert_eq!(m.sticky_reads, 2);
    m.read(7, &mut b);
    assert_eq!(b[0], 9);
    let mut b4 = [1u8; 4];
    m.read(6, &mut b4);
    assert_eq!(b4, [0, 0, 0, 0]);
    assert_eq!(m.unknown, [(6, 4)]);
}

#[test]
fn mock_i2c_decodes_the_register_protocol() {
    use embedded_hal::i2c::{I2c, Operation};
    let st = shared(RegisterModel::from_ops(&[Op::read(0x0001, &[0x0c])]));
    let mut bus = differential_tests::MockI2c(st.clone());
    bus.write(0x29, &[0x7f, 0xff, 0x00]).unwrap();
    bus.transaction(0x29, &mut [Operation::Write(&[0x2c, 0x34]), Operation::Write(&[1, 2, 3])]).unwrap();
    let mut rx = [0u8; 1];
    bus.write_read(0x29, &[0x00, 0x01], &mut rx).unwrap();
    assert_eq!(rx, [0x0c]);
    let st = st.borrow();
    assert_eq!(st.transcript, vec![Op::write(0x7fff, &[0]), Op::write(0x2c34, &[1, 2, 3]), Op::read(1, &[0x0c])]);
    assert_eq!(st.i2c_addresses, [0x29, 0x29, 0x29]);
}
