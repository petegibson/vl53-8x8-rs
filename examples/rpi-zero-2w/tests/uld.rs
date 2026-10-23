//! Hardware-in-the-loop tests for ST's C ULD on the Pi: `vl53l8cx-uld` over
//! SPI0 by default, `vl53l5cx-uld` over I2C1 with `--features i2c-bus`
//! (`rpi_zero_2w::uld` picks the wrapper, bus and LPn pin).
//!
//! Run on the Pi itself with `cargo test --test uld [--features i2c-bus]`, or
//! from the dev shell with `just test-pi [i2c-bus]`, which cross-compiles for
//! musl and runs the binary on the Pi over ssh (`PI_HOST`, default
//! `pi@toftestpi`). The tests share one sensor, so they take a process-wide lock
//! and the remote runner also sets `RUST_TEST_THREADS=1`.
//!
//! Ported from `examples/evo-f412re-uld/tests/sensor.rs` /
//! `examples/nucleo-f401re-uld/tests/sensor.rs` without the power-cycle tests
//! (the Pololu boards have no power-enable pin) and with an LPn gating test
//! on I2C. Scene: whatever is in front of the sensor, so distances are only
//! bounded, not matched.

use std::sync::{Mutex, MutexGuard};
use std::time::{Duration, Instant};

use rpi_zero_2w::uld::{
    self, Driver, PowerMode, RangingMode, Resolution, ResultsData, TargetOrder, NB_TARGET_PER_ZONE,
};
use rpi_zero_2w::LPN_GPIO;

/// One sensor, one test at a time.
static HW: Mutex<()> = Mutex::new(());

/// Open the bus and LPn (LPn high) without initialising the sensor.
fn connect() -> (MutexGuard<'static, ()>, Driver) {
    let guard = HW.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
    #[cfg(feature = "i2c-bus")]
    if rpi_zero_2w::recover_default_address(TEMP_ADDRESS).expect("probe for a stray address") {
        eprintln!("moved a sensor left at {TEMP_ADDRESS:#04x} back to the default address");
    }
    let sensor = uld::connect().expect("open the bus and LPn (device permissions? wiring?)");
    (guard, sensor)
}

/// Address used by `set_address_round_trip`.
const TEMP_ADDRESS: u8 = 0x35;

/// [`connect`] followed by `init()` (firmware upload).
fn init() -> (MutexGuard<'static, ()>, Driver) {
    let (guard, mut sensor) = connect();
    sensor.init().expect("init (firmware upload)");
    (guard, sensor)
}

/// Spin on `check_data_ready` with a timeout generous enough for 5 Hz ranging.
fn wait_ready(sensor: &mut Driver) {
    let started = Instant::now();
    while !sensor.check_data_ready().expect("check_data_ready") {
        assert!(started.elapsed() < Duration::from_secs(3), "timed out waiting for a frame");
        std::thread::sleep(Duration::from_millis(2));
    }
}

/// Assert that every zone whose first target is a valid return (ST status 5,
/// or 6/9 for lower confidence) carries a plausible distance, and that at
/// least three quarters of the first `zones` zones are valid. Zones looking
/// at nothing (a corner past the ceiling, a dark absorber) legitimately
/// report status 255 with distance 0. Per-target arrays are laid out
/// `zone * NB_TARGET_PER_ZONE + target`.
fn assert_zone_distances(data: &ResultsData, zones: usize) {
    let mut valid = 0;
    for zone in 0..zones {
        let i = zone * NB_TARGET_PER_ZONE;
        let (d, s) = (data.distance_mm[i], data.target_status[i]);
        if matches!(s, 5 | 6 | 9) {
            assert!(d > 0 && d < 4000, "zone {zone} status {s} distance {d} mm out of range");
            valid += 1;
        }
    }
    assert!(valid * 4 >= zones * 3, "only {valid}/{zones} zones have a valid target");
}

#[test]
fn sensor_alive() {
    let (_hw, mut sensor) = connect();
    assert!(
        sensor.is_alive().expect("is_alive bus transaction"),
        "sensor.is_alive returned false: check wiring, the bus-mode selector and LPn on BCM GPIO{LPN_GPIO}"
    );
}

#[test]
fn init_uploads_firmware() {
    let (_hw, mut sensor) = connect();
    let started = Instant::now();
    sensor.init().expect("init");
    eprintln!("init (firmware upload) took {:.2} s", started.elapsed().as_secs_f32());
    assert!(sensor.is_alive().unwrap());
}

#[test]
fn set_resolution() {
    let (_hw, mut sensor) = init();

    sensor.set_resolution(Resolution::Res4x4).unwrap();
    assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res4x4);

    sensor.set_resolution(Resolution::Res8x8).unwrap();
    assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res8x8);
}

#[test]
fn ranging_4x4() {
    let (_hw, mut sensor) = init();
    sensor.set_resolution(Resolution::Res4x4).unwrap();
    sensor.set_ranging_frequency_hz(5).unwrap();
    sensor.start_ranging().unwrap();

    wait_ready(&mut sensor);
    let data = sensor.get_ranging_data().unwrap();
    assert_zone_distances(&data, 16);

    sensor.stop_ranging().unwrap();
}

#[test]
fn ranging_8x8() {
    let (_hw, mut sensor) = init();
    sensor.set_resolution(Resolution::Res8x8).unwrap();
    sensor.set_ranging_frequency_hz(5).unwrap();
    sensor.start_ranging().unwrap();

    wait_ready(&mut sensor);
    let data = sensor.get_ranging_data().unwrap();
    assert_zone_distances(&data, 64);

    sensor.stop_ranging().unwrap();
}

/// Several consecutive frames must all parse and stay plausible.
#[test]
fn ranging_streams_frames() {
    const FRAMES: usize = 10;
    let (_hw, mut sensor) = init();
    sensor.set_resolution(Resolution::Res4x4).unwrap();
    sensor.set_ranging_frequency_hz(10).unwrap();
    sensor.start_ranging().unwrap();

    let started = Instant::now();
    for _ in 0..FRAMES {
        wait_ready(&mut sensor);
        let data = sensor.get_ranging_data().unwrap();
        assert_zone_distances(&data, 16);
    }
    let elapsed = started.elapsed();
    eprintln!("{FRAMES} frames at 10 Hz in {:.2} s", elapsed.as_secs_f32());
    // 10 frames at 10 Hz is ~1 s; allow a wide margin for Linux scheduling.
    assert!(elapsed < Duration::from_secs(3), "frames arrived far slower than 10 Hz");

    sensor.stop_ranging().unwrap();
}

#[test]
fn ambient_light() {
    let (_hw, mut sensor) = init();
    sensor.set_resolution(Resolution::Res4x4).unwrap();
    sensor.set_ranging_frequency_hz(5).unwrap();
    sensor.start_ranging().unwrap();

    wait_ready(&mut sensor);
    let data = sensor.get_ranging_data().unwrap();
    // Per-zone ambient rounds to 0 in low light, so only require that the
    // block transferred and at least one zone reports light.
    let nonzero = data.ambient_per_spad[..16].iter().filter(|&&a| a > 0).count();
    eprintln!("ambient: {nonzero}/16 zones non-zero");
    assert!(nonzero > 0, "no zone reported ambient light (sensor in the dark?)");

    sensor.stop_ranging().unwrap();
}

#[test]
fn ranging_frequency() {
    let (_hw, mut sensor) = init();

    for hz in [5u8, 15, 30] {
        sensor.set_ranging_frequency_hz(hz).unwrap();
        assert_eq!(sensor.get_ranging_frequency_hz().unwrap(), hz);
    }
}

#[test]
fn target_order() {
    let (_hw, mut sensor) = init();

    sensor.set_target_order(TargetOrder::Closest).unwrap();
    assert_eq!(sensor.get_target_order().unwrap(), TargetOrder::Closest);

    sensor.set_target_order(TargetOrder::Strongest).unwrap();
    assert_eq!(sensor.get_target_order().unwrap(), TargetOrder::Strongest);
}

#[test]
fn ranging_mode() {
    let (_hw, mut sensor) = init();

    sensor.set_ranging_mode(RangingMode::Continuous).unwrap();
    assert_eq!(sensor.get_ranging_mode().unwrap(), RangingMode::Continuous);

    sensor.set_ranging_mode(RangingMode::Autonomous).unwrap();
    assert_eq!(sensor.get_ranging_mode().unwrap(), RangingMode::Autonomous);
}

#[test]
fn integration_time() {
    let (_hw, mut sensor) = init();

    sensor.set_integration_time_ms(20).unwrap();
    assert_eq!(sensor.get_integration_time_ms().unwrap(), 20);

    sensor.set_integration_time_ms(50).unwrap();
    assert_eq!(sensor.get_integration_time_ms().unwrap(), 50);
}

#[test]
fn sharpener_percent() {
    let (_hw, mut sensor) = init();

    // The sensor quantises via u8: set=(pct*255)/100, get=(raw*100)/255,
    // so read-back may differ by +-1 from the written value.
    sensor.set_sharpener_percent(25).unwrap();
    let v = sensor.get_sharpener_percent().unwrap();
    assert!((24..=26).contains(&v), "expected ~25, got {v}");

    sensor.set_sharpener_percent(75).unwrap();
    let v = sensor.get_sharpener_percent().unwrap();
    assert!((74..=76).contains(&v), "expected ~75, got {v}");
}

/// Sleep/Wakeup round trip. The VL53L5CX ULD has no deep-sleep mode and the
/// wrapper rejects it without touching the bus.
#[test]
fn power_mode() {
    let (_hw, mut sensor) = init();

    assert_eq!(sensor.get_power_mode().unwrap(), PowerMode::Wakeup);

    sensor.set_power_mode(PowerMode::Sleep).unwrap();
    assert_eq!(sensor.get_power_mode().unwrap(), PowerMode::Sleep);

    sensor.set_power_mode(PowerMode::Wakeup).unwrap();
    assert_eq!(sensor.get_power_mode().unwrap(), PowerMode::Wakeup);

    #[cfg(feature = "i2c-bus")]
    {
        use rpi_zero_2w::uld::{Error, FirmwareError};
        assert!(
            matches!(
                sensor.set_power_mode(PowerMode::DeepSleep),
                Err(Error::Firmware(FirmwareError::InvalidParam))
            ),
            "DeepSleep must be rejected with InvalidParam on the VL53L5CX"
        );
        assert_eq!(sensor.get_power_mode().unwrap(), PowerMode::Wakeup);
    }
}

/// Resolution is a stopped-sensor setting: stop, change, restart.
#[test]
fn set_resolution_between_ranging_sessions() {
    let (_hw, mut sensor) = init();
    sensor.set_resolution(Resolution::Res4x4).unwrap();
    sensor.set_ranging_frequency_hz(5).unwrap();
    sensor.start_ranging().unwrap();
    sensor.stop_ranging().unwrap();

    sensor.set_resolution(Resolution::Res8x8).unwrap();
    assert_eq!(sensor.get_resolution().unwrap(), Resolution::Res8x8);

    sensor.start_ranging().unwrap();
    sensor.stop_ranging().unwrap();
}

#[test]
fn vhv_repeat_count() {
    let (_hw, mut sensor) = init();

    sensor.set_vhv_repeat_count(5).unwrap();
    assert_eq!(sensor.get_vhv_repeat_count().unwrap(), 5);

    sensor.set_vhv_repeat_count(10).unwrap();
    assert_eq!(sensor.get_vhv_repeat_count().unwrap(), 10);
}

/// Move the sensor to 0x35 and back. There is no power control on the Pololu
/// board, so the address must be restored here: if this test dies half-way
/// the sensor stays at 0x35 until it is power-cycled (unplug the Pi's 3V3).
#[test]
fn set_address_round_trip() {
    const DEFAULT: u8 = uld::DEFAULT_ADDRESS;
    const TEMP: u8 = TEMP_ADDRESS;
    let (_hw, mut sensor) = connect();

    sensor.set_i2c_address(TEMP).unwrap();
    std::thread::sleep(Duration::from_millis(10));
    assert!(sensor.is_alive().unwrap(), "sensor did not answer at {TEMP:#04x}");

    sensor.set_i2c_address(DEFAULT).unwrap();
    std::thread::sleep(Duration::from_millis(10));
    assert!(sensor.is_alive().unwrap(), "sensor did not answer back at {DEFAULT:#04x}");

    sensor.init().unwrap();
}

/// LPn is the I2C comms enable: low must make the sensor stop acknowledging,
/// high must bring it back with no re-init needed. This is also the test that
/// proves the LPn wire really is on BCM GPIO5: if LPn were wired elsewhere,
/// the sensor would keep answering with the line low.
#[cfg(feature = "i2c-bus")]
#[test]
fn lpn_gates_i2c() {
    let (_hw, mut sensor) = init();
    assert!(sensor.is_alive().unwrap());

    sensor.lpn_disable().unwrap();
    std::thread::sleep(Duration::from_millis(2));
    let gated = sensor.is_alive();
    assert!(
        !matches!(gated, Ok(true)),
        "sensor still answered with LPn low (is LPn really on BCM GPIO{LPN_GPIO}?): {gated:?}"
    );

    sensor.lpn_enable().unwrap();
    std::thread::sleep(Duration::from_millis(2));
    assert!(sensor.is_alive().unwrap(), "sensor did not come back after LPn high");

    // Firmware and configuration survive an LPn pulse (it is not a reset).
    sensor.set_resolution(Resolution::Res4x4).unwrap();
    sensor.set_ranging_frequency_hz(5).unwrap();
    sensor.start_ranging().unwrap();
    wait_ready(&mut sensor);
    let data = sensor.get_ranging_data().unwrap();
    assert_zone_distances(&data, 16);
    sensor.stop_ranging().unwrap();
}

/// L5-only: the internal charge pump can be toggled; the sensor must stay
/// responsive and keep ranging afterwards.
#[cfg(feature = "i2c-bus")]
#[test]
fn internal_cp_round_trip() {
    let (_hw, mut sensor) = init();

    sensor.disable_internal_cp().unwrap();
    assert!(sensor.is_alive().unwrap());

    sensor.enable_internal_cp().unwrap();
    assert!(sensor.is_alive().unwrap());

    sensor.set_resolution(Resolution::Res4x4).unwrap();
    sensor.set_ranging_frequency_hz(5).unwrap();
    sensor.start_ranging().unwrap();
    wait_ready(&mut sensor);
    let data = sensor.get_ranging_data().unwrap();
    assert_zone_distances(&data, 16);
    sensor.stop_ranging().unwrap();
}
