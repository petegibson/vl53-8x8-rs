//! Hardware-in-the-loop tests for the pure-Rust `vl53-8x8` driver on the Pi:
//! VL53L8CX over SPI0 by default, VL53L5CX over I2C1 with `--features
//! i2c-bus` (`rpi_zero_2w::native` picks the bus and LPn pin; the driver
//! autodetects the part).
//!
//! Run on the Pi with `cargo test --test sensor_async [--features i2c-bus]`,
//! or from the dev shell with `just test-pi [i2c-bus]` (musl cross-build, run
//! over ssh). The tests share one sensor, so they take a process-wide lock
//! and the remote runner also sets `RUST_TEST_THREADS=1`.
//!
//! Ported from `examples/evo-f412re-embassy/tests/sensor_async.rs` without
//! the power-cycle tests (no power-enable pin on the Pololu boards) and with
//! an LPn gating test on I2C. Scene: whatever is in front of the sensor, so
//! distances are only bounded, not matched.

use std::sync::{Mutex, MutexGuard};
use std::time::{Duration, Instant};

use rpi_zero_2w::native::{
    self, Driver, PowerMode, RangingMode, Resolution, ResultsData, TargetOrder, Variant,
    EXPECTED_VARIANT, NB_TARGET_PER_ZONE,
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
    let sensor = native::connect().expect("open the bus and LPn (device permissions? wiring?)");
    (guard, sensor)
}

/// Address used by `set_address_round_trip`.
const TEMP_ADDRESS: u8 = 0x35;

/// [`connect`] followed by `init()` (autodetect + firmware upload).
async fn init() -> (MutexGuard<'static, ()>, Driver) {
    let (guard, mut sensor) = connect();
    sensor.init().await.expect("init (firmware upload)");
    (guard, sensor)
}

/// Wait for a frame, bounded so a dead sensor fails instead of hanging.
async fn wait_ready(sensor: &mut Driver) {
    tokio::time::timeout(Duration::from_secs(3), sensor.poll_data_ready())
        .await
        .expect("timed out waiting for a frame")
        .expect("poll_data_ready");
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

#[tokio::test(flavor = "current_thread")]
async fn sensor_alive() {
    let (_hw, mut sensor) = connect();
    assert!(
        sensor.is_alive().await.expect("is_alive bus transaction"),
        "sensor.is_alive returned false: check wiring, the bus-mode selector and LPn on BCM GPIO{LPN_GPIO}"
    );
}

/// The strong test: autodetect, patched-firmware upload and the variant's
/// init quirks all over this bus, ending on the part the wiring says it is.
#[tokio::test(flavor = "current_thread")]
async fn init_detects_expected_variant() {
    let (_hw, mut sensor) = connect();
    let started = Instant::now();
    sensor.init().await.expect("init");
    eprintln!(
        "init (firmware upload) took {:.2} s, detected {:?}",
        started.elapsed().as_secs_f32(),
        sensor.variant()
    );
    assert_eq!(sensor.variant(), EXPECTED_VARIANT);
    assert!(sensor.is_alive().await.unwrap());
}

#[tokio::test(flavor = "current_thread")]
async fn set_resolution() {
    let (_hw, mut sensor) = init().await;

    sensor.set_resolution(Resolution::Res4x4).await.unwrap();
    assert_eq!(sensor.get_resolution().await.unwrap(), Resolution::Res4x4);

    sensor.set_resolution(Resolution::Res8x8).await.unwrap();
    assert_eq!(sensor.get_resolution().await.unwrap(), Resolution::Res8x8);
}

#[tokio::test(flavor = "current_thread")]
async fn ranging_4x4() {
    let (_hw, mut sensor) = init().await;
    sensor.set_resolution(Resolution::Res4x4).await.unwrap();
    sensor.set_ranging_frequency_hz(5).await.unwrap();
    sensor.start_ranging().await.unwrap();

    wait_ready(&mut sensor).await;
    let data = sensor.get_ranging_data().await.unwrap();
    assert_zone_distances(&data, 16);

    sensor.stop_ranging().await.unwrap();
}

#[tokio::test(flavor = "current_thread")]
async fn ranging_8x8() {
    let (_hw, mut sensor) = init().await;
    sensor.set_resolution(Resolution::Res8x8).await.unwrap();
    sensor.set_ranging_frequency_hz(5).await.unwrap();
    sensor.start_ranging().await.unwrap();

    wait_ready(&mut sensor).await;
    let data = sensor.get_ranging_data().await.unwrap();
    assert_zone_distances(&data, 64);

    sensor.stop_ranging().await.unwrap();
}

/// Several consecutive frames must all parse and stay plausible.
#[tokio::test(flavor = "current_thread")]
async fn ranging_streams_frames() {
    const FRAMES: usize = 10;
    let (_hw, mut sensor) = init().await;
    sensor.set_resolution(Resolution::Res4x4).await.unwrap();
    sensor.set_ranging_frequency_hz(10).await.unwrap();
    sensor.start_ranging().await.unwrap();

    let started = Instant::now();
    for _ in 0..FRAMES {
        wait_ready(&mut sensor).await;
        let data = sensor.get_ranging_data().await.unwrap();
        assert_zone_distances(&data, 16);
    }
    let elapsed = started.elapsed();
    eprintln!("{FRAMES} frames at 10 Hz in {:.2} s", elapsed.as_secs_f32());
    assert!(elapsed < Duration::from_secs(3), "frames arrived far slower than 10 Hz");

    sensor.stop_ranging().await.unwrap();
}

#[tokio::test(flavor = "current_thread")]
async fn ambient_light() {
    let (_hw, mut sensor) = init().await;
    sensor.set_resolution(Resolution::Res4x4).await.unwrap();
    sensor.set_ranging_frequency_hz(5).await.unwrap();
    sensor.start_ranging().await.unwrap();

    wait_ready(&mut sensor).await;
    let data = sensor.get_ranging_data().await.unwrap();
    // Per-zone ambient magnitude is lighting/variant dependent (and rounds to
    // 0 after scaling in low light), so only require that the block
    // transferred and at least one zone reports light.
    let nonzero = data.ambient_per_spad[..16].iter().filter(|&&a| a > 0).count();
    eprintln!("ambient: {nonzero}/16 zones non-zero");
    assert!(nonzero > 0, "no zone reported ambient light (sensor in the dark?)");

    sensor.stop_ranging().await.unwrap();
}

#[tokio::test(flavor = "current_thread")]
async fn ranging_frequency() {
    let (_hw, mut sensor) = init().await;

    for hz in [5u8, 15, 30] {
        sensor.set_ranging_frequency_hz(hz).await.unwrap();
        assert_eq!(sensor.get_ranging_frequency_hz().await.unwrap(), hz);
    }
}

#[tokio::test(flavor = "current_thread")]
async fn integration_time() {
    let (_hw, mut sensor) = init().await;

    sensor.set_integration_time_ms(20).await.unwrap();
    assert_eq!(sensor.get_integration_time_ms().await.unwrap(), 20);

    sensor.set_integration_time_ms(50).await.unwrap();
    assert_eq!(sensor.get_integration_time_ms().await.unwrap(), 50);
}

#[tokio::test(flavor = "current_thread")]
async fn target_order() {
    let (_hw, mut sensor) = init().await;

    sensor.set_target_order(TargetOrder::Closest).await.unwrap();
    assert_eq!(sensor.get_target_order().await.unwrap(), TargetOrder::Closest);

    sensor.set_target_order(TargetOrder::Strongest).await.unwrap();
    assert_eq!(sensor.get_target_order().await.unwrap(), TargetOrder::Strongest);
}

#[tokio::test(flavor = "current_thread")]
async fn ranging_mode() {
    let (_hw, mut sensor) = init().await;

    sensor.set_ranging_mode(RangingMode::Continuous).await.unwrap();
    assert_eq!(sensor.get_ranging_mode().await.unwrap(), RangingMode::Continuous);

    sensor.set_ranging_mode(RangingMode::Autonomous).await.unwrap();
    assert_eq!(sensor.get_ranging_mode().await.unwrap(), RangingMode::Autonomous);
}

#[tokio::test(flavor = "current_thread")]
async fn sharpener_percent() {
    let (_hw, mut sensor) = init().await;

    // u8 quantisation: read-back may differ by +-1 from the written value.
    sensor.set_sharpener_percent(25).await.unwrap();
    let v = sensor.get_sharpener_percent().await.unwrap();
    assert!((24..=26).contains(&v), "expected ~25, got {v}");

    sensor.set_sharpener_percent(75).await.unwrap();
    let v = sensor.get_sharpener_percent().await.unwrap();
    assert!((74..=76).contains(&v), "expected ~75, got {v}");
}

/// Sleep/Wakeup round trip. Deep sleep exists only on the VL53L8CX; the
/// driver rejects it on an L5 without touching the bus.
#[tokio::test(flavor = "current_thread")]
async fn power_mode() {
    let (_hw, mut sensor) = init().await;

    assert_eq!(sensor.get_power_mode().await.unwrap(), PowerMode::Wakeup);

    sensor.set_power_mode(PowerMode::Sleep).await.unwrap();
    assert_eq!(sensor.get_power_mode().await.unwrap(), PowerMode::Sleep);

    sensor.set_power_mode(PowerMode::Wakeup).await.unwrap();
    assert_eq!(sensor.get_power_mode().await.unwrap(), PowerMode::Wakeup);

    if sensor.variant() == Variant::L5 {
        assert!(
            sensor.set_power_mode(PowerMode::DeepSleep).await.is_err(),
            "DeepSleep must be rejected on the VL53L5CX"
        );
        assert_eq!(sensor.get_power_mode().await.unwrap(), PowerMode::Wakeup);
    }
}

/// Resolution is a stopped-sensor setting: stop, change, restart.
#[tokio::test(flavor = "current_thread")]
async fn set_resolution_between_ranging_sessions() {
    let (_hw, mut sensor) = init().await;
    sensor.set_resolution(Resolution::Res4x4).await.unwrap();
    sensor.set_ranging_frequency_hz(5).await.unwrap();
    sensor.start_ranging().await.unwrap();
    sensor.stop_ranging().await.unwrap();

    sensor.set_resolution(Resolution::Res8x8).await.unwrap();
    assert_eq!(sensor.get_resolution().await.unwrap(), Resolution::Res8x8);

    sensor.start_ranging().await.unwrap();
    sensor.stop_ranging().await.unwrap();
}

#[tokio::test(flavor = "current_thread")]
async fn vhv_repeat_count() {
    let (_hw, mut sensor) = init().await;

    sensor.set_vhv_repeat_count(5).await.unwrap();
    assert_eq!(sensor.get_vhv_repeat_count().await.unwrap(), 5);

    sensor.set_vhv_repeat_count(10).await.unwrap();
    assert_eq!(sensor.get_vhv_repeat_count().await.unwrap(), 10);
}

/// Move the sensor to 0x35 and back. No power control on the Pololu board,
/// so the address is restored here; if this test dies half-way the sensor
/// stays at 0x35 until it is power-cycled.
#[tokio::test(flavor = "current_thread")]
async fn set_address_round_trip() {
    const DEFAULT: u8 = native::DEFAULT_ADDRESS;
    const TEMP: u8 = TEMP_ADDRESS;
    let (_hw, mut sensor) = connect();

    sensor.set_i2c_address(TEMP).await.unwrap();
    tokio::time::sleep(Duration::from_millis(10)).await;
    assert!(sensor.is_alive().await.unwrap(), "sensor did not answer at {TEMP:#04x}");

    sensor.set_i2c_address(DEFAULT).await.unwrap();
    tokio::time::sleep(Duration::from_millis(10)).await;
    assert!(sensor.is_alive().await.unwrap(), "sensor did not answer back at {DEFAULT:#04x}");

    sensor.init().await.unwrap();
}

/// LPn is the I2C comms enable: low must make the sensor stop acknowledging,
/// high must bring it back with no re-init. Also the proof that the LPn wire
/// is on BCM GPIO5: wired anywhere else, the sensor would keep answering.
#[cfg(feature = "i2c-bus")]
#[tokio::test(flavor = "current_thread")]
async fn lpn_gates_i2c() {
    let (_hw, mut sensor) = init().await;
    assert!(sensor.is_alive().await.unwrap());

    sensor.lpn_disable().unwrap();
    tokio::time::sleep(Duration::from_millis(2)).await;
    let gated = sensor.is_alive().await;
    assert!(
        !matches!(gated, Ok(true)),
        "sensor still answered with LPn low (is LPn really on BCM GPIO{LPN_GPIO}?): {gated:?}"
    );

    sensor.lpn_enable().unwrap();
    tokio::time::sleep(Duration::from_millis(2)).await;
    assert!(sensor.is_alive().await.unwrap(), "sensor did not come back after LPn high");

    // Firmware and configuration survive an LPn pulse (it is not a reset).
    sensor.set_resolution(Resolution::Res4x4).await.unwrap();
    sensor.set_ranging_frequency_hz(5).await.unwrap();
    sensor.start_ranging().await.unwrap();
    wait_ready(&mut sensor).await;
    let data = sensor.get_ranging_data().await.unwrap();
    assert_zone_distances(&data, 16);
    sensor.stop_ranging().await.unwrap();
}
