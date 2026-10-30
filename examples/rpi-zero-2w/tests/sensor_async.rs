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
    EXPECTED_VARIANTS, NB_TARGET_PER_ZONE,
};
use rpi_zero_2w::lpn_gpio;

/// One sensor, one test at a time.
static HW: Mutex<()> = Mutex::new(());

/// Integration time for the ranging tests. The ULD default of 5 ms leaves
/// most 8x8 zones of a VL53L7CX (wider lens, about half the per-zone signal
/// of an L5) below the signal threshold on a 1.5 m ceiling; 20 ms fits the
/// 5 Hz and 10 Hz frame periods used here with room to spare.
const INTEGRATION_MS: u32 = 20;

/// Open the bus and LPn (LPn high) without initialising the sensor.
fn connect() -> (MutexGuard<'static, ()>, Driver) {
    let guard = HW.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
    // connect() also moves a sensor back from a stray address (see
    // rpi_zero_2w::STRAY_ADDRESSES) if an earlier run died half-way.
    let sensor = native::connect().expect("open the bus and LPn (device permissions? wiring?)");
    (guard, sensor)
}

/// Address used by `set_address_round_trip`; listed in `STRAY_ADDRESSES`.
const TEMP_ADDRESS: u8 = 0x35;

/// Two boards on I2C1 at once (`i2c-bus` only): silence both, enable the
/// second (GPIO6), move it to `SECOND_ADDRESS`, enable the first (GPIO5),
/// init both and take a frame from each. Skips (passes) when nothing answers
/// on the second LPn line, so a single-board rig still runs the suite.
/// Restores the second board's address with the first silenced again.
#[cfg(feature = "i2c-bus")]
#[tokio::test(flavor = "current_thread")]
async fn two_sensors_one_bus() {
    use rpi_zero_2w::{open_all_lpn, open_i2c, LPN_HOLD_LOW, SECOND_ADDRESS};
    use vl53_8x8::{BlockingI2c, Sensor, DEFAULT_ADDRESS};
    let _hw = HW.lock().unwrap_or_else(|poisoned| poisoned.into_inner());

    // Every LPn claimed low: both boards silent.
    let mut lpns = open_all_lpn().expect("claim LPn lines");
    assert!(lpns.len() >= 2, "need two LPn lines for this test");
    let (gpio_a, lpn_a) = lpns.remove(0);
    let (gpio_b, lpn_b) = lpns.remove(0);
    std::thread::sleep(LPN_HOLD_LOW);

    // Second board first: enable it alone and move it off the default address.
    let mut b = Sensor::new(BlockingI2c(open_i2c().unwrap()), rpi_zero_2w::TokioDelay).with_lpn_pin(lpn_b);
    b.lpn_enable().unwrap();
    std::thread::sleep(LPN_HOLD_LOW);
    match b.is_alive().await {
        Ok(true) => {}
        other => {
            eprintln!("no second board on LPn GPIO{gpio_b} ({other:?}); skipping two-sensor test");
            return;
        }
    }
    b.set_i2c_address(SECOND_ADDRESS).await.unwrap();
    assert!(b.is_alive().await.unwrap(), "second board not answering at {SECOND_ADDRESS:#04x}");

    // Now the first board can come up at the default address.
    let mut a = Sensor::new(BlockingI2c(open_i2c().unwrap()), rpi_zero_2w::TokioDelay).with_lpn_pin(lpn_a);
    a.lpn_enable().unwrap();
    std::thread::sleep(LPN_HOLD_LOW);
    assert!(a.is_alive().await.unwrap(), "first board (GPIO{gpio_a}) not answering at the default address");

    a.init().await.unwrap();
    b.init().await.unwrap();
    eprintln!("GPIO{gpio_a} @ {DEFAULT_ADDRESS:#04x}: {:?}; GPIO{gpio_b} @ {SECOND_ADDRESS:#04x}: {:?}", a.variant(), b.variant());

    for s in [&mut a, &mut b] {
        s.set_resolution(Resolution::Res4x4).await.unwrap();
        s.set_ranging_frequency_hz(5).await.unwrap();
        s.set_integration_time_ms(INTEGRATION_MS).await.unwrap();
        s.start_ranging().await.unwrap();
    }
    for s in [&mut a, &mut b] {
        wait_ready(s).await;
        let data = s.get_ranging_data().await.unwrap();
        assert_zone_distances(&data, 16);
        s.stop_ranging().await.unwrap();
    }

    // Put the second board back at the default address, with the first silent.
    a.lpn_disable().unwrap();
    b.set_i2c_address(DEFAULT_ADDRESS).await.unwrap();
    assert!(b.is_alive().await.unwrap(), "second board did not return to the default address");
}

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
/// least half of the first `zones` zones are valid (see [`INTEGRATION_MS`]
/// for why not more). Zones looking at nothing (a corner past the ceiling, a
/// dark absorber) legitimately report status 255 with distance 0. Per-target
/// arrays are laid out `zone * NB_TARGET_PER_ZONE + target`.
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
    assert!(valid * 2 >= zones, "only {valid}/{zones} zones have a valid target");
}

#[tokio::test(flavor = "current_thread")]
async fn sensor_alive() {
    let (_hw, mut sensor) = connect();
    assert!(
        sensor.is_alive().await.expect("is_alive bus transaction"),
        "sensor.is_alive returned false: check wiring, the bus-mode selector and LPn on BCM GPIO{}", lpn_gpio()
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
    assert!(
        EXPECTED_VARIANTS.contains(&sensor.variant()),
        "detected {:?}, expected one of {EXPECTED_VARIANTS:?}",
        sensor.variant()
    );
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
    sensor.set_integration_time_ms(INTEGRATION_MS).await.unwrap();
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
    sensor.set_integration_time_ms(INTEGRATION_MS).await.unwrap();
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
    sensor.set_integration_time_ms(INTEGRATION_MS).await.unwrap();
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
    sensor.set_integration_time_ms(INTEGRATION_MS).await.unwrap();
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

    if sensor.variant() != Variant::L8 {
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
    sensor.set_integration_time_ms(INTEGRATION_MS).await.unwrap();
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
        "sensor still answered with LPn low (is LPn really on BCM GPIO{}?): {gated:?}", lpn_gpio()
    );

    sensor.lpn_enable().unwrap();
    tokio::time::sleep(Duration::from_millis(2)).await;
    assert!(sensor.is_alive().await.unwrap(), "sensor did not come back after LPn high");

    // Firmware and configuration survive an LPn pulse (it is not a reset).
    sensor.set_resolution(Resolution::Res4x4).await.unwrap();
    sensor.set_ranging_frequency_hz(5).await.unwrap();
    sensor.set_integration_time_ms(INTEGRATION_MS).await.unwrap();
    sensor.start_ranging().await.unwrap();
    wait_ready(&mut sensor).await;
    let data = sensor.get_ranging_data().await.unwrap();
    assert_zone_distances(&data, 16);
    sensor.stop_ranging().await.unwrap();
}

/// Report (never fails): dump the firmware's DCI block at 0xE0C4. The
/// VL53L8CX ULD reads 8 bytes there and treats byte 6 as the laser-safety
/// fault flag; ST's VL53LMZ ULD (VL53L7CH/VL53L8CH) names 0xE0C5, byte 1 of
/// the same block, `MODULE_TYPE_IDX`. Printed for each board so the values
/// can be compared across L5, L7 and L8. Run with `just report-pi-dci
/// [i2c-bus]`.
#[ignore]
#[tokio::test(flavor = "current_thread")]
async fn dci_probe_report() {
    let (_hw, mut sensor) = init().await;
    let mut block = [0u8; 8];
    sensor.dci_read(0xE0C4, &mut block).await.unwrap();
    let hex: Vec<String> = block.iter().map(|b| format!("{b:02x}")).collect();
    eprintln!("\nvariant: {:?}, DCI 0xE0C4..0xE0CC: {}", sensor.variant(), hex.join(" "));
    eprintln!("  byte 1 (LMZ MODULE_TYPE_IDX 0xE0C5) = {:#04x}, byte 6 (L8 laser-safety flag) = {:#04x}", block[1], block[6]);
}

/// Report (never fails): hex-dump the sensor's 492-byte factory NVM block and
/// decode the per-zone offset calibration ST's init sends back (8x8 signal
/// grid and range offsets). Reads it twice to show it is stable. Capture the
/// output per module under `nvm/` to compare parts. Run with
/// `just report-pi-nvm [i2c-bus]`.
#[ignore]
#[tokio::test(flavor = "current_thread")]
async fn nvm_report() {
    let (_hw, mut sensor) = init().await;
    let mut buf = [0u8; native::NVM_DATA_SIZE];
    let mut again = [0u8; native::NVM_DATA_SIZE];
    sensor.read_nvm_data(&mut buf).await.unwrap();
    sensor.read_nvm_data(&mut again).await.unwrap();
    // Leading newline: with --nocapture libtest has already printed
    // "test nvm_report ... " without a line break.
    eprintln!(
        "\nvariant: {:?}, nvm {} bytes, second read identical: {}",
        sensor.variant(),
        buf.len(),
        buf == again
    );
    for (i, chunk) in buf.chunks(16).enumerate() {
        let hex: Vec<String> = chunk.iter().map(|b| format!("{b:02x}")).collect();
        eprintln!("{:04x}: {}", i * 16, hex.join(" "));
    }
    // Decode as send_offset_data does: byte-swap each 32-bit word of the first 488 bytes.
    let mut sw = buf[..488].to_vec();
    for w in sw.chunks_mut(4) {
        w.reverse();
    }
    eprintln!("range offsets (i16, 8x8, word-swapped block @0x140):");
    for row in 0..8 {
        let v: Vec<String> = (0..8)
            .map(|c| {
                let o = 0x140 + (row * 8 + c) * 2;
                format!("{:>6}", i16::from_le_bytes([sw[o], sw[o + 1]]))
            })
            .collect();
        eprintln!("  {}", v.join(" "));
    }
    eprintln!("signal grid (u32, 8x8, word-swapped block @0x3C):");
    for row in 0..8 {
        let v: Vec<String> = (0..8)
            .map(|c| {
                let o = 0x3C + (row * 8 + c) * 4;
                format!("{:>8}", u32::from_le_bytes([sw[o], sw[o + 1], sw[o + 2], sw[o + 3]]))
            })
            .collect();
        eprintln!("  {}", v.join(" "));
    }
}

/// Report (never fails): print one distance grid and one target-status grid
/// at 4x4 and at 8x8 with the default 5 ms integration and with 20 ms, so a
/// scene or a new part can be eyeballed. A flat ceiling should read flat
/// across the grid (the firmware reports plane-perpendicular distance), and
/// the valid-zone count shows how much signal the lens gathers. Run with
/// `just report-pi-scene [i2c-bus]`.
#[ignore]
#[tokio::test(flavor = "current_thread")]
async fn scene_report() {
    let (_hw, mut sensor) = init().await;
    eprintln!("variant: {:?}", sensor.variant());
    for (res, grid, integration_ms) in [
        (Resolution::Res4x4, 4usize, 5u32),
        (Resolution::Res8x8, 8, 5),
        (Resolution::Res8x8, 8, 20),
    ] {
        sensor.set_resolution(res).await.unwrap();
        sensor.set_ranging_frequency_hz(5).await.unwrap();
        sensor.set_integration_time_ms(integration_ms).await.unwrap();
        sensor.start_ranging().await.unwrap();
        // The first frame after start carries status 6 (no wrap-around check).
        wait_ready(&mut sensor).await;
        let _ = sensor.get_ranging_data().await.unwrap();
        wait_ready(&mut sensor).await;
        let data = sensor.get_ranging_data().await.unwrap();
        sensor.stop_ranging().await.unwrap();

        let zones = grid * grid;
        let valid = (0..zones)
            .filter(|&z| matches!(data.target_status[z * NB_TARGET_PER_ZONE], 5 | 6 | 9))
            .count();
        eprintln!(
            "--- {grid}x{grid}, integration {integration_ms} ms: {valid}/{zones} zones valid \
             (distance mm | status | ambient kcps/spad | signal kcps/spad) ---"
        );
        for row in 0..grid {
            let cell = |f: &dyn Fn(usize, usize) -> String| -> String {
                (0..grid).map(|c| f(row * grid + c, (row * grid + c) * NB_TARGET_PER_ZONE)).collect::<Vec<_>>().join(" ")
            };
            eprintln!(
                "{}   |{}   |{}   |{}",
                cell(&|_, t| format!("{:>5}", data.distance_mm[t])),
                cell(&|_, t| format!("{:>3}", data.target_status[t])),
                cell(&|z, _| format!("{:>4}", data.ambient_per_spad[z])),
                cell(&|_, t| format!("{:>5}", data.signal_per_spad[t])),
            );
        }
    }
}
