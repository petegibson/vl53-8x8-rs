# Known divergences between ST's ULDs and `vl53-8x8`

Each entry is a differential check that is `#[ignore]`d because the two drivers
put different bytes on the bus for the same API calls:
`<test>::b_rust_driver_matches_c_recording` in `tests/differential.rs`
(VL53L8CX, `vl53l8cx-uld` vs `vl53-8x8`, C hardware recording) and
`<test>::d_c_l5_driver_matches_rust_recording` /
`<test>::d2_c_l5_driver_matches_rust_recording_with_writes_coalesced` in
`tests/differential_l5.rs` (VL53L5CX, `vl53l5cx-uld` vs `vl53-8x8`, Rust
hardware recording). The comparison was not loosened to hide them; run
`cargo test -p differential-tests -- --include-ignored` to see the current
diffs. No driver has been changed by the differential test work; what to do
about each entry is a separate decision. Entries 2-5 are between ST's VL53L5CX
ULD and the Rust driver's L5 path; op indices refer to the normalised
transcripts the failure messages print.

## 1. `test_detection_thresholds`: detection thresholds were scaled twice by `vl53l8cx-uld` — FIXED

**Status: fixed** (same day). The wrapper's `scale_thresholds_for_write/read`
were removed from `vl53l8cx-uld` and the freshly ported `vl53l5cx-uld`; ST's
ULD does the scaling. The C recording was re-taken on hardware with the fixed
wrapper and the differential check is no longer ignored. The original
analysis is kept below for the record.

### Original report

**Call**: `set_detection_thresholds` with zone 0 `{ measurement: DistanceMm,
threshold_type: LessThanEqualMin, low_threshold: 500, high_threshold: 0 }`,
all other entries zero, then `get_detection_thresholds`.

**Bus difference** (op 90 of 95 after normalisation, the DCI write of the 64
threshold structs to `VL53L8CX_DCI_DET_THRESH_START`, 780 bytes at register
`0x2cf4`; bytes 4..8 are the big-endian `low_threshold` of zone 0):

| side | bytes 0..16 of the write | `low_threshold` on the wire |
|---|---|---|
| `vl53l8cx-uld` (C recording and C driver on host) | `b6 e8 30 00 00 00 1f 40 00 00 00 00 00 00 02 01` | `0x1f40` = 8000 = 500 × 16 |
| `vl53-8x8` (host and hardware recording)            | `b6 e8 30 00 00 00 07 d0 00 00 00 00 00 00 02 01` | `0x07d0` = 2000 = 500 × 4 |

Every other op of the sequence is identical, including the DCI read back of the
same 780 bytes (the sensor returns what was written).

**Cause**: ST's ULD scales thresholds itself, in
`vl53l8cx_plugin_detection_thresholds.c`: `vl53l8cx_set_detection_thresholds`
multiplies `param_low_thresh`/`param_high_thresh` by 4 for `DISTANCE_MM`
(2048 for the per-SPAD kcps measurements, 128 for range sigma, 256 for SPAD
count, 65535 for the motion indicator) and `vl53l8cx_get_detection_thresholds`
divides by the same factor. The wrapper `vl53l8cx/src/lib.rs` applies the same
table again (`scale_thresholds_for_write` before calling the C function,
`scale_thresholds_for_read` after), so the value reaching the sensor is scaled
twice. The pure-Rust driver (`vl53-8x8/src/lib.rs`, same helper names) scales
once, matching what ST's C code sends when called directly.

**Why the on-target tests pass on both**: set and get are symmetric on each
side, so a round trip returns the input. The difference is only visible on the
wire, or when a threshold written by one driver is read by the other (the Rust
driver reading the C recording decodes 2000, which is what fails the host
assertion before the bus comparison runs).

**Impact**: with `vl53l8cx-uld` a 500 mm threshold is armed at 8000 (sensor
units of 1/4 mm, so 2000 mm). For `SignalPerSpadKcps`/`AmbientPerSpadKcps` the
double factor is 2048 × 2048 = 4 194 304, which overflows `i32` for inputs above
511 kcps/SPAD; `MotionIndicator` overflows for any input above 0 (65535² >
2³¹). The firmware's threshold semantics are documented in ST's UM3109 (section
on detection thresholds) in the units `vl53-8x8` sends.

## 2. Every L5 sequence with `init`: the firmware bank holding the L5 patch is uploaded in three writes by `vl53-8x8`, in one by ST's ULD

**Status: open.** Checks ignored: `d_c_l5_driver_matches_rust_recording` of
the 13 evo sequences that call `init` (all but `async_i2c_alive`). The bytes
that reach the sensor are identical: `l5_firmware_upload_differs_from_l8_at_exactly_the_patched_offsets`
(not ignored) assembles the image from the bank writes of the Rust hardware
recording and of ST's L5 code on the host, finds them equal, and finds them
different from the L8 image uploaded in the C L8 recording at exactly the
seven offsets of `L5_FW_PATCH`. Only the transfer boundaries differ.

**Bus difference** (op 55 of `init_uploads_firmware_over_async_i2c` after
normalisation, right after the page select `W 0x7fff [0a]`):

| side | writes for firmware bank 2 (image offsets `0x8000..0x10000`) |
|---|---|
| `vl53-8x8` (hardware recording and host) | `W 0x0000 len=19265` = `firmware_l5[0x8000..0xCB41]`, `W 0x4b41 len=64` = `firmware_l5[0xCB41..0xCB81]`, `W 0x4b81 len=13439` = `firmware_l5[0xCB81..0x10000]` |
| `vl53l5cx-uld` (host) | `W 0x0000 len=32768` = `firmware_l5[0x8000..0x10000]` |

Banks 1 and 3 (`W 0x7fff [09]` / `[0b]`) are one write of 32768 / 20480 bytes
on both sides. The 64-byte window starts at `0xCB41`, the first patched byte.

**Cause**: `vl53-8x8/src/ops.rs` lines 640-650 (the `FW_BANKS` loop) write
whatever `FirmwareSource::chunk` returns, and `PatchedFirmware::chunk`
(`vl53-8x8/src/firmware.rs` lines 100-125) returns the clean prefix up to the
first patched byte, then a `PATCH_SCRATCH` (= 64) byte window with the overlay
applied, then the clean rest, so that the 86 KB image is never copied. ST
uploads each bank with a single `VL53L5CX_WrMulti` (`vl53l5cx_api.c` lines
322-329). On the L8 the patch table is empty, each bank is one write and the
L8 checks are exact.

**Impact**: three I2C (or SPI) transactions instead of one for that bank; the
sensor's address auto-increment makes the result the same. The `d2` checks,
which merge register-contiguous writes on both sides before comparing, pass
for 10 of the 13 sequences; the other three are entries 3-5.

## 3. `power_mode_sleep_wakeup`: `vl53-8x8` reads `0x000F` on wake-up, ST's L5 ULD does not

**Status: open.** Check ignored: `d2` of `power_mode_sleep_wakeup`.

**Bus difference** (op 114 after normalisation and write coalescing, inside
`set_power_mode(Wakeup)` after `W 0x7fff [00]`, `W 0x0009 [04]`):

| side | next ops |
|---|---|
| `vl53-8x8` (hardware recording and host) | `R 0x000f len=1 [40]`, then the `R 0x0006` poll (`[00]`, `[01]`) |
| `vl53l5cx-uld` (host) | the `R 0x0006` poll directly |

Everything else in the sequence is identical, including sleep (`W 0x0009 [02]`,
poll `0x06 & 0x01 == 0`) and the `get_power_mode` reads of `0x0009`.

**Cause**: `vl53-8x8/src/lib.rs` line 589 reads `0x000F` on every wake-up to
tell deep sleep (`0x43`) from sleep and re-initialise after deep sleep (lines
590-605); this is the L8 ULD's `vl53l8cx_set_power_mode` behaviour applied to
both variants. `vl53l5cx_set_power_mode` (`vl53l5cx_api.c` lines 462-466) has
no deep-sleep mode and goes from `W 0x09 [04]` straight to the `0x06` poll.

**Impact**: one extra one-byte read; harmless. The read could be gated on
`SensorInfo::has_deep_sleep`.

## 4. `calibrate_xtalk_leaves_sensor_usable`: `calibrate_xtalk` reads the resolution last, ST's ULD reads it first

**Status: open.** Check ignored: `d2` of `calibrate_xtalk_leaves_sensor_usable`.

**Bus difference** (op 115 after normalisation and write coalescing, the first
op of `calibrate_xtalk(5, 4, 600)`; each configuration read is a DCI read:
`W 0x2ff4` request, `R 0x2c00` poll, `R 0x2c04` data):

| side | configuration reads, in order (DCI index, length of the `0x2c04` read) |
|---|---|
| `vl53-8x8` (hardware recording and host) | frequency `0x5458` (16), integration time `0x545C` (32), sharpener `0xAED8` (28), target order `0xAE64` (16), xtalk margin `0xAD94` (28), ranging mode `0xAD30` (20), **resolution `0x5450` (20)** |
| `vl53l5cx-uld` (host) | **resolution `0x5450` (20)**, then the six others in the same order |

At op 115 the recording has `W 0x2ff4 [54 58 00 40 00 00 00 0f 00 02 00 08]`,
ST's code writes `W 0x2ff4 [54 50 00 80 00 00 00 0f 00 02 00 08]`.

**Cause**: `vl53-8x8/src/lib.rs` lines 1124-1129 read the six settings and
`ops::calibrate_xtalk` (`vl53-8x8/src/ops.rs` line 1567) reads the resolution
afterwards; `vl53l5cx_calibrate_xtalk` reads the resolution first
(`vl53l5cx_plugin_xtalk.c` lines 165-172). The L8 ULD's
`vl53l8cx_calibrate_xtalk` has the same order as the L5 one, so this is not
L5-specific; the L8 C suite has no xtalk calibration test, which is why the L8
checks never saw it.

Because the resolution and the ranging mode are both 20-byte reads at
`(0x2c04, 20)`, the register model hands ST's code the two answers swapped,
and ST's code ends with `InvalidParam` when it "restores" resolution 3 at the
end. That is an artefact of the model's `(register, length)` keying, not a
fault of ST's code. The diagnostic
`calibrate_xtalk_matches_st_apart_from_the_configuration_read_order` (not
ignored, passes) replays ST's code against the recording with the resolution
read moved to ST's position: the remaining ~290 ops, i.e. the L5-patched
984-byte command table at `0x2c28`, the calibration configuration, the output
configuration, the start command, the completion poll, the xtalk data fetch,
the default configuration reset and the restore of the resolution and the six
settings, are identical.

**Impact**: order of seven reads only.

## 5. `calibrate_xtalk_rejects_invalid_params`: ST's ULD talks to the sensor for rejected arguments, `vl53-8x8` does not

**Status: open.** Check ignored: `d2` of `calibrate_xtalk_rejects_invalid_params`.

**Bus difference**: for each of the six invalid calls `(0, 4, 600)`,
`(100, 4, 600)`, `(5, 0, 600)`, `(5, 17, 600)`, `(5, 4, 599)`, `(5, 4, 3001)`
the Rust driver returns `InvalidParam` without a bus transfer (`vl53-8x8/src/lib.rs`
line 1119, `ops::xtalk_params_valid`); its recording holds `init`,
`set_resolution(Res8x8)` and the final `get_resolution`, 123 ops. ST's code
emits about 57 ops per call (463 in total on the host). After the recording's
op 112 (`R 0x2c04 len=20`, the `set_resolution` read-back) it continues with
the seven configuration reads of entry 4 (`W 0x2ff4 [54 50 ...]`,
`[54 58 ...]`, `[54 5c ...]`, `[ae d8 ...]`, `[ae 64 ...]`, `[ad 94 ...]`,
`[ad 30 ...]`), then `W 0x2fb8 len=72` (`VL53L5CX_GET_XTALK_CMD`), poll,
`R 0x2c04 len=780` (the xtalk buffer read back into `p_dev->xtalk_data`), then
`W 0x2c34 len=972` (`default_configuration_l5`), poll, then `set_resolution`
and the six setters with the values just read.

**Cause**: `vl53l5cx_calibrate_xtalk` reads the configuration before it checks
the arguments (`vl53l5cx_plugin_xtalk.c` lines 165-179), and the `if/else`
skips only the calibration itself: the xtalk read-back (lines 256-269), the
default configuration reset (lines 272-276) and the restore of the seven
settings (lines 278-285) run for invalid arguments too. The L8 ULD is
structured the same way.

**Unknown reads**: 30 reads hit `(reg, len)` keys the recording never had and
were answered with zeros, five per call: frequency `(0x2c04, 16)`, integration
time `(0x2c04, 32)`, target order `(0x2c04, 16)`, xtalk read-back
`(0x2c04, 780)`, and the `(0x2c04, 16)` read of the frequency re-apply. The
zero answers make ST's own setters reject the integration time, target order
and xtalk margin they read back, so the exact tail of the host transcript is
an artefact; the structure above is not.

**Impact**: with ST's code an invalid call costs some 60 bus transactions,
overwrites `xtalk_data` with the sensor's current table and re-sends the
default configuration; both drivers return `InvalidParam`. The Rust driver
leaves the sensor untouched, which its on-target test asserts.
