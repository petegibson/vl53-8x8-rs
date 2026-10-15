# Known divergences between `vl53l8cx-uld` and `vl53-8x8`

Each entry is a differential check (`<test>::b_rust_driver_matches_c_recording`
in `tests/differential.rs`) that is `#[ignore]`d because the two drivers put
different bytes on the bus for the same API calls. The comparison was not
loosened to hide them; run `cargo test -p differential-tests -- --include-ignored`
to see the current diff. Neither driver has been changed by the differential
test work; what to do about each entry is a separate decision.

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
