# differential-tests

Host-side proof that the pure-Rust driver `vl53-8x8` and ST's ULDs, wrapped
by `vl53l8cx-uld` (VL53L8CX) and `vl53l5cx-uld` (VL53L5CX), put
**byte-identical register traffic** on the bus for the same API calls. The
drivers run on the host against a mock sensor whose read responses come from
a recording of a real sensor, and their transcripts are compared op by op.
Not published; `cargo test -p differential-tests` (or `just test-differential`)
runs it in well under a second once built. Building the C drivers needs
libclang, like the rest of the workspace tests.

## What it proves

### VL53L8CX (`tests/differential.rs`)

For each re-enacted on-target test `T` there are three `#[test]`s:

| check | driver on host | model built from | transcript compared with | purpose |
|---|---|---|---|---|
| `T::a_c_driver_replays_c_recording` | `vl53l8cx-uld` | C recording of `T` | C recording of `T` | validates the model and the harness |
| `T::b_rust_driver_matches_c_recording` | `vl53-8x8` | C recording of `T` | C recording of `T` | **the differential check** |
| `T::c_rust_driver_replays_rust_recording` | `vl53-8x8` | Rust recording of `T` | Rust recording of `T` | sanity: the Rust driver reproduces its own hardware run |

Transcripts are compared after one normalisation: runs of identical consecutive
reads (same register, length and data) collapse to one, because poll loops ran
a timing-dependent number of iterations on hardware. Writes are never merged;
both drivers upload the firmware in the same three bank writes (32768, 32768,
20480 bytes), so that needed no normalisation. A mismatch reports the first
differing op index, the op from each side and three ops of context on both
sides. The delay each driver requested is printed (`-- --nocapture`), not
asserted. Against the C model the Rust driver requests exactly the C driver's
total in every sequence that does not stop ranging, and 10 ms less per stop
in the four that do (`test_ranging_4x4`, `test_ranging_8x8`,
`test_ambient_light`, and 20 ms in
`test_set_resolution_between_ranging_sessions`, which stops twice): ST's
`vl53l8cx_stop_ranging` waits 10 ms after every poll of register `0x06`,
including the one that sees the MCU stopped, while `vl53-8x8` breaks out of the
loop before that last wait. Same bytes, one `delay_ms(10)` fewer per stop.

### VL53L5CX (`tests/differential_l5.rs`)

There is no C recording of a VL53L5CX. The only hardware recording is of the
Rust driver running the variant-agnostic evo-f412re suite over I2C
(`recordings/evo-l5-i2c-8x8.json`), so the model is built from the Rust
recording and the question is turned around: given the responses the real
sensor gave the Rust driver, does ST's own L5 code issue the same writes and
reads? The Rust L5 path autodetects the part and applies `L5_FW_PATCH` /
`L5_CONFIG_PATCH` / `L5_CALIBRATE_XTALK_PATCH` plus the L5 init quirks (glare
filter writes, poll-based firmware verify, extra MCU wake-up writes); the
check covers all of them. For each evo test `T`:

| check | driver on host | model built from | transcript compared with | purpose |
|---|---|---|---|---|
| `T::c_rust_driver_replays_rust_recording` | `vl53-8x8` over `BlockingI2c` | Rust L5 recording of `T` | Rust L5 recording of `T` | sanity: model and I2C mock reproduce the hardware run |
| `T::d_c_l5_driver_matches_rust_recording` | `vl53l5cx-uld` | Rust L5 recording of `T` | Rust L5 recording of `T` | **the L5 differential check**, same normalisation as the L8 ones |
| `T::d2_c_l5_driver_matches_rust_recording_with_writes_coalesced` | `vl53l5cx-uld` | Rust L5 recording of `T` | Rust L5 recording of `T` | as (d), but register-contiguous writes are merged on both sides first |

(d2) exists because the Rust driver uploads the firmware bank that holds the
L5 patch in three writes where ST's code uses one (KNOWN_DIVERGENCES.md #2),
which fails the strict (d) check of every sequence with an `init` at that op;
(d2) keeps everything behind it under test. A read of a `(reg, len)` the
recording never saw is answered with zeros and listed in the failure message
rather than crashing the C code. Three cross-checks (`l5_*_differs_from_l8_at_exactly_the_patched_offsets`)
assemble the firmware, the default configuration and the xtalk command table
from the bus writes of the Rust L5 recording, of ST's L5 code on the host and
of the C L8 recording and assert that the L5 and L8 bytes differ at exactly the
offsets of the three patch tables. `dump_transcript` (ignored) prints one
driver's full transcript for one sequence, for investigations.

One L5-only allowance, in both directions: since `vl53-8x8` 0.3.0 the Rust
driver ends an L5-class `init` with a DCI read of the firmware flags at
0xE0C4 to tell an L5 from an L7 (KNOWN_DIVERGENCES.md #6). ST's L5 code never
issues it and the evo recording predates it, so the L5 checks remove that
probe from both transcripts (`strip_module_type_probe`) and the model serves
its reads synthetically rather than from the recording, which keeps the
per-key cursors of the following DCI operations in phase. The L8 checks do
neither: the L8 path does not probe, and the VL53L8CX ULD's own identical
read of 0xE0C4 in `start_ranging` is compared like any other op.

Current state: all 14 (c) checks pass; (d) passes for `async_i2c_alive` and
is ignored for the 13 sequences with an `init` (#2); (d2) passes for 11 of 14
and is ignored for `power_mode_sleep_wakeup` (#3),
`calibrate_xtalk_leaves_sensor_usable` (#4, read order; a diagnostic shows the
rest of the calibration path identical) and
`calibrate_xtalk_rejects_invalid_params` (#5, ST's code talks to the sensor
for rejected arguments).

Differential checks that currently fail are `#[ignore]`d with a pointer to
[`KNOWN_DIVERGENCES.md`](KNOWN_DIVERGENCES.md), which documents the exact bytes.
Run `cargo test -p differential-tests -- --include-ignored` to see them.

## Register model

The mock answers reads from a `RegisterModel` (`src/model.rs`) built from
**one recorded sequence** (one on-target test, which embedded-test ran from a
fresh sensor reset) and used for exactly one host replay of that test; nothing
carries over between tests.

Reads are keyed by `(register, length)`. For each key the model keeps the
recorded responses **in order, repeats included** (a poll loop that saw
`0x00, 0x00, 0x00, 0x01` keeps all four), with a cursor per key. A read returns
the response at the cursor and advances it; past the end the last response
sticks. Writes are accepted and not interpreted.

Why per-key rather than one positional cursor over all reads: a driver that
reads some register one time more or less than the recording would otherwise
shift every later answer; per-key cursors keep the other registers in phase.
Why repeats are kept rather than collapsed to distinct values: on the mock there
is no clock, so a poll loop's length is decided only by the answers it gets, and
a driver with the same logic as the recorded one then polls exactly as often as
recorded. Collapsing would break two consecutive poll loops on the same register
that each finished in one read with the same value (they would merge, and the
second loop would be served the first answer of the third). The two drivers did
poll a different number of times on hardware, which is what the read-collapsing
normalisation of the comparison is for.

## Recordings

`recordings/*.json` are compact transcripts of the raw defmt traces, one
sequence per on-target test, generated by `tools/bus_trace_to_transcript.py`.
Payloads of 64 bytes or more that are exact slices of a known ULD table (the
firmware image, `DEFAULT_CONFIGURATION`, `DEFAULT_XTALK`, `GET_NVM_CMD`,
`CALIBRATE_XTALK` from `vl53-8x8/src`, and their VL53L5CX versions
`firmware_l5`, `default_configuration_l5`, `calibrate_xtalk_l5`, which are the
L8 tables with `vl53_8x8::sensor::L5_FW_PATCH` / `L5_CONFIG_PATCH` /
`L5_CALIBRATE_XTALK_PATCH` applied) are stored as blob references and expanded
at load time; everything else is inline hex. A slice common to both variants
is named after the recording's variant. Current files:

| file | source suite | board / bus | tests | ops | size |
|---|---|---|---|---|---|
| `nucleo-l8-spi-uld.json` | `examples/nucleo-f401re-uld/tests/sensor.rs` (C wrapper) | NUCLEO-F401RE + X-NUCLEO-53L8A1, SPI | 25 | 3156 | 290 kB |
| `nucleo-l8-spi-8x8.json` | `examples/nucleo-f401re-embassy/tests/sensor_async.rs` (pure Rust) | NUCLEO-F401RE + X-NUCLEO-53L8A1, SPI | 28 | 3970 | 380 kB |
| `evo-l5-i2c-8x8.json` | `examples/evo-f412re-embassy/tests/sensor_async.rs` (pure Rust) | evo-f412re with a VL53L5CX, I2C at 0x29 | 14 | 2004 | 182 kB |

The Nucleo recordings were taken with features
`spi-bus,defmt-bus-trace,thresholds,xtalk,motion`, the evo one with
`defmt-bus-trace,thresholds,xtalk,motion` (I2C is the default bus). The
I2C trace flavour (`TRACE i2c W [0x29] reg=0x7fff len=1`) carries the 7-bit
address, which the JSON drops: transcripts are `(register, data)` pairs
whatever the bus, so an SPI and an I2C transcript of the same operation
compare equal, and the mock records the addresses separately (the L5 tests
assert they are all `0x29`).

### Recording

Flash and run each suite with trace logging, capturing probe-rs output to a file:

```sh
cd examples/nucleo-f401re-uld
DEFMT_LOG=trace PROBE_RS_PROBE=0483:374b cargo test \
    --features spi-bus,defmt-bus-trace,thresholds,xtalk,motion --test sensor \
    2>&1 | tee /tmp/nucleo-l8-spi-uld.log

cd examples/nucleo-f401re-embassy
DEFMT_LOG=trace PROBE_RS_PROBE=0483:374b cargo test \
    --features spi-bus,defmt-bus-trace,thresholds,xtalk,motion --test sensor_async \
    2>&1 | tee /tmp/nucleo-l8-spi-8x8.log

cd examples/evo-f412re-embassy
DEFMT_LOG=trace cargo test \
    --features defmt-bus-trace,thresholds,xtalk,motion --test sensor_async \
    2>&1 | tee /tmp/evo-l5-i2c-8x8.log
```

The file name matters: `-uld`/`-8x8` selects the `driver` field, `-l8-`/`-l5-`
the `variant` (which also selects the blob set to match against; `--variant`
overrides it), and the output lands at `recordings/<stem>.json`. The bus is
taken from the trace prefix.

### Regenerating the JSON

```sh
python3 tools/bus_trace_to_transcript.py --summary /tmp/nucleo-l8-spi-uld.log /tmp/nucleo-l8-spi-8x8.log
python3 tools/bus_trace_to_transcript.py --summary --features defmt-bus-trace thresholds xtalk motion /tmp/evo-l5-i2c-8x8.log
# or: just build-recording /tmp/nucleo-l8-spi-uld.log
python3 tools/bus_trace_to_transcript.py --check /tmp/nucleo-l8-spi-uld.log   # JSON matches the log, byte for byte
python3 tools/bus_trace_to_transcript.py --check differential-tests/recordings/*.json   # JSON is self-consistent
```

The script verifies the blob substitution round-trips before writing anything.
A recording from another board or bus is a new file; the test binaries load
the three above by name (`src/transcript.rs`).

## Sequences covered

The sequences (`src/driver.rs`, `run_sequence`) follow the C suite call for
call; the Rust suite's only differences are that it waits for data with
`Sensor::poll_data_ready` (same bus traffic as the manual `check_data_ready`
loop used here, plus a 10 ms delay between polls) and that it has xtalk
calibration tests the C suite lacks. Where a test asserted on a returned value
(resolution read-back, frequency, distances in range, ...) the assertion is
kept, so a differently decoded register fails too, after the bus comparison.

`test_sensor_alive`, `test_init`, `test_set_resolution`, `test_ranging_4x4`,
`test_ranging_8x8`, `test_ambient_light`, `test_ranging_frequency`,
`test_target_order`, `test_ranging_mode`, `test_integration_time`,
`test_sharpener_percent`, `test_power_mode`,
`test_set_resolution_between_ranging_sessions`, `test_reinit_after_power_cycle`
(mock power-enable and LPn pins, no bus traffic for the power cycle),
`test_vhv_repeat_count`, `test_external_sync_pin_enable`,
`test_wait_for_data_ready_blocking` (mock INT pin reads asserted),
`test_detection_thresholds_enable`, `test_detection_thresholds_auto_stop`,
`test_detection_thresholds` (all three checks ignored until the recordings are
re-taken: since 0.2.0 the sequence sets `VL53L8CX_LAST_THRESHOLD` on its last
entry as ST's header requires), `test_motion_indicator_init`,
`test_motion_indicator_set_distance`, `test_motion_indicator_set_resolution`,
`test_xtalk_margin`, `test_caldata_xtalk`.

Not covered: `test_set_address` (I2C only, absent from the SPI recordings) and
the Rust-only `test_calibrate_xtalk_*` tests (no C counterpart to compare with).

The evo-f412re sequences (`run_evo_sequence`, `EVO_SEQUENCES`) follow
`examples/evo-f412re-embassy/tests/sensor_async.rs`; where a test's calls
equal a Nucleo test's, that sequence is reused. `async_i2c_alive`,
`init_uploads_firmware_over_async_i2c`, `set_resolution`, `ranging_4x4_async`,
`ranging_8x8_async`, `ambient_light` (distance plausibility only: `Driver`
does not expose ambient), `ranging_frequency` (5 and 15 Hz), `integration_time`,
`target_order`, `ranging_mode`, `sharpener_percent`, `power_mode_sleep_wakeup`,
`calibrate_xtalk_leaves_sensor_usable` (`calibrate_xtalk(5, 4, 600)` may
return `XtalkFailed`; the hardware run accepted the scene),
`calibrate_xtalk_rejects_invalid_params` (six rejected argument sets, then the
resolution is checked). Not covered: `multi_target_scene_report` (`targets-N`
only, ignored on target too).

## Layout

- `src/transcript.rs`: JSON loading, blob expansion (firmware via
  `include_bytes!`, the `uld_data.rs` arrays parsed from the Rust source).
- `src/model.rs`: the register model.
- `src/mock.rs`: `MockSpi` (blocking `SpiDevice`, 16-bit address, bit 15 =
  write), `MockI2c` (blocking `I2c`, 2-byte register prefix), `MockDelay`
  (blocking and async `DelayNs`, counts requested time), `MockPin`, `block_on`.
- `src/compare.rs`: normalisation and the diff.
- `src/driver.rs`: the `Driver` trait, the `vl53-8x8` adapter (generic over
  `BlockingSpi<MockSpi>` / `BlockingI2c<MockI2c>`; the variant is whatever the
  model's revision id says) and the Nucleo and evo sequences.
- `tests/differential.rs`: the `vl53l8cx-uld` adapter (its
  `bind_platform_driver_spi!` platform symbols `vl53l8cx_platform_*` are
  `#[no_mangle]`, so all L8 C driver tests live in this one binary) and the
  generated L8 tests.
- `tests/differential_l5.rs`: the `vl53l5cx-uld` adapter
  (`bind_platform_driver!(MockI2c, MockDelay)`, symbols `vl53l5cx_platform_*`),
  the generated L5 tests, the patch-table cross-checks and `dump_transcript`.
