# Host + hardware test matrix for the VL53L5CX / VL53L8CX drivers.
# `just --list` shows the recipes grouped by target; `just <recipe>` runs one.
# Recipe names say what they do: `test-*` runs an on-target test suite,
# `run-*` flashes a demo and streams its log, `build-*` produces binaries,
# `report-*` runs an opt-in diagnostic that prints rather than passes/fails.
#
# Hardware recipes flash via probe-rs. The Nucleo + X-NUCLEO-53L8A1 shield
# must be jumpered into SPI mode for `spi-bus`; `blocking-i2c` (embassy demo)
# or no bus feature selects I2C. The evo board is I2C-only.

set shell := ["bash", "-euo", "pipefail", "-c"]

# Any ST-Link V2-1 (Nucleo on-board debugger); first match wins.
nucleo_probe := "0483:374b"
# ST-Link V3 mini on the evo-f412re board, pinned by serial.
evo_probe    := "0483:3754:003400273431511837393330"

nucleo_uld     := "examples/nucleo-f401re-uld"
nucleo_embassy := "examples/nucleo-f401re-embassy"
nucleo_rtic    := "examples/nucleo-f401re-rtic"
evo            := "examples/evo-f412re-embassy"
evo_uld        := "examples/evo-f412re-uld"
rpi            := "examples/rpi-zero-2w"

[private]
default:
    @just --list

# ---------------------------------------------------------------------------
# host
# ---------------------------------------------------------------------------

# Test: host unit + ULD-oracle tests for all crates (needs libclang)
[group('host')]
test:
    cargo test --workspace

# Test: ST's L8/L5 ULDs vs pure-Rust bus transcripts against recorded sensor responses (SPI L8 + I2C L5)
[group('host')]
test-differential:
    cargo test -p differential-tests

# Build: regenerate recordings/*.json from a raw trace log (name: -uld/-8x8, -l8-/-l5-; or `--variant L5`)
[group('host')]
build-recording log *args:
    python3 tools/bus_trace_to_transcript.py --summary {{args}} "{{log}}"

# Maintainer only: dry-run publish of the four library crates.
[private]
publish-dry-run:
    cargo publish --workspace --dry-run

# ---------------------------------------------------------------------------
# Nucleo F401RE + X-NUCLEO-53L8A1 (VL53L8CX)
# ---------------------------------------------------------------------------

# Test: blocking ULD wrapper suite (features e.g. `spi-bus,thresholds`)
[group('nucleo-f401re')]
test-nucleo-uld features="spi-bus":
    cd {{nucleo_uld}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor --features "{{features}}"

# Test: vl53-8x8 Embassy suite (features e.g. `spi-bus,thresholds`)
[group('nucleo-f401re')]
test-nucleo-embassy features="spi-bus":
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async --features "{{features}}"

# `probe-rs run` streams defmt until killed, so `timeout` ends the run after
# 25 s and `|| true` keeps that from failing the recipe. Expect init, the
# detected variant, one 4x4 grid per second and the heartbeat ticks between.

# Run: vl53-8x8 RTIC 2 demo for 25 s (I2C: `just run-nucleo-rtic ""`)
[group('nucleo-f401re')]
run-nucleo-rtic features="spi-bus":
    cd {{nucleo_rtic}} && PROBE_RS_PROBE={{nucleo_probe}} timeout 25 cargo run --release --features "{{features}}" || true

# Reports are #[ignore]d tests: they print what the sensor sees and warn
# instead of failing, because they depend on a staged scene or a person.

# Report: multi-target scene (needs two surfaces, e.g. lid + ceiling)
[group('nucleo-f401re')]
report-nucleo-scene features="spi-bus,targets-2":
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async --features "{{features}}" -- --include-ignored test_multi_target_scene_report

# Report: motion indicator (wave something in view while it runs)
[group('nucleo-f401re')]
report-nucleo-motion:
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async --features spi-bus,motion,xtalk -- --include-ignored test_motion_indicator_report

# Report: xtalk calibration, frames before and after
[group('nucleo-f401re')]
report-nucleo-xtalk:
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async --features spi-bus,xtalk -- --include-ignored test_calibrate_xtalk_report

# ---------------------------------------------------------------------------
# evo-f412re custom board (VL53L5CX or VL53L8CX, async DMA I2C)
# ---------------------------------------------------------------------------

# Test: vl53-8x8 Embassy suite (features e.g. `targets-2`)
[group('evo-f412re')]
test-evo features="":
    cd {{evo}} && PROBE_RS_PROBE={{evo_probe}} cargo test --test sensor_async --release --features "{{features}}"

# Report: multi-target scene (needs two surfaces in view)
[group('evo-f412re')]
report-evo-scene features="targets-2":
    cd {{evo}} && PROBE_RS_PROBE={{evo_probe}} cargo test --test sensor_async --release --features "{{features}}" -- --include-ignored multi_target_scene_report

# The C-ULD recipes need a VL53L5CX fitted: vl53l5cx-uld cannot drive an L8.

# Test: blocking vl53l5cx-uld (ST's L5 C ULD) suite (features e.g. `thresholds,xtalk`)
[group('evo-f412re')]
test-evo-uld features="":
    cd {{evo_uld}} && PROBE_RS_PROBE={{evo_probe}} cargo test --test sensor --release --features "{{features}}"

# Run: vl53l5cx-uld blocking demo for 25 s (init time, then one 4x4 grid per second)
[group('evo-f412re')]
run-evo-uld features="":
    cd {{evo_uld}} && PROBE_RS_PROBE={{evo_probe}} timeout 25 cargo run --release --features "{{features}}" || true

# ---------------------------------------------------------------------------
# Raspberry Pi Zero 2W (VL53L8CX on SPI0; VL53L5CX on I2C1 with `i2c-bus`)
# ---------------------------------------------------------------------------

# The Pi recipes cross-compile static musl binaries; for that target the
# crate's .cargo/config.toml installs tools/pi-run.sh as the cargo runner,
# which copies the binary to the Pi over ssh and runs it there (PI_HOST,
# default `toftestpi`). Features: "" = VL53L8CX/SPI, `i2c-bus` = VL53L5CX/I2C.

pi_target := "aarch64-unknown-linux-musl"

# Build: static musl demo binaries to scp to the Pi by hand (see its README)
[group('pi-zero-2w')]
build-pi features="":
    cd {{rpi}} && cargo build --release --target {{pi_target}} --features "{{features}}"

# Test: both Pi suites (ST ULD + vl53-8x8) on the Pi over ssh
[group('pi-zero-2w')]
test-pi features="":
    cd {{rpi}} && cargo test --release --target {{pi_target}} --features "{{features}}"

# Test: both Pi suites for both boards
[group('pi-zero-2w')]
test-pi-all:
    just test-pi
    just test-pi i2c-bus

# Run: a demo bin on the Pi for 25 s (bin: async_demo | blocking)
[group('pi-zero-2w')]
run-pi bin="async_demo" features="":
    cd {{rpi}} && timeout 25 cargo run --release --target {{pi_target}} --bin {{bin}} --features "{{features}}" || true

# ---------------------------------------------------------------------------
# all boards
# ---------------------------------------------------------------------------

# Test: every on-target suite and feature set on the Nucleo (SPI mode) and evo
[group('all-boards')]
test-all-hw:
    just test-nucleo-uld spi-bus
    just test-nucleo-uld spi-bus,thresholds
    just test-nucleo-uld spi-bus,targets-2
    just test-nucleo-embassy spi-bus
    just test-nucleo-embassy spi-bus,targets-2
    just test-nucleo-embassy spi-bus,targets-4
    just test-nucleo-embassy spi-bus,thresholds,xtalk,motion
    just test-evo
    just test-evo targets-2
    just test-evo targets-4
    just test-evo xtalk
    just test-evo-uld
    just test-evo-uld thresholds,xtalk,motion
    just test-evo-uld targets-2
