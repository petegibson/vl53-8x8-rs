# Host + hardware test matrix for the VL53L5CX / VL53L8CX drivers.
# `just --list` shows every recipe; `just <recipe>` runs one.
#
# Hardware recipes flash and run the embedded-test suites in examples/ via
# probe-rs. Features: the Nucleo + X-NUCLEO-53L8A1 shield must be jumpered
# into SPI mode for `spi-bus`; `blocking-i2c` (embassy demo) or no bus feature
# at all selects I2C. The evo board is I2C-only.

set shell := ["bash", "-euo", "pipefail", "-c"]

# Any ST-Link V2-1 (Nucleo on-board debugger); first match wins.
nucleo_probe := "0483:374b"
# ST-Link V3 mini on the evo-f412re board, pinned by serial.
evo_probe    := "0483:3754:003400273431511837393330"

nucleo_uld     := "examples/nucleo-f401re-uld"
nucleo_embassy := "examples/nucleo-f401re-embassy"
nucleo_rtic    := "examples/nucleo-f401re-rtic"
evo            := "examples/evo-f412re-embassy"
rpi            := "examples/rpi-zero-2w"

# Show this list.
default:
    @just --list

# --- host -------------------------------------------------------------------

# Host tests for all workspace crates, incl. vl53-8x8's C-oracle tests (needs libclang).
test:
    cargo test --workspace

# Prove vl53-8x8 builds for Cortex-M without libclang (no C, no bindgen downstream).
check-no-libclang:
    env -u LIBCLANG_PATH cargo build -p vl53-8x8 --target thumbv7em-none-eabihf

# Dry-run publish of every workspace crate (demos are excluded / publish = false).
publish-dry-run:
    cargo publish --workspace --dry-run

# --- hardware: Nucleo F401RE + X-NUCLEO-53L8A1 (VL53L8CX) --------------------

# Blocking ULD suite on the Nucleo (`--test sensor`); e.g. `just nucleo-uld spi-bus,thresholds`.
nucleo-uld features="spi-bus":
    cd {{nucleo_uld}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor --features "{{features}}"

# Async (Embassy) suites on the Nucleo, `sensor_async` then `sensor_async_extra`.
nucleo-embassy features="spi-bus":
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async --features "{{features}}"
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async_extra --features "{{features}}"

# Not a test suite: `probe-rs run` streams defmt forever, so `timeout` ends it after 25 s and
# `|| true` keeps that from failing the recipe. Read the log: init, variant, one 4x4 grid per
# second, and the heartbeat ticks in between.

# RTIC 2 + blocking stm32f4xx-hal demo on the Nucleo (BlockingI2c/BlockingSpi); runs for 25 s, e.g. `just nucleo-rtic` (I2C: `just nucleo-rtic ""`).
nucleo-rtic features="spi-bus":
    cd {{nucleo_rtic}} && PROBE_RS_PROBE={{nucleo_probe}} timeout 25 cargo run --release --features "{{features}}" || true

# --- hardware: evo-f412re custom board (VL53L5CX or VL53L8CX, async DMA I2C) --

# Async suite on the evo board (`--test sensor_async --release`); e.g. `just evo targets-2`.
evo features="":
    cd {{evo}} && PROBE_RS_PROBE={{evo_probe}} cargo test --test sensor_async --release --features "{{features}}"

# --- matrix -------------------------------------------------------------------

# Full default hardware matrix (every feature set validated so far). Nucleo shield in SPI mode; both boards connected.
hw-matrix:
    just nucleo-uld spi-bus
    just nucleo-uld spi-bus,thresholds
    just nucleo-uld spi-bus,targets-2
    just nucleo-embassy spi-bus
    just nucleo-embassy spi-bus,targets-2
    just nucleo-embassy spi-bus,targets-4
    just nucleo-embassy spi-bus,thresholds,xtalk,motion
    just evo
    just evo targets-2
    just evo targets-4
    just evo xtalk

# --- human-assisted / scene reports (#[ignore]d; they print, and warn instead of failing) --

# Multi-target scene report on the Nucleo. Needs a two-surface scene (e.g. a clear lid ~20 mm above the sensor and a surface beyond).
scene-nucleo features="spi-bus,targets-2":
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async --features "{{features}}" -- --include-ignored test_multi_target_scene_report

# Multi-target scene report on the evo board. Needs the same two-surface scene.
scene-evo features="targets-2":
    cd {{evo}} && PROBE_RS_PROBE={{evo_probe}} cargo test --test sensor_async --release --features "{{features}}" -- --include-ignored multi_target_scene_report

# Motion indicator report on the Nucleo. Someone must move something in front of the sensor while it runs.
motion-nucleo:
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async_extra --features spi-bus,motion,xtalk -- --include-ignored test_motion_indicator_report

# Crosstalk calibration report on the Nucleo (before/after frames; cover glass recommended; KNOWN_DISTANCE_MM overrides the target distance).
xtalk-report-nucleo:
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async_extra --features spi-bus,xtalk -- --include-ignored test_calibrate_xtalk_report

# --- Raspberry Pi Zero 2W ----------------------------------------------------

# Static musl build of the Pi demo bins (scp and run; no patchelf). On the Pi, spidev needs `bufsiz=65536` (see its README).
pi-build:
    cd {{rpi}} && cargo build --release --target aarch64-unknown-linux-musl
