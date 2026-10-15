# Host + hardware test matrix for the VL53L5CX / VL53L8CX drivers.
# `just --list` shows the recipes grouped by target; `just <recipe>` runs one.
#
# Hardware recipes flash and run the embedded-test suites in examples/ via
# probe-rs. The Nucleo + X-NUCLEO-53L8A1 shield must be jumpered into SPI
# mode for `spi-bus`; `blocking-i2c` (embassy demo) or no bus feature selects
# I2C. The evo board is I2C-only.

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

[private]
default:
    @just --list

# ---------------------------------------------------------------------------
# host
# ---------------------------------------------------------------------------

# Host tests, all crates (oracle tests need libclang)
[group('host')]
test:
    cargo test --workspace

# Dry-run publish of the four library crates
[group('host')]
publish-dry-run:
    cargo publish --workspace --dry-run

# ---------------------------------------------------------------------------
# Nucleo F401RE + X-NUCLEO-53L8A1 (VL53L8CX)
# ---------------------------------------------------------------------------

# Blocking ULD suite (e.g. `spi-bus,thresholds`)
[group('nucleo-f401re')]
nucleo-uld features="spi-bus":
    cd {{nucleo_uld}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor --features "{{features}}"

# Embassy suites (sensor_async + sensor_async_extra)
[group('nucleo-f401re')]
nucleo-embassy features="spi-bus":
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async --features "{{features}}"
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async_extra --features "{{features}}"

# `probe-rs run` streams defmt until killed, so `timeout` ends the run after
# 25 s and `|| true` keeps that from failing the recipe. Expect init, the
# detected variant, one 4x4 grid per second and the heartbeat ticks between.

# RTIC 2 demo for 25 s (I2C: `just nucleo-rtic ""`)
[group('nucleo-f401re')]
nucleo-rtic features="spi-bus":
    cd {{nucleo_rtic}} && PROBE_RS_PROBE={{nucleo_probe}} timeout 25 cargo run --release --features "{{features}}" || true

# The three reports below are #[ignore]d tests: they print and warn, never fail.

# Multi-target report; needs a two-surface scene
[group('nucleo-f401re')]
scene-nucleo features="spi-bus,targets-2":
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async --features "{{features}}" -- --include-ignored test_multi_target_scene_report

# Motion report; wave something in view while it runs
[group('nucleo-f401re')]
motion-nucleo:
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async_extra --features spi-bus,motion,xtalk -- --include-ignored test_motion_indicator_report

# Xtalk calibration report (before/after frames)
[group('nucleo-f401re')]
xtalk-report-nucleo:
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async_extra --features spi-bus,xtalk -- --include-ignored test_calibrate_xtalk_report

# ---------------------------------------------------------------------------
# evo-f412re custom board (VL53L5CX or VL53L8CX, async DMA I2C)
# ---------------------------------------------------------------------------

# Embassy suite; e.g. `just evo targets-2`
[group('evo-f412re')]
evo features="":
    cd {{evo}} && PROBE_RS_PROBE={{evo_probe}} cargo test --test sensor_async --release --features "{{features}}"

# Multi-target report (#[ignore]d); needs a two-surface scene
[group('evo-f412re')]
scene-evo features="targets-2":
    cd {{evo}} && PROBE_RS_PROBE={{evo_probe}} cargo test --test sensor_async --release --features "{{features}}" -- --include-ignored multi_target_scene_report

# ---------------------------------------------------------------------------
# Raspberry Pi Zero 2W
# ---------------------------------------------------------------------------

# Static musl bins (spidev setup: see its README)
[group('pi-zero-2w')]
pi-build:
    cd {{rpi}} && cargo build --release --target aarch64-unknown-linux-musl

# ---------------------------------------------------------------------------
# all boards
# ---------------------------------------------------------------------------

# Full matrix; Nucleo in SPI mode, both boards connected
[group('all-boards')]
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
