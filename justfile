# Host + hardware test matrix for the VL53L5CX / VL53L8CX drivers.
# `just --list` shows the recipes by section; `just <recipe>` runs one.
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
# tests (host)
# ---------------------------------------------------------------------------

# Host tests, all crates (oracle tests need libclang)
[group('tests')]
test:
    cargo test --workspace

# Dry-run publish of the four library crates
[group('tests')]
publish-dry-run:
    cargo publish --workspace --dry-run

# ---------------------------------------------------------------------------
# hardware: on-target suites
# ---------------------------------------------------------------------------

# Nucleo, blocking ULD suite (e.g. `spi-bus,thresholds`)
[group('hardware')]
nucleo-uld features="spi-bus":
    cd {{nucleo_uld}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor --features "{{features}}"

# Nucleo, Embassy suites (sensor_async + sensor_async_extra)
[group('hardware')]
nucleo-embassy features="spi-bus":
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async --features "{{features}}"
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async_extra --features "{{features}}"

# evo board, Embassy suite; e.g. `just evo targets-2`
[group('hardware')]
evo features="":
    cd {{evo}} && PROBE_RS_PROBE={{evo_probe}} cargo test --test sensor_async --release --features "{{features}}"

# Full matrix; Nucleo in SPI mode, both boards connected
[group('hardware')]
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

# ---------------------------------------------------------------------------
# demos: run a binary and watch its defmt log
# ---------------------------------------------------------------------------

# `probe-rs run` streams defmt until killed, so `timeout` ends the run after
# 25 s and `|| true` keeps that from failing the recipe. Expect init, the
# detected variant, one 4x4 grid per second and the heartbeat ticks between.

# Nucleo, RTIC 2 demo for 25 s (I2C: `just nucleo-rtic ""`)
[group('demos')]
nucleo-rtic features="spi-bus":
    cd {{nucleo_rtic}} && PROBE_RS_PROBE={{nucleo_probe}} timeout 25 cargo run --release --features "{{features}}" || true

# Pi Zero 2W, static musl bins (spidev setup: see its README)
[group('demos')]
pi-build:
    cd {{rpi}} && cargo build --release --target aarch64-unknown-linux-musl

# ---------------------------------------------------------------------------
# reports: #[ignore]d scene / human-assisted tests (print and warn, never fail)
# ---------------------------------------------------------------------------

# Nucleo multi-target report; needs a two-surface scene
[group('reports')]
scene-nucleo features="spi-bus,targets-2":
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async --features "{{features}}" -- --include-ignored test_multi_target_scene_report

# evo multi-target report; needs a two-surface scene
[group('reports')]
scene-evo features="targets-2":
    cd {{evo}} && PROBE_RS_PROBE={{evo_probe}} cargo test --test sensor_async --release --features "{{features}}" -- --include-ignored multi_target_scene_report

# Nucleo motion report; wave something in view while it runs
[group('reports')]
motion-nucleo:
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async_extra --features spi-bus,motion,xtalk -- --include-ignored test_motion_indicator_report

# Nucleo xtalk calibration report (before/after frames)
[group('reports')]
xtalk-report-nucleo:
    cd {{nucleo_embassy}} && PROBE_RS_PROBE={{nucleo_probe}} cargo test --test sensor_async_extra --features spi-bus,xtalk -- --include-ignored test_calibrate_xtalk_report
