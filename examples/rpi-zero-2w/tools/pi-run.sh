#!/usr/bin/env bash
# Cargo "runner" for the musl target: copy the freshly built binary to the Pi
# over ssh, run it there with the given arguments, and return its exit status,
# so `cargo run` and `cargo test` from the dev shell behave as if the Pi were
# local. Wired up in .cargo/config.toml for aarch64-unknown-linux-musl only;
# building on the Pi itself never goes through here.
#
#   PI_HOST        ssh destination (default: pi@toftestpi)
#   PI_REMOTE_DIR  directory on the Pi for the binaries, relative to the ssh
#                  user's home (default: vl53-pi-bins). This is the one place
#                  binaries live on the Pi: the runner drops the latest build
#                  of each bin/test here (cargo's hash suffix stripped, so
#                  `uld`, `sensor_async`, `blocking`, `async_demo`), and
#                  hand-copied builds go in the same folder with a `-l8-spi` /
#                  `-l5-i2c` suffix for running with sudo.
#
# Test binaries share one sensor, so RUST_TEST_THREADS=1 is forced on the Pi;
# RUST_LOG is forwarded for the demo bins (default: info).
set -euo pipefail

host="${PI_HOST:-pi@toftestpi}"
remote_dir="${PI_REMOTE_DIR:-vl53-pi-bins}"
bin="$1"
shift
# tests/uld.rs builds to deps/uld-<16 hex>; keep just `uld`.
name="$(basename "$bin" | sed -E 's/-[0-9a-f]{16}$//')"

ssh -o BatchMode=yes "$host" "mkdir -p '$remote_dir'"
scp -q "$bin" "$host:$remote_dir/$name"

# A tty on the remote side lets Ctrl-C stop a streaming demo; only ask for one
# when we have one ourselves (cargo test output is piped by some harnesses).
tty_flag=""
if [ -t 0 ] && [ -t 1 ]; then
    tty_flag="-t"
fi

args=""
for a in "$@"; do
    args+=" $(printf '%q' "$a")"
done

# VL53_LPN_GPIO (which I2C board to drive, 5 or 6) is forwarded when set.
lpn_env=""
if [ -n "${VL53_LPN_GPIO:-}" ]; then
    lpn_env="VL53_LPN_GPIO='$VL53_LPN_GPIO' "
fi

exec ssh $tty_flag "$host" "cd '$remote_dir' && ${lpn_env}RUST_TEST_THREADS=1 RUST_LOG='${RUST_LOG:-info}' ./'$name'$args"
