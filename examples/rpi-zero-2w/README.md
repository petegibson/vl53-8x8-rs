# rpi-zero-2w

Raspberry Pi Zero 2W examples and hardware-in-the-loop tests for two Pololu
breakouts, selected with a cargo feature:

| Feature     | Sensor   | Bus                    | LPn        | ULD crate      |
| ----------- | -------- | ---------------------- | ---------- | -------------- |
| (default)   | VL53L8CX | SPI0, `/dev/spidev0.0` | BCM GPIO17 | `vl53l8cx-uld` |
| `i2c-bus`   | VL53L5CX | I2C1, `/dev/i2c-1`     | BCM GPIO5  | `vl53l5cx-uld` |

Two bins and two test suites, each built for whichever board the feature
selects:

- `blocking` / `tests/uld.rs` — ST's C ULD through the blocking wrapper
  ([`vl53l8cx-uld`](../../vl53l8cx-uld) or [`vl53l5cx-uld`](../../vl53l5cx-uld);
  `rpi_zero_2w::uld` picks one, the two share an API).
- `async_demo` / `tests/sensor_async.rs` — the pure-Rust async
  [`vl53-8x8`](../../vl53-8x8) driver on a single-threaded Tokio runtime. It
  autodetects the part and the bin logs which one it found.

Both boards can stay wired at once; the feature only decides which one a
build talks to.

> **Not truly async.** `linux-embedded-hal` only implements the *blocking*
> `embedded-hal` bus traits, so `async_demo` hands the bus to the driver
> through `vl53_8x8::BlockingSpi` / `vl53_8x8::BlockingI2c`. Every transfer
> blocks the Tokio task; only the driver's `DelayNs` waits yield. Fine on the
> `current_thread` runtime the bin uses; on a multi-threaded runtime run the
> driver inside `spawn_blocking`.

## Wiring

### VL53L8CX on SPI0 (default)

| Pi BCM | Header pin | Pololu pin | Notes                        |
| ------ | ---------- | ---------- | ---------------------------- |
| 3V3    |  1         | VIN        | sensor power                 |
| GND    |  6         | GND        |                              |
| GPIO11 | 23         | SCL        | SPI0 SCLK                    |
| GPIO10 | 19         | SDA        | SPI0 MOSI                    |
| GPIO9  | 21         | MISO       | SPI0 MISO                    |
| GPIO8  | 24         | NCS        | SPI0 CE0 (managed by spidev) |
| GPIO17 | 11         | LPn        | I2C enable, active low       |

The Pololu silkscreen says `SCL`/`SDA` because the board ships in I2C
mode. **Set the Pololu VL53L8CX board's SPI/I2C selector to SPI mode**
before running.

### VL53L5CX on I2C1 (`--features i2c-bus`)

| Pi BCM | Header pin | Pololu pin | Notes                      |
| ------ | ---------- | ---------- | -------------------------- |
| 3V3    | 17         | VIN        | sensor power               |
| GND    | 25         | GND        |                            |
| GPIO2  |  3         | SDA        | I2C1 SDA (Pi has 1.8 kΩ pull-up) |
| GPIO3  |  5         | SCL        | I2C1 SCL (Pi has 1.8 kΩ pull-up) |
| GPIO5  | 29         | LPn        | I2C enable, active low     |

Pin numbers in the driver are **BCM GPIO numbers**, not header pins: LPn is
BCM GPIO5, which is header pin 29 (header pin 5 is SCL). The
`lpn_gates_i2c` test checks the wire is really there: with LPn low the
sensor must stop acknowledging.

## Pi setup (one-time)

### Enable the buses

On Raspberry Pi OS, in `/boot/firmware/config.txt` (`/boot/config.txt` on
older images):

```
dtparam=spi=on
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000   # optional; 100 kHz default works, upload is 4x slower
```

On NixOS both are device-tree overlays; the maintainers' Pi carries an
overlay that turns on SPI0 with active-low CE0 and sets I2C1 to 400 kHz.
I2C1 is enabled at 100 kHz in the stock DTB, so only the speed needs an
overlay fragment:

```dts
fragment@3 {
  target = <&i2c1>;
  __overlay__ { status = "okay"; clock-frequency = <400000>; };
};
```

### Raise spidev's transaction-size limit (SPI only)

Both drivers upload the firmware in **32 KB** register writes. Linux's
`spidev` defaults to a 4 KB ioctl buffer, so any transaction over that
returns `EMSGSIZE` and the firmware never reaches the chip — the bin then
fails with `Firmware error: Timeout` while polling for boot.

```bash
# one-shot test (resets on reboot):
sudo rmmod spidev && sudo modprobe spidev bufsiz=65536
cat /sys/module/spidev/parameters/bufsiz   # should print 65536
```

Permanent on NixOS:
```nix
boot.extraModprobeConfig = ''
  options spidev bufsiz=65536
'';
```

(or `boot.kernelParams = [ "spidev.bufsiz=65536" ];` if spidev is
built-in rather than a module — check `lsmod | grep spidev`).

The I2C path needs no such knob: `i2c-dev` caps a message at 8 KB and
cannot be raised, so the crate's `LinuxI2c` adapter (in `src/lib.rs`)
splits long register writes into re-addressed 8 KB messages. It also merges
the drivers' index + data write pairs into one message, because
`linux-embedded-hal` would otherwise send each `Operation::Write` with its
own START and address, which the sensor reads as a second write.

### Group membership

Add your user to the `spi`, `i2c` and `gpio` groups so the bins run
without sudo:

```bash
sudo usermod -aG spi,i2c,gpio $USER
# log out + back in for groups to take effect
```

On NixOS `hardware.i2c.enable = true;` creates the `i2c` group and the
udev rule for `/dev/i2c-*`; add `"i2c"` to the user's `extraGroups`.

### Verify

```bash
ls /dev/spidev0.0 /dev/i2c-1 /dev/gpiochip0
cat /sys/module/spidev/parameters/bufsiz
i2cdetect -y 1          # 0x29 appears once LPn is high (it is low until a bin runs)
```

### Note on CS polarity (dt-overlay)

The BCM2835 SPI driver doesn't always honour the `SPI_CS_HIGH` flag for
the native CE0/CE1 lines — the polarity is fixed by the dt-overlay
instead. If you see CS toggling active-high on a logic analyzer (the
chip will fail to respond at all in that case), you need to fix the
overlay's `cs-gpios`/`spi-cs-high` properties so CE0 is active-low. The
default Raspberry Pi OS overlay gets this right; some custom NixOS
overlays do not.

## Build, run and test

### From the dev shell (cross-compile, run on the Pi over ssh)

The dev shell ships the `aarch64-unknown-linux-musl` target and toolchain;
static musl binaries run on any aarch64 Linux, NixOS included, with no
`patchelf`. For that target `.cargo/config.toml` installs
[`tools/pi-run.sh`](./tools/pi-run.sh) as the cargo *runner*: it copies the
binary to the Pi with `scp`, runs it there and returns its exit status, so
`cargo run` and `cargo test` behave as if the Pi were local. `PI_HOST`
names the ssh destination (default `toftestpi`); ssh keys must already work.

From the repository root:

```bash
just test-pi                      # VL53L8CX / SPI: both suites
just test-pi i2c-bus              # VL53L5CX / I2C: both suites
just test-pi-all                  # both boards
just run-pi async_demo            # stream 4x4 frames for 25 s
just run-pi blocking i2c-bus
just build-pi                     # binaries only, to scp by hand
```

or directly, from this directory:

```bash
cargo test --release --target aarch64-unknown-linux-musl --features i2c-bus
cargo run  --release --target aarch64-unknown-linux-musl --bin blocking
```

The test binaries share one sensor, so they lock a process-wide mutex and
the runner sets `RUST_TEST_THREADS=1`. Every test re-uploads the firmware
(about 0.4 s over SPI at 2 MHz, 2 s over I2C at 400 kHz, 8 s at 100 kHz).

### On the Pi (no cross toolchain)

Install Rust on the Pi (rustup), then from this directory:

```bash
cargo run --bin blocking
cargo run --bin async_demo --features i2c-bus
cargo test --features i2c-bus
```

The crate's default target is `aarch64-unknown-linux-gnu`, which on the Pi
is the host, so there is no runner involved. First build is slow on a Pi
Zero 2W (~10 min); incremental rebuilds are fast.

### Cross-compile for stock Raspberry Pi OS (glibc)

```bash
cargo build --bin blocking                         # aarch64-unknown-linux-gnu
cargo build --target armv7-unknown-linux-gnueabihf --bin blocking   # 32-bit OS
```

Nix's cross-toolchain bakes the *laptop's* nix-store glibc path into the
ELF as the dynamic linker, which won't exist on the Pi. Patch it to the
standard path before copying:

```bash
patchelf --set-interpreter /lib/ld-linux-aarch64.so.1 \
  target/aarch64-unknown-linux-gnu/debug/blocking
# armv7: /lib/ld-linux-armhf.so.1
scp target/aarch64-unknown-linux-gnu/debug/blocking pi@raspberrypi:~/
```

On a NixOS Pi `/lib/ld-linux-aarch64.so.1` is a stub that refuses generic
binaries; either enable `programs.nix-ld.enable = true;` there or use the
musl build above.

## Expected output

```
[INFO] VL53L5CX on I2C1: opening /dev/i2c-1 (...) with LPn on BCM GPIO5...
[INFO] Uploading firmware (~86 KB)...
[INFO] Sensor ready (2.1 s), detected variant: L5 (expected L5).
[INFO] Ranging 4x4 @ 10 Hz. Ctrl-C to stop.
--- Distance (mm) ---
  840   855   860   850
  820   810   790   795
  ...
```

Wave a hand in front of the sensor — center cells should drop from
background distance to a few hundred millimetres.
