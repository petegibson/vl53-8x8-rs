# rpi-zero-2w

VL53L8CX example bins for a Raspberry Pi Zero 2W driving a Pololu VL53L8CX
breakout over SPI0. Two bins:

- `blocking` — uses [`vl53l8cx-uld`](../../vl53l8cx-uld), the blocking
  wrapper around the ST ULD C firmware.
- `async_demo` — uses [`vl53-8x8`](../../vl53-8x8), the pure-Rust
  async port, on a single-threaded Tokio runtime.

> **Note on `async_demo`:** `linux-embedded-hal`'s `SpidevDevice` only
> implements the *blocking* `embedded-hal` SPI traits (no `embedded-hal-async`
> impl exists), so the async bin wraps it in a thin `BlockingSpi` adapter
> (see [`src/lib.rs`](./src/lib.rs)) that satisfies the async `SpiDevice`
> trait by calling the blocking transaction inside `async fn`. The
> upshot: SPI I/O still **blocks** the Tokio task during each transfer.
> Only the driver's internal `DelayNs` calls actually yield to the
> runtime. For a single-threaded `current_thread` executor this is fine
> (and what the bin uses); if you put the driver on a multi-threaded
> runtime, run it in a `spawn_blocking` task to keep other tasks
> responsive during the ~hundred-milliseconds-of-blocking firmware
> upload.

Unlike the FTDI host-side example, the Pi runs Linux natively, so the
peripheral access goes through `linux-embedded-hal` (spidev for SPI,
gpio-cdev for LPn). Linux's spidev produces clean Mode 3 SPI clocking
(no FT2232H CPHA bug), but the Pi's BCM2835 driver has its own
peculiarities — see "Note on CS polarity" and "Raise spidev's
transaction-size limit" below.

## Wiring

| Pi BCM | Header pin | Pololu pin   | Notes                       |
| ------ | ---------- | ------------ | --------------------------- |
| 3V3    |  1         | VIN          | sensor power                |
| GND    |  6         | GND          |                             |
| GPIO11 | 23         | SCL          | SPI0 SCLK                   |
| GPIO10 | 19         | SDA          | SPI0 MOSI                   |
| GPIO9  | 21         | MISO         | SPI0 MISO                   |
| GPIO8  | 24         | NCS          | SPI0 CE0 (managed by spidev)|
| GPIO17 | 11         | LPn          | I2C enable, active low      |

The Pololu silkscreen labels `SCL`/`SDA` because the board ships in I2C
mode. **Set the Pololu's SPI/I2C selector to SPI mode** before running.

## Pi setup (one-time)

### Enable SPI0

On Raspberry Pi OS:
```
# /boot/firmware/config.txt (or /boot/config.txt on older images)
dtparam=spi=on
```

### Raise spidev's transaction-size limit

The C ULD uploads the VL53L8CX firmware in **32 KB chunks**. Linux's
`spidev` defaults to a 4 KB ioctl buffer, so any transaction over that
returns `EMSGSIZE` and the firmware never reaches the chip — the bin
will then fail with `Firmware error: Timeout` while polling for boot.

Bump `spidev.bufsiz` to comfortably exceed 32 KB:

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

### Group membership

Add your user to the `spi` and `gpio` groups so the bins can run without
sudo:

```bash
sudo usermod -aG spi,gpio $USER
# log out + back in for groups to take effect
```

### Verify

```bash
ls /dev/spidev0.0 /dev/gpiochip0
cat /sys/module/spidev/parameters/bufsiz
```

### Note on CS polarity (dt-overlay)

The BCM2835 SPI driver doesn't always honour the `SPI_CS_HIGH` flag for
the native CE0/CE1 lines — the polarity is fixed by the dt-overlay
instead. If you see CS toggling active-high on a logic analyzer (the
chip will fail to respond at all in that case), you need to fix the
overlay's `cs-gpios`/`spi-cs-high` properties so CE0 is active-low. The
default Raspberry Pi OS overlay gets this right; some custom NixOS
overlays do not.

## Build & run

### On the Pi (simplest)

Install Rust on the Pi (rustup), then from this directory:

```bash
cargo run --bin blocking
cargo run --bin async_demo
```

First build is slow on a Pi Zero 2W (~10 min); incremental rebuilds are fast.

### Cross-compile from a desktop

The dev shell ships with both `aarch64-unknown-linux-gnu` and
`armv7-unknown-linux-gnueabihf` Rust targets plus their `pkgsCross` GCC
toolchains. The crate's `.cargo/config.toml` defaults the build target to
`aarch64-unknown-linux-gnu` (Pi Zero 2W on 64-bit Raspberry Pi OS Bookworm).

```bash
nix develop
cd examples/rpi-zero-2w
cargo build --bin blocking
# → target/aarch64-unknown-linux-gnu/debug/blocking
```

For 32-bit Raspberry Pi OS:

```bash
cargo build --target armv7-unknown-linux-gnueabihf --bin blocking
```

#### Deployment: ELF interpreter on the target

Nix's cross-toolchain bakes the *laptop's* nix-store glibc path into the
ELF as the dynamic linker, which won't exist on the Pi. Three options
depending on what's running on the Pi:

**On stock Raspberry Pi OS (Debian-based)** — patch the interpreter to
the standard glibc path:

```bash
patchelf --set-interpreter /lib/ld-linux-aarch64.so.1 \
  target/aarch64-unknown-linux-gnu/debug/blocking
# armv7: /lib/ld-linux-armhf.so.1
scp target/aarch64-unknown-linux-gnu/debug/blocking pi@raspberrypi:~/
```

**On NixOS** — `/lib/ld-linux-aarch64.so.1` is a stub that refuses
generic-Linux binaries by default. Either:

a) **Enable `nix-ld` on the Pi** (recommended for NixOS Pi) — one-line
   config change on the Pi:
   ```nix
   programs.nix-ld.enable = true;
   ```
   Then `sudo nixos-rebuild switch` and run the patchelf'd binary as in
   the stock-RPiOS instructions above.

b) **Build statically with musl** — no Pi-side config change, no
   patchelf, scp and run:
   ```bash
   cargo build --target aarch64-unknown-linux-musl --bin blocking
   scp target/aarch64-unknown-linux-musl/debug/blocking pi@raspberrypi:~/
   ssh pi@raspberrypi ./blocking
   ```
   The binary is fully self-contained (`file` says `statically linked`).
   Slightly larger than glibc, but trivially deployable to any aarch64
   Linux.

## Expected output

The actual SPI device path, mode, frequency and firmware-upload
duration printed by the bin reflect the constants in
[`src/lib.rs`](./src/lib.rs) — change `SPI_FREQ_HZ` there and the log
lines update automatically. With the defaults at the time of writing:

```
[INFO] Opening /dev/spidev0.0 in Mode 3 at 2 MHz...
[INFO] Acquiring LPn on BCM GPIO17...
[INFO] Uploading firmware (~86 KB, ~0.4 s at 2 MHz)...
[INFO] Sensor ready.
[INFO] Ranging 4x4 @ 10 Hz. Ctrl-C to stop.
--- Distance (mm) ---
  840   855   860   850
  820   810   790   795
  ...
```

Wave a hand in front of the sensor — center cells should drop from
background distance to a few hundred millimetres.
