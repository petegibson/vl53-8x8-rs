# vl53-8x8

A pure-Rust, `no_std`, async driver for the STMicroelectronics **VL53L5CX**
and **VL53L8CX** 8x8 multizone Time-of-Flight sensors, over
`embedded-hal-async` I2C or SPI.

There is no C code, no FFI and no build script. The ST Ultra Lite Driver
(ULD) protocol is reimplemented in Rust, so the crate builds for any target
with nothing more than a Rust toolchain. Its only runtime dependencies are
`embedded-hal`, `embedded-hal-async` and `vl53-8x8-types`.

## Crate family

| Crate              | Role                                                      |
| ------------------ | --------------------------------------------------------- |
| `vl53-8x8`         | this crate: pure-Rust async driver, VL53L5CX + VL53L8CX   |
| `vl53-8x8-types`   | shared enums and `Error`; re-exported here                |
| `vl53l8cx-uld-sys` | bindgen + cc over ST's VL53L8CX ULD; here only a test oracle |
| `vl53l8cx-uld`     | blocking wrapper over the C ULD (VL53L8CX only)           |

`vl53l8cx-uld-sys` is a **dev-dependency**: the oracle tests compare this
crate's constants, data tables and `ResultsData` layout against the C build.
Downstream users never pull it in and never need libclang.

## Variant detection and firmware

The VL53L5CX and VL53L8CX run the same ULD firmware image apart from a handful
of bytes. `Sensor::init` reads the device and revision IDs (`detect`,
`Variant`), then uploads the shipped VL53L8CX image with the per-variant patch
applied on the fly (`PatchedFirmware`); a few per-variant quirks (laser-safety
check, negative-distance clamping) are selected the same way. The detected
part is available from `Sensor::variant()` after `init`.

## Usage

Construct from an async I2C bus (or an `SpiDevice` via `Sensor::new_spi`)
and a `DelayNs`, optionally attach LPn and INT pins, then `init` uploads the
firmware. Every call returns `Result<_, Error<BusError>>`.

```rust,ignore
use vl53_8x8::{Resolution, Sensor};

let mut sensor = Sensor::new(i2c, delay)          // or Sensor::new_spi(spi_device, delay)
    .with_reset_pin_active_low(lpn);               // optional
sensor.reset().await?;
sensor.init().await?;                              // detect variant, upload ~86 KB firmware
defmt::info!("found {:?}", sensor.variant());

sensor.set_resolution(Resolution::Res4x4).await?;
sensor.set_ranging_frequency_hz(10).await?;
sensor.start_ranging().await?;

loop {
    sensor.poll_data_ready().await?;               // or wait_for_data_ready() with an INT pin
    let frame = sensor.get_ranging_data().await?;
    // frame.distance_mm: [i16; 64], 8x8 layout; first 16 entries used in 4x4 mode
}
```

- A blocking `embedded_hal::i2c::I2c` can be used through the `BlockingI2c`
  wrapper (for HALs that only offer blocking I2C), and a blocking
  `embedded_hal::spi::SpiDevice` through `BlockingSpi`. Both speak the same
  wire protocol as the async paths; only the transfers block, the driver's
  delays still yield. This is how the driver runs on a blocking HAL without
  an async executor, e.g. `stm32f4xx-hal` under RTIC 2 with the SysTick
  monotonic as the `DelayNs` (`examples/nucleo-f401re-rtic`).
- `with_address` + `redetect` re-attach a handle to a sensor already moved
  with `set_i2c_address`, without re-uploading firmware; see the
  `examples/evo-f412re-embassy` demo for two sensors on one bus.

## Features

| Feature                              | Effect                                                            |
| ------------------------------------ | ----------------------------------------------------------------- |
| `xtalk`                              | Crosstalk calibration for coverglass use (`calibrate_xtalk`, `get/set_caldata_xtalk`, `get/set_xtalk_margin`) |
| `motion`                             | Motion indicator plugin and the `motion_indicator` result field   |
| `thresholds`                         | Per-zone detection thresholds, incl. auto-stop (VL53L8CX only)    |
| `raw-format`                         | Return the sensor's fixed-point units instead of mm / kcps / %    |
| `targets-2`, `targets-3`, `targets-4` | Targets per zone (default 1); mutually exclusive. See below       |
| `disable-*`                          | Drop a result field to shrink frames and `ResultsData`            |
| `defmt`                              | `defmt` trace logging of bus traffic                              |

### Multi-target builds

The `targets-N` features set the ULD's `VL53L8CX_NB_TARGET_PER_ZONE` for the
whole driver, following the C sources:

- `ResultsData` per-target arrays hold `64 * N` entries, indexed
  `zone * NB_TARGET_PER_ZONE + target` (the stride is exported as
  `vl53_8x8::NB_TARGET_PER_ZONE`); frame and scratch-buffer sizes follow.
- The multi-target set of result block headers (`vl53l8cx_api.h`, the
  `#else` branch) is used for both the output list `start_ranging` programs
  and the block indices `get_ranging_data` parses.
- At init, byte 107 of the default configuration (the ULD's
  `VL53L8CX_FW_NBTAR_RANGING` macro, generated for one target) is overwritten
  with N by `sensor::NB_TARGET_CONFIG_PATCH`, and the firmware target count
  DCI (`VL53L8CX_DCI_FW_NB_TARGET`) is written as the ULD does for `N != 1`.

Enabling more than one `targets-N` feature is a compile error. The
single-target build is checked against the C ULD by the oracle tests; the
multi-target constants are checked against the header's literal values.
`targets-2` and `targets-4` have been run on hardware (VL53L5CX, async I2C, and
VL53L8CX, SPI): the full on-target suites pass, and with a two-surface scene
(a clear lid ~20 mm above the sensor and a surface beyond it) the firmware
reports second targets with ST status 13 and first targets with status 12,
as documented for overlapping returns. See `multi_target_scene_report` in
`examples/evo-f412re-embassy/tests/sensor_async.rs`.

## ST data tables and regeneration

`src/vl53l8cx_fw_base.bin` (the firmware image) and `src/uld_data.rs` (default
configuration, default crosstalk shape, NVM-read command) are extracted
byte-for-byte from ST's `vl53l8cx_buffers.h` by `tools/extract_uld_blobs.py`
in the repository. To take a new ULD release:

```sh
# replace the C tree under vl53l8cx-uld-sys/src/VL53L8CX_ULD_API/ first
python3 tools/extract_uld_blobs.py        # rewrite both outputs (--check verifies only)
cargo test -p vl53-8x8                     # oracle tests against the C arrays (needs libclang)
git diff vl53-8x8/src/uld_data.rs          # review the table changes
```

## Hardware verification

Verified with this crate layout, via the demo crates' on-target test suites:

* STM32F401RE Nucleo + X-NUCLEO-53L8A1 shield (VL53L8CX) over SPI and over
  I2C, both true async DMA I2C and the `BlockingI2c` adapter
  (`examples/nucleo-f401re-embassy`, embassy-stm32 0.6 / embassy-executor 0.10).
  The RTIC 2 + `stm32f4xx-hal` 0.23 demo on the same board
  (`examples/nucleo-f401re-rtic`, `BlockingSpi` over SPI) is verified:
  firmware upload, 4x4 ranging at 10 Hz alongside a 1 Hz RTIC task.
* STM32F412RE custom board over async DMA I2C, with both a VL53L8CX and a
  VL53L5CX fitted in turn: autodetect, the patched-firmware upload path and
  the full test suite pass on each (`examples/evo-f412re-embassy`).
* Raspberry Pi Zero 2W (Linux, `linux-embedded-hal`) with a Pololu VL53L8CX
  breakout over `/dev/spidev0.0`, Tokio current-thread (`examples/rpi-zero-2w`).


## License

BSD-3-Clause; see [`LICENSE`](LICENSE).

The firmware image and the data tables in `src/uld_data.rs` are taken from the
official **STMicroelectronics VL53L8CX Ultra Lite Driver**, and the driver
logic is a Rust port of that C code. ST distributes the ULD under the BSD
3-Clause License (see [`LICENSE-ST`](LICENSE-ST) and `LICENSE.txt` in ST's
[STSW-IMG040](https://www.st.com/en/embedded-software/stsw-img040.html)
package).

The shared types this crate re-exports from `vl53-8x8-types` originate from
Scott Wakefield's earlier `vl53l5cx-driver` work and are included under BSD
3-Clause with the original author's permission.
