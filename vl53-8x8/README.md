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
  wrapper (useful where the HAL's async I2C is unreliable).
- `with_address` + `redetect` re-attach a handle to a sensor already moved
  with `set_i2c_address`, without re-uploading firmware; see the
  `evo-f412re-async` demo for two sensors on one bus.

## Features

| Feature                              | Effect                                                            |
| ------------------------------------ | ----------------------------------------------------------------- |
| `xtalk`                              | Crosstalk calibration (`calibrate_xtalk`, `get/set_caldata_xtalk`) |
| `motion`                             | Motion indicator plugin and the `motion_indicator` result field   |
| `thresholds`                         | Per-zone detection thresholds                                     |
| `raw-format`                         | Return the sensor's fixed-point units instead of mm / kcps / %    |
| `targets-2`, `targets-3`, `targets-4` | Targets per zone (default 1). **Untested on hardware**, see below |
| `disable-*`                          | Drop a result field to shrink frames and `ResultsData`            |
| `defmt`                              | `defmt` trace logging of bus traffic                              |

### Multi-target builds

The `targets-N` features size `ResultsData` and the result parser for N
targets per zone, but the generated default-configuration table
(`src/uld_data.rs`) is produced for the **single-target** ULD build: the
ULD's `VL53L8CX_FW_NBTAR_RANGING` macro inside that table is resolved to its
one-target value. Multi-target operation has not been exercised on hardware
and the oracle layout tests only run under the default feature set. Treat
`targets-2/3/4` as experimental.

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

* STM32F401RE Nucleo + X-NUCLEO-53L8A1 shield (VL53L8CX) over SPI
  (`nucleo-f401re-async`).
* STM32F412RE custom board (VL53L8CX) over async DMA I2C, including the
  firmware upload path (`evo-f412re-async`).

VL53L5CX support is implemented and was validated on hardware before this
crate layout; re-verification with the current layout is pending.

## License

BSD-3-Clause; see [`LICENSE`](LICENSE). The firmware image and the data
tables in `src/uld_data.rs` are STMicroelectronics' VL53L8CX Ultra Lite
Driver data, redistributed under ST's BSD-3-Clause terms in
[`LICENSE-ST`](LICENSE-ST).
