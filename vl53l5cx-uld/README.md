# vl53l5cx-uld

A safe, blocking, `no_std` wrapper over STMicroelectronics' **VL53L5CX**
Ultra Lite Driver (ULD) C code, through `embedded-hal` 1.0 I2C or SPI.

The ranging logic, register sequences and firmware image are ST's, compiled
and bound by `vl53l5cx-uld-sys`; this crate adds the Rust API, the platform
callbacks the ULD needs, and typed errors. It is the VL53L5CX twin of
[`vl53l8cx-uld`](../vl53l8cx-uld) and deliberately has the same API shape:
`Vl53l5cx::new` / `new_spi`, `with_reset_pin_*`, `with_interrupt_pin_*`, the
same method names, the same `Error` type from `vl53-8x8-types`, and the same
`bind_platform_driver!` / `bind_platform_driver_spi!` macros (defining the
`vl53l5cx_platform_*` symbols).

**Not published.** Its purpose right now is to let host-side differential
tests drive ST's L5 C code against a mock bus alongside the L8 one. If you
want a VL53L5CX driver for an application, use the pure-Rust
[`vl53-8x8`](../vl53-8x8) crate (async, no C toolchain, supports both parts).
The crate is complete enough to publish later; to do so, publish
`vl53l5cx-uld-sys` first and drop the `publish = false` lines.

| Crate              | Role                                                       |
| ------------------ | ---------------------------------------------------------- |
| `vl53l5cx-uld`     | this crate: blocking safe wrapper, `Vl53l5cx`              |
| `vl53l5cx-uld-sys` | bindgen + cc over ST's VL53L5CX ULD sources (needs libclang) |
| `vl53l8cx-uld`     | the same wrapper for the VL53L8CX (published)              |
| `vl53-8x8`         | pure-Rust async driver for VL53L5CX **and** VL53L8CX, no C |
| `vl53-8x8-types`   | shared enums and `Error`; re-exported here                 |

## API parity with `vl53l8cx-uld`

Every `pub fn` of `vl53l8cx-uld` exists here with the same signature, except
where the two ST ULDs differ:

| Only in `vl53l8cx-uld`                        | Only in `vl53l5cx-uld`                     |
| --------------------------------------------- | ------------------------------------------ |
| `get_external_sync_pin_enable`, `set_external_sync_pin_enable` | `enable_internal_cp`, `disable_internal_cp` |
| `get_detection_thresholds_auto_stop`, `set_detection_thresholds_auto_stop` (`thresholds`) | |

Other differences that follow from the ULD headers:

* The VL53L5CX ULD has no deep-sleep power mode. `set_power_mode(PowerMode::DeepSleep)`
  returns `Error::Firmware(FirmwareError::InvalidParam)` without touching the
  bus; `get_power_mode` only ever reports `Sleep` or `Wakeup`.
* Status byte 3 is named `VL53L5CX_STATUS_CRC_CSUM_FAILED` by the L5 ULD
  (L8: `LASER_SAFETY`) and is never returned by the L5 C sources; status 5
  (`FW_CHECKSUM_FAIL`) does not exist. The shared `FirmwareError` decoder is
  used unchanged.

## Usage

Identical to `vl53l8cx-uld` with the crate and type names swapped:

```rust,ignore
use vl53l5cx_uld::{Resolution, Vl53l5cx};

// Once per binary. Concrete HAL types; `MyDelay: embedded_hal::delay::DelayNs`.
vl53l5cx_uld::bind_platform_driver!(MyI2c, MyDelay);
// SPI instead:  vl53l5cx_uld::bind_platform_driver_spi!(MySpiDevice, MyDelay);

let mut sensor = Vl53l5cx::new(i2c, delay)         // or Vl53l5cx::new_spi(spi_device, delay)
    .with_reset_pin_active_low(lpn)                 // optional
    .with_interrupt_pin_active_low(int);            // optional
sensor.reset()?;
sensor.init()?;                                     // is_alive probe + ~86 KB firmware upload
sensor.set_resolution(Resolution::Res4x4)?;
sensor.start_ranging()?;
while !sensor.check_data_ready()? {}
let frame = sensor.get_ranging_data()?;             // ResultsData, see vl53l8cx-uld README
```

Note that a binary can only bind one set of platform symbols per ULD, but the
L5 (`vl53l5cx_platform_*`) and L8 (`vl53l8cx_platform_*`) sets are distinct,
so both wrappers can be linked into the same test binary.

## Features

Same names and meaning as `vl53l8cx-uld`: `xtalk`, `motion`, `thresholds`,
`raw-format`, `disable-*`, `defmt` (bus-trace logging of I2C and SPI traffic)
and `targets-1` .. `targets-4` (`VL53L5CX_NB_TARGET_PER_ZONE`; at most one,
two at once is a build error). The selected value is exported as
`NB_TARGET_PER_ZONE`.

## Build requirements

`vl53l5cx-uld-sys` runs `bindgen` and `cc`: you need **libclang** (set
`LIBCLANG_PATH` if it is not found) and, for bare-metal targets, a C cross
compiler. The repository's Nix flake sets this up.

## Hardware verification

None yet through this wrapper; there is no VL53L5CX board wired for the C
driver. The pure-Rust `vl53-8x8` driver is the hardware-tested VL53L5CX path.

## License

BSD-3-Clause; see [`LICENSE`](LICENSE).

The ULD C code and firmware this crate links are the official
**STMicroelectronics VL53L5CX Ultra Lite Driver**, distributed by ST under the
BSD 3-Clause License (see [`LICENSE-ST`](LICENSE-ST), the per-file
copyright headers, and `LICENSE.txt` in ST's
[STSW-IMG023](https://www.st.com/en/embedded-software/stsw-img023.html)
package).

The Rust wrapper originates from Scott Wakefield's earlier `vl53l5cx-driver`
work and is included here under BSD 3-Clause with the original author's
permission.
