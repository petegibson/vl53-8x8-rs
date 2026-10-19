# vl53l5cx-uld

A safe, blocking, `no_std` wrapper over STMicroelectronics' **VL53L5CX**
Ultra Lite Driver (ULD) C code, through `embedded-hal` 1.0 I2C or SPI.

The ranging logic, register sequences and firmware image are ST's, compiled
and bound by `vl53l5cx-uld-sys`; this crate adds the Rust API, the platform
callbacks the ULD needs, and typed errors. It is the VL53L5CX twin of
[`vl53l8cx-uld`](../vl53l8cx-uld) and deliberately has the same API shape:
`Vl53l5cx::new` / `new_spi`, `with_lpn_pin`, `with_interrupt_pin_*`,
`with_power_enable_pin_*`, `power_cycle`, `pulse_i2c_rst`, the
same method names, the same `Error` type from `vl53-8x8-types`, and the same
`bind_platform_driver!` / `bind_platform_driver_spi!` macros (defining the
`vl53l5cx_platform_*` symbols).

Use this crate when you want ST's reference implementation for the VL53L5CX
behind an `embedded-hal` interface. If you want an async driver, or one
without a C toolchain, or one that handles both parts, use the pure-Rust
[`vl53-8x8`](../vl53-8x8) crate instead. The two are checked against each
other: the repository's differential tests replay recorded sensor traffic to
both and compare their register transcripts byte for byte.

| Crate              | Role                                                       |
| ------------------ | ---------------------------------------------------------- |
| `vl53l5cx-uld`     | this crate: blocking safe wrapper, `Vl53l5cx`              |
| `vl53l5cx-uld-sys` | bindgen + cc over ST's VL53L5CX ULD sources (needs libclang) |
| `vl53l8cx-uld`     | the same wrapper for the VL53L8CX                          |
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
    .with_lpn_pin(lpn)                              // optional: LPn, I2C comms enable (active low)
    .with_interrupt_pin_active_low(int)             // optional
    .with_power_enable_pin_active_low(pwr_en);      // optional: enables power_cycle(), the real reset
sensor.power_cycle()?;                              // optional; init() reboots the firmware anyway
sensor.init()?;                                     // is_alive probe + ~86 KB firmware upload
// vl53l5cx_uld::pulse_i2c_rst(&mut i2c_rst, &mut delay)?  resets the I2C interface only
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

Verified on a custom STM32F412RE board with a VL53L5CX over I2C
([`examples/evo-f412re-uld`](../examples/evo-f412re-uld): blocking
`stm32f4xx-hal`, `tests/sensor.rs` ported from the L8 Nucleo suite). The demo
ranges, and the on-target suite passes in its default configuration, with the
`thresholds`, `xtalk` and `motion` plugins enabled, and with `targets-2`.
Host-side, the differential tests replay recorded sensor responses through
this wrapper and compare its bus traffic with the pure-Rust `vl53-8x8` driver.

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
