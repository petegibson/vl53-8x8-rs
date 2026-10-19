# vl53-8x8-types

Shared types for the Rust drivers of the STMicroelectronics
**VL53L5CX** and **VL53L8CX** 8x8 multizone Time-of-Flight sensors.

This crate exists so that the two driver crates in the family agree on one
set of enums, status codes and one error type:

| Crate              | Role                                                       |
| ------------------ | ---------------------------------------------------------- |
| `vl53-8x8-types`   | this crate: enums, constants, `Error`; `no_std`           |
| `vl53-8x8`         | pure-Rust async driver for VL53L5CX and VL53L8CX           |
| `vl53l8cx-uld-sys` | bindgen + cc over ST's VL53L8CX ULD C sources              |
| `vl53l8cx-uld`     | safe blocking wrapper over `vl53l8cx-uld-sys`              |

Both drivers re-export everything here, so you normally do not depend on this
crate directly. Depend on it only when you write code that must accept either
driver's values (e.g. a shared `Error` handler or a configuration struct).

## What is in it

- Configuration enums: `Resolution`, `RangingMode`, `TargetOrder`, `PowerMode`.
- `Error<E>` (bus, `NotAlive`, `UnknownRevision`, `Firmware`, `Pin`) with
  `core::fmt::Display` and `core::error::Error` impls, and `FirmwareError`
  decoded from the ULD status codes.
- `NoPin`, the placeholder for an absent LPn / INT / power-enable pin, and
  `MaybeOutputPin`, implemented by every `embedded-hal` `OutputPin` and (as a
  no-op) by `NoPin`, so driver methods such as `power_cycle()` compile with
  or without an LPn pin attached.
- `DEFAULT_ADDRESS` (7-bit, `0x29`) and the ULD's `VL53L8CX_*` status,
  resolution, ranging-mode and power-mode constants.
- Behind feature flags: the ULD plugin types `MotionConfiguration`,
  `DetectionThreshold`, `ThresholdMeasurement`, `ThresholdType`,
  `ThresholdOperation` and their constants.

```rust
use vl53_8x8_types::{Error, FirmwareError, Resolution};

fn describe<E: core::fmt::Debug>(e: &Error<E>) -> &'static str {
    match e {
        Error::Bus(_) => "bus",
        Error::NotAlive => "no sensor at address",
        Error::UnknownRevision(_) => "unrecognised VL53L part",
        Error::Firmware(FirmwareError::Timeout) => "firmware timeout",
        Error::Firmware(_) => "firmware",
        Error::Pin => "GPIO",
    }
}

let _ = (describe::<()>(&Error::NotAlive), Resolution::Res8x8);
```

## Features

The crate has no code of its own behind these; they only gate the types that
belong to the matching ULD plugin, and the driver crates forward them.

| Feature      | Gates                                                        |
| ------------ | ------------------------------------------------------------ |
| `xtalk`      | `VL53L8CX_GET_XTALK_CMD` (crosstalk calibration command)      |
| `motion`     | `MotionConfiguration`                                        |
| `thresholds` | `DetectionThreshold`, `Threshold*` enums, `thresholds` consts |

`no_std`, no build script; the only dependency is `embedded-hal` 1.0 (for the
`OutputPin` bound of `MaybeOutputPin`). Requires Rust 1.81 or later
(`core::error::Error`).

## License

BSD-3-Clause; see [`LICENSE`](LICENSE).

The constants and struct layouts mirror the headers of the official
**STMicroelectronics VL53L8CX Ultra Lite Driver**, which ST distributes under
the BSD 3-Clause License ([STSW-IMG040](https://www.st.com/en/embedded-software/stsw-img040.html)).

The hand-written Rust types originate from Scott Wakefield's earlier
`vl53l5cx-driver` work and are included here under BSD 3-Clause with the
original author's permission.
