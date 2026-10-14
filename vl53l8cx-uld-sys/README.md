# vl53l8cx-uld-sys

Raw FFI bindings to the STMicroelectronics **VL53L8CX** Ultra Lite Driver
(ULD). ST's C sources under `src/VL53L8CX_ULD_API/` are compiled with `cc`
and bound with `bindgen` at build time; the ~86 KB firmware image and the
configuration tables are linked in as static C arrays.

You normally want one of the crates built on top of it:

| Crate              | Role                                                         |
| ------------------ | ------------------------------------------------------------ |
| `vl53l8cx-uld-sys` | this crate: `vl53l8cx_*` functions, `VL53L8CX_*` types/consts |
| `vl53l8cx-uld`     | safe, blocking `embedded-hal` wrapper over this crate        |
| `vl53-8x8`         | pure-Rust async driver (VL53L5CX + VL53L8CX); uses this crate only as a test oracle |
| `vl53-8x8-types`   | shared enums and `Error`; re-exported here                   |

## Build requirements

`bindgen` needs **libclang** on the build host. Install clang/libclang and,
if it is not found automatically, point `LIBCLANG_PATH` at the directory
containing `libclang.so`. For bare-metal `thumbv*` targets the build script
also queries the C cross compiler (`CC_<target>`, as used by the `cc` crate,
e.g. `arm-none-eabi-gcc`) for its include paths, and for `armv7`/`aarch64`
Linux cross builds it passes `--target` to clang. The repository's Nix flake
sets all of this up.

If you cannot or do not want to build C at all, use `vl53-8x8` instead; it
has no such requirement.

## What the crate exposes

- The bindgen output: `vl53l8cx_init`, `vl53l8cx_start_ranging`, ... and
  `VL53L8CX_Configuration`, `VL53L8CX_ResultsData`, plugin structs.
- `wrappers`: thin safe functions returning the ULD status byte, e.g.
  `wrappers::init(&mut cfg) -> u8`, `wrappers::get_ranging_data(&mut cfg) ->
  (u8, VL53L8CX_ResultsData)`.
- The static data arrays `VL53L8CX_FIRMWARE`, `VL53L8CX_DEFAULT_CONFIGURATION`,
  `VL53L8CX_DEFAULT_XTALK`, `VL53L8CX_GET_NVM_CMD`.
- Constants bindgen skips (status codes, DCI addresses, ...), defined by hand.
- A re-export of everything in `vl53-8x8-types`.

The ULD is built with `USE_ULP_PLATFORM` and calls back into Rust for bus
access and delays; the program must provide those platform functions. The
`vl53l8cx-uld` crate does this through its `bind_platform_driver!` macros,
which is the recommended way to use these bindings.

```rust,ignore
use vl53l8cx_uld_sys::{wrappers, VL53L8CX_Configuration, VL53L8CX_STATUS_OK};

let mut cfg = VL53L8CX_Configuration::default();   // platform field set up by your shim
assert_eq!(wrappers::init(&mut cfg), VL53L8CX_STATUS_OK);
let (status, frame) = wrappers::get_ranging_data(&mut cfg);
```

## Features

Each feature maps to a ULD compile-time macro, applied to both the C build and
the bindgen pass so the Rust and C struct layouts stay in step.

| Feature                           | ULD macro                               |
| --------------------------------- | --------------------------------------- |
| `xtalk`                           | `VL53L8CX_USE_XTALK` + xtalk plugin     |
| `motion`, `thresholds`            | `VL53L8CX_USE_MOTION_INDICATOR` + motion/threshold plugins |
| `raw-format`                      | `VL53L8CX_USE_RAW_FORMAT`               |
| `disable-ambient`                 | `VL53L8CX_DISABLE_AMBIENT_PER_SPAD`     |
| `disable-spads`                   | `VL53L8CX_DISABLE_NB_SPADS_ENABLED`     |
| `disable-nb-targets`              | `VL53L8CX_DISABLE_NB_TARGET_DETECTED`   |
| `disable-signal`                  | `VL53L8CX_DISABLE_SIGNAL_PER_SPAD`      |
| `disable-range-sigma`             | `VL53L8CX_DISABLE_RANGE_SIGMA_MM`       |
| `disable-distance`                | `VL53L8CX_DISABLE_DISTANCE_MM`          |
| `disable-target-status`           | `VL53L8CX_DISABLE_TARGET_STATUS`        |
| `disable-reflectance`             | `VL53L8CX_DISABLE_REFLECTANCE_PERCENT`  |
| `disable-motion-indicator-output` | `VL53L8CX_DISABLE_MOTION_INDICATOR`     |
| `targets-1` .. `targets-4`        | `VL53L8CX_NB_TARGET_PER_ZONE=N` (default 1; at most one, else `build.rs` panics) |

Without `motion` the `motion_indicator` result field is removed from the C
struct as well, so `VL53L8CX_ResultsData` is only a few hundred bytes.
The selected target count is exported as the Rust constant
`VL53L8CX_NB_TARGET_PER_ZONE`; per-target result arrays hold `64 * N` entries
indexed `zone * N + target`.

## Hardware verification

Verified with this crate layout through `vl53l8cx-uld` on an STM32F401RE
Nucleo with the X-NUCLEO-53L8A1 shield over SPI (`nucleo-f401re` demo and
on-target tests).

## License

The Rust parts are BSD-3-Clause; see [`LICENSE`](LICENSE). The C sources and
data under `src/VL53L8CX_ULD_API/` are STMicroelectronics' VL53L8CX Ultra
Lite Driver (STSW-IMG040, ULD 2.0.0), redistributed under ST's BSD-3-Clause
terms in [`LICENSE-ST`](LICENSE-ST). They are included verbatim apart from
line-ending normalisation and one missing-semicolon fix in `vl53l8cx_api.c`.
