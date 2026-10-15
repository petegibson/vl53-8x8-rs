# vl53l5cx-uld-sys

Raw FFI bindings to STMicroelectronics' **VL53L5CX Ultra Lite Driver** (ULD
2.0.x, package STSW-IMG023), compiled with `cc` and bound with `bindgen` at
build time.

**This crate is not published and is not a user-facing driver.** The
pure-Rust [`vl53-8x8`](../vl53-8x8) crate drives the VL53L5CX (and VL53L8CX)
natively. `vl53l5cx-uld-sys` has two in-repo roles:

* It is the FFI layer under [`vl53l5cx-uld`](../vl53l5cx-uld), the blocking
  safe wrapper that mirrors `vl53l8cx-uld` and lets host-side differential
  tests drive ST's L5 C code against a mock bus. For that it carries the same
  `wrappers` module, `targets-N` features and `vl53-8x8-types` re-export as
  `vl53l8cx-uld-sys`.
* It is a *test oracle* for `vl53-8x8`, which builds the VL53L8CX firmware
  image and default configuration into the VL53L5CX ones by applying small
  byte patches; the unit tests compare the patched results against ST's actual
  VL53L5CX arrays (`VL53L5CX_FIRMWARE`, `VL53L5CX_DEFAULT_CONFIGURATION`, ...)
  byte for byte, and check that the register map really is shared between the
  two parts.

It is a path-only dev-dependency of `vl53-8x8`, which Cargo strips from the
published manifest, so downstream users of `vl53-8x8` never build it and
never need libclang.

For the VL53L8CX the equivalent crate, [`vl53l8cx-uld-sys`](../vl53l8cx-uld-sys),
*is* published because the blocking wrapper `vl53l8cx-uld` depends on it.
Differences between the two ULDs visible here: the L5 has no deep-sleep power
mode, no external sync pin and no detection-threshold auto-stop, but has
`vl53l5cx_enable_internal_cp` / `vl53l5cx_disable_internal_cp`; status code 3
is `VL53L5CX_STATUS_CRC_CSUM_FAILED` (L8: `LASER_SAFETY`) and code 5
(`FW_CHECKSUM_FAIL`) does not exist.

## Build requirements

`bindgen` needs libclang (`LIBCLANG_PATH`). The repository's `flake.nix` dev
shell provides it.

## License

BSD-3-Clause. The ULD C sources under `src/VL53L5CX_ULD_API/` are
copyright STMicroelectronics, BSD-3-Clause (see `LICENSE-ST`). The Rust
build glue originates from Scott Wakefield's `vl53l5cx-sys` crate,
relicensed under BSD-3-Clause with the author's permission.
