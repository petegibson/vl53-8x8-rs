# vl53l7cx-uld-sys

Raw FFI bindings to STMicroelectronics' **VL53L7CX Ultra Lite Driver** (ULD
2.0.1, package STSW-IMG036), compiled with `cc` and bound with `bindgen` at
build time.

This is not a user-facing driver. It has two roles:

* It is the FFI layer under [`vl53l7cx-uld`](../vl53l7cx-uld), the blocking
  safe wrapper that mirrors `vl53l8cx-uld` and `vl53l5cx-uld`. It carries the
  same `wrappers` module, `targets-N` features and `vl53-8x8-types` re-export
  as those crates' sys layers.
* It is a *test oracle* for the pure-Rust [`vl53-8x8`](../vl53-8x8) driver.
  That driver reports a VL53L7CX as its own variant but drives it with the
  VL53L5CX tables, because ST's L7CX ULD is byte-identical to the L5CX ULD in
  its firmware image, default configuration, xtalk table and NVM command. The
  tests in `vl53-8x8` assert exactly that against the arrays exported here
  (`VL53L7CX_FIRMWARE`, `VL53L7CX_DEFAULT_CONFIGURATION`, ...), so an ST
  release that gives the L7 its own bytes fails the build rather than a
  sensor in the field.

For `vl53-8x8` it is a path-only dev-dependency, which Cargo strips from the
published manifest, so downstream users of `vl53-8x8` never build it and
never need libclang.

The VL53L5CX and VL53L8CX equivalents are
[`vl53l5cx-uld-sys`](../vl53l5cx-uld-sys) and
[`vl53l8cx-uld-sys`](../vl53l8cx-uld-sys). Compared with the L8 ULD, the L7
ULD (like the L5's) has no deep-sleep power mode, no external sync pin and no
detection-threshold auto-stop, but has `vl53l7cx_enable_internal_cp` /
`vl53l7cx_disable_internal_cp`; its header defines no status code 3 or 5
(L8: `LASER_SAFETY`, `FW_CHECKSUM_FAIL`). Compared with the L5 ULD its C
sources differ only in comments, one declaration order and the threshold
scaling loop in `set_detection_thresholds` (`p_thresholds->measurement`,
as in the L8 ULD, where the L5 ULD indexes per threshold).

## Local changes to ST's sources

`src/VL53L7CX_ULD_API/` is STSW-IMG036 2.0.1 verbatim (CRLF line endings
normalised) with one fix: `inc/vl53l7cx_api.h` declares the internal
charge-pump functions as `vl53ll7cx_enable_internal_cp` /
`vl53ll7cx_disable_internal_cp` (a typo in ST's header; the `.c` defines
`vl53l7cx_*`), corrected here so that bindgen binds the symbols that exist.
`inc/platform.h` is this crate's own, routing the ULD's platform calls to the
Rust callbacks defined by `vl53l7cx-uld`'s `bind_platform_driver!` macros.

## Build requirements

`bindgen` needs libclang (`LIBCLANG_PATH`). The repository's `flake.nix` dev
shell provides it.

## License

BSD-3-Clause. The ULD C sources under `src/VL53L7CX_ULD_API/` are
copyright STMicroelectronics, BSD-3-Clause (see `LICENSE-ST`). The Rust
build glue originates from Scott Wakefield's `vl53l5cx-sys` crate,
relicensed under BSD-3-Clause with the author's permission.
