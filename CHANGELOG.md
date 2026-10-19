# Changelog

All notable changes to the published crates (`vl53-8x8`, `vl53l8cx-uld`,
`vl53l5cx-uld`, `vl53-8x8-types`, `vl53l8cx-uld-sys`, `vl53l5cx-uld-sys`).
The three drivers share one API shape and are versioned together.

## 0.2.0 (unreleased)

Drivers: `vl53-8x8`, `vl53l8cx-uld`, `vl53l5cx-uld` 0.2.0. Types:
`vl53-8x8-types` 0.1.1 (compatible). `vl53l8cx-uld-sys` / `vl53l5cx-uld-sys`
unchanged at 0.1.0.

### Pin semantics corrected

The 0.1 API called the sensor's **LPn** line a "reset pin". It is not one:
the datasheet names it "Comms enable" (active low). Low disables the sensor's
I2C interface, high enables it, and its purpose is multi-sensor address
assignment (UM2884 §4.1). Pulsing it never reset the sensor. The VL53L5CX's
**I2C_RST** (active high) resets the I2C interface only; the VL53L8CX has
SPI_I2C_N instead. The sensor's only real reset is a supply cycle: VDDIO, AVDD
and LPn low, 10 ms, high (UM2884 §4.2 / UM3109). `init()` always runs ST's
software reboot sequence, so it needs no hardware reset at all.

### API changes (all three drivers, identical shape)

- `Sensor<BUS, D, LPN = NoPin, INT = NoPin, PWR = NoPin>` (and `Vl53l8cx` /
  `Vl53l5cx`): the `RST` type parameter is now `LPN`, and a defaulted `PWR`
  parameter is appended, so `Sensor<I2c, Delay, Output, Input>` aliases still
  compile.
- New `with_lpn_pin(pin)` (always active low) with `lpn_enable()` /
  `lpn_disable()`.
- New `with_power_enable_pin_active_high(pin)` /
  `with_power_enable_pin_active_low(pin)` with `power_on()`, `power_off()` and
  `power_cycle()`, which implements the UM2884 §4.2 reset (power off, LPn low
  if a GPIO holds it, 10 ms, power on, LPn high, 10 ms settle). It compiles
  with or without an LPn pin through the new `vl53_8x8_types::MaybeOutputPin`
  (implemented by every `OutputPin` and, as a no-op, by `NoPin`).
- New free function `pulse_i2c_rst(&mut pin, &mut delay)` (async in
  `vl53-8x8`, blocking in the wrappers): high, 10 ms, low on the VL53L5CX's
  I2C_RST.
- `set_i2c_address` in the two C wrappers no longer requires a pin on the
  handle (it was in the `OutputPin`-bounded impl block although it never used
  the pin).
- `VL53L8CX_LAST_THRESHOLD` and `MaybeOutputPin` are re-exported by the
  drivers.

### Deprecated (still work)

- `with_reset_pin_active_low(pin)` -> `with_lpn_pin(pin)`.
- `with_reset_pin_active_high(pin)`: attaches an inverted line in the LPn
  slot; use `pulse_i2c_rst` for I2C_RST, or `with_lpn_pin` with your own
  inverting pin.
- `reset()`: pulses the LPn slot exactly as before (honouring the inversion)
  and still does not reset the sensor; use `power_cycle()` or just `init()`.

### Fixed

- `motion_indicator_set_distance_motion` now range-checks
  `distance_min_mm <= distance_max_mm` before subtracting and returns
  `FirmwareError::InvalidParam`; `(1500, 400)` used to overflow the `u16`
  subtraction and panic in debug builds (`vl53-8x8`), or reach ST's ULD, which
  lets it through and computes garbage (wrappers). Bytes on the wire for valid
  inputs are unchanged.
- `bind_platform_driver!` / `bind_platform_driver_spi!` (both C wrappers) no
  longer expand `#[cfg(feature = "defmt")]` and `defmt::trace!` into the
  calling crate. The bus-trace hooks now live in the wrapper crate
  (`__trace`, hidden), so the wrapper's own `defmt` feature controls tracing,
  the user needs neither a `defmt` dependency nor a `defmt` feature of their
  own, and crates without one no longer get an `unexpected_cfgs` warning
  (seen in the Pi Zero 2W example). Trace format unchanged; verified by
  re-recording the Nucleo C suite and checking it byte for byte against the
  committed transcript.

### Detection thresholds

ST's header requires `VL53L8CX_LAST_THRESHOLD` (128) OR'd into the `zone_num`
of the last threshold entry used. The drivers still pass the array through
byte-identically to ST's C (a missing flag is not detected); this is now
documented on `set_detection_thresholds` and `DetectionThreshold`, the
on-target tests set the flag, and the differential sequence does too (its
recordings must be re-taken; the three `test_detection_thresholds` checks are
ignored until then).

### Docs

- Every public method documented from UM2884 / UM3109 and the ULD headers
  (units, ranges, defaults, ordering rules); defaults and units on the shared
  types; the 600 mm multi-target separation rule.
- Examples and READMEs migrated to the new pin API; the Nucleo examples attach
  `PWR_EN_C` and power-cycle through the driver, the evo C-ULD example attaches
  `PWR_EN` (active low) and pulses I2C_RST with `pulse_i2c_rst`, the Pi example
  drives LPn high with `lpn_enable()`. `test_reinit_after_reset` is now
  `test_reinit_after_power_cycle` in every on-target suite, and
  `test_set_resolution_while_ranging` became
  `test_set_resolution_between_ranging_sessions`: changing the resolution
  mid-stream is not a supported operation and once left the firmware reporting
  GO2 status 0xc5 at the following stop.
- Differential-test recordings of the Nucleo L8 suites re-taken with the 0.2.0
  sequences (`VL53L8CX_LAST_THRESHOLD` set, power cycle, new test names).

## 0.1.0 — 2026-10-16

Initial release on crates.io.

- `vl53-8x8`: pure-Rust, `no_std`, async `embedded-hal` 1.0 driver for the
  VL53L5CX and VL53L8CX, with runtime variant detection and one shared firmware
  image patched per part (the two differ by seven bytes); I2C and SPI, blocking
  bus adapters (`BlockingI2c`, `BlockingSpi`), `xtalk` / `motion` /
  `thresholds` plugins, `targets-N`, `raw-format` and `disable-*` features.
- `vl53l8cx-uld` and `vl53l5cx-uld`: safe blocking wrappers over ST's VL53L8CX
  and VL53L5CX Ultra Lite Drivers (`vl53l8cx-uld-sys`, `vl53l5cx-uld-sys`:
  bindgen + cc), same API shape as `vl53-8x8`.
- `vl53-8x8-types`: shared enums, `Error`, plugin types and ULD constants.
- Host-side differential tests replaying recorded sensor traffic through all
  three drivers; on-target suites for NUCLEO-F401RE + X-NUCLEO-53L8A1
  (Embassy, RTIC 2, `stm32f4xx-hal`), a custom STM32F412RE board and a
  Raspberry Pi Zero 2W.
