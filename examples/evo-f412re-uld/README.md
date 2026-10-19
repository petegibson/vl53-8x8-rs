# evo-f412re-uld

> **Note:** this example targets a custom in-house PCB used to test the
> multi-sensor arrangement (two sensors on one I2C bus). It is kept in the
> repository because it is the maintainers' VL53L5CX test hardware for the C
> wrapper. For a board you can buy, see the `nucleo-f401re-*` examples.

**VL53L5CX** on the custom **STM32F412RE** "evo" board through ST's C Ultra
Lite Driver, via the blocking [`vl53l5cx-uld`](../../vl53l5cx-uld) wrapper and
`stm32f4xx-hal` 0.22.

This is the blocking / C-ULD counterpart of
[`../evo-f412re-embassy`](../evo-f412re-embassy) (pure-Rust async `vl53-8x8`
on the same board). Both use the same pins, clock tree and power-on sequence;
only the driver and HAL differ.

**The board must have a VL53L5CX fitted.** The board accepts either a
VL53L5CX or a VL53L8CX module, but ST's VL53L5CX ULD cannot drive an L8:
`init()` fails (`is_alive` rejects the device id). For an L8, or to let the
driver detect the part, use the Embassy sibling.

## What it does

- `src/lib.rs`: `board_init()` sets up the clocks (HSE 8 MHz -> 96 MHz
  SYSCLK), runs the sensor power-on sequence, configures I2C1 and returns a
  `Sensor` (`Vl53l5cx<I2c<I2C1>, SysDelay, Pin<'B', 8, Output>>`), with
  `bind_platform_driver!` giving the C code its bus and delay.
- `src/bin/basic_ranging.rs` (binary `evo-f412re-uld`): `init()` with the
  firmware-upload time logged, then 4x4 @ 10 Hz, one grid of first-target
  distances per second over defmt.
- `tests/sensor.rs`: embedded-test suite, the VL53L5CX port of
  `examples/nucleo-f401re-uld/tests/sensor.rs`.

## Wiring (I2C1 @ 1 MHz, internal pull-ups)

| Signal      | Pin | Notes                                                     |
|-------------|-----|-----------------------------------------------------------|
| SCL         | PB6 | AF4, open drain, internal pull-up                         |
| SDA         | PB7 | AF4, open drain, internal pull-up                         |
| PWR_EN      | PD2 | active **low** (low = sensor rail on)                     |
| I2C_RESET   | PB8 | active **high**; the sensors' I2C_RST (I2C interface reset) |
| LPn (2nd)   | PB9 | gates the optional second sensor; held **low**            |

Clock: HSE 8 MHz crystal -> PLL -> 96 MHz SYSCLK, 48 MHz APB1 (I2C1), 96 MHz
APB2. The STM32F4's I2C block is only specified to 400 kHz, but the Embassy
demo runs this board at 1 MHz, so this one does too; `I2C_FREQ_HZ` in
`src/lib.rs` is the one place to lower it if the bus proves marginal.

The board can carry two sensors on one bus; only the always-on sensor is used
and answers at the default 7-bit address 0x29. PB9 is held low so the second
position stays off the bus. The always-on sensor's LPn is not on a GPIO, so
its reset is the power/I2C_RESET sequence in `board_init`: rail off and reset
asserted for 500 ms, rail on, 100 ms, reset released, 1 ms. `Sensor::reset()`
pulses I2C_RESET for 10 ms (as the Embassy demo's recovery step does).

## Run

```bash
cargo run --release                         # flash + stream defmt (probe-rs, STM32F412RETx)
cargo test --test sensor --release          # on-target suite
cargo test --test sensor --release --features thresholds,xtalk,motion
cargo test --test sensor --release --features targets-2
```

From the repository root, `just run-evo-uld` (25 s timeout) and
`just test-evo-uld [features]` do the same, pinned to the board's ST-Link V3
serial via `PROBE_RS_PROBE`. The runner and `DEFMT_LOG`
(`info,vl53l5cx_uld=trace`) live in `.cargo/config.toml`; enable the
`defmt-bus-trace` feature to see every I2C transfer.

Expected output: the init line with the firmware-upload time (a few hundred
ms at 1 MHz), then `--- Distance (mm), frame N ---` and four rows of four
distances once a second.

## Tests

Common to both ST ULDs: `test_sensor_alive`, `test_init`,
`test_set_resolution`, `test_ranging_4x4`, `test_ranging_8x8`,
`test_ambient_light`, `test_ranging_frequency`, `test_target_order`,
`test_ranging_mode`, `test_integration_time`, `test_sharpener_percent`,
`test_set_resolution_while_ranging`, `test_reinit_after_reset`,
`test_vhv_repeat_count`, `test_set_address`.

VL53L5CX-specific: `test_power_mode` covers Sleep/Wakeup and checks that
`DeepSleep` is rejected with `InvalidParam`; `test_internal_cp_round_trip`
toggles the L5-only internal charge pump and ranges afterwards.

Dropped from the L8 suite because the L5 ULD has no such API: external sync
pin, detection-threshold auto-stop, deep sleep.

Feature-gated: `thresholds` (enable, set/get thresholds, motion indicator
init / distance / resolution), `xtalk` (margin, caldata round-trip). With
`targets-2..4`, result indexing follows `zone * NB_TARGET_PER_ZONE + target`.

Distances are checked against `0 < d < 4000` mm (bench scene, ceiling above
the sensor); ambient only requires some zone to be non-zero.

## Status

Hardware-verified on the board with a VL53L5CX fitted: the demo ranges
(firmware upload about 1.2 s), and the on-target suite passes in its default
configuration (17 tests), with `thresholds,xtalk,motion` (24) and with
`targets-2` (17).

## Features

| Feature            | Effect                                                      |
|--------------------|-------------------------------------------------------------|
| `defmt-bus-trace`  | Byte-level I2C trace from the wrapper (`vl53l5cx-uld/defmt`) |
| `xtalk`, `motion`, `thresholds` | Forwarded to `vl53l5cx-uld` (plugins)          |
| `targets-2..4`     | Targets per zone, at most one; forwarded                    |

Building needs libclang (bindgen over ST's headers) and `arm-none-eabi-gcc`
for the C sources; the repository's Nix flake provides both.
