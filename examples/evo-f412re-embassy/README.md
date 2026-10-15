# evo-f412re-embassy

Async VL53L5CX / VL53L8CX ranging demo for a custom **STM32F412RE** board.

The driver autodetects which sensor variant is fitted (from the silicon
revision id) and uploads the matching patched firmware, so the same binary runs
whether the board has a VL53L5CX or a VL53L8CX.

## Wiring (I2C1, fast-mode-plus @ 1 MHz, internal pull-ups)

| Signal      | Pin       | Notes                                   |
|-------------|-----------|-----------------------------------------|
| SCL         | PB6       |                                         |
| SDA         | PB7       |                                         |
| TX DMA      | DMA1_CH7  |                                         |
| RX DMA      | DMA1_CH0  |                                         |
| PWR_EN      | PD2       | active **low** (low = sensor powered)   |
| I2C_RESET   | PB8       | active **high** (high = reset asserted) |
| LPn (2nd)   | PB9       | gates the optional second sensor        |

Clock: HSE 8 MHz crystal → PLL → 96 MHz SYSCLK.

The board can carry two sensors on one bus; only the always-on sensor answers at
the default address (0x52). PB9 gates the second sensor and is held low so a
single sensor enumerates cleanly. If your sensor is the second position, drive
PB9 high after the power-up sequence in `main.rs`.

## Run

```
cargo run --release
```

Requires a debug probe and `probe-rs` (configured as the runner for
`STM32F412RETx`). Flashes directly to flash origin — no bootloader required.
