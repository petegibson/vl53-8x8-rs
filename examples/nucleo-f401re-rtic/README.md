# nucleo-f401re-rtic

`vl53-8x8` (the pure-Rust async VL53L5CX/VL53L8CX driver) under **RTIC 2**
with the blocking **`stm32f4xx-hal`** on a NUCLEO-F401RE + X-NUCLEO-53L8A1
shield (VL53L8CX, centre socket).

## What it shows

- The driver's async API running without an async HAL: the bus is the
  blocking `stm32f4xx-hal` I2C or SPI peripheral, wrapped in
  `vl53_8x8::BlockingI2c` / `vl53_8x8::BlockingSpi`. Transfers block (no DMA),
  but every delay inside the driver is an `await` on the SysTick monotonic
  (`rtic_monotonics::systick_monotonic!(Mono, 1000)`, which implements
  `embedded_hal_async::delay::DelayNs`), so the RTIC executor keeps scheduling.
- Two software tasks: `ranging` (priority 1) resets and initialises the
  sensor, logs the detected variant and the firmware-upload time, then ranges
  4x4 at 10 Hz and prints one grid of first-target distances per second; a
  1 Hz `heartbeat` (priority 2) logs `tick N` throughout, including during the
  ~86 KB firmware upload, proving the driver does not hog the executor.
- No interrupt pin: data-ready is polled over the bus (`poll_data_ready`).

The Embassy demo next door (`../nucleo-f401re-embassy`) is the contrast: same
board, same driver, but true async DMA I2C/SPI from `embassy-stm32`.

## Run

```bash
cargo run --release                     # I2C1 @ 400 kHz (PB8/PB9), BlockingI2c
cargo run --release --features spi-bus  # SPI1 @ 1 MHz (PB3/PB4/PB5, NCS PB6), BlockingSpi
```

`probe-rs run` (configured in `.cargo/config.toml`) flashes and streams defmt
until you stop it. From the repository root, `just nucleo-rtic [features]`
does the same with a 25 s timeout.

Plugin features forward to the driver like in the other demos: `xtalk`,
`motion`, `thresholds`, `targets-2..4`, and `defmt-bus-trace` for byte-level
bus logging (raise `DEFMT_LOG` in `.cargo/config.toml` to see it).

## Shield jumpers

The X-NUCLEO-53L8A1 selects the bus with its solder/jumper straps; set them
for the bus you build for. In I2C mode the firmware also drives the shield's
I2C_RST line (PB3) low, which is required for the sensor's I2C interface; in
SPI mode PB3 is SPI1 SCK. The centre sensor's LPn (PB0) is used as the active
low reset; PWR_EN_C (PA7) is cycled at boot like in the other Nucleo demos.

## Expected log

```
INFO  VL53L8CX RTIC demo starting (SPI bus, BlockingSpi adapter)
INFO  Initialising sensor (firmware upload)...
INFO  tick 1
INFO  tick 2
INFO  Sensor ready: L8 after 1xxx ms
INFO  Configuring: 4x4 @ 10 Hz
INFO  Entering measurement loop
INFO  tick 3
INFO  --- Frame 10 distance (mm) ---
INFO    612 598 603 621
...
```
