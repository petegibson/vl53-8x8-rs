# vl53l8cx-uld

A safe, blocking, `no_std` driver for the STMicroelectronics **VL53L8CX** 8x8
multizone Time-of-Flight sensor, wrapping ST's own Ultra Lite Driver (ULD) C
code through `embedded-hal` 1.0 I2C or SPI.

The ranging logic, register sequences and firmware image are ST's, compiled
and bound by `vl53l8cx-uld-sys`; this crate adds the Rust API, the
platform callbacks the ULD needs, and typed errors.

| Crate              | Role                                                          |
| ------------------ | ------------------------------------------------------------- |
| `vl53l8cx-uld`     | this crate: blocking safe wrapper, `Vl53l8cx`                 |
| `vl53l8cx-uld-sys` | bindgen + cc over the ULD C sources (needs libclang)          |
| `vl53-8x8`         | pure-Rust async driver for VL53L5CX **and** VL53L8CX, no C    |
| `vl53-8x8-types`   | shared enums and `Error`; re-exported here                    |

If you need async, VL53L5CX support, or a build with no C toolchain, use
`vl53-8x8` instead. This crate supports the VL53L8CX only.

## Build requirements

Building pulls in `vl53l8cx-uld-sys`, which runs `bindgen` and `cc`: you need
**libclang** (set `LIBCLANG_PATH` if it is not found) and, for bare-metal
targets, a C cross compiler such as `arm-none-eabi-gcc`. See the
`vl53l8cx-uld-sys` README for details.

## Usage

The ULD calls back into Rust for bus access and delays, so a program must
invoke `bind_platform_driver!` (I2C) or `bind_platform_driver_spi!` (SPI)
**exactly once**, at crate level, with its concrete bus and delay types.
Then construct a `Vl53l8cx`, optionally attach LPn and INT pins, and `init`.

```rust,ignore
use vl53l8cx_uld::{Resolution, Vl53l8cx};

// Once per binary. Concrete HAL types; `MyDelay: embedded_hal::delay::DelayNs`.
vl53l8cx_uld::bind_platform_driver!(MyI2c, MyDelay);
// SPI instead:  vl53l8cx_uld::bind_platform_driver_spi!(MySpiDevice, MyDelay);

let mut sensor = Vl53l8cx::new(i2c, delay)         // or Vl53l8cx::new_spi(spi_device, delay)
    .with_reset_pin_active_low(lpn)                 // optional
    .with_interrupt_pin_active_low(int);            // optional
sensor.reset()?;
sensor.init()?;                                     // is_alive probe + ~86 KB firmware upload

sensor.set_resolution(Resolution::Res4x4)?;
sensor.set_ranging_frequency_hz(10)?;
sensor.start_ranging()?;

loop {
    if sensor.check_data_ready()? {                 // or wait_for_data_ready_blocking() with INT
        let frame = sensor.get_ranging_data()?;
        // frame.distance_mm: [i16; 64 * NB_TARGET_PER_ZONE], indexed
        // zone * NB_TARGET_PER_ZONE + target; first 16 zones used in 4x4 mode
    }
}
```

`ResultsData` (ST's `VL53L8CX_ResultsData`) has two array layouts. Per-zone
arrays (`ambient_per_spad`, `nb_target_detected`, `nb_spads_enabled`) have 64
entries indexed by zone. Per-target arrays (`distance_mm`, `target_status`,
`signal_per_spad`, `range_sigma_mm`, `reflectance`) have
`64 * NB_TARGET_PER_ZONE` entries indexed `zone * NB_TARGET_PER_ZONE + target`,
where `vl53l8cx_uld::NB_TARGET_PER_ZONE` is 1 by default and set by the
`targets-N` features. Only the first `nb_target_detected[zone]` targets of a
zone hold data; `target_status` 5 or 9 marks a valid range.

All methods return `Result<_, Error<BusError>>`. Configuration setters and
getters cover resolution, frequency, ranging mode, integration time, sharpener,
target order, power mode, VHV repeat count and the external sync pin.

## Features

| Feature                               | Effect                                                              |
| ------------------------------------- | ------------------------------------------------------------------- |
| `xtalk`                               | Crosstalk calibration (`calibrate_xtalk`, `get/set_caldata_xtalk`, margin) |
| `motion`                              | Motion indicator plugin (`motion_indicator_*`)                      |
| `thresholds`                          | Detection thresholds (`get/set_detection_thresholds`, enable, auto-stop) |
| `raw-format`                          | Return fixed-point sensor units instead of mm / kcps / %            |
| `disable-*`                           | Drop a result field from `ResultsData` (forwarded to the C build)   |
| `defmt`                               | `defmt` trace logging of bus traffic                                |
| `targets-1` .. `targets-4`            | Set `VL53L8CX_NB_TARGET_PER_ZONE` for the C build and bindings (default 1). Enable **at most one**; two at once is a build error. Per-target result arrays grow to `64 * N` entries indexed `zone * NB_TARGET_PER_ZONE + target` |

With `thresholds`, `set_detection_thresholds_auto_stop(true)` makes the sensor
raise INT and abort the measurement in progress as soon as a threshold is
crossed (autonomous mode), instead of after the measurement completes; call
`stop_ranging` afterwards for a clean stop.

## Hardware verification

Verified with this crate layout:

* STM32F401RE Nucleo with the X-NUCLEO-53L8A1 shield over SPI, via the
  `examples/nucleo-f401re-uld` demo and its on-target tests.
* Raspberry Pi Zero 2W (Linux, `linux-embedded-hal`) with a Pololu VL53L8CX
  breakout over `/dev/spidev0.0` (`examples/rpi-zero-2w`, blocking demo).

## License

BSD-3-Clause; see [`LICENSE`](LICENSE).

The ULD C code and firmware this crate links are the official
**STMicroelectronics VL53L8CX Ultra Lite Driver**, distributed by ST under the
BSD 3-Clause License (see `LICENSE-ST` in `vl53l8cx-uld-sys`, the per-file
copyright headers, and `LICENSE.txt` in ST's
[STSW-IMG040](https://www.st.com/en/embedded-software/stsw-img040.html)
package).

The Rust wrapper originates from Scott Wakefield's earlier `vl53l5cx-driver`
work and is included here under BSD 3-Clause with the original author's
permission.
