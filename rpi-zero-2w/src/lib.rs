//! Shared helpers for driving a Pololu VL53L8CX breakout from a Raspberry
//! Pi Zero 2W over SPI0.
//!
//! Wiring (Pi BCM pin → Pololu silkscreen):
//!
//! | Pi BCM | Header | Pololu pin   | Notes                       |
//! | ------ | ------ | ------------ | --------------------------- |
//! |  3V3   |    1   | VIN          | sensor power                |
//! |  GND   |    6   | GND          |                             |
//! |  GPIO11| 23     | SCL          | SPI0 SCLK                   |
//! |  GPIO10| 19     | SDA          | SPI0 MOSI                   |
//! |  GPIO9 | 21     | MISO         | SPI0 MISO                   |
//! |  GPIO8 | 24     | NCS          | SPI0 CE0 (managed by spidev)|
//! |  GPIO17| 11     | LPn          | sensor reset (active low)   |
//!
//! On the Pololu breakout, set the SPI/I2C selector for SPI mode (the
//! board ships in I2C by default; UM2884-style jumper or solder pad).
//!
//! Enable SPI0 on the Pi: add `dtparam=spi=on` to `/boot/firmware/config.txt`
//! (or `/boot/config.txt` on older Raspberry Pi OS images) and reboot. The
//! `/dev/spidev0.0` device should appear and the user must be in the `spi`
//! and `gpio` groups (or run as root).

use std::time::Duration;

use linux_embedded_hal::gpio_cdev::{Chip, LineRequestFlags};
use linux_embedded_hal::spidev::{SpiModeFlags, SpidevOptions};
use linux_embedded_hal::{CdevPin, SpidevDevice};

/// SPI bus device. The BCM2835 SPI driver on this Pi/NixOS combo doesn't
/// honour `SPI_CS_HIGH` so the native CE0 line ends up active-high and
/// won't talk to the VL53L8CX. We work around that by setting
/// `SPI_NO_CS` (kernel doesn't drive CE0 at all) and relying on the line
/// idling low — the VL53L8CX is the only slave on this bus, so leaving
/// it permanently selected is fine. If the line ever floats high in
/// practice, tie BCM GPIO8 to GND or rewire NCS to GND directly.
pub const SPI_DEVICE: &str = "/dev/spidev0.0";

/// Pi GPIO chip used for general-purpose pins.
pub const GPIOCHIP: &str = "/dev/gpiochip0";

/// LPn (sensor reset, active low) on BCM GPIO17 (header pin 11).
pub const LPN_GPIO: u32 = 17;

/// SPI clock frequency. The VL53L8CX tolerates several MHz on SPI; the
/// firmware upload (~86 KB) finishes in well under a second at 1 MHz.
/// Bump higher (4–8 MHz) once the link is verified.
pub const SPI_FREQ_HZ: u32 = 2_000_000;

/// Human-readable label for the SPI mode actually programmed by [`open_spi`].
/// Keep this in sync with the `SpiModeFlags::SPI_MODE_*` value used there.
pub const SPI_MODE_LABEL: &str = "Mode 3";

/// Approximate VL53L8CX firmware blob size in KiB (used by log messages).
pub const FIRMWARE_SIZE_KB: u32 = 86;

// --- Concrete peripheral types (used by `bind_platform_driver_spi!`) ---

/// What the blocking driver actually talks to.
pub type SensorSpi = SpidevDevice;
pub type LpnPin = CdevPin;

/// Format an SPI clock frequency as a short human-readable string,
/// picking the most natural unit (`MHz` / `kHz` / `Hz`).
pub fn format_freq(hz: u32) -> String {
    if hz >= 1_000_000 && hz % 1_000_000 == 0 {
        format!("{} MHz", hz / 1_000_000)
    } else if hz >= 1_000_000 {
        format!("{:.1} MHz", hz as f64 / 1_000_000.0)
    } else if hz >= 1_000 {
        format!("{} kHz", hz / 1_000)
    } else {
        format!("{} Hz", hz)
    }
}

/// Estimated firmware-upload duration in seconds at the configured clock,
/// counting only the bulk firmware bytes (the rest of init is small).
pub fn firmware_upload_secs() -> f32 {
    (FIRMWARE_SIZE_KB as f32 * 1024.0 * 8.0) / SPI_FREQ_HZ as f32
}

/// Open `/dev/spidev0.0` configured for the VL53L8CX. We use **Mode 0**
/// while the dt-overlay CS issue is being sorted out — Mode 0 and Mode 3
/// have identical data-edge timing (data sampled on rising, changed on
/// falling); they only differ in the clock's idle level, and the
/// VL53L8CX has been observed to tolerate Mode 0. Switch back to
/// `SPI_MODE_3` once the dt-overlay is happy.
///
/// Kernel-managed CS is enabled (no `SPI_NO_CS`) — CE0 toggles per
/// transaction. Polarity comes from the dt-overlay; a fixed overlay
/// should give us active-low CS as the VL53L8CX expects.
pub fn open_spi() -> anyhow::Result<SensorSpi> {
    let mut spi = SpidevDevice::open(SPI_DEVICE)?;
    let options = SpidevOptions::new()
        .max_speed_hz(SPI_FREQ_HZ)
        .mode(SpiModeFlags::SPI_MODE_3)
        .bits_per_word(8)
        .build();
    spi.configure(&options)?;
    Ok(spi)
}

/// Acquire BCM GPIO17 as an OutputPin to drive LPn (initial low: held in reset).
pub fn open_lpn() -> anyhow::Result<LpnPin> {
    let mut chip = Chip::new(GPIOCHIP)?;
    let line = chip.get_line(LPN_GPIO)?;
    let handle = line.request(LineRequestFlags::OUTPUT, 0, "vl53l8cx-lpn")?;
    Ok(CdevPin::new(handle)?)
}

// --- Delay providers ---

/// Blocking `DelayNs` backed by `std::thread::sleep`.
#[derive(Clone, Copy, Default)]
pub struct StdDelay;

impl embedded_hal::delay::DelayNs for StdDelay {
    fn delay_ns(&mut self, ns: u32) {
        std::thread::sleep(Duration::from_nanos(ns as u64));
    }
}

/// Async `DelayNs` backed by `tokio::time::sleep`. Only usable inside a Tokio runtime.
#[derive(Clone, Copy, Default)]
pub struct TokioDelay;

impl embedded_hal_async::delay::DelayNs for TokioDelay {
    async fn delay_ns(&mut self, ns: u32) {
        tokio::time::sleep(Duration::from_nanos(ns as u64)).await;
    }
}

// --- Blocking → async SpiDevice adapter ---
//
// `linux-embedded-hal`'s `SpidevDevice` only implements the blocking SPI
// traits. `vl53_8x8::Spi<T>` needs `T:
// embedded_hal_async::spi::SpiDevice`. This wrapper bridges the two by
// invoking the blocking transaction inside an `async fn`. I/O blocks the
// current task; pair it with a single-threaded Tokio runtime
// (`tokio::main(flavor = "current_thread")`).

pub struct BlockingSpi<T>(pub T);

impl<T: embedded_hal::spi::ErrorType> embedded_hal::spi::ErrorType for BlockingSpi<T> {
    type Error = T::Error;
}

impl<T: embedded_hal::spi::SpiDevice<u8>> embedded_hal_async::spi::SpiDevice<u8> for BlockingSpi<T> {
    async fn transaction(
        &mut self,
        operations: &mut [embedded_hal::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        embedded_hal::spi::SpiDevice::transaction(&mut self.0, operations)
    }
}

/// Pulse PWR_EN low → 50 ms → high. The Pololu breakout doesn't expose a
/// PWR_EN pin (the LDO is always on while VIN has 3V3), so this is a
/// no-op kept for API symmetry with the FTDI helper. If you switch to a
/// board that gates the LDO, wire it to a spare GPIO and call into the
/// usual `OutputPin` toggle here.
pub fn power_up_board() {
    // No-op on the Pololu board.
}

// --- Pretty-printing helper used by both bins ---

/// Print an NxN distance grid from the sensor results buffer.
pub fn print_distance_grid(distance_mm: &[i16], grid: usize) {
    println!("--- Distance (mm) ---");
    for row in 0..grid {
        let base = row * grid;
        let mut line = String::new();
        for col in 0..grid {
            line.push_str(&format!("{:>5} ", distance_mm[base + col]));
        }
        println!("{line}");
    }
}
