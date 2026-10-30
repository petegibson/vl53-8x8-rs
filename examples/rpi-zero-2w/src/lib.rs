//! Shared helpers for the Raspberry Pi Zero 2W demos and hardware tests.
//!
//! Two Pololu breakouts are supported, chosen at build time:
//!
//! | Feature     | Sensor   | Bus                    | LPn        |
//! | ----------- | -------- | ---------------------- | ---------- |
//! | (default)   | VL53L8CX | SPI0, `/dev/spidev0.0` | BCM GPIO17 |
//! | `i2c-bus`   | VL53L5CX | I2C1, `/dev/i2c-1`     | BCM GPIO5  |
//!
//! Wiring for the VL53L8CX on SPI0 (Pi BCM pin → Pololu silkscreen):
//!
//! | Pi BCM | Header | Pololu pin | Notes                        |
//! | ------ | ------ | ---------- | ---------------------------- |
//! | 3V3    |  1     | VIN        | sensor power                 |
//! | GND    |  6     | GND        |                              |
//! | GPIO11 | 23     | SCL        | SPI0 SCLK                    |
//! | GPIO10 | 19     | SDA        | SPI0 MOSI                    |
//! | GPIO9  | 21     | MISO       | SPI0 MISO                    |
//! | GPIO8  | 24     | NCS        | SPI0 CE0 (managed by spidev) |
//! | GPIO17 | 11     | LPn        | I2C enable, active low       |
//!
//! Set the Pololu VL53L8CX board's SPI/I2C selector to SPI mode (it ships
//! in I2C mode).
//!
//! Wiring for the VL53L5CX on I2C1:
//!
//! | Pi BCM | Header | Pololu pin | Notes                     |
//! | ------ | ------ | ---------- | ------------------------- |
//! | 3V3    | 17     | VIN        | sensor power              |
//! | GND    | 25     | GND        |                           |
//! | GPIO2  |  3     | SDA        | I2C1 SDA (1.8 kΩ pull-up) |
//! | GPIO3  |  5     | SCL        | I2C1 SCL (1.8 kΩ pull-up) |
//! | GPIO5  | 29     | LPn        | I2C enable, active low    |
//!
//! Pi setup: `dtparam=spi=on` for SPI0 and `dtparam=i2c_arm=on` for I2C1 on
//! Raspberry Pi OS (both are device-tree overlays on NixOS, see the README),
//! and membership of the `spi`, `i2c` and `gpio` groups so the bins run
//! without sudo.
//!
//! [`uld`] binds ST's C ULD (`vl53l8cx-uld` or `vl53l5cx-uld`) to the
//! selected bus and [`native`] does the same for the pure-Rust `vl53-8x8`
//! driver. Each exposes a `Driver` alias and a `connect()` that opens the bus
//! and LPn, raises LPn and hands back a sensor ready for `init()`.

use std::time::Duration;

use anyhow::Context;
use linux_embedded_hal::gpio_cdev::{Chip, LineRequestFlags};
use linux_embedded_hal::spidev::{SpiModeFlags, SpidevOptions};
use linux_embedded_hal::{CdevPin, I2cdev, SpidevDevice};

// --- Board constants ---

/// Pi GPIO chip used for general-purpose pins.
pub const GPIOCHIP: &str = "/dev/gpiochip0";

/// LPn lines (I2C comms enable, active low; not a reset) of every board on
/// this build's bus, as BCM GPIO numbers. SPI: the VL53L8CX on GPIO17
/// (header pin 11). I2C: up to two boards sharing I2C1, GPIO5 (header pin
/// 29) and GPIO6 (header pin 31). The Pololu carriers pull LPn high on
/// board, so [`open_lpn`] claims *all* of these and holds the unselected
/// ones low: two enabled sensors at the default address would both answer.
#[cfg(not(feature = "i2c-bus"))]
pub const LPN_GPIOS: &[u32] = &[17];
#[cfg(feature = "i2c-bus")]
pub const LPN_GPIOS: &[u32] = &[5, 6];

/// Environment variable naming the BCM GPIO of the LPn line to drive, i.e.
/// which board the bins and tests talk to. Must be one of [`LPN_GPIOS`].
pub const LPN_ENV: &str = "VL53_LPN_GPIO";

/// The LPn line in use: `VL53_LPN_GPIO` if set, else the first of
/// [`LPN_GPIOS`] (GPIO17 for SPI, GPIO5 for I2C).
pub fn lpn_gpio() -> u32 {
    match std::env::var(LPN_ENV) {
        Ok(v) => {
            let gpio: u32 = v.trim().parse().unwrap_or_else(|_| panic!("{LPN_ENV}={v:?} is not a GPIO number"));
            assert!(LPN_GPIOS.contains(&gpio), "{LPN_ENV}={gpio} is not one of this build's LPn lines {LPN_GPIOS:?}");
            gpio
        }
        Err(_) => LPN_GPIOS[0],
    }
}

/// Address the second I2C board is moved to by the `two_i2c` bin and test,
/// so both can be enabled at once.
pub const SECOND_ADDRESS: u8 = 0x2A;

/// Which breakout this build talks to, for log lines.
#[cfg(not(feature = "i2c-bus"))]
pub const SENSOR_LABEL: &str = "VL53L8CX on SPI0";
#[cfg(feature = "i2c-bus")]
pub const SENSOR_LABEL: &str = "VL53L5CX on I2C1";

/// SPI bus device (SPI0 CE0). Kernel-managed, active-low CS comes from the
/// dt-overlay: the BCM2835 driver does not honour `SPI_CS_HIGH`, so an
/// overlay with the wrong `cs-gpios` polarity leaves the chip unselected
/// (see the README's note on CS polarity).
pub const SPI_DEVICE: &str = "/dev/spidev0.0";

/// SPI clock frequency. The VL53L8CX tolerates several MHz on SPI; the
/// firmware upload (~86 KB) finishes in well under a second at 2 MHz.
pub const SPI_FREQ_HZ: u32 = 2_000_000;

/// Human-readable label for the SPI mode programmed by [`open_spi`].
/// Keep this in sync with the `SpiModeFlags::SPI_MODE_*` value used there.
pub const SPI_MODE_LABEL: &str = "Mode 3";

/// I2C bus device (I2C1 on BCM GPIO2/GPIO3).
pub const I2C_DEVICE: &str = "/dev/i2c-1";

/// Largest I2C register write [`LinuxI2c`] issues in one ioctl, in bytes
/// including the two register-index bytes. Two kernel limits apply:
/// `i2c-dev` rejects any message over 8192 bytes, and the BCM2835 I2C
/// adapter aborts a whole `I2C_RDWR` call that takes longer than 1 s
/// (`ETIMEDOUT`, errno 110). 4 KB is 0.37 s at 100 kHz, comfortably inside
/// both; the 32 KB firmware chunks become eight such writes.
pub const I2C_MAX_MSG_LEN: usize = 4096;

/// Approximate firmware blob size in KiB (used by log messages).
pub const FIRMWARE_SIZE_KB: u32 = 86;

/// How long `connect()` holds LPn low before enabling comms.
pub const LPN_HOLD_LOW: Duration = Duration::from_millis(10);

// --- Concrete peripheral types ---

/// What the SPI drivers talk to.
pub type SensorSpi = SpidevDevice;
/// What the I2C drivers talk to: `I2cdev` behind the [`LinuxI2c`] adapter.
pub type SensorI2c = LinuxI2c;
/// The LPn line.
pub type LpnPin = CdevPin;

/// Format a clock frequency as a short human-readable string.
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

/// Estimated SPI firmware-upload duration in seconds at [`SPI_FREQ_HZ`],
/// counting only the bulk firmware bytes (the rest of init is small).
pub fn firmware_upload_secs() -> f32 {
    (FIRMWARE_SIZE_KB as f32 * 1024.0 * 8.0) / SPI_FREQ_HZ as f32
}

/// The bus this build opens, for log lines.
pub fn bus_description() -> String {
    #[cfg(not(feature = "i2c-bus"))]
    {
        format!("{SPI_DEVICE} ({SPI_MODE_LABEL}, {})", format_freq(SPI_FREQ_HZ))
    }
    #[cfg(feature = "i2c-bus")]
    {
        format!("{I2C_DEVICE} (bus clock set by the device tree, 100 kHz unless overridden)")
    }
}

/// Open `/dev/spidev0.0` configured for the VL53L8CX: Mode 3, 8-bit words,
/// kernel-managed CS on CE0.
pub fn open_spi() -> anyhow::Result<SensorSpi> {
    let mut spi = SpidevDevice::open(SPI_DEVICE)
        .with_context(|| format!("open {SPI_DEVICE}: is `dtparam=spi=on` set and the user in the spi group?"))?;
    let options = SpidevOptions::new()
        .max_speed_hz(SPI_FREQ_HZ)
        .mode(SpiModeFlags::SPI_MODE_3)
        .bits_per_word(8)
        .build();
    spi.configure(&options).context("configure spidev")?;
    Ok(spi)
}

/// Open `/dev/i2c-1` behind the [`LinuxI2c`] adapter.
pub fn open_i2c() -> anyhow::Result<SensorI2c> {
    let dev = I2cdev::new(I2C_DEVICE).with_context(|| {
        format!("open {I2C_DEVICE}: is I2C1 enabled and the user in the i2c group (NixOS: hardware.i2c.enable)?")
    })?;
    Ok(LinuxI2c(dev))
}

/// LPn lines of unselected boards, claimed low once per process and kept
/// (gpio-cdev holds a line only while its handle lives, and a line cannot be
/// requested twice, so `connect()` can be called repeatedly, as the test
/// suites do).
static HELD_LOW: std::sync::Mutex<Vec<(u32, CdevPin)>> = std::sync::Mutex::new(Vec::new());

/// Claim every LPn line in [`LPN_GPIOS`] low and return the handle for the
/// selected one ([`lpn_gpio`]), still low. The other lines stay claimed and
/// low for the rest of the process (see [`HELD_LOW`]), so an unselected board
/// on the same bus cannot answer at the default address.
pub fn open_lpn() -> anyhow::Result<LpnPin> {
    let selected = lpn_gpio();
    anyhow::ensure!(LPN_GPIOS.contains(&selected), "LPn GPIO{selected} is not in {LPN_GPIOS:?}");
    let mut held = HELD_LOW.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
    for &gpio in LPN_GPIOS {
        if gpio != selected && !held.iter().any(|(g, _)| *g == gpio) {
            held.push((gpio, open_lpn_gpio(gpio)?));
        }
    }
    open_lpn_gpio(selected)
}

/// Claim every LPn line in [`LPN_GPIOS`] low and return all of them, in
/// [`LPN_GPIOS`] order, for code that drives several boards itself. Lines
/// already held low by an earlier [`open_lpn`] in this process are handed
/// over rather than requested again.
pub fn open_all_lpn() -> anyhow::Result<Vec<(u32, LpnPin)>> {
    let mut held = HELD_LOW.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
    let mut out = Vec::with_capacity(LPN_GPIOS.len());
    for &gpio in LPN_GPIOS {
        let pin = match held.iter().position(|(g, _)| *g == gpio) {
            Some(i) => held.remove(i).1,
            None => open_lpn_gpio(gpio)?,
        };
        out.push((gpio, pin));
    }
    Ok(out)
}

/// Acquire BCM `gpio` as an output driving LPn, initially low (I2C disabled).
pub fn open_lpn_gpio(gpio: u32) -> anyhow::Result<LpnPin> {
    let mut chip = Chip::new(GPIOCHIP).with_context(|| format!("open {GPIOCHIP}: is the user in the gpio group?"))?;
    let line = chip.get_line(gpio)?;
    let handle = line
        .request(LineRequestFlags::OUTPUT, 0, "vl53-lpn")
        .with_context(|| format!("request BCM GPIO{gpio} as an output"))?;
    Ok(CdevPin::new(handle)?)
}

// --- Linux I2C adapter ---

/// `embedded-hal` I2C over `/dev/i2c-*` with the two fixes the VL53L5CX/L8CX
/// register protocol needs on Linux:
///
/// 1. **Merged writes.** The drivers send a register write as two
///    `Operation::Write`s (2-byte big-endian index, then the data).
///    `embedded-hal` requires adjacent writes to go out as one message, but
///    `linux-embedded-hal` maps every operation to its own `I2C_RDWR` message
///    with a fresh START and address, which the sensor would read as a second
///    write whose index is the first two data bytes. This adapter concatenates
///    adjacent writes into one message.
/// 2. **Chunked long writes.** `i2c-dev` rejects any message longer than
///    8192 bytes and the BCM2835 adapter aborts any `I2C_RDWR` call that
///    runs past 1 s, while the firmware goes up in 32 KB register writes. A
///    merged write over [`I2C_MAX_MSG_LEN`] is split into writes of at most
///    that size, each issued as its own ioctl and re-addressed with the
///    running register index (the sensor auto-increments the index within a
///    message, so the split is transparent to it; the pure-Rust driver
///    uploads the patched VL53L5CX firmware in the same addressed-chunk
///    fashion).
///
/// Reads and write-then-read pairs are passed through unchanged, so a
/// `write_read` keeps its repeated START.
pub struct LinuxI2c(pub I2cdev);

impl embedded_hal::i2c::ErrorType for LinuxI2c {
    type Error = <I2cdev as embedded_hal::i2c::ErrorType>::Error;
}

impl embedded_hal::i2c::I2c for LinuxI2c {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        use embedded_hal::i2c::Operation;

        enum Msg {
            Write(Vec<u8>),
            Read,
        }

        // Pass 1: plan the ioctls. Adjacent writes merge into one message;
        // an oversized merged write becomes a run of single-message ioctls.
        let mut calls: Vec<Vec<Msg>> = vec![Vec::new()];
        let mut i = 0;
        while i < operations.len() {
            match operations[i] {
                Operation::Write(_) => {
                    let mut buf = Vec::new();
                    while let Some(Operation::Write(w)) = operations.get(i) {
                        buf.extend_from_slice(w);
                        i += 1;
                    }
                    if buf.len() <= I2C_MAX_MSG_LEN {
                        calls.last_mut().unwrap().push(Msg::Write(buf));
                    } else {
                        let index = u16::from_be_bytes([buf[0], buf[1]]);
                        let mut offset = 0usize;
                        for chunk in buf[2..].chunks(I2C_MAX_MSG_LEN - 2) {
                            let mut msg = Vec::with_capacity(chunk.len() + 2);
                            msg.extend_from_slice(&index.wrapping_add(offset as u16).to_be_bytes());
                            msg.extend_from_slice(chunk);
                            calls.push(vec![Msg::Write(msg)]);
                            offset += chunk.len();
                        }
                        calls.push(Vec::new());
                    }
                }
                Operation::Read(_) => {
                    calls.last_mut().unwrap().push(Msg::Read);
                    i += 1;
                }
            }
        }

        // Pass 2: issue each planned ioctl as one I2C_RDWR.
        let mut reads = operations.iter_mut().filter_map(|op| match op {
            Operation::Read(r) => Some(&mut **r),
            Operation::Write(_) => None,
        });
        for call in &calls {
            if call.is_empty() {
                continue;
            }
            let mut ops: Vec<Operation<'_>> = Vec::with_capacity(call.len());
            for msg in call {
                match msg {
                    Msg::Write(buf) => ops.push(Operation::Write(buf)),
                    Msg::Read => ops.push(Operation::Read(reads.next().expect("one read per planned read"))),
                }
            }
            embedded_hal::i2c::I2c::transaction(&mut self.0, address, &mut ops)?;
        }
        Ok(())
    }
}

/// Addresses an interrupted test or demo may have left the enabled sensor at:
/// the test suites' temporary address and [`SECOND_ADDRESS`].
pub const STRAY_ADDRESSES: &[u8] = &[0x35, SECOND_ADDRESS];

/// Put a sensor that an interrupted run left at one of `strays` back at the
/// default address. Only the sensor whose LPn is high can answer, so call it
/// after [`open_lpn`] has silenced the other boards. Probes the strays only
/// if nothing answers at the default (a page-select write, harmless), then
/// sends ST's address-change sequence (`0x7FFF=0x00`, `0x0004=new address`,
/// `0x7FFF=0x02`). Returns the address it moved the sensor from, if any.
/// `connect()` runs this so one failed run does not need a power cycle.
#[cfg(feature = "i2c-bus")]
pub fn recover_default_address(strays: &[u8]) -> anyhow::Result<Option<u8>> {
    use embedded_hal::i2c::I2c;
    const DEFAULT: u8 = vl53_8x8::DEFAULT_ADDRESS;
    let mut bus = open_i2c()?;
    if bus.write(DEFAULT, &[0x7F, 0xFF, 0x00]).is_ok() {
        return Ok(None);
    }
    for &stray in strays {
        if bus.write(stray, &[0x7F, 0xFF, 0x00]).is_ok() {
            bus.write(stray, &[0x00, 0x04, DEFAULT]).context("write new address")?;
            bus.write(DEFAULT, &[0x7F, 0xFF, 0x02]).context("restore page")?;
            return Ok(Some(stray));
        }
    }
    Ok(None)
}

/// Shared tail of the `connect()` functions: hold LPn low briefly, enable
/// comms on the selected board, then (I2C) move it back to the default
/// address if an earlier run left it elsewhere.
fn finish_connect<E: std::fmt::Debug>(lpn_enable: impl FnOnce() -> Result<(), E>) -> anyhow::Result<()> {
    // open_lpn() leaves LPn low (comms disabled); hold it there briefly so a
    // transaction a previous process aborted half-way is dropped by the
    // sensor's I2C block before comms are re-enabled.
    std::thread::sleep(LPN_HOLD_LOW);
    lpn_enable().map_err(|e| anyhow::anyhow!("drive LPn high: {e:?}"))?;
    #[cfg(feature = "i2c-bus")]
    if let Some(from) = recover_default_address(STRAY_ADDRESSES)? {
        log::warn!("moved the sensor on GPIO{} back from {from:#04x} to the default address", lpn_gpio());
    }
    Ok(())
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

// --- ST's C ULD on the selected bus ---

/// ST's C ULD on the selected bus. `Driver` is `vl53l8cx_uld::Vl53l8cx` over
/// SPI0 by default and `vl53l5cx_uld::Vl53l5cx` over I2C1 with `i2c-bus`;
/// the two wrappers share one API shape, so bins and tests written against
/// this module compile for either. The ULD's platform callbacks are bound
/// here, once, for the whole crate.
pub mod uld {
    use anyhow::Context;

    #[cfg(not(feature = "i2c-bus"))]
    pub use vl53l8cx_uld::*;
    #[cfg(feature = "i2c-bus")]
    pub use vl53l5cx_uld::*;

    #[cfg(not(feature = "i2c-bus"))]
    vl53l8cx_uld::bind_platform_driver_spi!(crate::SensorSpi, crate::StdDelay);
    #[cfg(feature = "i2c-bus")]
    vl53l5cx_uld::bind_platform_driver!(crate::SensorI2c, crate::StdDelay);

    /// The blocking ULD wrapper over this build's bus, with LPn attached.
    #[cfg(not(feature = "i2c-bus"))]
    pub type Driver = Vl53l8cx<Vl53l8cxSpiDevice<crate::SensorSpi>, crate::StdDelay, crate::LpnPin>;
    #[cfg(feature = "i2c-bus")]
    pub type Driver = Vl53l5cx<crate::SensorI2c, crate::StdDelay, crate::LpnPin>;

    /// Open the bus and LPn, raise LPn (comms enable) and return the driver,
    /// not yet initialised. The Pololu boards have no power-enable pin, so
    /// there is no `power_cycle()`; `init()` reboots the sensor in software.
    pub fn connect() -> anyhow::Result<Driver> {
        let lpn = crate::open_lpn().context("open LPn")?;
        #[cfg(not(feature = "i2c-bus"))]
        let mut sensor = Vl53l8cx::new_spi(crate::open_spi()?, crate::StdDelay).with_lpn_pin(lpn);
        #[cfg(feature = "i2c-bus")]
        let mut sensor = Vl53l5cx::new(crate::open_i2c()?, crate::StdDelay).with_lpn_pin(lpn);
        crate::finish_connect(|| sensor.lpn_enable())?;
        Ok(sensor)
    }
}

// --- The pure-Rust vl53-8x8 driver on the selected bus ---

/// The pure-Rust `vl53-8x8` driver on the selected bus. It autodetects the
/// part at `init()`; [`EXPECTED_VARIANT`](native::EXPECTED_VARIANT) is what
/// this build's wiring should report.
///
/// `linux-embedded-hal` only implements the *blocking* `embedded-hal` bus
/// traits, so the bus goes through `vl53_8x8::BlockingSpi` /
/// `vl53_8x8::BlockingI2c`: every transfer blocks the Tokio task for its
/// duration and only the driver's `DelayNs` waits yield. That is fine on
/// the `current_thread` runtime the bins use; on a multi-threaded runtime
/// run the driver inside `spawn_blocking`.
pub mod native {
    use anyhow::Context;

    pub use vl53_8x8::*;

    /// The bus adapter handed to `Sensor::new`.
    #[cfg(not(feature = "i2c-bus"))]
    pub type Bus = vl53_8x8::BlockingSpi<crate::SensorSpi>;
    #[cfg(feature = "i2c-bus")]
    pub type Bus = vl53_8x8::BlockingI2c<crate::SensorI2c>;

    /// The async driver over this build's bus, with LPn attached.
    pub type Driver = vl53_8x8::Sensor<Bus, crate::TokioDelay, crate::LpnPin>;

    /// The part this build's wiring carries.
    #[cfg(not(feature = "i2c-bus"))]
    pub const EXPECTED_VARIANT: Variant = Variant::L8;
    #[cfg(feature = "i2c-bus")]
    pub const EXPECTED_VARIANT: Variant = Variant::L5;

    /// Open the bus and LPn, raise LPn (comms enable) and return the driver,
    /// not yet initialised. Call from inside a Tokio runtime (the delay
    /// provider needs one).
    pub fn connect() -> anyhow::Result<Driver> {
        let lpn = crate::open_lpn().context("open LPn")?;
        #[cfg(not(feature = "i2c-bus"))]
        let bus = vl53_8x8::BlockingSpi(crate::open_spi()?);
        #[cfg(feature = "i2c-bus")]
        let bus = vl53_8x8::BlockingI2c(crate::open_i2c()?);
        let mut sensor = Sensor::new(bus, crate::TokioDelay).with_lpn_pin(lpn);
        crate::finish_connect(|| sensor.lpn_enable())?;
        Ok(sensor)
    }
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
