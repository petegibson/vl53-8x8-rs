#![cfg_attr(not(test), no_std)]
#![allow(async_fn_in_trait)]
/*!
`vl53-8x8`: an async, platform-agnostic, pure-Rust driver for the
STMicroelectronics VL53L5CX and VL53L8CX 8x8 multizone Time-of-Flight sensors.

The driver speaks to the sensor over `embedded-hal-async` I2C or SPI and
reimplements the ST Ultra Lite Driver (ULD) protocol in Rust: there is no C
code, no FFI and no build script, so it builds for any target without libclang.
It detects the connected part at runtime ([`detect`], [`Variant`]) and uploads a
single shared firmware image patched per variant ([`firmware`], [`sensor`]).
The entry point is [`Sensor`]; its optional pins are LPn (I2C comms enable,
[`Sensor::with_lpn_pin`]), INT (data ready) and a power-enable line
([`Sensor::power_cycle`] is the sensor's only real reset); [`pulse_i2c_rst`]
drives the VL53L5CX's I2C_RST. [`Spi`] adapts an `SpiDevice` to the bus trait
and [`BlockingI2c`] / [`BlockingSpi`] wrap blocking `embedded-hal` I2C and SPI
buses (blocking HALs, RTIC 2, no executor).

Crate family:

- `vl53-8x8-types`: shared enums, `Error`, pin placeholders and result types
  (its only dependency is `embedded-hal`). Re-exported here so dependants need
  only this crate.
- `vl53-8x8` (this crate): the pure-Rust L5CX/L8CX driver.
- `vl53l8cx-uld-sys` / `vl53l5cx-uld-sys`: bindgen + cc over ST's L8CX and
  L5CX ULD C sources.
- `vl53l8cx-uld` / `vl53l5cx-uld`: safe blocking wrappers over those ULDs.

What comes from where:

- Protocol constants are transcribed from the ULD headers in a private
  `consts` module.
- ST's data tables (default configuration, default crosstalk shape, NVM-read
  command, and the xtalk plugin's calibration command with its two-byte L5
  overlay) are the generated `uld_data` module; the 86 KB firmware image is
  `vl53l8cx_fw_base.bin`. Both are produced from the ULD's
  `vl53l8cx_buffers.h` and the `vl53l{5,8}cx_plugin_xtalk.h` headers by
  `tools/extract_uld_blobs.py`. To take a new ULD release: replace the C
  tree under `vl53l8cx-uld-sys/src/VL53L8CX_ULD_API/`, run
  `python3 tools/extract_uld_blobs.py`, run `cargo test -p vl53-8x8`
  and review the diff.
- [`ResultsData`] is a `#[repr(C)]` rendering of the ULD's result struct
  with the same feature gating.

`vl53l8cx-uld-sys` and `vl53l5cx-uld-sys` are dev-dependencies only, used by
the oracle tests to check the constants, tables, patches and layouts above
against ST's C builds for both parts.
*/

pub mod bus;
mod consts;
pub mod firmware;
pub mod ops;
mod results;
pub mod sensor;
mod state;
mod uld_data;

#[cfg(test)]
mod oracle_tests;
#[cfg(test)]
mod pin_tests;

use bus::VlBusAsync;
use embedded_hal::digital::{InputPin, OutputPin};
pub use embedded_hal::i2c::SevenBitAddress;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use state::State;
use vl53_8x8_types as types;

// Shared types, re-exported so dependants need only this crate.
pub use types::{
    Error, FirmwareError, MaybeOutputPin, NoPin, PowerMode, RangingMode, Resolution, TargetOrder,
    DEFAULT_ADDRESS, XTALK_BUFFER_SIZE,
};

pub use results::ResultsData;
#[cfg(all(feature = "motion", not(feature = "disable-motion-indicator-output")))]
pub use results::MotionIndicator;

/// Targets reported per zone (the ULD's `VL53L8CX_NB_TARGET_PER_ZONE`): 1 by
/// default, or 2/3/4 with the matching `targets-N` feature. It is the stride of
/// the per-target arrays in [`ResultsData`]: entry `zone * NB_TARGET_PER_ZONE +
/// target`.
pub const NB_TARGET_PER_ZONE: usize = consts::NB_TARGET_PER_ZONE as usize;

#[cfg(feature = "thresholds")]
pub use types::{
    DetectionThreshold, ThresholdMeasurement, ThresholdOperation, ThresholdType,
    VL53L8CX_LAST_THRESHOLD, VL53L8CX_NB_THRESHOLDS,
};

/// Number of detection thresholds as usize for array sizing.
#[cfg(feature = "thresholds")]
const NB_THRESHOLDS: usize = VL53L8CX_NB_THRESHOLDS as usize;

#[cfg(any(feature = "motion", feature = "thresholds"))]
pub use types::MotionConfiguration;

pub use bus::{BlockingI2c, BlockingSpi, Spi};
pub use firmware::{FirmwareSource, PatchedFirmware, SliceFirmware};
pub use sensor::{detect, Variant};

/// Driver handle. `LPN`, `INT` and `PWR` are the optional LPn (I2C comms
/// enable), INT (data ready) and power-enable pins, [`NoPin`] when absent;
/// the `with_*_pin*` builders set them.
pub struct Sensor<BUS, D, LPN = NoPin, INT = NoPin, PWR = NoPin> {
    bus: BUS,
    delay: D,
    address: SevenBitAddress,
    lpn_pin: LPN,
    int_pin: INT,
    pwr_pin: PWR,
    /// `true` when the line in `lpn_pin` is inverted (an inverted LPn or the
    /// VL53L5CX's I2C_RST attached via the deprecated active-high builder).
    lpn_inverted: bool,
    int_is_data_ready_high: bool,
    pwr_active_high: bool,
    stdev: State,
    /// Sensor variant. Defaults to [`Variant::L8`] and is set to the detected
    /// part by [`Sensor::init`].
    variant: Variant,
}

// --- Constructors ---

impl<BUS, D> Sensor<BUS, D, NoPin, NoPin, NoPin>
where
    BUS: VlBusAsync,
    D: DelayNs,
{
    /// Create a driver over `bus` (an `embedded-hal-async` I2C bus, or
    /// [`BlockingI2c`] / [`BlockingSpi`] / [`Spi`] adapters) at
    /// [`DEFAULT_ADDRESS`] (0x29), with no LPn, INT or power-enable pin
    /// attached. Nothing is sent to the sensor until [`Sensor::init`]. Attach
    /// pins with [`Sensor::with_lpn_pin`], the `with_interrupt_pin_*` and
    /// `with_power_enable_pin_*` builders, and pick another address with
    /// [`Sensor::with_address`].
    pub fn new(bus: BUS, delay: D) -> Self {
        Self {
            bus,
            delay,
            address: DEFAULT_ADDRESS,
            lpn_pin: NoPin,
            int_pin: NoPin,
            pwr_pin: NoPin,
            lpn_inverted: false,
            int_is_data_ready_high: false,
            pwr_active_high: true,
            stdev: State::new(),
            variant: Variant::L8,
        }
    }
}

impl<SPI, D> Sensor<Spi<SPI>, D, NoPin, NoPin, NoPin>
where
    SPI: embedded_hal_async::spi::SpiDevice,
    D: DelayNs,
{
    /// Create a driver over an `embedded-hal-async` `SpiDevice` (the
    /// VL53L8CX's SPI interface; the VL53L5CX is I2C only). Otherwise as
    /// [`Sensor::new`]: no pins attached, nothing sent until [`Sensor::init`].
    pub fn new_spi(spi: SPI, delay: D) -> Self {
        Sensor::new(Spi { spi }, delay)
    }
}

// --- Builder methods ---

impl<BUS, D, LPN, INT, PWR> Sensor<BUS, D, LPN, INT, PWR> {
    /// Attach at `address` instead of [`DEFAULT_ADDRESS`], without touching the
    /// sensor. For re-attaching a handle to a sensor previously moved by
    /// [`Sensor::set_i2c_address`]; pair with [`Sensor::redetect`] to skip
    /// the firmware re-upload.
    pub fn with_address(mut self, address: SevenBitAddress) -> Self {
        self.address = address;
        self
    }

    /// Attach the sensor's **LPn** line, which is always active low.
    ///
    /// LPn is what the datasheet calls the "Comms enable": low disables the
    /// sensor's I2C interface, high enables it. It is not a reset and has
    /// nothing to do with power. Its purpose is multi-sensor address
    /// assignment: hold every other sensor's LPn low
    /// ([`lpn_disable`](Self::lpn_disable)), move one sensor with
    /// [`Sensor::set_i2c_address`], then release the others
    /// ([`lpn_enable`](Self::lpn_enable)). Leave it high otherwise; `init()`
    /// needs it high. When attached, [`power_cycle`](Self::power_cycle) also
    /// drives it low with the supplies, as UM2884 §4.2 requires. An LPn that
    /// hangs only on its pull-up to IOVDD needs no GPIO at all. The pin's
    /// level is not changed here.
    pub fn with_lpn_pin<LpnNew>(self, lpn_pin: LpnNew) -> Sensor<BUS, D, LpnNew, INT, PWR> {
        let Sensor { bus, delay, address, int_pin, pwr_pin, int_is_data_ready_high, pwr_active_high, stdev, variant, .. } = self;
        Sensor {
            bus,
            delay,
            address,
            lpn_pin,
            int_pin,
            pwr_pin,
            lpn_inverted: false,
            int_is_data_ready_high,
            pwr_active_high,
            stdev,
            variant,
        }
    }

    /// Deprecated name of [`with_lpn_pin`](Self::with_lpn_pin): LPn is the I2C
    /// comms-enable line, not a reset.
    #[deprecated(
        since = "0.2.0",
        note = "LPn is not a reset; use with_lpn_pin(), and power_cycle() for a real reset"
    )]
    pub fn with_reset_pin_active_low<RstNew>(self, reset_pin: RstNew) -> Sensor<BUS, D, RstNew, INT, PWR> {
        self.with_lpn_pin(reset_pin)
    }

    /// Deprecated: attach an **inverted** line in the LPn slot (high = held
    /// off). It was meant for an inverted LPn or for the VL53L5CX's I2C_RST,
    /// which resets only the I2C interface. The LPn methods and the deprecated
    /// [`reset`](Self::reset) honour the inversion, so existing code keeps its
    /// meaning. For I2C_RST use the free [`pulse_i2c_rst`] instead; for an
    /// inverted LPn use [`with_lpn_pin`](Self::with_lpn_pin) with your own
    /// inverting `OutputPin`.
    #[deprecated(
        since = "0.2.0",
        note = "attach I2C_RST with pulse_i2c_rst(), or an inverted LPn with with_lpn_pin() plus your own inversion"
    )]
    pub fn with_reset_pin_active_high<RstNew>(self, reset_pin: RstNew) -> Sensor<BUS, D, RstNew, INT, PWR> {
        let mut s = self.with_lpn_pin(reset_pin);
        s.lpn_inverted = true;
        s
    }

    /// Attach the sensor's INT (data-ready) pin, which goes **low** when a frame
    /// is ready. This is the part's default: INT is an open-drain output pulled
    /// low for the interrupt pulse.
    pub fn with_interrupt_pin_active_low<IntNew>(self, int_pin: IntNew) -> Sensor<BUS, D, LPN, IntNew, PWR> {
        let Sensor { bus, delay, address, lpn_pin, pwr_pin, lpn_inverted, pwr_active_high, stdev, variant, .. } = self;
        Sensor {
            bus,
            delay,
            address,
            lpn_pin,
            int_pin,
            pwr_pin,
            lpn_inverted,
            int_is_data_ready_high: false,
            pwr_active_high,
            stdev,
            variant,
        }
    }

    /// Attach a data-ready line that goes **high** when a frame is ready, for
    /// boards that invert the sensor's INT output.
    pub fn with_interrupt_pin_active_high<IntNew>(self, int_pin: IntNew) -> Sensor<BUS, D, LPN, IntNew, PWR> {
        let mut s = self.with_interrupt_pin_active_low(int_pin);
        s.int_is_data_ready_high = true;
        s
    }

    /// Attach a power-enable line where **high** powers the sensor (e.g. the
    /// X-NUCLEO-53L8A1 shield's `PWR_EN`). Enables [`power_on`](Self::power_on),
    /// [`power_off`](Self::power_off) and [`power_cycle`](Self::power_cycle), the
    /// only real reset the sensor has. The pin's level is not changed here.
    pub fn with_power_enable_pin_active_high<PwrNew>(self, pwr_pin: PwrNew) -> Sensor<BUS, D, LPN, INT, PwrNew> {
        let Sensor { bus, delay, address, lpn_pin, int_pin, lpn_inverted, int_is_data_ready_high, stdev, variant, .. } = self;
        Sensor {
            bus,
            delay,
            address,
            lpn_pin,
            int_pin,
            pwr_pin,
            lpn_inverted,
            int_is_data_ready_high,
            pwr_active_high: true,
            stdev,
            variant,
        }
    }

    /// Attach a power-enable line where **low** powers the sensor (a P-channel
    /// high-side switch or an LDO with an active-low enable). Otherwise as
    /// [`with_power_enable_pin_active_high`](Self::with_power_enable_pin_active_high).
    pub fn with_power_enable_pin_active_low<PwrNew>(self, pwr_pin: PwrNew) -> Sensor<BUS, D, LPN, INT, PwrNew> {
        let mut s = self.with_power_enable_pin_active_high(pwr_pin);
        s.pwr_active_high = false;
        s
    }
}

// --- LPn methods (requires OutputPin) ---

impl<BUS, D, LPN, INT, PWR> Sensor<BUS, D, LPN, INT, PWR>
where
    BUS: VlBusAsync,
    LPN: OutputPin,
{
    /// Drive LPn to the logical level `high` (true = comms enabled), honouring
    /// the deprecated active-high builder's inversion.
    fn set_lpn_level(&mut self, high: bool) -> Result<(), Error<BUS::Error>> {
        let result = if high != self.lpn_inverted {
            self.lpn_pin.set_high()
        } else {
            self.lpn_pin.set_low()
        };
        result.map_err(|_| Error::Pin)
    }

    /// Drive LPn high: the sensor's I2C interface listens ("Comms enable" in
    /// the datasheet). This is the resting state; `init()` needs it. Part of
    /// multi-sensor address assignment, see [`Sensor::set_i2c_address`].
    pub fn lpn_enable(&mut self) -> Result<(), Error<BUS::Error>> {
        self.set_lpn_level(true)
    }

    /// Drive LPn low: the sensor ignores the I2C bus until
    /// [`lpn_enable`](Self::lpn_enable). Hold every other sensor low while
    /// moving one with [`Sensor::set_i2c_address`]. This does not reset the
    /// sensor, drop its firmware or change its power state.
    pub fn lpn_disable(&mut self) -> Result<(), Error<BUS::Error>> {
        self.set_lpn_level(false)
    }
}

impl<BUS, D, LPN, INT, PWR> Sensor<BUS, D, LPN, INT, PWR>
where
    BUS: VlBusAsync,
    D: DelayNs,
    LPN: OutputPin,
{
    /// Deprecated: pulse the line in the LPn slot (assert for 10 ms, release).
    ///
    /// On LPn this only disables I2C for 10 ms; on an I2C_RST attached with the
    /// deprecated active-high builder it resets the I2C interface only. Neither
    /// resets the sensor or discards its firmware, and ST's ULD has no reset
    /// function either. The real reset is [`power_cycle`](Self::power_cycle)
    /// (UM2884 §4.2); to re-initialise a running sensor just call `init()`,
    /// which always starts with ST's software reboot sequence.
    #[deprecated(
        since = "0.2.0",
        note = "pulsing LPn does not reset the sensor; use power_cycle() or just init()"
    )]
    pub async fn reset(&mut self) -> Result<(), Error<BUS::Error>> {
        self.set_lpn_level(false)?;
        self.delay.delay_ms(10).await;
        self.set_lpn_level(true)
    }
}

// --- Power methods (requires a power-enable OutputPin) ---

impl<BUS, D, LPN, INT, PWR> Sensor<BUS, D, LPN, INT, PWR>
where
    BUS: VlBusAsync,
    PWR: OutputPin,
{
    fn set_power(&mut self, on: bool) -> Result<(), Error<BUS::Error>> {
        let result = if on == self.pwr_active_high {
            self.pwr_pin.set_high()
        } else {
            self.pwr_pin.set_low()
        };
        result.map_err(|_| Error::Pin)
    }

    /// Cut the sensor's supply via the power-enable pin. The firmware, the
    /// configuration and any changed I2C address are lost; the sensor restarts
    /// at [`DEFAULT_ADDRESS`] after [`power_on`](Self::power_on) and needs
    /// `init()`. If an LPn GPIO is attached, drive it low too
    /// ([`lpn_disable`](Self::lpn_disable)): a digital input must not sit above
    /// IOVDD while IOVDD is off.
    pub fn power_off(&mut self) -> Result<(), Error<BUS::Error>> {
        self.set_power(false)
    }

    /// Enable the sensor's supply via the power-enable pin. Allow the rails to
    /// settle (10 ms is enough on the ST boards), make sure LPn is high and
    /// I2C_RST low, then `init()`.
    pub fn power_on(&mut self) -> Result<(), Error<BUS::Error>> {
        self.set_power(true)
    }
}

impl<BUS, D, LPN, INT, PWR> Sensor<BUS, D, LPN, INT, PWR>
where
    BUS: VlBusAsync,
    D: DelayNs,
    LPN: MaybeOutputPin,
    PWR: OutputPin,
{
    /// Drive LPn (if a GPIO is attached) to the logical level `high`, else no-op.
    fn set_lpn_level_or_skip(&mut self, high: bool) -> Result<(), Error<BUS::Error>> {
        if high != self.lpn_inverted {
            self.lpn_pin.set_high_or_skip()
        } else {
            self.lpn_pin.set_low_or_skip()
        }
    }

    /// Power-cycle the sensor: the only real reset it has (ST's ULD has none).
    ///
    /// Implements the sensor reset procedure of UM2884 §4.2 (VL53L5CX) / UM3109
    /// (VL53L8CX): drive the supplies and LPn low, wait 10 ms, drive them high.
    /// Concretely: [`power_off`](Self::power_off); LPn low if a GPIO is attached
    /// (an LPn on its IOVDD pull-up follows the rail by itself); 10 ms;
    /// [`power_on`](Self::power_on), then LPn high; then 10 ms for the supplies
    /// to settle. Boards with slow supplies may need a longer settle: add your
    /// own delay before `init()`. Afterwards the firmware is gone and the sensor
    /// answers at [`DEFAULT_ADDRESS`], so follow with [`Sensor::init`] (and
    /// `with_address`/`set_i2c_address` again if you had moved it). Compiles
    /// with or without an LPn pin via [`MaybeOutputPin`].
    pub async fn power_cycle(&mut self) -> Result<(), Error<BUS::Error>> {
        self.set_power(false)?;
        self.set_lpn_level_or_skip(false)?;
        self.delay.delay_ms(10).await;
        self.set_power(true)?;
        self.set_lpn_level_or_skip(true)?;
        self.delay.delay_ms(10).await;
        Ok(())
    }
}

/// Pulse the VL53L5CX's **I2C_RST** input: high for 10 ms, then low.
///
/// I2C_RST (active high, internal pull-down) resets the sensor's I2C interface
/// only, when toggled 0 -> 1 -> 0; it does not reset the sensor, drop its
/// firmware or change its address. Use it to recover a hung I2C block; `init()`
/// expects the line low. The VL53L8CX has no I2C_RST: its SPI_I2C_N pin selects
/// the interface and toggling it likewise only clears the I2C block. A real
/// reset is [`Sensor::power_cycle`]. Fails with `Error::Pin` on a pin error.
pub async fn pulse_i2c_rst<P: OutputPin, D: DelayNs>(
    pin: &mut P,
    delay: &mut D,
) -> Result<(), Error<core::convert::Infallible>> {
    pin.set_high().map_err(|_| Error::Pin)?;
    delay.delay_ms(10).await;
    pin.set_low().map_err(|_| Error::Pin)
}

// --- Interrupt-based data ready (requires Wait) ---

impl<BUS, D, LPN, INT, PWR> Sensor<BUS, D, LPN, INT, PWR>
where
    BUS: VlBusAsync,
    INT: Wait,
    <INT as embedded_hal::digital::ErrorType>::Error: core::fmt::Debug,
{
    /// Wait for data ready using the interrupt pin (async).
    pub async fn wait_for_data_ready(&mut self) -> Result<(), Error<BUS::Error>> {
        let result = if self.int_is_data_ready_high {
            self.int_pin.wait_for_high().await
        } else {
            self.int_pin.wait_for_low().await
        };
        result.map_err(|_| Error::Pin)
    }
}

impl<BUS, D, LPN, INT, PWR> Sensor<BUS, D, LPN, INT, PWR>
where
    BUS: VlBusAsync,
    D: DelayNs,
    INT: Wait,
    <INT as embedded_hal::digital::ErrorType>::Error: core::fmt::Debug,
{
    /// Wait for data ready, then read the ranging data.
    pub async fn next_ranging_data(&mut self) -> Result<ResultsData, Error<BUS::Error>> {
        self.wait_for_data_ready().await?;
        self.get_ranging_data().await
    }
}

// --- Blocking interrupt data ready (requires InputPin) ---

impl<BUS, D, LPN, INT, PWR> Sensor<BUS, D, LPN, INT, PWR>
where
    BUS: VlBusAsync,
    INT: InputPin,
    <INT as embedded_hal::digital::ErrorType>::Error: core::fmt::Debug,
{
    /// Poll the interrupt pin for data ready (busy-wait).
    pub fn wait_for_data_ready_blocking(&mut self) -> Result<(), Error<BUS::Error>> {
        loop {
            let is_high = self.int_pin.is_high().map_err(|_| Error::Pin)?;
            if is_high == self.int_is_data_ready_high {
                break;
            }
        }
        Ok(())
    }
}

// --- Core async methods ---

impl<BUS, D, LPN, INT, PWR> Sensor<BUS, D, LPN, INT, PWR>
where
    BUS: VlBusAsync,
    D: DelayNs,
{
    /// Detect the connected sensor variant, then upload its (patched) firmware
    /// and bring the sensor up. The detected variant is stored and available
    /// via [`Sensor::variant`].
    ///
    /// Call after power-on with AVDD, IOVDD and LPn high (I2C_RST low on a
    /// VL53L5CX); the sensor must answer at the current address, else
    /// `Error::NotAlive`. The upload is ~84 KB and takes a few hundred ms; the
    /// same call re-initialises a running sensor. Leaves 4x4, 1 Hz,
    /// [`RangingMode::Autonomous`], 5 ms integration, sharpener 14 %,
    /// [`TargetOrder::Strongest`], [`PowerMode::Wakeup`] and detection
    /// thresholds disabled.
    pub async fn init(&mut self) -> Result<(), Error<BUS::Error>> {
        self.variant = sensor::detect(&mut self.bus, self.address).await?;

        let mut fw = self.variant.firmware();
        let info = self.variant.info();
        ops::init_sensor(
            &mut self.bus,
            self.address,
            &mut self.delay,
            &mut self.stdev,
            &mut fw,
            info,
        )
        .await
    }

    /// The sensor variant detected by [`Sensor::init`]. Before `init` this is
    /// the default ([`Variant::L8`]), not a detected value.
    pub fn variant(&self) -> Variant {
        self.variant
    }

    /// Re-detect the sensor variant at the current address, without the
    /// firmware upload done by [`Sensor::init`]. For handles created with
    /// [`Sensor::with_address`] against an already-initialised sensor:
    /// [`Sensor::start_ranging`] recomputes all per-session state, so after
    /// this the handle can resume ranging where the old one left off.
    pub async fn redetect(&mut self) -> Result<Variant, Error<BUS::Error>> {
        self.variant = sensor::detect(&mut self.bus, self.address).await?;
        Ok(self.variant)
    }

    /// Returns true if a recognised sensor (VL53L5CX or VL53L8CX) is present.
    pub async fn is_alive(&mut self) -> Result<bool, Error<BUS::Error>> {
        match sensor::detect(&mut self.bus, self.address).await {
            Ok(_) => Ok(true),
            Err(Error::NotAlive) | Err(Error::UnknownRevision(_)) => Ok(false),
            Err(e) => Err(e),
        }
    }

    /// Move the sensor to `new_address` (7-bit) and use it for all further
    /// traffic. With several sensors on one bus, hold every other sensor's LPn
    /// low while calling this so only the target sensor hears the command, then
    /// release them ([`Sensor::with_lpn_pin`], [`Sensor::lpn_disable`] /
    /// [`Sensor::lpn_enable`]). It is a register write, not a firmware
    /// command, so it works before [`Sensor::init`] as in ST's multi-sensor
    /// example, and needs no pin on this handle. The address is not retained: a power cycle
    /// returns the sensor to [`DEFAULT_ADDRESS`]. The value is not validated.
    pub async fn set_i2c_address(&mut self, new_address: SevenBitAddress) -> Result<(), Error<BUS::Error>> {
        ops::set_i2c_address(&mut self.bus, self.address, new_address).await?;
        self.address = new_address;
        Ok(())
    }

    /// Start a ranging session with the current configuration. Settings cannot
    /// be changed while ranging: set resolution, frequency, mode, integration
    /// time, sharpener, target order, thresholds and motion first, then start,
    /// then loop on [`Sensor::check_data_ready`] (or the INT pin) and
    /// [`Sensor::get_ranging_data`], and finish with [`Sensor::stop_ranging`].
    /// On a VL53L8CX the firmware's laser-safety self-test is read here
    /// and a fault is `Error::Firmware(FirmwareError::LaserSafety)`.
    pub async fn start_ranging(&mut self) -> Result<(), Error<BUS::Error>> {
        ops::start_ranging(
            &mut self.bus,
            self.address,
            &mut self.delay,
            &mut self.stdev,
            self.variant.info().has_laser_safety_check,
        )
        .await
    }

    /// Stop the session started by [`Sensor::start_ranging`]. Required before
    /// changing settings or the power mode and, on a VL53L8CX, after a detection-threshold auto-stop
    /// interrupt ([`Sensor::set_detection_thresholds_auto_stop`]). Afterwards the sensor is
    /// idle in its current power mode, keeps its configuration and can be
    /// reconfigured and restarted.
    pub async fn stop_ranging(&mut self) -> Result<(), Error<BUS::Error>> {
        ops::stop_ranging(&mut self.bus, self.address, &mut self.delay, &mut self.stdev).await
    }

    /// Poll the sensor over the bus for a new frame: `Ok(true)` once per frame,
    /// when it is ready to be read with [`Sensor::get_ranging_data`]. Polling
    /// pattern without an INT pin: call this in a loop with a short delay (a few
    /// ms) until it returns `true`, read the frame, repeat.
    /// [`Sensor::poll_data_ready`] does exactly this; with INT wired use
    /// [`Sensor::wait_for_data_ready`] instead. A
    /// firmware error flagged in the status bytes is returned as
    /// `Error::Firmware`.
    pub async fn check_data_ready(&mut self) -> Result<bool, Error<BUS::Error>> {
        ops::check_data_ready(&mut self.bus, self.address, &mut self.stdev).await
    }

    /// Read the latest frame into a [`ResultsData`]. Call after
    /// [`Sensor::check_data_ready`] returned `true` or INT fired. Per-zone arrays
    /// (`nb_target_detected`, `ambient_per_spad`, ...) are indexed by zone;
    /// per-target arrays (`distance_mm`, `target_status`, ...) by
    /// `zone * NB_TARGET_PER_ZONE + target` ([`NB_TARGET_PER_ZONE`]). In 4x4
    /// only the first 16 zones are filled. Units are mm, kcps/SPAD and %
    /// unless `raw-format`. A frame whose block headers do not match is
    /// `Error::Firmware(FirmwareError::CorruptedFrame)`. See [`ResultsData`].
    pub async fn get_ranging_data(&mut self) -> Result<ResultsData, Error<BUS::Error>> {
        ops::get_ranging_data(
            &mut self.bus,
            self.address,
            &mut self.stdev,
            self.variant.info().clamp_negative_distance,
        )
        .await
    }

    /// Poll for data ready using async delay between checks (no INT pin needed).
    pub async fn poll_data_ready(&mut self) -> Result<(), Error<BUS::Error>> {
        loop {
            if self.check_data_ready().await? {
                return Ok(());
            }
            self.delay.delay_ms(10).await;
        }
    }

    // --- Configuration methods ---

    /// Set the zone grid. Also re-sends the offset and crosstalk calibration
    /// data, which are resolution dependent (see [`ops::set_resolution`]).
    ///
    /// Default after [`Sensor::init`] is [`Resolution::Res4x4`]; 8x8 means four
    /// times the bus traffic per frame. Set this **before**
    /// [`Sensor::set_ranging_frequency_hz`], whose allowed range depends on
    /// it, and with the motion indicator follow with
    /// [`Sensor::motion_indicator_set_resolution`]. Not while ranging.
    pub async fn set_resolution(&mut self, resolution: Resolution) -> Result<(), Error<BUS::Error>> {
        ops::set_resolution(
            &mut self.bus, self.address, &mut self.delay, &mut self.stdev, resolution as u8,
        ).await
    }

    /// Read the current zone grid. Default after [`Sensor::init`] is
    /// [`Resolution::Res4x4`].
    pub async fn get_resolution(&mut self) -> Result<Resolution, Error<BUS::Error>> {
        let res = ops::get_resolution(
            &mut self.bus, self.address, &mut self.delay, &mut self.stdev.temp_buffer,
        ).await?;
        if res == types::VL53L8CX_RESOLUTION_8X8 {
            Ok(Resolution::Res8x8)
        } else {
            Ok(Resolution::Res4x4)
        }
    }

    /// Set the ranging frequency in Hz (frames per second): 1..=60 Hz in 4x4,
    /// 1..=15 Hz in 8x8; default 1 Hz after [`Sensor::init`]. The limits depend on
    /// the resolution, so call [`Sensor::set_resolution`] first. Neither this
    /// driver nor ST's ULD checks the value: the firmware enforces the limits.
    /// In [`RangingMode::Autonomous`] the frame period must also exceed the
    /// integration time(s) plus 1 ms, see [`Sensor::set_integration_time_ms`].
    /// Not while ranging.
    pub async fn set_ranging_frequency_hz(&mut self, frequency_hz: u8) -> Result<(), Error<BUS::Error>> {
        let mut data = [0u8; 4];
        ops::dci_replace(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut data,
            consts::VL53L8CX_DCI_FREQ_HZ as u32, 4,
            &[frequency_hz], 0x01,
        ).await
    }

    /// Read the ranging frequency in Hz. Default after [`Sensor::init`] is 1 Hz.
    pub async fn get_ranging_frequency_hz(&mut self) -> Result<u8, Error<BUS::Error>> {
        let mut buf = [0u8; 4];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_FREQ_HZ as u32, 4,
        ).await?;
        Ok(buf[0x01])
    }

    /// Set the integration time in ms, 2..=1000; anything else returns
    /// `Error::Firmware(FirmwareError::InvalidParam)` without touching the bus.
    /// Default 5 ms after [`Sensor::init`]. Only meaningful in
    /// [`RangingMode::Autonomous`] (no effect in continuous mode). 4x4
    /// integrates once per frame, 8x8 four times; the sum of the integrations
    /// plus 1 ms of overhead must fit in the frame period (1 / frequency),
    /// otherwise the sensor lengthens the period by itself. Not while ranging.
    pub async fn set_integration_time_ms(&mut self, time_ms: u32) -> Result<(), Error<BUS::Error>> {
        if time_ms < 2 || time_ms > 1000 {
            return Err(Error::Firmware(FirmwareError::InvalidParam));
        }
        let integration = (time_ms * 1000).to_le_bytes();
        let mut data = [0u8; 20];
        ops::dci_replace(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut data,
            consts::VL53L8CX_DCI_INT_TIME as u32, 20,
            &integration, 0x00,
        ).await
    }

    /// Read the integration time in ms. Default after [`Sensor::init`] is 5 ms.
    pub async fn get_integration_time_ms(&mut self) -> Result<u32, Error<BUS::Error>> {
        let mut buf = [0u8; 20];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_INT_TIME as u32, 20,
        ).await?;
        let time_us = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        Ok(time_us / 1000)
    }

    /// Choose which detected target a zone reports first: [`TargetOrder::Closest`]
    /// or [`TargetOrder::Strongest`] (default after [`Sensor::init`]). With the
    /// default single target per zone this selects the one target reported;
    /// with a `targets-N` feature it orders the list ([`NB_TARGET_PER_ZONE`]).
    /// Two targets in one zone are only told apart when at least 600 mm apart.
    /// Not while ranging.
    pub async fn set_target_order(&mut self, order: TargetOrder) -> Result<(), Error<BUS::Error>> {
        let val = order as u8;
        let mut data = [0u8; 4];
        ops::dci_replace(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut data,
            consts::VL53L8CX_DCI_TARGET_ORDER as u32, 4,
            &[val], 0x00,
        ).await
    }

    /// Read the target order. Default after [`Sensor::init`] is
    /// [`TargetOrder::Strongest`].
    pub async fn get_target_order(&mut self) -> Result<TargetOrder, Error<BUS::Error>> {
        let mut buf = [0u8; 4];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_TARGET_ORDER as u32, 4,
        ).await?;
        if buf[0] == types::VL53L8CX_TARGET_ORDER_STRONGEST {
            Ok(TargetOrder::Strongest)
        } else {
            Ok(TargetOrder::Closest)
        }
    }

    /// Set the ranging mode. [`RangingMode::Autonomous`] (default after
    /// [`Sensor::init`]) fires the VCSEL only for the integration time of each
    /// frame: lower power, the more so at low frequency; the first frame's
    /// period may be shorter than configured. [`RangingMode::Continuous`] keeps
    /// the VCSEL on throughout: best range and ambient immunity, for fast or
    /// high-performance ranging; the integration time is ignored. Not while
    /// ranging.
    pub async fn set_ranging_mode(&mut self, mode: RangingMode) -> Result<(), Error<BUS::Error>> {
        let mut data = [0u8; 8];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut data,
            consts::VL53L8CX_DCI_RANGING_MODE as u32, 8,
        ).await?;

        let single_range: u32;
        match mode {
            RangingMode::Continuous => {
                data[0x01] = 0x01;
                data[0x03] = 0x03;
                single_range = 0x00;
            }
            RangingMode::Autonomous => {
                data[0x01] = 0x03;
                data[0x03] = 0x02;
                single_range = 0x01;
            }
        }

        ops::dci_write(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut data,
            consts::VL53L8CX_DCI_RANGING_MODE as u32, 8,
        ).await?;

        let mut sr_bytes = single_range.to_le_bytes();
        ops::dci_write(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut sr_bytes,
            consts::VL53L8CX_DCI_SINGLE_RANGE as u32, 4,
        ).await
    }

    /// Read the ranging mode. Default after [`Sensor::init`] is
    /// [`RangingMode::Autonomous`].
    pub async fn get_ranging_mode(&mut self) -> Result<RangingMode, Error<BUS::Error>> {
        let mut buf = [0u8; 8];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_RANGING_MODE as u32, 8,
        ).await?;
        if buf[0x01] == 0x01 {
            Ok(RangingMode::Continuous)
        } else {
            Ok(RangingMode::Autonomous)
        }
    }

    /// Set the sharpener in percent, 0..=99; 100 or more returns
    /// `Error::Firmware(FirmwareError::InvalidParam)` without touching the bus.
    /// Default 14 % after [`Sensor::init`]. The sharpener removes veiling-glare
    /// signal spilling from a close target into neighbouring zones; a higher
    /// value trims more of that spread, 0 disables it. Stored in 1/255 steps, so
    /// the readback can be 1 % below the value set. Not while ranging.
    pub async fn set_sharpener_percent(&mut self, percent: u8) -> Result<(), Error<BUS::Error>> {
        if percent >= 100 {
            return Err(Error::Firmware(FirmwareError::InvalidParam));
        }
        let sharpener = (percent as u16 * 255 / 100) as u8;
        let mut data = [0u8; 16];
        ops::dci_replace(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut data,
            consts::VL53L8CX_DCI_SHARPENER as u32, 16,
            &[sharpener], 0x0D,
        ).await
    }

    /// Read the sharpener in percent. Default after [`Sensor::init`] is 14 %; the
    /// value is stored in 1/255 steps, so it can read 1 % below what was set.
    pub async fn get_sharpener_percent(&mut self) -> Result<u8, Error<BUS::Error>> {
        let mut buf = [0u8; 16];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_SHARPENER as u32, 16,
        ).await?;
        Ok((buf[0x0D] as u16 * 100 / 255) as u8)
    }

    /// Set the power mode; the sensor must not be ranging (call
    /// [`Sensor::stop_ranging`] first). Default after [`Sensor::init`] is
    /// [`PowerMode::Wakeup`] (HP idle, ready to range). [`PowerMode::Sleep`]
    /// (LP idle) keeps firmware and configuration but cannot range until woken.
    /// [`PowerMode::DeepSleep`] is VL53L8CX only: waking from it re-uploads the
    /// firmware, so the configuration returns to the `init()` defaults; on a
    /// VL53L5CX it returns `Error::Firmware(FirmwareError::InvalidParam)`
    /// without touching the bus. Setting the current mode is a no-op.
    pub async fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), Error<BUS::Error>> {
        // Deep sleep is L8 only; reject it on parts without it (e.g. L5).
        if mode == PowerMode::DeepSleep && !self.variant.info().has_deep_sleep {
            return Err(Error::Firmware(FirmwareError::InvalidParam));
        }
        let current = self.get_power_mode().await?;
        if mode == current {
            return Ok(());
        }

        match mode {
            PowerMode::Wakeup => {
                self.bus.write_byte(self.address, 0x7FFF, 0x00).await.map_err(Error::Bus)?;
                self.bus.write_byte(self.address, 0x09, 0x04).await.map_err(Error::Bus)?;
                // Deep-sleep marker check: VL53L8CX ULD only. The VL53L5CX
                // ULD has no deep sleep and goes straight to the boot poll
                // (differential test vs ST's L5 code, KNOWN_DIVERGENCES #3).
                let mut was_deep_sleep = false;
                if self.variant.info().has_deep_sleep {
                    let stored = self.bus.read_byte(self.address, 0x000F).await.map_err(Error::Bus)?;
                    was_deep_sleep = stored == 0x43;
                    if was_deep_sleep {
                        self.bus.write_byte(self.address, 0x000F, 0x40).await.map_err(Error::Bus)?;
                    }
                }
                ops::poll_for_answer(
                    &mut self.bus, self.address, &mut self.delay,
                    &mut self.stdev.temp_buffer, 1, 0, 0x06, 0x01, 1,
                ).await?;
                if was_deep_sleep {
                    // Re-init after deep sleep, using the detected variant.
                    let mut fw = self.variant.firmware();
                    let info = self.variant.info();
                    ops::init_sensor(
                        &mut self.bus, self.address, &mut self.delay, &mut self.stdev,
                        &mut fw, info,
                    ).await?;
                }
            }
            PowerMode::Sleep => {
                self.bus.write_byte(self.address, 0x7FFF, 0x00).await.map_err(Error::Bus)?;
                self.bus.write_byte(self.address, 0x09, 0x02).await.map_err(Error::Bus)?;
                ops::poll_for_answer(
                    &mut self.bus, self.address, &mut self.delay,
                    &mut self.stdev.temp_buffer, 1, 0, 0x06, 0x01, 0,
                ).await?;
            }
            PowerMode::DeepSleep => {
                self.bus.write_byte(self.address, 0x7FFF, 0x00).await.map_err(Error::Bus)?;
                self.bus.write_byte(self.address, 0x09, 0x02).await.map_err(Error::Bus)?;
                ops::poll_for_answer(
                    &mut self.bus, self.address, &mut self.delay,
                    &mut self.stdev.temp_buffer, 1, 0, 0x06, 0x01, 0,
                ).await?;
                self.bus.write_byte(self.address, 0x000F, 0x43).await.map_err(Error::Bus)?;
            }
        }

        self.bus.write_byte(self.address, 0x7FFF, 0x02).await.map_err(Error::Bus)?;
        Ok(())
    }

    /// Read the power mode. Default after [`Sensor::init`] is
    /// [`PowerMode::Wakeup`]. Deep sleep is only distinguished from sleep on a VL53L8CX.
    pub async fn get_power_mode(&mut self) -> Result<PowerMode, Error<BUS::Error>> {
        self.bus.write_byte(self.address, 0x7FFF, 0x00).await.map_err(Error::Bus)?;
        let tmp = self.bus.read_byte(self.address, 0x009).await.map_err(Error::Bus)?;

        let mode = match tmp {
            0x04 => PowerMode::Wakeup,
            // Deep-sleep state (reg 0x000F == 0x43) only exists on the L8; on
            // parts without it, 0x02 is plain sleep.
            0x02 if self.variant.info().has_deep_sleep => {
                let mode_reg = self.bus.read_byte(self.address, 0x000F).await.map_err(Error::Bus)?;
                if mode_reg == 0x43 {
                    PowerMode::DeepSleep
                } else {
                    PowerMode::Sleep
                }
            }
            0x02 => PowerMode::Sleep,
            _ => {
                self.bus.write_byte(self.address, 0x7FFF, 0x02).await.map_err(Error::Bus)?;
                return Err(Error::Firmware(FirmwareError::Unknown(types::VL53L8CX_STATUS_ERROR)));
            }
        };

        self.bus.write_byte(self.address, 0x7FFF, 0x02).await.map_err(Error::Bus)?;
        Ok(mode)
    }

    /// Set the period of the firmware's temperature (VHV) recalibration in
    /// frames: `count` > 0 recalibrates every `count` frames, each run taking a
    /// few ms; 0 (default after [`Sensor::init`]) disables it, so compensation is
    /// calibrated once at the start of each ranging session only. The value is
    /// not validated. Not while ranging.
    pub async fn set_vhv_repeat_count(&mut self, count: u32) -> Result<(), Error<BUS::Error>> {
        let count_bytes = count.to_le_bytes();
        let mut data = [0u8; 16];
        ops::dci_replace(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut data,
            consts::VL53L8CX_DCI_VHV_CONFIG as u32, 16,
            &count_bytes, 0x04,
        ).await
    }

    /// Read the temperature (VHV) recalibration period in frames; 0 (the
    /// default after [`Sensor::init`]) means disabled. See
    /// [`Sensor::set_vhv_repeat_count`].
    pub async fn get_vhv_repeat_count(&mut self) -> Result<u32, Error<BUS::Error>> {
        let mut buf = [0u8; 16];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_VHV_CONFIG as u32, 16,
        ).await?;
        Ok(u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]))
    }

    /// Enable/disable the external synchronisation pin (VL53L8CX only).
    pub async fn set_external_sync_pin_enable(&mut self, enabled: bool) -> Result<(), Error<BUS::Error>> {
        if !self.variant.info().has_sync_pin {
            return Err(Error::Firmware(FirmwareError::InvalidParam));
        }
        let mut buf = [0u8; 4];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_SYNC_PIN as u32, 4,
        ).await?;

        if enabled {
            buf[3] |= 0x02;
        } else {
            buf[3] &= !0x02;
        }

        ops::dci_write(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_SYNC_PIN as u32, 4,
        ).await
    }

    /// Read the external synchronisation pin enable state (VL53L8CX only).
    pub async fn get_external_sync_pin_enable(&mut self) -> Result<bool, Error<BUS::Error>> {
        if !self.variant.info().has_sync_pin {
            return Err(Error::Firmware(FirmwareError::InvalidParam));
        }
        let mut buf = [0u8; 4];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_SYNC_PIN as u32, 4,
        ).await?;
        Ok((buf[3] & 0x02) != 0)
    }

    /// Enable the internal charge pump (VL53L5CX only).
    pub async fn enable_internal_cp(&mut self) -> Result<(), Error<BUS::Error>> {
        self.set_internal_cp(true).await
    }

    /// Disable the internal charge pump (VL53L5CX only).
    pub async fn disable_internal_cp(&mut self) -> Result<(), Error<BUS::Error>> {
        self.set_internal_cp(false).await
    }

    async fn set_internal_cp(&mut self, enable: bool) -> Result<(), Error<BUS::Error>> {
        if !self.variant.info().has_internal_cp {
            return Err(Error::Firmware(FirmwareError::InvalidParam));
        }
        // Two read-modify-writes into the internal-CP DCI block: vcsel bootup
        // FSM at 0x0A and analog dynamic pad 0 at 0x0E (inverted of each other).
        let vcsel_bootup_fsm: u8 = enable as u8;
        let analog_dynamic_pad_0: u8 = !enable as u8;
        let mut data = [0u8; 16];
        ops::dci_replace(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut data,
            consts::VL53L8CX_DCI_INTERNAL_CP as u32, 16,
            &[vcsel_bootup_fsm], 0x0A,
        ).await?;
        ops::dci_replace(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut data,
            consts::VL53L8CX_DCI_INTERNAL_CP as u32, 16,
            &[analog_dynamic_pad_0], 0x0E,
        ).await
    }
}

// --- Detection thresholds (requires "thresholds" feature) ---

#[cfg(feature = "thresholds")]
impl<BUS, D, LPN, INT, PWR> Sensor<BUS, D, LPN, INT, PWR>
where
    BUS: VlBusAsync,
    D: DelayNs,
{
    /// Read whether the detection thresholds are enabled. Default after
    /// [`Sensor::init`] is off. See [`Sensor::set_detection_thresholds_enable`].
    pub async fn get_detection_thresholds_enable(&mut self) -> Result<bool, Error<BUS::Error>> {
        let mut buf = [0u8; 8];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_DET_THRESH_GLOBAL_CONFIG as u32, 8,
        ).await?;
        Ok(buf[0x01] != 0)
    }

    /// Enable or disable the detection thresholds programmed with
    /// [`Sensor::set_detection_thresholds`] (default off after [`Sensor::init`]).
    /// When enabled, INT fires only for a frame in which a zone meets its
    /// threshold, instead of for every frame; combined with
    /// [`RangingMode::Autonomous`] (and a threshold on the motion indicator)
    /// this is the lowest-power presence detection. Program the thresholds
    /// first, then enable, then [`Sensor::start_ranging`].
    pub async fn set_detection_thresholds_enable(&mut self, enabled: bool) -> Result<(), Error<BUS::Error>> {
        let grp_global_config: [u8; 4] = if enabled {
            [0x01, 0x01, 0x01, 0x00]
        } else {
            [0x01, 0x00, 0x01, 0x00]
        };
        let tmp: u8 = if enabled { 0x04 } else { 0x0C };

        let mut data = [0u8; 8];
        ops::dci_replace(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut data,
            consts::VL53L8CX_DCI_DET_THRESH_GLOBAL_CONFIG as u32, 8,
            &grp_global_config, 0x00,
        ).await?;

        let mut data2 = [0u8; 20];
        ops::dci_replace(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut data2,
            consts::VL53L8CX_DCI_DET_THRESH_CONFIG as u32, 20,
            &[tmp], 0x11,
        ).await
    }

    fn scale_thresholds_for_read(thresholds: &mut [DetectionThreshold]) {
        for t in thresholds.iter_mut() {
            let scale: i32 = match t.measurement {
                ThresholdMeasurement::DistanceMm => 4,
                ThresholdMeasurement::SignalPerSpadKcps
                | ThresholdMeasurement::AmbientPerSpadKcps => 2048,
                ThresholdMeasurement::RangeSigmaMm => 128,
                ThresholdMeasurement::NbSpadsEnabled => 256,
                ThresholdMeasurement::MotionIndicator => 65535,
                _ => 1,
            };
            if scale != 1 {
                t.low_threshold /= scale;
                t.high_threshold /= scale;
            }
        }
    }

    fn scale_thresholds_for_write(thresholds: &mut [DetectionThreshold]) {
        for t in thresholds.iter_mut() {
            let scale: i32 = match t.measurement {
                ThresholdMeasurement::DistanceMm => 4,
                ThresholdMeasurement::SignalPerSpadKcps
                | ThresholdMeasurement::AmbientPerSpadKcps => 2048,
                ThresholdMeasurement::RangeSigmaMm => 128,
                ThresholdMeasurement::NbSpadsEnabled => 256,
                ThresholdMeasurement::MotionIndicator => 65535,
                _ => 1,
            };
            if scale != 1 {
                t.low_threshold *= scale;
                t.high_threshold *= scale;
            }
        }
    }

    /// Read the 64 detection-threshold entries, with `low_threshold` /
    /// `high_threshold` converted back from the firmware's fixed-point format
    /// to each entry's natural units (mm, kcps/SPAD, ...). See
    /// [`DetectionThreshold`] and [`Sensor::set_detection_thresholds`].
    pub async fn get_detection_thresholds(
        &mut self,
    ) -> Result<[DetectionThreshold; NB_THRESHOLDS], Error<BUS::Error>> {
        // Each DetectionThreshold is 12 bytes (repr(C)), total = 64 * 12 = 768
        const THRESH_SIZE: u16 = (NB_THRESHOLDS * core::mem::size_of::<DetectionThreshold>()) as u16;
        let mut buf = [0u8; THRESH_SIZE as usize];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_DET_THRESH_START as u32, THRESH_SIZE,
        ).await?;

        // Safety: DetectionThreshold is repr(C) with known size, and buf is properly aligned
        // on the stack. We transmute the raw bytes into the threshold array.
        let mut thresholds: [DetectionThreshold; NB_THRESHOLDS] =
            unsafe { core::mem::transmute(buf) };
        Self::scale_thresholds_for_read(&mut thresholds);
        Ok(thresholds)
    }

    /// Read the detection-thresholds auto-stop flag (VL53L8CX only).
    ///
    /// See [`Sensor::set_detection_thresholds_auto_stop`]. Returns
    /// [`FirmwareError::InvalidParam`] on a VL53L5CX, whose ULD has no such
    /// function.
    pub async fn get_detection_thresholds_auto_stop(&mut self) -> Result<bool, Error<BUS::Error>> {
        if !self.variant.info().has_threshold_auto_stop {
            return Err(Error::Firmware(FirmwareError::InvalidParam));
        }
        ops::get_detection_thresholds_auto_stop(
            &mut self.bus, self.address, &mut self.delay, &mut self.stdev,
        ).await
    }

    /// Enable or disable the detection-thresholds auto-stop (VL53L8CX only;
    /// default off).
    ///
    /// In autonomous ranging with detection thresholds enabled the sensor only
    /// raises INT when a threshold is met, at the end of the measurement. With
    /// auto-stop on it raises INT and aborts the measurement in progress
    /// straight away; [`Sensor::stop_ranging`] must still be called afterwards
    /// for a clean stop, and it skips the MCU-stop handshake when this flag is
    /// set. Ports `vl53l8cx_set_detection_thresholds_auto_stop`. Returns
    /// [`FirmwareError::InvalidParam`] on a VL53L5CX, whose ULD has no such
    /// function.
    pub async fn set_detection_thresholds_auto_stop(&mut self, enabled: bool) -> Result<(), Error<BUS::Error>> {
        if !self.variant.info().has_threshold_auto_stop {
            return Err(Error::Firmware(FirmwareError::InvalidParam));
        }
        ops::set_detection_thresholds_auto_stop(
            &mut self.bus, self.address, &mut self.delay, &mut self.stdev, enabled,
        ).await
    }

    /// Program the 64 detection-threshold entries (`VL53L8CX_NB_THRESHOLDS`).
    /// Give `low_threshold` / `high_threshold` in each entry's natural units
    /// (mm, kcps/SPAD, ...); the driver applies the firmware's fixed-point scaling.
    /// Three layouts: 4x4 with one entry per zone (16 used), 4x4 with two per
    /// zone (32; `mathematic_operation` combines the pair), or 8x8 with one per
    /// zone (64). ST's header requires the caller to OR
    /// [`VL53L8CX_LAST_THRESHOLD`] (128) into the `zone_num` of the last entry
    /// used (`zone_num: 15 | VL53L8CX_LAST_THRESHOLD`) and to leave unused
    /// entries zero; the array is sent exactly as given, byte-identical to ST's
    /// ULD, so a missing flag is not detected and the readback returns it as
    /// written. Then enable with [`Sensor::set_detection_thresholds_enable`]
    /// before ranging.
    pub async fn set_detection_thresholds(
        &mut self,
        mut thresholds: [DetectionThreshold; NB_THRESHOLDS],
    ) -> Result<(), Error<BUS::Error>> {
        Self::scale_thresholds_for_write(&mut thresholds);

        // Write valid target config
        let mut grp_valid_target_cfg: [u8; 8] = [0x05; 8];
        ops::dci_write(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut grp_valid_target_cfg,
            consts::VL53L8CX_DCI_DET_THRESH_VALID_STATUS as u32, 8,
        ).await?;

        // Write thresholds
        const THRESH_SIZE: u16 = (NB_THRESHOLDS * core::mem::size_of::<DetectionThreshold>()) as u16;
        let mut buf: [u8; THRESH_SIZE as usize] = unsafe { core::mem::transmute(thresholds) };
        ops::dci_write(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_DET_THRESH_START as u32, THRESH_SIZE,
        ).await
    }
}

// --- Motion indicator (requires "motion" or "thresholds" feature) ---

#[cfg(any(feature = "motion", feature = "thresholds"))]
impl<BUS, D, LPN, INT, PWR> Sensor<BUS, D, LPN, INT, PWR>
where
    BUS: VlBusAsync,
    D: DelayNs,
{
    /// Initialise the motion indicator: fill `motion_config` with ST's defaults
    /// (movement window 400..=1500 mm) for `resolution` and send it to the
    /// sensor. `resolution` must match the sensor's; after every later
    /// [`Sensor::set_resolution`] call [`Sensor::motion_indicator_set_resolution`].
    /// Adjust the window with [`Sensor::motion_indicator_set_distance_motion`].
    /// The per-aggregate `motion` results grow with movement (a typical moving
    /// target gives 100..=500); call before [`Sensor::start_ranging`].
    pub async fn motion_indicator_init(
        &mut self,
        motion_config: &mut MotionConfiguration,
        resolution: Resolution,
    ) -> Result<(), Error<BUS::Error>> {
        // Zero and set defaults
        *motion_config = unsafe { core::mem::zeroed() };
        motion_config.ref_bin_offset = 13633;
        motion_config.detection_threshold = 2883584;
        motion_config.extra_noise_sigma = 0;
        motion_config.null_den_clip_value = 0;
        motion_config.mem_update_mode = 6;
        motion_config.mem_update_choice = 2;
        motion_config.sum_span = 4;
        motion_config.feature_length = 9;
        motion_config.nb_of_aggregates = 16;
        motion_config.nb_of_temporal_accumulations = 16;
        motion_config.min_nb_for_global_detection = 1;
        motion_config.global_indicator_format_1 = 8;
        motion_config.global_indicator_format_2 = 0;

        self.motion_indicator_set_resolution(motion_config, resolution).await
    }

    /// Set the distance window the motion indicator watches, in mm, and send the
    /// updated `motion_config`. Requires `min <= max`, `min >= 400`,
    /// `max <= 4000` and `max - min <= 1500`; otherwise returns
    /// `Error::Firmware(FirmwareError::InvalidParam)` without touching the bus
    /// (ST's ULD lets `max < min` through and computes garbage).
    /// Default window 400..=1500 mm. Sensitivity also depends on integration
    /// time, distance and target reflectance. Not while ranging.
    pub async fn motion_indicator_set_distance_motion(
        &mut self,
        motion_config: &mut MotionConfiguration,
        distance_min_mm: u16,
        distance_max_mm: u16,
    ) -> Result<(), Error<BUS::Error>> {
        // Check the order first: the u16 subtraction below would overflow
        // (and panic in debug builds) for max < min.
        if distance_min_mm > distance_max_mm
            || distance_min_mm < 400
            || distance_max_mm > 4000
            || (distance_max_mm - distance_min_mm) > 1500
        {
            return Err(Error::Firmware(FirmwareError::InvalidParam));
        }

        // ref_bin_offset = ((distance_min / 37.5348) - 4.0) * 2048.5
        let min_f = distance_min_mm as f32;
        let tmp = ((min_f / 37.5348) - 4.0) * 2048.5;
        motion_config.ref_bin_offset = tmp as i32;

        // feature_length = (((max - min) / 10.0 + 30.02784) / 15.01392) + 0.5
        let range_f = (distance_max_mm - distance_min_mm) as f32;
        let tmp = ((range_f / 10.0 + 30.02784) / 15.01392) + 0.5;
        motion_config.feature_length = tmp as u8;

        let config_size = core::mem::size_of::<MotionConfiguration>() as u16;
        let mut buf: [u8; core::mem::size_of::<MotionConfiguration>()] =
            unsafe { core::mem::transmute_copy(motion_config) };
        ops::dci_write(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_MOTION_DETECTOR_CFG as u32, config_size,
        ).await
    }

    /// Rebuild the motion indicator's zone map for `resolution` and send the
    /// updated `motion_config`. Call after every [`Sensor::set_resolution`]: the
    /// motion plugin does not follow the sensor's resolution by itself. The 4x4
    /// map uses one aggregate per zone, the 8x8 map groups the 64 zones into 16
    /// aggregates of 2x2. Not while ranging.
    pub async fn motion_indicator_set_resolution(
        &mut self,
        motion_config: &mut MotionConfiguration,
        resolution: Resolution,
    ) -> Result<(), Error<BUS::Error>> {
        match resolution {
            Resolution::Res4x4 => {
                for i in 0..16u8 {
                    motion_config.map_id[i as usize] = i as i8;
                }
                for i in 16..64 {
                    motion_config.map_id[i] = -1;
                }
            }
            Resolution::Res8x8 => {
                for i in 0..64u8 {
                    motion_config.map_id[i as usize] =
                        ((i % 8) / 2 + 4 * (i / 16)) as i8;
                }
            }
        }

        let config_size = core::mem::size_of::<MotionConfiguration>() as u16;
        let mut buf: [u8; core::mem::size_of::<MotionConfiguration>()] =
            unsafe { core::mem::transmute_copy(motion_config) };
        ops::dci_write(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_MOTION_DETECTOR_CFG as u32, config_size,
        ).await
    }
}

// --- Xtalk (requires "xtalk" feature) ---

#[cfg(feature = "xtalk")]
impl<BUS, D, LPN, INT, PWR> Sensor<BUS, D, LPN, INT, PWR>
where
    BUS: VlBusAsync,
    D: DelayNs,
{
    /// Read the crosstalk margin in kcps/SPAD. ST documents the default as
    /// 50 kcps/SPAD. See [`Sensor::set_xtalk_margin`].
    pub async fn get_xtalk_margin(&mut self) -> Result<u32, Error<BUS::Error>> {
        let mut buf = [0u8; 16];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_XTALK_CFG as u32, 16,
        ).await?;
        let margin = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        Ok(margin / 2048)
    }

    /// Set the crosstalk margin in kcps/SPAD, 0..=10000; above that returns
    /// `Error::Firmware(FirmwareError::InvalidParam)` without touching the bus.
    /// ST documents the default as 50 kcps/SPAD. The margin raises the
    /// crosstalk detection threshold so a cover glass is never reported as a
    /// target once a calibration ([`Sensor::calibrate_xtalk`] /
    /// [`Sensor::set_caldata_xtalk`]) is loaded, and lets one unit's calibration
    /// be reused on others and tuned per unit. Not while ranging.
    pub async fn set_xtalk_margin(&mut self, margin: u32) -> Result<(), Error<BUS::Error>> {
        if margin > 10000 {
            return Err(Error::Firmware(FirmwareError::InvalidParam));
        }
        let margin_kcps = margin * 2048;
        let mut data = [0u8; 16];
        ops::dci_replace(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut data,
            consts::VL53L8CX_DCI_XTALK_CFG as u32, 16,
            &margin_kcps.to_le_bytes(), 0x00,
        ).await
    }

    /// Read the crosstalk shape the sensor is currently using into `buffer`
    /// (at least [`XTALK_BUFFER_SIZE`] bytes): the ULD default, the shape
    /// produced by [`Sensor::calibrate_xtalk`], or whatever
    /// [`Sensor::set_caldata_xtalk`] loaded. Store it to replay with
    /// `set_caldata_xtalk` after the next power-up.
    pub async fn get_caldata_xtalk(&mut self, buffer: &mut [u8]) -> Result<(), Error<BUS::Error>> {
        if buffer.len() < XTALK_BUFFER_SIZE {
            return Err(Error::Firmware(FirmwareError::InvalidParam));
        }

        // Save and set 8x8 resolution
        let resolution = self.get_resolution().await?;
        self.set_resolution(Resolution::Res8x8).await?;

        ops::fetch_xtalk_data(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, buffer,
        ).await?;

        // Restore resolution
        self.set_resolution(resolution).await
    }

    /// Run the firmware's crosstalk calibration against a flat, uniform
    /// target and keep the resulting shape (port of `vl53l8cx_calibrate_xtalk`;
    /// one code path for both parts, see [`sensor::L5_CALIBRATE_XTALK_PATCH`]).
    ///
    /// Crosstalk is the light reflected back into the receiver by a coverglass
    /// in front of the sensor. ST recommends running this calibration once per
    /// device whenever a coverglass is fitted; without one the default shape
    /// is adequate and the firmware will usually refuse to calibrate.
    ///
    /// # Setup
    ///
    /// Place a flat target of known, low reflectance so that it fills the full
    /// field of view, in a dark environment, and pass:
    ///
    /// - `reflectance_percent`: the target's reflectance, 1..=99 %. ST
    ///   recommends a 3 % target (a matte dark grey card); 5 % is common.
    /// - `nb_samples`: frames averaged, 1..=16. More samples give a more
    ///   accurate shape but take longer; 4 is a good compromise.
    /// - `distance_mm`: target distance, 600..=3000 mm. The target must stay
    ///   inside the full FoV, so short distances (600 mm) are easiest.
    ///
    /// Arguments outside those ranges return [`FirmwareError::InvalidParam`]
    /// before anything is sent to the sensor.
    ///
    /// # Behaviour
    ///
    /// The calibration takes a few seconds (the driver waits up to 20 s). On
    /// return the sensor is back in the configuration it had on entry
    /// (resolution, ranging frequency, integration time, sharpener, target
    /// order, crosstalk margin and ranging mode are saved and restored, as the
    /// ULD does), with the new crosstalk shape applied. It is kept in the
    /// driver state and re-sent on every [`Sensor::set_resolution`]; retrieve
    /// it with [`Sensor::get_caldata_xtalk`] to persist it, and replay it with
    /// [`Sensor::set_caldata_xtalk`] after each [`Sensor::init`]. The margin
    /// ([`Sensor::set_xtalk_margin`]) can then be used to trade false
    /// positives for sensitivity.
    ///
    /// # Errors
    ///
    /// - [`FirmwareError::XtalkFailed`]: the firmware found the coverglass
    ///   too good (too little crosstalk) to calibrate, or there was no
    ///   suitable target; the default shape is restored. Expected when no
    ///   coverglass is fitted.
    /// - [`FirmwareError::Timeout`]: the calibration did not finish within
    ///   20 s.
    ///
    /// In both cases the previous configuration has still been restored and
    /// the sensor is usable.
    pub async fn calibrate_xtalk(
        &mut self,
        reflectance_percent: u8,
        nb_samples: u8,
        distance_mm: u16,
    ) -> Result<(), Error<BUS::Error>> {
        // Validate before the configuration reads below touch the bus (the
        // ULD reads first, then checks; see `ops::calibrate_xtalk`).
        if !ops::xtalk_params_valid(reflectance_percent, nb_samples, distance_mm) {
            return Err(Error::Firmware(FirmwareError::InvalidParam));
        }

        // Get initial configuration, in the ULD's order (resolution first).
        let resolution = self.get_resolution().await? as u8;
        let frequency = self.get_ranging_frequency_hz().await?;
        let integration_time_ms = self.get_integration_time_ms().await?;
        let sharpener = self.get_sharpener_percent().await?;
        let target_order = self.get_target_order().await?;
        let xtalk_margin = self.get_xtalk_margin().await?;
        let ranging_mode = self.get_ranging_mode().await?;

        let outcome = ops::calibrate_xtalk(
            &mut self.bus, self.address, &mut self.delay, &mut self.stdev,
            self.variant.info(), resolution, reflectance_percent, nb_samples, distance_mm,
        ).await;

        // Reset initial configuration, in the ULD's order and whatever the
        // calibration outcome, so the sensor is left as it was found. The op
        // has already reset the default configuration and the resolution.
        self.set_ranging_frequency_hz(frequency).await?;
        self.set_integration_time_ms(integration_time_ms).await?;
        self.set_sharpener_percent(sharpener).await?;
        self.set_target_order(target_order).await?;
        self.set_xtalk_margin(xtalk_margin).await?;
        self.set_ranging_mode(ranging_mode).await?;

        outcome
    }

    /// Load a crosstalk shape saved with [`Sensor::get_caldata_xtalk`] (e.g. from
    /// flash) so a calibrated cover glass works without recalibrating after
    /// each power-up. `buffer` must hold at least [`XTALK_BUFFER_SIZE`] (776)
    /// bytes, else `Error::Firmware(FirmwareError::InvalidParam)`; only the
    /// first 776 are used. Call after [`Sensor::init`] and before ranging; set
    /// [`Sensor::set_xtalk_margin`] if the shape came from another unit.
    /// The shape is kept in the driver state and re-sent on every
    /// [`Sensor::set_resolution`].
    pub async fn set_caldata_xtalk(&mut self, buffer: &mut [u8]) -> Result<(), Error<BUS::Error>> {
        if buffer.len() < XTALK_BUFFER_SIZE {
            return Err(Error::Firmware(FirmwareError::InvalidParam));
        }

        // Save resolution, copy xtalk data, re-apply resolution to trigger send_xtalk_data
        let resolution = self.get_resolution().await?;
        self.stdev.xtalk_data[..XTALK_BUFFER_SIZE]
            .copy_from_slice(&buffer[..XTALK_BUFFER_SIZE]);
        self.set_resolution(resolution).await
    }
}
