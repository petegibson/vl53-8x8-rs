#![no_std]
/*!
`vl53l7cx-uld`: a safe, blocking, platform-agnostic wrapper over the official
STMicroelectronics VL53L7CX ULD (Ultra Lite Driver).

The C ULD is compiled and bound by the `vl53l7cx-uld-sys` crate (bindgen + cc,
so libclang is needed at build time); this crate exposes it through
`embedded-hal` 1.0 I2C/SPI and delay traits. The ULD calls back into Rust for
bus access, so a program must invoke [`bind_platform_driver!`] or
[`bind_platform_driver_spi!`] exactly once with its concrete bus and delay
types.

This crate mirrors `vl53l8cx-uld` method for method so the two ST ULDs can be
driven through one API shape (its main use today is host-side differential
testing of ST's L5 C code against a mock bus). The differences follow the ULD
headers:

* The VL53L7CX ULD has no deep-sleep power mode:
  [`set_power_mode`](Vl53l7cx::set_power_mode) rejects
  [`PowerMode::DeepSleep`] with `Error::Firmware(FirmwareError::InvalidParam)`.
* No external sync pin (`get/set_external_sync_pin_enable`) and no
  detection-threshold auto-stop (`get/set_detection_thresholds_auto_stop`).
* The L5-only [`enable_internal_cp`](Vl53l7cx::enable_internal_cp) /
  [`disable_internal_cp`](Vl53l7cx::disable_internal_cp) are added.
* The ULD status byte is decoded with the shared [`FirmwareError`]; the L5 ULD
  names code 3 `CRC_CSUM_FAILED` (decoded as `LaserSafety`, the L8 name) but
  never returns it, and has no code 5 (`FwChecksumFail`).

For a pure-Rust driver covering both the VL53L7CX and VL53L8CX with no C
toolchain requirement, see the `vl53-8x8` crate. Shared types come from
`vl53-8x8-types` and are re-exported here.
*/

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::SpiDevice;

// We import the sys crate internally but do NOT make it public.
use vl53l7cx_uld_sys as sys;

// --- Public Type Re-exports from sys ---
pub use sys::VL53L7CX_Platform;
/// One frame of ranging results, as filled in by [`Vl53l7cx::get_ranging_data`].
///
/// This is ST's `VL53L7CX_ResultsData`; its fields depend on the `disable-*`
/// features. Arrays come in two layouts:
///
/// * **Per zone** (`ambient_per_spad`, `nb_target_detected`, `nb_spads_enabled`):
///   64 entries, indexed by zone. In 4x4 mode only the first 16 are used.
/// * **Per target** (`distance_mm`, `target_status`, `signal_per_spad`,
///   `range_sigma_mm`, `reflectance`): `64 * NB_TARGET_PER_ZONE` entries,
///   indexed `zone * NB_TARGET_PER_ZONE + target`. With the default of one
///   target per zone this collapses to plain zone indexing.
///
/// Only the first `nb_target_detected[zone]` targets of a zone are valid;
/// check `target_status` (5 or 9 means a good range) before using a value.
///
/// ```rust,ignore
/// use vl53l7cx_uld::NB_TARGET_PER_ZONE;
/// let frame = sensor.get_ranging_data()?;
/// for zone in 0..16 {
///     for target in 0..usize::from(frame.nb_target_detected[zone]) {
///         let i = zone * NB_TARGET_PER_ZONE + target;
///         let (mm, status) = (frame.distance_mm[i], frame.target_status[i]);
///     }
/// }
/// ```
pub use sys::VL53L7CX_ResultsData as ResultsData;
/// Number of targets reported per zone, selected by the `targets-N` features
/// (default 1). Per-target arrays in [`ResultsData`] are indexed
/// `zone * NB_TARGET_PER_ZONE + target`.
pub use sys::VL53L7CX_NB_TARGET_PER_ZONE as NB_TARGET_PER_ZONE;
pub use sys::{
    Resolution, TargetOrder, RangingMode, PowerMode,
    FirmwareError, Error, MaybeOutputPin, NoPin, DEFAULT_ADDRESS, XTALK_BUFFER_SIZE,
};
#[cfg(feature = "thresholds")]
pub use sys::{
    ThresholdMeasurement, ThresholdType, ThresholdOperation, DetectionThreshold,
    VL53L8CX_LAST_THRESHOLD,
};

#[cfg(any(feature = "motion", feature = "thresholds"))]
pub use sys::VL53L7CX_Motion_Configuration as MotionConfiguration;

#[cfg(feature = "thresholds")]
// The threshold constants live in `vl53-8x8-types` under their L8 names; the
// L5 plugin header defines the same value (64).
use sys::VL53L8CX_NB_THRESHOLDS as VL53L7CX_NB_THRESHOLDS;

/// Trait abstracting bus error types for I2C and SPI.
pub trait VlBus {
    type Error: core::fmt::Debug;
}

impl<T: I2c> VlBus for T {
    type Error = <T as embedded_hal::i2c::ErrorType>::Error;
}

/// Wrapper around an `SpiDevice` to implement `VlBus`.
pub struct Vl53l7cxSpiDevice<SPI> {
    pub spi: SPI,
}

impl<SPI: SpiDevice> VlBus for Vl53l7cxSpiDevice<SPI> {
    type Error = <SPI as embedded_hal::spi::ErrorType>::Error;
}

#[repr(C)]
pub struct Vl53l7cxComms<BUS, D> {
    pub bus: BUS,
    pub delay: D,
}

/// Driver handle. `LPN`, `INT` and `PWR` are the optional LPn (I2C comms
/// enable), INT (data ready) and power-enable pins, [`NoPin`] when absent;
/// the `with_*_pin*` builders set them.
pub struct Vl53l7cx<BUS, D, LPN = NoPin, INT = NoPin, PWR = NoPin> {
    pub comms: Vl53l7cxComms<BUS, D>,
    address: SevenBitAddress,
    lpn_pin: LPN,
    int_pin: INT,
    pwr_pin: PWR,
    /// `true` when the line in `lpn_pin` is inverted (an inverted LPn or the
    /// VL53L7CX's I2C_RST attached via the deprecated active-high builder).
    lpn_inverted: bool,
    int_is_data_ready_high: bool,
    pwr_active_high: bool,
    stdev: sys::VL53L7CX_Configuration,
}

impl<BUS, D> Vl53l7cx<BUS, D, NoPin, NoPin, NoPin>
where
    BUS: I2c,
    D: DelayNs,
{
    /// Create a driver over an `embedded-hal` I2C bus at [`DEFAULT_ADDRESS`]
    /// (0x29), with no LPn, INT or power-enable pin attached. Nothing is sent
    /// to the sensor until [`Vl53l7cx::init`]. Attach pins with
    /// [`Vl53l7cx::with_lpn_pin`], the `with_interrupt_pin_*` and
    /// `with_power_enable_pin_*` builders. The program must also invoke
    /// [`bind_platform_driver!`] once with the same bus and delay types so the
    /// ULD can call back into Rust.
    pub fn new(i2c: BUS, delay: D) -> Self {
        Self {
            comms: Vl53l7cxComms { bus: i2c, delay },
            address: DEFAULT_ADDRESS,
            lpn_pin: NoPin,
            int_pin: NoPin,
            pwr_pin: NoPin,
            lpn_inverted: false,
            int_is_data_ready_high: false,
            pwr_active_high: true,
            stdev: unsafe { core::mem::zeroed() },
        }
    }
}

impl<SPI, D> Vl53l7cx<Vl53l7cxSpiDevice<SPI>, D, NoPin, NoPin, NoPin>
where
    SPI: SpiDevice,
    D: DelayNs,
{
    /// Create a driver over an `embedded-hal` `SpiDevice`, for API parity
    /// with `vl53l8cx-uld`; the VL53L7CX itself is an I2C-only part.
    /// Otherwise as [`Vl53l7cx::new`]: no pins attached, nothing sent until
    /// [`Vl53l7cx::init`]; the program must invoke [`bind_platform_driver_spi!`]
    /// once with the same SPI and delay types.
    pub fn new_spi(spi: SPI, delay: D) -> Self {
        Self {
            comms: Vl53l7cxComms { bus: Vl53l7cxSpiDevice { spi }, delay },
            address: DEFAULT_ADDRESS,
            lpn_pin: NoPin,
            int_pin: NoPin,
            pwr_pin: NoPin,
            lpn_inverted: false,
            int_is_data_ready_high: false,
            pwr_active_high: true,
            stdev: unsafe { core::mem::zeroed() },
        }
    }
}

impl<BUS, D, LPN, INT, PWR, E> Vl53l7cx<BUS, D, LPN, INT, PWR>
where
    BUS: VlBus<Error = E>,
    INT: InputPin,
    <INT as embedded_hal::digital::ErrorType>::Error: core::fmt::Debug,
{
    /// Busy-wait on the INT pin until it shows data ready (the level set by
    /// the `with_interrupt_pin_*` builder), then return; follow with
    /// [`Vl53l7cx::get_ranging_data`]. Spins without delay and without bus
    /// traffic. The alternative without an INT pin is polling
    /// [`Vl53l7cx::check_data_ready`].
    pub fn wait_for_data_ready_blocking(&mut self) -> Result<(), Error<E>> {
        loop {
            let is_high_result = self.int_pin.is_high().map_err(|_| Error::Pin)?;
            let is_data_ready = is_high_result == self.int_is_data_ready_high;

            if is_data_ready {
                break;
            }
        }
        Ok(())
    }
}

impl<BUS, D, LPN, INT, PWR> Vl53l7cx<BUS, D, LPN, INT, PWR> {
    /// Attach the sensor's **LPn** line, which is always active low.
    ///
    /// LPn is what the datasheet calls the "Comms enable": low disables the
    /// sensor's I2C interface, high enables it. It is not a reset and has
    /// nothing to do with power. Its purpose is multi-sensor address
    /// assignment: hold every other sensor's LPn low
    /// ([`lpn_disable`](Self::lpn_disable)), move one sensor with
    /// [`Vl53l7cx::set_i2c_address`], then release the others
    /// ([`lpn_enable`](Self::lpn_enable)). Leave it high otherwise; `init()`
    /// needs it high. When attached, [`power_cycle`](Self::power_cycle) also
    /// drives it low with the supplies, as UM2884 §4.2 requires. An LPn that
    /// hangs only on its pull-up to IOVDD needs no GPIO at all. The pin's
    /// level is not changed here.
    pub fn with_lpn_pin<LpnNew>(self, lpn_pin: LpnNew) -> Vl53l7cx<BUS, D, LpnNew, INT, PWR> {
        let Vl53l7cx { comms, address, int_pin, pwr_pin, int_is_data_ready_high, pwr_active_high, stdev, .. } = self;
        Vl53l7cx {
            comms,
            address,
            lpn_pin,
            int_pin,
            pwr_pin,
            lpn_inverted: false,
            int_is_data_ready_high,
            pwr_active_high,
            stdev,
        }
    }

    /// Deprecated name of [`with_lpn_pin`](Self::with_lpn_pin): LPn is the I2C
    /// comms-enable line, not a reset.
    #[deprecated(
        since = "0.2.0",
        note = "LPn is not a reset; use with_lpn_pin(), and power_cycle() for a real reset"
    )]
    pub fn with_reset_pin_active_low<RstNew>(self, reset_pin: RstNew) -> Vl53l7cx<BUS, D, RstNew, INT, PWR> {
        self.with_lpn_pin(reset_pin)
    }

    /// Deprecated: attach an **inverted** line in the LPn slot (high = held
    /// off). It was meant for an inverted LPn or for the VL53L7CX's I2C_RST,
    /// which resets only the I2C interface. The LPn methods and the deprecated
    /// [`reset`](Self::reset) honour the inversion, so existing code keeps its
    /// meaning. For I2C_RST use the free [`pulse_i2c_rst`] instead; for an
    /// inverted LPn use [`with_lpn_pin`](Self::with_lpn_pin) with your own
    /// inverting `OutputPin`.
    #[deprecated(
        since = "0.2.0",
        note = "attach I2C_RST with pulse_i2c_rst(), or an inverted LPn with with_lpn_pin() plus your own inversion"
    )]
    pub fn with_reset_pin_active_high<RstNew>(self, reset_pin: RstNew) -> Vl53l7cx<BUS, D, RstNew, INT, PWR> {
        let mut s = self.with_lpn_pin(reset_pin);
        s.lpn_inverted = true;
        s
    }

    /// Attach the sensor's INT (data-ready) pin, which goes **low** when a frame
    /// is ready. This is the part's default: INT is an open-drain output pulled
    /// low for the interrupt pulse.
    pub fn with_interrupt_pin_active_low<IntNew>(self, int_pin: IntNew) -> Vl53l7cx<BUS, D, LPN, IntNew, PWR> {
        let Vl53l7cx { comms, address, lpn_pin, pwr_pin, lpn_inverted, pwr_active_high, stdev, .. } = self;
        Vl53l7cx {
            comms,
            address,
            lpn_pin,
            int_pin,
            pwr_pin,
            lpn_inverted,
            int_is_data_ready_high: false,
            pwr_active_high,
            stdev,
        }
    }

    /// Attach a data-ready line that goes **high** when a frame is ready, for
    /// boards that invert the sensor's INT output.
    pub fn with_interrupt_pin_active_high<IntNew>(self, int_pin: IntNew) -> Vl53l7cx<BUS, D, LPN, IntNew, PWR> {
        let mut s = self.with_interrupt_pin_active_low(int_pin);
        s.int_is_data_ready_high = true;
        s
    }

    /// Attach a power-enable line where **high** powers the sensor (e.g. the
    /// X-NUCLEO-53L8A1 shield's `PWR_EN`). Enables [`power_on`](Self::power_on),
    /// [`power_off`](Self::power_off) and [`power_cycle`](Self::power_cycle), the
    /// only real reset the sensor has. The pin's level is not changed here.
    pub fn with_power_enable_pin_active_high<PwrNew>(self, pwr_pin: PwrNew) -> Vl53l7cx<BUS, D, LPN, INT, PwrNew> {
        let Vl53l7cx { comms, address, lpn_pin, int_pin, lpn_inverted, int_is_data_ready_high, stdev, .. } = self;
        Vl53l7cx {
            comms,
            address,
            lpn_pin,
            int_pin,
            pwr_pin,
            lpn_inverted,
            int_is_data_ready_high,
            pwr_active_high: true,
            stdev,
        }
    }

    /// Attach a power-enable line where **low** powers the sensor (a P-channel
    /// high-side switch or an LDO with an active-low enable). Otherwise as
    /// [`with_power_enable_pin_active_high`](Self::with_power_enable_pin_active_high).
    pub fn with_power_enable_pin_active_low<PwrNew>(self, pwr_pin: PwrNew) -> Vl53l7cx<BUS, D, LPN, INT, PwrNew> {
        let mut s = self.with_power_enable_pin_active_high(pwr_pin);
        s.pwr_active_high = false;
        s
    }
}

// --- LPn methods (requires OutputPin) ---

impl<BUS, D, LPN, INT, PWR, E> Vl53l7cx<BUS, D, LPN, INT, PWR>
where
    BUS: VlBus<Error = E>,
    LPN: OutputPin,
{
    /// Drive LPn to the logical level `high` (true = comms enabled), honouring
    /// the deprecated active-high builder's inversion.
    fn set_lpn_level(&mut self, high: bool) -> Result<(), Error<E>> {
        let result = if high != self.lpn_inverted {
            self.lpn_pin.set_high()
        } else {
            self.lpn_pin.set_low()
        };
        result.map_err(|_| Error::Pin)
    }

    /// Drive LPn high: the sensor's I2C interface listens ("Comms enable" in
    /// the datasheet). This is the resting state; `init()` needs it. Part of
    /// multi-sensor address assignment, see [`Vl53l7cx::set_i2c_address`].
    pub fn lpn_enable(&mut self) -> Result<(), Error<E>> {
        self.set_lpn_level(true)
    }

    /// Drive LPn low: the sensor ignores the I2C bus until
    /// [`lpn_enable`](Self::lpn_enable). Hold every other sensor low while
    /// moving one with [`Vl53l7cx::set_i2c_address`]. This does not reset the
    /// sensor, drop its firmware or change its power state.
    pub fn lpn_disable(&mut self) -> Result<(), Error<E>> {
        self.set_lpn_level(false)
    }
}

impl<BUS, D, LPN, INT, PWR, E> Vl53l7cx<BUS, D, LPN, INT, PWR>
where
    BUS: VlBus<Error = E>,
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
    pub fn reset(&mut self) -> Result<(), Error<E>> {
        self.set_lpn_level(false)?;
        self.comms.delay.delay_ms(10);
        self.set_lpn_level(true)
    }
}

// --- Power methods (requires a power-enable OutputPin) ---

impl<BUS, D, LPN, INT, PWR, E> Vl53l7cx<BUS, D, LPN, INT, PWR>
where
    BUS: VlBus<Error = E>,
    PWR: OutputPin,
{
    fn set_power(&mut self, on: bool) -> Result<(), Error<E>> {
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
    pub fn power_off(&mut self) -> Result<(), Error<E>> {
        self.set_power(false)
    }

    /// Enable the sensor's supply via the power-enable pin. Allow the rails to
    /// settle (10 ms is enough on the ST boards), make sure LPn is high and
    /// I2C_RST low, then `init()`.
    pub fn power_on(&mut self) -> Result<(), Error<E>> {
        self.set_power(true)
    }
}

impl<BUS, D, LPN, INT, PWR, E> Vl53l7cx<BUS, D, LPN, INT, PWR>
where
    BUS: VlBus<Error = E>,
    D: DelayNs,
    LPN: MaybeOutputPin,
    PWR: OutputPin,
{
    /// Drive LPn (if a GPIO is attached) to the logical level `high`, else no-op.
    fn set_lpn_level_or_skip(&mut self, high: bool) -> Result<(), Error<E>> {
        if high != self.lpn_inverted {
            self.lpn_pin.set_high_or_skip()
        } else {
            self.lpn_pin.set_low_or_skip()
        }
    }

    /// Power-cycle the sensor: the only real reset it has (ST's ULD has none).
    ///
    /// Implements the sensor reset procedure of UM2884 §4.2 (VL53L7CX) / UM3109
    /// (VL53L8CX): drive the supplies and LPn low, wait 10 ms, drive them high.
    /// Concretely: [`power_off`](Self::power_off); LPn low if a GPIO is attached
    /// (an LPn on its IOVDD pull-up follows the rail by itself); 10 ms;
    /// [`power_on`](Self::power_on), then LPn high; then 10 ms for the supplies
    /// to settle. Boards with slow supplies may need a longer settle: add your
    /// own delay before `init()`. Afterwards the firmware is gone and the sensor
    /// answers at [`DEFAULT_ADDRESS`], so follow with [`Vl53l7cx::init`] (and
    /// `set_i2c_address` again if you had moved it). Compiles with or without
    /// an LPn pin via [`MaybeOutputPin`].
    pub fn power_cycle(&mut self) -> Result<(), Error<E>> {
        self.set_power(false)?;
        self.set_lpn_level_or_skip(false)?;
        self.comms.delay.delay_ms(10);
        self.set_power(true)?;
        self.set_lpn_level_or_skip(true)?;
        self.comms.delay.delay_ms(10);
        Ok(())
    }
}

/// Pulse the VL53L7CX's **I2C_RST** input: high for 10 ms, then low.
///
/// I2C_RST (active high, internal pull-down) resets the sensor's I2C interface
/// only, when toggled 0 -> 1 -> 0; it does not reset the sensor, drop its
/// firmware or change its address. Use it to recover a hung I2C block; `init()`
/// expects the line low. The VL53L8CX has no I2C_RST: its SPI_I2C_N pin selects
/// the interface and toggling it likewise only clears the I2C block. A real
/// reset is [`Vl53l7cx::power_cycle`]. Fails with `Error::Pin` on a pin error.
pub fn pulse_i2c_rst<P: OutputPin, D: DelayNs>(
    pin: &mut P,
    delay: &mut D,
) -> Result<(), Error<core::convert::Infallible>> {
    pin.set_high().map_err(|_| Error::Pin)?;
    delay.delay_ms(10);
    pin.set_low().map_err(|_| Error::Pin)
}

impl<BUS, D, LPN, INT, PWR, E> Vl53l7cx<BUS, D, LPN, INT, PWR>
where
    BUS: VlBus<Error = E>,
    D: DelayNs,
{
    #[inline]
    fn bind_platform(&mut self) {
        self.stdev.platform.p_com = &mut self.comms as *mut _ as *mut core::ffi::c_void;
        self.stdev.platform.address = (self.address as u16) << 1;
    }

    /// Move the sensor to `new_address` (7-bit) and use it for all further
    /// traffic. With several sensors on one bus, hold every other sensor's LPn
    /// low while calling this so only the target sensor hears the command, then
    /// release them ([`Vl53l7cx::with_lpn_pin`], [`Vl53l7cx::lpn_disable`] /
    /// [`Vl53l7cx::lpn_enable`]). It is a register write, not a firmware
    /// command, so it works before [`Vl53l7cx::init`] as in ST's multi-sensor
    /// example, and needs no pin on this handle. The address is not retained:
    /// a power cycle returns the sensor to [`DEFAULT_ADDRESS`]. The value is
    /// not validated.
    pub fn set_i2c_address(&mut self, new_address: SevenBitAddress) -> Result<(), Error<E>> {
        self.bind_platform();
        let i2c_address_8bit = (new_address as u16) << 1;
        let status = unsafe {
            sys::vl53l7cx_set_i2c_address(&mut self.stdev, i2c_address_8bit)
        };
        if status != 0 {
            return Err(Error::Firmware(status.into()));
        }
        self.address = new_address;
        Ok(())
    }

    /// Start a ranging session with the current configuration. Settings cannot
    /// be changed while ranging: set resolution, frequency, mode, integration
    /// time, sharpener, target order, thresholds and motion first, then start,
    /// then loop on [`Vl53l7cx::check_data_ready`] (or the INT pin) and
    /// [`Vl53l7cx::get_ranging_data`], and finish with [`Vl53l7cx::stop_ranging`].
    pub fn start_ranging(&mut self) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::start_ranging(&mut self.stdev);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Stop the session started by [`Vl53l7cx::start_ranging`]. Required before
    /// changing settings or the power mode. Afterwards the sensor is
    /// idle in its current power mode, keeps its configuration and can be
    /// reconfigured and restarted.
    pub fn stop_ranging(&mut self) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::stop_ranging(&mut self.stdev);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Poll the sensor over the bus for a new frame: `Ok(true)` once per frame,
    /// when it is ready to be read with [`Vl53l7cx::get_ranging_data`]. Polling
    /// pattern without an INT pin: call this in a loop with a short delay (a few
    /// ms) until it returns `true`, read the frame, repeat. With INT wired
    /// use [`Vl53l7cx::wait_for_data_ready_blocking`] instead. A
    /// firmware error flagged in the status bytes is returned as
    /// `Error::Firmware`.
    pub fn check_data_ready(&mut self) -> Result<bool, Error<E>> {
        self.bind_platform();
        let (status, is_ready) = sys::wrappers::check_data_ready(&mut self.stdev);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(is_ready != 0),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Set the zone grid: [`Resolution::Res4x4`] (16 zones; default after
    /// [`Vl53l7cx::init`]) or [`Resolution::Res8x8`] (64 zones, four times the bus
    /// traffic per frame). The allowed ranging frequency depends on it, so
    /// call this **before** [`Vl53l7cx::set_ranging_frequency_hz`]; with the
    /// motion indicator, follow with [`Vl53l7cx::motion_indicator_set_resolution`].
    /// The ULD also re-sends the resolution-dependent offset and crosstalk
    /// calibration data. Not while ranging.
    pub fn set_resolution(&mut self, resolution: Resolution) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_resolution(&mut self.stdev, resolution as u8);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Set the ranging frequency in Hz (frames per second): 1..=60 Hz in 4x4,
    /// 1..=15 Hz in 8x8; default 1 Hz after [`Vl53l7cx::init`]. The limits depend on
    /// the resolution, so call [`Vl53l7cx::set_resolution`] first. Neither this
    /// driver nor ST's ULD checks the value: the firmware enforces the limits.
    /// In [`RangingMode::Autonomous`] the frame period must also exceed the
    /// integration time(s) plus 1 ms, see [`Vl53l7cx::set_integration_time_ms`].
    /// Not while ranging.
    pub fn set_ranging_frequency_hz(&mut self, frequency_hz: u8) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_ranging_frequency_hz(&mut self.stdev, frequency_hz);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Set the ranging mode. [`RangingMode::Autonomous`] (default after
    /// [`Vl53l7cx::init`]) fires the VCSEL only for the integration time of each
    /// frame: lower power, the more so at low frequency; the first frame's
    /// period may be shorter than configured. [`RangingMode::Continuous`] keeps
    /// the VCSEL on throughout: best range and ambient immunity, for fast or
    /// high-performance ranging; the integration time is ignored. Not while
    /// ranging.
    pub fn set_ranging_mode(&mut self, mode: RangingMode) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_ranging_mode(&mut self.stdev, mode as u8);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Set the integration time in ms, 2..=1000; anything else the ULD returns
    /// `Error::Firmware(FirmwareError::InvalidParam)` without touching the bus.
    /// Default 5 ms after [`Vl53l7cx::init`]. Only meaningful in
    /// [`RangingMode::Autonomous`] (no effect in continuous mode). 4x4
    /// integrates once per frame, 8x8 four times; the sum of the integrations
    /// plus 1 ms of overhead must fit in the frame period (1 / frequency),
    /// otherwise the sensor lengthens the period by itself. Not while ranging.
    pub fn set_integration_time_ms(&mut self, time_ms: u32) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_integration_time_ms(&mut self.stdev, time_ms);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Set the sharpener in percent, 0..=99; 100 or more the ULD returns
    /// `Error::Firmware(FirmwareError::InvalidParam)` without touching the bus.
    /// Default 14 % after [`Vl53l7cx::init`]. The sharpener removes veiling-glare
    /// signal spilling from a close target into neighbouring zones; a higher
    /// value trims more of that spread, 0 disables it. Stored in 1/255 steps, so
    /// the readback can be 1 % below the value set. Not while ranging.
    pub fn set_sharpener_percent(&mut self, percent: u8) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_sharpener_percent(&mut self.stdev, percent);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Choose which detected target a zone reports first: [`TargetOrder::Closest`]
    /// or [`TargetOrder::Strongest`] (default after [`Vl53l7cx::init`]). With the
    /// default single target per zone this selects the one target reported;
    /// with a `targets-N` feature it orders the list ([`NB_TARGET_PER_ZONE`]).
    /// Two targets in one zone are only told apart when at least 600 mm apart.
    /// Not while ranging.
    pub fn set_target_order(&mut self, order: TargetOrder) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_target_order(&mut self.stdev, order as u8);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Set the power mode.
    ///
    /// The VL53L7CX ULD only knows [`PowerMode::Sleep`] and
    /// [`PowerMode::Wakeup`] (`vl53l7cx_api.h` has no
    /// `VL53L7CX_POWER_MODE_DEEP_SLEEP`); passing [`PowerMode::DeepSleep`]
    /// returns `Error::Firmware(FirmwareError::InvalidParam)` without touching
    /// the bus, matching what the L8 ULD itself returns for an unknown mode.
    ///
    /// Stop ranging first ([`Vl53l7cx::stop_ranging`]). Default after
    /// [`Vl53l7cx::init`] is [`PowerMode::Wakeup`] (HP idle, ready to range);
    /// [`PowerMode::Sleep`] (LP idle) keeps firmware and configuration but
    /// cannot range until woken.
    pub fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), Error<E>> {
        if mode == PowerMode::DeepSleep {
            return Err(Error::Firmware(sys::VL53L7CX_STATUS_INVALID_PARAM.into()));
        }
        self.bind_platform();
        let status = sys::wrappers::set_power_mode(&mut self.stdev, mode as u8);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Read the current zone grid. Default after [`Vl53l7cx::init`] is
    /// [`Resolution::Res4x4`].
    pub fn get_resolution(&mut self) -> Result<Resolution, Error<E>> {
        self.bind_platform();
        let mut resolution: u8 = 0;
        let status = unsafe { sys::vl53l7cx_get_resolution(&mut self.stdev, &mut resolution) };
        match status {
            sys::VL53L7CX_STATUS_OK => {
                if resolution == sys::VL53L7CX_RESOLUTION_8X8 {
                     Ok(Resolution::Res8x8)
                } else {
                     Ok(Resolution::Res4x4)
                }
            },
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Read the ranging frequency in Hz. Default after [`Vl53l7cx::init`] is 1 Hz.
    pub fn get_ranging_frequency_hz(&mut self) -> Result<u8, Error<E>> {
        self.bind_platform();
        let mut freq: u8 = 0;
        let status = unsafe { sys::vl53l7cx_get_ranging_frequency_hz(&mut self.stdev, &mut freq) };
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(freq),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Read the integration time in ms. Default after [`Vl53l7cx::init`] is 5 ms.
    pub fn get_integration_time_ms(&mut self) -> Result<u32, Error<E>> {
        self.bind_platform();
        let (status, time) = sys::wrappers::get_integration_time_ms(&mut self.stdev);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(time),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Read the ranging mode. Default after [`Vl53l7cx::init`] is
    /// [`RangingMode::Autonomous`].
    pub fn get_ranging_mode(&mut self) -> Result<RangingMode, Error<E>> {
        self.bind_platform();
        let (status, mode) = sys::wrappers::get_ranging_mode(&mut self.stdev);
        match status {
            sys::VL53L7CX_STATUS_OK => {
                if mode == sys::VL53L7CX_RANGING_MODE_CONTINUOUS {
                    Ok(RangingMode::Continuous)
                } else {
                    Ok(RangingMode::Autonomous)
                }
            },
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Read the sharpener in percent. Default after [`Vl53l7cx::init`] is 14 %; the
    /// value is stored in 1/255 steps, so it can read 1 % below what was set.
    pub fn get_sharpener_percent(&mut self) -> Result<u8, Error<E>> {
        self.bind_platform();
        let (status, percent) = sys::wrappers::get_sharpener_percent(&mut self.stdev);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(percent),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Read the target order. Default after [`Vl53l7cx::init`] is
    /// [`TargetOrder::Strongest`].
    pub fn get_target_order(&mut self) -> Result<TargetOrder, Error<E>> {
        self.bind_platform();
        let (status, order) = sys::wrappers::get_target_order(&mut self.stdev);
        match status {
            sys::VL53L7CX_STATUS_OK => {
                if order == sys::VL53L7CX_TARGET_ORDER_STRONGEST {
                    Ok(TargetOrder::Strongest)
                } else {
                    Ok(TargetOrder::Closest)
                }
            },
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Read the power mode. Default after [`Vl53l7cx::init`] is
    /// [`PowerMode::Wakeup`]. Never [`PowerMode::DeepSleep`] on this part.
    pub fn get_power_mode(&mut self) -> Result<PowerMode, Error<E>> {
        self.bind_platform();
        let (status, mode) = sys::wrappers::get_power_mode(&mut self.stdev);
        match status {
            sys::VL53L7CX_STATUS_OK => {
                // No deep-sleep mode on the VL53L7CX.
                if mode == sys::VL53L7CX_POWER_MODE_WAKEUP {
                    Ok(PowerMode::Wakeup)
                } else {
                    Ok(PowerMode::Sleep)
                }
            },
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Read the temperature (VHV) recalibration period in frames; 0 (the
    /// default after [`Vl53l7cx::init`]) means disabled. See
    /// [`Vl53l7cx::set_vhv_repeat_count`].
    pub fn get_vhv_repeat_count(&mut self) -> Result<u32, Error<E>> {
        self.bind_platform();
        let (status, count) = sys::wrappers::get_vhv_repeat_count(&mut self.stdev);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(count),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Set the period of the firmware's temperature (VHV) recalibration in
    /// frames: `count` > 0 recalibrates every `count` frames, each run taking a
    /// few ms; 0 (default after [`Vl53l7cx::init`]) disables it, so compensation is
    /// calibrated once at the start of each ranging session only. The value is
    /// not validated. Not while ranging.
    pub fn set_vhv_repeat_count(&mut self, count: u32) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_vhv_repeat_count(&mut self.stdev, count);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Enable the sensor's internal charge pump (VL53L7CX only; the VL53L8CX
    /// ULD has no equivalent). ST advise against calling this unless
    /// instructed to by ST.
    pub fn enable_internal_cp(&mut self) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::enable_internal_cp(&mut self.stdev);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Disable the sensor's internal charge pump (VL53L7CX only). See
    /// [`enable_internal_cp`](Self::enable_internal_cp).
    pub fn disable_internal_cp(&mut self) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::disable_internal_cp(&mut self.stdev);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Initialise the sensor: check that it answers ([`Vl53l7cx::is_alive`], else
    /// `Error::NotAlive`), then run ST's `vl53l7cx_init`, which reboots the device,
    /// uploads the ~84 KB firmware and loads the default configuration; it
    /// takes a few hundred ms. Call after power-on with AVDD, IOVDD and LPn
    /// high and I2C_RST low; it also re-initialises a running sensor. Leaves 4x4, 1 Hz,
    /// [`RangingMode::Autonomous`], 5 ms integration, sharpener 14 %,
    /// [`TargetOrder::Strongest`], [`PowerMode::Wakeup`] and detection
    /// thresholds disabled.
    pub fn init(&mut self) -> Result<(), Error<E>> {
        self.stdev.platform.address = (self.address as u16) << 1;
        self.bind_platform();

        if !self.is_alive()? {
            return Err(Error::NotAlive);
        }

        self.bind_platform();
        let status = unsafe { sys::vl53l7cx_init(&mut self.stdev) };
        if status != 0 {
            return Err(Error::Firmware(status.into()));
        }
        Ok(())
    }

    /// Check that a VL53L7CX answers at the current address (device id 0xF0,
    /// revision 0x02, read via the ULD's `is_alive`). Needs no firmware, so it
    /// works before [`Vl53l7cx::init`]; `Ok(false)` if the ids do not match. A bus
    /// failure surfaces through the ULD status as `Error::Firmware`.
    pub fn is_alive(&mut self) -> Result<bool, Error<E>> {
        self.bind_platform();
        let (status, is_alive) = sys::wrappers::is_alive(&mut self.stdev);
        match status { sys::VL53L7CX_STATUS_OK => Ok(is_alive != 0), s => Err(Error::Firmware(s.into())), }
    }

    /// Read the latest ranging frame. See [`ResultsData`] for the array layout
    /// (per-target arrays are indexed `zone * NB_TARGET_PER_ZONE + target`).
    pub fn get_ranging_data(&mut self) -> Result<ResultsData, Error<E>> {
        self.bind_platform();
        let (status, value) = sys::wrappers::get_ranging_data(&mut self.stdev);
        match status { sys::VL53L7CX_STATUS_OK => Ok(value), s => Err(Error::Firmware(s.into())), }
    }

}

/// Bus-trace hooks called from the `bind_platform_driver*!` macros.
///
/// They live here rather than in the macro bodies so that the `defmt` feature
/// of *this* crate decides whether tracing is compiled in. A `cfg(feature)`
/// inside an exported macro is evaluated in the crate that invokes it, which
/// would make the user depend on `defmt`, declare a `defmt` feature of their
/// own, or get an `unexpected_cfgs` warning. Without the feature every hook is
/// an empty inline function.
#[doc(hidden)]
pub mod __trace {
    #[cfg(feature = "defmt")]
    fn data(data: &[u8]) {
        for chunk in data.chunks(16) {
            defmt::trace!("  {=[u8]:#04x}", chunk);
        }
    }

    #[inline(always)]
    pub fn i2c_read_ok(address: u8, index: u16, size: u32, data_slice: &[u8]) {
        #[cfg(feature = "defmt")]
        {
            defmt::trace!("i2c R [{:#04x}] reg={:#06x} len={}", address, index, size);
            data(data_slice);
        }
        let _ = (address, index, size, data_slice);
    }

    #[inline(always)]
    pub fn i2c_read_err(address: u8, index: u16) {
        #[cfg(feature = "defmt")]
        defmt::trace!("i2c R [{:#04x}] reg={:#06x} -> ERR", address, index);
        let _ = (address, index);
    }

    #[inline(always)]
    pub fn i2c_write(address: u8, index: u16, size: u32, data_slice: &[u8]) {
        #[cfg(feature = "defmt")]
        {
            defmt::trace!("i2c W [{:#04x}] reg={:#06x} len={}", address, index, size);
            data(data_slice);
        }
        let _ = (address, index, size, data_slice);
    }

    #[inline(always)]
    pub fn i2c_write_err() {
        #[cfg(feature = "defmt")]
        defmt::trace!("i2c W -> ERR");
    }

    #[inline(always)]
    pub fn spi_read_ok(index: u16, size: u32, data_slice: &[u8]) {
        #[cfg(feature = "defmt")]
        {
            defmt::trace!("spi R reg={:#06x} len={}", index, size);
            data(data_slice);
        }
        let _ = (index, size, data_slice);
    }

    #[inline(always)]
    pub fn spi_read_err(index: u16) {
        #[cfg(feature = "defmt")]
        defmt::trace!("spi R reg={:#06x} -> ERR", index);
        let _ = index;
    }

    #[inline(always)]
    pub fn spi_write(index: u16, size: u32, data_slice: &[u8]) {
        #[cfg(feature = "defmt")]
        {
            defmt::trace!("spi W reg={:#06x} len={}", index, size);
            data(data_slice);
        }
        let _ = (index, size, data_slice);
    }

    #[inline(always)]
    pub fn spi_write_err() {
        #[cfg(feature = "defmt")]
        defmt::trace!("spi W -> ERR");
    }
}

// --- Macro for Dynamic Platform Context ---
#[macro_export]
macro_rules! bind_platform_driver {
    ($i2c_type:ty, $delay_type:ty) => {
        #[no_mangle]
        pub extern "C" fn vl53l7cx_platform_read(
            p_platform: *mut $crate::VL53L7CX_Platform,
            index: u16,
            p_values: *mut u8,
            size: u32,
        ) -> u8 {
            if p_platform.is_null() { return 255; }
            let platform = unsafe { &*p_platform };
            let comms = unsafe { &mut *(platform.p_com as *mut $crate::Vl53l7cxComms<$i2c_type, $delay_type>) };

            let data_slice = unsafe { core::slice::from_raw_parts_mut(p_values, size as usize) };
            let index_bytes = index.to_be_bytes();
            let address = (platform.address >> 1) as u8;

            match embedded_hal::i2c::I2c::write_read(&mut comms.bus, address, &index_bytes, data_slice) {
                Ok(_) => {
                    $crate::__trace::i2c_read_ok(address, index, size, data_slice);
                    0
                }
                Err(_) => {
                    $crate::__trace::i2c_read_err(address, index);
                    1
                }
            }
        }

        #[no_mangle]
        pub extern "C" fn vl53l7cx_platform_write(
            p_platform: *mut $crate::VL53L7CX_Platform,
            index: u16,
            p_values: *mut u8,
            size: u32,
        ) -> u8 {
            if p_platform.is_null() { return 255; }
            let platform = unsafe { &*p_platform };
            let comms = unsafe { &mut *(platform.p_com as *mut $crate::Vl53l7cxComms<$i2c_type, $delay_type>) };

            let data_slice = unsafe { core::slice::from_raw_parts(p_values, size as usize) };
            let index_bytes = index.to_be_bytes();
            let address = (platform.address >> 1) as u8;

            $crate::__trace::i2c_write(address, index, size, data_slice);

            let mut operations = [
                embedded_hal::i2c::Operation::Write(&index_bytes),
                embedded_hal::i2c::Operation::Write(data_slice),
            ];

            match embedded_hal::i2c::I2c::transaction(&mut comms.bus, address, &mut operations) {
                Ok(_) => 0,
                Err(_) => {
                    $crate::__trace::i2c_write_err();
                    1
                }
            }
        }

        #[no_mangle]
        pub extern "C" fn vl53l7cx_platform_wait_ms(p_platform: *mut $crate::VL53L7CX_Platform, milliseconds: u32) -> u8 {
            if p_platform.is_null() { return 255; }
            let platform = unsafe { &*p_platform };
            let comms = unsafe { &mut *(platform.p_com as *mut $crate::Vl53l7cxComms<$i2c_type, $delay_type>) };

            embedded_hal::delay::DelayNs::delay_ms(&mut comms.delay, milliseconds);
            0
        }

        #[no_mangle]
        pub extern "C" fn vl53l7cx_platform_init(_p_platform: *mut $crate::VL53L7CX_Platform) -> u8 { 0 }

        #[no_mangle]
        pub extern "C" fn vl53l7cx_platform_terminate(_p_platform: *mut $crate::VL53L7CX_Platform) -> u8 { 0 }
    };
}

#[macro_export]
macro_rules! bind_platform_driver_spi {
    ($spi_type:ty, $delay_type:ty) => {
        #[no_mangle]
        pub extern "C" fn vl53l7cx_platform_read(
            p_platform: *mut $crate::VL53L7CX_Platform,
            index: u16,
            p_values: *mut u8,
            size: u32,
        ) -> u8 {
            if p_platform.is_null() { return 255; }
            let platform = unsafe { &*p_platform };
            let comms = unsafe { &mut *(platform.p_com as *mut $crate::Vl53l7cxComms<$crate::Vl53l7cxSpiDevice<$spi_type>, $delay_type>) };

            let data_slice = unsafe { core::slice::from_raw_parts_mut(p_values, size as usize) };
            let addr_bytes = (index & 0x7FFF).to_be_bytes();

            match embedded_hal::spi::SpiDevice::transaction(&mut comms.bus.spi, &mut [
                embedded_hal::spi::Operation::Write(&addr_bytes),
                embedded_hal::spi::Operation::Read(data_slice),
            ]) {
                Ok(_) => {
                    $crate::__trace::spi_read_ok(index, size, data_slice);
                    0
                }
                Err(_) => {
                    $crate::__trace::spi_read_err(index);
                    1
                }
            }
        }

        #[no_mangle]
        pub extern "C" fn vl53l7cx_platform_write(
            p_platform: *mut $crate::VL53L7CX_Platform,
            index: u16,
            p_values: *mut u8,
            size: u32,
        ) -> u8 {
            if p_platform.is_null() { return 255; }
            let platform = unsafe { &*p_platform };
            let comms = unsafe { &mut *(platform.p_com as *mut $crate::Vl53l7cxComms<$crate::Vl53l7cxSpiDevice<$spi_type>, $delay_type>) };

            let data_slice = unsafe { core::slice::from_raw_parts(p_values, size as usize) };
            let addr_bytes = (index | 0x8000).to_be_bytes();

            $crate::__trace::spi_write(index, size, data_slice);
            match embedded_hal::spi::SpiDevice::transaction(&mut comms.bus.spi, &mut [
                embedded_hal::spi::Operation::Write(&addr_bytes),
                embedded_hal::spi::Operation::Write(data_slice),
            ]) {
                Ok(_) => 0,
                Err(_) => {
                    $crate::__trace::spi_write_err();
                    1
                }
            }
        }

        #[no_mangle]
        pub extern "C" fn vl53l7cx_platform_wait_ms(p_platform: *mut $crate::VL53L7CX_Platform, milliseconds: u32) -> u8 {
            if p_platform.is_null() { return 255; }
            let platform = unsafe { &*p_platform };
            let comms = unsafe { &mut *(platform.p_com as *mut $crate::Vl53l7cxComms<$crate::Vl53l7cxSpiDevice<$spi_type>, $delay_type>) };

            embedded_hal::delay::DelayNs::delay_ms(&mut comms.delay, milliseconds);
            0
        }

        #[no_mangle]
        pub extern "C" fn vl53l7cx_platform_init(_p_platform: *mut $crate::VL53L7CX_Platform) -> u8 { 0 }

        #[no_mangle]
        pub extern "C" fn vl53l7cx_platform_terminate(_p_platform: *mut $crate::VL53L7CX_Platform) -> u8 { 0 }
    };
}

#[cfg(any(feature = "motion", feature = "thresholds"))]
impl<BUS, D, LPN, INT, PWR, E> Vl53l7cx<BUS, D, LPN, INT, PWR>
where
    BUS: VlBus<Error = E>,
    D: DelayNs,
{
    /// Initialise the motion indicator: fill `motion_config` with ST's defaults
    /// (movement window 400..=1500 mm) for `resolution` and send it to the
    /// sensor. `resolution` must match the sensor's; after every later
    /// [`Vl53l7cx::set_resolution`] call [`Vl53l7cx::motion_indicator_set_resolution`].
    /// Adjust the window with [`Vl53l7cx::motion_indicator_set_distance_motion`].
    /// The per-aggregate `motion` results grow with movement (a typical moving
    /// target gives 100..=500); call before [`Vl53l7cx::start_ranging`].
    pub fn motion_indicator_init(&mut self, motion_config: &mut sys::VL53L7CX_Motion_Configuration, resolution: Resolution) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::motion_indicator_init(&mut self.stdev, motion_config, resolution as u8);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Set the distance window the motion indicator watches, in mm, and send the
    /// updated `motion_config`. Requires `min <= max`, `min >= 400`,
    /// `max <= 4000` and `max - min <= 1500`; otherwise returns
    /// `Error::Firmware(FirmwareError::InvalidParam)` without touching the bus.
    /// ST's ULD checks the last three itself but lets `max < min` through and
    /// computes garbage; this wrapper rejects it before calling the ULD.
    /// Default window 400..=1500 mm. Sensitivity also depends on integration
    /// time, distance and target reflectance. Not while ranging.
    pub fn motion_indicator_set_distance_motion(&mut self, motion_config: &mut sys::VL53L7CX_Motion_Configuration, min_distance_mm: u16, max_distance_mm: u16) -> Result<(), Error<E>> {
        if min_distance_mm > max_distance_mm
            || min_distance_mm < 400
            || max_distance_mm > 4000
            || (max_distance_mm - min_distance_mm) > 1500
        {
            return Err(Error::Firmware(FirmwareError::InvalidParam));
        }
        self.bind_platform();
        let status = sys::wrappers::motion_indicator_set_distance_motion(&mut self.stdev, motion_config, min_distance_mm, max_distance_mm);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Rebuild the motion indicator's zone map for `resolution` and send the
    /// updated `motion_config`. Call after every [`Vl53l7cx::set_resolution`]: the
    /// motion plugin does not follow the sensor's resolution by itself. The 4x4
    /// map uses one aggregate per zone, the 8x8 map groups the 64 zones into 16
    /// aggregates of 2x2. Not while ranging.
    pub fn motion_indicator_set_resolution(
        &mut self,
        motion_config: &mut sys::VL53L7CX_Motion_Configuration,
        resolution: Resolution,
    ) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::motion_indicator_set_resolution(&mut self.stdev, motion_config, resolution as u8);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Read whether the detection thresholds are enabled. Default after
    /// [`Vl53l7cx::init`] is off. See [`Vl53l7cx::set_detection_thresholds_enable`].
    pub fn get_detection_thresholds_enable(&mut self) -> Result<bool, Error<E>> {
        self.bind_platform();
        let (status, enabled) = sys::wrappers::get_detection_thresholds_enable(&mut self.stdev);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(enabled == 1),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Enable or disable the detection thresholds programmed with
    /// [`Vl53l7cx::set_detection_thresholds`] (default off after [`Vl53l7cx::init`]).
    /// When enabled, INT fires only for a frame in which a zone meets its
    /// threshold, instead of for every frame; combined with
    /// [`RangingMode::Autonomous`] (and a threshold on the motion indicator)
    /// this is the lowest-power presence detection. Program the thresholds
    /// first, then enable, then [`Vl53l7cx::start_ranging`].
    pub fn set_detection_thresholds_enable(&mut self, enabled: bool) -> Result<(), Error<E>> {
        self.bind_platform();
        let val = if enabled { 1 } else { 0 };
        let status = sys::wrappers::set_detection_thresholds_enable(&mut self.stdev, val);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }
}

#[cfg(feature = "xtalk")]
impl<BUS, D, LPN, INT, PWR, E> Vl53l7cx<BUS, D, LPN, INT, PWR>
where
    BUS: VlBus<Error = E>,
    D: DelayNs,
{
    /// Run the ULD's crosstalk calibration (`vl53l7cx_calibrate_xtalk`) against a flat, uniform
    /// target filling the field of view, in the dark, and keep the resulting
    /// shape. `reflectance_percent` 1..=99 (ST recommends 3 %), `nb_samples`
    /// 1..=16 (more is slower but more accurate), `distance_mm` 600..=3000
    /// (600 mm is easiest). Out-of-range arguments return
    /// `Error::Firmware(FirmwareError::InvalidParam)`; too little crosstalk
    /// (no cover glass) returns `FirmwareError::XtalkFailed`. Takes a few
    /// seconds and restores the previous configuration. Save the shape with
    /// [`Vl53l7cx::get_caldata_xtalk`]. Not while ranging.
    pub fn calibrate_xtalk(&mut self, reflectance_percent: u16, nb_samples: u8, distance_mm: u16) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::calibrate_xtalk(&mut self.stdev, reflectance_percent, nb_samples, distance_mm);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Read the crosstalk margin in kcps/SPAD. ST documents the default as
    /// 50 kcps/SPAD. See [`Vl53l7cx::set_xtalk_margin`].
    pub fn get_xtalk_margin(&mut self) -> Result<u32, Error<E>> {
        self.bind_platform();
        let (status, margin) = sys::wrappers::get_xtalk_margin(&mut self.stdev);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(margin),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Set the crosstalk margin in kcps/SPAD, 0..=10000; above that the ULD returns
    /// `Error::Firmware(FirmwareError::InvalidParam)` without touching the bus.
    /// ST documents the default as 50 kcps/SPAD. The margin raises the
    /// crosstalk detection threshold so a cover glass is never reported as a
    /// target once a calibration ([`Vl53l7cx::calibrate_xtalk`] /
    /// [`Vl53l7cx::set_caldata_xtalk`]) is loaded, and lets one unit's calibration
    /// be reused on others and tuned per unit. Not while ranging.
    pub fn set_xtalk_margin(&mut self, margin: u32) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_xtalk_margin(&mut self.stdev, margin);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Read the crosstalk shape the sensor is currently using into `buffer`,
    /// which must hold at least [`XTALK_BUFFER_SIZE`] (776) bytes, else
    /// `Error::Firmware(FirmwareError::InvalidParam)`. This is the ULD
    /// default, the shape produced by [`Vl53l7cx::calibrate_xtalk`], or whatever
    /// [`Vl53l7cx::set_caldata_xtalk`] loaded; store it and replay it with
    /// `set_caldata_xtalk` after the next power-up. Not while ranging.
    pub fn get_caldata_xtalk(&mut self, buffer: &mut [u8]) -> Result<(), Error<E>> {
        // Use local constant for buffer size
        if buffer.len() < XTALK_BUFFER_SIZE {
             return Err(Error::Firmware(sys::VL53L7CX_STATUS_INVALID_PARAM.into()));
        }
        self.bind_platform();
        let status = sys::wrappers::get_caldata_xtalk(&mut self.stdev, buffer.as_mut_ptr());
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Load a crosstalk shape saved with [`Vl53l7cx::get_caldata_xtalk`] (e.g. from
    /// flash) so a calibrated cover glass works without recalibrating after
    /// each power-up. `buffer` must hold at least [`XTALK_BUFFER_SIZE`] (776)
    /// bytes, else `Error::Firmware(FirmwareError::InvalidParam)`; only the
    /// first 776 are used. Call after [`Vl53l7cx::init`] and before ranging; set
    /// [`Vl53l7cx::set_xtalk_margin`] if the shape came from another unit.
    pub fn set_caldata_xtalk(&mut self, buffer: &mut [u8]) -> Result<(), Error<E>> {
         // Use local constant for buffer size
         if buffer.len() < XTALK_BUFFER_SIZE {
             return Err(Error::Firmware(sys::VL53L7CX_STATUS_INVALID_PARAM.into()));
        }
        self.bind_platform();
        let status = sys::wrappers::set_caldata_xtalk(&mut self.stdev, buffer.as_mut_ptr());
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }
}

#[cfg(feature = "thresholds")]
impl<BUS, D, LPN, INT, PWR, E> Vl53l7cx<BUS, D, LPN, INT, PWR>
where
    BUS: VlBus<Error = E>,
    D: DelayNs,
{
    /// Read the 64 detection-threshold entries.
    ///
    /// Values are in the units of each entry's `measurement` (mm, kcps/SPAD, ...).
    /// ST's ULD converts to and from the firmware's fixed-point representation
    /// itself (x4 for distance, x2048 for per-SPAD rates, x128 for sigma, x256
    /// for SPAD count, x65535 for the motion indicator), so this wrapper passes
    /// the values through unchanged. An earlier version scaled them a second
    /// time, which the symmetric set/get round trip hid but which put x16 (for
    /// mm) on the wire; the differential tests against `vl53-8x8` caught it.
    pub fn get_detection_thresholds(&mut self) -> Result<[DetectionThreshold; VL53L7CX_NB_THRESHOLDS as usize], Error<E>> {
        self.bind_platform();
        let mut raw_thresholds: [sys::VL53L7CX_DetectionThresholds; VL53L7CX_NB_THRESHOLDS as usize] = unsafe { core::mem::zeroed() };
        let ptr = raw_thresholds.as_mut_ptr() as *mut sys::VL53L7CX_DetectionThresholds;

        let status = sys::wrappers::get_detection_thresholds(&mut self.stdev, ptr);
        if status != sys::VL53L7CX_STATUS_OK {
            return Err(Error::Firmware(status.into()));
        }

        let rust_thresholds: [DetectionThreshold; VL53L7CX_NB_THRESHOLDS as usize] = unsafe {
            core::mem::transmute(raw_thresholds)
        };


        Ok(rust_thresholds)
    }

    /// Program the 64 detection-threshold entries (`VL53L8CX_NB_THRESHOLDS`).
    /// Give `low_threshold` / `high_threshold` in each entry's natural units
    /// (mm, kcps/SPAD, ...); ST's ULD applies the firmware's fixed-point scaling.
    /// Three layouts: 4x4 with one entry per zone (16 used), 4x4 with two per
    /// zone (32; `mathematic_operation` combines the pair), or 8x8 with one per
    /// zone (64). ST's header requires the caller to OR
    /// [`VL53L8CX_LAST_THRESHOLD`] (128) into the `zone_num` of the last entry
    /// used (`zone_num: 15 | VL53L8CX_LAST_THRESHOLD`) and to leave unused
    /// entries zero; the array is handed to ST's ULD exactly as given, so a
    /// missing flag is not detected and the readback returns it as written.
    /// Then enable with [`Vl53l7cx::set_detection_thresholds_enable`] before
    /// ranging.
    pub fn set_detection_thresholds(&mut self, thresholds: [DetectionThreshold; VL53L7CX_NB_THRESHOLDS as usize]) -> Result<(), Error<E>> {
        self.bind_platform();

        let raw_thresholds: [sys::VL53L7CX_DetectionThresholds; VL53L7CX_NB_THRESHOLDS as usize] = unsafe {
            core::mem::transmute(thresholds)
        };
        let ptr = raw_thresholds.as_ptr() as *mut sys::VL53L7CX_DetectionThresholds;

        let status = sys::wrappers::set_detection_thresholds(&mut self.stdev, ptr);
        match status {
            sys::VL53L7CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    // No `get/set_detection_thresholds_auto_stop`: the VL53L7CX ULD's
    // detection-thresholds plugin does not provide them.
}

/// Host-side smoke tests: a recording mock I2C bus bound through
/// [`bind_platform_driver!`], driving ST's real L5 C code.
#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    /// Mock I2C bus. Records every (register, bytes) write and answers reads
    /// of register 0x0000/0x0001 with the VL53L7CX device/revision id.
    pub struct MockI2c {
        pub writes: Vec<(u16, Vec<u8>)>,
        pub reads: Vec<(u16, usize)>,
    }

    impl embedded_hal::i2c::ErrorType for MockI2c {
        type Error = core::convert::Infallible;
    }

    impl I2c for MockI2c {
        fn transaction(
            &mut self,
            _address: u8,
            operations: &mut [embedded_hal::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            use embedded_hal::i2c::Operation;
            let mut reg: Option<u16> = None;
            for op in operations.iter_mut() {
                match op {
                    Operation::Write(bytes) => match reg {
                        None => reg = Some(u16::from_be_bytes([bytes[0], bytes[1]])),
                        Some(r) => self.writes.push((r, bytes.to_vec())),
                    },
                    Operation::Read(buf) => {
                        let r = reg.expect("read without register");
                        self.reads.push((r, buf.len()));
                        for (i, b) in buf.iter_mut().enumerate() {
                            *b = match r as usize + i {
                                0 => 0xF0, // device id
                                1 => 0x02, // VL53L7CX revision id (L8 is 0x0C)
                                _ => 0x00,
                            };
                        }
                    }
                }
            }
            Ok(())
        }
    }

    pub struct MockDelay;
    impl DelayNs for MockDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    crate::bind_platform_driver!(MockI2c, MockDelay);

    fn sensor() -> Vl53l7cx<MockI2c, MockDelay> {
        Vl53l7cx::new(MockI2c { writes: Vec::new(), reads: Vec::new() }, MockDelay)
    }

    #[test]
    fn is_alive_through_c_uld_and_platform_macro() {
        let mut s = sensor();
        assert_eq!(s.is_alive(), Ok(true));
        // vl53l7cx_is_alive: page select 0x7FFF=0, read 0x0000 and 0x0001,
        // page select 0x7FFF=2.
        assert_eq!(
            s.comms.bus.writes,
            std::vec![(0x7FFF, std::vec![0x00]), (0x7FFF, std::vec![0x02])]
        );
        assert_eq!(s.comms.bus.reads, std::vec![(0x0000, 1), (0x0001, 1)]);
    }

    #[test]
    fn deep_sleep_is_rejected_without_bus_traffic() {
        let mut s = sensor();
        assert_eq!(
            s.set_power_mode(PowerMode::DeepSleep),
            Err(Error::Firmware(FirmwareError::InvalidParam))
        );
        assert!(s.comms.bus.writes.is_empty());
        assert!(s.comms.bus.reads.is_empty());
    }
}
