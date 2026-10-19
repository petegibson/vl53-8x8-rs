#![no_std]
/*!
`vl53l8cx-uld`: a safe, blocking, platform-agnostic wrapper over the official
STMicroelectronics VL53L8CX ULD (Ultra Lite Driver).

The C ULD is compiled and bound by the `vl53l8cx-uld-sys` crate (bindgen + cc,
so libclang is needed at build time); this crate exposes it through
`embedded-hal` 1.0 I2C/SPI and delay traits. The ULD calls back into Rust for
bus access, so a program must invoke [`bind_platform_driver!`] or
[`bind_platform_driver_spi!`] exactly once with its concrete bus and delay
types.

For a pure-Rust driver covering both the VL53L5CX and VL53L8CX with no C
toolchain requirement, see the `vl53-8x8` crate. Shared types come from
`vl53-8x8-types` and are re-exported here.
*/

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::SpiDevice;

// We import the sys crate internally but do NOT make it public.
use vl53l8cx_uld_sys as sys;

// --- Public Type Re-exports from sys ---
pub use sys::VL53L8CX_Platform;
/// One frame of ranging results, as filled in by [`Vl53l8cx::get_ranging_data`].
///
/// This is ST's `VL53L8CX_ResultsData`; its fields depend on the `disable-*`
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
/// use vl53l8cx_uld::NB_TARGET_PER_ZONE;
/// let frame = sensor.get_ranging_data()?;
/// for zone in 0..16 {
///     for target in 0..usize::from(frame.nb_target_detected[zone]) {
///         let i = zone * NB_TARGET_PER_ZONE + target;
///         let (mm, status) = (frame.distance_mm[i], frame.target_status[i]);
///     }
/// }
/// ```
pub use sys::VL53L8CX_ResultsData as ResultsData;
/// Number of targets reported per zone, selected by the `targets-N` features
/// (default 1). Per-target arrays in [`ResultsData`] are indexed
/// `zone * NB_TARGET_PER_ZONE + target`.
pub use sys::VL53L8CX_NB_TARGET_PER_ZONE as NB_TARGET_PER_ZONE;
pub use sys::{
    Resolution, TargetOrder, RangingMode, PowerMode,
    FirmwareError, Error, NoPin, DEFAULT_ADDRESS, XTALK_BUFFER_SIZE,
};
#[cfg(feature = "thresholds")]
pub use sys::{
    ThresholdMeasurement, ThresholdType, ThresholdOperation, DetectionThreshold,
};

#[cfg(any(feature = "motion", feature = "thresholds"))]
pub use sys::VL53L8CX_Motion_Configuration as MotionConfiguration;

#[cfg(feature = "thresholds")]
use sys::VL53L8CX_NB_THRESHOLDS;

/// Trait abstracting bus error types for I2C and SPI.
pub trait VlBus {
    type Error: core::fmt::Debug;
}

impl<T: I2c> VlBus for T {
    type Error = <T as embedded_hal::i2c::ErrorType>::Error;
}

/// Wrapper around an `SpiDevice` to implement `VlBus`.
pub struct Vl53l8cxSpiDevice<SPI> {
    pub spi: SPI,
}

impl<SPI: SpiDevice> VlBus for Vl53l8cxSpiDevice<SPI> {
    type Error = <SPI as embedded_hal::spi::ErrorType>::Error;
}

#[repr(C)]
pub struct Vl53l8cxComms<BUS, D> {
    pub bus: BUS,
    pub delay: D,
}

pub struct Vl53l8cx<BUS, D, RST = NoPin, INT = NoPin> {
    pub comms: Vl53l8cxComms<BUS, D>,
    address: SevenBitAddress,
    reset_pin: RST,
    int_pin: INT,
    lp_is_reset_high: bool,
    int_is_data_ready_high: bool,
    stdev: sys::VL53L8CX_Configuration,
}

impl<BUS, D> Vl53l8cx<BUS, D, NoPin, NoPin>
where
    BUS: I2c,
    D: DelayNs,
{
    pub fn new(i2c: BUS, delay: D) -> Self {
        Self {
            comms: Vl53l8cxComms { bus: i2c, delay },
            address: DEFAULT_ADDRESS,
            reset_pin: NoPin,
            int_pin: NoPin,
            lp_is_reset_high: false,
            int_is_data_ready_high: false,
            stdev: unsafe { core::mem::zeroed() },
        }
    }
}

impl<SPI, D> Vl53l8cx<Vl53l8cxSpiDevice<SPI>, D, NoPin, NoPin>
where
    SPI: SpiDevice,
    D: DelayNs,
{
    pub fn new_spi(spi: SPI, delay: D) -> Self {
        Self {
            comms: Vl53l8cxComms { bus: Vl53l8cxSpiDevice { spi }, delay },
            address: DEFAULT_ADDRESS,
            reset_pin: NoPin,
            int_pin: NoPin,
            lp_is_reset_high: false,
            int_is_data_ready_high: false,
            stdev: unsafe { core::mem::zeroed() },
        }
    }
}

impl<BUS, D, RST, INT, E> Vl53l8cx<BUS, D, RST, INT>
where
    BUS: VlBus<Error = E>,
    INT: InputPin,
    <INT as embedded_hal::digital::ErrorType>::Error: core::fmt::Debug,
{
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

impl<BUS, D, RST, INT> Vl53l8cx<BUS, D, RST, INT> {
    /// Attach the sensor's **LPn** line (active low), or another line where
    /// low means "held off".
    ///
    /// Despite its name, LPn is not a reset and has nothing to do with power: it
    /// is what the datasheet calls the "Comms enable". Low disables the sensor's I2C
    /// interface, high enables it, and ST's guidance is that its only purpose is
    /// multi-sensor address assignment: hold every other sensor's LPn low, change
    /// one sensor's address, then leave LPn high. Pulsing it via
    /// [`reset`](Self::reset) therefore does **not** reset the sensor or discard
    /// its firmware; `init()` performs ST's software reboot sequence itself. A
    /// real reset is a power cycle of the sensor supplies (UM2884 for the
    /// VL53L5CX, UM3109 for the VL53L8CX).
    pub fn with_reset_pin_active_low<RstNew>(self, reset_pin: RstNew) -> Vl53l8cx<BUS, D, RstNew, INT> {
        Vl53l8cx {
            comms: self.comms,
            address: self.address,
            reset_pin,
            int_pin: self.int_pin,
            lp_is_reset_high: false,
            int_is_data_ready_high: self.int_is_data_ready_high,
            stdev: self.stdev,
        }
    }

    /// Attach a line where **high** means "held off" or "reset".
    ///
    /// Use this for an inverted LPn (level shifter, transistor) or for the
    /// VL53L5CX's I2C_RST input, which is active high and resets only the I2C
    /// interface, not the sensor. [`reset`](Self::reset) drives the pin high for
    /// 10 ms and releases it. The VL53L8CX has no I2C_RST; its SPI_I2C_N pin
    /// selects the interface, and toggling it only clears the I2C block.
    pub fn with_reset_pin_active_high<RstNew>(self, reset_pin: RstNew) -> Vl53l8cx<BUS, D, RstNew, INT> {
        Vl53l8cx {
            comms: self.comms,
            address: self.address,
            reset_pin,
            int_pin: self.int_pin,
            lp_is_reset_high: true,
            int_is_data_ready_high: self.int_is_data_ready_high,
            stdev: self.stdev,
        }
    }

    /// Attach the sensor's INT (data-ready) pin, which goes **low** when a frame
    /// is ready. This is the part's default: INT is an open-drain output pulled
    /// low for the interrupt pulse.
    pub fn with_interrupt_pin_active_low<IntNew>(self, int_pin: IntNew) -> Vl53l8cx<BUS, D, RST, IntNew> {
        Vl53l8cx {
            comms: self.comms,
            address: self.address,
            reset_pin: self.reset_pin,
            int_pin,
            lp_is_reset_high: self.lp_is_reset_high,
            int_is_data_ready_high: false,
            stdev: self.stdev,
        }
    }

    /// Attach a data-ready line that goes **high** when a frame is ready, for
    /// boards that invert the sensor's INT output.
    pub fn with_interrupt_pin_active_high<IntNew>(self, int_pin: IntNew) -> Vl53l8cx<BUS, D, RST, IntNew> {
        Vl53l8cx {
            comms: self.comms,
            address: self.address,
            reset_pin: self.reset_pin,
            int_pin,
            lp_is_reset_high: self.lp_is_reset_high,
            int_is_data_ready_high: true,
            stdev: self.stdev,
        }
    }
}

impl<BUS, D, RST, INT, E> Vl53l8cx<BUS, D, RST, INT>
where
    BUS: VlBus<Error = E>,
    D: DelayNs,
    RST: OutputPin,
    <RST as embedded_hal::digital::ErrorType>::Error: core::fmt::Debug,
{
    fn set_reset_state(&mut self, target_state_reset: bool) -> Result<(), Error<E>> {
        let set_pin_high = target_state_reset == self.lp_is_reset_high;
        let result = if set_pin_high {
            self.reset_pin.set_high()
        } else {
            self.reset_pin.set_low()
        };
        result.map_err(|_| Error::Pin)
    }

    /// Pulse the line configured by
    /// [`with_reset_pin_active_low`](Self::with_reset_pin_active_low) /
    /// [`with_reset_pin_active_high`](Self::with_reset_pin_active_high): assert
    /// for 10 ms, then release.
    ///
    /// What this does depends on the line. On LPn it only disables I2C for 10 ms
    /// and does not reset the sensor; on the VL53L5CX's I2C_RST it resets the
    /// I2C interface only. Neither discards the firmware. ST's ULD has no reset
    /// function either. A real reset is the supply cycle from UM2884 / UM3109:
    /// drive VDDIO, AVDD and LPn low, wait 10 ms, then drive them high again
    /// (LPn goes low with the supplies because a digital input must not sit above
    /// IOVDD while IOVDD is off; if LPn hangs only on its pull-up to IOVDD rather
    /// than on a GPIO, it follows the rail by itself and needs no driving). `init()`
    /// then expects the supplies and LPn high and I2C_RST low. Since `init()` always starts with ST's software reboot
    /// sequence, calling it is sufficient to re-initialise a running sensor.
    pub fn reset(&mut self) -> Result<(), Error<E>> {
        self.set_reset_state(true)?;
        self.comms.delay.delay_ms(10);
        self.set_reset_state(false)?;
        Ok(())
    }

    pub fn set_i2c_address(&mut self, new_address: SevenBitAddress) -> Result<(), Error<E>> {
        self.bind_platform();
        let i2c_address_8bit = (new_address as u16) << 1;
        let status = unsafe {
            sys::vl53l8cx_set_i2c_address(&mut self.stdev, i2c_address_8bit)
        };
        if status != 0 {
            return Err(Error::Firmware(status.into()));
        }
        self.address = new_address;
        Ok(())
    }
}

impl<BUS, D, RST, INT, E> Vl53l8cx<BUS, D, RST, INT>
where
    BUS: VlBus<Error = E>,
    D: DelayNs,
{
    #[inline]
    fn bind_platform(&mut self) {
        self.stdev.platform.p_com = &mut self.comms as *mut _ as *mut core::ffi::c_void;
        self.stdev.platform.address = (self.address as u16) << 1;
    }

    pub fn start_ranging(&mut self) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::start_ranging(&mut self.stdev);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn stop_ranging(&mut self) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::stop_ranging(&mut self.stdev);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn check_data_ready(&mut self) -> Result<bool, Error<E>> {
        self.bind_platform();
        let (status, is_ready) = sys::wrappers::check_data_ready(&mut self.stdev);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(is_ready != 0),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn set_resolution(&mut self, resolution: Resolution) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_resolution(&mut self.stdev, resolution as u8);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn set_ranging_frequency_hz(&mut self, frequency_hz: u8) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_ranging_frequency_hz(&mut self.stdev, frequency_hz);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn set_ranging_mode(&mut self, mode: RangingMode) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_ranging_mode(&mut self.stdev, mode as u8);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn set_integration_time_ms(&mut self, time_ms: u32) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_integration_time_ms(&mut self.stdev, time_ms);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn set_sharpener_percent(&mut self, percent: u8) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_sharpener_percent(&mut self.stdev, percent);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn set_target_order(&mut self, order: TargetOrder) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_target_order(&mut self.stdev, order as u8);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_power_mode(&mut self.stdev, mode as u8);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn get_resolution(&mut self) -> Result<Resolution, Error<E>> {
        self.bind_platform();
        let mut resolution: u8 = 0;
        let status = unsafe { sys::vl53l8cx_get_resolution(&mut self.stdev, &mut resolution) };
        match status {
            sys::VL53L8CX_STATUS_OK => {
                if resolution == sys::VL53L8CX_RESOLUTION_8X8 {
                     Ok(Resolution::Res8x8)
                } else {
                     Ok(Resolution::Res4x4)
                }
            },
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn get_ranging_frequency_hz(&mut self) -> Result<u8, Error<E>> {
        self.bind_platform();
        let mut freq: u8 = 0;
        let status = unsafe { sys::vl53l8cx_get_ranging_frequency_hz(&mut self.stdev, &mut freq) };
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(freq),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn get_integration_time_ms(&mut self) -> Result<u32, Error<E>> {
        self.bind_platform();
        let (status, time) = sys::wrappers::get_integration_time_ms(&mut self.stdev);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(time),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn get_ranging_mode(&mut self) -> Result<RangingMode, Error<E>> {
        self.bind_platform();
        let (status, mode) = sys::wrappers::get_ranging_mode(&mut self.stdev);
        match status {
            sys::VL53L8CX_STATUS_OK => {
                if mode == sys::VL53L8CX_RANGING_MODE_CONTINUOUS {
                    Ok(RangingMode::Continuous)
                } else {
                    Ok(RangingMode::Autonomous)
                }
            },
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn get_sharpener_percent(&mut self) -> Result<u8, Error<E>> {
        self.bind_platform();
        let (status, percent) = sys::wrappers::get_sharpener_percent(&mut self.stdev);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(percent),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn get_target_order(&mut self) -> Result<TargetOrder, Error<E>> {
        self.bind_platform();
        let (status, order) = sys::wrappers::get_target_order(&mut self.stdev);
        match status {
            sys::VL53L8CX_STATUS_OK => {
                if order == sys::VL53L8CX_TARGET_ORDER_STRONGEST {
                    Ok(TargetOrder::Strongest)
                } else {
                    Ok(TargetOrder::Closest)
                }
            },
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn get_power_mode(&mut self) -> Result<PowerMode, Error<E>> {
        self.bind_platform();
        let (status, mode) = sys::wrappers::get_power_mode(&mut self.stdev);
        match status {
            sys::VL53L8CX_STATUS_OK => {
                if mode == sys::VL53L8CX_POWER_MODE_WAKEUP {
                    Ok(PowerMode::Wakeup)
                } else if mode == sys::VL53L8CX_POWER_MODE_DEEP_SLEEP {
                    Ok(PowerMode::DeepSleep)
                } else {
                    Ok(PowerMode::Sleep)
                }
            },
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn get_vhv_repeat_count(&mut self) -> Result<u32, Error<E>> {
        self.bind_platform();
        let (status, count) = sys::wrappers::get_vhv_repeat_count(&mut self.stdev);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(count),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn set_vhv_repeat_count(&mut self, count: u32) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_vhv_repeat_count(&mut self.stdev, count);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn get_external_sync_pin_enable(&mut self) -> Result<bool, Error<E>> {
        self.bind_platform();
        let (status, enabled) = sys::wrappers::get_external_sync_pin_enable(&mut self.stdev);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(enabled != 0),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn set_external_sync_pin_enable(&mut self, enabled: bool) -> Result<(), Error<E>> {
        self.bind_platform();
        let val = if enabled { 1 } else { 0 };
        let status = sys::wrappers::set_external_sync_pin_enable(&mut self.stdev, val);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn init(&mut self) -> Result<(), Error<E>> {
        self.stdev.platform.address = (self.address as u16) << 1;
        self.bind_platform();

        if !self.is_alive()? {
            return Err(Error::NotAlive);
        }

        self.bind_platform();
        let status = unsafe { sys::vl53l8cx_init(&mut self.stdev) };
        if status != 0 {
            return Err(Error::Firmware(status.into()));
        }
        Ok(())
    }

    pub fn is_alive(&mut self) -> Result<bool, Error<E>> {
        self.bind_platform();
        let (status, is_alive) = sys::wrappers::is_alive(&mut self.stdev);
        match status { sys::VL53L8CX_STATUS_OK => Ok(is_alive != 0), s => Err(Error::Firmware(s.into())), }
    }

    /// Read the latest ranging frame. See [`ResultsData`] for the array layout
    /// (per-target arrays are indexed `zone * NB_TARGET_PER_ZONE + target`).
    pub fn get_ranging_data(&mut self) -> Result<ResultsData, Error<E>> {
        self.bind_platform();
        let (status, value) = sys::wrappers::get_ranging_data(&mut self.stdev);
        match status { sys::VL53L8CX_STATUS_OK => Ok(value), s => Err(Error::Firmware(s.into())), }
    }

}

#[cfg(test)]
mod tests;

/// Log data in 16-byte chunks when the `defmt` feature is enabled.
#[cfg(feature = "defmt")]
pub fn trace_data(data: &[u8]) {
    for chunk in data.chunks(16) {
        defmt::trace!("  {=[u8]:#04x}", chunk);
    }
}

// --- Macro for Dynamic Platform Context ---
#[macro_export]
macro_rules! bind_platform_driver {
    ($i2c_type:ty, $delay_type:ty) => {
        #[no_mangle]
        pub extern "C" fn vl53l8cx_platform_read(
            p_platform: *mut $crate::VL53L8CX_Platform,
            index: u16,
            p_values: *mut u8,
            size: u32,
        ) -> u8 {
            if p_platform.is_null() { return 255; }
            let platform = unsafe { &*p_platform };
            let comms = unsafe { &mut *(platform.p_com as *mut $crate::Vl53l8cxComms<$i2c_type, $delay_type>) };

            let data_slice = unsafe { core::slice::from_raw_parts_mut(p_values, size as usize) };
            let index_bytes = index.to_be_bytes();
            let address = (platform.address >> 1) as u8;

            match embedded_hal::i2c::I2c::write_read(&mut comms.bus, address, &index_bytes, data_slice) {
                Ok(_) => {
                    #[cfg(feature = "defmt")]
                    {
                        defmt::trace!("i2c R [{:#04x}] reg={:#06x} len={}", address, index, size);
                        $crate::trace_data(data_slice);
                    }
                    0
                }
                Err(_) => {
                    #[cfg(feature = "defmt")]
                    defmt::trace!("i2c R [{:#04x}] reg={:#06x} -> ERR", address, index);
                    1
                }
            }
        }

        #[no_mangle]
        pub extern "C" fn vl53l8cx_platform_write(
            p_platform: *mut $crate::VL53L8CX_Platform,
            index: u16,
            p_values: *mut u8,
            size: u32,
        ) -> u8 {
            if p_platform.is_null() { return 255; }
            let platform = unsafe { &*p_platform };
            let comms = unsafe { &mut *(platform.p_com as *mut $crate::Vl53l8cxComms<$i2c_type, $delay_type>) };

            let data_slice = unsafe { core::slice::from_raw_parts(p_values, size as usize) };
            let index_bytes = index.to_be_bytes();
            let address = (platform.address >> 1) as u8;

            #[cfg(feature = "defmt")]
            {
                defmt::trace!("i2c W [{:#04x}] reg={:#06x} len={}", address, index, size);
                $crate::trace_data(data_slice);
            }

            let mut operations = [
                embedded_hal::i2c::Operation::Write(&index_bytes),
                embedded_hal::i2c::Operation::Write(data_slice),
            ];

            match embedded_hal::i2c::I2c::transaction(&mut comms.bus, address, &mut operations) {
                Ok(_) => 0,
                Err(_) => {
                    #[cfg(feature = "defmt")]
                    defmt::trace!("i2c W -> ERR");
                    1
                }
            }
        }

        #[no_mangle]
        pub extern "C" fn vl53l8cx_platform_wait_ms(p_platform: *mut $crate::VL53L8CX_Platform, milliseconds: u32) -> u8 {
            if p_platform.is_null() { return 255; }
            let platform = unsafe { &*p_platform };
            let comms = unsafe { &mut *(platform.p_com as *mut $crate::Vl53l8cxComms<$i2c_type, $delay_type>) };

            embedded_hal::delay::DelayNs::delay_ms(&mut comms.delay, milliseconds);
            0
        }

        #[no_mangle]
        pub extern "C" fn vl53l8cx_platform_init(_p_platform: *mut $crate::VL53L8CX_Platform) -> u8 { 0 }

        #[no_mangle]
        pub extern "C" fn vl53l8cx_platform_terminate(_p_platform: *mut $crate::VL53L8CX_Platform) -> u8 { 0 }
    };
}

#[macro_export]
macro_rules! bind_platform_driver_spi {
    ($spi_type:ty, $delay_type:ty) => {
        #[no_mangle]
        pub extern "C" fn vl53l8cx_platform_read(
            p_platform: *mut $crate::VL53L8CX_Platform,
            index: u16,
            p_values: *mut u8,
            size: u32,
        ) -> u8 {
            if p_platform.is_null() { return 255; }
            let platform = unsafe { &*p_platform };
            let comms = unsafe { &mut *(platform.p_com as *mut $crate::Vl53l8cxComms<$crate::Vl53l8cxSpiDevice<$spi_type>, $delay_type>) };

            let data_slice = unsafe { core::slice::from_raw_parts_mut(p_values, size as usize) };
            let addr_bytes = (index & 0x7FFF).to_be_bytes();

            match embedded_hal::spi::SpiDevice::transaction(&mut comms.bus.spi, &mut [
                embedded_hal::spi::Operation::Write(&addr_bytes),
                embedded_hal::spi::Operation::Read(data_slice),
            ]) {
                Ok(_) => {
                    #[cfg(feature = "defmt")]
                    {
                        defmt::trace!("spi R reg={:#06x} len={}", index, size);
                        $crate::trace_data(data_slice);
                    }
                    0
                }
                Err(_) => {
                    #[cfg(feature = "defmt")]
                    defmt::trace!("spi R reg={:#06x} -> ERR", index);
                    1
                }
            }
        }

        #[no_mangle]
        pub extern "C" fn vl53l8cx_platform_write(
            p_platform: *mut $crate::VL53L8CX_Platform,
            index: u16,
            p_values: *mut u8,
            size: u32,
        ) -> u8 {
            if p_platform.is_null() { return 255; }
            let platform = unsafe { &*p_platform };
            let comms = unsafe { &mut *(platform.p_com as *mut $crate::Vl53l8cxComms<$crate::Vl53l8cxSpiDevice<$spi_type>, $delay_type>) };

            let data_slice = unsafe { core::slice::from_raw_parts(p_values, size as usize) };
            let addr_bytes = (index | 0x8000).to_be_bytes();

            #[cfg(feature = "defmt")]
            {
                defmt::trace!("spi W reg={:#06x} len={}", index, size);
                $crate::trace_data(data_slice);
            }
            match embedded_hal::spi::SpiDevice::transaction(&mut comms.bus.spi, &mut [
                embedded_hal::spi::Operation::Write(&addr_bytes),
                embedded_hal::spi::Operation::Write(data_slice),
            ]) {
                Ok(_) => 0,
                Err(_) => {
                    #[cfg(feature = "defmt")]
                    defmt::trace!("spi W -> ERR");
                    1
                }
            }
        }

        #[no_mangle]
        pub extern "C" fn vl53l8cx_platform_wait_ms(p_platform: *mut $crate::VL53L8CX_Platform, milliseconds: u32) -> u8 {
            if p_platform.is_null() { return 255; }
            let platform = unsafe { &*p_platform };
            let comms = unsafe { &mut *(platform.p_com as *mut $crate::Vl53l8cxComms<$crate::Vl53l8cxSpiDevice<$spi_type>, $delay_type>) };

            embedded_hal::delay::DelayNs::delay_ms(&mut comms.delay, milliseconds);
            0
        }

        #[no_mangle]
        pub extern "C" fn vl53l8cx_platform_init(_p_platform: *mut $crate::VL53L8CX_Platform) -> u8 { 0 }

        #[no_mangle]
        pub extern "C" fn vl53l8cx_platform_terminate(_p_platform: *mut $crate::VL53L8CX_Platform) -> u8 { 0 }
    };
}

#[cfg(any(feature = "motion", feature = "thresholds"))]
impl<BUS, D, RST, INT, E> Vl53l8cx<BUS, D, RST, INT>
where
    BUS: VlBus<Error = E>,
    D: DelayNs,
{
    pub fn motion_indicator_init(&mut self, motion_config: &mut sys::VL53L8CX_Motion_Configuration, resolution: Resolution) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::motion_indicator_init(&mut self.stdev, motion_config, resolution as u8);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn motion_indicator_set_distance_motion(&mut self, motion_config: &mut sys::VL53L8CX_Motion_Configuration, min_distance_mm: u16, max_distance_mm: u16) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::motion_indicator_set_distance_motion(&mut self.stdev, motion_config, min_distance_mm, max_distance_mm);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn motion_indicator_set_resolution(
        &mut self,
        motion_config: &mut sys::VL53L8CX_Motion_Configuration,
        resolution: Resolution,
    ) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::motion_indicator_set_resolution(&mut self.stdev, motion_config, resolution as u8);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn get_detection_thresholds_enable(&mut self) -> Result<bool, Error<E>> {
        self.bind_platform();
        let (status, enabled) = sys::wrappers::get_detection_thresholds_enable(&mut self.stdev);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(enabled == 1),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn set_detection_thresholds_enable(&mut self, enabled: bool) -> Result<(), Error<E>> {
        self.bind_platform();
        let val = if enabled { 1 } else { 0 };
        let status = sys::wrappers::set_detection_thresholds_enable(&mut self.stdev, val);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }
}

#[cfg(feature = "xtalk")]
impl<BUS, D, RST, INT, E> Vl53l8cx<BUS, D, RST, INT>
where
    BUS: VlBus<Error = E>,
    D: DelayNs,
{
    pub fn calibrate_xtalk(&mut self, reflectance_percent: u16, nb_samples: u8, distance_mm: u16) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::calibrate_xtalk(&mut self.stdev, reflectance_percent, nb_samples, distance_mm);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn get_xtalk_margin(&mut self) -> Result<u32, Error<E>> {
        self.bind_platform();
        let (status, margin) = sys::wrappers::get_xtalk_margin(&mut self.stdev);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(margin),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn set_xtalk_margin(&mut self, margin: u32) -> Result<(), Error<E>> {
        self.bind_platform();
        let status = sys::wrappers::set_xtalk_margin(&mut self.stdev, margin);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn get_caldata_xtalk(&mut self, buffer: &mut [u8]) -> Result<(), Error<E>> {
        // Use local constant for buffer size
        if buffer.len() < XTALK_BUFFER_SIZE {
             return Err(Error::Firmware(sys::VL53L8CX_STATUS_INVALID_PARAM.into()));
        }
        self.bind_platform();
        let status = sys::wrappers::get_caldata_xtalk(&mut self.stdev, buffer.as_mut_ptr());
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    pub fn set_caldata_xtalk(&mut self, buffer: &mut [u8]) -> Result<(), Error<E>> {
         // Use local constant for buffer size
         if buffer.len() < XTALK_BUFFER_SIZE {
             return Err(Error::Firmware(sys::VL53L8CX_STATUS_INVALID_PARAM.into()));
        }
        self.bind_platform();
        let status = sys::wrappers::set_caldata_xtalk(&mut self.stdev, buffer.as_mut_ptr());
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }
}

#[cfg(feature = "thresholds")]
impl<BUS, D, RST, INT, E> Vl53l8cx<BUS, D, RST, INT>
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
    pub fn get_detection_thresholds(&mut self) -> Result<[DetectionThreshold; VL53L8CX_NB_THRESHOLDS as usize], Error<E>> {
        self.bind_platform();
        let mut raw_thresholds: [sys::VL53L8CX_DetectionThresholds; VL53L8CX_NB_THRESHOLDS as usize] = unsafe { core::mem::zeroed() };
        let ptr = raw_thresholds.as_mut_ptr() as *mut sys::VL53L8CX_DetectionThresholds;

        let status = sys::wrappers::get_detection_thresholds(&mut self.stdev, ptr);
        if status != sys::VL53L8CX_STATUS_OK {
            return Err(Error::Firmware(status.into()));
        }

        let rust_thresholds: [DetectionThreshold; VL53L8CX_NB_THRESHOLDS as usize] = unsafe {
            core::mem::transmute(raw_thresholds)
        };


        Ok(rust_thresholds)
    }

    pub fn set_detection_thresholds(&mut self, thresholds: [DetectionThreshold; VL53L8CX_NB_THRESHOLDS as usize]) -> Result<(), Error<E>> {
        self.bind_platform();

        let raw_thresholds: [sys::VL53L8CX_DetectionThresholds; VL53L8CX_NB_THRESHOLDS as usize] = unsafe {
            core::mem::transmute(thresholds)
        };
        let ptr = raw_thresholds.as_ptr() as *mut sys::VL53L8CX_DetectionThresholds;

        let status = sys::wrappers::set_detection_thresholds(&mut self.stdev, ptr);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Read whether detection-threshold auto-stop is enabled (default: off).
    ///
    /// See [`set_detection_thresholds_auto_stop`](Self::set_detection_thresholds_auto_stop).
    pub fn get_detection_thresholds_auto_stop(&mut self) -> Result<bool, Error<E>> {
        self.bind_platform();
        let (status, auto_stop) = sys::wrappers::get_detection_thresholds_auto_stop(&mut self.stdev);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(auto_stop != 0),
            s => Err(Error::Firmware(s.into())),
        }
    }

    /// Enable or disable detection-threshold auto-stop.
    ///
    /// In [`RangingMode::Autonomous`] with detection thresholds enabled, the
    /// sensor only raises INT when a threshold is crossed, and normally does so
    /// once the measurement completes. With auto-stop enabled the sensor
    /// instead raises INT and aborts the measurement in progress straight
    /// away, so the host wakes sooner and the sensor stops ranging on its own.
    /// After such an interrupt [`stop_ranging`](Self::stop_ranging) must still
    /// be called for a clean stop before ranging is started again.
    ///
    /// Configure this while the sensor is initialised and not ranging.
    pub fn set_detection_thresholds_auto_stop(&mut self, enabled: bool) -> Result<(), Error<E>> {
        self.bind_platform();
        let val = if enabled { 1 } else { 0 };
        let status = sys::wrappers::set_detection_thresholds_auto_stop(&mut self.stdev, val);
        match status {
            sys::VL53L8CX_STATUS_OK => Ok(()),
            s => Err(Error::Firmware(s.into())),
        }
    }
}
