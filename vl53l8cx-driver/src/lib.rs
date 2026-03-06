#![no_std]
/*!
A platform-agnostic driver for the VL53L8CX Time-of-Flight sensor.

This driver is built on top of the `embedded-hal` traits and uses the official
ST Microelectronics VL53L8CX ULD (Ultra Lite Driver) via the `vl53l8cx-sys` crate.
*/

use embedded_hal::delay::DelayNs;

#[cfg(feature = "async-hal")]
use embedded_hal_async::digital::Wait;
use embedded_hal::digital::InputPin;
use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::digital::OutputPin;

// We import the sys crate internally but do NOT make it public.
use vl53l8cx_sys as sys;

// --- Public Type Re-exports ---
pub use sys::VL53L8CX_Platform;
pub use sys::VL53L8CX_ResultsData as ResultsData;

#[cfg(any(feature = "motion", feature = "thresholds"))]
pub use sys::VL53L8CX_Motion_Configuration as MotionConfiguration;

/// The default I2C address of the VL53L8CX sensor.
pub const DEFAULT_ADDRESS: u8 = 0x29;

/// The required buffer size for crosstalk calibration data (776 bytes).
pub const XTALK_BUFFER_SIZE: usize = 776;

#[cfg(feature = "thresholds")]
use sys::VL53L8CX_NB_THRESHOLDS;

// --- Idiomatic Rust Enums ---

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Resolution {
    Res4x4 = sys::VL53L8CX_RESOLUTION_4X4,
    Res8x8 = sys::VL53L8CX_RESOLUTION_8X8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TargetOrder {
    Closest = sys::VL53L8CX_TARGET_ORDER_CLOSEST,
    Strongest = sys::VL53L8CX_TARGET_ORDER_STRONGEST,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RangingMode {
    Continuous = sys::VL53L8CX_RANGING_MODE_CONTINUOUS,
    Autonomous = sys::VL53L8CX_RANGING_MODE_AUTONOMOUS,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PowerMode {
    Sleep = sys::VL53L8CX_POWER_MODE_SLEEP,
    Wakeup = sys::VL53L8CX_POWER_MODE_WAKEUP,
    DeepSleep = sys::VL53L8CX_POWER_MODE_DEEP_SLEEP,
}

#[cfg(feature = "thresholds")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ThresholdMeasurement {
    DistanceMm = sys::VL53L8CX_DISTANCE_MM,
    SignalPerSpadKcps = sys::VL53L8CX_SIGNAL_PER_SPAD_KCPS,
    RangeSigmaMm = sys::VL53L8CX_RANGE_SIGMA_MM,
    AmbientPerSpadKcps = sys::VL53L8CX_AMBIENT_PER_SPAD_KCPS,
    NbTargetDetected = sys::VL53L8CX_NB_TARGET_DETECTED,
    TargetStatus = sys::VL53L8CX_TARGET_STATUS,
    NbSpadsEnabled = sys::VL53L8CX_NB_SPADS_ENABLED,
    MotionIndicator = sys::VL53L8CX_MOTION_INDICATOR,
}

#[cfg(feature = "thresholds")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ThresholdType {
    InWindow = sys::VL53L8CX_IN_WINDOW,
    OutOfWindow = sys::VL53L8CX_OUT_OF_WINDOW,
    LessThanEqualMin = sys::VL53L8CX_LESS_THAN_EQUAL_MIN_CHECKER,
    GreaterThanMax = sys::VL53L8CX_GREATER_THAN_MAX_CHECKER,
    EqualMin = sys::VL53L8CX_EQUAL_MIN_CHECKER,
    NotEqualMin = sys::VL53L8CX_NOT_EQUAL_MIN_CHECKER,
}

#[cfg(feature = "thresholds")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ThresholdOperation {
    Or = sys::VL53L8CX_OPERATION_OR,
    And = sys::VL53L8CX_OPERATION_AND,
    None = sys::VL53L8CX_OPERATION_NONE,
}

#[cfg(feature = "thresholds")]
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct DetectionThreshold {
    pub low_threshold: i32,
    pub high_threshold: i32,
    pub measurement: ThresholdMeasurement,
    pub threshold_type: ThresholdType,
    pub zone_num: u8,
    pub mathematic_operation: ThresholdOperation,
}

/// A placeholder type for an unconfigured hardware pin.
pub struct NoPin;

impl embedded_hal::digital::ErrorType for NoPin {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for NoPin {
    fn set_low(&mut self) -> Result<(), Self::Error> { Ok(()) }
    fn set_high(&mut self) -> Result<(), Self::Error> { Ok(()) }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum FirmwareError {
    Timeout,
    CorruptedFrame,
    LaserSafety,
    XtalkFailed,
    FwChecksumFail,
    McuError,
    InvalidParam,
    Unknown(u8),
}

impl From<u8> for FirmwareError {
    fn from(code: u8) -> Self {
        match code {
            sys::VL53L8CX_STATUS_TIMEOUT_ERROR => FirmwareError::Timeout,
            sys::VL53L8CX_STATUS_CORRUPTED_FRAME => FirmwareError::CorruptedFrame,
            sys::VL53L8CX_STATUS_LASER_SAFETY => FirmwareError::LaserSafety,
            sys::VL53L8CX_STATUS_XTALK_FAILED => FirmwareError::XtalkFailed,
            sys::VL53L8CX_STATUS_FW_CHECKSUM_FAIL => FirmwareError::FwChecksumFail,
            sys::VL53L8CX_MCU_ERROR => FirmwareError::McuError,
            sys::VL53L8CX_STATUS_INVALID_PARAM => FirmwareError::InvalidParam,
            c => FirmwareError::Unknown(c),
        }
    }
}

/// Represents errors that can occur in the driver.
#[derive(Debug)]
pub enum Error<E> {
    I2c(E),
    NotAlive,
    Firmware(FirmwareError),
    PinMissing,
    Pin,
}

#[cfg(feature = "std")]
impl<E: std::error::Error + 'static> std::error::Error for Error<E> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::I2c(e) => Some(e),
            _ => None,
        }
    }
}

#[cfg(feature = "std")]
impl<E: std::fmt::Debug> std::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::I2c(e) => write!(f, "I2C error: {:?}", e),
            Error::NotAlive => write!(f, "Sensor not found at address"),
            Error::Firmware(e) => write!(f, "Firmware error: {:?}", e),
            Error::PinMissing => write!(f, "A required hardware pin was not configured"),
            Error::Pin => write!(f, "Pin error"),
        }
    }
}

#[repr(C)]
pub struct Vl53l8cxComms<I2C, D> {
    pub i2c: I2C,
    pub delay: D,
}

pub struct Vl53l8cx<I2C, D, RST = NoPin, INT = NoPin> {
    pub comms: Vl53l8cxComms<I2C, D>,
    address: SevenBitAddress,
    reset_pin: RST,
    int_pin: INT,
    has_reset: bool,
    lp_is_reset_high: bool,
    int_is_data_ready_high: bool,
    stdev: sys::VL53L8CX_Configuration,
}

impl<I2C, D> Vl53l8cx<I2C, D, NoPin, NoPin>
where
    I2C: I2c,
    D: DelayNs,
{
    pub fn new(i2c: I2C, delay: D) -> Self {
        Self {
            comms: Vl53l8cxComms { i2c, delay },
            address: DEFAULT_ADDRESS,
            reset_pin: NoPin,
            int_pin: NoPin,
            has_reset: false,
            lp_is_reset_high: false,
            int_is_data_ready_high: false,
            stdev: unsafe { core::mem::zeroed() },
        }
    }
}

impl<I2C, D, RST, INT, E> Vl53l8cx<I2C, D, RST, INT>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
    RST: OutputPin,
    <RST as embedded_hal::digital::ErrorType>::Error: core::fmt::Debug,
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

#[cfg(feature = "async-hal")]
impl<I2C, D, RST, INT, E> Vl53l8cx<I2C, D, RST, INT>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
    RST: OutputPin,
    <RST as embedded_hal::digital::ErrorType>::Error: core::fmt::Debug,
    INT: Wait,
    <INT as embedded_hal::digital::ErrorType>::Error: core::fmt::Debug,
{
    pub async fn wait_for_data_ready_async(&mut self) -> Result<(), Error<E>> {
        let result = if self.int_is_data_ready_high {
            self.int_pin.wait_for_high().await
        } else {
            self.int_pin.wait_for_low().await
        };
        result.map_err(|_| Error::Pin)?;
        Ok(())
    }
}

impl<I2C, D, RST, INT> Vl53l8cx<I2C, D, RST, INT> {
    pub fn with_reset_pin_active_low<RstNew>(self, reset_pin: RstNew) -> Vl53l8cx<I2C, D, RstNew, INT> {
        Vl53l8cx {
            comms: self.comms,
            address: self.address,
            reset_pin,
            int_pin: self.int_pin,
            has_reset: true,
            lp_is_reset_high: false,
            int_is_data_ready_high: self.int_is_data_ready_high,
            stdev: self.stdev,
        }
    }

    pub fn with_reset_pin_active_high<RstNew>(self, reset_pin: RstNew) -> Vl53l8cx<I2C, D, RstNew, INT> {
        Vl53l8cx {
            comms: self.comms,
            address: self.address,
            reset_pin,
            int_pin: self.int_pin,
            has_reset: true,
            lp_is_reset_high: true,
            int_is_data_ready_high: self.int_is_data_ready_high,
            stdev: self.stdev,
        }
    }

    pub fn with_interrupt_pin_active_low<IntNew>(self, int_pin: IntNew) -> Vl53l8cx<I2C, D, RST, IntNew> {
        Vl53l8cx {
            comms: self.comms,
            address: self.address,
            reset_pin: self.reset_pin,
            int_pin,
            has_reset: self.has_reset,
            lp_is_reset_high: self.lp_is_reset_high,
            int_is_data_ready_high: false,
            stdev: self.stdev,
        }
    }

    pub fn with_interrupt_pin_active_high<IntNew>(self, int_pin: IntNew) -> Vl53l8cx<I2C, D, RST, IntNew> {
        Vl53l8cx {
            comms: self.comms,
            address: self.address,
            reset_pin: self.reset_pin,
            int_pin,
            has_reset: self.has_reset,
            lp_is_reset_high: self.lp_is_reset_high,
            int_is_data_ready_high: true,
            stdev: self.stdev,
        }
    }
}

impl<I2C, D, RST, INT, E> Vl53l8cx<I2C, D, RST, INT>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
    RST: OutputPin,
    <RST as embedded_hal::digital::ErrorType>::Error: core::fmt::Debug,
{
    #[inline]
    fn bind_platform(&mut self) {
        self.stdev.platform.p_com = &mut self.comms as *mut _ as *mut core::ffi::c_void;
        self.stdev.platform.address = self.address as u16;
    }

    fn set_reset_state(&mut self, target_state_reset: bool) -> Result<(), Error<E>> {
        let set_pin_high = target_state_reset ^ self.lp_is_reset_high;
        let result = if set_pin_high {
            self.reset_pin.set_high()
        } else {
            self.reset_pin.set_low()
        };
        result.map_err(|_| Error::Pin)
    }

    pub fn reset(&mut self) -> Result<(), Error<E>> {
        if !self.has_reset {
            return Err(Error::PinMissing);
        }
        self.set_reset_state(true)?;
        self.comms.delay.delay_ms(10);
        self.set_reset_state(false)?;
        Ok(())
    }

    pub fn set_address(&mut self, new_address: u8) -> Result<(), Error<E>> {
        if !self.has_reset {
            return Err(Error::PinMissing);
        }
        self.set_reset_state(true)?;
        self.comms.delay.delay_ms(2);
        self.set_reset_state(false)?;
        self.comms.delay.delay_ms(2);

        self.bind_platform();
        let status = unsafe {
            sys::vl53l8cx_set_i2c_address(&mut self.stdev, new_address as u16)
        };

        if status != 0 {
            return Err(Error::Firmware(status.into()));
        }
        self.address = new_address;
        self.stdev.platform.address = new_address as u16;
        Ok(())
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
        self.set_reset_state(false)?;
        self.comms.delay.delay_ms(2);

        self.stdev.platform.address = self.address as u16;
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

    pub fn get_ranging_data(&mut self) -> Result<sys::VL53L8CX_ResultsData, Error<E>> {
        self.bind_platform();
        let (status, value) = sys::wrappers::get_ranging_data(&mut self.stdev);
        match status { sys::VL53L8CX_STATUS_OK => Ok(value), s => Err(Error::Firmware(s.into())), }
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
            let address = platform.address as u8;

            match embedded_hal::i2c::I2c::write_read(&mut comms.i2c, address, &index_bytes, data_slice) {
                Ok(_) => 0,
                Err(_) => 1,
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
            let address = platform.address as u8;

            let mut operations = [
                embedded_hal::i2c::Operation::Write(&index_bytes),
                embedded_hal::i2c::Operation::Write(data_slice),
            ];

            match embedded_hal::i2c::I2c::transaction(&mut comms.i2c, address, &mut operations) {
                Ok(_) => 0,
                Err(_) => 1,
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


#[cfg(any(feature = "motion", feature = "thresholds"))]
impl<I2C, D, RST, INT, E> Vl53l8cx<I2C, D, RST, INT>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
    RST: OutputPin,
    <RST as embedded_hal::digital::ErrorType>::Error: core::fmt::Debug,
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
impl<I2C, D, RST, INT, E> Vl53l8cx<I2C, D, RST, INT>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
    RST: OutputPin,
    <RST as embedded_hal::digital::ErrorType>::Error: core::fmt::Debug,
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
impl<I2C, D, RST, INT, E> Vl53l8cx<I2C, D, RST, INT>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
    RST: OutputPin,
    <RST as embedded_hal::digital::ErrorType>::Error: core::fmt::Debug,
{
    fn scale_thresholds_for_read(thresholds: &mut [DetectionThreshold]) {
        for t in thresholds.iter_mut() {
            let scale = match t.measurement {
                ThresholdMeasurement::DistanceMm => 4,
                ThresholdMeasurement::SignalPerSpadKcps | ThresholdMeasurement::AmbientPerSpadKcps => 2048,
                ThresholdMeasurement::RangeSigmaMm => 128,
                ThresholdMeasurement::NbSpadsEnabled => 256,
                ThresholdMeasurement::MotionIndicator => 65535,
                _ => 1,
            };
            if scale != 1 {
                t.low_threshold /= scale as i32;
                t.high_threshold /= scale as i32;
            }
        }
    }

    fn scale_thresholds_for_write(thresholds: &mut [DetectionThreshold]) {
        for t in thresholds.iter_mut() {
            let scale = match t.measurement {
                ThresholdMeasurement::DistanceMm => 4,
                ThresholdMeasurement::SignalPerSpadKcps | ThresholdMeasurement::AmbientPerSpadKcps => 2048,
                ThresholdMeasurement::RangeSigmaMm => 128,
                ThresholdMeasurement::NbSpadsEnabled => 256,
                ThresholdMeasurement::MotionIndicator => 65535,
                _ => 1,
            };
            if scale != 1 {
                t.low_threshold *= scale as i32;
                t.high_threshold *= scale as i32;
            }
        }
    }

    pub fn get_detection_thresholds(&mut self) -> Result<[DetectionThreshold; VL53L8CX_NB_THRESHOLDS as usize], Error<E>> {
        self.bind_platform();
        let mut raw_thresholds: [sys::VL53L8CX_DetectionThresholds; VL53L8CX_NB_THRESHOLDS as usize] = unsafe { core::mem::zeroed() };
        let ptr = raw_thresholds.as_mut_ptr() as *mut sys::VL53L8CX_DetectionThresholds;

        let status = sys::wrappers::get_detection_thresholds(&mut self.stdev, ptr);
        if status != sys::VL53L8CX_STATUS_OK {
            return Err(Error::Firmware(status.into()));
        }

        let mut rust_thresholds: [DetectionThreshold; VL53L8CX_NB_THRESHOLDS as usize] = unsafe {
            core::mem::transmute(raw_thresholds)
        };

        Self::scale_thresholds_for_read(&mut rust_thresholds);

        Ok(rust_thresholds)
    }

    pub fn set_detection_thresholds(&mut self, mut thresholds: [DetectionThreshold; VL53L8CX_NB_THRESHOLDS as usize]) -> Result<(), Error<E>> {
        self.bind_platform();
        Self::scale_thresholds_for_write(&mut thresholds);

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
}
