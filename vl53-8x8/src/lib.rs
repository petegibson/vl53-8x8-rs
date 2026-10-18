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
The entry point is [`Sensor`]; [`Spi`] adapts an `SpiDevice` to the bus trait
and [`BlockingI2c`] / [`BlockingSpi`] wrap blocking `embedded-hal` I2C and SPI
buses (blocking HALs, RTIC 2, no executor).

Crate family:

- `vl53-8x8-types`: shared enums, `Error` and result types; no dependencies.
  Re-exported here so dependants need only this crate.
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

use bus::VlBusAsync;
use embedded_hal::digital::{InputPin, OutputPin};
pub use embedded_hal::i2c::SevenBitAddress;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use state::State;
use vl53_8x8_types as types;

// Shared types, re-exported so dependants need only this crate.
pub use types::{
    Error, FirmwareError, NoPin, PowerMode, RangingMode, Resolution, TargetOrder,
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
    VL53L8CX_NB_THRESHOLDS,
};

/// Number of detection thresholds as usize for array sizing.
#[cfg(feature = "thresholds")]
const NB_THRESHOLDS: usize = VL53L8CX_NB_THRESHOLDS as usize;

#[cfg(any(feature = "motion", feature = "thresholds"))]
pub use types::MotionConfiguration;

pub use bus::{BlockingI2c, BlockingSpi, Spi};
pub use firmware::{FirmwareSource, PatchedFirmware, SliceFirmware};
pub use sensor::{detect, Variant};

pub struct Sensor<BUS, D, RST = NoPin, INT = NoPin> {
    bus: BUS,
    delay: D,
    address: SevenBitAddress,
    reset_pin: RST,
    int_pin: INT,
    lp_is_reset_high: bool,
    int_is_data_ready_high: bool,
    stdev: State,
    /// Sensor variant. Defaults to [`Variant::L8`] and is set to the detected
    /// part by [`Sensor::init`].
    variant: Variant,
}

// --- Constructors ---

impl<BUS, D> Sensor<BUS, D, NoPin, NoPin>
where
    BUS: VlBusAsync,
    D: DelayNs,
{
    pub fn new(bus: BUS, delay: D) -> Self {
        Self {
            bus,
            delay,
            address: DEFAULT_ADDRESS,
            reset_pin: NoPin,
            int_pin: NoPin,
            lp_is_reset_high: false,
            int_is_data_ready_high: false,
            stdev: State::new(),
            variant: Variant::L8,
        }
    }
}

impl<SPI, D> Sensor<Spi<SPI>, D, NoPin, NoPin>
where
    SPI: embedded_hal_async::spi::SpiDevice,
    D: DelayNs,
{
    pub fn new_spi(spi: SPI, delay: D) -> Self {
        Self {
            bus: Spi { spi },
            delay,
            address: DEFAULT_ADDRESS,
            reset_pin: NoPin,
            int_pin: NoPin,
            lp_is_reset_high: false,
            int_is_data_ready_high: false,
            stdev: State::new(),
            variant: Variant::L8,
        }
    }
}

// --- Builder methods ---

impl<BUS, D, RST, INT> Sensor<BUS, D, RST, INT> {
    /// Attach at `address` instead of [`DEFAULT_ADDRESS`], without touching the
    /// sensor. For re-attaching a handle to a sensor previously moved by
    /// [`Sensor::set_i2c_address`]; pair with [`Sensor::redetect`] to skip
    /// the firmware re-upload.
    pub fn with_address(mut self, address: SevenBitAddress) -> Self {
        self.address = address;
        self
    }

    /// Attach the sensor's **LPn** pin, or any reset line where low means reset.
    ///
    /// LPn is the part's own low-power / reset input and is active low: holding it
    /// low keeps the sensor in reset, high lets it run. [`reset`](Self::reset)
    /// pulses it low for 10 ms. With several sensors on one bus, LPn is also how
    /// the others are kept quiet while one sensor's I2C address is changed.
    pub fn with_reset_pin_active_low<RstNew>(
        self,
        reset_pin: RstNew,
    ) -> Sensor<BUS, D, RstNew, INT> {
        Sensor {
            bus: self.bus,
            delay: self.delay,
            address: self.address,
            reset_pin,
            int_pin: self.int_pin,
            lp_is_reset_high: false,
            int_is_data_ready_high: self.int_is_data_ready_high,
            stdev: self.stdev,
            variant: self.variant,
        }
    }

    /// Attach a reset line where **high** means reset.
    ///
    /// The sensor's LPn pin itself is active low; use this variant when the
    /// signal is inverted on its way to the sensor (level shifter, transistor),
    /// or when the board's reset line is the sensor's I2C_RST input, which is
    /// active high. Either way [`reset`](Self::reset) drives the pin high for
    /// 10 ms and then releases it.
    pub fn with_reset_pin_active_high<RstNew>(
        self,
        reset_pin: RstNew,
    ) -> Sensor<BUS, D, RstNew, INT> {
        Sensor {
            bus: self.bus,
            delay: self.delay,
            address: self.address,
            reset_pin,
            int_pin: self.int_pin,
            lp_is_reset_high: true,
            int_is_data_ready_high: self.int_is_data_ready_high,
            stdev: self.stdev,
            variant: self.variant,
        }
    }

    /// Attach the sensor's INT (data-ready) pin, which goes **low** when a frame
    /// is ready. This is the part's default: INT is an open-drain output pulled
    /// low for the interrupt pulse.
    pub fn with_interrupt_pin_active_low<IntNew>(
        self,
        int_pin: IntNew,
    ) -> Sensor<BUS, D, RST, IntNew> {
        Sensor {
            bus: self.bus,
            delay: self.delay,
            address: self.address,
            reset_pin: self.reset_pin,
            int_pin,
            lp_is_reset_high: self.lp_is_reset_high,
            int_is_data_ready_high: false,
            stdev: self.stdev,
            variant: self.variant,
        }
    }

    /// Attach a data-ready line that goes **high** when a frame is ready, for
    /// boards that invert the sensor's INT output.
    pub fn with_interrupt_pin_active_high<IntNew>(
        self,
        int_pin: IntNew,
    ) -> Sensor<BUS, D, RST, IntNew> {
        Sensor {
            bus: self.bus,
            delay: self.delay,
            address: self.address,
            reset_pin: self.reset_pin,
            int_pin,
            lp_is_reset_high: self.lp_is_reset_high,
            int_is_data_ready_high: true,
            stdev: self.stdev,
            variant: self.variant,
        }
    }
}

// --- Reset methods (requires OutputPin) ---

impl<BUS, D, RST, INT> Sensor<BUS, D, RST, INT>
where
    BUS: VlBusAsync,
    D: DelayNs,
    RST: OutputPin,
    <RST as embedded_hal::digital::ErrorType>::Error: core::fmt::Debug,
{
    fn set_reset_state(&mut self, target_state_reset: bool) -> Result<(), Error<BUS::Error>> {
        let set_pin_high = target_state_reset == self.lp_is_reset_high;
        let result = if set_pin_high {
            self.reset_pin.set_high()
        } else {
            self.reset_pin.set_low()
        };
        result.map_err(|_| Error::Pin)
    }

    /// Pulse the configured reset pin: assert for 10 ms, then release.
    ///
    /// The polarity comes from [`with_reset_pin_active_low`](Self::with_reset_pin_active_low)
    /// or [`with_reset_pin_active_high`](Self::with_reset_pin_active_high). ST's
    /// ULD has no equivalent; it leaves LPn and power to the application. A reset
    /// discards the uploaded firmware, so call [`init`](Self::init) afterwards.
    pub async fn reset(&mut self) -> Result<(), Error<BUS::Error>> {
        self.set_reset_state(true)?;
        self.delay.delay_ms(10).await;
        self.set_reset_state(false)?;
        Ok(())
    }
}

// --- Interrupt-based data ready (requires Wait) ---

impl<BUS, D, RST, INT> Sensor<BUS, D, RST, INT>
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

impl<BUS, D, RST, INT> Sensor<BUS, D, RST, INT>
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

impl<BUS, D, RST, INT> Sensor<BUS, D, RST, INT>
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

impl<BUS, D, RST, INT> Sensor<BUS, D, RST, INT>
where
    BUS: VlBusAsync,
    D: DelayNs,
{
    /// Detect the connected sensor variant, then upload its (patched) firmware
    /// and bring the sensor up. The detected variant is stored and available
    /// via [`Sensor::variant`].
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

    pub async fn set_i2c_address(&mut self, new_address: SevenBitAddress) -> Result<(), Error<BUS::Error>> {
        ops::set_i2c_address(&mut self.bus, self.address, new_address).await?;
        self.address = new_address;
        Ok(())
    }

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

    pub async fn stop_ranging(&mut self) -> Result<(), Error<BUS::Error>> {
        ops::stop_ranging(&mut self.bus, self.address, &mut self.delay, &mut self.stdev).await
    }

    pub async fn check_data_ready(&mut self) -> Result<bool, Error<BUS::Error>> {
        ops::check_data_ready(&mut self.bus, self.address, &mut self.stdev).await
    }

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
    pub async fn set_resolution(&mut self, resolution: Resolution) -> Result<(), Error<BUS::Error>> {
        ops::set_resolution(
            &mut self.bus, self.address, &mut self.delay, &mut self.stdev, resolution as u8,
        ).await
    }

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

    pub async fn set_ranging_frequency_hz(&mut self, frequency_hz: u8) -> Result<(), Error<BUS::Error>> {
        let mut data = [0u8; 4];
        ops::dci_replace(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut data,
            consts::VL53L8CX_DCI_FREQ_HZ as u32, 4,
            &[frequency_hz], 0x01,
        ).await
    }

    pub async fn get_ranging_frequency_hz(&mut self) -> Result<u8, Error<BUS::Error>> {
        let mut buf = [0u8; 4];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_FREQ_HZ as u32, 4,
        ).await?;
        Ok(buf[0x01])
    }

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

    pub async fn get_sharpener_percent(&mut self) -> Result<u8, Error<BUS::Error>> {
        let mut buf = [0u8; 16];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_SHARPENER as u32, 16,
        ).await?;
        Ok((buf[0x0D] as u16 * 100 / 255) as u8)
    }

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
impl<BUS, D, RST, INT> Sensor<BUS, D, RST, INT>
where
    BUS: VlBusAsync,
    D: DelayNs,
{
    pub async fn get_detection_thresholds_enable(&mut self) -> Result<bool, Error<BUS::Error>> {
        let mut buf = [0u8; 8];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            consts::VL53L8CX_DCI_DET_THRESH_GLOBAL_CONFIG as u32, 8,
        ).await?;
        Ok(buf[0x01] != 0)
    }

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
impl<BUS, D, RST, INT> Sensor<BUS, D, RST, INT>
where
    BUS: VlBusAsync,
    D: DelayNs,
{
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

    pub async fn motion_indicator_set_distance_motion(
        &mut self,
        motion_config: &mut MotionConfiguration,
        distance_min_mm: u16,
        distance_max_mm: u16,
    ) -> Result<(), Error<BUS::Error>> {
        if (distance_max_mm - distance_min_mm) > 1500
            || distance_min_mm < 400
            || distance_max_mm > 4000
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
impl<BUS, D, RST, INT> Sensor<BUS, D, RST, INT>
where
    BUS: VlBusAsync,
    D: DelayNs,
{
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
