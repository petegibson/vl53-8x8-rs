#![no_std]
/*!
An async, platform-agnostic driver for the VL53L8CX Time-of-Flight sensor.

This driver uses `embedded-hal-async` traits and reimplements all I/O in pure Rust —
no C FFI calls at runtime. It uses `vl53l8cx-sys` for shared types, constants, and
data blobs (firmware, configuration, xtalk tables).
*/

pub mod bus;
pub mod ops;

use bus::VlBusAsync;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use vl53l8cx_sys as sys;

// Re-export shared types from sys
pub use sys::{
    Error, FirmwareError, NoPin, PowerMode, RangingMode, Resolution, TargetOrder,
    VL53L8CX_Configuration, VL53L8CX_ResultsData as ResultsData, DEFAULT_ADDRESS,
    XTALK_BUFFER_SIZE,
};

#[cfg(feature = "thresholds")]
pub use sys::{DetectionThreshold, ThresholdMeasurement, ThresholdOperation, ThresholdType};

pub use bus::{BlockingI2c, Vl53l8cxSpiDevice};

pub struct Vl53l8cx<BUS, D, RST = NoPin, INT = NoPin> {
    bus: BUS,
    delay: D,
    address: u8,
    reset_pin: RST,
    int_pin: INT,
    lp_is_reset_high: bool,
    int_is_data_ready_high: bool,
    stdev: VL53L8CX_Configuration,
}

// --- Constructors ---

impl<BUS, D> Vl53l8cx<BUS, D, NoPin, NoPin>
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
            stdev: unsafe { core::mem::zeroed() },
        }
    }
}

// --- Builder methods ---

impl<BUS, D, RST, INT> Vl53l8cx<BUS, D, RST, INT> {
    pub fn with_reset_pin_active_low<RstNew>(
        self,
        reset_pin: RstNew,
    ) -> Vl53l8cx<BUS, D, RstNew, INT> {
        Vl53l8cx {
            bus: self.bus,
            delay: self.delay,
            address: self.address,
            reset_pin,
            int_pin: self.int_pin,
            lp_is_reset_high: false,
            int_is_data_ready_high: self.int_is_data_ready_high,
            stdev: self.stdev,
        }
    }

    pub fn with_reset_pin_active_high<RstNew>(
        self,
        reset_pin: RstNew,
    ) -> Vl53l8cx<BUS, D, RstNew, INT> {
        Vl53l8cx {
            bus: self.bus,
            delay: self.delay,
            address: self.address,
            reset_pin,
            int_pin: self.int_pin,
            lp_is_reset_high: true,
            int_is_data_ready_high: self.int_is_data_ready_high,
            stdev: self.stdev,
        }
    }

    pub fn with_interrupt_pin_active_low<IntNew>(
        self,
        int_pin: IntNew,
    ) -> Vl53l8cx<BUS, D, RST, IntNew> {
        Vl53l8cx {
            bus: self.bus,
            delay: self.delay,
            address: self.address,
            reset_pin: self.reset_pin,
            int_pin,
            lp_is_reset_high: self.lp_is_reset_high,
            int_is_data_ready_high: false,
            stdev: self.stdev,
        }
    }

    pub fn with_interrupt_pin_active_high<IntNew>(
        self,
        int_pin: IntNew,
    ) -> Vl53l8cx<BUS, D, RST, IntNew> {
        Vl53l8cx {
            bus: self.bus,
            delay: self.delay,
            address: self.address,
            reset_pin: self.reset_pin,
            int_pin,
            lp_is_reset_high: self.lp_is_reset_high,
            int_is_data_ready_high: true,
            stdev: self.stdev,
        }
    }
}

// --- Reset methods (requires OutputPin) ---

impl<BUS, D, RST, INT> Vl53l8cx<BUS, D, RST, INT>
where
    BUS: VlBusAsync,
    D: DelayNs,
    RST: OutputPin,
    <RST as embedded_hal::digital::ErrorType>::Error: core::fmt::Debug,
{
    fn set_reset_state(&mut self, target_state_reset: bool) -> Result<(), Error<BUS::Error>> {
        let set_pin_high = target_state_reset ^ self.lp_is_reset_high;
        let result = if set_pin_high {
            self.reset_pin.set_high()
        } else {
            self.reset_pin.set_low()
        };
        result.map_err(|_| Error::Pin)
    }

    pub async fn reset(&mut self) -> Result<(), Error<BUS::Error>> {
        self.set_reset_state(true)?;
        self.delay.delay_ms(10).await;
        self.set_reset_state(false)?;
        Ok(())
    }

    pub async fn set_address(&mut self, new_address: u8) -> Result<(), Error<BUS::Error>> {
        self.set_reset_state(true)?;
        self.delay.delay_ms(2).await;
        self.set_reset_state(false)?;
        self.delay.delay_ms(2).await;

        ops::set_i2c_address(&mut self.bus, self.address, new_address).await?;
        self.address = new_address;
        Ok(())
    }
}

// --- Interrupt-based data ready (requires Wait) ---

impl<BUS, D, RST, INT> Vl53l8cx<BUS, D, RST, INT>
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

impl<BUS, D, RST, INT> Vl53l8cx<BUS, D, RST, INT>
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

impl<BUS, D, RST, INT> Vl53l8cx<BUS, D, RST, INT>
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

impl<BUS, D, RST, INT> Vl53l8cx<BUS, D, RST, INT>
where
    BUS: VlBusAsync,
    D: DelayNs,
{
    pub async fn init(&mut self) -> Result<(), Error<BUS::Error>> {
        self.stdev.platform.address = self.address as u16;

        if !ops::is_alive(&mut self.bus, self.address).await? {
            return Err(Error::NotAlive);
        }

        ops::init_sensor(&mut self.bus, self.address, &mut self.delay, &mut self.stdev).await
    }

    pub async fn is_alive(&mut self) -> Result<bool, Error<BUS::Error>> {
        ops::is_alive(&mut self.bus, self.address).await
    }

    pub async fn start_ranging(&mut self) -> Result<(), Error<BUS::Error>> {
        ops::start_ranging(&mut self.bus, self.address, &mut self.delay, &mut self.stdev).await
    }

    pub async fn stop_ranging(&mut self) -> Result<(), Error<BUS::Error>> {
        ops::stop_ranging(&mut self.bus, self.address, &mut self.delay, &mut self.stdev).await
    }

    pub async fn check_data_ready(&mut self) -> Result<bool, Error<BUS::Error>> {
        ops::check_data_ready(&mut self.bus, self.address, &mut self.stdev).await
    }

    pub async fn get_ranging_data(&mut self) -> Result<ResultsData, Error<BUS::Error>> {
        ops::get_ranging_data(&mut self.bus, self.address, &mut self.stdev).await
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

    pub async fn set_resolution(&mut self, resolution: Resolution) -> Result<(), Error<BUS::Error>> {
        let res = resolution as u8;
        match resolution {
            Resolution::Res4x4 => {
                ops::dci_read(
                    &mut self.bus, self.address, &mut self.delay,
                    &mut self.stdev.temp_buffer, &mut self.stdev.offset_data[..16],
                    sys::VL53L8CX_DCI_DSS_CONFIG as u32, 16,
                ).await?;
                self.stdev.offset_data[0x04] = 64;
                self.stdev.offset_data[0x06] = 64;
                self.stdev.offset_data[0x09] = 4;
                ops::dci_write(
                    &mut self.bus, self.address, &mut self.delay,
                    &mut self.stdev.temp_buffer, &mut self.stdev.offset_data[..16],
                    sys::VL53L8CX_DCI_DSS_CONFIG as u32, 16,
                ).await?;

                ops::dci_read(
                    &mut self.bus, self.address, &mut self.delay,
                    &mut self.stdev.temp_buffer, &mut self.stdev.offset_data[..8],
                    sys::VL53L8CX_DCI_ZONE_CONFIG as u32, 8,
                ).await?;
                self.stdev.offset_data[0x00] = 4;
                self.stdev.offset_data[0x01] = 4;
                self.stdev.offset_data[0x04] = 8;
                self.stdev.offset_data[0x05] = 8;
                ops::dci_write(
                    &mut self.bus, self.address, &mut self.delay,
                    &mut self.stdev.temp_buffer, &mut self.stdev.offset_data[..8],
                    sys::VL53L8CX_DCI_ZONE_CONFIG as u32, 8,
                ).await?;
            }
            Resolution::Res8x8 => {
                ops::dci_read(
                    &mut self.bus, self.address, &mut self.delay,
                    &mut self.stdev.temp_buffer, &mut self.stdev.offset_data[..16],
                    sys::VL53L8CX_DCI_DSS_CONFIG as u32, 16,
                ).await?;
                self.stdev.offset_data[0x04] = 16;
                self.stdev.offset_data[0x06] = 16;
                self.stdev.offset_data[0x09] = 1;
                ops::dci_write(
                    &mut self.bus, self.address, &mut self.delay,
                    &mut self.stdev.temp_buffer, &mut self.stdev.offset_data[..16],
                    sys::VL53L8CX_DCI_DSS_CONFIG as u32, 16,
                ).await?;

                ops::dci_read(
                    &mut self.bus, self.address, &mut self.delay,
                    &mut self.stdev.temp_buffer, &mut self.stdev.offset_data[..8],
                    sys::VL53L8CX_DCI_ZONE_CONFIG as u32, 8,
                ).await?;
                self.stdev.offset_data[0x00] = 8;
                self.stdev.offset_data[0x01] = 8;
                self.stdev.offset_data[0x04] = 4;
                self.stdev.offset_data[0x05] = 4;
                ops::dci_write(
                    &mut self.bus, self.address, &mut self.delay,
                    &mut self.stdev.temp_buffer, &mut self.stdev.offset_data[..8],
                    sys::VL53L8CX_DCI_ZONE_CONFIG as u32, 8,
                ).await?;
            }
        }

        ops::send_offset_data(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &self.stdev.offset_data, res,
        ).await?;
        ops::send_xtalk_data(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &self.stdev.xtalk_data, res,
        ).await?;

        Ok(())
    }

    pub async fn get_resolution(&mut self) -> Result<Resolution, Error<BUS::Error>> {
        let mut buf = [0u8; 8];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            sys::VL53L8CX_DCI_ZONE_CONFIG as u32, 8,
        ).await?;
        let res = buf[0] * buf[1];
        if res == sys::VL53L8CX_RESOLUTION_8X8 {
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
            sys::VL53L8CX_DCI_FREQ_HZ as u32, 4,
            &[frequency_hz], 0x01,
        ).await
    }

    pub async fn get_ranging_frequency_hz(&mut self) -> Result<u8, Error<BUS::Error>> {
        let mut buf = [0u8; 4];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            sys::VL53L8CX_DCI_FREQ_HZ as u32, 4,
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
            sys::VL53L8CX_DCI_INT_TIME as u32, 20,
            &integration, 0x00,
        ).await
    }

    pub async fn get_integration_time_ms(&mut self) -> Result<u32, Error<BUS::Error>> {
        let mut buf = [0u8; 20];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            sys::VL53L8CX_DCI_INT_TIME as u32, 20,
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
            sys::VL53L8CX_DCI_TARGET_ORDER as u32, 4,
            &[val], 0x00,
        ).await
    }

    pub async fn get_target_order(&mut self) -> Result<TargetOrder, Error<BUS::Error>> {
        let mut buf = [0u8; 4];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            sys::VL53L8CX_DCI_TARGET_ORDER as u32, 4,
        ).await?;
        if buf[0] == sys::VL53L8CX_TARGET_ORDER_STRONGEST {
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
            sys::VL53L8CX_DCI_RANGING_MODE as u32, 8,
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
            sys::VL53L8CX_DCI_RANGING_MODE as u32, 8,
        ).await?;

        let mut sr_bytes = single_range.to_le_bytes();
        ops::dci_write(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut sr_bytes,
            sys::VL53L8CX_DCI_SINGLE_RANGE as u32, 4,
        ).await
    }

    pub async fn get_ranging_mode(&mut self) -> Result<RangingMode, Error<BUS::Error>> {
        let mut buf = [0u8; 8];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            sys::VL53L8CX_DCI_RANGING_MODE as u32, 8,
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
            sys::VL53L8CX_DCI_SHARPENER as u32, 16,
            &[sharpener], 0x0D,
        ).await
    }

    pub async fn get_sharpener_percent(&mut self) -> Result<u8, Error<BUS::Error>> {
        let mut buf = [0u8; 16];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            sys::VL53L8CX_DCI_SHARPENER as u32, 16,
        ).await?;
        Ok((buf[0x0D] as u16 * 100 / 255) as u8)
    }

    pub async fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), Error<BUS::Error>> {
        let current = self.get_power_mode().await?;
        if mode == current {
            return Ok(());
        }

        match mode {
            PowerMode::Wakeup => {
                self.bus.write_byte(self.address, 0x7FFF, 0x00).await.map_err(Error::Bus)?;
                self.bus.write_byte(self.address, 0x09, 0x04).await.map_err(Error::Bus)?;
                let stored = self.bus.read_byte(self.address, 0x000F).await.map_err(Error::Bus)?;
                if stored == 0x43 {
                    self.bus.write_byte(self.address, 0x000F, 0x40).await.map_err(Error::Bus)?;
                }
                ops::poll_for_answer(
                    &mut self.bus, self.address, &mut self.delay,
                    &mut self.stdev.temp_buffer, 1, 0, 0x06, 0x01, 1,
                ).await?;
                if stored == 0x43 {
                    // Re-init after deep sleep
                    ops::init_sensor(
                        &mut self.bus, self.address, &mut self.delay, &mut self.stdev,
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
            0x02 => {
                let mode_reg = self.bus.read_byte(self.address, 0x000F).await.map_err(Error::Bus)?;
                if mode_reg == 0x43 {
                    PowerMode::DeepSleep
                } else {
                    PowerMode::Sleep
                }
            }
            _ => {
                self.bus.write_byte(self.address, 0x7FFF, 0x02).await.map_err(Error::Bus)?;
                return Err(Error::Firmware(FirmwareError::Unknown(sys::VL53L8CX_STATUS_ERROR)));
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
            sys::VL53L8CX_DCI_VHV_CONFIG as u32, 16,
            &count_bytes, 0x04,
        ).await
    }

    pub async fn get_vhv_repeat_count(&mut self) -> Result<u32, Error<BUS::Error>> {
        let mut buf = [0u8; 16];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            sys::VL53L8CX_DCI_VHV_CONFIG as u32, 16,
        ).await?;
        Ok(u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]))
    }

    pub async fn set_external_sync_pin_enable(&mut self, enabled: bool) -> Result<(), Error<BUS::Error>> {
        let mut buf = [0u8; 4];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            sys::VL53L8CX_DCI_SYNC_PIN as u32, 4,
        ).await?;

        if enabled {
            buf[3] |= 0x02;
        } else {
            buf[3] &= !0x02;
        }

        ops::dci_write(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            sys::VL53L8CX_DCI_SYNC_PIN as u32, 4,
        ).await
    }

    pub async fn get_external_sync_pin_enable(&mut self) -> Result<bool, Error<BUS::Error>> {
        let mut buf = [0u8; 4];
        ops::dci_read(
            &mut self.bus, self.address, &mut self.delay,
            &mut self.stdev.temp_buffer, &mut buf,
            sys::VL53L8CX_DCI_SYNC_PIN as u32, 4,
        ).await?;
        Ok((buf[3] & 0x02) != 0)
    }
}
