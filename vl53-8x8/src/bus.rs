use embedded_hal::i2c::SevenBitAddress;
use embedded_hal_async::i2c::I2c;
use embedded_hal_async::spi::SpiDevice;

/// Log data in 16-byte chunks when the `defmt` feature is enabled.
#[cfg(feature = "defmt")]
fn trace_data(data: &[u8]) {
    for chunk in data.chunks(16) {
        defmt::trace!("  {=[u8]:#04x}", chunk);
    }
}

/// Async bus abstraction for talking to a VL53L5CX/VL53L8CX sensor.
pub trait VlBusAsync {
    type Error: core::fmt::Debug;

    async fn write(&mut self, addr: SevenBitAddress, reg: u16, data: &[u8]) -> Result<(), Self::Error>;
    async fn read(&mut self, addr: SevenBitAddress, reg: u16, buf: &mut [u8]) -> Result<(), Self::Error>;
    async fn write_byte(&mut self, addr: SevenBitAddress, reg: u16, value: u8) -> Result<(), Self::Error> {
        self.write(addr, reg, &[value]).await
    }
    async fn read_byte(&mut self, addr: SevenBitAddress, reg: u16) -> Result<u8, Self::Error> {
        let mut buf = [0u8; 1];
        self.read(addr, reg, &mut buf).await?;
        Ok(buf[0])
    }
}

impl<T: I2c> VlBusAsync for T {
    type Error = <T as embedded_hal::i2c::ErrorType>::Error;

    async fn write(&mut self, addr: SevenBitAddress, reg: u16, data: &[u8]) -> Result<(), Self::Error> {
        #[cfg(feature = "defmt")]
        {
            defmt::trace!("i2c W [{:#04x}] reg={:#06x} len={}", addr, reg, data.len());
            trace_data(data);
        }
        // Use transaction with Write ops: per embedded-hal spec, consecutive
        // writes are concatenated without RESTART (reg addr + data in one frame).
        let reg_bytes = reg.to_be_bytes();
        let result = if data.len() <= 32 {
            // Small writes: concatenate into a single buffer to avoid
            // multi-operation transaction issues on some I2C peripherals.
            let mut buf = [0u8; 34]; // 2 reg + up to 32 data
            buf[..2].copy_from_slice(&reg_bytes);
            buf[2..2 + data.len()].copy_from_slice(data);
            I2c::write(self, addr, &buf[..2 + data.len()]).await
        } else {
            let mut ops = [
                embedded_hal::i2c::Operation::Write(&reg_bytes),
                embedded_hal::i2c::Operation::Write(data),
            ];
            I2c::transaction(self, addr, &mut ops).await
        };
        #[cfg(feature = "defmt")]
        if result.is_err() {
            defmt::trace!("i2c W -> ERR");
        }
        result
    }

    async fn read(&mut self, addr: SevenBitAddress, reg: u16, buf: &mut [u8]) -> Result<(), Self::Error> {
        let reg_bytes = reg.to_be_bytes();
        let result = I2c::write_read(self, addr, &reg_bytes, buf).await;
        #[cfg(feature = "defmt")]
        if result.is_ok() {
            defmt::trace!("i2c R [{:#04x}] reg={:#06x} len={}", addr, reg, buf.len());
            trace_data(buf);
        } else {
            defmt::trace!("i2c R [{:#04x}] reg={:#06x} -> ERR", addr, reg);
        }
        result
    }
}

/// Wrapper that implements `VlBusAsync` using blocking `embedded_hal::i2c::I2c`.
///
/// Useful on platforms where the async I2C DMA driver is unreliable
/// (e.g. STM32F4/F1/F2/L1 with I2C v1). I2C transfers will block the
/// executor, but async delays in the driver still yield correctly.
///
/// On embassy-stm32, the async `I2c` peripheral also implements the blocking
/// trait, so you can wrap it directly:
/// ```ignore
/// let i2c = embassy_stm32::i2c::I2c::new(/* async params */);
/// let sensor = Sensor::new(BlockingI2c(i2c), Delay);
/// ```
pub struct BlockingI2c<T>(pub T);

impl<T: embedded_hal::i2c::I2c> VlBusAsync for BlockingI2c<T> {
    type Error = <T as embedded_hal::i2c::ErrorType>::Error;

    async fn write(&mut self, addr: SevenBitAddress, reg: u16, data: &[u8]) -> Result<(), Self::Error> {
        #[cfg(feature = "defmt")]
        {
            defmt::trace!("i2c W [{:#04x}] reg={:#06x} len={}", addr, reg, data.len());
            trace_data(data);
        }
        let reg_bytes = reg.to_be_bytes();
        let result = if data.len() <= 32 {
            let mut buf = [0u8; 34];
            buf[..2].copy_from_slice(&reg_bytes);
            buf[2..2 + data.len()].copy_from_slice(data);
            embedded_hal::i2c::I2c::write(&mut self.0, addr, &buf[..2 + data.len()])
        } else {
            let mut ops = [
                embedded_hal::i2c::Operation::Write(&reg_bytes),
                embedded_hal::i2c::Operation::Write(data),
            ];
            embedded_hal::i2c::I2c::transaction(&mut self.0, addr, &mut ops)
        };
        #[cfg(feature = "defmt")]
        if result.is_err() {
            defmt::trace!("i2c W -> ERR");
        }
        result
    }

    async fn read(&mut self, addr: SevenBitAddress, reg: u16, buf: &mut [u8]) -> Result<(), Self::Error> {
        let reg_bytes = reg.to_be_bytes();
        let result = embedded_hal::i2c::I2c::write_read(&mut self.0, addr, &reg_bytes, buf);
        #[cfg(feature = "defmt")]
        if result.is_ok() {
            defmt::trace!("i2c R [{:#04x}] reg={:#06x} len={}", addr, reg, buf.len());
            trace_data(buf);
        } else {
            defmt::trace!("i2c R [{:#04x}] reg={:#06x} -> ERR", addr, reg);
        }
        result
    }
}

/// Wrapper around an async `SpiDevice` implementing the sensor's SPI register protocol.
pub struct Spi<SPI> {
    pub spi: SPI,
}

impl<SPI: SpiDevice> VlBusAsync for Spi<SPI> {
    type Error = <SPI as embedded_hal::spi::ErrorType>::Error;

    async fn write(&mut self, _addr: SevenBitAddress, reg: u16, data: &[u8]) -> Result<(), Self::Error> {
        #[cfg(feature = "defmt")]
        {
            defmt::trace!("spi W reg={:#06x} len={}", reg, data.len());
            trace_data(data);
        }
        let addr_bytes = (reg | 0x8000).to_be_bytes();
        self.spi
            .transaction(&mut [
                embedded_hal::spi::Operation::Write(&addr_bytes),
                embedded_hal::spi::Operation::Write(data),
            ])
            .await
    }

    async fn read(&mut self, _addr: SevenBitAddress, reg: u16, buf: &mut [u8]) -> Result<(), Self::Error> {
        let addr_bytes = (reg & 0x7FFF).to_be_bytes();
        let result = self.spi
            .transaction(&mut [
                embedded_hal::spi::Operation::Write(&addr_bytes),
                embedded_hal::spi::Operation::Read(buf),
            ])
            .await;
        #[cfg(feature = "defmt")]
        {
            defmt::trace!("spi R reg={:#06x} len={}", reg, buf.len());
            trace_data(buf);
        }
        result
    }
}
