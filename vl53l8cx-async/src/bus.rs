use embedded_hal_async::i2c::I2c;
use embedded_hal_async::spi::SpiDevice;

/// Async bus abstraction for VL53L8CX communication.
pub trait VlBusAsync {
    type Error: core::fmt::Debug;

    async fn write(&mut self, addr: u8, reg: u16, data: &[u8]) -> Result<(), Self::Error>;
    async fn read(&mut self, addr: u8, reg: u16, buf: &mut [u8]) -> Result<(), Self::Error>;
    async fn write_byte(&mut self, addr: u8, reg: u16, value: u8) -> Result<(), Self::Error> {
        self.write(addr, reg, &[value]).await
    }
    async fn read_byte(&mut self, addr: u8, reg: u16) -> Result<u8, Self::Error> {
        let mut buf = [0u8; 1];
        self.read(addr, reg, &mut buf).await?;
        Ok(buf[0])
    }
}

impl<T: I2c> VlBusAsync for T {
    type Error = <T as embedded_hal::i2c::ErrorType>::Error;

    async fn write(&mut self, addr: u8, reg: u16, data: &[u8]) -> Result<(), Self::Error> {
        let reg_bytes = reg.to_be_bytes();
        let mut ops = [
            embedded_hal::i2c::Operation::Write(&reg_bytes),
            embedded_hal::i2c::Operation::Write(data),
        ];
        I2c::transaction(self, addr, &mut ops).await
    }

    async fn read(&mut self, addr: u8, reg: u16, buf: &mut [u8]) -> Result<(), Self::Error> {
        let reg_bytes = reg.to_be_bytes();
        I2c::write_read(self, addr, &reg_bytes, buf).await
    }
}

/// Wrapper around an async `SpiDevice` for VL53L8CX communication.
pub struct Vl53l8cxSpiDevice<SPI> {
    pub spi: SPI,
}

impl<SPI: SpiDevice> VlBusAsync for Vl53l8cxSpiDevice<SPI> {
    type Error = <SPI as embedded_hal::spi::ErrorType>::Error;

    async fn write(&mut self, _addr: u8, reg: u16, data: &[u8]) -> Result<(), Self::Error> {
        let addr_bytes = (reg | 0x8000).to_be_bytes();
        self.spi
            .transaction(&mut [
                embedded_hal::spi::Operation::Write(&addr_bytes),
                embedded_hal::spi::Operation::Write(data),
            ])
            .await
    }

    async fn read(&mut self, _addr: u8, reg: u16, buf: &mut [u8]) -> Result<(), Self::Error> {
        let addr_bytes = (reg & 0x7FFF).to_be_bytes();
        self.spi
            .transaction(&mut [
                embedded_hal::spi::Operation::Write(&addr_bytes),
                embedded_hal::spi::Operation::Read(buf),
            ])
            .await
    }
}
