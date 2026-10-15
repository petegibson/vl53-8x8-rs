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

/// Wrapper that implements `VlBusAsync` over a blocking `embedded_hal::spi::SpiDevice`.
///
/// The SPI counterpart of [`BlockingI2c`]: it speaks exactly the same register
/// protocol as the async [`Spi`] adapter (16-bit register address, MSB first,
/// bit 15 set for writes, followed by the payload in one chip-select
/// transaction), but drives a blocking `SpiDevice`. Each transfer blocks the
/// executor; the driver's async delays still yield correctly, so other tasks
/// run between transfers.
///
/// Use it where no async SPI implementation is available or wanted: blocking
/// HALs such as `stm32f4xx-hal`, RTIC 2 applications (the monotonic provides
/// the async `DelayNs`), or any executor-less `block_on` setup. Chip-select
/// handling comes from the `SpiDevice`, e.g. `embedded_hal_bus::spi::ExclusiveDevice`:
/// ```ignore
/// let dev = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, cs).unwrap();
/// let sensor = Sensor::new(BlockingSpi(dev), delay);
/// ```
pub struct BlockingSpi<SPI>(pub SPI);

impl<SPI: embedded_hal::spi::SpiDevice> VlBusAsync for BlockingSpi<SPI> {
    type Error = <SPI as embedded_hal::spi::ErrorType>::Error;

    async fn write(&mut self, _addr: SevenBitAddress, reg: u16, data: &[u8]) -> Result<(), Self::Error> {
        #[cfg(feature = "defmt")]
        {
            defmt::trace!("spi W reg={:#06x} len={}", reg, data.len());
            trace_data(data);
        }
        let addr_bytes = (reg | 0x8000).to_be_bytes();
        embedded_hal::spi::SpiDevice::transaction(
            &mut self.0,
            &mut [
                embedded_hal::spi::Operation::Write(&addr_bytes),
                embedded_hal::spi::Operation::Write(data),
            ],
        )
    }

    async fn read(&mut self, _addr: SevenBitAddress, reg: u16, buf: &mut [u8]) -> Result<(), Self::Error> {
        let addr_bytes = (reg & 0x7FFF).to_be_bytes();
        let result = embedded_hal::spi::SpiDevice::transaction(
            &mut self.0,
            &mut [
                embedded_hal::spi::Operation::Write(&addr_bytes),
                embedded_hal::spi::Operation::Read(buf),
            ],
        );
        #[cfg(feature = "defmt")]
        {
            defmt::trace!("spi R reg={:#06x} len={}", reg, buf.len());
            trace_data(buf);
        }
        result
    }
}

#[cfg(test)]
mod tests {
    //! The blocking adapters must put exactly the same bytes on the wire as
    //! their async twins: a mock device records every transaction (the
    //! operations inside one chip-select) for both trait flavours.
    use super::*;
    use core::convert::Infallible;
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    /// Drive a future that never actually pends (the adapters are synchronous
    /// under the hood) to completion.
    fn block_on<F: Future>(fut: F) -> F::Output {
        fn noop(_: *const ()) {}
        fn clone(p: *const ()) -> RawWaker {
            RawWaker::new(p, &VTABLE)
        }
        static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, noop, noop, noop);
        let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
        let mut cx = Context::from_waker(&waker);
        let mut fut = pin!(fut);
        loop {
            if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
                return v;
            }
        }
    }

    /// One SPI operation as seen by the mock: bytes written, or a read of `n`
    /// bytes (filled with a counting pattern so the caller can check it too).
    #[derive(Debug, PartialEq, Eq, Clone)]
    enum Op {
        Write(Vec<u8>),
        Read(usize),
    }

    #[derive(Default)]
    struct MockSpi {
        /// Transactions, each a list of operations between CS assert/release.
        transactions: Vec<Vec<Op>>,
    }

    impl MockSpi {
        fn record(&mut self, ops: &mut [embedded_hal::spi::Operation<'_, u8>]) {
            let mut tx = Vec::new();
            for op in ops.iter_mut() {
                match op {
                    embedded_hal::spi::Operation::Write(w) => tx.push(Op::Write(w.to_vec())),
                    embedded_hal::spi::Operation::Read(r) => {
                        for (i, b) in r.iter_mut().enumerate() {
                            *b = i as u8;
                        }
                        tx.push(Op::Read(r.len()));
                    }
                    _ => panic!("adapters only use Write and Read operations"),
                }
            }
            self.transactions.push(tx);
        }
    }

    impl embedded_hal::spi::ErrorType for MockSpi {
        type Error = Infallible;
    }

    impl embedded_hal::spi::SpiDevice for MockSpi {
        fn transaction(
            &mut self,
            operations: &mut [embedded_hal::spi::Operation<'_, u8>],
        ) -> Result<(), Infallible> {
            self.record(operations);
            Ok(())
        }
    }

    impl embedded_hal_async::spi::SpiDevice for MockSpi {
        async fn transaction(
            &mut self,
            operations: &mut [embedded_hal::spi::Operation<'_, u8>],
        ) -> Result<(), Infallible> {
            self.record(operations);
            Ok(())
        }
    }

    const ADDR: SevenBitAddress = 0x29;

    /// Run the same write + read script through a bus and return the recorded
    /// transactions plus the bytes the read returned.
    fn script<B: VlBusAsync<Error = Infallible>>(mut bus: B) -> (B, Vec<u8>) {
        let payload: Vec<u8> = (0..40).map(|i| 0xA0 ^ i as u8).collect();
        let mut rx = [0u8; 12];
        block_on(async {
            bus.write(ADDR, 0x7FFF, &payload).await.unwrap();
            bus.write_byte(ADDR, 0x0001, 0x5A).await.unwrap();
            bus.read(ADDR, 0x8123, &mut rx).await.unwrap();
            assert_eq!(bus.read_byte(ADDR, 0x0010).await.unwrap(), 0);
        });
        (bus, rx.to_vec())
    }

    #[test]
    fn blocking_spi_encodes_register_protocol() {
        let (bus, _) = script(BlockingSpi(MockSpi::default()));
        let t = &bus.0.transactions;
        assert_eq!(t.len(), 4);
        // Write: bit 15 set, MSB first, payload in the same transaction.
        assert_eq!(t[0][0], Op::Write(vec![0xFF, 0xFF]));
        assert_eq!(t[0][1], Op::Write((0..40).map(|i| 0xA0 ^ i as u8).collect()));
        assert_eq!(t[1], vec![Op::Write(vec![0x80, 0x01]), Op::Write(vec![0x5A])]);
        // Read: bit 15 cleared (even if the caller passed it set), then the read.
        assert_eq!(t[2], vec![Op::Write(vec![0x01, 0x23]), Op::Read(12)]);
        assert_eq!(t[3], vec![Op::Write(vec![0x00, 0x10]), Op::Read(1)]);
    }

    #[test]
    fn blocking_spi_matches_async_spi_byte_for_byte() {
        let (blocking, rx_blocking) = script(BlockingSpi(MockSpi::default()));
        let (asynchronous, rx_async) = script(Spi { spi: MockSpi::default() });
        assert_eq!(blocking.0.transactions, asynchronous.spi.transactions);
        assert_eq!(rx_blocking, rx_async);
        assert_eq!(rx_blocking, (0..12).collect::<Vec<u8>>());
    }
}
