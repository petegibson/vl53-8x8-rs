//! Mock bus, delay and pins. Every transfer is decoded to a register [`Op`]
//! and appended to a shared transcript; reads are answered by the model.

use std::cell::RefCell;
use std::convert::Infallible;
use std::future::Future;
use std::pin::pin;
use std::rc::Rc;
use std::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use embedded_hal::i2c::{self, SevenBitAddress};
use embedded_hal::spi;

use crate::{Dir, Op, RegisterModel, Transcript};

/// State shared between a mock bus and the test that owns it: the register
/// model answering reads, the recorded transcript and the delay total.
#[derive(Default)]
pub struct SharedState {
    pub model: RegisterModel,
    pub transcript: Transcript,
    pub delay_ns: u64,
    /// I2C 7-bit addresses seen on the mock I2C bus (SPI leaves it empty).
    pub i2c_addresses: Vec<SevenBitAddress>,
}

pub type Shared = Rc<RefCell<SharedState>>;

pub fn shared(model: RegisterModel) -> Shared {
    Rc::new(RefCell::new(SharedState { model, ..Default::default() }))
}

impl SharedState {
    fn write(&mut self, reg: u16, data: &[u8]) {
        self.model.note_write(reg, data);
        self.transcript.push(Op::write(reg, data));
    }
    fn read(&mut self, reg: u16, buf: &mut [u8]) {
        self.model.read(reg, buf);
        self.transcript.push(Op { dir: Dir::R, reg, data: buf.to_vec(), err: false });
    }
}

// --- SPI -------------------------------------------------------------------

/// Blocking `SpiDevice` speaking the sensor's SPI register protocol: a 16-bit
/// big-endian address with bit 15 set for writes, then the payload, inside one
/// chip-select transaction.
#[derive(Clone)]
pub struct MockSpi(pub Shared);

impl spi::ErrorType for MockSpi {
    type Error = Infallible;
}

impl spi::SpiDevice for MockSpi {
    fn transaction(&mut self, operations: &mut [spi::Operation<'_, u8>]) -> Result<(), Infallible> {
        let mut st = self.0.borrow_mut();
        let mut header: Option<u16> = None;
        let mut payload: Vec<u8> = Vec::new();
        let mut read_done = false;
        for op in operations.iter_mut() {
            match op {
                spi::Operation::Write(w) => {
                    if header.is_none() {
                        assert!(w.len() >= 2, "SPI transaction must start with a 2-byte address, got {w:?}");
                        header = Some(u16::from_be_bytes([w[0], w[1]]));
                        payload.extend_from_slice(&w[2..]);
                    } else {
                        payload.extend_from_slice(w);
                    }
                }
                spi::Operation::Read(r) => {
                    let addr = header.expect("SPI read before the address bytes");
                    assert!(addr & 0x8000 == 0, "SPI read with the write bit set: {addr:#06x}");
                    assert!(payload.is_empty(), "SPI read after payload bytes");
                    st.read(addr & 0x7FFF, r);
                    read_done = true;
                }
                spi::Operation::Transfer(..) | spi::Operation::TransferInPlace(..) => {
                    panic!("drivers use Write/Read operations only")
                }
                spi::Operation::DelayNs(ns) => st.delay_ns += u64::from(*ns),
            }
        }
        if !read_done {
            let addr = header.expect("empty SPI transaction");
            assert!(addr & 0x8000 != 0, "SPI write without the write bit: {addr:#06x}");
            st.write(addr & 0x7FFF, &payload);
        }
        Ok(())
    }
}

// --- I2C -------------------------------------------------------------------

/// Blocking `I2c` speaking the sensor's I2C register protocol: a 16-bit
/// big-endian register index followed by the payload (write), or followed by a
/// repeated-start read.
#[derive(Clone)]
pub struct MockI2c(pub Shared);

impl i2c::ErrorType for MockI2c {
    type Error = Infallible;
}

impl i2c::I2c<SevenBitAddress> for MockI2c {
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [i2c::Operation<'_>],
    ) -> Result<(), Infallible> {
        let mut st = self.0.borrow_mut();
        st.i2c_addresses.push(address);
        let mut written: Vec<u8> = Vec::new();
        let mut read_done = false;
        for op in operations.iter_mut() {
            match op {
                i2c::Operation::Write(w) => {
                    assert!(!read_done, "I2C write after read in one transaction");
                    written.extend_from_slice(w);
                }
                i2c::Operation::Read(r) => {
                    assert!(written.len() == 2, "I2C read must follow exactly the 2 index bytes, got {written:?}");
                    let reg = u16::from_be_bytes([written[0], written[1]]);
                    st.read(reg, r);
                    read_done = true;
                }
            }
        }
        if !read_done {
            assert!(written.len() >= 2, "I2C write shorter than the 2 index bytes: {written:?}");
            let reg = u16::from_be_bytes([written[0], written[1]]);
            st.write(reg, &written[2..]);
        }
        Ok(())
    }
}

// --- delay and pins ----------------------------------------------------------

/// No-op delay that totals the time requested, for both the blocking and the
/// async `DelayNs`. The total is reported by the tests, not asserted.
#[derive(Clone)]
pub struct MockDelay(pub Shared);

impl embedded_hal::delay::DelayNs for MockDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.borrow_mut().delay_ns += u64::from(ns);
    }
}

impl embedded_hal_async::delay::DelayNs for MockDelay {
    async fn delay_ns(&mut self, ns: u32) {
        self.0.borrow_mut().delay_ns += u64::from(ns);
    }
}

/// Output pin that swallows writes; input pin that always reads low (the
/// active-low INT line asserted, i.e. "data ready").
#[derive(Clone, Copy, Debug, Default)]
pub struct MockPin;

impl embedded_hal::digital::ErrorType for MockPin {
    type Error = Infallible;
}

impl embedded_hal::digital::OutputPin for MockPin {
    fn set_low(&mut self) -> Result<(), Infallible> {
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Infallible> {
        Ok(())
    }
}

impl embedded_hal::digital::InputPin for MockPin {
    fn is_high(&mut self) -> Result<bool, Infallible> {
        Ok(false)
    }
    fn is_low(&mut self) -> Result<bool, Infallible> {
        Ok(true)
    }
}

// --- block_on ---------------------------------------------------------------

/// Drive a future to completion with a no-op waker. The mock bus and delay
/// never pend, so the loop runs exactly once per `Poll::Pending`-free future;
/// a future that did pend would spin, which is acceptable for test code.
pub fn block_on<F: Future>(fut: F) -> F::Output {
    fn noop(_: *const ()) {}
    fn clone(p: *const ()) -> RawWaker {
        RawWaker::new(p, &VTABLE)
    }
    static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, noop, noop, noop);
    let waker = unsafe { Waker::from_raw(RawWaker::new(std::ptr::null(), &VTABLE)) };
    let mut cx = Context::from_waker(&waker);
    let mut fut = pin!(fut);
    loop {
        if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
            return v;
        }
    }
}
