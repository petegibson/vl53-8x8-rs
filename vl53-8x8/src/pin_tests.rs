//! Host unit tests for the pin handling: `power_cycle()` must drive the
//! power-enable and LPn pins in the UM2884 §4.2 order (with and without an LPn
//! pin attached), `pulse_i2c_rst` must pulse high, and the motion-indicator
//! distance check must reject `max < min` without panicking or touching the bus.

use std::cell::RefCell;
use std::convert::Infallible;
use std::future::Future;
use std::pin::pin;
use std::rc::Rc;
use std::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use crate::bus::VlBusAsync;
#[cfg(any(feature = "motion", feature = "thresholds"))]
use crate::{Error, FirmwareError, MotionConfiguration};
use crate::{NoPin, Sensor};
use embedded_hal::digital::{ErrorType, OutputPin};
use embedded_hal::i2c::SevenBitAddress;

/// Shared event log: pin transitions and delays, in the order they happened.
type Log = Rc<RefCell<Vec<String>>>;

/// Output pin that records every transition into the shared log under `name`.
struct LogPin {
    name: &'static str,
    log: Log,
}

impl ErrorType for LogPin {
    type Error = Infallible;
}

impl OutputPin for LogPin {
    fn set_low(&mut self) -> Result<(), Infallible> {
        self.log.borrow_mut().push(format!("{} low", self.name));
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Infallible> {
        self.log.borrow_mut().push(format!("{} high", self.name));
        Ok(())
    }
}

/// Delay that records the requested milliseconds into the shared log.
struct LogDelay(Log);

impl embedded_hal_async::delay::DelayNs for LogDelay {
    async fn delay_ns(&mut self, ns: u32) {
        self.0.borrow_mut().push(format!("delay {} ms", ns / 1_000_000));
    }
}

/// Bus that must never be touched: every transfer panics.
struct NoBus;

impl VlBusAsync for NoBus {
    type Error = Infallible;
    async fn write(&mut self, _: SevenBitAddress, reg: u16, _: &[u8]) -> Result<(), Infallible> {
        panic!("unexpected bus write to {reg:#06x}");
    }
    async fn read(&mut self, _: SevenBitAddress, reg: u16, _: &mut [u8]) -> Result<(), Infallible> {
        panic!("unexpected bus read from {reg:#06x}");
    }
}

/// Drive a future to completion; nothing here ever pends.
fn block_on<F: Future>(fut: F) -> F::Output {
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

fn log() -> Log {
    Rc::new(RefCell::new(Vec::new()))
}

fn pin(name: &'static str, log: &Log) -> LogPin {
    LogPin { name, log: log.clone() }
}

#[test]
fn power_cycle_with_lpn_pin_follows_um2884_order() {
    let log = log();
    let mut s = Sensor::new(NoBus, LogDelay(log.clone()))
        .with_lpn_pin(pin("lpn", &log))
        .with_power_enable_pin_active_high(pin("pwr", &log));
    block_on(s.power_cycle()).unwrap();
    assert_eq!(
        *log.borrow(),
        ["pwr low", "lpn low", "delay 10 ms", "pwr high", "lpn high", "delay 10 ms"]
    );
}

#[test]
fn power_cycle_without_lpn_pin_only_drives_power() {
    let log = log();
    let mut s = Sensor::new(NoBus, LogDelay(log.clone()))
        .with_power_enable_pin_active_high(pin("pwr", &log));
    block_on(s.power_cycle()).unwrap();
    assert_eq!(*log.borrow(), ["pwr low", "delay 10 ms", "pwr high", "delay 10 ms"]);
}

#[test]
fn power_cycle_honours_active_low_power_enable() {
    let log = log();
    let mut s = Sensor::new(NoBus, LogDelay(log.clone()))
        .with_lpn_pin(pin("lpn", &log))
        .with_power_enable_pin_active_low(pin("pwr", &log));
    block_on(s.power_cycle()).unwrap();
    assert_eq!(
        *log.borrow(),
        ["pwr high", "lpn low", "delay 10 ms", "pwr low", "lpn high", "delay 10 ms"]
    );
}

#[test]
fn power_on_off_and_lpn_methods_drive_the_expected_levels() {
    let log = log();
    let mut s = Sensor::new(NoBus, LogDelay(log.clone()))
        .with_lpn_pin(pin("lpn", &log))
        .with_power_enable_pin_active_high(pin("pwr", &log));
    s.power_off().unwrap();
    s.lpn_disable().unwrap();
    s.power_on().unwrap();
    s.lpn_enable().unwrap();
    assert_eq!(*log.borrow(), ["pwr low", "lpn low", "pwr high", "lpn high"]);
}

/// The deprecated active-high builder stores an inverted line: the deprecated
/// `reset()` and the LPn methods must keep their old meaning, and
/// `power_cycle()` must hold the line "off" (pin high) while power is down.
#[test]
#[allow(deprecated)]
fn deprecated_active_high_builder_keeps_inverted_meaning() {
    let log = log();
    let mut s = Sensor::new(NoBus, LogDelay(log.clone()))
        .with_reset_pin_active_high(pin("rst", &log))
        .with_power_enable_pin_active_high(pin("pwr", &log));
    block_on(s.reset()).unwrap();
    assert_eq!(*log.borrow(), ["rst high", "delay 10 ms", "rst low"]);
    log.borrow_mut().clear();
    block_on(s.power_cycle()).unwrap();
    assert_eq!(
        *log.borrow(),
        ["pwr low", "rst high", "delay 10 ms", "pwr high", "rst low", "delay 10 ms"]
    );
}

#[test]
#[allow(deprecated)]
fn deprecated_active_low_builder_is_with_lpn_pin() {
    let log = log();
    let mut s: Sensor<NoBus, LogDelay, LogPin, NoPin, NoPin> =
        Sensor::new(NoBus, LogDelay(log.clone())).with_reset_pin_active_low(pin("lpn", &log));
    block_on(s.reset()).unwrap();
    assert_eq!(*log.borrow(), ["lpn low", "delay 10 ms", "lpn high"]);
}

#[test]
fn pulse_i2c_rst_pulses_high_for_10_ms() {
    let log = log();
    let mut rst = pin("i2c_rst", &log);
    let mut delay = LogDelay(log.clone());
    block_on(crate::pulse_i2c_rst(&mut rst, &mut delay)).unwrap();
    assert_eq!(*log.borrow(), ["i2c_rst high", "delay 10 ms", "i2c_rst low"]);
}

/// `motion_indicator_set_distance_motion(1500, 400)` used to subtract
/// `max - min` on u16 before range-checking and panicked in debug builds.
#[cfg(any(feature = "motion", feature = "thresholds"))]
#[test]
fn motion_distance_rejects_max_below_min_without_touching_the_bus() {
    let log = log();
    let mut s = Sensor::new(NoBus, LogDelay(log.clone()));
    let mut cfg: MotionConfiguration = unsafe { core::mem::zeroed() };
    for (min, max) in [(1500u16, 400u16), (399, 1000), (400, 4001), (400, 1901), (u16::MAX, 0)] {
        let r = block_on(s.motion_indicator_set_distance_motion(&mut cfg, min, max));
        assert_eq!(r, Err(Error::Firmware(FirmwareError::InvalidParam)), "({min}, {max})");
    }
    assert!(log.borrow().is_empty(), "no delays expected: {:?}", log.borrow());
}
