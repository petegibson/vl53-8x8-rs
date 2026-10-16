#![no_std]

//! Board support for the custom STM32F412RE "evo" board driving its
//! always-on VL53L5CX through ST's C ULD (`vl53l5cx-uld`, blocking
//! `stm32f4xx-hal`).
//!
//! The board carries up to two sensors on I2C1. Only the always-on one is
//! used here; the second sensor's LPn gate (PB9) is held low so it stays off
//! the bus and the default address (0x29 7-bit) answers unambiguously.
//!
//! Wiring (shared with the Embassy sibling, `../evo-f412re-embassy`):
//!
//! | Signal      | Pin | Notes                                        |
//! |-------------|-----|----------------------------------------------|
//! | SCL         | PB6 | I2C1, 1 MHz, internal pull-up                |
//! | SDA         | PB7 | I2C1, 1 MHz, internal pull-up                |
//! | PWR_EN      | PD2 | active **low** (low = sensor rail on)        |
//! | I2C_RESET   | PB8 | active **high** (high = reset asserted)      |
//! | LPn (2nd)   | PB9 | gates the optional second sensor; held low   |
//!
//! Clock: HSE 8 MHz crystal -> PLL -> 96 MHz SYSCLK, 48 MHz APB1, 96 MHz APB2.

use embedded_hal::delay::DelayNs;
use stm32f4xx_hal::gpio::{Output, Pin, PinState};
use stm32f4xx_hal::{pac, prelude::*};
use vl53l5cx_uld::Vl53l5cx;

/// Core clock after `board_init`; handy for `cortex_m::asm::delay` budgets.
pub const SYSCLK_HZ: u32 = 96_000_000;
/// I2C1 bus clock. The I2C v1 block on the F412 is only specified to 400 kHz
/// but, like the Embassy demo on this board, runs the sensor at 1 MHz with
/// the internal pull-ups. Drop to 400 kHz here if the bus proves marginal.
pub const I2C_FREQ_HZ: u32 = 1_000_000;

pub type SensorI2c = stm32f4xx_hal::i2c::I2c<pac::I2C1>;
pub type SensorDelay = stm32f4xx_hal::timer::SysDelay;
/// I2C_RESET (PB8, active high), attached as the sensor's reset pin so
/// `Sensor::reset` and `Sensor::set_i2c_address` are available.
pub type SensorResetPin = Pin<'B', 8, Output>;

vl53l5cx_uld::bind_platform_driver!(SensorI2c, SensorDelay);

pub type Sensor = Vl53l5cx<SensorI2c, SensorDelay, SensorResetPin>;

/// Initialise clocks, GPIOs and I2C1, run the sensor power-on sequence and
/// return a ready-to-`init()` sensor handle.
///
/// The sensor is powered and out of reset but **not** `init()`-ed: callers
/// decide when to pay for the ~86 KB firmware upload.
///
/// Power-on sequence (same timing as the Embassy demo): rail off and reset
/// asserted for 500 ms, rail on, 100 ms, reset released, 1 ms. PB9 is held
/// low throughout so the second sensor position stays gated off.
pub fn board_init() -> Sensor {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Clock tree: HSE 8 MHz crystal -> PLL -> 96 MHz SYSCLK.
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(SYSCLK_HZ.Hz())
        .pclk1(48.MHz())
        .pclk2(96.MHz())
        .freeze();

    let mut delay = cp.SYST.delay(&clocks);

    let gpiob = dp.GPIOB.split();
    let gpiod = dp.GPIOD.split();

    // --- Sensor control pins, initial state: rail off, reset asserted, 2nd gated ---
    let mut pwr_en = gpiod.pd2.into_push_pull_output_in_state(PinState::High); // active low
    let mut i2c_reset = gpiob.pb8.into_push_pull_output_in_state(PinState::High); // active high
    let tof_lp = gpiob.pb9.into_push_pull_output_in_state(PinState::Low); // gate 2nd sensor

    // --- Power-on sequence ---
    delay.delay_ms(500);
    pwr_en.set_low(); // rail on
    delay.delay_ms(100);
    i2c_reset.set_low(); // release reset
    delay.delay_ms(1);

    // PWR_EN and the LPn gate must keep their levels for the life of the
    // program; leaking the handles is the simplest way to guarantee that.
    core::mem::forget(pwr_en);
    core::mem::forget(tof_lp);

    // --- I2C1: PB6 = SCL, PB7 = SDA, open-drain AF4 with internal pull-ups ---
    let scl = gpiob.pb6.into_alternate_open_drain::<4>().internal_pull_up(true);
    let sda = gpiob.pb7.into_alternate_open_drain::<4>().internal_pull_up(true);
    let i2c = stm32f4xx_hal::i2c::I2c::new(dp.I2C1, (scl, sda), I2C_FREQ_HZ.Hz(), &clocks);

    Vl53l5cx::new(i2c, delay).with_reset_pin_active_high(i2c_reset)
}
