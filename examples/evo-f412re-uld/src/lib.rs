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
use vl53l5cx_uld::{pulse_i2c_rst, NoPin, Vl53l5cx};

/// Core clock after `board_init`; handy for `cortex_m::asm::delay` budgets.
pub const SYSCLK_HZ: u32 = 96_000_000;
/// I2C1 bus clock. The I2C v1 block on the F412 is only specified to 400 kHz
/// but, like the Embassy demo on this board, runs the sensor at 1 MHz with
/// the internal pull-ups. Drop to 400 kHz here if the bus proves marginal.
pub const I2C_FREQ_HZ: u32 = 1_000_000;

pub type SensorI2c = stm32f4xx_hal::i2c::I2c<pac::I2C1>;
pub type SensorDelay = stm32f4xx_hal::timer::SysDelay;
/// PWR_EN (PD2, active low) switches the sensor rail; attached as the driver's
/// power-enable pin so `Sensor::power_cycle()` (the only real reset) works.
pub type SensorPowerPin = Pin<'D', 2, Output>;
/// I2C_RESET (PB8, active high) is the sensors' I2C_RST input: it resets the
/// I2C interface only, not the sensor. `board_init` pulses it once with
/// `vl53l5cx_uld::pulse_i2c_rst` after the rail is up and then leaves it low.
pub type I2cResetPin = Pin<'B', 8, Output>;

vl53l5cx_uld::bind_platform_driver!(SensorI2c, SensorDelay);

/// The always-on sensor's LPn is not on a GPIO (it follows IOVDD), so no LPn
/// pin; `set_i2c_address` needs none either.
pub type Sensor = Vl53l5cx<SensorI2c, SensorDelay, NoPin, NoPin, SensorPowerPin>;

/// Initialise clocks, GPIOs and I2C1, run the sensor power-on sequence and
/// return a ready-to-`init()` sensor handle.
///
/// The sensor is powered, its I2C interface reset, but **not** `init()`-ed:
/// callers decide when to pay for the ~86 KB firmware upload.
///
/// Power-on sequence (same timing as the Embassy demo): rail off for 500 ms,
/// rail on, 100 ms, then I2C_RST pulsed high for 10 ms (`pulse_i2c_rst`) and
/// 1 ms settle. PB9 is held low throughout so the second sensor position stays
/// gated off. `Sensor::power_cycle()` repeats the rail cycle with UM2884 §4.2
/// timing whenever a real reset is needed later.
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

    // --- Sensor control pins, initial state: rail off, I2C_RST idle (low), 2nd gated ---
    let mut pwr_en: SensorPowerPin = gpiod.pd2.into_push_pull_output_in_state(PinState::High); // active low
    let mut i2c_reset: I2cResetPin = gpiob.pb8.into_push_pull_output_in_state(PinState::Low); // active high
    let tof_lp = gpiob.pb9.into_push_pull_output_in_state(PinState::Low); // gate 2nd sensor

    // --- Power-on sequence (board bring-up; `power_cycle()` does the rail part) ---
    delay.delay_ms(500);
    pwr_en.set_low(); // rail on
    delay.delay_ms(100);
    pulse_i2c_rst(&mut i2c_reset, &mut delay).unwrap(); // reset the I2C interface only
    delay.delay_ms(1);

    // The LPn gate and I2C_RST must keep their levels for the life of the
    // program; leaking the handles is the simplest way to guarantee that.
    // PWR_EN goes into the sensor handle (`power_cycle()`).
    core::mem::forget(i2c_reset);
    core::mem::forget(tof_lp);

    // --- I2C1: PB6 = SCL, PB7 = SDA, open-drain AF4 with internal pull-ups ---
    let scl = gpiob.pb6.into_alternate_open_drain::<4>().internal_pull_up(true);
    let sda = gpiob.pb7.into_alternate_open_drain::<4>().internal_pull_up(true);
    let i2c = stm32f4xx_hal::i2c::I2c::new(dp.I2C1, (scl, sda), I2C_FREQ_HZ.Hz(), &clocks);

    Vl53l5cx::new(i2c, delay).with_power_enable_pin_active_low(pwr_en)
}
