#![no_std]

use stm32f4xx_hal::{pac, prelude::*};
#[cfg(not(feature = "l7-uld"))]
use vl53l8cx_uld::Vl53l8cx;
#[cfg(feature = "l7-uld")]
use vl53l7cx_uld::Vl53l7cx;

/// The ULD wrapper crate this build drives the shield with: `vl53l8cx-uld`
/// for the X-NUCLEO-53L8A1, `vl53l7cx-uld` for the X-NUCLEO-53L7A1
/// (`l7-uld`). The two have the same API shape; bins and tests import
/// through this alias.
#[cfg(not(feature = "l7-uld"))]
pub use vl53l8cx_uld as uld;
#[cfg(feature = "l7-uld")]
pub use vl53l7cx_uld as uld;

#[cfg(all(feature = "l7-uld", feature = "spi-bus"))]
compile_error!("the VL53L7CX has no SPI interface: `l7-uld` cannot be combined with `spi-bus`");

pub type SensorDelay = stm32f4xx_hal::timer::SysDelay;

// --- I2C mode (default) ---

#[cfg(not(feature = "spi-bus"))]
pub type SensorI2c = stm32f4xx_hal::i2c::I2c<pac::I2C1>;

#[cfg(all(not(feature = "spi-bus"), not(feature = "l7-uld")))]
vl53l8cx_uld::bind_platform_driver!(SensorI2c, SensorDelay);
#[cfg(feature = "l7-uld")]
vl53l7cx_uld::bind_platform_driver!(SensorI2c, SensorDelay);

#[cfg(all(not(feature = "spi-bus"), not(feature = "l7-uld")))]
pub type Sensor = Vl53l8cx<
    SensorI2c,
    SensorDelay,
    stm32f4xx_hal::gpio::Pin<'B', 0, stm32f4xx_hal::gpio::Output>, // LPn (comms enable)
    stm32f4xx_hal::gpio::Pin<'A', 4, stm32f4xx_hal::gpio::Input>,  // INT
    stm32f4xx_hal::gpio::Pin<'A', 7, stm32f4xx_hal::gpio::Output>, // PWR_EN_C (active high)
>;
/// X-NUCLEO-53L7A1: same control pins as the 53L8A1 (LPn A3, INT A2, PWR_EN D11).
#[cfg(feature = "l7-uld")]
pub type Sensor = Vl53l7cx<
    SensorI2c,
    SensorDelay,
    stm32f4xx_hal::gpio::Pin<'B', 0, stm32f4xx_hal::gpio::Output>, // LPn (comms enable)
    stm32f4xx_hal::gpio::Pin<'A', 4, stm32f4xx_hal::gpio::Input>,  // INT
    stm32f4xx_hal::gpio::Pin<'A', 7, stm32f4xx_hal::gpio::Output>, // PWR_EN_C (active high)
>;

// --- SPI mode ---

#[cfg(feature = "spi-bus")]
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
#[cfg(feature = "spi-bus")]
use vl53l8cx_uld::Vl53l8cxSpiDevice;

#[cfg(feature = "spi-bus")]
pub type SensorSpi = ExclusiveDevice<
    stm32f4xx_hal::spi::Spi<pac::SPI1>,
    stm32f4xx_hal::gpio::Pin<'B', 6, stm32f4xx_hal::gpio::Output>,
    NoDelay,
>;

#[cfg(feature = "spi-bus")]
vl53l8cx_uld::bind_platform_driver_spi!(SensorSpi, SensorDelay);

#[cfg(feature = "spi-bus")]
pub type Sensor = Vl53l8cx<
    Vl53l8cxSpiDevice<SensorSpi>,
    SensorDelay,
    stm32f4xx_hal::gpio::Pin<'B', 0, stm32f4xx_hal::gpio::Output>, // LPn (comms enable)
    stm32f4xx_hal::gpio::Pin<'A', 4, stm32f4xx_hal::gpio::Input>,  // INT
    stm32f4xx_hal::gpio::Pin<'A', 7, stm32f4xx_hal::gpio::Output>, // PWR_EN_C (active high)
>;

// --- Board init ---

/// Initialise clocks, GPIOs, and the bus, returning a ready-to-use sensor handle.
///
/// The sensor is power-cycled (its only real reset) but **not** `init()`-ed —
/// callers decide when to do the (slow) firmware upload.
#[cfg(not(feature = "spi-bus"))]
pub fn board_init() -> Sensor {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Clock tree: HSI 16 MHz -> PLL -> 84 MHz SYSCLK
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .sysclk(84.MHz())
        .pclk1(42.MHz())
        .pclk2(84.MHz())
        .freeze();

    let delay = cp.SYST.delay(&clocks);

    // GPIO banks
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    // --- Center sensor control pins ---
    let mut pwr_en_c = gpioa.pa7.into_push_pull_output(); // PWR_EN_C
    let lp_c = gpiob.pb0.into_push_pull_output(); // LPn center
    let int_c = gpioa.pa4.into_pull_up_input(); // INT center
    // The shield's I2C_RST_C line is on PB3 (D3, shared with SPI1 SCK). It is
    // active high and must sit low for the sensor's I2C interface to answer;
    // left floating it reads as held in reset (every transfer NACKed). Drive
    // it low for the life of the program, as the RTIC and Embassy examples do.
    let i2c_rst_c = gpiob
        .pb3
        .into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::Low);
    core::mem::forget(i2c_rst_c);

    // --- Power-up sequence (board bring-up; `power_cycle()` below does the same) ---
    pwr_en_c.set_low();
    cortex_m::asm::delay(84_000 * 50);

    pwr_en_c.set_high();
    cortex_m::asm::delay(84_000 * 50);

    // I2C1: PB8 = SCL, PB9 = SDA @ 400 kHz
    let i2c = stm32f4xx_hal::i2c::I2c::new(
        dp.I2C1,
        (gpiob.pb8, gpiob.pb9),
        400.kHz(),
        &clocks,
    );

    #[cfg(not(feature = "l7-uld"))]
    let sensor = Vl53l8cx::new(i2c, delay);
    #[cfg(feature = "l7-uld")]
    let sensor = Vl53l7cx::new(i2c, delay);
    let mut sensor = sensor
        .with_lpn_pin(lp_c)
        .with_interrupt_pin_active_low(int_c)
        .with_power_enable_pin_active_high(pwr_en_c);

    // UM2884 §4.2 reset: PWR_EN and LPn low, 10 ms, high (leaves LPn high =
    // I2C enabled); the only real reset the sensor has. 100 ms extra margin.
    sensor.power_cycle().unwrap();
    cortex_m::asm::delay(84_000 * 100);

    sensor
}

#[cfg(feature = "spi-bus")]
pub fn board_init() -> Sensor {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .sysclk(84.MHz())
        .pclk1(42.MHz())
        .pclk2(84.MHz())
        .freeze();

    let delay = cp.SYST.delay(&clocks);

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    // PWR_EN_C
    let mut pwr_en_c = gpioa.pa7.into_push_pull_output();
    // LPn centre sensor: I2C enable (active low = disabled; it is not a reset)
    let lp_c = gpiob.pb0.into_push_pull_output();
    // INT center
    let int_c = gpioa.pa4.into_pull_up_input();

    // Power-up sequence (board bring-up; `power_cycle()` below does the same)
    pwr_en_c.set_low();
    cortex_m::asm::delay(84_000 * 50);
    pwr_en_c.set_high();
    cortex_m::asm::delay(84_000 * 50);

    // SPI1: PB3=SCK, PB4=MISO, PB5=MOSI, Mode 3, 1 MHz
    let sck = gpiob.pb3.into_alternate();
    let miso = gpiob.pb4.into_alternate();
    let mosi = gpiob.pb5.into_alternate();

    let spi = stm32f4xx_hal::spi::Spi::new(
        dp.SPI1,
        (sck, miso, mosi),
        embedded_hal::spi::MODE_3,
        1.MHz(),
        &clocks,
    );

    // NCS on PB6
    let cs = gpiob.pb6.into_push_pull_output();
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    let mut sensor = Vl53l8cx::new_spi(spi_device, delay)
        .with_lpn_pin(lp_c)
        .with_interrupt_pin_active_low(int_c)
        .with_power_enable_pin_active_high(pwr_en_c);

    // UM2884 §4.2 reset: PWR_EN and LPn low, 10 ms, high (leaves LPn high =
    // I2C enabled); the only real reset the sensor has. 100 ms extra margin.
    sensor.power_cycle().unwrap();
    cortex_m::asm::delay(84_000 * 100);

    sensor
}
