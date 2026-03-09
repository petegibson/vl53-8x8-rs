#![no_std]

#[cfg(not(feature = "spi-bus"))]
use stm32f4xx_hal::{pac, prelude::*};

#[cfg(not(feature = "spi-bus"))]
use vl53l8cx_driver::Vl53l8cx;

// --- I2C mode (default) ---

#[cfg(not(feature = "spi-bus"))]
pub type SensorI2c = stm32f4xx_hal::i2c::I2c<pac::I2C1>;
#[cfg(not(feature = "spi-bus"))]
pub type SensorDelay = stm32f4xx_hal::timer::SysDelay;

#[cfg(not(feature = "spi-bus"))]
vl53l8cx_driver::bind_platform_driver!(SensorI2c, SensorDelay);

#[cfg(not(feature = "spi-bus"))]
pub type Sensor = Vl53l8cx<
    SensorI2c,
    SensorDelay,
    stm32f4xx_hal::gpio::Pin<'B', 0, stm32f4xx_hal::gpio::Output>,
    stm32f4xx_hal::gpio::Pin<'A', 4, stm32f4xx_hal::gpio::Input>,
>;

/// Initialise clocks, GPIOs, I2C and return a ready-to-use sensor handle.
///
/// The sensor is reset but **not** `init()`-ed — callers decide when to do
/// the (slow) firmware upload.
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
    let mut i2c_rst_c = gpiob.pb3.into_push_pull_output(); // I2C_RST
    let lp_c = gpiob.pb0.into_push_pull_output(); // LPn center
    let int_c = gpioa.pa4.into_pull_up_input(); // INT center

    // --- Power-up sequence ---
    pwr_en_c.set_low();
    i2c_rst_c.set_low();
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

    let mut sensor = Vl53l8cx::new(i2c, delay)
        .with_reset_pin_active_high(lp_c)
        .with_interrupt_pin_active_low(int_c);

    sensor.reset().unwrap();
    cortex_m::asm::delay(84_000 * 100);

    sensor
}

// --- SPI mode ---

#[cfg(feature = "spi-bus")]
pub mod spi {
    use stm32f4xx_hal::{pac, prelude::*};
    use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
    use vl53l8cx_driver::{Vl53l8cx, Vl53l8cxSpiDevice};

    pub type SensorSpi = ExclusiveDevice<
        stm32f4xx_hal::spi::Spi<pac::SPI1>,
        stm32f4xx_hal::gpio::Pin<'B', 6, stm32f4xx_hal::gpio::Output>,
        NoDelay,
    >;
    pub type SensorDelay = stm32f4xx_hal::timer::SysDelay;

    vl53l8cx_driver::bind_platform_driver_spi!(SensorSpi, SensorDelay);

    pub type Sensor = Vl53l8cx<
        Vl53l8cxSpiDevice<SensorSpi>,
        SensorDelay,
        stm32f4xx_hal::gpio::Pin<'B', 0, stm32f4xx_hal::gpio::Output>,
        stm32f4xx_hal::gpio::Pin<'A', 4, stm32f4xx_hal::gpio::Input>,
    >;

    /// Initialise clocks, GPIOs, SPI and return a ready-to-use sensor handle.
    ///
    /// The sensor is reset but **not** `init()`-ed — callers decide when to do
    /// the (slow) firmware upload.
    pub fn board_init_spi() -> Sensor {
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
        // LPn center (reset, active high)
        let lp_c = gpiob.pb0.into_push_pull_output();
        // INT center
        let int_c = gpioa.pa4.into_pull_up_input();

        // Power-up sequence
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
            .with_reset_pin_active_high(lp_c)
            .with_interrupt_pin_active_low(int_c);

        sensor.reset().unwrap();
        cortex_m::asm::delay(84_000 * 100);

        sensor
    }
}
