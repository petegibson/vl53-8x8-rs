#![no_std]
#![no_main]

use defmt_rtt as _;

use embassy_stm32::i2c;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_time::Delay;
use vl53l8cx_async::Vl53l8cx;

use panic_probe as _;

// I2C1 interrupt bindings for async operation
embassy_stm32::bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    defmt::info!("power on, init board...");

    // Clock config: HSI 16 MHz -> PLL -> 84 MHz SYSCLK
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = true;
        config.rcc.pll_src = PllSource::HSI;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV16,   // 16 MHz / 16 = 1 MHz
            mul: PllMul::MUL336,        // 1 MHz * 336 = 336 MHz VCO
            divp: Some(PllPDiv::DIV4),  // 336 / 4 = 84 MHz SYSCLK
            divq: Some(PllQDiv::DIV7),  // 336 / 7 = 48 MHz (USB)
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;   // 84 MHz
        config.rcc.apb1_pre = APBPrescaler::DIV2;   // 42 MHz
        config.rcc.apb2_pre = APBPrescaler::DIV1;   // 84 MHz
    }

    let p = embassy_stm32::init(config);

    // Power-up sequence (matches blocking driver)
    let mut pwr_en_c = Output::new(p.PA7, Level::Low, Speed::Low);
    let _i2c_rst_c = Output::new(p.PB3, Level::Low, Speed::Low); // I2C_RST held low
    cortex_m::asm::delay(84_000 * 50);
    pwr_en_c.set_high();
    cortex_m::asm::delay(84_000 * 50);
    // Keep power pin alive (i2c_rst_c stays low via _i2c_rst_c binding)
    core::mem::forget(pwr_en_c);

    // Reset pin (LPn center) — active-high reset: assert high, delay, deassert low
    let mut lp_c = Output::new(p.PB0, Level::Low, Speed::Low);
    lp_c.set_high(); // assert reset
    cortex_m::asm::delay(84_000 * 10);
    lp_c.set_low(); // deassert reset
    cortex_m::asm::delay(84_000 * 100);

    let int_c = embassy_stm32::gpio::Input::new(p.PA4, Pull::Up);

    // Async I2C1: PB8 = SCL, PB9 = SDA @ 400 kHz
    // Read PCLK1 from RCC registers directly for diagnostics
    let cfgr = unsafe { core::ptr::read_volatile(0x4002_3808 as *const u32) };
    let ppre1 = (cfgr >> 10) & 0x7;
    defmt::info!("RCC_CFGR: {:#010x}, PPRE1 bits: {}", cfgr, ppre1);

    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = embassy_stm32::time::Hertz(400_000);
    let i2c = i2c::I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH0,
        i2c_config,
    );

    let mut sensor = Vl53l8cx::new(i2c, Delay)
        .with_reset_pin_active_high(lp_c)
        .with_interrupt_pin_active_low(int_c);

    defmt::info!("init sensor...");
    let result = sensor.init().await;
    match result {
        Ok(_) => defmt::info!("init complete"),
        Err(e) => defmt::error!("init failed: {:?}", defmt::Debug2Format(&e)),
    }
}

