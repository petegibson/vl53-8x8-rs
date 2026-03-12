#![no_std]
#![no_main]

use defmt_rtt as _;

use embassy_stm32::i2c;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Delay;
use vl53l8cx_async::{BlockingI2c, Vl53l8cx};

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

    // Reset pin (LPn center) — "active high" means pin HIGH = sensor enabled.
    // Assert reset = pin LOW, deassert = pin HIGH.
    let mut lp_c = Output::new(p.PB0, Level::Low, Speed::Low);
    lp_c.set_low(); // assert reset
    cortex_m::asm::delay(84_000 * 10);
    lp_c.set_high(); // deassert reset, sensor boots
    cortex_m::asm::delay(84_000 * 100);

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

    // Use BlockingI2c wrapper to avoid STM32F4 I2C v1 async DMA read bug.
    // The embassy async I2C driver (with DMA) fails on write_read with
    // Arbitration errors on I2C v1 peripherals (F401, F1, F2, L1).
    // BlockingI2c uses the blocking trait instead, which works correctly.
    let mut sensor = Vl53l8cx::new(BlockingI2c(i2c), Delay);

    defmt::info!("checking sensor is alive...");
    match sensor.is_alive().await {
        Ok(true) => defmt::info!("sensor is alive"),
        Ok(false) => {
            defmt::error!("sensor not alive");
            return;
        }
        Err(e) => {
            defmt::error!("is_alive error: {:?}", defmt::Debug2Format(&e));
            return;
        }
    }

    defmt::info!("initializing sensor...");
    match sensor.init().await {
        Ok(()) => defmt::info!("sensor initialized successfully!"),
        Err(e) => {
            defmt::error!("init error: {:?}", defmt::Debug2Format(&e));
            return;
        }
    }

    defmt::info!("starting ranging...");
    match sensor.start_ranging().await {
        Ok(()) => defmt::info!("ranging started"),
        Err(e) => {
            defmt::error!("start_ranging error: {:?}", defmt::Debug2Format(&e));
            return;
        }
    }

    loop {
        match sensor.poll_data_ready().await {
            Ok(()) => {}
            Err(e) => {
                defmt::error!("poll error: {:?}", defmt::Debug2Format(&e));
                continue;
            }
        }

        match sensor.get_ranging_data().await {
            Ok(data) => {
                // Print center zone distances (zones 27,28 for 8x8 or 5 for 4x4)
                defmt::info!(
                    "range: z0={}mm z27={}mm z28={}mm z63={}mm",
                    data.distance_mm[0],
                    data.distance_mm[27],
                    data.distance_mm[28],
                    data.distance_mm[63],
                );
            }
            Err(e) => {
                defmt::error!("get_ranging_data error: {:?}", defmt::Debug2Format(&e));
            }
        }
    }
}

