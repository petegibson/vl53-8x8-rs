#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Delay;
use vl53l8cx_async::{Resolution, Vl53l8cx};

// --- Configuration ---
const RESOLUTION: Resolution = Resolution::Res4x4;
const RANGING_FREQ_HZ: u8 = 10;

// I2C1 interrupt bindings
#[cfg(not(feature = "spi-bus"))]
embassy_stm32::bind_interrupts!(struct Irqs {
    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    #[cfg(not(feature = "spi-bus"))]
    defmt::info!("VL53L8CX async driver starting (I2C bus, BlockingI2c wrapper)");
    #[cfg(feature = "spi-bus")]
    defmt::info!("VL53L8CX async driver starting (SPI bus)");

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

    // --- Center sensor control pins ---
    let mut pwr_en_c = Output::new(p.PA7, Level::Low, Speed::Low); // PWR_EN_C
    let lp_c = Output::new(p.PB0, Level::Low, Speed::Low);         // LPn center

    // --- Power-up sequence ---
    pwr_en_c.set_low();
    cortex_m::asm::delay(84_000 * 50);
    pwr_en_c.set_high();
    cortex_m::asm::delay(84_000 * 50);

    // --- I2C mode (default) ---
    #[cfg(not(feature = "spi-bus"))]
    let mut sensor = {
        let _i2c_rst_c = Output::new(p.PB3, Level::Low, Speed::Low); // I2C_RST held low

        let mut i2c_config = embassy_stm32::i2c::Config::default();
        i2c_config.frequency = embassy_stm32::time::Hertz(400_000);
        let i2c = embassy_stm32::i2c::I2c::new(
            p.I2C1,
            p.PB8,
            p.PB9,
            Irqs,
            p.DMA1_CH6,
            p.DMA1_CH0,
            i2c_config,
        );

        Vl53l8cx::new(vl53l8cx_async::BlockingI2c(i2c), Delay)
            .with_reset_pin_active_high(lp_c)
    };

    // --- SPI mode ---
    #[cfg(feature = "spi-bus")]
    let mut sensor = {
        use embassy_stm32::spi;
        use embedded_hal_bus::spi::ExclusiveDevice;

        // SPI1: PB3=SCK, PB4=MISO, PB5=MOSI, Mode 3, 1 MHz
        let mut spi_config = spi::Config::default();
        spi_config.mode = spi::Mode {
            polarity: spi::Polarity::IdleHigh,
            phase: spi::Phase::CaptureOnSecondTransition,
        };
        spi_config.frequency = embassy_stm32::time::Hertz(1_000_000);

        let spi = spi::Spi::new(
            p.SPI1,
            p.PB3,  // SCK
            p.PB5,  // MOSI
            p.PB4,  // MISO
            p.DMA2_CH3, // TX
            p.DMA2_CH0, // RX
            spi_config,
        );

        // NCS on PB6
        let cs = Output::new(p.PB6, Level::High, Speed::VeryHigh);
        let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

        Vl53l8cx::new_spi(spi_device, Delay)
            .with_reset_pin_active_high(lp_c)
    };

    sensor.reset().await.unwrap();
    cortex_m::asm::delay(84_000 * 100);

    defmt::info!("Initialising sensor...");
    if let Err(e) = sensor.init().await {
        defmt::error!("Sensor init failed: {}", defmt::Debug2Format(&e));
        #[cfg(not(feature = "spi-bus"))]
        defmt::error!("Check that X-NUCLEO-53L8A1 is connected and I2C jumpers are set");
        #[cfg(feature = "spi-bus")]
        defmt::error!("Check that X-NUCLEO-53L8A1 is connected and SPI jumpers are set");
        loop {}
    }

    let grid = match RESOLUTION {
        Resolution::Res4x4 => 4usize,
        Resolution::Res8x8 => 8,
    };
    defmt::info!("Configuring: {}x{} @ {} Hz", grid, grid, RANGING_FREQ_HZ);
    if let Err(e) = sensor.set_resolution(RESOLUTION).await {
        defmt::error!("set_resolution failed: {}", defmt::Debug2Format(&e));
        loop {}
    }
    if let Err(e) = sensor.set_ranging_frequency_hz(RANGING_FREQ_HZ).await {
        defmt::error!("set_ranging_frequency_hz failed: {}", defmt::Debug2Format(&e));
        loop {}
    }
    if let Err(e) = sensor.start_ranging().await {
        defmt::error!("start_ranging failed: {}", defmt::Debug2Format(&e));
        loop {}
    }

    defmt::info!("Entering measurement loop");
    loop {
        match sensor.poll_data_ready().await {
            Ok(()) => {}
            Err(e) => {
                defmt::error!("Data ready check error: {}", defmt::Debug2Format(&e));
                continue;
            }
        }

        match sensor.get_ranging_data().await {
            Ok(data) => {
                defmt::info!("--- Distance (mm) ---");
                for row in 0..grid {
                    let b = row * grid;
                    match grid {
                        4 => defmt::info!(
                            "  {} {} {} {}",
                            data.distance_mm[b],
                            data.distance_mm[b + 1],
                            data.distance_mm[b + 2],
                            data.distance_mm[b + 3],
                        ),
                        _ => defmt::info!(
                            "  {} {} {} {} {} {} {} {}",
                            data.distance_mm[b],
                            data.distance_mm[b + 1],
                            data.distance_mm[b + 2],
                            data.distance_mm[b + 3],
                            data.distance_mm[b + 4],
                            data.distance_mm[b + 5],
                            data.distance_mm[b + 6],
                            data.distance_mm[b + 7],
                        ),
                    }
                }
            }
            Err(e) => {
                defmt::error!("Ranging read error: {}", defmt::Debug2Format(&e));
            }
        }
    }
}
