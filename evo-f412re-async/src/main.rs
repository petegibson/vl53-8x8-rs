#![no_std]
#![no_main]

//! Async VL53L5CX / VL53L8CX ranging demo for a custom STM32F412RE board.
//!
//! The driver autodetects which sensor variant is fitted (by silicon revision
//! id) and uploads the matching patched firmware, so the same binary runs on a
//! board populated with either part.
//!
//! Board wiring (I2C1, fast-mode-plus @ 1 MHz with internal pull-ups):
//! - SCL          PB6
//! - SDA          PB7
//! - TX DMA       DMA1_CH7
//! - RX DMA       DMA1_CH0
//! - PWR_EN       PD2  (active LOW: low = sensor powered)
//! - I2C_RESET    PB8  (active HIGH: high = reset asserted)
//! - LPn (2nd)    PB9  (gates the optional second sensor; held low here)
//! - INFO LED     PA8  (onboard, active low: low = on)
//!
//! The board carries up to two sensors sharing one I2C bus. Only the always-on
//! sensor responds at the default address (0x52); the second sensor's LPn is on
//! PB9 and is held low so a single sensor enumerates cleanly. If your sensor is
//! the second position instead, drive PB9 high after the power-up sequence.

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, dma, i2c, peripherals};
use embassy_time::{Delay, Timer};
use vl53l8cx_async::{Resolution, Vl53l8cx};

// --- Configuration ---
const RESOLUTION: Resolution = Resolution::Res4x4;
const RANGING_FREQ_HZ: u8 = 10;
/// Light the LED when any zone reports a valid target at or within this range.
const OBJECT_THRESHOLD_MM: i16 = 200;
/// VL53 target status for a fully-valid range measurement.
const STATUS_VALID: u8 = 5;

// I2C1 + DMA interrupt bindings (embassy 0.6 routes DMA completion through
// these handlers for async transfers).
bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    DMA1_STREAM7 => dma::InterruptHandler<peripherals::DMA1_CH7>;
    DMA1_STREAM0 => dma::InterruptHandler<peripherals::DMA1_CH0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Clock: HSE 8 MHz crystal -> PLL -> 96 MHz SYSCLK (48 MHz on Q for USB).
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV6,
            mul: PllMul::MUL144,
            divp: Some(PllPDiv::DIV2), // 8 MHz / 6 * 144 / 2 = 96 MHz
            divq: Some(PllQDiv::DIV4), // 8 MHz / 6 * 144 / 4 = 48 MHz
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1; // 96 MHz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 48 MHz
        config.rcc.apb2_pre = APBPrescaler::DIV1; // 96 MHz
    }

    let p = embassy_stm32::init(config);
    defmt::info!("VL53Lxcx async demo starting (STM32F412RE)");

    // Onboard info LED on PA8 (active low: drive low to light it).
    let mut led = Output::new(p.PA8, Level::High, Speed::Low);

    // Flash twice on startup, before bringing up the sensor.
    for _ in 0..2 {
        led.set_low(); // on
        Timer::after_millis(120).await;
        led.set_high(); // off
        Timer::after_millis(120).await;
    }

    // --- Sensor power / reset control pins ---
    // Initial levels = sensor off, reset asserted.
    let mut pwr_en = Output::new(p.PD2, Level::High, Speed::Low); // active low
    let mut i2c_reset = Output::new(p.PB8, Level::High, Speed::Low); // active high
    let mut tof_lp = Output::new(p.PB9, Level::Low, Speed::Low); // gate 2nd sensor

    // --- Power-up sequence (matches the production board bring-up timing) ---
    pwr_en.set_high(); // power off
    i2c_reset.set_high(); // assert reset
    Timer::after_millis(500).await;
    pwr_en.set_low(); // power on
    Timer::after_millis(100).await;
    i2c_reset.set_low(); // release reset
    Timer::after_millis(1).await;
    tof_lp.set_low(); // keep the second sensor disabled

    // --- I2C1 @ 1 MHz, internal pull-ups enabled, async DMA ---
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = Hertz(1_000_000);
    i2c_config.scl_pullup = true;
    i2c_config.sda_pullup = true;
    let i2c = I2c::new(
        p.I2C1,
        p.PB6,       // SCL
        p.PB7,       // SDA
        p.DMA1_CH7,  // TX DMA
        p.DMA1_CH0,  // RX DMA
        Irqs,
        i2c_config,
    );

    let mut sensor = Vl53l8cx::new(i2c, Delay);

    // init() autodetects the variant and uploads its patched firmware. Flash
    // the LED continuously throughout by racing a blink loop against init() on
    // the executor — init() yields during each I2C/DMA transfer, letting the
    // blink loop run.
    defmt::info!("Initialising sensor (uploading firmware)...");
    let init_result = match select(sensor.init(), async {
        loop {
            led.set_low(); // on
            Timer::after_millis(80).await;
            led.set_high(); // off
            Timer::after_millis(80).await;
        }
    })
    .await
    {
        Either::First(result) => result,
        Either::Second(()) => unreachable!("blink loop never returns"),
    };
    led.set_high(); // ensure LED off once upload completes
    if let Err(e) = init_result {
        defmt::error!("Sensor init failed: {}", defmt::Debug2Format(&e));
        defmt::error!("Check sensor power (PD2), I2C reset (PB8), and bus wiring");
        loop {}
    }
    defmt::info!(
        "Detected sensor variant: {}",
        defmt::Debug2Format(&sensor.variant())
    );

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
        if let Err(e) = sensor.poll_data_ready().await {
            defmt::error!("Data ready check error: {}", defmt::Debug2Format(&e));
            continue;
        }

        match sensor.get_ranging_data().await {
            Ok(data) => {
                // Light the LED if any zone has a valid target within the
                // threshold. INFO_LED is active low (low = on).
                let zones = grid * grid;
                let object_near = data.distance_mm[..zones]
                    .iter()
                    .zip(data.target_status[..zones].iter())
                    .any(|(&d, &status)| status == STATUS_VALID && d > 0 && d <= OBJECT_THRESHOLD_MM);
                if object_near {
                    led.set_low();
                } else {
                    led.set_high();
                }

                defmt::info!(
                    "--- Distance (mm){} ---",
                    if object_near { "  [OBJECT <200mm]" } else { "" }
                );
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
            Err(e) => defmt::error!("Ranging read error: {}", defmt::Debug2Format(&e)),
        }
    }
}
