//! VL53L8CX on a NUCLEO-F401RE + X-NUCLEO-53L8A1 under RTIC 2 with the
//! blocking `stm32f4xx-hal`, through the `vl53-8x8` driver's blocking bus
//! adapters (`BlockingI2c` / `BlockingSpi`). The SysTick monotonic is the
//! driver's async `DelayNs`, so every `delay` inside the driver yields to the
//! RTIC executor while the bus transfers themselves block.
//!
//! Two software tasks run concurrently: `ranging` (sensor bring-up and the
//! measurement loop, priority 1) and a 1 Hz `heartbeat` (priority 2). The
//! heartbeat keeps ticking during the ~86 KB firmware upload, which shows that
//! the driver's awaits really do hand control back.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use rtic_monotonics::systick::prelude::*;
use stm32f4xx_hal::gpio::{Output, Pin};
use stm32f4xx_hal::pac;
use vl53_8x8::{Resolution, Sensor, NB_TARGET_PER_ZONE};

// --- Configuration ---
const RESOLUTION: Resolution = Resolution::Res4x4;
const RANGING_FREQ_HZ: u8 = 10;
/// Log one frame out of this many (10 Hz ranging -> one grid per second).
const LOG_EVERY_N_FRAMES: u32 = RANGING_FREQ_HZ as u32;
const SYSCLK_HZ: u32 = 84_000_000;

// SysTick monotonic at 1 kHz; implements `embedded_hal_async::delay::DelayNs`.
systick_monotonic!(Mono, 1000);

/// LPn of the centre sensor (reset, active low).
type LpnPin = Pin<'B', 0, Output>;

#[cfg(not(feature = "spi-bus"))]
type SensorBus = vl53_8x8::BlockingI2c<stm32f4xx_hal::i2c::I2c<pac::I2C1>>;

#[cfg(feature = "spi-bus")]
type SensorBus = vl53_8x8::BlockingSpi<
    embedded_hal_bus::spi::ExclusiveDevice<
        stm32f4xx_hal::spi::Spi<pac::SPI1>,
        Pin<'B', 6, Output>,
        embedded_hal_bus::spi::NoDelay,
    >,
>;

type AppSensor = Sensor<SensorBus, Mono, LpnPin>;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [EXTI1, EXTI2], peripherals = true)]
mod app {
    use super::*;
    use stm32f4xx_hal::prelude::*;
    use stm32f4xx_hal::rcc::Config;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        sensor: AppSensor,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        #[cfg(not(feature = "spi-bus"))]
        defmt::info!("VL53L8CX RTIC demo starting (I2C bus, BlockingI2c adapter)");
        #[cfg(feature = "spi-bus")]
        defmt::info!("VL53L8CX RTIC demo starting (SPI bus, BlockingSpi adapter)");

        let dp = ctx.device;

        // Clock tree: HSI 16 MHz -> PLL -> 84 MHz SYSCLK, 42 MHz APB1, 84 MHz APB2.
        let mut rcc = dp.RCC.freeze(
            Config::hsi()
                .sysclk(SYSCLK_HZ.Hz())
                .pclk1(42.MHz())
                .pclk2(84.MHz()),
        );

        // SysTick runs off the core clock; start the monotonic before anything awaits.
        Mono::start(ctx.core.SYST, SYSCLK_HZ);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpiob = dp.GPIOB.split(&mut rcc);

        // --- Centre sensor control pins (X-NUCLEO-53L8A1) ---
        let mut pwr_en_c = gpioa.pa7.into_push_pull_output(); // PWR_EN_C
        let lp_c = gpiob.pb0.into_push_pull_output(); // LPn centre, active low

        // --- Power-up sequence (busy-wait: the monotonic cannot be awaited in init) ---
        pwr_en_c.set_low();
        cortex_m::asm::delay(SYSCLK_HZ / 1000 * 50);
        pwr_en_c.set_high();
        cortex_m::asm::delay(SYSCLK_HZ / 1000 * 50);

        // --- I2C1: PB8 = SCL, PB9 = SDA @ 400 kHz ---
        // The shield's I2C_RST line is on PB3 (shared with SPI1 SCK). It must sit
        // low for the sensor's I2C interface to work; drive it low explicitly.
        #[cfg(not(feature = "spi-bus"))]
        let bus = {
            let _i2c_rst = gpiob
                .pb3
                .into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::Low);
            let i2c = stm32f4xx_hal::i2c::I2c::new(
                dp.I2C1,
                (gpiob.pb8, gpiob.pb9),
                400.kHz(),
                &mut rcc,
            );
            vl53_8x8::BlockingI2c(i2c)
        };

        // --- SPI1: PB3 = SCK, PB4 = MISO, PB5 = MOSI, NCS = PB6; mode 3, 1 MHz ---
        #[cfg(feature = "spi-bus")]
        let bus = {
            let spi = stm32f4xx_hal::spi::Spi::new(
                dp.SPI1,
                // PB3/PB4 come up in AF0 (JTAG), so move them explicitly.
                (
                    Some(gpiob.pb3.into_alternate()),
                    Some(gpiob.pb4.into_alternate()),
                    Some(gpiob.pb5.into_alternate()),
                ),
                embedded_hal::spi::MODE_3,
                1.MHz(),
                &mut rcc,
            );
            let cs = gpiob.pb6.into_push_pull_output();
            let dev = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, cs).unwrap();
            vl53_8x8::BlockingSpi(dev)
        };

        // The monotonic type itself is the driver's `DelayNs`.
        let sensor = Sensor::new(bus, Mono).with_reset_pin_active_low(lp_c);

        ranging::spawn().ok();
        heartbeat::spawn().ok();

        (Shared {}, Local { sensor })
    }

    /// Sensor bring-up and the measurement loop. Priority 1: the heartbeat
    /// preempts it, including in the middle of a blocking bus transfer.
    #[task(priority = 1, local = [sensor])]
    async fn ranging(ctx: ranging::Context) {
        let sensor = ctx.local.sensor;

        if let Err(e) = sensor.reset().await {
            defmt::error!("Sensor reset failed: {}", defmt::Debug2Format(&e));
            return;
        }
        Mono::delay(100.millis()).await;

        defmt::info!("Initialising sensor (firmware upload)...");
        let t0 = Mono::now();
        if let Err(e) = sensor.init().await {
            defmt::error!("Sensor init failed: {}", defmt::Debug2Format(&e));
            #[cfg(not(feature = "spi-bus"))]
            defmt::error!("Check that X-NUCLEO-53L8A1 is connected and I2C jumpers are set");
            #[cfg(feature = "spi-bus")]
            defmt::error!("Check that X-NUCLEO-53L8A1 is connected and SPI jumpers are set");
            return;
        }
        let elapsed = Mono::now() - t0;
        defmt::info!(
            "Sensor ready: {} after {} ms",
            defmt::Debug2Format(&sensor.variant()),
            elapsed.to_millis()
        );

        let grid = match RESOLUTION {
            Resolution::Res4x4 => 4usize,
            Resolution::Res8x8 => 8,
        };
        defmt::info!("Configuring: {}x{} @ {} Hz", grid, grid, RANGING_FREQ_HZ);
        if let Err(e) = sensor.set_resolution(RESOLUTION).await {
            defmt::error!("set_resolution failed: {}", defmt::Debug2Format(&e));
            return;
        }
        if let Err(e) = sensor.set_ranging_frequency_hz(RANGING_FREQ_HZ).await {
            defmt::error!("set_ranging_frequency_hz failed: {}", defmt::Debug2Format(&e));
            return;
        }
        if let Err(e) = sensor.start_ranging().await {
            defmt::error!("start_ranging failed: {}", defmt::Debug2Format(&e));
            return;
        }

        defmt::info!("Entering measurement loop");
        let mut frames: u32 = 0;
        loop {
            // Polls the sensor over the bus with async delays in between; no INT pin needed.
            if let Err(e) = sensor.poll_data_ready().await {
                defmt::error!("Data ready check error: {}", defmt::Debug2Format(&e));
                continue;
            }

            let data = match sensor.get_ranging_data().await {
                Ok(data) => data,
                Err(e) => {
                    defmt::error!("Ranging read error: {}", defmt::Debug2Format(&e));
                    continue;
                }
            };

            frames += 1;
            if frames % LOG_EVERY_N_FRAMES != 0 {
                continue;
            }

            defmt::info!("--- Frame {} distance (mm) ---", frames);
            // First target of each zone (per-target arrays are strided by N).
            let d = |zone: usize| data.distance_mm[zone * NB_TARGET_PER_ZONE];
            for row in 0..grid {
                let b = row * grid;
                match grid {
                    4 => defmt::info!("  {} {} {} {}", d(b), d(b + 1), d(b + 2), d(b + 3)),
                    _ => defmt::info!(
                        "  {} {} {} {} {} {} {} {}",
                        d(b),
                        d(b + 1),
                        d(b + 2),
                        d(b + 3),
                        d(b + 4),
                        d(b + 5),
                        d(b + 6),
                        d(b + 7),
                    ),
                }
            }
        }
    }

    /// 1 Hz heartbeat, independent of the sensor. It keeps running during the
    /// firmware upload: the driver's awaits yield to the executor.
    #[task(priority = 2, local = [ticks: u32 = 0])]
    async fn heartbeat(ctx: heartbeat::Context) {
        let mut next = Mono::now();
        loop {
            next += 1000.millis();
            Mono::delay_until(next).await;
            *ctx.local.ticks += 1;
            defmt::info!("tick {}", *ctx.local.ticks);
        }
    }
}
