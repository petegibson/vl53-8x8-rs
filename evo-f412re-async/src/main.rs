#![no_std]
#![no_main]

//! Dual-sensor power-comparison demo for a custom STM32F412RE board.
//!
//! The board carries two VL53L5CX / VL53L8CX sensors on one I2C bus. This demo
//! brings both up and runs them simultaneously at 8x8 @ 5 Hz so their power
//! draw can be compared on a bench supply (e.g. swap an L5 for an L8 in the
//! second position and watch the delta).
//!
//! Bring-up sequence (only one sensor may sit at the default address at a time):
//!   1. Power up with the second sensor gated off (LPn low) so only the
//!      always-on sensor enumerates at the default address (0x29 7-bit).
//!   2. Init sensor 1, then move it to [`SENSOR1_ADDRESS`].
//!   3. Release the second sensor's LPn; it now answers at the default address.
//!   4. Init sensor 2, configure both, and start ranging.
//!
//! Recovery: any init or ranging error tears the session down and re-runs the
//! whole bring-up. The first attempt pulses the shared reset line; if that does
//! not recover the sensors, the next attempt power-cycles the rail.
//!
//! Board wiring (I2C1, fast-mode-plus @ 1 MHz with internal pull-ups):
//! - SCL          PB6
//! - SDA          PB7
//! - TX DMA       DMA1_CH7
//! - RX DMA       DMA1_CH0
//! - PWR_EN       PD2  (active LOW: low = sensor powered)
//! - I2C_RESET    PB8  (active HIGH: high = reset asserted; shared by both sensors)
//! - LPn (2nd)    PB9  (gates the second sensor: low = gated off)
//! - INFO LED     PA8  (onboard, active low: low = on)

use defmt_rtt as _;
use panic_probe as _;

use defmt::Debug2Format;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::I2cDeviceError;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c, Master};
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, dma, i2c, peripherals};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Timer};
use vl53l8cx_async::{
    Error, FirmwareError, ResultsData, Resolution, SevenBitAddress, Vl53l8cx, DEFAULT_ADDRESS,
};

// --- Configuration ---
const RESOLUTION: Resolution = Resolution::Res8x8;
const RANGING_FREQ_HZ: u8 = 5;
/// 7-bit address sensor 1 is moved to so the second sensor can take the default
/// address ([`DEFAULT_ADDRESS`], 0x29). Any free 7-bit address works.
const SENSOR1_ADDRESS: SevenBitAddress = 0x2A;
/// Light the LED when any zone reports a valid target at or within this range.
const OBJECT_THRESHOLD_MM: i16 = 200;
/// VL53 target status for a fully-valid range measurement.
const STATUS_VALID: u8 = 5;
/// Consecutive transient frame errors tolerated before forcing a full recovery.
/// A corrupt frame on the 1 MHz bus is usually a one-off and clears on the next
/// read; only a persistent run of them indicates a sensor that needs resetting.
const MAX_TRANSIENT_ERRORS: u8 = 5;

// Concrete bus types: both sensors share one I2C peripheral through a mutex, so
// each `I2cDevice` locks the bus only for the duration of a single transfer.
type SharedI2c = Mutex<NoopRawMutex, I2c<'static, Async, Master>>;
type SensorBus<'a> = I2cDevice<'a, NoopRawMutex, I2c<'static, Async, Master>>;
type Sensor<'a> = Vl53l8cx<SensorBus<'a>, Delay>;
type SensorError = Error<I2cDeviceError<embassy_stm32::i2c::Error>>;

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
    defmt::info!("VL53Lxcx dual-sensor power demo starting (STM32F412RE)");

    // Onboard info LED on PA8 (active low: drive low to light it).
    let mut led = Output::new(p.PA8, Level::High, Speed::Low);

    // Flash twice on startup, before bringing up the sensors.
    for _ in 0..2 {
        led.set_low(); // on
        Timer::after_millis(120).await;
        led.set_high(); // off
        Timer::after_millis(120).await;
    }

    // --- Sensor power / reset control pins (owned here for recovery) ---
    // Initial levels = sensor off, reset asserted, second sensor gated off.
    let mut pwr_en = Output::new(p.PD2, Level::High, Speed::Low); // active low
    let mut i2c_reset = Output::new(p.PB8, Level::High, Speed::Low); // active high
    let mut tof_lp = Output::new(p.PB9, Level::Low, Speed::Low); // gate 2nd sensor

    // --- I2C1 @ 1 MHz, internal pull-ups enabled, async DMA ---
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = Hertz(1_000_000);
    i2c_config.scl_pullup = true;
    i2c_config.sda_pullup = true;
    let i2c = I2c::new(
        p.I2C1,
        p.PB6,      // SCL
        p.PB7,      // SDA
        p.DMA1_CH7, // TX DMA
        p.DMA1_CH0, // RX DMA
        Irqs,
        i2c_config,
    );
    // Shared bus: both sensor handles borrow this mutex for the whole run.
    let bus: SharedI2c = Mutex::new(i2c);

    // Initial bring-up uses a full power-on sequence.
    power_on_sequence(&mut pwr_en, &mut i2c_reset, &mut tof_lp).await;

    // Run a session; on any error escalate from a reset pulse to a power cycle,
    // then re-run the full two-sensor bring-up.
    let mut tried_reset = false;
    loop {
        // `run_session` only returns on error: the measurement loop never ends.
        if let Err(e) = run_session(&bus, &mut tof_lp, &mut led).await {
            defmt::error!("Session failed: {}", Debug2Format(&e));
        }
        led.set_high(); // LED off while recovering

        if !tried_reset {
            defmt::warn!("Recovery: pulsing shared reset line");
            reset_pulse(&mut i2c_reset, &mut tof_lp).await;
            tried_reset = true;
        } else {
            defmt::warn!("Recovery: reset did not help, power-cycling sensors");
            power_on_sequence(&mut pwr_en, &mut i2c_reset, &mut tof_lp).await;
            tried_reset = false;
        }
    }
}

/// Bring up both sensors and run them at 8x8 @ 5 Hz. Returns only on error; a
/// successful run loops forever reporting frames.
async fn run_session(
    bus: &SharedI2c,
    tof_lp: &mut Output<'static>,
    led: &mut Output<'static>,
) -> Result<(), SensorError> {
    // --- Sensor 1: the always-on sensor, currently at the default address ---
    defmt::info!("Bringing up sensor 1 (default address {:#04x})...", DEFAULT_ADDRESS);
    let mut s1 = Vl53l8cx::new(I2cDevice::new(bus), Delay);
    init_blinking(&mut s1, led).await?;
    defmt::info!("Sensor 1 variant: {}", Debug2Format(&s1.variant()));

    // Move sensor 1 off the default address so sensor 2 can take it.
    s1.set_i2c_address(SENSOR1_ADDRESS).await?;
    defmt::info!("Sensor 1 readdressed to 7-bit {:#04x}", SENSOR1_ADDRESS);

    // --- Sensor 2: release its LPn gate; it now answers at the default address ---
    tof_lp.set_high();
    Timer::after_millis(10).await;
    defmt::info!("Bringing up sensor 2 (default address {:#04x})...", DEFAULT_ADDRESS);
    let mut s2 = Vl53l8cx::new(I2cDevice::new(bus), Delay);
    init_blinking(&mut s2, led).await?;
    defmt::info!("Sensor 2 variant: {}", Debug2Format(&s2.variant()));

    // --- Configure both and start ranging ---
    let grid = match RESOLUTION {
        Resolution::Res4x4 => 4usize,
        Resolution::Res8x8 => 8,
    };
    defmt::info!("Configuring both sensors: {}x{} @ {} Hz", grid, grid, RANGING_FREQ_HZ);
    for s in [&mut s1, &mut s2] {
        s.set_resolution(RESOLUTION).await?;
        s.set_ranging_frequency_hz(RANGING_FREQ_HZ).await?;
        s.start_ranging().await?;
    }

    defmt::info!("Both sensors ranging; comparing power draw");
    let mut transient_errors = 0u8;
    loop {
        let (near1, near2) = match (
            read_and_report(&mut s1, "S1", grid).await,
            read_and_report(&mut s2, "S2", grid).await,
        ) {
            (Ok(a), Ok(b)) => {
                transient_errors = 0; // a clean round clears the transient streak
                (a, b)
            }
            (a, b) => {
                // A corrupt frame is a recoverable one-off: skip it and retry.
                // Any other error (or too many corrupt frames in a row) escalates
                // to the caller's reset / power-cycle recovery.
                for e in [a.err(), b.err()].into_iter().flatten() {
                    if !matches!(e, Error::Firmware(FirmwareError::CorruptedFrame)) {
                        return Err(e);
                    }
                    transient_errors += 1;
                    defmt::warn!(
                        "Corrupt frame {}/{}: {}",
                        transient_errors,
                        MAX_TRANSIENT_ERRORS,
                        Debug2Format(&e)
                    );
                }
                if transient_errors >= MAX_TRANSIENT_ERRORS {
                    return Err(Error::Firmware(FirmwareError::CorruptedFrame));
                }
                continue;
            }
        };

        // INFO_LED is active low: light it if either sensor sees a near object.
        if near1 || near2 {
            led.set_low();
        } else {
            led.set_high();
        }
    }
}

/// Run `init()` while flashing the LED. `init()` yields during each I2C/DMA
/// transfer, so racing it against a blink loop keeps the LED visibly active for
/// the duration of the ~84 KB firmware upload.
async fn init_blinking(s: &mut Sensor<'_>, led: &mut Output<'static>) -> Result<(), SensorError> {
    let result = match select(s.init(), async {
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
    result
}

/// Wait for the next frame from one sensor, log it, and report whether any zone
/// holds a valid target within [`OBJECT_THRESHOLD_MM`].
async fn read_and_report(
    s: &mut Sensor<'_>,
    label: &str,
    grid: usize,
) -> Result<bool, SensorError> {
    s.poll_data_ready().await?;
    let data = s.get_ranging_data().await?;

    let zones = grid * grid;
    let object_near = data.distance_mm[..zones]
        .iter()
        .zip(data.target_status[..zones].iter())
        .any(|(&d, &status)| status == STATUS_VALID && d > 0 && d <= OBJECT_THRESHOLD_MM);

    log_grid(label, &data, grid, object_near);
    Ok(object_near)
}

/// Print one frame's distances as a grid.
fn log_grid(label: &str, data: &ResultsData, grid: usize, object_near: bool) {
    defmt::info!(
        "--- {} distance (mm){} ---",
        label,
        if object_near { "  [OBJECT <200mm]" } else { "" }
    );
    let d = &data.distance_mm;
    for row in 0..grid {
        let b = row * grid;
        match grid {
            4 => defmt::info!("  {} {} {} {}", d[b], d[b + 1], d[b + 2], d[b + 3]),
            _ => defmt::info!(
                "  {} {} {} {} {} {} {} {}",
                d[b],
                d[b + 1],
                d[b + 2],
                d[b + 3],
                d[b + 4],
                d[b + 5],
                d[b + 6],
                d[b + 7],
            ),
        }
    }
}

/// Full power-on sequence: cut the rail, bring it back, release reset, and gate
/// off the second sensor so only the always-on sensor enumerates at the default
/// address. Matches the production board bring-up timing. Used on first start
/// and as the heavier recovery step.
async fn power_on_sequence(
    pwr_en: &mut Output<'static>,
    i2c_reset: &mut Output<'static>,
    tof_lp: &mut Output<'static>,
) {
    pwr_en.set_high(); // power off
    i2c_reset.set_high(); // assert reset
    tof_lp.set_low(); // gate 2nd sensor off
    Timer::after_millis(500).await;
    pwr_en.set_low(); // power on
    Timer::after_millis(100).await;
    i2c_reset.set_low(); // release reset
    Timer::after_millis(1).await;
}

/// Pulse the shared reset line without cutting power — the lighter recovery
/// step. Both sensors lose their loaded firmware and revert to the default
/// address, so the caller must re-run the full two-sensor bring-up afterwards.
async fn reset_pulse(i2c_reset: &mut Output<'static>, tof_lp: &mut Output<'static>) {
    tof_lp.set_low(); // re-gate the 2nd sensor before resetting
    i2c_reset.set_high(); // assert reset
    Timer::after_millis(10).await;
    i2c_reset.set_low(); // release reset
    Timer::after_millis(1).await;
}
