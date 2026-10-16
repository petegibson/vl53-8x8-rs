#![no_std]
#![no_main]

//! VL53L5CX on the evo STM32F412RE board through ST's C ULD: init (timed
//! firmware upload), 4x4 @ 10 Hz, one distance grid logged per second.
//!
//! The C wrapper is VL53L5CX-only: a board fitted with a VL53L8CX fails in
//! `init()` (use `../evo-f412re-embassy`, which autodetects both parts).

use defmt_rtt as _;
use panic_probe as _;

use cortex_m_rt::entry;
use vl53l5cx_uld::{Resolution, NB_TARGET_PER_ZONE};

// --- Configuration ---
const RESOLUTION: Resolution = Resolution::Res4x4;
const RANGING_FREQ_HZ: u8 = 10;
/// Log one frame out of this many (10 Hz ranging -> one grid per second).
const LOG_EVERY_N_FRAMES: u32 = RANGING_FREQ_HZ as u32;

#[entry]
fn main() -> ! {
    defmt::info!("VL53L5CX ULD demo starting (STM32F412RE, I2C1 @ {} kHz)", evo_f412re_uld::I2C_FREQ_HZ / 1000);

    let mut sensor = evo_f412re_uld::board_init();

    // DWT cycle counter times the firmware upload (SysTick is the sensor's
    // delay, and `board_init` already took the core peripherals).
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    defmt::info!("Initialising sensor (firmware upload)...");
    let t0 = cortex_m::peripheral::DWT::cycle_count();
    if let Err(e) = sensor.init() {
        defmt::error!("Sensor init failed: {}", defmt::Debug2Format(&e));
        defmt::error!("Check the board has a VL53L5CX fitted (this demo is L5-only) and is powered");
        loop {
            cortex_m::asm::wfi();
        }
    }
    let elapsed_ms = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(t0)
        / (evo_f412re_uld::SYSCLK_HZ / 1000);
    defmt::info!("Sensor ready after {} ms", elapsed_ms);

    let grid = match RESOLUTION {
        Resolution::Res4x4 => 4usize,
        Resolution::Res8x8 => 8,
    };
    defmt::info!("Configuring: {}x{} @ {} Hz", grid, grid, RANGING_FREQ_HZ);
    sensor.set_resolution(RESOLUTION).unwrap();
    sensor.set_ranging_frequency_hz(RANGING_FREQ_HZ).unwrap();
    sensor.start_ranging().unwrap();

    defmt::info!("Entering measurement loop");
    let mut frames: u32 = 0;
    loop {
        match sensor.check_data_ready() {
            Ok(true) => {}
            Ok(false) => continue,
            Err(e) => {
                defmt::error!("Data ready check error: {}", defmt::Debug2Format(&e));
                continue;
            }
        }

        let data = match sensor.get_ranging_data() {
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

        defmt::info!("--- Distance (mm), frame {} ---", frames);
        // First target of each zone (entry zone * NB_TARGET_PER_ZONE).
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
