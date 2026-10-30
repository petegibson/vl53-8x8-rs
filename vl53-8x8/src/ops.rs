use crate::bus::VlBusAsync;
use crate::firmware::FirmwareSource;
use crate::sensor::{SensorInfo, Variant, NB_TARGET_CONFIG_PATCH};
use embedded_hal::i2c::SevenBitAddress;
use embedded_hal_async::delay::DelayNs;
use crate::consts::*;
use crate::results::ResultsData;
use crate::state::State;
use crate::uld_data::{DEFAULT_CONFIGURATION, DEFAULT_XTALK, GET_NVM_CMD};
#[cfg(feature = "xtalk")]
use crate::uld_data::{CALIBRATE_XTALK, FW_NBTAR_XTALK_INDEX};
use vl53_8x8_types::{
    Error, FirmwareError, VL53L8CX_RESOLUTION_4X4, VL53L8CX_RESOLUTION_8X8,
    VL53L8CX_STATUS_ERROR, XTALK_BUFFER_SIZE,
};
#[cfg(feature = "xtalk")]
use vl53_8x8_types::VL53L8CX_GET_XTALK_CMD;

/// L5-only DCI index for the glare filter (absent on the VL53L8CX).
const VL53L5CX_GLARE_FILTER: u16 = 0xE108;

/// Swap endianness of 4-byte groups in a buffer.
pub fn swap_buffer(buf: &mut [u8], len: usize) {
    for i in (0..len).step_by(4) {
        buf.swap(i, i + 3);
        buf.swap(i + 1, i + 2);
    }
}

/// Poll the sensor for an expected answer. 2s timeout, 10ms intervals.
//
// Porting note on error handling: the ST C driver threads a `status |= cmd()`
// accumulator (bitwise OR-assign — it never short-circuits, so every step runs
// regardless of prior failures) and returns the accumulated status at the end.
// This port uses `?` and returns early on the first error instead — generally
// safer, but a behavioural divergence if you ever chase a quirk where ST's
// "keep going after an error" was load-bearing. The one place it demonstrably
// matters: ST's poll returns a timeout error but its callers ignore it and
// proceed to read/write the response anyway (the sensor usually has valid data
// despite the status), so this fn returns `Ok` on timeout (see the tail below)
// to reproduce that — at the cost of not surfacing the timeout the way ST does.
pub async fn poll_for_answer<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    temp_buf: &mut [u8],
    size: u8,
    pos: u8,
    register: u16,
    mask: u8,
    expected: u8,
) -> Result<(), Error<BUS::Error>> {
    for _attempt in 0..200u8 {
        bus.read(addr, register, &mut temp_buf[..size as usize])
            .await
            .map_err(Error::Bus)?;
        delay.delay_ms(10).await;

        if size >= 4 && temp_buf[2] >= 0x7F {
            #[cfg(feature = "defmt")]
            defmt::warn!(
                "poll MCU error: reg={:#06x} buf={=[u8]:#04x} attempt={}",
                register,
                &temp_buf[..size as usize],
                _attempt,
            );
            return Err(Error::Firmware(FirmwareError::McuError));
        }

        if (temp_buf[pos as usize] & mask) == expected {
            return Ok(());
        }
    }

    // The C driver uses a `status |=` pattern that accumulates errors
    // without stopping. Callers proceed to read/write the response
    // even after a poll timeout, and the sensor often has valid data
    // ready despite not reporting the expected status byte. Match that
    // behaviour here: log a warning but return Ok so the caller continues.
    #[cfg(feature = "defmt")]
    defmt::warn!(
        "poll timeout (continuing): reg={:#06x} pos={} mask={:#04x} expected={:#04x} last buf={=[u8]:#04x}",
        register,
        pos,
        mask,
        expected,
        &temp_buf[..size as usize],
    );
    Ok(())
}

/// Poll for MCU boot. 500ms timeout, 1ms intervals.
pub async fn poll_for_mcu_boot<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
) -> Result<(), Error<BUS::Error>> {
    for _ in 0..500u16 {
        let go2_status0 = bus.read_byte(addr, 0x06).await.map_err(Error::Bus)?;

        if (go2_status0 & 0x80) != 0 {
            let go2_status1 = bus.read_byte(addr, 0x07).await.map_err(Error::Bus)?;
            if (go2_status1 & 0x01) != 0 {
                return Ok(());
            }
        }

        delay.delay_ms(1).await;

        if (go2_status0 & 0x01) != 0 {
            return Ok(());
        }
    }
    Ok(())
}

/// DCI read: send command → poll → read → swap → copy data out.
pub async fn dci_read<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    temp_buf: &mut [u8],
    data: &mut [u8],
    index: u32,
    data_size: u16,
) -> Result<(), Error<BUS::Error>> {
    let rd_size = data_size as u32 + 12;

    let cmd: [u8; 12] = [
        (index >> 8) as u8,
        (index & 0xFF) as u8,
        ((data_size & 0xFF0) >> 4) as u8,
        ((data_size & 0x0F) << 4) as u8,
        0x00,
        0x00,
        0x00,
        0x0F,
        0x00,
        0x02,
        0x00,
        0x08,
    ];

    bus.write(addr, VL53L8CX_UI_CMD_END - 11, &cmd)
        .await
        .map_err(Error::Bus)?;
    poll_for_answer(
        bus, addr, delay, temp_buf, 4, 1, VL53L8CX_UI_CMD_STATUS, 0xFF, 0x03,
    )
    .await?;

    bus.read(
        addr,
        VL53L8CX_UI_CMD_START,
        &mut temp_buf[..rd_size as usize],
    )
    .await
    .map_err(Error::Bus)?;
    swap_buffer(temp_buf, (data_size + 12) as usize);

    // Copy data out, skipping the 4-byte header
    data[..data_size as usize].copy_from_slice(&temp_buf[4..4 + data_size as usize]);

    Ok(())
}

/// DCI write: build packet (header + data + footer) → swap → write → poll.
pub async fn dci_write<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    temp_buf: &mut [u8],
    data: &mut [u8],
    index: u32,
    data_size: u16,
) -> Result<(), Error<BUS::Error>> {
    let headers: [u8; 4] = [
        (index >> 8) as u8,
        (index & 0xFF) as u8,
        ((data_size & 0xFF0) >> 4) as u8,
        ((data_size & 0x0F) << 4) as u8,
    ];

    let total_size = data_size + 8;
    let footer: [u8; 8] = [
        0x00,
        0x00,
        0x00,
        0x0F,
        0x05,
        0x01,
        (total_size >> 8) as u8,
        (total_size & 0xFF) as u8,
    ];

    let address = VL53L8CX_UI_CMD_END - (data_size + 12) + 1;

    // Swap the input data
    swap_buffer(data, data_size as usize);

    // Build packet in temp_buf: header + data + footer
    temp_buf[..4].copy_from_slice(&headers);
    for i in (0..data_size as usize).rev() {
        temp_buf[i + 4] = data[i];
    }
    temp_buf[data_size as usize + 4..data_size as usize + 12].copy_from_slice(&footer);

    bus.write(addr, address, &temp_buf[..(data_size as usize + 12)])
        .await
        .map_err(Error::Bus)?;
    poll_for_answer(
        bus, addr, delay, temp_buf, 4, 1, VL53L8CX_UI_CMD_STATUS, 0xFF, 0x03,
    )
    .await?;

    // Swap data back to host order
    swap_buffer(data, data_size as usize);

    Ok(())
}

/// DCI replace: read → modify → write.
pub async fn dci_replace<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    temp_buf: &mut [u8],
    data: &mut [u8],
    index: u32,
    data_size: u16,
    new_data: &[u8],
    new_data_pos: u16,
) -> Result<(), Error<BUS::Error>> {
    dci_read(bus, addr, delay, temp_buf, data, index, data_size).await?;
    let pos = new_data_pos as usize;
    data[pos..pos + new_data.len()].copy_from_slice(new_data);
    dci_write(bus, addr, delay, temp_buf, data, index, data_size).await?;
    Ok(())
}

/// Read the sensor's factory NVM block into `temp_buf[..VL53L8CX_NVM_DATA_SIZE]`
/// (492 bytes): ST's `GET_NVM_CMD` at 0x2FD8, poll `UI_CMD_STATUS` for 0x02,
/// then read the answer at `UI_CMD_START`. `init_sensor` keeps the first
/// `VL53L8CX_OFFSET_BUFFER_SIZE` (488) bytes as the per-zone offset
/// calibration it sends back with [`send_offset_data`]. `temp_buf` doubles as
/// the poll scratch buffer, so it is overwritten whole.
pub async fn read_nvm_data<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    temp_buf: &mut [u8],
) -> Result<(), Error<BUS::Error>> {
    bus.write(addr, 0x2FD8, &GET_NVM_CMD)
        .await
        .map_err(Error::Bus)?;
    poll_for_answer(
        bus,
        addr,
        delay,
        temp_buf,
        4,
        0,
        VL53L8CX_UI_CMD_STATUS,
        0xFF,
        0x02,
    )
    .await?;
    bus.read(
        addr,
        VL53L8CX_UI_CMD_START,
        &mut temp_buf[..VL53L8CX_NVM_DATA_SIZE as usize],
    )
    .await
    .map_err(Error::Bus)
}

/// Send offset data gathered from NVM.
pub async fn send_offset_data<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    temp_buf: &mut [u8],
    offset_data: &[u8],
    resolution: u8,
) -> Result<(), Error<BUS::Error>> {
    let buf_size = VL53L8CX_OFFSET_BUFFER_SIZE as usize;
    temp_buf[..buf_size].copy_from_slice(&offset_data[..buf_size]);

    if resolution == VL53L8CX_RESOLUTION_4X4 {
        let dss_4x4: [u8; 8] = [0x0F, 0x04, 0x04, 0x00, 0x08, 0x10, 0x10, 0x07];
        temp_buf[0x10..0x18].copy_from_slice(&dss_4x4);
        swap_buffer(temp_buf, buf_size);

        let mut signal_grid = [0u32; 64];
        let mut range_grid = [0i16; 64];

        for i in 0..64 {
            let off = 0x3C + i * 4;
            signal_grid[i] = u32::from_le_bytes([
                temp_buf[off],
                temp_buf[off + 1],
                temp_buf[off + 2],
                temp_buf[off + 3],
            ]);
        }
        for i in 0..64 {
            let off = 0x140 + i * 2;
            range_grid[i] =
                i16::from_le_bytes([temp_buf[off], temp_buf[off + 1]]);
        }

        // Extrapolate 8x8 → 4x4
        for j in 0..4i8 {
            for i in 0..4i8 {
                let dst = (i + 4 * j) as usize;
                let base = (2 * i + 16 * j) as usize;
                signal_grid[dst] = (signal_grid[base]
                    + signal_grid[base + 1]
                    + signal_grid[base + 8]
                    + signal_grid[base + 9])
                    / 4;
                range_grid[dst] = ((range_grid[base] as i32
                    + range_grid[base + 1] as i32
                    + range_grid[base + 8] as i32
                    + range_grid[base + 9] as i32)
                    / 4) as i16;
            }
        }

        // Zero out unused entries
        for i in 0x10..64 {
            signal_grid[i] = 0;
            range_grid[i] = 0;
        }

        // Write back
        for i in 0..64 {
            let off = 0x3C + i * 4;
            let bytes = signal_grid[i].to_le_bytes();
            temp_buf[off..off + 4].copy_from_slice(&bytes);
        }
        for i in 0..64 {
            let off = 0x140 + i * 2;
            let bytes = range_grid[i].to_le_bytes();
            temp_buf[off..off + 2].copy_from_slice(&bytes);
        }
        swap_buffer(temp_buf, buf_size);
    }

    // Shift data left to drop the 8-byte header. Bound to buf_size - 8 so we
    // only move the actual payload [8, buf_size): the ST driver runs to buf_size
    // and reads up to 8 bytes past the offset data (harmless there only because
    // its scratch buffer is oversized and the footer write below overwrites the
    // tail). buf_size - 8 shifts exactly the payload and reads nothing past it.
    for k in 0..(buf_size - 8) {
        temp_buf[k] = temp_buf[k + 8];
    }

    let footer: [u8; 8] = [0x00, 0x00, 0x00, 0x0F, 0x03, 0x01, 0x01, 0xE4];
    temp_buf[0x1E0..0x1E8].copy_from_slice(&footer);

    bus.write(addr, 0x2E18, &temp_buf[..buf_size])
        .await
        .map_err(Error::Bus)?;
    poll_for_answer(
        bus, addr, delay, temp_buf, 4, 1, VL53L8CX_UI_CMD_STATUS, 0xFF, 0x03,
    )
    .await?;

    Ok(())
}

/// Send xtalk data from default configuration or user calibration.
pub async fn send_xtalk_data<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    temp_buf: &mut [u8],
    xtalk_data: &[u8],
    resolution: u8,
) -> Result<(), Error<BUS::Error>> {
    let buf_size = XTALK_BUFFER_SIZE;
    temp_buf[..buf_size].copy_from_slice(&xtalk_data[..buf_size]);

    if resolution == VL53L8CX_RESOLUTION_4X4 {
        let res4x4: [u8; 8] = [0x0F, 0x04, 0x04, 0x17, 0x08, 0x10, 0x10, 0x07];
        let dss_4x4: [u8; 8] = [0x00, 0x78, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08];
        let profile_4x4: [u8; 4] = [0xA0, 0xFC, 0x01, 0x00];

        temp_buf[0x08..0x10].copy_from_slice(&res4x4);
        temp_buf[0x20..0x28].copy_from_slice(&dss_4x4);

        swap_buffer(temp_buf, buf_size);

        let mut signal_grid = [0u32; 64];
        for i in 0..64 {
            let off = 0x34 + i * 4;
            signal_grid[i] = u32::from_le_bytes([
                temp_buf[off],
                temp_buf[off + 1],
                temp_buf[off + 2],
                temp_buf[off + 3],
            ]);
        }

        for j in 0..4i8 {
            for i in 0..4i8 {
                let dst = (i + 4 * j) as usize;
                let base = (2 * i + 16 * j) as usize;
                signal_grid[dst] = (signal_grid[base]
                    + signal_grid[base + 1]
                    + signal_grid[base + 8]
                    + signal_grid[base + 9])
                    / 4;
            }
        }

        for i in 0x10..64 {
            signal_grid[i] = 0;
        }

        for i in 0..64 {
            let off = 0x34 + i * 4;
            let bytes = signal_grid[i].to_le_bytes();
            temp_buf[off..off + 4].copy_from_slice(&bytes);
        }

        swap_buffer(temp_buf, buf_size);

        temp_buf[0x134..0x138].copy_from_slice(&profile_4x4);
        temp_buf[0x078..0x07C].fill(0);
    }

    bus.write(addr, 0x2CF8, &temp_buf[..buf_size])
        .await
        .map_err(Error::Bus)?;
    poll_for_answer(
        bus, addr, delay, temp_buf, 4, 1, VL53L8CX_UI_CMD_STATUS, 0xFF, 0x03,
    )
    .await?;

    Ok(())
}

/// Upload the default configuration (`p_dev->default_configuration` in the
/// ULD, written to 0x2C34 by `vl53l8cx_init` and again by
/// `vl53l8cx_calibrate_xtalk` to reset the sensor after a calibration).
///
/// The shared base is the L8 blob; the variant config overlay (5 bytes for L5)
/// is applied into `temp_buffer` before sending, then the target-count overlay
/// (byte 107, `VL53L8CX_FW_NBTAR_RANGING`, only non-empty for
/// `targets-2/3/4`). The two never overlap.
pub async fn send_default_configuration<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    stdev: &mut State,
    info: &SensorInfo,
) -> Result<(), Error<BUS::Error>> {
    let cfg_len = VL53L8CX_CONFIGURATION_SIZE as usize;
    stdev.temp_buffer[..cfg_len].copy_from_slice(&DEFAULT_CONFIGURATION);
    for &(off, val) in info.config_patch {
        stdev.temp_buffer[off] = val;
    }
    for &(off, val) in NB_TARGET_CONFIG_PATCH {
        stdev.temp_buffer[off] = val;
    }
    bus.write(addr, 0x2C34, &stdev.temp_buffer[..cfg_len])
        .await
        .map_err(Error::Bus)?;
    poll_for_answer(
        bus,
        addr,
        delay,
        &mut stdev.temp_buffer,
        4,
        1,
        VL53L8CX_UI_CMD_STATUS,
        0xFF,
        0x03,
    )
    .await
}

/// Current resolution as the ULD encodes it (`vl53l8cx_get_resolution`):
/// the product of the two zone-grid dimensions in `VL53L8CX_DCI_ZONE_CONFIG`,
/// i.e. `VL53L8CX_RESOLUTION_4X4` (16) or `VL53L8CX_RESOLUTION_8X8` (64).
pub async fn get_resolution<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    temp_buf: &mut [u8],
) -> Result<u8, Error<BUS::Error>> {
    let mut buf = [0u8; 8];
    dci_read(
        bus,
        addr,
        delay,
        temp_buf,
        &mut buf,
        VL53L8CX_DCI_ZONE_CONFIG as u32,
        8,
    )
    .await?;
    Ok(buf[0] * buf[1])
}

/// Set the resolution (`vl53l8cx_set_resolution`): reprogram the DSS and
/// zone configuration for the new grid, then re-send the offset and xtalk
/// calibration data, which are resolution dependent. `resolution` is
/// `VL53L8CX_RESOLUTION_4X4` or `VL53L8CX_RESOLUTION_8X8`; anything else is
/// `InvalidParam`, as in the ULD's `default:` case.
///
/// Because it re-sends `State::xtalk_data`, this is also how a new crosstalk
/// shape (from `set_caldata_xtalk` or `calibrate_xtalk`) reaches the sensor.
pub async fn set_resolution<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    stdev: &mut State,
    resolution: u8,
) -> Result<(), Error<BUS::Error>> {
    // Local buffers for the DCI round-trips: never `stdev.offset_data`, which
    // holds the calibration `send_offset_data` needs below.
    let mut dss_buf = [0u8; 16];
    let mut zone_buf = [0u8; 8];
    let (dss, zone): ([u8; 3], [u8; 4]) = match resolution {
        VL53L8CX_RESOLUTION_4X4 => ([64, 64, 4], [4, 4, 8, 8]),
        VL53L8CX_RESOLUTION_8X8 => ([16, 16, 1], [8, 8, 4, 4]),
        _ => return Err(Error::Firmware(FirmwareError::InvalidParam)),
    };

    dci_read(
        bus, addr, delay, &mut stdev.temp_buffer, &mut dss_buf,
        VL53L8CX_DCI_DSS_CONFIG as u32, 16,
    )
    .await?;
    dss_buf[0x04] = dss[0];
    dss_buf[0x06] = dss[1];
    dss_buf[0x09] = dss[2];
    dci_write(
        bus, addr, delay, &mut stdev.temp_buffer, &mut dss_buf,
        VL53L8CX_DCI_DSS_CONFIG as u32, 16,
    )
    .await?;

    dci_read(
        bus, addr, delay, &mut stdev.temp_buffer, &mut zone_buf,
        VL53L8CX_DCI_ZONE_CONFIG as u32, 8,
    )
    .await?;
    zone_buf[0x00] = zone[0];
    zone_buf[0x01] = zone[1];
    zone_buf[0x04] = zone[2];
    zone_buf[0x05] = zone[3];
    dci_write(
        bus, addr, delay, &mut stdev.temp_buffer, &mut zone_buf,
        VL53L8CX_DCI_ZONE_CONFIG as u32, 8,
    )
    .await?;

    send_offset_data(
        bus, addr, delay, &mut stdev.temp_buffer, &stdev.offset_data, resolution,
    )
    .await?;
    send_xtalk_data(
        bus, addr, delay, &mut stdev.temp_buffer, &stdev.xtalk_data, resolution,
    )
    .await
}

/// Full sensor initialization sequence (translates vl53l8cx_init).
pub async fn init_sensor<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    stdev: &mut State,
    fw: &mut impl FirmwareSource,
    info: &SensorInfo,
) -> Result<(), Error<BUS::Error>> {
    stdev.is_auto_stop_enabled = false;
    let is_l5 = info.variant == Variant::L5;

    // SW reboot sequence
    bus.write_byte(addr, 0x7FFF, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x0009, 0x04).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x000F, 0x40).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x000A, 0x03).await.map_err(Error::Bus)?;
    let _tmp = bus.read_byte(addr, 0x7FFF).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x000C, 0x01).await.map_err(Error::Bus)?;

    bus.write_byte(addr, 0x0101, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x0102, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x010A, 0x01).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x4002, 0x01).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x4002, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x010A, 0x03).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x0103, 0x01).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x000C, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x000F, 0x43).await.map_err(Error::Bus)?;
    delay.delay_ms(1).await;

    bus.write_byte(addr, 0x000F, 0x40).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x000A, 0x01).await.map_err(Error::Bus)?;
    delay.delay_ms(100).await;

    // Wait for sensor booted
    bus.write_byte(addr, 0x7FFF, 0x00).await.map_err(Error::Bus)?;
    poll_for_answer(
        bus,
        addr,
        delay,
        &mut stdev.temp_buffer,
        1,
        0,
        0x06,
        0xFF,
        0x01,
    )
    .await?;

    bus.write_byte(addr, 0x000E, 0x01).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x7FFF, 0x02).await.map_err(Error::Bus)?;

    // Enable FW access. L5 and L8 use a different register write and poll mask
    // here (L5 polls 0x21 & 0x10 == 0x10; L8 writes 0x06 and polls 0x21 & 0xFF).
    if is_l5 {
        bus.write_byte(addr, 0x03, 0x0D).await.map_err(Error::Bus)?;
        bus.write_byte(addr, 0x7FFF, 0x01).await.map_err(Error::Bus)?;
        poll_for_answer(bus, addr, delay, &mut stdev.temp_buffer, 1, 0, 0x21, 0x10, 0x10).await?;
    } else {
        bus.write_byte(addr, 0x7FFF, 0x01).await.map_err(Error::Bus)?;
        bus.write_byte(addr, 0x06, 0x01).await.map_err(Error::Bus)?;
        poll_for_answer(bus, addr, delay, &mut stdev.temp_buffer, 1, 0, 0x21, 0xFF, 0x04).await?;
    }

    bus.write_byte(addr, 0x7FFF, 0x00).await.map_err(Error::Bus)?;

    // Enable host access to GO1
    let _tmp = bus.read_byte(addr, 0x7FFF).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x0C, 0x01).await.map_err(Error::Bus)?;

    // Power ON status
    bus.write_byte(addr, 0x7FFF, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x101, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x102, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x010A, 0x01).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x4002, 0x01).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x4002, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x010A, 0x03).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x103, 0x01).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x400F, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x21A, 0x43).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x21A, 0x03).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x21A, 0x01).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x21A, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x219, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x21B, 0x00).await.map_err(Error::Bus)?;

    // Wake up MCU (L5 toggles GO1 access off and issues two extra 0x20 writes)
    bus.write_byte(addr, 0x7FFF, 0x00).await.map_err(Error::Bus)?;
    let _tmp = bus.read_byte(addr, 0x7FFF).await.map_err(Error::Bus)?;
    if is_l5 {
        bus.write_byte(addr, 0x0C, 0x00).await.map_err(Error::Bus)?;
    }
    bus.write_byte(addr, 0x7FFF, 0x01).await.map_err(Error::Bus)?;
    if is_l5 {
        bus.write_byte(addr, 0x20, 0x07).await.map_err(Error::Bus)?;
        bus.write_byte(addr, 0x20, 0x06).await.map_err(Error::Bus)?;
    }

    // Download firmware into the sensor in 3 banks (32KB + 32KB + 20KB), each
    // written to device address 0 of its own page. Bytes are pulled from the
    // `fw` source in runs; an unpatched source yields a whole bank per call
    // (one zero-copy write), a patched source splits only around its overlay.
    const FW_BANKS: [(u8, usize, usize); 3] =
        [(0x09, 0, 0x8000), (0x0A, 0x8000, 0x10000), (0x0B, 0x10000, 0x15000)];
    for (page, start, bank_end) in FW_BANKS {
        bus.write_byte(addr, 0x7FFF, page).await.map_err(Error::Bus)?;
        let mut off = start;
        while off < bank_end {
            let chunk = fw.chunk(off, bank_end - off);
            bus.write(addr, (off - start) as u16, chunk)
                .await
                .map_err(Error::Bus)?;
            off += chunk.len();
        }
    }
    bus.write_byte(addr, 0x7FFF, 0x01).await.map_err(Error::Bus)?;

    // Check if FW correctly downloaded. L5 verifies by polling; L8 writes 0x06
    // and waits (the CRC is checked separately further down for L8).
    if is_l5 {
        bus.write_byte(addr, 0x7FFF, 0x02).await.map_err(Error::Bus)?;
        bus.write_byte(addr, 0x03, 0x0D).await.map_err(Error::Bus)?;
        bus.write_byte(addr, 0x7FFF, 0x01).await.map_err(Error::Bus)?;
        poll_for_answer(bus, addr, delay, &mut stdev.temp_buffer, 1, 0, 0x21, 0x10, 0x10).await?;
    } else {
        bus.write_byte(addr, 0x7FFF, 0x01).await.map_err(Error::Bus)?;
        bus.write_byte(addr, 0x06, 0x03).await.map_err(Error::Bus)?;
        delay.delay_ms(5).await;
    }
    bus.write_byte(addr, 0x7FFF, 0x00).await.map_err(Error::Bus)?;
    let _tmp = bus.read_byte(addr, 0x7FFF).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x0C, 0x01).await.map_err(Error::Bus)?;

    // Reset MCU and wait boot
    bus.write_byte(addr, 0x7FFF, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x114, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x115, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x116, 0x42).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x117, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x0B, 0x00).await.map_err(Error::Bus)?;
    let _tmp = bus.read_byte(addr, 0x7FFF).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x0C, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x0B, 0x01).await.map_err(Error::Bus)?;

    poll_for_mcu_boot(bus, addr, delay).await?;

    bus.write_byte(addr, 0x7FFF, 0x02).await.map_err(Error::Bus)?;

    // Firmware checksum (only the parts that expose one, e.g. L8).
    if let Some(expected) = info.fw_checksum {
        bus.read(addr, 0x2FFC, &mut stdev.temp_buffer[..4])
            .await
            .map_err(Error::Bus)?;
        swap_buffer(&mut stdev.temp_buffer, 4);
        let crc = u32::from_le_bytes([
            stdev.temp_buffer[0],
            stdev.temp_buffer[1],
            stdev.temp_buffer[2],
            stdev.temp_buffer[3],
        ]);
        if crc != expected {
            return Err(Error::Firmware(FirmwareError::FwChecksumFail));
        }
    }

    // Get offset NVM data
    read_nvm_data(bus, addr, delay, &mut stdev.temp_buffer).await?;
    stdev.offset_data[..VL53L8CX_OFFSET_BUFFER_SIZE as usize]
        .copy_from_slice(&stdev.temp_buffer[..VL53L8CX_OFFSET_BUFFER_SIZE as usize]);
    send_offset_data(
        bus,
        addr,
        delay,
        &mut stdev.temp_buffer,
        &stdev.offset_data,
        VL53L8CX_RESOLUTION_4X4,
    )
    .await?;

    // Set default Xtalk shape
    let default_xtalk = &DEFAULT_XTALK;
    stdev.xtalk_data[..XTALK_BUFFER_SIZE].copy_from_slice(default_xtalk);
    send_xtalk_data(
        bus,
        addr,
        delay,
        &mut stdev.temp_buffer,
        &stdev.xtalk_data,
        VL53L8CX_RESOLUTION_4X4,
    )
    .await?;

    // Send default configuration.
    send_default_configuration(bus, addr, delay, stdev, info).await?;

    // DCI pipe control
    let mut pipe_ctrl: [u8; 4] = [NB_TARGET_PER_ZONE, 0x00, 0x01, 0x00];
    dci_write(
        bus,
        addr,
        delay,
        &mut stdev.temp_buffer,
        &mut pipe_ctrl,
        VL53L8CX_DCI_PIPE_CONTROL as u32,
        4,
    )
    .await?;

    // Multi-target builds also program the firmware's target count: byte 0x0C
    // of the 16-byte `VL53L8CX_DCI_FW_NB_TARGET` block (`vl53l8cx_api.c`
    // lines 396-401, under `#if VL53L8CX_NB_TARGET_PER_ZONE != 1`; the L5 ULD
    // has the identical block). Skipped for one target, as in the ULD.
    if NB_TARGET_PER_ZONE != 1 {
        let mut fw_nb_target = [0u8; 16];
        dci_replace(
            bus,
            addr,
            delay,
            &mut stdev.temp_buffer,
            &mut fw_nb_target,
            VL53L8CX_DCI_FW_NB_TARGET as u32,
            16,
            &[NB_TARGET_PER_ZONE],
            0x0C,
        )
        .await?;
    }

    // Single range
    let mut single_range = 0x01u32.to_le_bytes();
    dci_write(
        bus,
        addr,
        delay,
        &mut stdev.temp_buffer,
        &mut single_range,
        VL53L8CX_DCI_SINGLE_RANGE as u32,
        4,
    )
    .await?;

    // Glare filter (L5 only): two read-modify-writes into the glare-filter DCI
    // block, enabling the filter at byte positions 0x26 then 0x25.
    if info.has_glare_filter {
        let mut glare = [0u8; 40];
        let one = [1u8];
        dci_replace(
            bus, addr, delay, &mut stdev.temp_buffer, &mut glare,
            VL53L5CX_GLARE_FILTER as u32, 40, &one, 0x26,
        )
        .await?;
        dci_replace(
            bus, addr, delay, &mut stdev.temp_buffer, &mut glare,
            VL53L5CX_GLARE_FILTER as u32, 40, &one, 0x25,
        )
        .await?;
    }

    Ok(())
}

/// Check if sensor is alive by reading device and revision IDs.
pub async fn is_alive<BUS: VlBusAsync>(
    bus: &mut BUS,
    addr: SevenBitAddress,
) -> Result<bool, Error<BUS::Error>> {
    bus.write_byte(addr, 0x7FFF, 0x00).await.map_err(Error::Bus)?;
    let device_id = bus.read_byte(addr, 0x00).await.map_err(Error::Bus)?;
    let revision_id = bus.read_byte(addr, 0x01).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x7FFF, 0x02).await.map_err(Error::Bus)?;
    Ok(device_id == 0xF0 && revision_id == 0x0C)
}

/// Check if new ranging data is available.
pub async fn check_data_ready<BUS: VlBusAsync>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    stdev: &mut State,
) -> Result<bool, Error<BUS::Error>> {
    bus.read(addr, 0x0000, &mut stdev.temp_buffer[..4])
        .await
        .map_err(Error::Bus)?;

    if stdev.temp_buffer[0] != stdev.streamcount
        && stdev.temp_buffer[0] != 255
        && stdev.temp_buffer[1] == 0x05
        && (stdev.temp_buffer[2] & 0x05) == 0x05
        && (stdev.temp_buffer[3] & 0x10) == 0x10
    {
        stdev.streamcount = stdev.temp_buffer[0];
        Ok(true)
    } else {
        if (stdev.temp_buffer[3] & 0x80) != 0 {
            return Err(Error::Firmware(stdev.temp_buffer[2].into()));
        }
        Ok(false)
    }
}

/// Read ranging data from the sensor.
/// Copy one result block into a fixed-capacity destination field, rejecting any
/// frame whose block size would overflow it. Guards against a corrupt block
/// header: an oversized length could otherwise write past the field (UB for the
/// raw-pointer destinations) or panic on a slice index. `bh_idx` is only used
/// for diagnostics.
#[inline]
fn copy_result_block<E>(
    dst: *mut u8,
    cap_bytes: usize,
    data: &[u8],
    _bh_idx: u16,
) -> Result<(), Error<E>> {
    if data.len() > cap_bytes {
        #[cfg(feature = "defmt")]
        defmt::warn!(
            "corrupt frame (block too big for field): idx={:#06x} block_size={} field_cap={}",
            _bh_idx,
            data.len(),
            cap_bytes
        );
        return Err(Error::Firmware(FirmwareError::CorruptedFrame));
    }
    // SAFETY: `dst` points to a result field of at least `cap_bytes` bytes, and
    // we copy exactly `data.len() <= cap_bytes` bytes into it.
    unsafe { core::slice::from_raw_parts_mut(dst, data.len()) }.copy_from_slice(data);
    Ok(())
}

// `clamp_negative_distance` is only consulted by the unit conversion that
// `raw-format` removes.
#[cfg_attr(feature = "raw-format", allow(unused_variables))]
pub async fn get_ranging_data<BUS: VlBusAsync>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    stdev: &mut State,
    clamp_negative_distance: bool,
) -> Result<ResultsData, Error<BUS::Error>> {
    let read_size = stdev.data_read_size as usize;
    bus.read(addr, 0x0000, &mut stdev.temp_buffer[..read_size])
        .await
        .map_err(Error::Bus)?;

    stdev.streamcount = stdev.temp_buffer[0];
    swap_buffer(&mut stdev.temp_buffer, read_size);

    let mut results = ResultsData::default();

    // Parse block headers starting at position 16. The data blocks occupy
    // [16, read_size - 4); the final 4 bytes are the frame footer (a tag whose
    // id is checked against the header below), NOT a block — parsing it as one
    // walks off the end. Its first byte is the per-frame streamcount, so the
    // bogus "block size" it decodes to grows once the streamcount reaches 0x10,
    // which is why it only bit ~16 frames (3 s @ 5 Hz) into a run.
    let blocks_end = read_size.saturating_sub(4);
    let mut i = 16usize;
    // Need a full 4-byte header before the footer.
    while i + 4 <= blocks_end {
        // Block header is a 4-byte value: idx(16) | size(12) | type(4)
        let bh = u32::from_le_bytes([
            stdev.temp_buffer[i],
            stdev.temp_buffer[i + 1],
            stdev.temp_buffer[i + 2],
            stdev.temp_buffer[i + 3],
        ]);
        let bh_type = bh & 0x0F;
        let bh_size = ((bh >> 4) & 0x0FFF) as usize;
        let bh_idx = (bh >> 16) as u16;

        let msize = if bh_type > 1 && bh_type < 0x0D {
            bh_type as usize * bh_size
        } else {
            bh_size
        };

        let data_start = i + 4;
        let data_end = data_start + msize;
        // A corrupt/misaligned frame can yield a bogus block size that runs past
        // the buffer; bail out rather than panic on the out-of-range slice.
        if data_end > read_size {
            #[cfg(feature = "defmt")]
            defmt::warn!(
                "corrupt frame: block at i={} idx={:#06x} size={} runs past read_size={}",
                i, bh_idx, msize, read_size
            );
            return Err(Error::Firmware(FirmwareError::CorruptedFrame));
        }
        let data = &stdev.temp_buffer[data_start..data_end];

        match bh_idx {
            VL53L8CX_METADATA_IDX => {
                // Silicon temperature lives at byte 12 of the block; only read it
                // if a corrupt size hasn't left that offset outside the frame.
                if i + 12 < read_size {
                    results.silicon_temp_degc = stdev.temp_buffer[i + 12] as i8;
                }
            }
            #[cfg(not(feature = "disable-ambient"))]
            VL53L8CX_AMBIENT_RATE_IDX => copy_result_block(
                results.ambient_per_spad.as_mut_ptr() as *mut u8,
                core::mem::size_of_val(&results.ambient_per_spad),
                data,
                bh_idx,
            )?,
            #[cfg(not(feature = "disable-spads"))]
            VL53L8CX_SPAD_COUNT_IDX => copy_result_block(
                results.nb_spads_enabled.as_mut_ptr() as *mut u8,
                core::mem::size_of_val(&results.nb_spads_enabled),
                data,
                bh_idx,
            )?,
            #[cfg(not(feature = "disable-nb-targets"))]
            VL53L8CX_NB_TARGET_DETECTED_IDX => copy_result_block(
                results.nb_target_detected.as_mut_ptr(),
                core::mem::size_of_val(&results.nb_target_detected),
                data,
                bh_idx,
            )?,
            #[cfg(not(feature = "disable-signal"))]
            VL53L8CX_SIGNAL_RATE_IDX => copy_result_block(
                results.signal_per_spad.as_mut_ptr() as *mut u8,
                core::mem::size_of_val(&results.signal_per_spad),
                data,
                bh_idx,
            )?,
            #[cfg(not(feature = "disable-range-sigma"))]
            VL53L8CX_RANGE_SIGMA_MM_IDX => copy_result_block(
                results.range_sigma_mm.as_mut_ptr() as *mut u8,
                core::mem::size_of_val(&results.range_sigma_mm),
                data,
                bh_idx,
            )?,
            #[cfg(not(feature = "disable-distance"))]
            VL53L8CX_DISTANCE_IDX => copy_result_block(
                results.distance_mm.as_mut_ptr() as *mut u8,
                core::mem::size_of_val(&results.distance_mm),
                data,
                bh_idx,
            )?,
            #[cfg(not(feature = "disable-reflectance"))]
            VL53L8CX_REFLECTANCE_EST_PC_IDX => copy_result_block(
                results.reflectance.as_mut_ptr(),
                core::mem::size_of_val(&results.reflectance),
                data,
                bh_idx,
            )?,
            #[cfg(not(feature = "disable-target-status"))]
            VL53L8CX_TARGET_STATUS_IDX => copy_result_block(
                results.target_status.as_mut_ptr(),
                core::mem::size_of_val(&results.target_status),
                data,
                bh_idx,
            )?,
            #[cfg(all(feature = "motion", not(feature = "disable-motion-indicator-output")))]
            VL53L8CX_MOTION_DETEC_IDX => copy_result_block(
                &mut results.motion_indicator as *mut _ as *mut u8,
                core::mem::size_of_val(&results.motion_indicator),
                data,
                bh_idx,
            )?,
            _ => {}
        }
        i += 4 + msize;
    }

    #[cfg(not(feature = "raw-format"))]
    {
        // Convert data into their real format
        #[cfg(not(feature = "disable-ambient"))]
        for v in results.ambient_per_spad.iter_mut() {
            *v /= 2048;
        }

        let zones = VL53L8CX_RESOLUTION_8X8 as usize * NB_TARGET_PER_ZONE as usize;
        #[cfg(not(feature = "disable-distance"))]
        for v in results.distance_mm[..zones].iter_mut() {
            *v /= 4;
            // L5 clamps negative distances to 0; L8 leaves them signed.
            if clamp_negative_distance && *v < 0 {
                *v = 0;
            }
        }
        #[cfg(not(feature = "disable-reflectance"))]
        for v in results.reflectance[..zones].iter_mut() {
            *v /= 2;
        }
        #[cfg(not(feature = "disable-range-sigma"))]
        for v in results.range_sigma_mm[..zones].iter_mut() {
            *v /= 128;
        }
        #[cfg(not(feature = "disable-signal"))]
        for v in results.signal_per_spad[..zones].iter_mut() {
            *v /= 2048;
        }

        // Set target status to 255 if no target detected
        #[cfg(not(feature = "disable-nb-targets"))]
        for i in 0..VL53L8CX_RESOLUTION_8X8 as usize {
            if results.nb_target_detected[i] == 0 {
                for j in 0..NB_TARGET_PER_ZONE as usize {
                    #[cfg(not(feature = "disable-target-status"))]
                    {
                        results.target_status[NB_TARGET_PER_ZONE as usize * i + j] = 255;
                    }
                }
            }
        }

        #[cfg(all(feature = "motion", not(feature = "disable-motion-indicator-output")))]
        for v in results.motion_indicator.motion.iter_mut() {
            *v /= 65535;
        }
    }

    // Check header/footer match
    let header_id = ((stdev.temp_buffer[0x8] as u16) << 8) | (stdev.temp_buffer[0x9] as u16);
    let footer_id = ((stdev.temp_buffer[read_size - 4] as u16) << 8)
        | (stdev.temp_buffer[read_size - 3] as u16);
    if header_id != footer_id {
        #[cfg(feature = "defmt")]
        defmt::warn!(
            "corrupt frame: header/footer id mismatch ({:#06x} != {:#06x})",
            header_id, footer_id
        );
        return Err(Error::Firmware(FirmwareError::CorruptedFrame));
    }

    Ok(results)
}

/// Start ranging session.
pub async fn start_ranging<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    stdev: &mut State,
    has_laser_safety_check: bool,
) -> Result<(), Error<BUS::Error>> {
    // Get current resolution (use local buffer to avoid corrupting offset_data)
    let mut zone_buf = [0u8; 8];
    dci_read(
        bus,
        addr,
        delay,
        &mut stdev.temp_buffer,
        &mut zone_buf,
        VL53L8CX_DCI_ZONE_CONFIG as u32,
        8,
    )
    .await?;
    let resolution = zone_buf[0] as u32 * zone_buf[1] as u32;

    stdev.data_read_size = 0;
    stdev.streamcount = 255;

    // Output block header enables (mandatory: meta, common data, start)
    let mut output_bh_enable: [u32; 4] = [0x00000007, 0x00000000, 0x00000000, 0xC0000000];

    // Output block headers: the `*_BH` set for this build's target count
    // (`consts::OUTPUT_BLOCK_HEADERS`, from vl53l8cx_api.h lines 101-151).
    // Format: idx(16) | size(12) | type(4), little-endian u32. The size field
    // is rewritten below for the current resolution and target count.
    let mut output: [u32; 12] = OUTPUT_BLOCK_HEADERS;

    // Enable selected outputs (matching C #ifndef guards)
    #[cfg(not(feature = "disable-ambient"))]
    { output_bh_enable[0] += 8; }
    #[cfg(not(feature = "disable-spads"))]
    { output_bh_enable[0] += 16; }
    #[cfg(not(feature = "disable-nb-targets"))]
    { output_bh_enable[0] += 32; }
    #[cfg(not(feature = "disable-signal"))]
    { output_bh_enable[0] += 64; }
    #[cfg(not(feature = "disable-range-sigma"))]
    { output_bh_enable[0] += 128; }
    #[cfg(not(feature = "disable-distance"))]
    { output_bh_enable[0] += 256; }
    #[cfg(not(feature = "disable-reflectance"))]
    { output_bh_enable[0] += 512; }
    #[cfg(not(feature = "disable-target-status"))]
    { output_bh_enable[0] += 1024; }
    #[cfg(all(feature = "motion", not(feature = "disable-motion-indicator-output")))]
    { output_bh_enable[0] += 2048; }

    // Compute data_read_size
    for i in 0..output.len() {
        let bh = output[i];
        if bh == 0 {
            continue;
        }
        let enable_word = i / 32;
        let enable_bit = i % 32;
        if (output_bh_enable[enable_word] & (1 << enable_bit)) == 0 {
            continue;
        }

        let bh_type = bh & 0x0F;
        let bh_size = ((bh >> 4) & 0x0FFF) as u32;
        let bh_idx = (bh >> 16) as u16;

        if bh_type >= 1 && bh_type < 0x0D {
            let sz = if bh_idx >= 0x54D0 && bh_idx < (0x54D0 + 960) {
                resolution
            } else {
                resolution * NB_TARGET_PER_ZONE as u32
            };
            // Update the output entry with the actual size
            output[i] = (bh_idx as u32) << 16 | (sz & 0x0FFF) << 4 | bh_type;
            stdev.data_read_size += bh_type * sz;
        } else {
            stdev.data_read_size += bh_size;
        }
        stdev.data_read_size += 4;
    }
    stdev.data_read_size += 24;

    // Write output list
    let mut output_bytes = [0u8; 48]; // 12 * 4
    for (i, val) in output.iter().enumerate() {
        output_bytes[i * 4..(i + 1) * 4].copy_from_slice(&val.to_le_bytes());
    }
    dci_write(
        bus,
        addr,
        delay,
        &mut stdev.temp_buffer,
        &mut output_bytes,
        VL53L8CX_DCI_OUTPUT_LIST as u32,
        48,
    )
    .await?;

    // Write header config (data_read_size, nb_outputs)
    let mut header_config = [0u8; 8];
    header_config[..4].copy_from_slice(&stdev.data_read_size.to_le_bytes());
    let nb_outputs = (output.len() as u32 + 1).to_le_bytes();
    header_config[4..8].copy_from_slice(&nb_outputs);
    dci_write(
        bus,
        addr,
        delay,
        &mut stdev.temp_buffer,
        &mut header_config,
        VL53L8CX_DCI_OUTPUT_CONFIG as u32,
        8,
    )
    .await?;

    // Write output enables
    let mut enables_bytes = [0u8; 16];
    for (i, val) in output_bh_enable.iter().enumerate() {
        enables_bytes[i * 4..(i + 1) * 4].copy_from_slice(&val.to_le_bytes());
    }
    dci_write(
        bus,
        addr,
        delay,
        &mut stdev.temp_buffer,
        &mut enables_bytes,
        VL53L8CX_DCI_OUTPUT_ENABLES as u32,
        16,
    )
    .await?;

    // Start xshut bypass (interrupt mode)
    bus.write_byte(addr, 0x7FFF, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x09, 0x05).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x7FFF, 0x02).await.map_err(Error::Bus)?;

    // Start ranging session
    let cmd: [u8; 4] = [0x00, 0x03, 0x00, 0x00];
    bus.write(addr, VL53L8CX_UI_CMD_END - 3, &cmd)
        .await
        .map_err(Error::Bus)?;
    poll_for_answer(
        bus,
        addr,
        delay,
        &mut stdev.temp_buffer,
        4,
        1,
        VL53L8CX_UI_CMD_STATUS,
        0xFF,
        0x03,
    )
    .await?;

    // Verify data_read_size matches firmware expectation
    let mut verify_buf = [0u8; 12];
    dci_read(
        bus,
        addr,
        delay,
        &mut stdev.temp_buffer,
        &mut verify_buf,
        0x5440,
        12,
    )
    .await?;
    let reported_size = u16::from_le_bytes([verify_buf[0x8], verify_buf[0x9]]);
    if reported_size as u32 != stdev.data_read_size {
        return Err(Error::Firmware(FirmwareError::Unknown(VL53L8CX_STATUS_ERROR)));
    }

    // Ensure no laser safety fault (L8 only; the L5 ULD has no such check).
    if has_laser_safety_check {
        let mut safety_buf = [0u8; 8];
        dci_read(
            bus,
            addr,
            delay,
            &mut stdev.temp_buffer,
            &mut safety_buf,
            0xE0C4,
            8,
        )
        .await?;
        if safety_buf[0x6] != 0 {
            return Err(Error::Firmware(FirmwareError::LaserSafety));
        }
    }

    Ok(())
}

/// Stop ranging session.
pub async fn stop_ranging<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    stdev: &mut State,
) -> Result<(), Error<BUS::Error>> {
    // Check auto-stop flag
    bus.read(addr, 0x2FFC, &mut stdev.temp_buffer[..4])
        .await
        .map_err(Error::Bus)?;
    let auto_stop_flag = u32::from_le_bytes([
        stdev.temp_buffer[0],
        stdev.temp_buffer[1],
        stdev.temp_buffer[2],
        stdev.temp_buffer[3],
    ]);

    if auto_stop_flag != 0x4FF && !stdev.is_auto_stop_enabled {
        bus.write_byte(addr, 0x7FFF, 0x00).await.map_err(Error::Bus)?;

        // Provoke MCU stop
        bus.write_byte(addr, 0x15, 0x16).await.map_err(Error::Bus)?;
        bus.write_byte(addr, 0x14, 0x01).await.map_err(Error::Bus)?;

        // Poll for G02 status 0 MCU stop (5s timeout, 10ms intervals)
        for _ in 0..500u16 {
            let tmp = bus.read_byte(addr, 0x06).await.map_err(Error::Bus)?;
            if (tmp & 0x80) != 0 {
                break;
            }
            delay.delay_ms(10).await;
        }
    }

    // Check GO2 status 1
    let tmp = bus.read_byte(addr, 0x06).await.map_err(Error::Bus)?;
    if (tmp & 0x80) != 0 {
        let tmp2 = bus.read_byte(addr, 0x07).await.map_err(Error::Bus)?;
        if tmp2 != 0x84 && tmp2 != 0x85 {
            return Err(Error::Firmware(tmp2.into()));
        }
    }

    // Undo MCU stop
    bus.write_byte(addr, 0x7FFF, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x14, 0x00).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x15, 0x00).await.map_err(Error::Bus)?;

    // Stop xshut bypass
    bus.write_byte(addr, 0x09, 0x04).await.map_err(Error::Bus)?;
    bus.write_byte(addr, 0x7FFF, 0x02).await.map_err(Error::Bus)?;

    Ok(())
}

/// Read the detection-thresholds auto-stop flag: byte 3 of the 4-byte
/// `VL53L8CX_DCI_PIPE_CONTROL` block (`vl53l8cx_plugin_detection_thresholds.c`,
/// `vl53l8cx_get_detection_thresholds_auto_stop`, lines 162-173).
#[cfg(feature = "thresholds")]
pub async fn get_detection_thresholds_auto_stop<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    stdev: &mut State,
) -> Result<bool, Error<BUS::Error>> {
    let mut pipe_ctrl = [0u8; 4];
    dci_read(
        bus,
        addr,
        delay,
        &mut stdev.temp_buffer,
        &mut pipe_ctrl,
        VL53L8CX_DCI_PIPE_CONTROL as u32,
        4,
    )
    .await?;
    Ok(pipe_ctrl[0x3] != 0)
}

/// Write the detection-thresholds auto-stop flag (byte 3 of
/// `VL53L8CX_DCI_PIPE_CONTROL`) and mirror it into `State::is_auto_stop_enabled`,
/// which `stop_ranging` consults (`vl53l8cx_plugin_detection_thresholds.c`,
/// `vl53l8cx_set_detection_thresholds_auto_stop`, lines 175-187).
#[cfg(feature = "thresholds")]
pub async fn set_detection_thresholds_auto_stop<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    stdev: &mut State,
    enabled: bool,
) -> Result<(), Error<BUS::Error>> {
    let mut pipe_ctrl = [0u8; 4];
    dci_replace(
        bus,
        addr,
        delay,
        &mut stdev.temp_buffer,
        &mut pipe_ctrl,
        VL53L8CX_DCI_PIPE_CONTROL as u32,
        4,
        &[enabled as u8],
        0x03,
    )
    .await?;
    stdev.is_auto_stop_enabled = enabled;
    Ok(())
}

// --- Xtalk plugin (`vl53l8cx_plugin_xtalk.c`) ------------------------------------

/// Read the sensor's current crosstalk shape into `out` (the shared tail of
/// `vl53l8cx_get_caldata_xtalk` and `vl53l8cx_calibrate_xtalk`): send
/// `VL53L8CX_GET_XTALK_CMD` at 0x2FB8, wait for it, read the 780-byte answer
/// from `VL53L8CX_UI_CMD_START`, and keep bytes 8..776 followed by the fixed
/// 8-byte footer. `out` must hold `XTALK_BUFFER_SIZE` bytes. The caller is
/// responsible for the resolution being 8x8, as the ULD callers are.
#[cfg(feature = "xtalk")]
pub async fn fetch_xtalk_data<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    temp_buf: &mut [u8],
    out: &mut [u8],
) -> Result<(), Error<BUS::Error>> {
    bus.write(addr, 0x2FB8, &VL53L8CX_GET_XTALK_CMD)
        .await
        .map_err(Error::Bus)?;
    poll_for_answer(
        bus, addr, delay, temp_buf, 4, 1, VL53L8CX_UI_CMD_STATUS, 0xFF, 0x03,
    )
    .await?;
    bus.read(
        addr,
        VL53L8CX_UI_CMD_START,
        &mut temp_buf[..XTALK_BUFFER_SIZE + 4],
    )
    .await
    .map_err(Error::Bus)?;

    const FOOTER: [u8; 8] = [0x00, 0x00, 0x00, 0x0F, 0x00, 0x01, 0x03, 0x04];
    out[..XTALK_BUFFER_SIZE - 8].copy_from_slice(&temp_buf[8..XTALK_BUFFER_SIZE]);
    out[XTALK_BUFFER_SIZE - 8..XTALK_BUFFER_SIZE].copy_from_slice(&FOOTER);
    Ok(())
}

/// Argument check of `vl53l8cx_calibrate_xtalk`: target reflectance 1..=99 %,
/// 1..=16 samples, target distance 600..=3000 mm.
#[cfg(feature = "xtalk")]
pub const fn xtalk_params_valid(reflectance_percent: u8, nb_samples: u8, distance_mm: u16) -> bool {
    reflectance_percent >= 1
        && reflectance_percent <= 99
        && nb_samples >= 1
        && nb_samples <= 16
        && distance_mm >= 600
        && distance_mm <= 3000
}

/// Port of the plugin's private `_vl53l8cx_program_output_config`: program the
/// output list the firmware produces during a crosstalk calibration (17
/// blocks, all enabled) and compute `State::data_read_size` for it, the same
/// way `start_ranging` does for a ranging session.
#[cfg(feature = "xtalk")]
async fn program_xtalk_output_config<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    stdev: &mut State,
) -> Result<(), Error<BUS::Error>> {
    let resolution = get_resolution(bus, addr, delay, &mut stdev.temp_buffer).await? as u32;
    stdev.data_read_size = 0;

    // Enable mandatory output (meta and common data)
    let output_bh_enable: [u32; 4] = [0x0001FFFF, 0x00000000, 0x00000000, 0xC0000000];

    // Send addresses of possible output (`output[]` in the C; block headers,
    // idx(16) | size(12) | type(4), little-endian).
    let mut output: [u32; 17] = [
        0x0000000D, 0x54000040, 0x9FD800C0, 0x9FE40140, 0x9FF80040, 0x9FFC0404,
        0xA0FC0100, 0xA10C0100, 0xA11C00C0, 0xA1280902, 0xA2480040, 0xA24C0081,
        0xA2540081, 0xA25C0081, 0xA2640081, 0xA26C0084, 0xA28C0082,
    ];

    // Update data size
    for i in 0..output.len() {
        let bh = output[i];
        if bh == 0 || (output_bh_enable[i / 32] & (1 << (i % 32))) == 0 {
            continue;
        }
        let bh_type = bh & 0x0F;
        let bh_size = (bh >> 4) & 0x0FFF;
        let bh_idx = (bh >> 16) as u16;
        if bh_type >= 1 && bh_type < 0x0D {
            let sz = if bh_idx >= 0x54D0 && bh_idx < (0x54D0 + 960) {
                resolution
            } else {
                resolution * NB_TARGET_PER_ZONE as u32
            };
            output[i] = (bh_idx as u32) << 16 | (sz & 0x0FFF) << 4 | bh_type;
            stdev.data_read_size += bh_type * sz;
        } else {
            stdev.data_read_size += bh_size;
        }
        stdev.data_read_size += 4;
    }
    stdev.data_read_size += 24;

    let mut output_bytes = [0u8; 17 * 4];
    for (i, val) in output.iter().enumerate() {
        output_bytes[i * 4..(i + 1) * 4].copy_from_slice(&val.to_le_bytes());
    }
    dci_write(
        bus, addr, delay, &mut stdev.temp_buffer, &mut output_bytes,
        VL53L8CX_DCI_OUTPUT_LIST as u32, 17 * 4,
    )
    .await?;

    // header_config = { data_read_size, nb_outputs + 1 }
    let mut header_config = [0u8; 8];
    header_config[..4].copy_from_slice(&stdev.data_read_size.to_le_bytes());
    header_config[4..].copy_from_slice(&(output.len() as u32 + 1).to_le_bytes());
    dci_write(
        bus, addr, delay, &mut stdev.temp_buffer, &mut header_config,
        VL53L8CX_DCI_OUTPUT_CONFIG as u32, 8,
    )
    .await?;

    let mut enables_bytes = [0u8; 16];
    for (i, val) in output_bh_enable.iter().enumerate() {
        enables_bytes[i * 4..(i + 1) * 4].copy_from_slice(&val.to_le_bytes());
    }
    dci_write(
        bus, addr, delay, &mut stdev.temp_buffer, &mut enables_bytes,
        VL53L8CX_DCI_OUTPUT_ENABLES as u32, 16,
    )
    .await
}

/// Run a crosstalk calibration: port of `vl53l8cx_calibrate_xtalk`
/// (`vl53l8cx_plugin_xtalk.c`), identical in the VL53L5CX ULD apart from the
/// two command-table bytes carried by `SensorInfo::calibrate_xtalk_patch`.
///
/// Sequence, as in the C: read the resolution and switch to 8x8; upload the
/// `CALIBRATE_XTALK` command table at 0x2C28 (variant overlay and, for
/// `targets-N`, the `FW_NBTAR_XTALK` byte applied); write the target distance
/// (x4), reflectance (x16) and sample count into `VL53L8CX_DCI_CAL_CFG`;
/// program the calibration output list; start the session and wait up to
/// 400 x 50 ms for the firmware to finish; read the resulting shape into
/// `State::xtalk_data`; reset the sensor with the default configuration; and
/// restore the resolution, which re-sends the (new) `xtalk_data`.
///
/// Split with the caller: the ULD brackets this body with a save and restore
/// of ranging frequency, integration time, sharpener, target order, xtalk
/// margin and ranging mode through its public getters/setters. Those are
/// `Sensor` methods here, so `Sensor::calibrate_xtalk` does that bracket
/// around this function, in the ULD's order (resolution is restored first,
/// by the caller first, then the six scalars; both are restored afterwards).
///
/// Two deliberate divergences from the C's `status |=` accumulation:
/// arguments are validated before any bus traffic (the ULD reads the
/// configuration first and, on bad arguments, still runs the readback and
/// restore), and a calibration that never completes is reported as
/// `FirmwareError::Timeout` rather than the generic `VL53L8CX_STATUS_ERROR`.
/// `FirmwareError::XtalkFailed` means the firmware found the coverglass too
/// good to calibrate (the ULD then reloads the default shape, as done here).
/// In both failure cases the reset and restore still run, so the sensor is
/// left configured as before.
#[cfg(feature = "xtalk")]
pub async fn calibrate_xtalk<BUS: VlBusAsync, D: DelayNs>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    delay: &mut D,
    stdev: &mut State,
    info: &SensorInfo,
    resolution: u8,
    reflectance_percent: u8,
    nb_samples: u8,
    distance_mm: u16,
) -> Result<(), Error<BUS::Error>> {
    if !xtalk_params_valid(reflectance_percent, nb_samples, distance_mm) {
        return Err(Error::Firmware(FirmwareError::InvalidParam));
    }

    // `resolution` is the caller's saved value: the ULD reads it FIRST, before
    // the other settings, and the differential test against ST's code checks
    // that order (KNOWN_DIVERGENCES #4). It is restored at the end.
    set_resolution(bus, addr, delay, stdev, VL53L8CX_RESOLUTION_8X8).await?;

    // Send Xtalk calibration buffer: the shared (L8) table with the variant
    // overlay, then the target-count byte (`VL53L8CX_FW_NBTAR_XTALK`, which
    // the C resolves at compile time). The two never overlap.
    let cmd_len = CALIBRATE_XTALK.len();
    stdev.temp_buffer[..cmd_len].copy_from_slice(&CALIBRATE_XTALK);
    for &(off, val) in info.calibrate_xtalk_patch {
        stdev.temp_buffer[off] = val;
    }
    if NB_TARGET_PER_ZONE != 1 {
        stdev.temp_buffer[FW_NBTAR_XTALK_INDEX] = NB_TARGET_PER_ZONE;
    }
    bus.write(addr, 0x2C28, &stdev.temp_buffer[..cmd_len])
        .await
        .map_err(Error::Bus)?;
    poll_for_answer(
        bus, addr, delay, &mut stdev.temp_buffer, 4, 1, VL53L8CX_UI_CMD_STATUS, 0xFF, 0x03,
    )
    .await?;

    // Format input arguments and update the required fields of the
    // 8-byte calibration config block.
    let distance = (distance_mm * 4).to_le_bytes();
    let reflectance = (reflectance_percent as u16 * 16).to_le_bytes();
    let mut cal_cfg = [0u8; 8];
    dci_replace(
        bus, addr, delay, &mut stdev.temp_buffer, &mut cal_cfg,
        VL53L8CX_DCI_CAL_CFG as u32, 8, &distance, 0x00,
    )
    .await?;
    dci_replace(
        bus, addr, delay, &mut stdev.temp_buffer, &mut cal_cfg,
        VL53L8CX_DCI_CAL_CFG as u32, 8, &reflectance, 0x02,
    )
    .await?;
    dci_replace(
        bus, addr, delay, &mut stdev.temp_buffer, &mut cal_cfg,
        VL53L8CX_DCI_CAL_CFG as u32, 8, &[nb_samples], 0x04,
    )
    .await?;

    // Program output for Xtalk calibration
    program_xtalk_output_config(bus, addr, delay, stdev).await?;

    // Start ranging session (no xshut bypass, unlike `start_ranging`; the
    // firmware stops by itself when the calibration is done).
    let cmd: [u8; 4] = [0x00, 0x03, 0x00, 0x00];
    bus.write(addr, VL53L8CX_UI_CMD_END - 3, &cmd)
        .await
        .map_err(Error::Bus)?;
    poll_for_answer(
        bus, addr, delay, &mut stdev.temp_buffer, 4, 1, VL53L8CX_UI_CMD_STATUS, 0xFF, 0x03,
    )
    .await?;

    // Wait for end of calibration: the status word at 0x0000 leaves 0xFF
    // when the firmware is done. Up to 400 x 50 ms = 20 s.
    let mut outcome: Result<(), FirmwareError> = Ok(());
    let mut timeout = 0u16;
    loop {
        bus.read(addr, 0x0000, &mut stdev.temp_buffer[..4])
            .await
            .map_err(Error::Bus)?;
        if stdev.temp_buffer[0] != VL53L8CX_STATUS_ERROR {
            // Coverglass too good for Xtalk calibration
            if stdev.temp_buffer[2] >= 0x7F && (stdev.temp_buffer[3] & 0x80) != 0 {
                stdev.xtalk_data[..XTALK_BUFFER_SIZE].copy_from_slice(&DEFAULT_XTALK);
                outcome = Err(FirmwareError::XtalkFailed);
            }
            break;
        } else if timeout >= 400 {
            outcome = Err(FirmwareError::Timeout);
            break;
        } else {
            timeout += 1;
            delay.delay_ms(50).await;
        }
    }

    // Save Xtalk data into the Xtalk buffer (the C does this unconditionally,
    // overwriting the default shape it just loaded on failure).
    fetch_xtalk_data(
        bus, addr, delay, &mut stdev.temp_buffer, &mut stdev.xtalk_data,
    )
    .await?;

    // Reset default buffer
    send_default_configuration(bus, addr, delay, stdev, info).await?;

    // Reset initial configuration (resolution; this re-sends `xtalk_data`).
    // The caller restores the remaining settings.
    set_resolution(bus, addr, delay, stdev, resolution).await?;

    outcome.map_err(Error::Firmware)
}

/// Set I2C address.
pub async fn set_i2c_address<BUS: VlBusAsync>(
    bus: &mut BUS,
    addr: SevenBitAddress,
    new_addr: SevenBitAddress,
) -> Result<(), Error<BUS::Error>> {
    bus.write_byte(addr, 0x7FFF, 0x00).await.map_err(Error::Bus)?;
    // Register 0x04 takes the 7-bit address directly.
    bus.write_byte(addr, 0x04, new_addr).await.map_err(Error::Bus)?;
    bus.write_byte(new_addr, 0x7FFF, 0x02).await.map_err(Error::Bus)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::convert::Infallible;
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    /// Minimal executor: the mock bus never yields, so this just polls to ready.
    fn block_on<F: Future>(fut: F) -> F::Output {
        fn raw() -> RawWaker {
            RawWaker::new(core::ptr::null(), &VTABLE)
        }
        static VTABLE: RawWakerVTable = RawWakerVTable::new(|_| raw(), |_| {}, |_| {}, |_| {});
        let waker = unsafe { Waker::from_raw(raw()) };
        let mut cx = Context::from_waker(&waker);
        let mut fut = pin!(fut);
        loop {
            if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
                return v;
            }
        }
    }

    /// Bus that returns a fixed result frame for the ranging read.
    struct FrameBus<'a>(&'a [u8]);
    impl VlBusAsync for FrameBus<'_> {
        type Error = Infallible;
        async fn write(&mut self, _: SevenBitAddress, _: u16, _: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }
        async fn read(&mut self, _: SevenBitAddress, _: u16, buf: &mut [u8]) -> Result<(), Self::Error> {
            let n = buf.len().min(self.0.len());
            buf[..n].copy_from_slice(&self.0[..n]);
            Ok(())
        }
    }

    /// Build a minimal well-formed result frame: a 16-byte header, no data
    /// blocks, and a 4-byte footer whose first byte carries `streamcount`. The
    /// header/footer id (bytes 8..10) is set to match. Returned in the on-wire
    /// (pre-swap) byte order the driver reads.
    fn make_frame(streamcount: u8) -> [u8; 32] {
        let read_size = 32;
        let mut f = [0u8; 32];
        // Header/footer id must match for the frame to be accepted.
        f[8] = streamcount;
        f[read_size - 4] = streamcount;
        swap_buffer(&mut f, read_size); // post-swap layout -> on-wire order
        f
    }

    /// Bus that fails the test on any traffic.
    #[cfg(feature = "xtalk")]
    struct PanicBus;
    #[cfg(feature = "xtalk")]
    impl VlBusAsync for PanicBus {
        type Error = Infallible;
        async fn write(&mut self, _: SevenBitAddress, reg: u16, _: &[u8]) -> Result<(), Self::Error> {
            panic!("unexpected bus write to {reg:#06x}");
        }
        async fn read(&mut self, _: SevenBitAddress, reg: u16, _: &mut [u8]) -> Result<(), Self::Error> {
            panic!("unexpected bus read from {reg:#06x}");
        }
    }

    #[cfg(feature = "xtalk")]
    struct NoDelay;
    #[cfg(feature = "xtalk")]
    impl DelayNs for NoDelay {
        async fn delay_ns(&mut self, _: u32) {}
    }

    /// The ULD's argument ranges, inclusive at both ends.
    #[cfg(feature = "xtalk")]
    #[test]
    fn xtalk_param_ranges_match_uld() {
        assert!(xtalk_params_valid(1, 1, 600));
        assert!(xtalk_params_valid(99, 16, 3000));
        assert!(xtalk_params_valid(3, 4, 600)); // ST's recommended setup
        assert!(!xtalk_params_valid(0, 4, 600));
        assert!(!xtalk_params_valid(100, 4, 600));
        assert!(!xtalk_params_valid(5, 0, 600));
        assert!(!xtalk_params_valid(5, 17, 600));
        assert!(!xtalk_params_valid(5, 4, 599));
        assert!(!xtalk_params_valid(5, 4, 3001));
    }

    /// Invalid arguments are rejected before any bus traffic.
    #[cfg(feature = "xtalk")]
    #[test]
    fn calibrate_xtalk_rejects_bad_params_without_touching_the_bus() {
        for (r, n, d) in [(0u8, 4u8, 600u16), (100, 4, 600), (5, 0, 600), (5, 17, 600), (5, 4, 599), (5, 4, 3001)] {
            let mut stdev = State::new();
            let result = block_on(calibrate_xtalk(
                &mut PanicBus, 0x29, &mut NoDelay, &mut stdev, Variant::L8.info(),
                VL53L8CX_RESOLUTION_4X4, r, n, d,
            ));
            assert!(
                matches!(result, Err(Error::Firmware(FirmwareError::InvalidParam))),
                "({r}, {n}, {d}) -> {result:?}"
            );
        }
    }

    /// Regression: the trailing footer word must not be parsed as a block. Its
    /// first byte is the per-frame streamcount; once it reaches 0x10 the decoded
    /// "block size" became non-zero and walked off the end of the frame.
    #[test]
    fn footer_streamcount_is_not_parsed_as_a_block() {
        for sc in [0x00u8, 0x0F, 0x10, 0x11, 0x80, 0xFF] {
            let frame = make_frame(sc);
            let mut stdev = State::new();
            stdev.data_read_size = 32;
            let mut bus = FrameBus(&frame);
            let result = block_on(get_ranging_data(&mut bus, 0x29, &mut stdev, false));
            assert!(
                result.is_ok(),
                "streamcount {:#04x} parsed the footer as a block: {:?}",
                sc,
                result.err()
            );
        }
    }
}
