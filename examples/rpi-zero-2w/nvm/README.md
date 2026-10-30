# Factory NVM dumps

Output of `just report-pi-nvm [i2c-bus]` (`nvm_report` in
`tests/sensor_async.rs`), one file per module: the 492-byte block that
`Sensor::read_nvm_data` returns, which is what ST's `GET_NVM_CMD` fetches at
init and whose first 488 bytes the drivers send back to the sensor as its
per-zone offset calibration. ST documents none of this; the notes below are
read off the dumps and off what `send_offset_data` does with the block.

| File | Module | Board |
| ---- | ------ | ----- |
| `vl53l7cx-pololu-2026-10-30.txt` | VL53L7CX (detected as L5) | Pololu carrier, Pi I2C1 |
| `vl53l8cx-pololu-2026-10-30.txt` | VL53L8CX | Pololu carrier, Pi SPI0 |

## Layout (raw bytes, before ST's word swap)

The block is a sequence of DCI-style sub-blocks, each introduced by a 4-byte
header `ii ii ss ss` (index, then size/type), big-endian:

| Offset | Bytes | Content |
| ------ | ----- | ------- |
| 0x000 | `54 00 00 40 00 04 00 05` | outer header, same on both parts |
| 0x008 | `9e 14 00 c0` + `00 d0 01 90` | sub-block 0x9E14 |
| 0x010 | `0f 04 04 00 08 10 10 07` | DSS settings; identical to the `dss_4x4` bytes ST overwrites for 4x4 |
| 0x018 | `9e 20 01 40` + zeros | sub-block 0x9E20 |
| 0x030 | `9e 34 00 40` + zeros | sub-block 0x9E34 |
| 0x038 | `9e 38 04 04` + 16 × `u32` | **signal grid**: per-zone signal at factory calibration, 4x4 (entries 16..63 zero) |
| 0x13c | `9f 38 04 02` + 16 × `i16` | **range offsets**: per-zone distance offset, 4x4 (entries 16..63 zero) |
| 0x1c0 | `9f b8 01 00` + zeros | sub-block 0x9FB8 |
| 0x1d4 | `9f c8 01 00` + zeros | sub-block 0x9FC8 |
| 0x1e8 | `00 00 00 0f` | tail, not re-sent by ST |

Everything outside the two grids is byte-identical between the L7 and the
L8. There is no serial number, module id, lens id or product code in the
block: the headers are fixed and the payload is calibration only. The ULD
exposes no other NVM read, so a per-device identity, if the silicon has one,
is not reachable through it.

## What differs between modules

| | VL53L7CX | VL53L8CX |
| --- | --- | --- |
| range offsets (16 zones) | −111 … −178, one outlier −281 | −89 … −134, one outlier −258 |
| signal grid (16 zones) | 674 k … 884 k | 3.97 M … 5.25 M |

Both are per-device calibration values and will differ between two modules
of the same part as well, so neither grid can identify a part type without
sampling many modules. The L8's calibration signal is about 6x the L7's,
consistent with the L8's longer rated range and with the ~4x higher
signal per SPAD it returned on the Pi under the same box.

Add a VL53L5CX dump when one is on the Pi (`just report-pi-nvm i2c-bus`) and
extend the table.
