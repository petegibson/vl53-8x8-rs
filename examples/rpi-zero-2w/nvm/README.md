# Factory NVM dumps

Output of `just report-pi-nvm [i2c-bus]` (`nvm_report` in
`tests/sensor_async.rs`), one file per module: the 492-byte block that
`Sensor::read_nvm_data` returns, which is what ST's `GET_NVM_CMD` fetches at
init and whose first 488 bytes the drivers send back to the sensor as its
per-zone offset calibration. ST documents none of this; the notes below are
read off the dumps and off what `send_offset_data` does with the block.

| File | Module | Board |
| ---- | ------ | ----- |
| `vl53l5cx-pololu-2026-10-30.txt` | VL53L5CX | Pololu carrier, Pi I2C1, LPn GPIO5 |
| `vl53l7cx-pololu-2026-10-30.txt` | VL53L7CX (detected as L5) | Pololu carrier, Pi I2C1, LPn GPIO6 |
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

Everything outside the two grids is byte-identical across the L5, L7 and
L8. There is no serial number, module id, lens id or product code in the
block: the headers are fixed and the payload is calibration only. The ULD
exposes no other NVM read, so a per-device identity, if the silicon has one,
is not reachable through it.

## What differs between modules

| | VL53L5CX | VL53L7CX | VL53L8CX |
| --- | --- | --- | --- |
| range offsets, zones 0–14 | −112 … −168 | −111 … −178 | −89 … −134 |
| 16th entry | −286 | −281 | −258 |
| signal grid (16 zones) | 1.29 M … 1.80 M | 674 k … 884 k | 3.97 M … 5.25 M |

The range offsets are per-device and overlap between parts. The 16th `i16`
is far from the other fifteen on all three modules, so it is probably not a
zone offset but another field sharing the block.

The signal grid does separate the three modules, and in the ratios the
optics predict: the L7's calibration signal is about half the L5's (its 60°
lens spreads the emitter over roughly twice the area of the 45° lens), and
the L8's about 3x the L5's (brighter emitter, longer rated range). With one
module of each that is suggestive, not a detector: it is a factory
measurement that also varies module to module, and the L5/L7 split would
have to be confirmed on a batch of each before a threshold could be trusted.
The drivers do not use it.
