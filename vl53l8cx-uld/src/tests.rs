//! Host unit tests: drive ST's real C ULD through the platform macro and a
//! mock I2C bus, with no hardware.

extern crate std;
use super::*;
use std::vec::Vec;

/// Mock I2C bus. Records every (register, bytes) write and answers reads of
/// register 0x0000/0x0001 with a device/revision id; everything else reads
/// as zero.
pub struct MockI2c {
    pub revision_id: u8,
    pub writes: Vec<(u16, Vec<u8>)>,
    pub reads: Vec<(u16, usize)>,
}

impl embedded_hal::i2c::ErrorType for MockI2c {
    type Error = core::convert::Infallible;
}

impl I2c for MockI2c {
    fn transaction(
        &mut self,
        _address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        use embedded_hal::i2c::Operation;
        let mut reg: Option<u16> = None;
        for op in operations.iter_mut() {
            match op {
                Operation::Write(bytes) => match reg {
                    None => reg = Some(u16::from_be_bytes([bytes[0], bytes[1]])),
                    Some(r) => self.writes.push((r, bytes.to_vec())),
                },
                Operation::Read(buf) => {
                    let r = reg.expect("read without register");
                    self.reads.push((r, buf.len()));
                    for (i, b) in buf.iter_mut().enumerate() {
                        *b = match r as usize + i {
                            0 => 0xF0, // device id
                            1 => self.revision_id,
                            _ => 0x00,
                        };
                    }
                }
            }
        }
        Ok(())
    }
}

pub struct MockDelay;
impl DelayNs for MockDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}

crate::bind_platform_driver!(MockI2c, MockDelay);

fn sensor(revision_id: u8) -> Vl53l8cx<MockI2c, MockDelay> {
    Vl53l8cx::new(MockI2c { revision_id, writes: Vec::new(), reads: Vec::new() }, MockDelay)
}

/// Drives ST's real `vl53l8cx_is_alive` through the platform macro and the
/// mock bus: proves the FFI plumbing end to end on the host.
#[test]
fn is_alive_through_c_uld_and_platform_macro() {
    let mut s = sensor(0x0C);
    assert_eq!(s.is_alive(), Ok(true));
    // vl53l8cx_is_alive: page select 0x7FFF=0, read 0x0000 and 0x0001,
    // page select 0x7FFF=2.
    assert_eq!(
        s.comms.bus.writes,
        std::vec![(0x7FFF, std::vec![0x00]), (0x7FFF, std::vec![0x02])]
    );
    assert_eq!(s.comms.bus.reads, std::vec![(0x0000, 1), (0x0001, 1)]);
}

/// A VL53L5CX (revision 0x02) is not "alive" to the L8 ULD.
#[test]
fn is_alive_rejects_the_l5_revision_id() {
    let mut s = sensor(0x02);
    assert_eq!(s.is_alive(), Ok(false));
}

/// Regression test for the threshold double-scaling bug found by the
/// differential tests: a 500 mm threshold must reach the DCI write as
/// 500 * 4 = 2000 (ST's ULD scales once), not 8000.
#[cfg(feature = "thresholds")]
#[test]
fn detection_thresholds_are_scaled_once_on_the_wire() {
    let mut s = sensor(0x0C);
    let mut thresholds: [DetectionThreshold; VL53L8CX_NB_THRESHOLDS as usize] =
        unsafe { core::mem::zeroed() };
    thresholds[0] = DetectionThreshold {
        low_threshold: 500,
        high_threshold: 0,
        measurement: ThresholdMeasurement::DistanceMm,
        threshold_type: ThresholdType::LessThanEqualMin,
        zone_num: 0,
        mathematic_operation: ThresholdOperation::None,
    };
    // The mock never answers the DCI handshake, so the call reports a
    // firmware status; only the bytes it put on the bus matter here.
    let _ = s.set_detection_thresholds(thresholds);
    let dci = s
        .comms
        .bus
        .writes
        .iter()
        .find(|(_, bytes)| bytes.len() >= 8 && bytes[..4] == [0xb6, 0xe8, 0x30, 0x00])
        .expect("DCI write of the threshold table");
    let low = i32::from_be_bytes([dci.1[4], dci.1[5], dci.1[6], dci.1[7]]);
    assert_eq!(low, 2000, "500 mm must be scaled x4 exactly once");
}
