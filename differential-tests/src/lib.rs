//! Host-side differential test harness for the VL53L8CX / VL53L5CX drivers.
//!
//! The pieces, bottom up:
//!
//! * [`transcript`]: the JSON recordings written by
//!   `tools/bus_trace_to_transcript.py` (one sequence per on-target test),
//!   expanded back to raw bytes using the ULD blobs compiled into `vl53-8x8`.
//! * [`model`]: a register model built from one recorded sequence. It answers
//!   the mock sensor's reads with what the real sensor answered.
//! * [`mock`]: a mock bus implementing blocking `SpiDevice` and `I2c`, a
//!   counting no-op delay and no-op pins. Every transfer is recorded as an
//!   [`Op`], the same shape as the JSON.
//! * [`mod@compare`]: normalisation (collapse runs of identical reads) and a diff
//!   that points at the first differing op with context.
//! * [`driver`]: the `Driver` trait all drivers are driven through, the
//!   `vl53-8x8` adapter (driven with a minimal `block_on`, over the blocking
//!   SPI or I2C bus adapter), and the call sequences re-enacting the on-target
//!   tests (Nucleo suites: [`run_sequence`]; evo-f412re suite:
//!   [`run_evo_sequence`]). The C adapters live in `tests/differential.rs`
//!   (L8 ULD) and `tests/differential_l5.rs` (L5 ULD) because their platform
//!   symbols are `#[no_mangle]`.

use std::fmt;

pub mod compare;
pub mod driver;
pub mod mock;
pub mod model;
pub mod transcript;

pub use compare::{coalesce_contiguous_writes, compare, normalise};
pub use driver::{run_evo_sequence, run_sequence, Driver, RustDriver, EVO_SEQUENCES, SEQUENCES};
pub use mock::{block_on, MockDelay, MockI2c, MockPin, MockSpi, Shared};
pub use model::RegisterModel;
pub use transcript::{Recording, Sequence};

/// Direction of one bus transfer.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum Dir {
    W,
    R,
}

/// One register transfer: the 16-bit register index and the payload written
/// or read. A failed transfer (`-> ERR` in the trace) has `err` set and no data.
#[derive(Clone, PartialEq, Eq)]
pub struct Op {
    pub dir: Dir,
    pub reg: u16,
    pub data: Vec<u8>,
    pub err: bool,
}

impl Op {
    pub fn write(reg: u16, data: &[u8]) -> Self {
        Op { dir: Dir::W, reg, data: data.to_vec(), err: false }
    }
    pub fn read(reg: u16, data: &[u8]) -> Self {
        Op { dir: Dir::R, reg, data: data.to_vec(), err: false }
    }
}

impl fmt::Debug for Op {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} 0x{:04x} len={}", if self.dir == Dir::W { "W" } else { "R" }, self.reg, self.data.len())?;
        if self.err {
            return write!(f, " -> ERR");
        }
        if let Some(name) = transcript::blob_name_of(&self.data) {
            return write!(f, " [{}]", name);
        }
        let shown = self.data.len().min(24);
        write!(f, " [")?;
        for (i, b) in self.data[..shown].iter().enumerate() {
            if i > 0 {
                write!(f, " ")?;
            }
            write!(f, "{:02x}", b)?;
        }
        if shown < self.data.len() {
            write!(f, " ... +{} bytes", self.data.len() - shown)?;
        }
        write!(f, "]")
    }
}

/// A list of transfers, as recorded by the mock or loaded from a recording.
pub type Transcript = Vec<Op>;
