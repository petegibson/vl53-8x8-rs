//! The register model: what the mock sensor answers to reads.
//!
//! Modelling rule (also in the crate README):
//!
//! * The model is built from **one recorded sequence** (one on-target test,
//!   which ran from a fresh sensor reset) and is used for exactly one host
//!   replay of that test. Nothing carries over between tests.
//! * Reads are keyed by `(register, length)`. For each key the model keeps the
//!   recorded responses **in order, runs included** (a poll loop that read
//!   `0x00` three times and then `0x01` yields `[00, 00, 00, 01]`), with a
//!   cursor per key. Each read returns the response at the cursor and advances
//!   it; past the end the last response sticks.
//! * Writes are accepted and not interpreted.
//!
//! Why not a single positional cursor over all reads: a driver that reads one
//! register more or less often than the recording would shift every later
//! answer. Per-key cursors keep the other registers in phase.
//!
//! Why not collapse runs of identical responses: on the mock there is no clock,
//! so a poll loop's length is decided by the answers it gets, not by timing. A
//! driver with the same logic as the recorded one therefore polls exactly as
//! often as recorded when the runs are kept. Collapsing would break the case of
//! two consecutive poll loops on the same register that each completed in one
//! read with the same value (`[01]` then `[01]` collapse to `[01]`, and the
//! second loop would be served the first answer of the third). The two drivers
//! did poll a different number of times on hardware, which is why the
//! comparison additionally collapses identical consecutive reads (see
//! `compare::normalise`) before transcripts are compared.
//!
//! Reads of a key that never appears in the sequence return zeros and are
//! remembered in `unknown`; they show up in the transcript diff anyway.

use std::collections::HashMap;

use crate::{Dir, Op};

#[derive(Debug, Clone, Default)]
pub struct RegisterModel {
    responses: HashMap<(u16, usize), Vec<Vec<u8>>>,
    cursor: HashMap<(u16, usize), usize>,
    /// (reg, len) pairs read by the driver that the recording never read.
    pub unknown: Vec<(u16, usize)>,
    /// Number of reads served past the end of a key's recorded responses.
    pub sticky_reads: usize,
}

impl RegisterModel {
    pub fn from_ops(ops: &[Op]) -> Self {
        let mut m = RegisterModel::default();
        for op in ops.iter().filter(|o| o.dir == Dir::R && !o.err) {
            m.responses.entry((op.reg, op.data.len())).or_default().push(op.data.clone());
        }
        m
    }

    /// Rewind every cursor; the model then answers as if from reset again.
    pub fn reset(&mut self) {
        self.cursor.clear();
        self.unknown.clear();
        self.sticky_reads = 0;
    }

    /// Fill `buf` with the next recorded response for `(reg, buf.len())`.
    pub fn read(&mut self, reg: u16, buf: &mut [u8]) {
        let key = (reg, buf.len());
        match self.responses.get(&key) {
            None => {
                buf.fill(0);
                self.unknown.push(key);
            }
            Some(list) => {
                let idx = self.cursor.entry(key).or_insert(0);
                if *idx >= list.len() {
                    self.sticky_reads += 1;
                    buf.copy_from_slice(&list[list.len() - 1]);
                } else {
                    buf.copy_from_slice(&list[*idx]);
                    *idx += 1;
                }
            }
        }
    }

    /// Number of recorded responses for a key (for diagnostics).
    pub fn recorded(&self, reg: u16, len: usize) -> usize {
        self.responses.get(&(reg, len)).map_or(0, Vec::len)
    }
}
