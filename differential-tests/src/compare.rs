//! Transcript normalisation and comparison.

use crate::{Dir, Op};

/// Collapse runs of identical consecutive reads (same register, length and
/// data) into one. Poll loops ran for a timing-dependent number of iterations
/// on hardware; after this the count no longer matters, only the values seen.
/// Writes are never merged: the two drivers chunk the firmware upload the same
/// way (three bank writes of 32768, 32768 and 20480 bytes), so a differing
/// write is a real difference.
pub fn normalise(ops: &[Op]) -> Vec<Op> {
    let mut out: Vec<Op> = Vec::with_capacity(ops.len());
    for op in ops {
        if op.dir == Dir::R {
            if let Some(last) = out.last() {
                if last.dir == Dir::R && last == op {
                    continue;
                }
            }
        }
        out.push(op.clone());
    }
    out
}

/// Compare two normalised transcripts. On a mismatch the message names the
/// first differing op index, shows the op from each side and 3 ops of context
/// before and after on both sides.
pub fn compare(expected_label: &str, expected: &[Op], actual_label: &str, actual: &[Op]) -> Result<(), String> {
    let n = expected.len().min(actual.len());
    let first_diff = (0..n).find(|&i| expected[i] != actual[i]);
    let idx = match first_diff {
        Some(i) => i,
        None if expected.len() == actual.len() => return Ok(()),
        None => n,
    };
    let mut msg = format!(
        "transcripts differ at op {idx} ({expected_label}: {} ops, {actual_label}: {} ops)\n",
        expected.len(),
        actual.len()
    );
    let show = |label: &str, ops: &[Op], msg: &mut String| {
        msg.push_str(&format!("  {label}:\n"));
        let lo = idx.saturating_sub(3);
        let hi = (idx + 4).min(ops.len());
        for i in lo..hi {
            let marker = if i == idx { ">>" } else { "  " };
            msg.push_str(&format!("    {marker} [{i:>4}] {:?}\n", ops[i]));
        }
        if idx >= ops.len() {
            msg.push_str(&format!("    >> [{idx:>4}] <end of transcript>\n"));
        }
    };
    show(expected_label, expected, &mut msg);
    show(actual_label, actual, &mut msg);
    Err(msg)
}
