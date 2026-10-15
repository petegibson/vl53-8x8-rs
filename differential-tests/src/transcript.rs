//! Loading the JSON recordings and expanding blob references.
//!
//! Schema (see `tools/bus_trace_to_transcript.py`):
//!
//! ```json
//! { "source": "nucleo-l8-spi-uld.log", "driver": "vl53l8cx-uld", "variant": "L8",
//!   "bus": "spi", "features": ["spi-bus", ...],
//!   "sequences": [ { "test": "test_init", "stats": {...},
//!                    "ops": [ {"op":"W","reg":32767,"data":"00"},
//!                             {"op":"W","reg":0,"blob":"firmware","offset":0,"len":32768} ] } ] }
//! ```
//!
//! Blobs are the ULD tables shipped in `vl53-8x8`: the firmware image is
//! included as bytes, the `uld_data.rs` arrays are parsed from the Rust source
//! (they are `pub(crate)`), exactly as the Python script does.

use std::collections::HashMap;
use std::sync::OnceLock;

use serde::Deserialize;

use crate::{Dir, Op, Transcript};

pub const FIRMWARE: &[u8] = include_bytes!("../../vl53-8x8/src/vl53l8cx_fw_base.bin");
const ULD_DATA_RS: &str = include_str!("../../vl53-8x8/src/uld_data.rs");

pub const RECORDING_C: &str = include_str!("../recordings/nucleo-l8-spi-uld.json");
pub const RECORDING_RUST: &str = include_str!("../recordings/nucleo-l8-spi-8x8.json");

/// (Rust const name, blob name in the JSON)
const RUST_BLOBS: [(&str, &str); 4] = [
    ("DEFAULT_CONFIGURATION", "default_configuration"),
    ("DEFAULT_XTALK", "default_xtalk"),
    ("GET_NVM_CMD", "get_nvm_cmd"),
    ("CALIBRATE_XTALK", "calibrate_xtalk"),
];

/// Parse `const NAME: [u8; N] = [ 0x.., ... ];` arrays out of `uld_data.rs`.
fn parse_rust_arrays(src: &str) -> HashMap<String, Vec<u8>> {
    let mut out = HashMap::new();
    let mut rest = src;
    while let Some(pos) = rest.find("const ") {
        rest = &rest[pos + 6..];
        let Some(colon) = rest.find(':') else { break };
        let name = rest[..colon].trim().to_string();
        let Some(open) = rest.find("= [") else { break };
        let body_start = open + 3;
        let Some(close) = rest[body_start..].find("];") else { break };
        let body = &rest[body_start..body_start + close];
        // Only byte arrays: every token is 0x.. . Other consts (tuples) are skipped.
        let bytes: Option<Vec<u8>> = body
            .split(',')
            .map(str::trim)
            .filter(|t| !t.is_empty())
            .map(|t| t.strip_prefix("0x").and_then(|h| u8::from_str_radix(h, 16).ok()))
            .collect();
        if let Some(bytes) = bytes {
            if !bytes.is_empty() {
                out.insert(name, bytes);
            }
        }
        rest = &rest[body_start + close..];
    }
    out
}

/// All known blobs by JSON name.
pub fn blobs() -> &'static HashMap<&'static str, Vec<u8>> {
    static BLOBS: OnceLock<HashMap<&'static str, Vec<u8>>> = OnceLock::new();
    BLOBS.get_or_init(|| {
        let arrays = parse_rust_arrays(ULD_DATA_RS);
        let mut m = HashMap::new();
        m.insert("firmware", FIRMWARE.to_vec());
        for (rust_name, json_name) in RUST_BLOBS {
            let data = arrays
                .get(rust_name)
                .unwrap_or_else(|| panic!("uld_data.rs: array {rust_name} not found"))
                .clone();
            m.insert(json_name, data);
        }
        m
    })
}

/// Name the blob a whole payload is equal to, for compact `Debug` output.
pub fn blob_name_of(data: &[u8]) -> Option<&'static str> {
    if data.len() < 64 {
        return None;
    }
    blobs().iter().find_map(|(name, blob)| {
        if blob.as_slice() == data {
            Some(*name)
        } else if *name == "firmware" && data.len() >= 1024 {
            blob.windows(data.len()).position(|w| w == data).map(|off| {
                // Leak one small string per distinct firmware slice; only used in failure output.
                let s: &'static str = Box::leak(format!("firmware[{off}..{}]", off + data.len()).into_boxed_str());
                s
            })
        } else {
            None
        }
    })
}

#[derive(Deserialize, Debug, Clone)]
pub struct Stats {
    pub ops: usize,
    pub writes: usize,
    pub reads: usize,
    pub bytes_written: usize,
    pub bytes_read: usize,
}

#[derive(Deserialize, Debug, Clone)]
struct JsonOp {
    op: String,
    reg: u16,
    #[serde(default)]
    data: Option<String>,
    #[serde(default)]
    blob: Option<String>,
    #[serde(default)]
    offset: Option<usize>,
    #[serde(default)]
    len: Option<usize>,
    #[serde(default)]
    err: bool,
}

#[derive(Deserialize, Debug, Clone)]
struct JsonSequence {
    test: String,
    stats: Stats,
    ops: Vec<JsonOp>,
}

#[derive(Deserialize, Debug, Clone)]
struct JsonRecording {
    source: String,
    driver: String,
    variant: String,
    bus: String,
    features: Vec<String>,
    sequences: Vec<JsonSequence>,
}

/// One on-target test's traffic, expanded to raw bytes.
#[derive(Debug, Clone)]
pub struct Sequence {
    pub test: String,
    pub stats: Stats,
    pub ops: Transcript,
}

/// A whole recording file.
#[derive(Debug, Clone)]
pub struct Recording {
    pub source: String,
    pub driver: String,
    pub variant: String,
    pub bus: String,
    pub features: Vec<String>,
    pub sequences: Vec<Sequence>,
}

fn hex(s: &str) -> Vec<u8> {
    assert!(s.len() % 2 == 0, "odd hex string");
    (0..s.len() / 2)
        .map(|i| u8::from_str_radix(&s[2 * i..2 * i + 2], 16).expect("bad hex"))
        .collect()
}

impl Recording {
    pub fn parse(json: &str) -> Recording {
        let raw: JsonRecording = serde_json::from_str(json).expect("recording JSON");
        let blobs = blobs();
        let sequences = raw
            .sequences
            .into_iter()
            .map(|s| {
                let ops = s
                    .ops
                    .iter()
                    .map(|o| {
                        let dir = match o.op.as_str() {
                            "W" => Dir::W,
                            "R" => Dir::R,
                            other => panic!("{}: bad op {other:?}", s.test),
                        };
                        let data = if o.err {
                            Vec::new()
                        } else if let Some(name) = &o.blob {
                            let blob = blobs
                                .get(name.as_str())
                                .unwrap_or_else(|| panic!("{}: unknown blob {name}", s.test));
                            let (off, len) = (o.offset.expect("offset"), o.len.expect("len"));
                            blob[off..off + len].to_vec()
                        } else {
                            hex(o.data.as_deref().expect("data"))
                        };
                        Op { dir, reg: o.reg, data, err: o.err }
                    })
                    .collect::<Vec<_>>();
                assert_eq!(ops.len(), s.stats.ops, "{}: stats.ops disagrees with ops", s.test);
                Sequence { test: s.test, stats: s.stats, ops }
            })
            .collect();
        Recording {
            source: raw.source,
            driver: raw.driver,
            variant: raw.variant,
            bus: raw.bus,
            features: raw.features,
            sequences,
        }
    }

    /// The committed recording of the C ULD wrapper suite (`vl53l8cx-uld`).
    pub fn c_uld() -> &'static Recording {
        static R: OnceLock<Recording> = OnceLock::new();
        R.get_or_init(|| Recording::parse(RECORDING_C))
    }

    /// The committed recording of the pure-Rust suite (`vl53-8x8`).
    pub fn rust() -> &'static Recording {
        static R: OnceLock<Recording> = OnceLock::new();
        R.get_or_init(|| Recording::parse(RECORDING_RUST))
    }

    pub fn sequence(&self, test: &str) -> Option<&Sequence> {
        self.sequences.iter().find(|s| s.test == test)
    }
}
