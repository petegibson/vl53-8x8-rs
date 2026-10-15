#!/usr/bin/env python3
"""Turn a defmt bus-trace recording into a compact JSON transcript.

The on-target test suites (`examples/nucleo-f401re-uld/tests/sensor.rs` for the
C ULD wrapper, `examples/nucleo-f401re-embassy/tests/sensor_async.rs` for the
pure-Rust driver) log every bus transfer when built with `defmt-bus-trace`:

    test tests::test_init ... TRACE spi W reg=0x7fff len=1
    TRACE   [0x00]
    TRACE spi R reg=0x0000 len=1
    TRACE   [0xf0]
    TRACE i2c W [0x29] reg=0x7fff len=1        (I2C flavour)
    TRACE spi R reg=0x0006 -> ERR               (failed transfer)

Each test runs from a fresh reset, so a recording is a list of independent
sequences, one per test. This script parses the probe-rs output into

    {"source": ..., "driver": ..., "variant": ..., "bus": ..., "features": [...],
     "sequences": [{"test": "test_init", "stats": {...},
                    "ops": [{"op": "W", "reg": 32767, "data": "00"},
                            {"op": "R", "reg": 0, "data": "f0"},
                            {"op": "W", "reg": 0, "blob": "firmware",
                             "offset": 0, "len": 32768}]}]}

`data` is lowercase hex. A payload of 64 bytes or more that is byte-for-byte a
slice of a known ULD table (the firmware image `vl53-8x8/src/vl53l8cx_fw_base.bin`
or one of the `uld_data.rs` arrays) is stored as a `blob` reference instead;
expansion is exact, and every conversion verifies the round trip before
writing. Shorter payloads always stay inline so the JSON stays unambiguous.

The VL53L5CX tables are the VL53L8CX ones with a few bytes patched
(`L5_FW_PATCH`, `L5_CONFIG_PATCH` in `vl53-8x8/src/sensor.rs`,
`L5_CALIBRATE_XTALK_PATCH` in `uld_data.rs`); they are available as the blobs
`firmware_l5`, `default_configuration_l5` and `calibrate_xtalk_l5`. A slice
that is in both the L8 and the L5 table (most of the firmware) is referenced
through the blob set of the recording's variant, so an L5 recording names
only `*_l5` blobs for them.

Usage:

    bus_trace_to_transcript.py LOG...               convert, writing
                                                    differential-tests/recordings/<stem>.json
    bus_trace_to_transcript.py LOG -o OUT.json      explicit output
    bus_trace_to_transcript.py --check LOG...       re-parse and compare with the
                                                    existing JSON (exit 1 on mismatch)
    bus_trace_to_transcript.py --check JSON...      validate a transcript: blob
                                                    references resolve, hex is well
                                                    formed, stats agree with the ops
    bus_trace_to_transcript.py --summary PATH...    per-sequence op/byte counts

Driver, variant and bus are inferred from the file name (`-uld`/`-8x8`,
`-l8-`/`-l5-`) and the trace prefix (`spi`/`i2c`); override with
--driver/--variant/--features. The default feature list is
`thresholds,xtalk,motion`, plus `spi-bus` for an SPI recording.
Python 3 standard library only.
"""

import argparse
import json
import pathlib
import re
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
FIRMWARE_BIN = REPO_ROOT / "vl53-8x8/src/vl53l8cx_fw_base.bin"
ULD_DATA_RS = REPO_ROOT / "vl53-8x8/src/uld_data.rs"
SENSOR_RS = REPO_ROOT / "vl53-8x8/src/sensor.rs"
RECORDINGS_DIR = REPO_ROOT / "differential-tests/recordings"

# Rust const name -> blob name in the JSON. Order matters only for the report.
RUST_BLOBS = {
    "DEFAULT_CONFIGURATION": "default_configuration",
    "DEFAULT_XTALK": "default_xtalk",
    "GET_NVM_CMD": "get_nvm_cmd",
    "CALIBRATE_XTALK": "calibrate_xtalk",
}
# (patched blob name, base blob name, Rust patch const, file holding the const).
# The patch consts are `&[(usize, u8)]` overlays on the L8 table.
L5_PATCHES = [
    ("firmware_l5", "firmware", "L5_FW_PATCH", "sensor"),
    ("default_configuration_l5", "default_configuration", "L5_CONFIG_PATCH", "sensor"),
    ("calibrate_xtalk_l5", "calibrate_xtalk", "L5_CALIBRATE_XTALK_PATCH", "uld_data"),
]
# Payloads shorter than this are always written inline.
MIN_BLOB_LEN = 64

DEFAULT_FEATURES = ["thresholds", "xtalk", "motion"]


def default_features(bus):
    return (["spi-bus"] if bus == "spi" else []) + DEFAULT_FEATURES

TEST_RE = re.compile(r"^test tests::(\w+)\s+\.\.\.\s*(.*)$")
RECORD_RE = re.compile(
    r"TRACE (spi|i2c) (W|R)(?: \[0x[0-9a-fA-F]+\])? reg=0x([0-9a-fA-F]+)"
    r"(?: len=(\d+))?( -> ERR)?\s*$"
)
PAYLOAD_RE = re.compile(r"^TRACE\s+\[([0-9a-fA-Fx, ]*)\]\s*$")
# The test verdict ends the current test's traffic. INFO lines do not: the
# suites log progress (e.g. `INFO  xtalk calibration: OK`) in the middle of a test.
END_RE = re.compile(r"^(ok|FAILED|test result:)")
RUST_ARRAY_RE = re.compile(
    r"const\s+(\w+)\s*:\s*\[u8;\s*(\d+)\]\s*=\s*\[(.*?)\];", re.S
)
RUST_PATCH_RE = re.compile(
    r"const\s+(\w+)\s*:\s*&\[\(usize,\s*u8\)\]\s*=\s*&\[(.*?)\];", re.S
)
PATCH_PAIR_RE = re.compile(r"\(\s*(0x[0-9A-Fa-f]+|\d+)\s*,\s*(0x[0-9A-Fa-f]+|\d+)\s*\)")


# --------------------------------------------------------------------------
# blobs
# --------------------------------------------------------------------------

def load_patches(texts):
    """Return {const_name: [(offset, value)]} for the `&[(usize, u8)]` consts in `texts`."""
    patches = {}
    for text in texts.values():
        for m in RUST_PATCH_RE.finditer(text):
            name, body = m.group(1), m.group(2)
            body = re.sub(r"//[^\n]*", "", body)
            patches[name] = [(int(o, 0), int(v, 0)) for o, v in PATCH_PAIR_RE.findall(body)]
    return patches


def apply_patch(base, patch):
    out = bytearray(base)
    for off, val in patch:
        if not 0 <= off < len(out):
            raise ValueError(f"patch offset {off:#x} outside a {len(out)}-byte table")
        out[off] = val
    return bytes(out)


def load_blobs(firmware_bin=FIRMWARE_BIN, uld_data_rs=ULD_DATA_RS, sensor_rs=SENSOR_RS):
    """Return {blob_name: bytes} for the firmware image, the ULD tables and
    their VL53L5CX-patched versions."""
    blobs = {"firmware": firmware_bin.read_bytes()}
    text = uld_data_rs.read_text()
    for m in RUST_ARRAY_RE.finditer(text):
        name, length, body = m.group(1), int(m.group(2)), m.group(3)
        if name not in RUST_BLOBS:
            continue
        values = bytes(int(tok, 0) for tok in body.replace("\n", " ").split(",") if tok.strip())
        if len(values) != length:
            sys.exit(f"{uld_data_rs}: {name} declares {length} bytes but holds {len(values)}")
        blobs[RUST_BLOBS[name]] = values
    missing = set(RUST_BLOBS.values()) - set(blobs)
    if missing:
        sys.exit(f"{uld_data_rs}: arrays not found: {sorted(missing)}")
    patches = load_patches({"sensor": sensor_rs.read_text(), "uld_data": text})
    for name, base, const, _ in L5_PATCHES:
        if const not in patches:
            sys.exit(f"{sensor_rs} / {uld_data_rs}: patch table {const} not found")
        if not patches[const]:
            sys.exit(f"patch table {const} is empty; an L5 blob would equal the L8 one")
        blobs[name] = apply_patch(blobs[base], patches[const])
    return blobs


def blob_order(blobs, variant):
    """Blob names in matching order: the recording variant's own tables first,
    so a slice common to both variants is named after the variant."""
    l5 = [name for name, *_ in L5_PATCHES]
    base = [n for n in blobs if n not in l5]
    return l5 + base if variant == "L5" else base + l5


def find_blob(blobs, payload, variant="L8"):
    """Return (name, offset) if `payload` is an exact slice of a blob, else None."""
    if len(payload) < MIN_BLOB_LEN:
        return None
    for name in blob_order(blobs, variant):
        off = blobs[name].find(payload)
        if off >= 0:
            return name, off
    return None


# --------------------------------------------------------------------------
# parsing
# --------------------------------------------------------------------------

class Op:
    __slots__ = ("op", "reg", "len", "data", "err")

    def __init__(self, op, reg, length, err):
        self.op = op
        self.reg = reg
        self.len = length
        self.data = bytearray()
        self.err = err

    def complete(self):
        return self.err or len(self.data) >= (self.len or 0)


def parse_log(path):
    """Return (bus, [(test_name, [Op])]) from a probe-rs defmt log."""
    bus = None
    sequences = []
    current_name = None
    current_ops = None
    pending = None  # Op still collecting payload lines

    def close_test():
        nonlocal current_name, current_ops, pending
        if current_name is not None:
            if pending is not None and not pending.complete():
                raise ValueError(
                    f"{path}:{current_name}: truncated payload for "
                    f"{pending.op} reg=0x{pending.reg:04x} len={pending.len}"
                )
            sequences.append((current_name, current_ops))
        current_name, current_ops, pending = None, None, None

    def handle_trace(text):
        nonlocal bus, pending
        m = RECORD_RE.search(text)
        if m:
            kind, direction, reg, length, err = m.groups()
            if bus is None:
                bus = kind
            elif bus != kind:
                raise ValueError(f"{path}: mixed {bus} and {kind} traffic")
            if pending is not None and not pending.complete():
                raise ValueError(
                    f"{path}:{current_name}: payload short for "
                    f"{pending.op} reg=0x{pending.reg:04x} len={pending.len}"
                )
            op = Op(direction, int(reg, 16), int(length) if length else 0, bool(err))
            if current_ops is None:
                raise ValueError(f"{path}: bus traffic before the first test header")
            current_ops.append(op)
            pending = op
            return
        m = PAYLOAD_RE.match(text)
        if m and pending is not None and not pending.complete():
            toks = [t for t in m.group(1).split(",") if t.strip()]
            pending.data.extend(int(t, 0) for t in toks)
            return
        # Other TRACE output (e.g. embassy `rcc:` lines) is ignored.

    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.rstrip("\n")
            m = TEST_RE.match(line)
            if m:
                close_test()
                name, rest = m.group(1), m.group(2)
                if rest.strip() == "ignored":
                    continue
                current_name, current_ops = name, []
                if rest.strip():
                    handle_trace(rest)
                continue
            if current_name is None:
                continue
            if line.startswith("TRACE"):
                handle_trace(line)
            elif END_RE.match(line):
                close_test()
            # anything else inside a test (INFO, DEBUG, blank) is ignored
    close_test()
    return bus or "spi", sequences


# --------------------------------------------------------------------------
# transcript <-> ops
# --------------------------------------------------------------------------

def op_to_json(op, blobs, variant):
    d = {"op": op.op, "reg": op.reg}
    if op.err:
        d["err"] = True
        d["len"] = op.len
        return d
    hit = find_blob(blobs, bytes(op.data), variant)
    if hit:
        name, off = hit
        d.update(blob=name, offset=off, len=len(op.data))
    else:
        d["data"] = op.data.hex()
    return d


def json_to_bytes(d, blobs):
    """Expand one JSON op to (op, reg, bytes, err)."""
    if d.get("err"):
        return d["op"], d["reg"], b"", True
    if "blob" in d:
        blob = blobs[d["blob"]]
        off, length = d["offset"], d["len"]
        if off < 0 or off + length > len(blob):
            raise ValueError(f"blob {d['blob']}[{off}:{off + length}] out of range")
        return d["op"], d["reg"], blob[off:off + length], False
    return d["op"], d["reg"], bytes.fromhex(d["data"]), False


def stats_for(ops):
    s = {"ops": len(ops), "writes": 0, "reads": 0, "bytes_written": 0, "bytes_read": 0}
    for op in ops:
        if op.op == "W":
            s["writes"] += 1
            s["bytes_written"] += len(op.data)
        else:
            s["reads"] += 1
            s["bytes_read"] += len(op.data)
    return s


def infer_meta(stem):
    driver = "vl53l8cx-uld" if "-uld" in stem else "vl53-8x8" if "-8x8" in stem else "unknown"
    variant = "L5" if "-l5-" in stem else "L8"
    return driver, variant


def build_transcript(log_path, bus, sequences, blobs, driver, variant, features):
    return {
        "source": log_path.name,
        "driver": driver,
        "variant": variant,
        "bus": bus,
        "features": features if features is not None else default_features(bus),
        "sequences": [
            {"test": name, "stats": stats_for(ops), "ops": [op_to_json(op, blobs, variant) for op in ops]}
            for name, ops in sequences
        ],
    }


def dump_transcript(t):
    """One op per line: small, and `git diff` shows exactly which transfer changed."""
    out = []
    head = {k: v for k, v in t.items() if k != "sequences"}
    head_json = json.dumps(head, separators=(", ", ": "))
    out.append(head_json[:-1] + ",")
    out.append('  "sequences": [')
    for si, seq in enumerate(t["sequences"]):
        out.append("    {" + json.dumps({"test": seq["test"], "stats": seq["stats"]}, separators=(", ", ": "))[1:-1] + ",")
        out.append('     "ops": [')
        n = len(seq["ops"])
        for oi, op in enumerate(seq["ops"]):
            out.append("      " + json.dumps(op, separators=(",", ":")) + ("," if oi + 1 < n else ""))
        out.append("     ]}" + ("," if si + 1 < len(t["sequences"]) else ""))
    out.append("  ]")
    out.append("}")
    return "\n".join(out) + "\n"


def raw_tuples(sequences):
    return [(name, [(o.op, o.reg, bytes(o.data), o.err) for o in ops]) for name, ops in sequences]


def expanded_tuples(transcript, blobs):
    return [
        (seq["test"], [json_to_bytes(d, blobs) for d in seq["ops"]])
        for seq in transcript["sequences"]
    ]


def first_difference(a, b):
    """Describe the first mismatch between two [(name, [ops])] lists, or None."""
    if len(a) != len(b):
        return f"{len(a)} vs {len(b)} sequences"
    for (na, oa), (nb, ob) in zip(a, b):
        if na != nb:
            return f"sequence name {na!r} vs {nb!r}"
        if len(oa) != len(ob):
            return f"{na}: {len(oa)} vs {len(ob)} ops"
        for i, (x, y) in enumerate(zip(oa, ob)):
            if x != y:
                return f"{na} op {i}: {x[:2]} len={len(x[2])} vs {y[:2]} len={len(y[2])}"
    return None


# --------------------------------------------------------------------------
# commands
# --------------------------------------------------------------------------

def print_summary(transcript, blobs, label):
    print(f"{label}: driver={transcript['driver']} variant={transcript['variant']} "
          f"bus={transcript['bus']} features={','.join(transcript['features'])}")
    total = {"ops": 0, "bytes_written": 0, "bytes_read": 0}
    blob_refs = 0
    for seq in transcript["sequences"]:
        s = seq["stats"]
        refs = sum(1 for d in seq["ops"] if "blob" in d)
        blob_refs += refs
        print(f"  {seq['test']:<45} ops={s['ops']:>5}  W={s['writes']:>4}/{s['bytes_written']:>7} B"
              f"  R={s['reads']:>4}/{s['bytes_read']:>6} B  blob-refs={refs}")
        for k in total:
            total[k] += s[k]
    print(f"  {'total':<45} ops={total['ops']:>5}  written={total['bytes_written']} B "
          f"read={total['bytes_read']} B  blob-refs={blob_refs} "
          f"({len(transcript['sequences'])} sequences)")


def validate_json(path, blobs):
    """Check a transcript file stands on its own. Returns the transcript."""
    t = json.loads(path.read_text())
    for key in ("source", "driver", "variant", "bus", "features", "sequences"):
        if key not in t:
            raise ValueError(f"{path}: missing {key!r}")
    for seq in t["sequences"]:
        ops = []
        for d in seq["ops"]:
            op, reg, data, err = json_to_bytes(d, blobs)
            if op not in ("W", "R") or not 0 <= reg <= 0xFFFF:
                raise ValueError(f"{path}:{seq['test']}: bad op {d}")
            if "blob" in d and len(data) < MIN_BLOB_LEN:
                raise ValueError(f"{path}:{seq['test']}: blob ref shorter than {MIN_BLOB_LEN}: {d}")
            o = Op(op, reg, len(data), err)
            o.data = bytearray(data)
            ops.append(o)
        if stats_for(ops) != seq["stats"]:
            raise ValueError(f"{path}:{seq['test']}: stats do not match ops")
    return t


def convert(log_path, out_path, blobs, args, check_only):
    bus, sequences = parse_log(log_path)
    if not sequences:
        sys.exit(f"{log_path}: no test sequences found")
    driver, variant = infer_meta(log_path.stem)
    transcript = build_transcript(
        log_path, bus, sequences, blobs,
        args.driver or driver, args.variant or variant, args.features,
    )
    # Internal round trip: the compact form must expand to exactly the parsed ops.
    diff = first_difference(raw_tuples(sequences), expanded_tuples(transcript, blobs))
    if diff:
        sys.exit(f"{log_path}: blob substitution is not exact: {diff}")
    text = dump_transcript(transcript)
    # dump_transcript hand-formats; make sure it is still valid JSON that reloads equal.
    if json.loads(text) != transcript:
        sys.exit(f"{log_path}: formatted JSON does not reload identically")

    if check_only:
        if not out_path.exists():
            print(f"MISSING    {out_path}")
            return False
        existing = validate_json(out_path, blobs)
        diff = first_difference(expanded_tuples(existing, blobs), raw_tuples(sequences))
        if diff:
            print(f"MISMATCH   {out_path}: {diff}")
            return False
        print(f"ok         {out_path} matches {log_path.name} "
              f"({len(sequences)} sequences, {sum(len(o) for _, o in sequences)} ops)")
        return True

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(text)
    print(f"wrote      {out_path} ({len(text)} bytes, {len(sequences)} sequences, "
          f"{sum(len(o) for _, o in sequences)} ops) from {log_path} "
          f"({log_path.stat().st_size} bytes)")
    if args.summary:
        print_summary(transcript, blobs, out_path.name)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", type=pathlib.Path, help=".log recordings or .json transcripts")
    ap.add_argument("-o", "--output", type=pathlib.Path, help="output JSON (single .log input only)")
    ap.add_argument("--check", action="store_true", help="verify instead of writing")
    ap.add_argument("--summary", action="store_true", help="print per-sequence statistics")
    ap.add_argument("--driver", help="override the driver name inferred from the file name")
    ap.add_argument("--variant", help="override the sensor variant (L8/L5)")
    ap.add_argument("--features", nargs="*", default=None,
                    help="cargo features the recording was made with "
                         "(default: thresholds xtalk motion, plus spi-bus for SPI)")
    ap.add_argument("--firmware", type=pathlib.Path, default=FIRMWARE_BIN)
    ap.add_argument("--uld-data", type=pathlib.Path, default=ULD_DATA_RS)
    ap.add_argument("--sensor-rs", type=pathlib.Path, default=SENSOR_RS,
                    help="vl53-8x8 sensor.rs holding the L5 patch tables")
    args = ap.parse_args()

    if args.output and len(args.paths) != 1:
        ap.error("-o needs exactly one input")
    blobs = load_blobs(args.firmware, args.uld_data, args.sensor_rs)

    ok = True
    for path in args.paths:
        if path.suffix == ".json":
            try:
                t = validate_json(path, blobs)
            except (ValueError, KeyError, json.JSONDecodeError) as e:
                print(f"INVALID    {path}: {e}")
                ok = False
                continue
            if args.check:
                print(f"ok         {path} is a valid transcript "
                      f"({len(t['sequences'])} sequences, {sum(len(s['ops']) for s in t['sequences'])} ops)")
            if args.summary or not args.check:
                print_summary(t, blobs, path.name)
        else:
            out = args.output or RECORDINGS_DIR / (path.stem + ".json")
            if args.summary and args.check:
                bus, sequences = parse_log(path)
                driver, variant = infer_meta(path.stem)
                print_summary(build_transcript(path, bus, sequences, blobs, args.driver or driver,
                                               args.variant or variant, args.features), blobs, path.name)
            ok &= convert(path, out, blobs, args, check_only=args.check)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
