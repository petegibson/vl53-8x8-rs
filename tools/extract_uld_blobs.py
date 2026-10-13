#!/usr/bin/env python3
"""Regenerate the ST ULD data tables compiled into `vl53l8cx-async`.

Parses `vl53l8cx-sys/src/VL53L8CX_ULD_API/inc/vl53l8cx_buffers.h` (ST's
Ultra Lite Driver) and writes:

  vl53l8cx-async/src/uld_data.rs          DEFAULT_CONFIGURATION, DEFAULT_XTALK,
                                          GET_NVM_CMD as Rust `const` arrays
  vl53l8cx-async/src/vl53l8cx_fw_base.bin VL53L8CX_FIRMWARE as raw bytes

Regeneration procedure after dropping a new ULD release into the sys crate:

    python3 tools/extract_uld_blobs.py        # rewrite both outputs
    cargo test -p vl53l8cx-async              # oracle tests against the C arrays
    git diff vl53l8cx-async/src/uld_data.rs   # review the table changes

`--check` verifies the files on disk without writing (exit 1 on mismatch).
Python 3 standard library only.

The configuration table contains one preprocessor macro,
`VL53L8CX_FW_NBTAR_RANGING`, which the ULD defines as 2 for a one-target-per-
zone build and as `VL53L8CX_NB_TARGET_PER_ZONE` otherwise. The sys crate has
always been built for one target, so it is resolved to 2 here.
"""

import argparse
import pathlib
import re
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_HEADER = REPO_ROOT / "vl53l8cx-sys/src/VL53L8CX_ULD_API/inc/vl53l8cx_buffers.h"
DEFAULT_API_HEADER = DEFAULT_HEADER.with_name("vl53l8cx_api.h")
DEFAULT_OUT_DIR = REPO_ROOT / "vl53l8cx-async/src"

# Preprocessor symbols that may appear inside the arrays, with the value they
# take in the build the sys crate performs (VL53L8CX_NB_TARGET_PER_ZONE == 1).
MACROS = {
    "VL53L8CX_FW_NBTAR_RANGING": 2,
}

# (C array name, Rust const name)
RUST_TABLES = [
    ("VL53L8CX_DEFAULT_CONFIGURATION", "DEFAULT_CONFIGURATION"),
    ("VL53L8CX_DEFAULT_XTALK", "DEFAULT_XTALK"),
    ("VL53L8CX_GET_NVM_CMD", "GET_NVM_CMD"),
]
FIRMWARE_ARRAY = "VL53L8CX_FIRMWARE"

ARRAY_RE = re.compile(r"const\s+uint8_t\s+(\w+)\s*\[\]\s*=\s*\{(.*?)\}\s*;", re.S)
COMMENT_RE = re.compile(r"/\*.*?\*/|//[^\n]*", re.S)
INT_RE = re.compile(r"^(0[xX][0-9a-fA-F]+|\d+)[uU]?$")


def parse_arrays(text):
    """Return {name: (bytes, [(index, macro_name), ...])} for every uint8_t array."""
    text = COMMENT_RE.sub("", text)
    arrays = {}
    for m in ARRAY_RE.finditer(text):
        name, body = m.group(1), m.group(2)
        values = bytearray()
        macro_uses = []
        for tok in body.split(","):
            tok = tok.strip()
            if not tok:
                continue  # trailing comma / blank line
            if INT_RE.match(tok):
                v = int(tok.rstrip("uU"), 0)
            elif tok in MACROS:
                macro_uses.append((len(values), tok))
                v = MACROS[tok]
            else:
                sys.exit(f"{name}: cannot resolve token {tok!r}")
            if not 0 <= v <= 255:
                sys.exit(f"{name}: value {v} out of uint8_t range")
            values.append(v)
        arrays[name] = (bytes(values), macro_uses)
    return arrays


def api_revision(api_header):
    try:
        m = re.search(r'VL53L8CX_API_REVISION\s+"([^"]+)"', api_header.read_text())
        return m.group(1) if m else "unknown"
    except OSError:
        return "unknown"


def render_rust(arrays, header_rel, revision):
    out = []
    out.append("// GENERATED FILE - DO NOT EDIT.")
    out.append(f"// Source: {header_rel} (ULD {revision}).")
    out.append("// Regenerate with: python3 tools/extract_uld_blobs.py")
    out.append("")
    out.append("//! ST ULD data tables (default configuration, default crosstalk shape and")
    out.append("//! the NVM-read command), transcribed byte-for-byte from `vl53l8cx_buffers.h`.")
    out.append("//! The firmware image from the same header is shipped separately as")
    out.append("//! `vl53l8cx_fw_base.bin`. Both are checked against the C arrays compiled")
    out.append("//! into `vl53l8cx-sys` by this crate's oracle tests.")
    out.append("")
    for c_name, rust_name in RUST_TABLES:
        if c_name not in arrays:
            sys.exit(f"{c_name} not found in header")
        data, macro_uses = arrays[c_name]
        out.append(f"/// C `{c_name}` ({len(data)} bytes).")
        for idx, macro in macro_uses:
            out.append(f"///")
            out.append(f"/// Byte {idx} is the ULD macro `{macro}`, resolved to {MACROS[macro]}")
            out.append(f"/// (one target per zone).")
        out.append(f"pub(crate) const {rust_name}: [u8; {len(data)}] = [")
        for i in range(0, len(data), 16):
            chunk = ", ".join(f"0x{b:02X}" for b in data[i : i + 16])
            out.append(f"    {chunk},")
        out.append("];")
        out.append("")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--header", type=pathlib.Path, default=DEFAULT_HEADER)
    ap.add_argument("--api-header", type=pathlib.Path, default=DEFAULT_API_HEADER)
    ap.add_argument("--out-dir", type=pathlib.Path, default=DEFAULT_OUT_DIR)
    ap.add_argument("--check", action="store_true", help="verify outputs; write nothing")
    args = ap.parse_args()

    arrays = parse_arrays(args.header.read_text())
    if FIRMWARE_ARRAY not in arrays:
        sys.exit(f"{FIRMWARE_ARRAY} not found in header")
    firmware, fw_macros = arrays[FIRMWARE_ARRAY]
    if fw_macros:
        sys.exit(f"{FIRMWARE_ARRAY} unexpectedly contains macros: {fw_macros}")

    try:
        header_rel = args.header.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        header_rel = args.header.name
    rust_src = render_rust(arrays, header_rel, api_revision(args.api_header))

    outputs = {
        args.out_dir / "uld_data.rs": rust_src.encode(),
        args.out_dir / "vl53l8cx_fw_base.bin": firmware,
    }

    ok = True
    for path, content in outputs.items():
        existing = path.read_bytes() if path.exists() else None
        if existing == content:
            print(f"unchanged  {path} ({len(content)} bytes)")
            continue
        if args.check:
            print(f"MISMATCH   {path}")
            ok = False
        else:
            path.write_bytes(content)
            print(f"wrote      {path} ({len(content)} bytes)")
    for name, (data, _) in arrays.items():
        print(f"  {name}: {len(data)} bytes")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
