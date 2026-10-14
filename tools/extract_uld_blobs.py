#!/usr/bin/env python3
"""Regenerate the ST ULD data tables compiled into `vl53-8x8`.

Parses `vl53l8cx-uld-sys/src/VL53L8CX_ULD_API/inc/vl53l8cx_buffers.h` (ST's
Ultra Lite Driver) plus the xtalk plugin headers of both the L8 and the L5 ULD
(`vl53l8cx_plugin_xtalk.h`, `vl53l5cx_plugin_xtalk.h`) and writes:

  vl53-8x8/src/uld_data.rs          DEFAULT_CONFIGURATION, DEFAULT_XTALK,
                                          GET_NVM_CMD, CALIBRATE_XTALK as Rust
                                          `const` arrays, plus the L5 overlay
                                          L5_CALIBRATE_XTALK_PATCH and the
                                          FW_NBTAR_XTALK_INDEX macro position
  vl53-8x8/src/vl53l8cx_fw_base.bin VL53L8CX_FIRMWARE as raw bytes

Regeneration procedure after dropping a new ULD release into the sys crate:

    python3 tools/extract_uld_blobs.py        # rewrite both outputs
    cargo test -p vl53-8x8              # oracle tests against the C arrays
    git diff vl53-8x8/src/uld_data.rs   # review the table changes

`--check` verifies the files on disk without writing (exit 1 on mismatch).
Python 3 standard library only.

The configuration table contains one preprocessor macro,
`VL53L8CX_FW_NBTAR_RANGING`, which the ULD defines as 2 for a one-target-per-
zone build and as `VL53L8CX_NB_TARGET_PER_ZONE` otherwise. The sys crate has
always been built for one target, so it is resolved to 2 here; the driver's
`sensor::NB_TARGET_CONFIG_PATCH` overwrites that byte at init for the
`targets-2/3/4` features. The xtalk-calibration command table carries the twin
macro `VL53L8CX_FW_NBTAR_XTALK` (same definition); it is resolved to 2 as well
and its position is emitted as `FW_NBTAR_XTALK_INDEX` so `ops::calibrate_xtalk`
can overwrite it for multi-target builds.

`VL53L8CX_CALIBRATE_XTALK` is a `static const` in a header (internal linkage),
so unlike the `vl53l8cx_buffers.h` tables the oracle tests cannot link to the C
array; they re-parse the header with `include_str!` instead. The L5 table
differs from the L8 one in two bytes, so it is emitted as an overlay in the
style of `sensor::L5_CONFIG_PATCH` rather than as a second full table.
"""

import argparse
import pathlib
import re
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_HEADER = REPO_ROOT / "vl53l8cx-uld-sys/src/VL53L8CX_ULD_API/inc/vl53l8cx_buffers.h"
DEFAULT_API_HEADER = DEFAULT_HEADER.with_name("vl53l8cx_api.h")
DEFAULT_XTALK_HEADER = DEFAULT_HEADER.with_name("vl53l8cx_plugin_xtalk.h")
DEFAULT_L5_XTALK_HEADER = (
    REPO_ROOT / "vl53l5cx-uld-sys/src/VL53L5CX_ULD_API/inc/vl53l5cx_plugin_xtalk.h"
)
DEFAULT_OUT_DIR = REPO_ROOT / "vl53-8x8/src"

# Preprocessor symbols that may appear inside the arrays, with the value they
# take in the build the sys crates perform (*_NB_TARGET_PER_ZONE == 1).
MACROS = {
    "VL53L8CX_FW_NBTAR_RANGING": 2,
    "VL53L8CX_FW_NBTAR_XTALK": 2,
    "VL53L5CX_FW_NBTAR_XTALK": 2,
}

# (C array name, Rust const name)
RUST_TABLES = [
    ("VL53L8CX_DEFAULT_CONFIGURATION", "DEFAULT_CONFIGURATION"),
    ("VL53L8CX_DEFAULT_XTALK", "DEFAULT_XTALK"),
    ("VL53L8CX_GET_NVM_CMD", "GET_NVM_CMD"),
]
FIRMWARE_ARRAY = "VL53L8CX_FIRMWARE"

# The xtalk plugin's calibration command, from the plugin headers. Only used by
# the `xtalk` feature, so the Rust consts are gated (tests keep them visible).
XTALK_TABLE = ("VL53L8CX_CALIBRATE_XTALK", "CALIBRATE_XTALK")
L5_XTALK_TABLE = "VL53L5CX_CALIBRATE_XTALK"
XTALK_MACRO = "VL53L8CX_FW_NBTAR_XTALK"
L5_XTALK_MACRO = "VL53L5CX_FW_NBTAR_XTALK"
XTALK_CFG = '#[cfg(any(feature = "xtalk", test))]'

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


def hex_rows(data, indent="    "):
    rows = []
    for i in range(0, len(data), 16):
        chunk = ", ".join(f"0x{b:02X}" for b in data[i : i + 16])
        rows.append(f"{indent}{chunk},")
    return rows


def xtalk_tables(l8_arrays, l5_arrays):
    """Return (l8_table, macro_index, l5_patch) for the calibrate-xtalk command.

    `l5_patch` is the list of (offset, value) pairs that turn the L8 table into
    the L5 one. Both headers must carry the macro at the same index and the
    patch must not touch it, so one multi-target overwrite serves both parts.
    """
    c_name, _ = XTALK_TABLE
    if c_name not in l8_arrays:
        sys.exit(f"{c_name} not found in xtalk header")
    if L5_XTALK_TABLE not in l5_arrays:
        sys.exit(f"{L5_XTALK_TABLE} not found in L5 xtalk header")
    l8, l8_macros = l8_arrays[c_name]
    l5, l5_macros = l5_arrays[L5_XTALK_TABLE]
    if len(l8_macros) != 1 or l8_macros[0][1] != XTALK_MACRO:
        sys.exit(f"{c_name}: expected exactly one use of {XTALK_MACRO}, got {l8_macros}")
    if len(l5_macros) != 1 or l5_macros[0][1] != L5_XTALK_MACRO:
        sys.exit(f"{L5_XTALK_TABLE}: expected exactly one use of {L5_XTALK_MACRO}, got {l5_macros}")
    macro_index = l8_macros[0][0]
    if l5_macros[0][0] != macro_index:
        sys.exit(
            f"{XTALK_MACRO} sits at {macro_index} in the L8 table "
            f"but at {l5_macros[0][0]} in the L5 one"
        )
    if len(l5) != len(l8):
        sys.exit(f"{c_name} is {len(l8)} bytes but {L5_XTALK_TABLE} is {len(l5)}")
    patch = [(i, l5[i]) for i in range(len(l8)) if l5[i] != l8[i]]
    if any(i == macro_index for i, _ in patch):
        sys.exit(f"L5 xtalk patch overlaps the {XTALK_MACRO} byte at {macro_index}")
    return l8, macro_index, patch


def render_rust(arrays, header_rel, revision, xtalk, xtalk_header_rel, l5_xtalk_header_rel):
    l8_xtalk, xtalk_macro_index, l5_xtalk_patch = xtalk
    out = []
    out.append("// GENERATED FILE - DO NOT EDIT.")
    out.append(f"// Source: {header_rel} (ULD {revision}),")
    out.append(f"//         {xtalk_header_rel}, {l5_xtalk_header_rel}.")
    out.append("// Regenerate with: python3 tools/extract_uld_blobs.py")
    out.append("")
    out.append("//! ST ULD data tables (default configuration, default crosstalk shape and")
    out.append("//! the NVM-read command), transcribed byte-for-byte from `vl53l8cx_buffers.h`,")
    out.append("//! plus the xtalk plugin's calibration command from `vl53l8cx_plugin_xtalk.h`")
    out.append("//! with the two-byte overlay that turns it into the VL53L5CX one.")
    out.append("//! The firmware image from the buffers header is shipped separately as")
    out.append("//! `vl53l8cx_fw_base.bin`. All are checked against the C sources by this")
    out.append("//! crate's oracle tests (linked arrays where the C has external linkage,")
    out.append("//! a re-parse of the header where it is a `static const`).")
    out.append("")
    for c_name, rust_name in RUST_TABLES:
        if c_name not in arrays:
            sys.exit(f"{c_name} not found in header")
        data, macro_uses = arrays[c_name]
        out.append(f"/// C `{c_name}` ({len(data)} bytes).")
        for idx, macro in macro_uses:
            out.append(f"///")
            out.append(f"/// Byte {idx} is the ULD macro `{macro}`, resolved to {MACROS[macro]}")
            out.append(f"/// (one target per zone); `sensor::NB_TARGET_CONFIG_PATCH` overrides it")
            out.append(f"/// for the multi-target features.")
        out.append(f"pub(crate) const {rust_name}: [u8; {len(data)}] = [")
        out.extend(hex_rows(data))
        out.append("];")
        out.append("")

    c_name, rust_name = XTALK_TABLE
    out.append(f"/// C `{c_name}` ({len(l8_xtalk)} bytes): the command sequence")
    out.append("/// `vl53l8cx_calibrate_xtalk` uploads at 0x2C28 to run a crosstalk calibration")
    out.append("/// (`vl53l8cx_plugin_xtalk.h`). This is the VL53L8CX table; apply")
    out.append("/// [`L5_CALIBRATE_XTALK_PATCH`] for the VL53L5CX.")
    out.append("///")
    out.append(f"/// Byte {xtalk_macro_index} ([`FW_NBTAR_XTALK_INDEX`]) is the ULD macro `{XTALK_MACRO}`,")
    out.append(f"/// resolved to {MACROS[XTALK_MACRO]} (one target per zone); `ops::calibrate_xtalk` overrides it")
    out.append("/// for the multi-target features.")
    out.append(XTALK_CFG)
    out.append(f"pub(crate) const {rust_name}: [u8; {len(l8_xtalk)}] = [")
    out.extend(hex_rows(l8_xtalk))
    out.append("];")
    out.append("")
    out.append(f"/// Index of the `{XTALK_MACRO}` macro inside [`CALIBRATE_XTALK`]")
    out.append(f"/// (`{L5_XTALK_MACRO}` sits at the same index in the VL53L5CX table).")
    out.append(XTALK_CFG)
    out.append(f"pub(crate) const FW_NBTAR_XTALK_INDEX: usize = {xtalk_macro_index};")
    out.append("")
    out.append("/// Patch overlaying [`CALIBRATE_XTALK`] (the VL53L8CX table) to produce the")
    out.append(f"/// VL53L5CX `{L5_XTALK_TABLE}` table from `vl53l5cx_plugin_xtalk.h`:")
    out.append(f"/// {len(l5_xtalk_patch)} byte(s) differ. Sorted ascending by offset; never touches")
    out.append("/// [`FW_NBTAR_XTALK_INDEX`]. Not feature-gated: `sensor::SensorInfo` carries it.")
    out.append("pub(crate) const L5_CALIBRATE_XTALK_PATCH: &[(usize, u8)] = &[")
    for off, val in l5_xtalk_patch:
        out.append(f"    (0x{off:04X}, 0x{val:02X}),")
    out.append("];")
    out.append("")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--header", type=pathlib.Path, default=DEFAULT_HEADER)
    ap.add_argument("--api-header", type=pathlib.Path, default=DEFAULT_API_HEADER)
    ap.add_argument("--xtalk-header", type=pathlib.Path, default=DEFAULT_XTALK_HEADER)
    ap.add_argument("--l5-xtalk-header", type=pathlib.Path, default=DEFAULT_L5_XTALK_HEADER)
    ap.add_argument("--out-dir", type=pathlib.Path, default=DEFAULT_OUT_DIR)
    ap.add_argument("--check", action="store_true", help="verify outputs; write nothing")
    args = ap.parse_args()

    arrays = parse_arrays(args.header.read_text())
    if FIRMWARE_ARRAY not in arrays:
        sys.exit(f"{FIRMWARE_ARRAY} not found in header")
    firmware, fw_macros = arrays[FIRMWARE_ARRAY]
    if fw_macros:
        sys.exit(f"{FIRMWARE_ARRAY} unexpectedly contains macros: {fw_macros}")

    xtalk = xtalk_tables(
        parse_arrays(args.xtalk_header.read_text()),
        parse_arrays(args.l5_xtalk_header.read_text()),
    )

    def rel(path):
        try:
            return path.resolve().relative_to(REPO_ROOT).as_posix()
        except ValueError:
            return path.name

    rust_src = render_rust(
        arrays,
        rel(args.header),
        api_revision(args.api_header),
        xtalk,
        rel(args.xtalk_header),
        rel(args.l5_xtalk_header),
    )

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
    l8_xtalk, macro_index, l5_patch = xtalk
    print(
        f"  {XTALK_TABLE[0]}: {len(l8_xtalk)} bytes "
        f"({XTALK_MACRO} at {macro_index}; L5 overlay {len(l5_patch)} bytes)"
    )
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
