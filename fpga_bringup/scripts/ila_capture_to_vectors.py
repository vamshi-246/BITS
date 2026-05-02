#!/usr/bin/env python3
"""Convert a Vivado ILA CSV sweep into decoder output vector files.

Expected minimal ILA probe:
  dbg_packed_output = {dbg_sweep_valid, dbg_hard_bits[3:0]}

The older 3-probe format is also accepted:
  dbg_sweep_valid, dbg_sweep_row, dbg_hard_bits

Only samples with dbg_sweep_valid != 0 are used.
"""

import argparse
import csv
from pathlib import Path


K = 3200
SEG_LEN = 1600
ROW_DEPTH = 800


def parse_int(text, width=None, default_base=10):
    value = str(text).strip().strip('"').strip("'")
    if not value:
        return 0
    value = value.replace("_", "")

    if "'" in value:
        _, value = value.split("'", 1)
        if value and value[0].lower() in ("h", "b", "d"):
            radix = value[0].lower()
            digits = value[1:]
            if radix == "h":
                return int(digits, 16)
            if radix == "b":
                return int(digits, 2)
            return int(digits, 10)

    if value.lower().startswith("0x"):
        return int(value, 16)
    if value.lower().startswith("0b"):
        return int(value, 2)

    if width is not None and all(ch in "01" for ch in value) and len(value) == width:
        return int(value, 2)
    if any(ch.lower() in "abcdef" for ch in value):
        return int(value, 16)
    return int(value, default_base)


def find_header_row(rows):
    for idx, row in enumerate(rows):
        joined = ",".join(row)
        if (
            "dbg_packed_output" in joined
            or "probe0" in joined
            or ("dbg_sweep_valid" in joined and "dbg_sweep_row" in joined)
        ):
            return idx
    raise SystemExit(
        "Could not find ILA CSV header containing dbg_packed_output or dbg_sweep_valid/dbg_sweep_row"
    )


def find_col(headers, name):
    candidates = [i for i, header in enumerate(headers) if name in header]
    if not candidates:
        raise SystemExit(f"Could not find column containing {name}")
    return candidates[0]


def find_optional_col(headers, names):
    for name in names:
        for idx, header in enumerate(headers):
            if name in header:
                return idx
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file", type=Path)
    parser.add_argument(
        "--out-prefix",
        type=Path,
        default=Path("fpga_button_ila"),
        help="Output prefix. Writes <prefix>_hard_bits.txt and <prefix>_rows.csv.",
    )
    args = parser.parse_args()

    with args.csv_file.open(newline="") as f:
        rows = list(csv.reader(f))

    header_idx = find_header_row(rows)
    headers = rows[header_idx]
    data_rows = rows[header_idx + 1 :]

    packed_col = find_optional_col(headers, ["dbg_packed_output", "probe0"])
    legacy_mode = packed_col is None

    if legacy_mode:
        valid_col = find_col(headers, "dbg_sweep_valid")
        row_col = find_col(headers, "dbg_sweep_row")
        hard_col = find_col(headers, "dbg_hard_bits")
        needed_cols = [valid_col, row_col, hard_col]
    else:
        needed_cols = [packed_col]

    bits = [0] * K
    captured = []

    for raw in data_rows:
        if len(raw) <= max(needed_cols):
            continue
        if raw[0].strip().lower().startswith("radix"):
            continue

        if legacy_mode:
            if parse_int(raw[valid_col], width=1) == 0:
                continue
            row = parse_int(raw[row_col], width=10)
            hard = parse_int(raw[hard_col], width=4) & 0xF
        else:
            packed = parse_int(raw[packed_col], width=5, default_base=16) & 0x1F
            if ((packed >> 4) & 1) == 0:
                continue
            row = len(captured)
            hard = packed & 0xF

        if row >= ROW_DEPTH:
            continue

        bits[2 * row] = (hard >> 0) & 1
        bits[SEG_LEN + (2 * row)] = (hard >> 1) & 1
        bits[(2 * row) + 1] = (hard >> 2) & 1
        bits[SEG_LEN + (2 * row) + 1] = (hard >> 3) & 1
        captured.append((row, hard))

    hard_path = args.out_prefix.with_name(args.out_prefix.name + "_hard_bits.txt")
    rows_path = args.out_prefix.with_name(args.out_prefix.name + "_rows.csv")

    with hard_path.open("w", newline="\n") as f:
        for bit in bits:
            f.write(f"{bit}\n")

    with rows_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["row", "hard_bits"])
        for row, hard in captured:
            writer.writerow([row, f"{hard:01x}"])

    print(f"Captured valid rows: {len(captured)} / {ROW_DEPTH}")
    print(f"Wrote {hard_path}")
    print(f"Wrote {rows_path}")


if __name__ == "__main__":
    main()
