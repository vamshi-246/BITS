#!/usr/bin/env python3
"""Compare FPGA ILA hard bits against RTL and truth vectors."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_FPGA = ROOT / "fpga_bringup" / "fpga_button_ila_hard_bits.txt"
DEFAULT_RTL = ROOT / "data" / "rtl_final_hard_bits.txt"
DEFAULT_TRUTH = ROOT / "data" / "true_info_bits.txt"


def read_bits(path):
    return [line.strip() for line in path.read_text().splitlines() if line.strip()]


def compare(label, actual, expected):
    n = min(len(actual), len(expected))
    mismatches = [i for i in range(n) if actual[i] != expected[i]]
    total = len(mismatches) + abs(len(actual) - len(expected))

    print(f"{label}: {total} mismatches")
    print(f"  actual lines:   {len(actual)}")
    print(f"  expected lines: {len(expected)}")
    if mismatches:
        print(f"  first mismatch indices: {mismatches[:10]}")
    return total


def main():
    fpga_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_FPGA
    rtl_path = Path(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_RTL
    truth_path = Path(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_TRUTH

    fpga_bits = read_bits(fpga_path)
    rtl_bits = read_bits(rtl_path)
    truth_bits = read_bits(truth_path)

    rtl_mismatches = compare("FPGA vs RTL hard bits", fpga_bits, rtl_bits)
    compare("FPGA vs true_info_bits", fpga_bits, truth_bits)

    return 0 if rtl_mismatches == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
