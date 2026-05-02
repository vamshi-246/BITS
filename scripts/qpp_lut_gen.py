#!/usr/bin/env python3
"""Generate the N=2 SISO-0 QPP LUT used by rtl/qpp_lut.v."""

import argparse
import os

QPP_TABLE = {
    3200: (111, 240),
    6144: (263, 480),
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--K", type=int, default=3200)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    if args.K not in QPP_TABLE:
        raise SystemExit(f"K={args.K} not supported; known QPP entries: {sorted(QPP_TABLE)}")

    k_len = args.K
    f1, f2 = QPP_TABLE[k_len]
    seg_len = k_len // 2
    hex_width = max(1, ((k_len - 1).bit_length() + 3) // 4)
    out = args.out
    if out is None:
        data_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "data"))
        out = os.path.join(data_dir, f"qpp_{k_len}.hex")

    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w") as f:
        for k in range(seg_len):
            pi_k = (f1 * k + f2 * k * k) % k_len
            f.write(f"{pi_k:0{hex_width}x}\n")

    pi_0 = (f1 * 0 + f2 * 0 * 0) % k_len
    pi_1 = (f1 * 1 + f2 * 1 * 1) % k_len
    pi_last = (f1 * (seg_len - 1) + f2 * (seg_len - 1) * (seg_len - 1)) % k_len
    assert pi_0 == 0
    assert 0 <= pi_1 < k_len
    assert 0 <= pi_last < k_len
    assert ((pi_0 + seg_len) % k_len) == ((f1 * seg_len + f2 * seg_len * seg_len) % k_len)

    print(f"Generated {out}")
    print(f"  K={k_len}, f1={f1}, f2={f2}, entries={seg_len}")
    print(f"  pi(1)={pi_1} (0x{pi_1:04x}), pi({seg_len - 1})={pi_last} (0x{pi_last:04x})")


if __name__ == "__main__":
    main()
