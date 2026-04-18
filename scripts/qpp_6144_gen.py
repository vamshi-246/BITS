#!/usr/bin/env python3
"""
Generate QPP interleaver LUT for LTE Turbo Decoder.
K=6144, f1=263, f2=480.
Output: qpp_6144.hex — 3072 lines, each 4 hex digits (13-bit values).
Entry k = pi(k) = (263*k + 480*k^2) mod 6144, for k in [0..3071].
"""

K  = 6144
f1 = 263
f2 = 480

with open("../data/qpp_6144.hex", "w") as f:
    for k in range(K // 2):   # 3072 entries for SISO-0 segment
        pi_k = (f1 * k + f2 * k * k) % K
        f.write(f"{pi_k:04x}\n")   # 4 hex digits, 13-bit value fits in 16 bits

# Verification
pi_1 = (f1 * 1 + f2 * 1 * 1) % K
print(f"Verify: pi(1) = {pi_1} (expected 743, hex 0x{pi_1:04x})")
assert pi_1 == 743, f"pi(1) mismatch: got {pi_1}"

pi_0 = (f1 * 0 + f2 * 0 * 0) % K
print(f"Verify: pi(0) = {pi_0} (expected 0)")
assert pi_0 == 0

pi_3071 = (f1 * 3071 + f2 * 3071 * 3071) % K
print(f"Verify: pi(3071) = {pi_3071} (hex 0x{pi_3071:04x})")
assert pi_3071 < K, f"pi(3071) out of range: {pi_3071}"

print("qpp_6144.hex generated successfully (3072 entries).")
