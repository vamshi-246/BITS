#!/usr/bin/env python3
"""
Generate test vector hex files for all 8 Group-A BRAMs.
Each file has 1536 lines. Each line is one row in hex.
For Group-A 5-bit BRAMs: row = 10-bit value = {col1[4:0], col0[4:0]}.

Outputs 9 files:
  - sys_even_ram.hex, sys_odd_ram.hex
  - par1_even_ram.hex, par1_odd_ram.hex
  - sys_ilv_even_ram.hex, sys_ilv_odd_ram.hex
  - par2_even_ram.hex, par2_odd_ram.hex
  - initial_extrinsic.hex  (all zeros, for extrinsic BRAM init)
"""

import random

# Fix seed for reproducibility
random.seed(42)

K  = 6144
S  = 3072  # Segment length = K / 2
f1 = 263
f2 = 480

def qpp(k):
    """QPP interleaver: pi(k) = (f1*k + f2*k^2) mod K"""
    return (f1 * k + f2 * k * k) % K

# Precompute full interleaver table
pi = [qpp(k) for k in range(K)]

def rand_llr():
    """Random 5-bit signed LLR as unsigned (range [-16,+15] → [0,31])"""
    return random.randint(-16, 15) & 0x1F

# Generate random channel LLRs for full block
L_s  = [rand_llr() for _ in range(K)]
L_p1 = [rand_llr() for _ in range(K)]
L_p2 = [rand_llr() for _ in range(K)]

def write_bram(fname, rows):
    """Write BRAM hex file: each row is {col1[4:0], col0[4:0]} = 10-bit."""
    with open(fname, "w") as f:
        for c0, c1 in rows:
            val = ((c1 & 0x1F) << 5) | (c0 & 0x1F)
            f.write(f"{val:03x}\n")

# ===========================================================================
# Natural-order BRAMs
# SISO-0 = global steps [0..3071], SISO-1 = global steps [3072..6143]
# Folded memory: row r stores step 2r (even) and step 2r+1 (odd)
# col0 = SISO-0 value, col1 = SISO-1 value
# ===========================================================================
write_bram("../data/sys_even_ram.hex",
    [(L_s[2*r], L_s[2*r + S]) for r in range(S // 2)])
write_bram("../data/sys_odd_ram.hex",
    [(L_s[2*r + 1], L_s[2*r + 1 + S]) for r in range(S // 2)])
write_bram("../data/par1_even_ram.hex",
    [(L_p1[2*r], L_p1[2*r + S]) for r in range(S // 2)])
write_bram("../data/par1_odd_ram.hex",
    [(L_p1[2*r + 1], L_p1[2*r + 1 + S]) for r in range(S // 2)])

# ===========================================================================
# Interleaved-order BRAMs (pre-permuted by π)
# col0 = L_s[π(2r)] for SISO-0, col1 = L_s[(π(2r)+S) % K] for SISO-1
# ===========================================================================
write_bram("../data/sys_ilv_even_ram.hex",
    [(L_s[pi[2*r]], L_s[(pi[2*r] + S) % K]) for r in range(S // 2)])
write_bram("../data/sys_ilv_odd_ram.hex",
    [(L_s[pi[2*r + 1]], L_s[(pi[2*r + 1] + S) % K]) for r in range(S // 2)])
write_bram("../data/par2_even_ram.hex",
    [(L_p2[pi[2*r]], L_p2[(pi[2*r] + S) % K]) for r in range(S // 2)])
write_bram("../data/par2_odd_ram.hex",
    [(L_p2[pi[2*r + 1]], L_p2[(pi[2*r + 1] + S) % K]) for r in range(S // 2)])

# ===========================================================================
# Initial extrinsic BRAM (all zeros — 12-bit words)
# ===========================================================================
with open("../data/initial_extrinsic.hex", "w") as f:
    for _ in range(1536):
        f.write("000\n")

print("Test vector hex files generated successfully:")
print("  Natural:     sys_even_ram.hex, sys_odd_ram.hex, par1_even_ram.hex, par1_odd_ram.hex")
print("  Interleaved: sys_ilv_even_ram.hex, sys_ilv_odd_ram.hex, par2_even_ram.hex, par2_odd_ram.hex")
print("  Extrinsic:   initial_extrinsic.hex")

# Verification
print(f"\nVerification:")
print(f"  pi(0) = {pi[0]}, pi(1) = {pi[1]}")
print(f"  L_s[0] = {L_s[0]:#05b}, L_s[3072] = {L_s[3072]:#05b}")
print(f"  sys_even_ram[0] = col0:{L_s[0]:#05b} col1:{L_s[S]:#05b}")
print(f"  sys_ilv_even_ram[0] = col0:{L_s[pi[0]]:#05b} col1:{L_s[(pi[0]+S)%K]:#05b}")
