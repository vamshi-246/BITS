#!/usr/bin/env python3
"""
Proper LTE Turbo Encoder + AWGN Channel Test Vector Generator
==============================================================
Generates test vectors with ACTUAL turbo-encoded data using
3GPP TS 36.212 compliant trellis termination (12 tail bits).

Steps:
  1. Generate K random information bits
  2. Turbo-encode with LTE constituent code (g0=13, g1=15 octal)
  3. Proper trellis termination: 3 tail bits per encoder
  4. QPP-interleave for second constituent encoder
  5. BPSK modulate: bit 0 -> +1, bit 1 -> -1
  6. Add AWGN noise at specified Eb/N0
  7. Quantize to 5-bit signed LLR
  8. Write to BRAM hex files in the correct folded memory format
  9. Save true information bits for BER comparison

Note on tail bits:
  The LTE turbo encoder appends 3 tail bits per constituent encoder
  after encoding K information bits, driving the shift register to
  the all-zero state. The current RTL target loads those three tail
  trellis steps for the last SISO core and suppresses BER decisions
  for tail positions, so the measured BER is still over K information bits.
"""

import argparse
import os
import numpy as np

# ==============================================================================
# Parameters
# ==============================================================================
K = 3200            # Current RTL block size
NUM_SISO = 2        # Current folded RTL/testbench format supports N=2
S = K // NUM_SISO   # Segment length
f1 = 111            # QPP parameter for K=3200
f2 = 240            # QPP parameter for K=3200
EBN0_DB = 1.0       # Eb/N0 in dB for a noisy, reproducible regression block
CODE_RATE = 0.375   # Paper-normalized channel/code rate used for RTL vectors

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data')

QPP_TABLE = {
    3200: (111, 240),
    6144: (263, 480),
}

# ==============================================================================
# QPP Interleaver
# ==============================================================================
def qpp(k, K=None, f1=None, f2=None):
    """QPP interleaver: pi(k) = (f1*k + f2*k^2) mod K"""
    if K is None:
        K = globals()["K"]
    if f1 is None:
        f1 = globals()["f1"]
    if f2 is None:
        f2 = globals()["f2"]
    return (f1 * k + f2 * k * k) % K

pi = [qpp(k) for k in range(K)]

# ==============================================================================
# LTE Constituent Encoder with Proper Trellis Termination
# ==============================================================================
def lte_rsc_encode(info_bits):
    """
    LTE rate-1/2 RSC encoder with 3GPP TS 36.212 trellis termination.

    Polynomials:
      g0 = 13 octal = 1 + D^2 + D^3  (feedback)
      g1 = 15 octal = 1 + D + D^3    (feedforward)

    State = [s0, s1, s2] where s0 is the most recent feedback bit.
      feedback = u_k XOR s[1] XOR s[2]
      parity   = feedback XOR s[0] XOR s[2]

    After K information bits, 3 termination steps drive state to zero:
      tail_input = s[1] XOR s[2]  (makes feedback = 0)

    Returns:
      sys_bits: length K+3 (K info bits + 3 tail systematic bits)
      par_bits: length K+3 (K parity bits + 3 tail parity bits)
    """
    n = len(info_bits)
    sys_bits = np.zeros(n + 3, dtype=int)
    par_bits = np.zeros(n + 3, dtype=int)
    sys_bits[:n] = info_bits

    s = [0, 0, 0]

    # Normal encoding of K information bits
    for k in range(n):
        feedback = int(info_bits[k]) ^ s[1] ^ s[2]
        par_bits[k] = feedback ^ s[0] ^ s[2]
        s[2] = s[1]; s[1] = s[0]; s[0] = feedback

    # 3 termination steps
    for i in range(3):
        tail_input = s[1] ^ s[2]  # forces feedback to 0
        sys_bits[n + i] = tail_input
        feedback = tail_input ^ s[1] ^ s[2]  # = 0
        par_bits[n + i] = feedback ^ s[0] ^ s[2]
        s[2] = s[1]; s[1] = s[0]; s[0] = feedback

    assert s == [0, 0, 0], "Encoder did not terminate to zero state!"
    return sys_bits, par_bits


# ==============================================================================
# Full Turbo Encoder
# ==============================================================================
def turbo_encode(info_bits):
    """
    LTE turbo encoder with proper trellis termination:
      - Constituent 1: encode info_bits -> sys1[K+3], par1[K+3]
      - Constituent 2: encode interleaved info_bits -> sys2[K+3], par2[K+3]
    Returns: sys1, par1, sys2, par2 (each length K+3)
    """
    interleaved_bits = np.array([info_bits[pi[k]] for k in range(K)])
    sys1, par1 = lte_rsc_encode(info_bits)
    sys2, par2 = lte_rsc_encode(interleaved_bits)
    return sys1, par1, sys2, par2

# ==============================================================================
# AWGN Channel
# ==============================================================================
def awgn_channel(bits, ebn0_db, code_rate=1.0/3):
    """
    BPSK modulate, add AWGN noise, compute channel LLRs.
    bit 0 -> +1, bit 1 -> -1

    LLR = 2*y/sigma^2 (exact for AWGN + BPSK)
    """
    # BPSK modulation
    x = 1.0 - 2.0 * bits.astype(float)  # 0 -> +1, 1 -> -1

    # Noise variance
    ebn0 = 10.0 ** (ebn0_db / 10.0)
    # For rate R code: Es/N0 = R * Eb/N0
    # sigma^2 = 1 / (2 * R * Eb/N0)
    sigma2 = 1.0 / (2.0 * code_rate * ebn0)
    sigma = np.sqrt(sigma2)

    # Add noise
    noise = sigma * np.random.randn(len(bits))
    y = x + noise

    # Compute LLR = 2*y / sigma^2
    llr = 2.0 * y / sigma2

    return llr, y

# ==============================================================================
# Quantize LLR to N-bit signed integer
# ==============================================================================
def quantize_llr(llr_float, n_bits=5):
    """
    Quantize floating-point LLR to n_bits signed integer.
    Range: [-(2^(n-1)), 2^(n-1)-1]
    """
    max_val = (1 << (n_bits - 1)) - 1   # +15 for 5-bit
    min_val = -(1 << (n_bits - 1))       # -16 for 5-bit

    # Scale: map the LLR range to the quantized range
    # For Eb/N0 = 1 dB, typical LLR magnitude is ~2-4
    # We want to use most of the dynamic range
    scale = max_val / 4.0  # Adjust this for different SNR points

    quantized = np.round(llr_float * scale).astype(int)
    quantized = np.clip(quantized, min_val, max_val)

    return quantized

# ==============================================================================
# Write BRAM hex files
# ==============================================================================
def to_unsigned_5bit(signed_val):
    """Convert signed 5-bit (-16..+15) to unsigned (0..31)."""
    return int(signed_val) & 0x1F

def write_bram_10bit(fname, col0_vals, col1_vals):
    """Write BRAM hex file: each row = {col1[4:0], col0[4:0]} = 10-bit."""
    with open(fname, "w") as f:
        for c0, c1 in zip(col0_vals, col1_vals):
            val = ((to_unsigned_5bit(c1)) << 5) | to_unsigned_5bit(c0)
            f.write(f"{val:03x}\n")

def get_or_zero(vals, idx):
    return int(vals[idx]) if 0 <= idx < len(vals) else 0


def write_all_bram_files(L_sys_q, L_par1_q, L_par2_q, tail_len=0, L_sys2_q=None):
    """
    Write all 8 Group-A BRAM hex files in the folded memory format.

    Natural-order BRAMs (for SISO half-iterations 0, 2, 4, ...):
      sys_even_ram[r]  = {L_s[2r+S], L_s[2r]}       for r = 0..1535
      sys_odd_ram[r]   = {L_s[2r+1+S], L_s[2r+1]}
      par1_even_ram[r] = {L_p1[2r+S], L_p1[2r]}
      par1_odd_ram[r]  = {L_p1[2r+1+S], L_p1[2r+1]}

    Interleaved-order BRAMs (for SISO half-iterations 1, 3, 5, ...):
      sys_ilv_even_ram[r]  = {L_s[pi(2r)+S mod K], L_s[pi(2r)]}
      sys_ilv_odd_ram[r]   = {L_s[pi(2r+1)+S mod K], L_s[pi(2r+1)]}
      par2_even_ram[r]     = {L_p2[2r+S], L_p2[2r]}
      par2_odd_ram[r]      = {L_p2[2r+1+S], L_p2[2r+1]}
    """
    if L_sys2_q is None:
        L_sys2_q = L_sys_q

    n_rows = (S + tail_len + 1) // 2

    def core0_info(vals, local):
        return get_or_zero(vals, local) if local < S else 0

    def core1_info_or_tail(vals, local):
        return get_or_zero(vals, S + local) if local < S else get_or_zero(vals, K + (local - S))

    def interleaved_sys_core0(local):
        return get_or_zero(L_sys_q, pi[local]) if local < S else 0

    def interleaved_sys_core1(local):
        if local < S:
            return get_or_zero(L_sys_q, pi[S + local])
        return get_or_zero(L_sys2_q, K + (local - S))

    # --- Natural order ---
    sys_even_c0  = [core0_info(L_sys_q, 2*r) for r in range(n_rows)]
    sys_even_c1  = [core1_info_or_tail(L_sys_q, 2*r) for r in range(n_rows)]
    sys_odd_c0   = [core0_info(L_sys_q, 2*r + 1) for r in range(n_rows)]
    sys_odd_c1   = [core1_info_or_tail(L_sys_q, 2*r + 1) for r in range(n_rows)]

    par1_even_c0 = [core0_info(L_par1_q, 2*r) for r in range(n_rows)]
    par1_even_c1 = [core1_info_or_tail(L_par1_q, 2*r) for r in range(n_rows)]
    par1_odd_c0  = [core0_info(L_par1_q, 2*r + 1) for r in range(n_rows)]
    par1_odd_c1  = [core1_info_or_tail(L_par1_q, 2*r + 1) for r in range(n_rows)]

    write_bram_10bit(os.path.join(DATA_DIR, "sys_even_ram.hex"),  sys_even_c0,  sys_even_c1)
    write_bram_10bit(os.path.join(DATA_DIR, "sys_odd_ram.hex"),   sys_odd_c0,   sys_odd_c1)
    write_bram_10bit(os.path.join(DATA_DIR, "par1_even_ram.hex"), par1_even_c0, par1_even_c1)
    write_bram_10bit(os.path.join(DATA_DIR, "par1_odd_ram.hex"),  par1_odd_c0,  par1_odd_c1)

    # --- Interleaved order ---
    silv_even_c0 = [interleaved_sys_core0(2*r) for r in range(n_rows)]
    silv_even_c1 = [interleaved_sys_core1(2*r) for r in range(n_rows)]
    silv_odd_c0  = [interleaved_sys_core0(2*r + 1) for r in range(n_rows)]
    silv_odd_c1  = [interleaved_sys_core1(2*r + 1) for r in range(n_rows)]

    par2_even_c0 = [core0_info(L_par2_q, 2*r) for r in range(n_rows)]
    par2_even_c1 = [core1_info_or_tail(L_par2_q, 2*r) for r in range(n_rows)]
    par2_odd_c0  = [core0_info(L_par2_q, 2*r + 1) for r in range(n_rows)]
    par2_odd_c1  = [core1_info_or_tail(L_par2_q, 2*r + 1) for r in range(n_rows)]

    write_bram_10bit(os.path.join(DATA_DIR, "sys_ilv_even_ram.hex"), silv_even_c0, silv_even_c1)
    write_bram_10bit(os.path.join(DATA_DIR, "sys_ilv_odd_ram.hex"),  silv_odd_c0,  silv_odd_c1)
    write_bram_10bit(os.path.join(DATA_DIR, "par2_even_ram.hex"),    par2_even_c0, par2_even_c1)
    write_bram_10bit(os.path.join(DATA_DIR, "par2_odd_ram.hex"),     par2_odd_c0,  par2_odd_c1)

    # --- Initial extrinsic BRAM (all zeros) ---
    with open(os.path.join(DATA_DIR, "initial_extrinsic.hex"), "w") as f:
        for _ in range(S // 2):
            f.write("000\n")

# ==============================================================================
# Save true information bits for BER comparison
# ==============================================================================
def save_true_bits(info_bits):
    """Save the original information bits to a file for BER comparison."""
    path = os.path.join(DATA_DIR, "true_info_bits.txt")
    with open(path, "w") as f:
        for b in info_bits:
            f.write(f"{int(b)}\n")
    print(f"  Saved {len(info_bits)} true info bits to {path}")

# ==============================================================================
# Main
# ==============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--K", type=int, default=K)
    parser.add_argument("--num-siso", type=int, default=NUM_SISO)
    parser.add_argument("--ebn0-db", type=float, default=EBN0_DB)
    parser.add_argument(
        "--channel-rate",
        type=float,
        default=CODE_RATE,
        help="AWGN code/channel rate used to convert Eb/N0 to noise variance",
    )
    parser.add_argument(
        "--include-tail",
        action="store_true",
        default=True,
        help="write K+3 trellis rows for the last SISO core; enabled by default for current RTL",
    )
    parser.add_argument(
        "--no-tail",
        action="store_false",
        dest="include_tail",
        help="disable tail rows for legacy experiments",
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--data-dir", default=DATA_DIR)
    args = parser.parse_args()

    if args.num_siso != 2:
        raise SystemExit("This BRAM writer targets the current folded N=2 RTL format; use --num-siso 2.")
    if args.K not in QPP_TABLE:
        raise SystemExit(f"K={args.K} not supported; known QPP entries: {sorted(QPP_TABLE)}")

    K = args.K
    NUM_SISO = args.num_siso
    S = K // NUM_SISO
    f1, f2 = QPP_TABLE[K]
    pi = [qpp(k) for k in range(K)]
    EBN0_DB = args.ebn0_db
    CODE_RATE = args.channel_rate
    if CODE_RATE <= 0:
        raise SystemExit("--channel-rate must be positive")
    DATA_DIR = os.path.abspath(args.data_dir)
    os.makedirs(DATA_DIR, exist_ok=True)
    np.random.seed(args.seed)
    tail_len = 3 if args.include_tail else 0

    print("=" * 70)
    print("  LTE Turbo Encoder + AWGN Channel Test Vector Generator")
    print("  (3GPP TS 36.212 compliant trellis termination)")
    print("=" * 70)

    # Step 1: Generate random information bits (NO bit-flipping needed!)
    info_bits = np.random.randint(0, 2, K).astype(np.uint8)
    ones = int(np.sum(info_bits))
    print(f"\n  Block size K = {K}")
    print(f"  NUM_SISO = {NUM_SISO}, segment length S = {S}")
    print(f"  QPP f1={f1}, f2={f2}")
    print(f"  Info bits: {ones} ones, {K - ones} zeros")

    # Step 2: Turbo encode with proper tail bits
    print("\n  Turbo encoding (with trellis termination)...")
    sys1, par1, sys2, par2 = turbo_encode(info_bits)
    print(f"  Encoder 1: {K} info + 3 tail = {len(sys1)} total trellis steps")
    print(f"  Encoder 2: {K} info + 3 tail = {len(sys2)} total trellis steps")
    print(f"  Encoder 1 terminal state: verified zero")
    print(f"  Encoder 2 terminal state: verified zero")

    # Verify: first K systematic bits should equal info bits
    assert np.array_equal(sys1[:K], info_bits), "Systematic bits mismatch!"
    print(f"  Systematic bits verified (= info bits)")
    print(f"  Par1: {sum(par1)} ones, Par2: {sum(par2)} ones")

    # Step 3 & 4: BPSK + AWGN channel
    print(f"\n  Eb/N0 = {EBN0_DB} dB, code rate = {CODE_RATE:.4f}")
    print(f"  RTL trellis tail rows: {'enabled' if args.include_tail else 'disabled'}")

    L_sys_f,  _ = awgn_channel(sys1[:K + tail_len], EBN0_DB, CODE_RATE)
    L_par1_f, _ = awgn_channel(par1[:K + tail_len], EBN0_DB, CODE_RATE)
    L_par2_f, _ = awgn_channel(par2[:K + tail_len], EBN0_DB, CODE_RATE)
    if args.include_tail:
        L_sys2_tail_f, _ = awgn_channel(sys2[K:K + tail_len], EBN0_DB, CODE_RATE)
        L_sys2_f = np.concatenate([L_sys_f[:K], L_sys2_tail_f])
    else:
        L_sys2_f = L_sys_f

    # Channel BER (hard decision on noisy systematic before decoding)
    channel_errors = sum(1 for i in range(K) if (L_sys_f[i] < 0) != (info_bits[i] == 1))
    channel_ber = channel_errors / K
    print(f"  Channel BER (before decoding): {channel_ber:.6f}  ({channel_errors}/{K})")

    # Step 5: Quantize to 5-bit
    L_sys_q  = quantize_llr(L_sys_f,  n_bits=5)
    L_sys2_q = quantize_llr(L_sys2_f, n_bits=5)
    L_par1_q = quantize_llr(L_par1_f, n_bits=5)
    L_par2_q = quantize_llr(L_par2_f, n_bits=5)

    # Quantization BER
    quant_errors = sum(1 for i in range(K) if (L_sys_q[i] < 0) != (info_bits[i] == 1))
    quant_ber = quant_errors / K
    print(f"  Quantized sys BER:             {quant_ber:.6f}  ({quant_errors}/{K})")

    # LLR statistics
    print(f"\n  Quantized LLR statistics:")
    print(f"    L_sys:  min={L_sys_q.min():3d}, max={L_sys_q.max():3d}, "
          f"avg|L|={np.mean(np.abs(L_sys_q)):.2f}")
    print(f"    L_par1: min={L_par1_q.min():3d}, max={L_par1_q.max():3d}, "
          f"avg|L|={np.mean(np.abs(L_par1_q)):.2f}")
    print(f"    L_par2: min={L_par2_q.min():3d}, max={L_par2_q.max():3d}, "
          f"avg|L|={np.mean(np.abs(L_par2_q)):.2f}")

    # Step 6: Write BRAM hex files
    print(f"\n  Writing BRAM hex files to {DATA_DIR}...")
    write_all_bram_files(L_sys_q, L_par1_q, L_par2_q, tail_len=tail_len, L_sys2_q=L_sys2_q)
    print("  Done writing all 9 BRAM hex files.")

    # Step 7: Save true bits
    save_true_bits(info_bits)

    # Step 8: Save tail bits for future RTL enhancement
    tail_path = os.path.join(DATA_DIR, "tail_bits.txt")
    with open(tail_path, "w") as f:
        f.write("# Encoder 1 tail: sys, par pairs\n")
        for i in range(3):
            f.write(f"enc1_tail_{i}: sys={sys1[K+i]} par={par1[K+i]}\n")
        f.write("# Encoder 2 tail: sys, par pairs\n")
        for i in range(3):
            f.write(f"enc2_tail_{i}: sys={sys2[K+i]} par={par2[K+i]}\n")
    print(f"  Saved tail bits to {tail_path}")

    meta_path = os.path.join(DATA_DIR, "vector_metadata.txt")
    with open(meta_path, "w") as f:
        f.write(f"K={K}\n")
        f.write(f"NUM_SISO={NUM_SISO}\n")
        f.write(f"SEG_LEN={S}\n")
        f.write(f"QPP_F1={f1}\n")
        f.write(f"QPP_F2={f2}\n")
        f.write(f"EBN0_DB={EBN0_DB}\n")
        f.write(f"CODE_RATE={CODE_RATE}\n")
        f.write(f"SEED={args.seed}\n")
        f.write(f"TAIL_LEN={tail_len}\n")
        f.write(f"CHANNEL_ERRORS={channel_errors}\n")
        f.write(f"CHANNEL_BER={channel_ber:.12f}\n")
        f.write(f"QUANT_SYS_ERRORS={quant_errors}\n")
        f.write(f"QUANT_SYS_BER={quant_ber:.12f}\n")
    print(f"  Saved vector metadata to {meta_path}")

    print(f"\n  Next steps:")
    print(f"    1. Run the Verilog simulation (tb_turbo_decoder)")
    print(f"    2. Run turbo_ref_model.py to compare RTL extrinsics and compute BER")
    print(f"    3. Run monte_carlo_ber.py for algorithmic BER curve validation")
    print("=" * 70)
