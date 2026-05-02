#!/usr/bin/env python3
"""
Monte Carlo BER Simulation for LTE Turbo Decoder
=================================================
Proper 3GPP TS 36.212 trellis termination (12 tail bits).
Full-block Max-Log M-BCJR decoder.

Reproduces the BER curve from:
  Studer et al., "Design and Implementation of a Parallel
  Turbo-Decoder ASIC for 3GPP-LTE", IEEE JSSC 2011, Fig. 9.

Usage:
  python monte_carlo_ber.py --K 3200
  python monte_carlo_ber.py --K 3200 --ebn0-start 0 --ebn0-stop 3
  python monte_carlo_ber.py --K 6144 --ebn0-start 0 --ebn0-stop 3
  python monte_carlo_ber.py --K 3200 --quantize
"""

import argparse
import os
import time
import numpy as np

# ==========================================================================
# QPP Parameters (3GPP TS 36.212, Table 5.1.3-3)
# ==========================================================================
QPP_TABLE = {
    40:   (3, 10),
    48:   (7, 12),
    64:   (7, 16),
    128:  (15, 32),
    256:  (15, 64),
    512:  (31, 64),
    1024: (17, 66),
    2048: (33, 130),
    3200: (111, 240),
    4096: (299, 256),
    6144: (263, 480),
}

NUM_STATES = 8
NEG_INF = -1.0e30

# LTE trellis: radix-2 predecessors per destination state
R2_PREDS = [
    (0, 1), (2, 3), (4, 5), (6, 7),
    (0, 1), (2, 3), (4, 5), (6, 7),
]

# PRE index mapping (matches rtl/bm_radix2.v)
R2_PRE_IDX = np.array(
    [0, 3, 2, 1, 1, 2, 3, 0, 3, 0, 1, 2, 2, 1, 0, 3],
    dtype=np.int32,
)

# Build transition table once
TRANSITIONS = []
TRANS_BY_PRED = [[] for _ in range(NUM_STATES)]
for _dest in range(NUM_STATES):
    for _pidx, _pred in enumerate(R2_PREDS[_dest]):
        _bm_idx = _dest * 2 + _pidx
        _xs = int(R2_PRE_IDX[_bm_idx]) >> 1
        _item = (_pred, _dest, _bm_idx, _xs)
        TRANSITIONS.append(_item)
        TRANS_BY_PRED[_pred].append(_item)


# ==========================================================================
# LTE Constituent Encoder with Proper Tail Bits
# ==========================================================================
def lte_rsc_encode(info_bits):
    """
    LTE rate-1/2 RSC encoder with 3GPP trellis termination.
    g0 = 13 octal (feedback), g1 = 15 octal (feedforward).

    Returns: sys_bits[K+3], par_bits[K+3]
    The last 3 positions are the tail bits.
    """
    K = len(info_bits)
    sys_out = np.zeros(K + 3, dtype=np.int32)
    par_out = np.zeros(K + 3, dtype=np.int32)
    sys_out[:K] = info_bits
    s = [0, 0, 0]

    # Normal encoding
    for k in range(K):
        feedback = int(info_bits[k]) ^ s[1] ^ s[2]
        par_out[k] = feedback ^ s[0] ^ s[2]
        s[2] = s[1]; s[1] = s[0]; s[0] = feedback

    # 3 termination steps: input = s[1]^s[2] makes feedback = 0
    for i in range(3):
        tail_in = s[1] ^ s[2]
        sys_out[K + i] = tail_in
        feedback = tail_in ^ s[1] ^ s[2]  # = 0
        par_out[K + i] = feedback ^ s[0] ^ s[2]  # = s[0] ^ s[2]
        s[2] = s[1]; s[1] = s[0]; s[0] = feedback

    assert s == [0, 0, 0], "Trellis termination failed!"
    return sys_out, par_out


def turbo_encode(info_bits, pi):
    """
    LTE turbo encoder: two constituent RSC encoders + QPP interleaver.
    Returns: sys1[K+3], par1[K+3], sys2[K+3], par2[K+3]
    """
    K = len(info_bits)
    sys1, par1 = lte_rsc_encode(info_bits)
    interleaved = np.array([info_bits[pi[k]] for k in range(K)], dtype=np.int32)
    sys2, par2 = lte_rsc_encode(interleaved)
    return sys1, par1, sys2, par2


# ==========================================================================
# AWGN Channel
# ==========================================================================
def awgn_channel(bits, ebn0_db, code_rate, rng):
    """BPSK modulate, add AWGN, return channel LLRs."""
    x = 1.0 - 2.0 * bits.astype(np.float64)  # 0->+1, 1->-1
    ebn0 = 10.0 ** (ebn0_db / 10.0)
    sigma2 = 1.0 / (2.0 * code_rate * ebn0)
    noise = np.sqrt(sigma2) * rng.standard_normal(len(bits))
    y = x + noise
    llr = 2.0 * y / sigma2
    return llr


def quantize_llr(llr, n_bits=5, scale_factor=None):
    """Quantize float LLR to n-bit signed integer."""
    max_val = (1 << (n_bits - 1)) - 1
    min_val = -(1 << (n_bits - 1))
    if scale_factor is None:
        scale_factor = max_val / 4.0
    q = np.round(llr * scale_factor).astype(np.float64)
    return np.clip(q, min_val, max_val)


# ==========================================================================
# Full-Block Max-Log BCJR Decoder
# ==========================================================================
def branch_metrics(sys_llr, par_llr, apr_llr):
    """Compute branch metrics for K_total trellis steps."""
    sa = sys_llr + apr_llr
    pre = np.column_stack([
        sa + par_llr,   # PRE[0]
        sa - par_llr,   # PRE[1]
        -sa + par_llr,  # PRE[2]
        -sa - par_llr,  # PRE[3]
    ])
    K_total = len(sys_llr)
    gamma = np.empty((K_total, NUM_STATES, 2), dtype=np.float64)
    for dest in range(NUM_STATES):
        gamma[:, dest, 0] = pre[:, R2_PRE_IDX[2 * dest]]
        gamma[:, dest, 1] = pre[:, R2_PRE_IDX[2 * dest + 1]]
    return gamma


def normalize_row(row):
    vmax = np.max(row)
    if vmax > NEG_INF / 2:
        row -= vmax
    return row


def siso_decode(sys_llr, par_llr, apr_llr, K_info, scale=0.6875,
                extr_clip=0, apr_clip=0):
    """
    Full-block Max-Log BCJR for one constituent code.

    sys_llr, par_llr: length K_info + 3 (includes tail)
    apr_llr: length K_info (no a-priori for tail)
    Returns: extrinsic[K_info], intrinsic[K_info]
    """
    K_total = K_info + 3

    # Pad a-priori with zeros for tail positions
    apr_full = np.zeros(K_total, dtype=np.float64)
    apr_full[:K_info] = apr_llr

    gamma = branch_metrics(sys_llr, par_llr, apr_full)

    # Forward recursion
    alpha = np.full((K_total + 1, NUM_STATES), NEG_INF)
    alpha[0, 0] = 0.0
    for k in range(K_total):
        for dest in range(NUM_STATES):
            preds = R2_PREDS[dest]
            alpha[k + 1, dest] = max(
                alpha[k, preds[0]] + gamma[k, dest, 0],
                alpha[k, preds[1]] + gamma[k, dest, 1],
            )
        normalize_row(alpha[k + 1])

    # Backward recursion (known terminal state = 0)
    beta = np.full((K_total + 1, NUM_STATES), NEG_INF)
    beta[K_total, 0] = 0.0
    for k in range(K_total - 1, -1, -1):
        for pred in range(NUM_STATES):
            vals = []
            for _, dest, bm_idx, _ in TRANS_BY_PRED[pred]:
                vals.append(beta[k + 1, dest] + gamma[k, dest, bm_idx & 1])
            beta[k, pred] = max(vals)
        normalize_row(beta[k])

    # LLR computation (only for K_info information bits)
    intrinsic = np.zeros(K_info, dtype=np.float64)
    extrinsic = np.zeros(K_info, dtype=np.float64)
    for k in range(K_info):
        paths0, paths1 = [], []
        for pred, dest, bm_idx, xs in TRANSITIONS:
            metric = alpha[k, pred] + gamma[k, dest, bm_idx & 1] + beta[k + 1, dest]
            if xs == 0:
                paths0.append(metric)
            else:
                paths1.append(metric)
        ld = max(paths0) - max(paths1)
        intrinsic[k] = ld
        extrinsic[k] = scale * (ld - sys_llr[k] - apr_llr[k])

    if extr_clip > 0:
        lo = -(1 << (extr_clip - 1))
        hi = (1 << (extr_clip - 1)) - 1
        extrinsic = np.clip(extrinsic, lo, hi)

    return extrinsic, intrinsic


# ==========================================================================
# Turbo Decoder (iterative)
# ==========================================================================
def turbo_decode(sys1_llr, par1_llr, sys2_llr, par2_llr, pi, K,
                 half_iters=11, scale=0.6875, extr_clip=0, apr_clip=0):
    """
    Iterative turbo decoder with proper tail bit handling.

    sys1_llr, par1_llr: length K+3 (natural + tail1)
    sys2_llr, par2_llr: length K+3 (interleaved + tail2)
    pi: interleaver permutation array (length K)
    Returns: final intrinsic LLR for K information bits
    """
    extr_nat = np.zeros(K, dtype=np.float64)
    final_intrinsic = np.zeros(K, dtype=np.float64)

    for half in range(half_iters):
        natural = (half % 2 == 0)
        if natural:
            apr = np.clip(extr_nat, -(1 << (apr_clip - 1)), (1 << (apr_clip - 1)) - 1) \
                  if apr_clip > 0 else extr_nat.copy()
            extr, intrinsic = siso_decode(
                sys1_llr, par1_llr, apr, K, scale, extr_clip, apr_clip
            )
            extr_nat = extr
            final_intrinsic = intrinsic
        else:
            # Interleave the a-priori (extrinsic from natural order)
            apr_ilv = extr_nat[pi]
            if apr_clip > 0:
                lo = -(1 << (apr_clip - 1))
                hi = (1 << (apr_clip - 1)) - 1
                apr_ilv = np.clip(apr_ilv, lo, hi)
            extr_ilv, intrinsic_ilv = siso_decode(
                sys2_llr, par2_llr, apr_ilv, K, scale, extr_clip, apr_clip
            )
            # De-interleave extrinsic back to natural order
            next_extr = np.zeros(K, dtype=np.float64)
            next_extr[pi] = extr_ilv
            extr_nat = next_extr
            # De-interleave intrinsic
            final_intrinsic[pi] = intrinsic_ilv

    return final_intrinsic


# ==========================================================================
# Monte Carlo BER Sweep
# ==========================================================================
def run_one_frame(K, pi, ebn0_db, code_rate, half_iters, scale,
                  quantize, quant_bits, quant_scale,
                  extr_clip, apr_clip, rng):
    """Encode one frame, transmit, decode, return (K, num_errors)."""
    # Random info bits
    info_bits = rng.integers(0, 2, K).astype(np.int32)

    # Encode with proper tail bits
    sys1, par1, sys2, par2 = turbo_encode(info_bits, pi)

    # Channel (each stream independently through AWGN)
    sys1_llr = awgn_channel(sys1, ebn0_db, code_rate, rng)
    par1_llr = awgn_channel(par1, ebn0_db, code_rate, rng)
    # For SISO 2: interleaved systematic + par2
    sys2_ch = np.zeros(K + 3, dtype=np.float64)
    sys2_ch[:K] = sys1_llr[:K][pi]  # reuse channel obs, reordered
    # Tail systematic bits for encoder 2 go through their own channel
    tail2_sys_llr = awgn_channel(sys2[K:], ebn0_db, code_rate, rng)
    sys2_ch[K:] = tail2_sys_llr
    par2_llr = awgn_channel(par2, ebn0_db, code_rate, rng)

    if quantize:
        sys1_llr = quantize_llr(sys1_llr, quant_bits, quant_scale)
        par1_llr = quantize_llr(par1_llr, quant_bits, quant_scale)
        sys2_ch = quantize_llr(sys2_ch, quant_bits, quant_scale)
        par2_llr = quantize_llr(par2_llr, quant_bits, quant_scale)

    # Decode
    intrinsic = turbo_decode(
        sys1_llr, par1_llr, sys2_ch, par2_llr, pi, K,
        half_iters=half_iters, scale=scale,
        extr_clip=extr_clip, apr_clip=apr_clip,
    )

    # BER: hard decision on intrinsic LLR vs true info bits
    hard = (intrinsic < 0).astype(np.int32)
    errors = int(np.count_nonzero(hard != info_bits))
    return K, errors


def monte_carlo_sweep(args):
    K = args.K
    if K not in QPP_TABLE:
        print(f"Error: K={K} not in QPP table. Supported: {sorted(QPP_TABLE.keys())}")
        return

    f1, f2 = QPP_TABLE[K]
    pi = np.array([(f1 * k + f2 * k * k) % K for k in range(K)], dtype=np.int32)
    code_rate = K / (3 * K + 12)

    ebn0_points = np.arange(args.ebn0_start, args.ebn0_stop + 1e-9, args.ebn0_step)

    quant_scale = None
    if args.quant_scale > 0:
        quant_scale = args.quant_scale

    print("=" * 72)
    print("  Monte Carlo BER Simulation — LTE Turbo Decoder")
    print("  Proper 3GPP TS 36.212 trellis termination")
    print("=" * 72)
    print(f"  K={K}, f1={f1}, f2={f2}, R={code_rate:.5f}")
    print(f"  Half-iterations: {args.half_iters}")
    print(f"  Extrinsic scale: {args.scale}")
    print(f"  Quantized: {args.quantize} ({args.quant_bits}-bit)")
    if args.quantize:
        print(f"  Extrinsic clip: {args.extr_clip}-bit, A-priori clip: {args.apr_clip}-bit")
    print(f"  Min errors/point: {args.min_errors}, Max frames/point: {args.max_frames}")
    print(f"  Eb/N0 points: {ebn0_points}")
    print()

    results = []
    rng = np.random.default_rng(args.seed)

    for ebn0 in ebn0_points:
        total_bits = 0
        total_errors = 0
        total_frames = 0
        t0 = time.time()

        while total_errors < args.min_errors and total_frames < args.max_frames:
            bits, errs = run_one_frame(
                K, pi, ebn0, code_rate, args.half_iters, args.scale,
                args.quantize, args.quant_bits, quant_scale,
                args.extr_clip if args.quantize else 0,
                args.apr_clip if args.quantize else 0,
                rng,
            )
            total_bits += bits
            total_errors += errs
            total_frames += 1

            # Progress update every 10 frames
            if total_frames % 10 == 0:
                ber_est = total_errors / total_bits if total_bits > 0 else 0
                elapsed = time.time() - t0
                print(f"\r  Eb/N0={ebn0:5.2f} dB: {total_frames:4d} frames, "
                      f"{total_errors:6d} errors, BER~{ber_est:.2e}, "
                      f"{elapsed:.1f}s", end="", flush=True)

        elapsed = time.time() - t0
        ber = total_errors / total_bits if total_bits > 0 else 0
        results.append((ebn0, ber, total_errors, total_bits, total_frames))
        print(f"\r  Eb/N0={ebn0:5.2f} dB: BER={ber:.6e}  "
              f"({total_errors}/{total_bits}, {total_frames} frames, {elapsed:.1f}s)")

    # Summary table
    print("\n" + "=" * 72)
    print(f"  {'Eb/N0 (dB)':>12s}  {'BER':>12s}  {'Errors':>8s}  {'Bits':>10s}  {'Frames':>7s}")
    print("-" * 72)
    for ebn0, ber, errs, bits, frames in results:
        print(f"  {ebn0:12.2f}  {ber:12.6e}  {errs:8d}  {bits:10d}  {frames:7d}")
    print("=" * 72)

    # Save results
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data')
    os.makedirs(out_dir, exist_ok=True)
    mode_tag = "quantized" if args.quantize else "float"
    out_path = os.path.join(out_dir, f"ber_curve_K{K}_{mode_tag}.csv")
    with open(out_path, "w") as f:
        f.write("ebn0_db,ber,errors,bits,frames\n")
        for ebn0, ber, errs, bits, frames in results:
            f.write(f"{ebn0:.2f},{ber:.10e},{errs},{bits},{frames}\n")
    print(f"\n  Saved: {out_path}")

    return results


# ==========================================================================
# Main
# ==========================================================================
def main():
    p = argparse.ArgumentParser(
        description="Monte Carlo BER for LTE Turbo Decoder (proper tail bits)"
    )
    p.add_argument("--K", type=int, default=3200,
                   help="Code block length (default: 3200)")
    p.add_argument("--half-iters", type=int, default=11,
                   help="Number of half-iterations (default: 11 = 5.5 full)")
    p.add_argument("--scale", type=float, default=0.75,
                   help="Extrinsic scaling factor (default: 0.75; paper hardware=0.6875)")
    p.add_argument("--ebn0-start", type=float, default=0.0)
    p.add_argument("--ebn0-stop", type=float, default=2.5)
    p.add_argument("--ebn0-step", type=float, default=0.5)
    p.add_argument("--min-errors", type=int, default=200,
                   help="Min bit errors per Eb/N0 point (default: 200)")
    p.add_argument("--max-frames", type=int, default=500,
                   help="Max frames per Eb/N0 point (default: 500)")
    p.add_argument("--seed", type=int, default=42)
    # Quantization options
    p.add_argument("--quantize", action="store_true",
                   help="Enable 5-bit LLR quantization (matches RTL)")
    p.add_argument("--quant-bits", type=int, default=5)
    p.add_argument("--quant-scale", type=float, default=-1,
                   help="Quantization scale; -1 = auto (max_val/4)")
    p.add_argument("--extr-clip", type=int, default=6,
                   help="Extrinsic clip bit-width (default: 6)")
    p.add_argument("--apr-clip", type=int, default=5,
                   help="A-priori clip bit-width (default: 5)")
    args = p.parse_args()
    monte_carlo_sweep(args)


if __name__ == "__main__":
    main()
