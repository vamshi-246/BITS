#!/usr/bin/env python3
"""
Floating-point full-block turbo decoder for algorithm benchmarking.

This model intentionally removes the RTL architecture approximations:
  - no SISO segmentation
  - no M=30 windowing
  - no state-metric quantization
  - no extrinsic saturation

It reads the current repository BRAM LLR files, so the first comparison uses
the exact same received/quantized channel values that were fed to the RTL.
BER is computed against data/true_info_bits.txt.
"""

import argparse
import os
import time

import numpy as np

K = 3200
F1 = 111
F2 = 240
NUM_STATES = 8
NUM_HALF_ITER = 11
NEG_INF = -1.0e100

PI = np.array([(F1 * k + F2 * k * k) % K for k in range(K)], dtype=np.int32)

R2_PREDS = [
    (0, 1), (2, 3), (4, 5), (6, 7),
    (0, 1), (2, 3), (4, 5), (6, 7),
]

# Same LTE trellis branch-label mapping as rtl/bm_radix2.v.
R2_PRE_IDX = np.array(
    [0, 3, 2, 1, 1, 2, 3, 0, 3, 0, 1, 2, 2, 1, 0, 3],
    dtype=np.int32,
)

TRANSITIONS = []
TRANS_BY_PRED = [[] for _ in range(NUM_STATES)]
for dest in range(NUM_STATES):
    for pred_idx, pred in enumerate(R2_PREDS[dest]):
        bm_idx = dest * 2 + pred_idx
        pre_idx = int(R2_PRE_IDX[bm_idx])
        xs = pre_idx >> 1
        item = (pred, dest, bm_idx, xs)
        TRANSITIONS.append(item)
        TRANS_BY_PRED[pred].append(item)


def sign_extend(val, width):
    val = int(val) & ((1 << width) - 1)
    sign = 1 << (width - 1)
    return val - (1 << width) if val & sign else val


def hard_bit(llr):
    return 1 if float(llr) < 0.0 else 0


def clip_signed(vals, bits):
    if bits <= 0:
        return vals
    lo = -(1 << (bits - 1))
    hi = (1 << (bits - 1)) - 1
    return np.clip(vals, lo, hi)


def load_folded_5(data_dir, even_name, odd_name):
    s = K // 2
    out = np.zeros(K, dtype=np.float64)

    def store_pair(local, word):
        if local < s:
            out[local] = sign_extend(word & 0x1F, 5)
            out[local + s] = sign_extend((word >> 5) & 0x1F, 5)

    with open(os.path.join(data_dir, even_name)) as f:
        for r, line in enumerate(f):
            word = int(line.strip(), 16)
            store_pair(2 * r, word)
    with open(os.path.join(data_dir, odd_name)) as f:
        for r, line in enumerate(f):
            word = int(line.strip(), 16)
            store_pair(2 * r + 1, word)
    return out


def load_inputs(data_dir):
    sys_nat = load_folded_5(data_dir, "sys_even_ram.hex", "sys_odd_ram.hex")
    par1_nat = load_folded_5(data_dir, "par1_even_ram.hex", "par1_odd_ram.hex")
    sys_ilv = load_folded_5(data_dir, "sys_ilv_even_ram.hex", "sys_ilv_odd_ram.hex")
    par2_ilv = load_folded_5(data_dir, "par2_even_ram.hex", "par2_odd_ram.hex")
    true_bits = np.loadtxt(os.path.join(data_dir, "true_info_bits.txt"), dtype=np.int32)
    if true_bits.size != K:
        raise ValueError(f"true_info_bits.txt has {true_bits.size} bits, expected {K}")
    return sys_nat, par1_nat, sys_ilv, par2_ilv, true_bits


def combine(vals, mode):
    vals = np.asarray(vals, dtype=np.float64)
    if mode == "maxlog":
        return float(np.max(vals))
    vmax = float(np.max(vals))
    if vmax <= NEG_INF / 2:
        return vmax
    return float(vmax + np.log(np.sum(np.exp(vals - vmax))))


def branch_metrics(sys_llr, par_llr, apr_llr):
    sa = sys_llr + apr_llr
    pre = np.empty((K, 4), dtype=np.float64)
    pre[:, 0] = sa + par_llr
    pre[:, 1] = sa - par_llr
    pre[:, 2] = -sa + par_llr
    pre[:, 3] = -sa - par_llr

    gamma = np.empty((K, NUM_STATES, 2), dtype=np.float64)
    for dest in range(NUM_STATES):
        gamma[:, dest, 0] = pre[:, R2_PRE_IDX[2 * dest]]
        gamma[:, dest, 1] = pre[:, R2_PRE_IDX[2 * dest + 1]]
    return gamma


def normalize(row):
    vmax = float(np.max(row))
    if vmax > NEG_INF / 2:
        row -= vmax
    return row


def siso_decode(sys_llr, par_llr, apr_llr, mode, scale, extrinsic_clip_bits):
    gamma = branch_metrics(sys_llr, par_llr, apr_llr)

    alpha = np.full((K + 1, NUM_STATES), NEG_INF, dtype=np.float64)
    alpha[0, 0] = 0.0
    for k in range(K):
        for dest in range(NUM_STATES):
            preds = R2_PREDS[dest]
            alpha[k + 1, dest] = combine(
                [
                    alpha[k, preds[0]] + gamma[k, dest, 0],
                    alpha[k, preds[1]] + gamma[k, dest, 1],
                ],
                mode,
            )
        normalize(alpha[k + 1])

    beta = np.full((K + 1, NUM_STATES), NEG_INF, dtype=np.float64)
    beta[K, 0] = 0.0
    for k in range(K - 1, -1, -1):
        for pred in range(NUM_STATES):
            beta[k, pred] = combine(
                [
                    beta[k + 1, dest] + gamma[k, dest, bm_idx & 1]
                    for _, dest, bm_idx, _ in TRANS_BY_PRED[pred]
                ],
                mode,
            )
        normalize(beta[k])

    intrinsic = np.zeros(K, dtype=np.float64)
    extrinsic = np.zeros(K, dtype=np.float64)
    for k in range(K):
        paths0 = []
        paths1 = []
        for pred, dest, bm_idx, xs in TRANSITIONS:
            pred_idx = bm_idx & 1
            metric = alpha[k, pred] + gamma[k, dest, pred_idx] + beta[k + 1, dest]
            if xs == 0:
                paths0.append(metric)
            else:
                paths1.append(metric)
        intrinsic[k] = combine(paths0, mode) - combine(paths1, mode)
        extrinsic[k] = scale * (intrinsic[k] - sys_llr[k] - apr_llr[k])

    extrinsic = clip_signed(extrinsic, extrinsic_clip_bits)
    return extrinsic, intrinsic


def ber_from_llr(llr, true_bits):
    hard = np.array([hard_bit(v) for v in llr], dtype=np.int32)
    errors = int(np.count_nonzero(hard != true_bits))
    return hard, errors, errors / len(true_bits)


def run_turbo(data_dir, half_iters, mode, scale, extrinsic_clip_bits, apr_clip_bits):
    sys_nat, par1_nat, sys_ilv, par2_ilv, true_bits = load_inputs(data_dir)
    extr_nat = np.zeros(K, dtype=np.float64)
    final_intrinsic_nat = np.zeros(K, dtype=np.float64)

    for half in range(half_iters):
        natural = (half % 2 == 0)
        t0 = time.time()
        if natural:
            apr = clip_signed(extr_nat, apr_clip_bits)
            extr, intrinsic = siso_decode(
                sys_nat, par1_nat, apr, mode, scale, extrinsic_clip_bits
            )
            extr_nat = extr
            final_intrinsic_nat = intrinsic
        else:
            apr_ilv = clip_signed(extr_nat[PI], apr_clip_bits)
            extr_ilv, intrinsic_ilv = siso_decode(
                sys_ilv, par2_ilv, apr_ilv, mode, scale, extrinsic_clip_bits
            )
            next_extr = np.zeros(K, dtype=np.float64)
            next_extr[PI] = extr_ilv
            extr_nat = next_extr
            final_intrinsic_nat[PI] = intrinsic_ilv
        _, err, ber = ber_from_llr(final_intrinsic_nat, true_bits)
        print(
            f"  half {half + 1:2d}/{half_iters} "
            f"({'natural' if natural else 'interleaved'}): "
            f"BER {err:5d}/{K} = {ber:.6f}, {time.time() - t0:.2f}s"
        )

    return sys_nat, extr_nat, final_intrinsic_nat, true_bits


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", default=os.path.join(os.path.dirname(__file__), "..", "data"))
    parser.add_argument("--half-iters", type=int, default=NUM_HALF_ITER)
    parser.add_argument("--mode", choices=["maxlog", "logmap"], default="maxlog")
    parser.add_argument("--scale", type=float, default=0.6875)
    parser.add_argument(
        "--extrinsic-clip-bits",
        type=int,
        default=0,
        help="clip exchanged extrinsics to this signed bit width; 0 disables clipping",
    )
    parser.add_argument(
        "--apr-clip-bits",
        type=int,
        default=0,
        help="clip a-priori inputs to this signed bit width; 0 disables clipping",
    )
    args = parser.parse_args()

    data_dir = os.path.abspath(args.data_dir)
    print("=" * 72)
    print("Floating-point full-block turbo decoder")
    print("=" * 72)
    print(f"Data directory: {data_dir}")
    print(f"K={K}, half-iters={args.half_iters}, mode={args.mode}, scale={args.scale}")
    print(f"extrinsic_clip_bits={args.extrinsic_clip_bits}, apr_clip_bits={args.apr_clip_bits}")
    print("Input source: current 5-bit quantized BRAM LLR files")

    sys_nat, extr, intrinsic, true_bits = run_turbo(
        data_dir=data_dir,
        half_iters=args.half_iters,
        mode=args.mode,
        scale=args.scale,
        extrinsic_clip_bits=args.extrinsic_clip_bits,
        apr_clip_bits=args.apr_clip_bits,
    )

    ch_hard, ch_err, ch_ber = ber_from_llr(sys_nat, true_bits)
    final_hard, final_err, final_ber = ber_from_llr(intrinsic, true_bits)

    clip_suffix = ""
    if args.extrinsic_clip_bits:
        clip_suffix += f"_extr{args.extrinsic_clip_bits}"
    if args.apr_clip_bits:
        clip_suffix += f"_apr{args.apr_clip_bits}"
    prefix = f"float_full_block_{args.mode}{clip_suffix}"
    np.savetxt(os.path.join(data_dir, f"{prefix}_intrinsic.txt"), intrinsic, fmt="%.9f")
    np.savetxt(os.path.join(data_dir, f"{prefix}_extrinsic.txt"), extr, fmt="%.9f")
    np.savetxt(os.path.join(data_dir, f"{prefix}_hard_bits.txt"), final_hard, fmt="%d")

    report_path = os.path.join(data_dir, f"{prefix}_results.txt")
    lines = [
        "Floating-Point Full-Block Turbo Decoder Results",
        "=" * 49,
        f"Data directory: {data_dir}",
        f"K: {K}",
        f"Half iterations: {args.half_iters}",
        f"Mode: {args.mode}",
        f"Extrinsic scale: {args.scale}",
        f"Extrinsic clip bits: {args.extrinsic_clip_bits}",
        f"A-priori clip bits: {args.apr_clip_bits}",
        "Input source: 5-bit quantized BRAM LLR files",
        "",
        "BER definition: original transmitted bit versus hard_decision(final intrinsic L_D)",
        "Hard decision: L_D < 0 -> bit 1, else bit 0",
        "",
        f"Channel hard decisions: {ch_err}/{K} = {ch_ber:.6f}",
        f"Floating full-block final L_D: {final_err}/{K} = {final_ber:.6f}",
        "",
        "Output files:",
        f"  {prefix}_intrinsic.txt",
        f"  {prefix}_extrinsic.txt",
        f"  {prefix}_hard_bits.txt",
    ]
    with open(report_path, "w") as f:
        f.write("\n".join(lines))
        f.write("\n")

    print("\nBER")
    print(f"  channel hard decisions:      {ch_err:5d}/{K} = {ch_ber:.6f}")
    print(f"  floating full-block final LD:{final_err:5d}/{K} = {final_ber:.6f}")
    print(f"\nWrote report: {report_path}")


if __name__ == "__main__":
    main()
