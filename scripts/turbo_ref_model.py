#!/usr/bin/env python3
"""
Fixed-point reference model for the repository's parallel LTE turbo decoder.

The model mirrors the RTL datapath closely enough for debug:
  - N=2 parallel SISO segments, S=1600 each for the current K=3200 target
  - radix-4 Max-Log M-BCJR with M=30 trellis steps
  - lockstep SISO scheduling with selectable rtl/paper boundary behavior
  - dummy backward recursion for beta window initialization
  - QPP exchange between natural and interleaved half-iterations
  - 5-bit input/a-priori LLRs, 6-bit extrinsics, 10-bit state metrics

It reports BER against data/true_info_bits.txt using the final intrinsic LLR.
The RTL's ld_ram_output.hex contains final extrinsics, so RTL-vs-reference
comparison is done on final extrinsic values, not final hard decisions.
"""

import argparse
import os
import numpy as np

K = 3200
NUM_SISO = 2
S = K // NUM_SISO
TAIL_LEN = 3
WIN_LEN = 30
WIN_LEN_R4 = 15
NUM_WINDOWS = (S + TAIL_LEN + WIN_LEN - 1) // WIN_LEN
NUM_HALF_ITER = 11
BOUNDARY_MODE = "paper"

SM_W = 10
NEG_INF = -256

QPP_TABLE = {
    3200: (111, 240),
    6144: (263, 480),
}

F1 = 111
F2 = 240
PI = np.array([(F1 * k + F2 * k * k) % K for k in range(K)], dtype=np.int32)

R2_PREDS = [
    (0, 1), (2, 3), (4, 5), (6, 7),
    (0, 1), (2, 3), (4, 5), (6, 7),
]

# Must match rtl/bm_radix2.v.
R2_PRE_IDX = np.array(
    [0, 3, 2, 1, 1, 2, 3, 0, 3, 0, 1, 2, 2, 1, 0, 3],
    dtype=np.int32,
)

TRANSITIONS = []
TRANS_BY_PRED = [[] for _ in range(8)]
for dest in range(8):
    for pred_idx, pred in enumerate(R2_PREDS[dest]):
        bm_idx = dest * 2 + pred_idx
        pre_idx = int(R2_PRE_IDX[bm_idx])
        xs = pre_idx >> 1
        item = (pred, dest, bm_idx, xs)
        TRANSITIONS.append(item)
        TRANS_BY_PRED[pred].append(item)


def trunc_signed(val, width):
    half = 1 << (width - 1)
    full = 1 << width
    return ((np.asarray(val, dtype=np.int64) + half) % full - half).astype(np.int32)


def trunc_int(val, width):
    half = 1 << (width - 1)
    full = 1 << width
    return int(((int(val) + half) % full) - half)


def mod_gt(a, b):
    diff = trunc_int(int(a) - int(b), SM_W)
    return diff != 0 and diff > 0


def mod_max(a, b):
    return int(a) if mod_gt(a, b) else int(b)


def mod_max_list(vals):
    vals = list(map(int, vals))
    cur = vals[0]
    for val in vals[1:]:
        cur = mod_max(cur, val)
    return cur


def sat6(val):
    val = int(val)
    if val > 31:
        return 31
    if val < -32:
        return -32
    return val


def sat6_to_5(val):
    val = int(val)
    if val > 15:
        return 15
    if val < -16:
        return -16
    return val


def hard_bit(llr):
    return 1 if int(llr) < 0 else 0


def sign_extend(val, width):
    mask = (1 << width) - 1
    val = int(val) & mask
    sign = 1 << (width - 1)
    return val - (1 << width) if val & sign else val


def load_folded_5(data_dir, even_name, odd_name, tail_len=0):
    out = np.zeros(K + tail_len, dtype=np.int32)
    even_path = os.path.join(data_dir, even_name)
    odd_path = os.path.join(data_dir, odd_name)
    with open(even_path) as f:
        for r, line in enumerate(f):
            word = int(line.strip(), 16)
            local = 2 * r
            if local < S:
                out[local] = sign_extend(word & 0x1F, 5)
            if S + local < K + tail_len:
                out[S + local] = sign_extend((word >> 5) & 0x1F, 5)
    with open(odd_path) as f:
        for r, line in enumerate(f):
            word = int(line.strip(), 16)
            local = 2 * r + 1
            if local < S:
                out[local] = sign_extend(word & 0x1F, 5)
            if S + local < K + tail_len:
                out[S + local] = sign_extend((word >> 5) & 0x1F, 5)
    return out


def load_folded_6_from_ld(path):
    out = np.zeros(K, dtype=np.int32)
    with open(path) as f:
        for r, line in enumerate(f):
            parts = line.strip().split()
            if len(parts) != 2:
                continue
            even = int(parts[0], 16)
            odd = int(parts[1], 16)
            out[2 * r] = sign_extend(even & 0x3F, 6)
            out[2 * r + S] = sign_extend((even >> 6) & 0x3F, 6)
            out[2 * r + 1] = sign_extend(odd & 0x3F, 6)
            out[2 * r + 1 + S] = sign_extend((odd >> 6) & 0x3F, 6)
    return out


def write_folded_6(path, vals):
    with open(path, "w") as f:
        for r in range(S // 2):
            c0e = int(vals[2 * r]) & 0x3F
            c1e = int(vals[2 * r + S]) & 0x3F
            c0o = int(vals[2 * r + 1]) & 0x3F
            c1o = int(vals[2 * r + 1 + S]) & 0x3F
            f.write(f"{((c1e << 6) | c0e):03x} {((c1o << 6) | c0o):03x}\n")


def prep_values(sys_llr, par_llr, apr_llr):
    sa = int(sys_llr) + int(apr_llr)
    return np.array(
        [
            sa + int(par_llr),
            sa - int(par_llr),
            -sa + int(par_llr),
            -sa - int(par_llr),
        ],
        dtype=np.int32,
    )


def compute_radix2(sys_even, sys_odd, par_even, par_odd, apr_even, apr_odd):
    pre_odd = trunc_signed(prep_values(sys_odd, par_odd, apr_odd), 7)
    pre_even = trunc_signed(prep_values(sys_even, par_even, apr_even), 7)
    bm_odd = np.array([pre_odd[i] for i in R2_PRE_IDX], dtype=np.int32)
    bm_even = np.array([pre_even[i] for i in R2_PRE_IDX], dtype=np.int32)
    return np.stack((bm_odd, bm_even))


def compute_radix4(bm_r2):
    bm_r4 = np.zeros(32, dtype=np.int32)
    for i in range(32):
        # Pair order is even address first, then odd address.
        bm_r4[i] = int(bm_r2[1, i % 16]) + int(bm_r2[0, i // 2])
    return trunc_signed(bm_r4, 8)


def acs_forward(sm, bm_r4):
    out = np.zeros(8, dtype=np.int32)
    for dest in range(8):
        preds = (0, 1, 2, 3) if (dest % 2 == 0) else (4, 5, 6, 7)
        cands = [
            trunc_int(int(sm[preds[i]]) + int(bm_r4[dest * 4 + i]), SM_W)
            for i in range(4)
        ]
        out[dest] = mod_max_list(cands)
    return out


def acs_backward(sm, bm_r4):
    out = np.zeros(8, dtype=np.int32)
    for pred in range(8):
        succs = (0, 2, 4, 6) if pred < 4 else (1, 3, 5, 7)
        cands = [
            trunc_int(int(sm[succ]) + int(bm_r4[succ * 4 + (pred % 4)]), SM_W)
            for succ in succs
        ]
        out[pred] = mod_max_list(cands)
    return out


def pair_at(sys_seg, par_seg, apr_seg, addr, trellis_len):
    even_valid = addr < trellis_len and addr < len(sys_seg)
    odd_valid = (addr + 1) < trellis_len and (addr + 1) < len(sys_seg)
    return (
        int(sys_seg[addr]) if even_valid else 0,
        int(sys_seg[addr + 1]) if odd_valid else 0,
        int(par_seg[addr]) if even_valid else 0,
        int(par_seg[addr + 1]) if odd_valid else 0,
        int(apr_seg[addr]) if even_valid else 0,
        int(apr_seg[addr + 1]) if odd_valid else 0,
    )


def bm_for_local_pair(sys_seg, par_seg, apr_seg, addr, trellis_len):
    se, so, pe, po, ae, ao = pair_at(sys_seg, par_seg, apr_seg, addr, trellis_len)
    return compute_radix2(se, so, pe, po, ae, ao)


def forward_pass(sys_seg, par_seg, apr_seg, core_id, trellis_len, boundary_mode="rtl", dummy_prefix=None):
    alpha_mem = np.zeros((NUM_WINDOWS, WIN_LEN_R4, 8), dtype=np.int32)
    gamma_mem = np.zeros((NUM_WINDOWS, WIN_LEN_R4, 2, 16), dtype=np.int32)

    if core_id == 0:
        sm = np.array([0] + [NEG_INF] * 7, dtype=np.int32)
    else:
        # The current two-core RTL keeps both cores cycle-aligned so the
        # folded top-level can write {core1, core0} with one valid/address.
        # Nonzero cores therefore use equal-probability alpha initialization
        # instead of a separate dummy-forward warm-up slot.
        sm = np.zeros(8, dtype=np.int32)
        if boundary_mode == "paper":
            if dummy_prefix is None:
                dummy_prefix = (
                    np.zeros(WIN_LEN, dtype=np.int32),
                    np.zeros(WIN_LEN, dtype=np.int32),
                    np.zeros(WIN_LEN, dtype=np.int32),
                )
            dummy_sys, dummy_par, dummy_apr = dummy_prefix
            for step in range(WIN_LEN_R4):
                addr = 2 * step
                bm_r2 = bm_for_local_pair(dummy_sys, dummy_par, dummy_apr, addr, WIN_LEN)
                sm = acs_forward(sm, compute_radix4(bm_r2))

    for win in range(NUM_WINDOWS):
        base = win * WIN_LEN
        for step in range(WIN_LEN_R4):
            addr = base + 2 * step
            bm_r2 = bm_for_local_pair(sys_seg, par_seg, apr_seg, addr, trellis_len)
            alpha_mem[win, step] = sm
            gamma_mem[win, step] = bm_r2
            sm = acs_forward(sm, compute_radix4(bm_r2))
    return alpha_mem, gamma_mem


def dummy_backward(sys_seg, par_seg, apr_seg, win, trellis_len):
    sm = np.zeros(8, dtype=np.int32)
    base = win * WIN_LEN
    for step in range(WIN_LEN_R4 - 1, -1, -1):
        bm_r2 = bm_for_local_pair(sys_seg, par_seg, apr_seg, base + 2 * step, trellis_len)
        sm = acs_backward(sm, compute_radix4(bm_r2))
    return sm


def derive_alpha_km1(alpha, bm_odd):
    out = np.zeros(8, dtype=np.int32)
    for dest in range(8):
        cands = []
        for pred_idx, pred in enumerate(R2_PREDS[dest]):
            idx = dest * 2 + pred_idx
            cands.append(trunc_int(int(alpha[pred]) + int(bm_odd[idx]), SM_W))
        out[dest] = mod_max(cands[0], cands[1])
    return out


def derive_beta_km1(beta, bm_even):
    out = np.zeros(8, dtype=np.int32)
    for pred in range(8):
        cands = []
        for _, dest, idx, _ in TRANS_BY_PRED[pred]:
            cands.append(trunc_int(int(beta[dest]) + int(bm_even[idx]), SM_W))
        out[pred] = mod_max(cands[0], cands[1])
    return out


def llr_one_r4(alpha_km2, beta_k, bm_r2):
    bm_odd = bm_r2[0]
    bm_even = bm_r2[1]
    alpha_km1 = derive_alpha_km1(alpha_km2, bm_even)
    beta_km1 = derive_beta_km1(beta_k, bm_odd)

    def intrinsic(alpha, beta, bm):
        paths0 = []
        paths1 = []
        for pred, dest, idx, xs in TRANSITIONS:
            metric = trunc_int(int(alpha[pred]) + int(bm[idx]) + int(beta[dest]), SM_W)
            if xs == 0:
                paths0.append(metric)
            else:
                paths1.append(metric)
        return trunc_int(mod_max_list(paths0) - mod_max_list(paths1), SM_W)

    ld_even = intrinsic(alpha_km2, beta_km1, bm_even)
    ld_odd = intrinsic(alpha_km1, beta_k, bm_odd)

    la_odd = (int(bm_odd[0]) + int(bm_odd[3])) >> 1
    la_even = (int(bm_even[0]) + int(bm_even[3])) >> 1

    le_odd = trunc_int(ld_odd - la_odd, SM_W)
    le_even = trunc_int(ld_even - la_even, SM_W)
    scaled_odd = trunc_int(le_odd - (le_odd >> 2) - (le_odd >> 4), SM_W)
    scaled_even = trunc_int(le_even - (le_even >> 2) - (le_even >> 4), SM_W)
    return sat6(scaled_odd), sat6(scaled_even), ld_odd, ld_even


def siso_decode(
    sys_seg,
    par_seg,
    apr_seg,
    core_id,
    trellis_len,
    boundary_mode="rtl",
    dummy_prefix=None,
    dbr_suffix=None,
):
    alpha_mem, gamma_mem = forward_pass(
        sys_seg, par_seg, apr_seg, core_id, trellis_len, boundary_mode, dummy_prefix
    )
    extr = np.zeros(S, dtype=np.int32)
    intrinsic = np.zeros(S, dtype=np.int32)
    if boundary_mode == "paper" and dbr_suffix is not None:
        suffix_sys, suffix_par, suffix_apr = dbr_suffix
        dbr_sys_seg = np.concatenate((sys_seg, suffix_sys))
        dbr_par_seg = np.concatenate((par_seg, suffix_par))
        dbr_apr_seg = np.concatenate((apr_seg, suffix_apr))
        dbr_trellis_len = len(dbr_sys_seg)
    else:
        dbr_sys_seg = sys_seg
        dbr_par_seg = par_seg
        dbr_apr_seg = apr_seg
        dbr_trellis_len = trellis_len

    for win in range(NUM_WINDOWS):
        if core_id == NUM_SISO - 1 and win == NUM_WINDOWS - 1:
            beta = np.array([0] + [NEG_INF] * 7, dtype=np.int32)
        else:
            beta = dummy_backward(dbr_sys_seg, dbr_par_seg, dbr_apr_seg, win + 1, dbr_trellis_len)

        for step in range(WIN_LEN_R4 - 1, -1, -1):
            addr = win * WIN_LEN + 2 * step
            bm_r2 = gamma_mem[win, step]
            le_odd, le_even, ld_odd, ld_even = llr_one_r4(alpha_mem[win, step], beta, bm_r2)
            if addr < S:
                extr[addr] = le_even
                intrinsic[addr] = ld_even
            if addr + 1 < S:
                extr[addr + 1] = le_odd
                intrinsic[addr + 1] = ld_odd
            beta = acs_backward(beta, compute_radix4(bm_r2))

    return extr, intrinsic


def run_turbo(data_dir, num_half_iter=NUM_HALF_ITER, tail_len=0, boundary_mode=BOUNDARY_MODE):
    sys_nat = load_folded_5(data_dir, "sys_even_ram.hex", "sys_odd_ram.hex", tail_len)
    par1_nat = load_folded_5(data_dir, "par1_even_ram.hex", "par1_odd_ram.hex", tail_len)
    sys_ilv = load_folded_5(data_dir, "sys_ilv_even_ram.hex", "sys_ilv_odd_ram.hex", tail_len)
    par2_ilv = load_folded_5(data_dir, "par2_even_ram.hex", "par2_odd_ram.hex", tail_len)

    extr_mem = np.zeros(K, dtype=np.int32)
    final_intrinsic = np.zeros(K, dtype=np.int32)

    def zero_pad(vals, length):
        out = np.zeros(length, dtype=np.int32)
        n = min(length, max(0, len(vals)))
        if n:
            out[:n] = vals[:n]
        return out

    def build_suffix(full_sys, full_par, apr_lookup, start_idx, length):
        sys_suffix = zero_pad(full_sys[start_idx:start_idx + length], length)
        par_suffix = zero_pad(full_par[start_idx:start_idx + length], length)
        apr_suffix = np.zeros(length, dtype=np.int32)
        for i in range(length):
            idx = start_idx + i
            if 0 <= idx < K:
                apr_suffix[i] = sat6_to_5(apr_lookup(idx))
        return sys_suffix, par_suffix, apr_suffix

    def paper_dummy_prefix(full_sys, full_par, apr_lookup, start_idx):
        prefix_start = start_idx - WIN_LEN
        sys_prefix = zero_pad(full_sys[prefix_start:start_idx], WIN_LEN) if prefix_start >= 0 else np.zeros(WIN_LEN, dtype=np.int32)
        par_prefix = zero_pad(full_par[prefix_start:start_idx], WIN_LEN) if prefix_start >= 0 else np.zeros(WIN_LEN, dtype=np.int32)
        apr_prefix = np.zeros(WIN_LEN, dtype=np.int32)
        for i in range(WIN_LEN):
            idx = prefix_start + i
            if 0 <= idx < K:
                apr_prefix[i] = sat6_to_5(apr_lookup(idx))
        return sys_prefix, par_prefix, apr_prefix

    for half in range(num_half_iter):
        next_extr = np.zeros(K, dtype=np.int32)
        natural = (half % 2 == 0)
        for core in range(NUM_SISO):
            lo = core * S
            hi = lo + S
            trellis_len = S + tail_len if core == NUM_SISO - 1 else S
            if natural:
                sys_seg = sys_nat[lo:lo + trellis_len]
                par_seg = par1_nat[lo:lo + trellis_len]
                apr_seg = np.zeros(trellis_len, dtype=np.int32)
                apr_seg[:S] = np.array([sat6_to_5(v) for v in extr_mem[lo:hi]], dtype=np.int32)
                suffix_len = max(0, (NUM_WINDOWS + 1) * WIN_LEN - trellis_len)
                dummy_prefix = paper_dummy_prefix(sys_nat, par1_nat, lambda idx: extr_mem[idx], lo)
                dbr_suffix = build_suffix(
                    sys_nat, par1_nat, lambda idx: extr_mem[idx], lo + trellis_len, suffix_len
                )
                extr, intrinsic = siso_decode(
                    sys_seg, par_seg, apr_seg, core, trellis_len,
                    boundary_mode=boundary_mode,
                    dummy_prefix=dummy_prefix,
                    dbr_suffix=dbr_suffix,
                )
                next_extr[lo:hi] = extr
                if half == num_half_iter - 1:
                    final_intrinsic[lo:hi] = intrinsic
            else:
                ks = np.arange(lo, hi, dtype=np.int32)
                sys_seg = sys_ilv[lo:lo + trellis_len]
                par_seg = par2_ilv[lo:lo + trellis_len]
                apr_seg = np.zeros(trellis_len, dtype=np.int32)
                apr_seg[:S] = np.array([sat6_to_5(extr_mem[PI[k]]) for k in ks], dtype=np.int32)
                suffix_len = max(0, (NUM_WINDOWS + 1) * WIN_LEN - trellis_len)
                dummy_prefix = paper_dummy_prefix(sys_ilv, par2_ilv, lambda idx: extr_mem[PI[idx]], lo)
                dbr_suffix = build_suffix(
                    sys_ilv, par2_ilv, lambda idx: extr_mem[PI[idx]], lo + trellis_len, suffix_len
                )
                extr, intrinsic = siso_decode(
                    sys_seg, par_seg, apr_seg, core, trellis_len,
                    boundary_mode=boundary_mode,
                    dummy_prefix=dummy_prefix,
                    dbr_suffix=dbr_suffix,
                )
                next_extr[PI[ks]] = extr
                if half == num_half_iter - 1:
                    final_intrinsic[PI[ks]] = intrinsic
        extr_mem = next_extr
        print(f"  reference half-iteration {half + 1:2d}/{num_half_iter} complete")

    return sys_nat, extr_mem, final_intrinsic


def load_true_bits(data_dir):
    path = os.path.join(data_dir, "true_info_bits.txt")
    return np.loadtxt(path, dtype=np.int32)


def load_signed_vector(path, expected_len=None):
    if expected_len is None:
        expected_len = K
    vals = np.loadtxt(path, dtype=np.int32)
    if vals.size != expected_len:
        raise ValueError(f"{path} has {vals.size} values, expected {expected_len}")
    return vals


def ber_from_llr(llr, bits):
    decisions = np.array([hard_bit(v) for v in llr], dtype=np.int32)
    errors = int(np.count_nonzero(decisions != bits))
    return errors, errors / len(bits)


def main():
    global K, NUM_SISO, S, TAIL_LEN, NUM_WINDOWS, BOUNDARY_MODE, F1, F2, PI

    parser = argparse.ArgumentParser()
    parser.add_argument("--K", type=int, default=K)
    parser.add_argument("--num-siso", type=int, default=NUM_SISO)
    parser.add_argument("--data-dir", default=os.path.join(os.path.dirname(__file__), "..", "data"))
    parser.add_argument("--half-iters", type=int, default=NUM_HALF_ITER)
    parser.add_argument("--tail-len", type=int, default=TAIL_LEN, help="tail trellis steps appended to the last SISO core")
    parser.add_argument(
        "--boundary-mode",
        choices=["rtl", "paper"],
        default=BOUNDARY_MODE,
        help="rtl keeps the legacy equal-alpha/local-DBR approximation; paper enables dummy-forward and cross-segment DBR",
    )
    parser.add_argument(
        "--show-mismatches",
        type=int,
        default=0,
        help="print the first N RTL/reference extrinsic mismatches",
    )
    args = parser.parse_args()
    if args.num_siso != 2:
        raise SystemExit("turbo_ref_model.py currently mirrors the folded N=2 RTL architecture; use --num-siso 2.")
    if args.K not in QPP_TABLE:
        raise SystemExit(f"K={args.K} not supported; known QPP entries: {sorted(QPP_TABLE)}")

    K = args.K
    NUM_SISO = args.num_siso
    S = K // NUM_SISO
    F1, F2 = QPP_TABLE[K]
    PI = np.array([(F1 * k + F2 * k * k) % K for k in range(K)], dtype=np.int32)
    TAIL_LEN = args.tail_len
    NUM_WINDOWS = (S + TAIL_LEN + WIN_LEN - 1) // WIN_LEN
    BOUNDARY_MODE = args.boundary_mode
    data_dir = os.path.abspath(args.data_dir)

    print("=" * 72)
    print("Fixed-point parallel turbo reference model")
    print("=" * 72)
    print(f"Data directory: {data_dir}")

    sys_nat, ref_extr, ref_intrinsic = run_turbo(
        data_dir, args.half_iters, args.tail_len, args.boundary_mode
    )
    true_bits = load_true_bits(data_dir)
    report_lines = [
        "LTE Parallel Turbo Decoder BER Results",
        "=" * 40,
        f"Data directory: {data_dir}",
        f"K: {K}",
        f"NUM_SISO: {NUM_SISO}",
        f"Segment length: {S}",
        f"QPP f1/f2: {F1}/{F2}",
        f"Tail trellis steps: {args.tail_len}",
        f"Boundary mode: {args.boundary_mode}",
        f"Half iterations: {args.half_iters}",
        "",
        "BER is computed as original transmitted bits versus hard decisions",
        "from the final intrinsic decision LLR L_D after the last half-iteration.",
        "",
    ]

    ch_err, ch_ber = ber_from_llr(sys_nat[:K], true_bits)
    ref_err, ref_ber = ber_from_llr(ref_intrinsic, true_bits)
    print("\nBER")
    print(f"  channel hard decisions: {ch_err:5d}/{K} = {ch_ber:.6f}")
    print(f"  reference final LD:     {ref_err:5d}/{K} = {ref_ber:.6f}")
    report_lines.extend([
        "Channel and Reference",
        f"  Channel hard decisions: {ch_err}/{K} = {ch_ber:.6f}",
        f"  Python reference final L_D: {ref_err}/{K} = {ref_ber:.6f}",
        "",
    ])

    write_folded_6(os.path.join(data_dir, "ref_final_extrinsic.hex"), ref_extr)
    np.savetxt(os.path.join(data_dir, "ref_final_intrinsic.txt"), ref_intrinsic, fmt="%d")
    ref_hard = np.array([hard_bit(v) for v in ref_intrinsic], dtype=np.int32)
    np.savetxt(os.path.join(data_dir, "ref_final_hard_bits.txt"), ref_hard, fmt="%d")

    rtl_intr_path = os.path.join(data_dir, "rtl_final_intrinsic.txt")
    if os.path.exists(rtl_intr_path):
        rtl_intr = load_signed_vector(rtl_intr_path)
        rtl_err, rtl_ber = ber_from_llr(rtl_intr, true_bits)
        rtl_hard = np.array([hard_bit(v) for v in rtl_intr], dtype=np.int32)
        np.savetxt(os.path.join(data_dir, "rtl_final_hard_bits.txt"), rtl_hard, fmt="%d")

        intr_mism_idx = np.nonzero(rtl_intr != ref_intrinsic)[0]
        hard_mism = int(np.count_nonzero(rtl_hard != ref_hard))
        max_intr_diff = int(np.max(np.abs(rtl_intr - ref_intrinsic))) if len(rtl_intr) else 0

        print("\nRTL BER from final intrinsic L_D")
        print(f"  RTL final LD:           {rtl_err:5d}/{K} = {rtl_ber:.6f}")
        print(f"  intrinsic mismatches:   {len(intr_mism_idx):5d}/{K}")
        print(f"  hard-bit mismatches:    {hard_mism:5d}/{K}")
        print(f"  max |difference|:       {max_intr_diff}")
        report_lines.extend([
            "RTL Final Intrinsic BER",
            f"  RTL final L_D: {rtl_err}/{K} = {rtl_ber:.6f}",
            f"  RTL-vs-reference intrinsic mismatches: {len(intr_mism_idx)}/{K}",
            f"  RTL-vs-reference hard-bit mismatches: {hard_mism}/{K}",
            f"  RTL-vs-reference max |difference|: {max_intr_diff}",
            "",
            "RTL output files",
            "  rtl_final_intrinsic.txt: signed final L_D values from RTL simulation",
            "  rtl_final_hard_bits.txt: hard decisions from RTL final L_D",
            "",
        ])
    else:
        print("\nRTL BER skipped: rtl_final_intrinsic.txt not found")
        report_lines.extend([
            "RTL Final Intrinsic BER",
            "  Skipped: rtl_final_intrinsic.txt not found. Run tb_turbo_decoder first.",
            "",
        ])

    rtl_path = os.path.join(data_dir, "ld_ram_output.hex")
    if os.path.exists(rtl_path):
        rtl_extr = load_folded_6_from_ld(rtl_path)
        mism_idx = np.nonzero(rtl_extr != ref_extr)[0]
        mism = int(len(mism_idx))
        maxdiff = int(np.max(np.abs(rtl_extr - ref_extr))) if len(rtl_extr) else 0
        rtl_sign_err = int(np.count_nonzero(
            np.array([hard_bit(v) for v in rtl_extr]) != np.array([hard_bit(v) for v in ref_extr])
        ))
        print("\nRTL final-extrinsic comparison")
        print(f"  value mismatches:       {mism:5d}/{K}")
        print(f"  sign mismatches:        {rtl_sign_err:5d}/{K}")
        print(f"  max |difference|:       {maxdiff}")
        report_lines.extend([
            "RTL Final Extrinsic Comparison",
            f"  Value mismatches: {mism}/{K}",
            f"  Sign mismatches: {rtl_sign_err}/{K}",
            f"  Max |difference|: {maxdiff}",
            "",
        ])
        for i in mism_idx[:args.show_mismatches]:
            print(
                f"    i={int(i):4d} core={int(i // S)} local={int(i % S):4d} "
                f"ref={int(ref_extr[i]):4d} rtl={int(rtl_extr[i]):4d}"
            )
    else:
        print("\nRTL final-extrinsic comparison skipped: ld_ram_output.hex not found")
        report_lines.extend([
            "RTL Final Extrinsic Comparison",
            "  Skipped: ld_ram_output.hex not found.",
            "",
        ])

    report_path = os.path.join(data_dir, "ber_results.txt")
    with open(report_path, "w") as f:
        f.write("\n".join(report_lines))
        f.write("\n")
    print(f"\nWrote BER report: {report_path}")


if __name__ == "__main__":
    main()
