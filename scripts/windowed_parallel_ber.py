#!/usr/bin/env python3
"""
Monte Carlo BER for a windowed parallel LTE turbo decoder model.

This script is the Python-side algorithm model to use before translating
changes into RTL.  It keeps the scheduling assumptions explicit:

  - QPP LTE interleaver from 3GPP TS 36.212.
  - Max-Log-MAP SISO recursion.
  - M-BCJR windowing with dummy backward recursion.
  - Optional N-way SISO segmentation.
  - 5-bit channel/apriori LLRs, 6-bit extrinsics, 10-bit state metrics.

The default mode models the paper-style M-BCJR boundary treatment:
nonzero SISO cores use one dummy forward window before their segment, and
dummy backward windows may read across the next segment boundary.  Use
``--boundary-mode rtl`` to model the current lockstep RTL approximation
where nonzero cores start from equal alpha metrics and DBR beyond the local
segment is zero-padded.
"""

import argparse
import csv
import math
import os
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, replace

import numpy as np


# 3GPP TS 36.212 Table 5.1.3-3 entries used in this repository/paper.
QPP_TABLE = {
    40: (3, 10),
    48: (7, 12),
    64: (7, 16),
    128: (15, 32),
    256: (15, 64),
    512: (31, 64),
    1024: (17, 66),
    2048: (33, 130),
    3200: (111, 240),
    4096: (299, 256),
    6144: (263, 480),
}

NUM_STATES = 8
NEG_INF_FLOAT = -1.0e30
NEG_INF_FIXED = -256
SM_W = 10

R2_PREDS = [
    (0, 1), (2, 3), (4, 5), (6, 7),
    (0, 1), (2, 3), (4, 5), (6, 7),
]

R2_PRE_IDX = np.array(
    [0, 3, 2, 1, 1, 2, 3, 0, 3, 0, 1, 2, 2, 1, 0, 3],
    dtype=np.int32,
)

TRANSITIONS = []
TRANS_BY_PRED = [[] for _ in range(NUM_STATES)]
for _dest in range(NUM_STATES):
    for _pred_idx, _pred in enumerate(R2_PREDS[_dest]):
        _bm_idx = _dest * 2 + _pred_idx
        _pre_idx = int(R2_PRE_IDX[_bm_idx])
        _xs = _pre_idx >> 1
        _item = (_pred, _dest, _bm_idx, _xs)
        TRANSITIONS.append(_item)
        TRANS_BY_PRED[_pred].append(_item)


@dataclass(frozen=True)
class DecoderConfig:
    k: int
    pi: np.ndarray
    num_siso: int
    window_size: int
    half_iters: int
    scale: float
    quantized: bool
    boundary_mode: str
    use_tail: bool
    channel_rate: float

    @property
    def segment_len(self):
        return self.k // self.num_siso


def make_qpp(k):
    if k not in QPP_TABLE:
        raise ValueError(f"K={k} is not in QPP_TABLE; supported: {sorted(QPP_TABLE)}")
    f1, f2 = QPP_TABLE[k]
    return np.array([(f1 * i + f2 * i * i) % k for i in range(k)], dtype=np.int32)


def lte_rsc_encode(info_bits):
    """LTE rate-1/2 RSC constituent encoder with 3 tail trellis steps."""
    k = len(info_bits)
    sys_out = np.zeros(k + 3, dtype=np.int32)
    par_out = np.zeros(k + 3, dtype=np.int32)
    sys_out[:k] = info_bits
    s = [0, 0, 0]

    for i in range(k):
        feedback = int(info_bits[i]) ^ s[1] ^ s[2]
        par_out[i] = feedback ^ s[0] ^ s[2]
        s[2] = s[1]
        s[1] = s[0]
        s[0] = feedback

    for i in range(3):
        tail_in = s[1] ^ s[2]
        sys_out[k + i] = tail_in
        feedback = tail_in ^ s[1] ^ s[2]
        par_out[k + i] = feedback ^ s[0] ^ s[2]
        s[2] = s[1]
        s[1] = s[0]
        s[0] = feedback

    if s != [0, 0, 0]:
        raise RuntimeError("RSC encoder did not terminate to zero state")
    return sys_out, par_out


def turbo_encode(info_bits, pi):
    interleaved = np.array([info_bits[pi[i]] for i in range(len(info_bits))], dtype=np.int32)
    sys1, par1 = lte_rsc_encode(info_bits)
    sys2, par2 = lte_rsc_encode(interleaved)
    return sys1, par1, sys2, par2


def awgn_channel(bits, ebn0_db, code_rate, rng):
    x = 1.0 - 2.0 * bits.astype(np.float64)
    ebn0 = 10.0 ** (ebn0_db / 10.0)
    sigma2 = 1.0 / (2.0 * code_rate * ebn0)
    y = x + np.sqrt(sigma2) * rng.standard_normal(len(bits))
    return 2.0 * y / sigma2


def quantize_llr(llr, n_bits=5, scale_factor=None):
    max_val = (1 << (n_bits - 1)) - 1
    min_val = -(1 << (n_bits - 1))
    if scale_factor is None:
        scale_factor = max_val / 4.0
    q = np.round(llr * scale_factor)
    return np.clip(q, min_val, max_val).astype(np.float64)


def clip_signed(vals, bits):
    if bits <= 0:
        return vals
    lo = -(1 << (bits - 1))
    hi = (1 << (bits - 1)) - 1
    return np.clip(vals, lo, hi)


def trunc_int(val, width):
    half = 1 << (width - 1)
    full = 1 << width
    return int(((int(val) + half) % full) - half)


def trunc_signed(val, width):
    half = 1 << (width - 1)
    full = 1 << width
    return ((np.asarray(val, dtype=np.int64) + half) % full - half).astype(np.int32)


def mod_gt(a, b):
    diff = trunc_int(int(a) - int(b), SM_W)
    return diff != 0 and diff > 0


def mod_max(a, b):
    return int(a) if mod_gt(a, b) else int(b)


def max_list(vals, quantized):
    if not quantized:
        return float(np.max(vals))
    vals = list(map(int, vals))
    cur = vals[0]
    for val in vals[1:]:
        cur = mod_max(cur, val)
    return cur


def mod_max_list(vals):
    vals = list(map(int, vals))
    cur = vals[0]
    for val in vals[1:]:
        cur = mod_max(cur, val)
    return cur


def fixed_add(*vals):
    total = 0
    for val in vals:
        total += int(val)
    return trunc_int(total, SM_W)


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


def branch_metrics_one(sys_llr, par_llr, apr_llr, quantized):
    sa = float(sys_llr) + float(apr_llr)
    pre = np.array(
        [sa + par_llr, sa - par_llr, -sa + par_llr, -sa - par_llr],
        dtype=np.float64,
    )
    if quantized:
        pre = (((pre.astype(np.int64) + 64) % 128) - 64).astype(np.float64)

    gamma = np.zeros((NUM_STATES, 2), dtype=np.float64)
    for dest in range(NUM_STATES):
        gamma[dest, 0] = pre[R2_PRE_IDX[2 * dest]]
        gamma[dest, 1] = pre[R2_PRE_IDX[2 * dest + 1]]
    return gamma


def normalize(row, quantized):
    if quantized:
        return row
    vmax = float(np.max(row))
    if vmax > NEG_INF_FLOAT / 2:
        row -= vmax
    return row


def forward_step(alpha, gamma, quantized):
    out = np.zeros(NUM_STATES, dtype=np.float64)
    for dest in range(NUM_STATES):
        p0, p1 = R2_PREDS[dest]
        if quantized:
            vals = [fixed_add(alpha[p0], gamma[dest, 0]), fixed_add(alpha[p1], gamma[dest, 1])]
        else:
            vals = [alpha[p0] + gamma[dest, 0], alpha[p1] + gamma[dest, 1]]
        out[dest] = max_list(vals, quantized)
    return normalize(out, quantized)


def backward_step(beta_next, gamma, quantized):
    out = np.zeros(NUM_STATES, dtype=np.float64)
    for pred in range(NUM_STATES):
        vals = []
        for _, dest, bm_idx, _ in TRANS_BY_PRED[pred]:
            pred_idx = bm_idx & 1
            if quantized:
                vals.append(fixed_add(beta_next[dest], gamma[dest, pred_idx]))
            else:
                vals.append(beta_next[dest] + gamma[dest, pred_idx])
        out[pred] = max_list(vals, quantized)
    return normalize(out, quantized)


def metric_add(a, b, c, quantized):
    if quantized:
        return fixed_add(a, b, c)
    return float(a + b + c)


def llr_from_alpha_beta(alpha, beta_next, gamma, sys_llr, apr_llr, cfg):
    paths0 = []
    paths1 = []
    for pred, dest, bm_idx, xs in TRANSITIONS:
        pred_idx = bm_idx & 1
        metric = metric_add(alpha[pred], gamma[dest, pred_idx], beta_next[dest], cfg.quantized)
        if xs == 0:
            paths0.append(metric)
        else:
            paths1.append(metric)

    ld = max_list(paths0, cfg.quantized) - max_list(paths1, cfg.quantized)
    if cfg.quantized:
        ld = trunc_int(ld, SM_W)
        le = trunc_int(ld - int(sys_llr) - int(apr_llr), SM_W)
        scaled = trunc_int(le - (le >> 2) - (le >> 4), SM_W)
        return sat6(scaled), ld

    le = cfg.scale * (float(ld) - float(sys_llr) - float(apr_llr))
    return le, float(ld)


def gamma_at(sys_full, par_full, apr_full, idx, quantized):
    if idx < 0 or idx >= len(sys_full):
        return branch_metrics_one(0.0, 0.0, 0.0, quantized)
    return branch_metrics_one(sys_full[idx], par_full[idx], apr_full[idx], quantized)


def initial_alpha(sys_full, par_full, apr_full, start, core_id, cfg):
    if core_id == 0:
        if cfg.quantized:
            return np.array([0] + [NEG_INF_FIXED] * 7, dtype=np.float64)
        return np.array([0.0] + [NEG_INF_FLOAT] * 7, dtype=np.float64)

    alpha = np.zeros(NUM_STATES, dtype=np.float64)
    if cfg.boundary_mode == "rtl":
        return alpha

    dummy_start = start - cfg.window_size
    for idx in range(dummy_start, start):
        alpha = forward_step(alpha, gamma_at(sys_full, par_full, apr_full, idx, cfg.quantized), cfg.quantized)
    return alpha


def dummy_backward(sys_full, par_full, apr_full, start, cfg):
    beta = np.zeros(NUM_STATES, dtype=np.float64)
    for idx in range(start + cfg.window_size - 1, start - 1, -1):
        beta = backward_step(beta, gamma_at(sys_full, par_full, apr_full, idx, cfg.quantized), cfg.quantized)
    return beta


def siso_windowed_decode(sys_full, par_full, apr_full, start, info_len, tail_len, core_id, cfg):
    total_len = info_len + tail_len
    num_windows = math.ceil(total_len / cfg.window_size)
    alpha = initial_alpha(sys_full, par_full, apr_full, start, core_id, cfg)

    alpha_mem = []
    gamma_mem = []
    for w in range(num_windows):
        win_alpha = []
        win_gamma = []
        for off in range(cfg.window_size):
            idx = start + w * cfg.window_size + off
            gamma = gamma_at(sys_full, par_full, apr_full, idx, cfg.quantized)
            win_alpha.append(alpha.copy())
            win_gamma.append(gamma)
            alpha = forward_step(alpha, gamma, cfg.quantized)
        alpha_mem.append(win_alpha)
        gamma_mem.append(win_gamma)

    extr = np.zeros(info_len, dtype=np.float64)
    intrinsic = np.zeros(info_len, dtype=np.float64)
    last_core = core_id == cfg.num_siso - 1

    for w in range(num_windows):
        win_valid_start = w * cfg.window_size
        win_valid_end = min((w + 1) * cfg.window_size, total_len)
        is_terminal_window = last_core and (w == num_windows - 1)

        if is_terminal_window:
            if cfg.quantized:
                beta = np.array([0] + [NEG_INF_FIXED] * 7, dtype=np.float64)
            else:
                beta = np.array([0.0] + [NEG_INF_FLOAT] * 7, dtype=np.float64)
        else:
            if cfg.boundary_mode == "rtl":
                next_start = start + (w + 1) * cfg.window_size
                if next_start >= start + total_len:
                    beta = np.zeros(NUM_STATES, dtype=np.float64)
                else:
                    beta = dummy_backward(sys_full, par_full, apr_full, next_start, cfg)
            else:
                beta = dummy_backward(sys_full, par_full, apr_full, start + (w + 1) * cfg.window_size, cfg)

        for local in range((w + 1) * cfg.window_size - 1, win_valid_start - 1, -1):
            off = local - w * cfg.window_size
            global_idx = start + local
            gamma = gamma_mem[w][off]
            if local < info_len:
                le, ld = llr_from_alpha_beta(
                    alpha_mem[w][off], beta, gamma,
                    sys_full[global_idx], apr_full[global_idx], cfg,
                )
                extr[local] = le
                intrinsic[local] = ld
            beta = backward_step(beta, gamma, cfg.quantized)

    return extr, intrinsic


def full_block_siso(sys_llr, par_llr, apr_llr, info_len, cfg):
    cfg_float = replace(cfg, quantized=False)
    total_len = info_len + (3 if cfg.use_tail else 0)
    alpha = np.full((total_len + 1, NUM_STATES), NEG_INF_FLOAT, dtype=np.float64)
    alpha[0, 0] = 0.0
    gamma = [branch_metrics_one(sys_llr[i], par_llr[i], apr_llr[i], False) for i in range(total_len)]

    for i in range(total_len):
        alpha[i + 1] = forward_step(alpha[i], gamma[i], False)

    beta = np.full((total_len + 1, NUM_STATES), NEG_INF_FLOAT, dtype=np.float64)
    beta[total_len, 0] = 0.0
    for i in range(total_len - 1, -1, -1):
        beta[i] = backward_step(beta[i + 1], gamma[i], False)

    extr = np.zeros(info_len, dtype=np.float64)
    intrinsic = np.zeros(info_len, dtype=np.float64)
    for i in range(info_len):
        le, ld = llr_from_alpha_beta(alpha[i], beta[i + 1], gamma[i], sys_llr[i], apr_llr[i], cfg_float)
        extr[i] = le
        intrinsic[i] = ld
    return extr, intrinsic


def turbo_decode_full(sys1, par1, sys2, par2, cfg):
    k = cfg.k
    extr_nat = np.zeros(k, dtype=np.float64)
    final_intrinsic = np.zeros(k, dtype=np.float64)
    tail = 3 if cfg.use_tail else 0

    for half in range(cfg.half_iters):
        if half % 2 == 0:
            apr = clip_signed(extr_nat, 5 if cfg.quantized else 0)
            apr_full = np.zeros(k + tail, dtype=np.float64)
            apr_full[:k] = apr
            extr, intrinsic = full_block_siso(sys1[:k + tail], par1[:k + tail], apr_full, k, cfg)
            extr_nat = clip_signed(extr, 6 if cfg.quantized else 0)
            final_intrinsic = intrinsic
        else:
            apr_ilv = clip_signed(extr_nat[cfg.pi], 5 if cfg.quantized else 0)
            apr_full = np.zeros(k + tail, dtype=np.float64)
            apr_full[:k] = apr_ilv
            extr_ilv, intrinsic_ilv = full_block_siso(sys2[:k + tail], par2[:k + tail], apr_full, k, cfg)
            next_extr = np.zeros(k, dtype=np.float64)
            next_extr[cfg.pi] = clip_signed(extr_ilv, 6 if cfg.quantized else 0)
            extr_nat = next_extr
            final_intrinsic[cfg.pi] = intrinsic_ilv
    return final_intrinsic


def turbo_decode_windowed_parallel(sys1, par1, sys2, par2, cfg):
    k = cfg.k
    s = cfg.segment_len
    tail = 3 if cfg.use_tail else 0
    extr_nat = np.zeros(k, dtype=np.float64)
    final_intrinsic = np.zeros(k, dtype=np.float64)

    for half in range(cfg.half_iters):
        next_extr = np.zeros(k, dtype=np.float64)
        natural = half % 2 == 0

        if natural:
            apr_full = np.zeros(k + tail, dtype=np.float64)
            apr_full[:k] = np.array([sat6_to_5(v) for v in extr_nat], dtype=np.float64) if cfg.quantized else extr_nat
            sys_full = sys1[:k + tail]
            par_full = par1[:k + tail]
        else:
            apr_full = np.zeros(k + tail, dtype=np.float64)
            apr_values = extr_nat[cfg.pi]
            apr_full[:k] = np.array([sat6_to_5(v) for v in apr_values], dtype=np.float64) if cfg.quantized else apr_values
            sys_full = sys2[:k + tail]
            par_full = par2[:k + tail]

        for core in range(cfg.num_siso):
            start = core * s
            info_len = s
            tail_len = tail if core == cfg.num_siso - 1 else 0
            extr, intrinsic = siso_windowed_decode(
                sys_full, par_full, apr_full, start, info_len, tail_len, core, cfg
            )
            if natural:
                lo = start
                hi = start + info_len
                next_extr[lo:hi] = extr
                final_intrinsic[lo:hi] = intrinsic
            else:
                local = np.arange(start, start + info_len, dtype=np.int32)
                next_extr[cfg.pi[local]] = extr
                final_intrinsic[cfg.pi[local]] = intrinsic

        extr_nat = next_extr

    return final_intrinsic


def r4_prep_values(sys_llr, par_llr, apr_llr):
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


def r4_compute_radix2(sys_even, sys_odd, par_even, par_odd, apr_even, apr_odd):
    pre_odd = trunc_signed(r4_prep_values(sys_odd, par_odd, apr_odd), 7)
    pre_even = trunc_signed(r4_prep_values(sys_even, par_even, apr_even), 7)
    bm_odd = np.array([pre_odd[i] for i in R2_PRE_IDX], dtype=np.int32)
    bm_even = np.array([pre_even[i] for i in R2_PRE_IDX], dtype=np.int32)
    return np.stack((bm_odd, bm_even))


def r4_compute_radix4(bm_r2):
    bm_r4 = np.zeros(32, dtype=np.int32)
    for i in range(32):
        # RTL order is even trellis address first, then odd trellis address.
        bm_r4[i] = int(bm_r2[1, i % 16]) + int(bm_r2[0, i // 2])
    return trunc_signed(bm_r4, 8)


def r4_acs_forward(sm, bm_r4):
    out = np.zeros(NUM_STATES, dtype=np.int32)
    for dest in range(NUM_STATES):
        preds = (0, 1, 2, 3) if (dest % 2 == 0) else (4, 5, 6, 7)
        cands = [
            trunc_int(int(sm[preds[i]]) + int(bm_r4[dest * 4 + i]), SM_W)
            for i in range(4)
        ]
        out[dest] = mod_max_list(cands)
    return out


def r4_acs_backward(sm, bm_r4):
    out = np.zeros(NUM_STATES, dtype=np.int32)
    for pred in range(NUM_STATES):
        succs = (0, 2, 4, 6) if pred < 4 else (1, 3, 5, 7)
        cands = [
            trunc_int(int(sm[succ]) + int(bm_r4[succ * 4 + (pred % 4)]), SM_W)
            for succ in succs
        ]
        out[pred] = mod_max_list(cands)
    return out


def r4_pair_at(sys_seg, par_seg, apr_seg, addr, trellis_len):
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


def r4_bm_for_local_pair(sys_seg, par_seg, apr_seg, addr, trellis_len):
    se, so, pe, po, ae, ao = r4_pair_at(sys_seg, par_seg, apr_seg, addr, trellis_len)
    return r4_compute_radix2(se, so, pe, po, ae, ao)


def r4_num_windows(cfg):
    tail = 3 if cfg.use_tail else 0
    return math.ceil((cfg.segment_len + tail) / cfg.window_size)


def r4_forward_pass(sys_seg, par_seg, apr_seg, core_id, trellis_len, cfg, dummy_prefix=None):
    win_len_r4 = cfg.window_size // 2
    num_windows = r4_num_windows(cfg)
    alpha_mem = np.zeros((num_windows, win_len_r4, NUM_STATES), dtype=np.int32)
    gamma_mem = np.zeros((num_windows, win_len_r4, 2, 16), dtype=np.int32)

    if core_id == 0:
        sm = np.array([0] + [NEG_INF_FIXED] * 7, dtype=np.int32)
    else:
        sm = np.zeros(NUM_STATES, dtype=np.int32)
        if cfg.boundary_mode == "paper":
            if dummy_prefix is None:
                dummy_prefix = (
                    np.zeros(cfg.window_size, dtype=np.int32),
                    np.zeros(cfg.window_size, dtype=np.int32),
                    np.zeros(cfg.window_size, dtype=np.int32),
                )
            dummy_sys, dummy_par, dummy_apr = dummy_prefix
            for step in range(win_len_r4):
                addr = 2 * step
                bm_r2 = r4_bm_for_local_pair(dummy_sys, dummy_par, dummy_apr, addr, cfg.window_size)
                sm = r4_acs_forward(sm, r4_compute_radix4(bm_r2))

    for win in range(num_windows):
        base = win * cfg.window_size
        for step in range(win_len_r4):
            addr = base + 2 * step
            bm_r2 = r4_bm_for_local_pair(sys_seg, par_seg, apr_seg, addr, trellis_len)
            alpha_mem[win, step] = sm
            gamma_mem[win, step] = bm_r2
            sm = r4_acs_forward(sm, r4_compute_radix4(bm_r2))
    return alpha_mem, gamma_mem


def r4_dummy_backward(sys_seg, par_seg, apr_seg, win, trellis_len, cfg):
    sm = np.zeros(NUM_STATES, dtype=np.int32)
    base = win * cfg.window_size
    win_len_r4 = cfg.window_size // 2
    for step in range(win_len_r4 - 1, -1, -1):
        bm_r2 = r4_bm_for_local_pair(sys_seg, par_seg, apr_seg, base + 2 * step, trellis_len)
        sm = r4_acs_backward(sm, r4_compute_radix4(bm_r2))
    return sm


def r4_derive_alpha_km1(alpha, bm_even):
    out = np.zeros(NUM_STATES, dtype=np.int32)
    for dest in range(NUM_STATES):
        cands = []
        for pred_idx, pred in enumerate(R2_PREDS[dest]):
            idx = dest * 2 + pred_idx
            cands.append(trunc_int(int(alpha[pred]) + int(bm_even[idx]), SM_W))
        out[dest] = mod_max(cands[0], cands[1])
    return out


def r4_derive_beta_km1(beta, bm_odd):
    out = np.zeros(NUM_STATES, dtype=np.int32)
    for pred in range(NUM_STATES):
        cands = []
        for _, dest, idx, _ in TRANS_BY_PRED[pred]:
            cands.append(trunc_int(int(beta[dest]) + int(bm_odd[idx]), SM_W))
        out[pred] = mod_max(cands[0], cands[1])
    return out


def r4_llr_one_pair(alpha_km2, beta_k, bm_r2):
    bm_odd = bm_r2[0]
    bm_even = bm_r2[1]
    alpha_km1 = r4_derive_alpha_km1(alpha_km2, bm_even)
    beta_km1 = r4_derive_beta_km1(beta_k, bm_odd)

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


def r4_siso_decode(sys_seg, par_seg, apr_seg, core_id, trellis_len, cfg, dummy_prefix=None, dbr_suffix=None):
    alpha_mem, gamma_mem = r4_forward_pass(sys_seg, par_seg, apr_seg, core_id, trellis_len, cfg, dummy_prefix)
    s = cfg.segment_len
    extr = np.zeros(s, dtype=np.int32)
    intrinsic = np.zeros(s, dtype=np.int32)

    if cfg.boundary_mode == "paper" and dbr_suffix is not None:
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

    num_windows = r4_num_windows(cfg)
    win_len_r4 = cfg.window_size // 2
    for win in range(num_windows):
        if core_id == cfg.num_siso - 1 and win == num_windows - 1:
            beta = np.array([0] + [NEG_INF_FIXED] * 7, dtype=np.int32)
        else:
            beta = r4_dummy_backward(dbr_sys_seg, dbr_par_seg, dbr_apr_seg, win + 1, dbr_trellis_len, cfg)

        for step in range(win_len_r4 - 1, -1, -1):
            addr = win * cfg.window_size + 2 * step
            bm_r2 = gamma_mem[win, step]
            le_odd, le_even, ld_odd, ld_even = r4_llr_one_pair(alpha_mem[win, step], beta, bm_r2)
            if addr < s:
                extr[addr] = le_even
                intrinsic[addr] = ld_even
            if addr + 1 < s:
                extr[addr + 1] = le_odd
                intrinsic[addr + 1] = ld_odd
            beta = r4_acs_backward(beta, r4_compute_radix4(bm_r2))

    return extr, intrinsic


def r4_zero_pad(vals, length):
    out = np.zeros(length, dtype=np.int32)
    vals = np.asarray(vals, dtype=np.int32)
    n = min(length, max(0, len(vals)))
    if n:
        out[:n] = vals[:n]
    return out


def r4_build_suffix(full_sys, full_par, apr_lookup, start_idx, length, cfg):
    sys_suffix = r4_zero_pad(full_sys[start_idx:start_idx + length], length)
    par_suffix = r4_zero_pad(full_par[start_idx:start_idx + length], length)
    apr_suffix = np.zeros(length, dtype=np.int32)
    for i in range(length):
        idx = start_idx + i
        if 0 <= idx < cfg.k:
            apr_suffix[i] = sat6_to_5(apr_lookup(idx))
    return sys_suffix, par_suffix, apr_suffix


def r4_paper_dummy_prefix(full_sys, full_par, apr_lookup, start_idx, cfg):
    prefix_start = start_idx - cfg.window_size
    if prefix_start >= 0:
        sys_prefix = r4_zero_pad(full_sys[prefix_start:start_idx], cfg.window_size)
        par_prefix = r4_zero_pad(full_par[prefix_start:start_idx], cfg.window_size)
    else:
        sys_prefix = np.zeros(cfg.window_size, dtype=np.int32)
        par_prefix = np.zeros(cfg.window_size, dtype=np.int32)

    apr_prefix = np.zeros(cfg.window_size, dtype=np.int32)
    for i in range(cfg.window_size):
        idx = prefix_start + i
        if 0 <= idx < cfg.k:
            apr_prefix[i] = sat6_to_5(apr_lookup(idx))
    return sys_prefix, par_prefix, apr_prefix


def turbo_decode_radix4(sys1, par1, sys2, par2, cfg, return_extrinsic=False):
    if not cfg.quantized:
        raise ValueError("--decoder radix4 models the fixed-point RTL and requires --quantize")
    if cfg.num_siso != 2:
        raise ValueError("--decoder radix4 is bit-accurate for the current N=2 RTL target only")
    if cfg.window_size % 2 != 0:
        raise ValueError("--decoder radix4 requires an even window size")

    k = cfg.k
    s = cfg.segment_len
    tail = 3 if cfg.use_tail else 0
    num_windows = r4_num_windows(cfg)

    sys_nat = np.asarray(sys1[:k + tail], dtype=np.int32)
    par1_nat = np.asarray(par1[:k + tail], dtype=np.int32)
    sys_ilv = np.asarray(sys2[:k + tail], dtype=np.int32)
    par2_ilv = np.asarray(par2[:k + tail], dtype=np.int32)

    extr_mem = np.zeros(k, dtype=np.int32)
    final_intrinsic = np.zeros(k, dtype=np.int32)

    for half in range(cfg.half_iters):
        next_extr = np.zeros(k, dtype=np.int32)
        natural = half % 2 == 0
        for core in range(cfg.num_siso):
            lo = core * s
            hi = lo + s
            trellis_len = s + tail if core == cfg.num_siso - 1 else s
            suffix_len = max(0, (num_windows + 1) * cfg.window_size - trellis_len)

            if natural:
                sys_seg = r4_zero_pad(sys_nat[lo:lo + trellis_len], trellis_len)
                par_seg = r4_zero_pad(par1_nat[lo:lo + trellis_len], trellis_len)
                apr_seg = np.zeros(trellis_len, dtype=np.int32)
                apr_seg[:s] = np.array([sat6_to_5(v) for v in extr_mem[lo:hi]], dtype=np.int32)
                dummy_prefix = r4_paper_dummy_prefix(sys_nat, par1_nat, lambda idx: extr_mem[idx], lo, cfg)
                dbr_suffix = r4_build_suffix(
                    sys_nat, par1_nat, lambda idx: extr_mem[idx], lo + trellis_len, suffix_len, cfg
                )
                extr, intrinsic = r4_siso_decode(
                    sys_seg, par_seg, apr_seg, core, trellis_len, cfg,
                    dummy_prefix=dummy_prefix,
                    dbr_suffix=dbr_suffix,
                )
                next_extr[lo:hi] = extr
                final_intrinsic[lo:hi] = intrinsic
            else:
                ks = np.arange(lo, hi, dtype=np.int32)
                sys_seg = r4_zero_pad(sys_ilv[lo:lo + trellis_len], trellis_len)
                par_seg = r4_zero_pad(par2_ilv[lo:lo + trellis_len], trellis_len)
                apr_seg = np.zeros(trellis_len, dtype=np.int32)
                apr_seg[:s] = np.array([sat6_to_5(extr_mem[cfg.pi[kidx]]) for kidx in ks], dtype=np.int32)
                dummy_prefix = r4_paper_dummy_prefix(sys_ilv, par2_ilv, lambda idx: extr_mem[cfg.pi[idx]], lo, cfg)
                dbr_suffix = r4_build_suffix(
                    sys_ilv, par2_ilv, lambda idx: extr_mem[cfg.pi[idx]], lo + trellis_len, suffix_len, cfg
                )
                extr, intrinsic = r4_siso_decode(
                    sys_seg, par_seg, apr_seg, core, trellis_len, cfg,
                    dummy_prefix=dummy_prefix,
                    dbr_suffix=dbr_suffix,
                )
                next_extr[cfg.pi[ks]] = extr
                final_intrinsic[cfg.pi[ks]] = intrinsic

        extr_mem = next_extr

    if return_extrinsic:
        return extr_mem, final_intrinsic
    return final_intrinsic


def decode_frame(sys1, par1, sys2, par2, cfg, decoder):
    if decoder == "full":
        return turbo_decode_full(sys1, par1, sys2, par2, cfg)
    if decoder == "windowed":
        return turbo_decode_windowed_parallel(sys1, par1, sys2, par2, cfg)
    if decoder == "radix4":
        return turbo_decode_radix4(sys1, par1, sys2, par2, cfg)
    raise ValueError(f"unknown decoder: {decoder}")


def hard_bits(llr):
    return (llr < 0).astype(np.int32)


def sign_extend(val, width):
    mask = (1 << width) - 1
    val = int(val) & mask
    sign = 1 << (width - 1)
    return val - (1 << width) if val & sign else val


def load_folded_bram_5(data_dir, even_name, odd_name, k, num_siso, tail_len):
    if num_siso != 2:
        raise ValueError("BRAM-vector mode currently supports the folded N=2 RTL memory format only")

    segment_len = k // num_siso
    total_len = k + tail_len
    out = np.zeros(total_len, dtype=np.float64)

    even_path = os.path.join(data_dir, even_name)
    odd_path = os.path.join(data_dir, odd_name)

    with open(even_path) as f:
        for r, line in enumerate(f):
            fields = line.strip().split()
            if not fields:
                continue
            word = int(fields[0], 16)
            local = 2 * r
            if local < segment_len:
                out[local] = sign_extend(word & 0x1F, 5)
            if segment_len + local < total_len:
                out[segment_len + local] = sign_extend((word >> 5) & 0x1F, 5)

    with open(odd_path) as f:
        for r, line in enumerate(f):
            fields = line.strip().split()
            if not fields:
                continue
            word = int(fields[0], 16)
            local = 2 * r + 1
            if local < segment_len:
                out[local] = sign_extend(word & 0x1F, 5)
            if segment_len + local < total_len:
                out[segment_len + local] = sign_extend((word >> 5) & 0x1F, 5)

    return out


def load_folded_bram_6_from_ld(path, k, num_siso):
    if num_siso != 2:
        raise ValueError("folded 6-bit LD/extrinsic load currently supports the N=2 RTL memory format only")

    segment_len = k // num_siso
    out = np.zeros(k, dtype=np.int32)
    with open(path) as f:
        for r, line in enumerate(f):
            parts = line.strip().split()
            if len(parts) != 2:
                continue
            even = int(parts[0], 16)
            odd = int(parts[1], 16)
            out[2 * r] = sign_extend(even & 0x3F, 6)
            out[2 * r + segment_len] = sign_extend((even >> 6) & 0x3F, 6)
            out[2 * r + 1] = sign_extend(odd & 0x3F, 6)
            out[2 * r + 1 + segment_len] = sign_extend((odd >> 6) & 0x3F, 6)
    return out


def load_signed_vector(path, expected_len):
    vals = np.atleast_1d(np.loadtxt(path, dtype=np.float64))
    if vals.size != expected_len:
        raise ValueError(f"{path} has {vals.size} values, expected {expected_len}")
    return vals


def load_true_bits(data_dir, k):
    path = os.path.join(data_dir, "true_info_bits.txt")
    if not os.path.exists(path):
        return None
    vals = np.atleast_1d(np.loadtxt(path, dtype=np.int32))
    if vals.size != k:
        raise ValueError(f"{path} has {vals.size} values, expected {k}")
    return vals


def ber_from_decisions(decisions, bits):
    errors = int(np.count_nonzero(decisions != bits))
    return errors, errors / len(bits)


def append_ber_report(lines, label, llr_or_bits, true_bits, is_bits=False):
    if true_bits is None:
        lines.append(f"  {label}: skipped, true_info_bits.txt not found")
        return None

    decisions = llr_or_bits.astype(np.int32) if is_bits else hard_bits(llr_or_bits)
    errors, ber = ber_from_decisions(decisions, true_bits)
    lines.append(f"  {label}: {errors}/{len(true_bits)} = {ber:.6f}")
    return errors, ber


def compare_intrinsic_vectors(lines, label, model_intrinsic, other_intrinsic, true_bits, show_mismatches=10):
    model_hard = hard_bits(model_intrinsic)
    other_hard = hard_bits(other_intrinsic)
    value_mism_idx = np.nonzero(other_intrinsic != model_intrinsic)[0]
    hard_mism_idx = np.nonzero(other_hard != model_hard)[0]
    max_diff = float(np.max(np.abs(other_intrinsic - model_intrinsic))) if model_intrinsic.size else 0.0

    lines.append(f"{label}")
    lines.append(f"  intrinsic value mismatches: {len(value_mism_idx)}/{model_intrinsic.size}")
    lines.append(f"  hard-bit mismatches: {len(hard_mism_idx)}/{model_intrinsic.size}")
    lines.append(f"  max |difference|: {max_diff:.10g}")
    append_ber_report(lines, "other final L_D BER", other_intrinsic, true_bits)

    if len(value_mism_idx):
        lines.append("  first value mismatches:")
        for idx in value_mism_idx[:show_mismatches]:
            lines.append(
                f"    i={int(idx)} model={model_intrinsic[idx]:.10g} "
                f"other={other_intrinsic[idx]:.10g} hard_model={int(model_hard[idx])} "
                f"hard_other={int(other_hard[idx])}"
            )
    lines.append("")


def compare_value_vectors(lines, label, model_vals, other_vals, show_mismatches=10):
    value_mism_idx = np.nonzero(other_vals != model_vals)[0]
    max_diff = int(np.max(np.abs(other_vals - model_vals))) if model_vals.size else 0
    lines.append(label)
    lines.append(f"  value mismatches: {len(value_mism_idx)}/{model_vals.size}")
    lines.append(f"  max |difference|: {max_diff}")
    if len(value_mism_idx):
        lines.append("  first value mismatches:")
        for idx in value_mism_idx[:show_mismatches]:
            lines.append(f"    i={int(idx)} model={int(model_vals[idx])} other={int(other_vals[idx])}")
    lines.append("")


def run_bram_frame(args):
    if args.K % args.num_siso != 0:
        raise ValueError("K must be divisible by --num-siso for equal SISO segmentation")
    if args.window_size % 2 != 0:
        raise ValueError("--window-size must be even for the radix-4 RTL schedule")

    data_dir = os.path.abspath(args.from_bram_dir)
    tail_len = 0 if args.no_tail else 3
    pi = make_qpp(args.K)
    f1, f2 = QPP_TABLE[args.K]
    info_rate = args.K / (3 * args.K + (12 if not args.no_tail else 0))

    if args.snr_rate_mode == "information":
        channel_rate = info_rate
    elif args.snr_rate_mode == "coded":
        channel_rate = 1.0
    else:
        channel_rate = args.channel_rate
        if channel_rate <= 0:
            raise ValueError("--channel-rate must be positive when --snr-rate-mode custom is used")

    cfg = DecoderConfig(
        k=args.K,
        pi=pi,
        num_siso=args.num_siso,
        window_size=args.window_size,
        half_iters=args.half_iters,
        scale=args.scale,
        quantized=args.quantize,
        boundary_mode=args.boundary_mode,
        use_tail=not args.no_tail,
        channel_rate=channel_rate,
    )
    if args.decoder == "radix4" and not cfg.quantized:
        raise ValueError("--decoder radix4 models the fixed-point RTL and requires --quantize")
    if args.decoder == "radix4" and cfg.num_siso != 2:
        raise ValueError("--decoder radix4 is bit-accurate for the current N=2 RTL target only")

    sys1 = load_folded_bram_5(data_dir, "sys_even_ram.hex", "sys_odd_ram.hex", args.K, args.num_siso, tail_len)
    par1 = load_folded_bram_5(data_dir, "par1_even_ram.hex", "par1_odd_ram.hex", args.K, args.num_siso, tail_len)
    sys2 = load_folded_bram_5(
        data_dir, "sys_ilv_even_ram.hex", "sys_ilv_odd_ram.hex", args.K, args.num_siso, tail_len
    )
    par2 = load_folded_bram_5(data_dir, "par2_even_ram.hex", "par2_odd_ram.hex", args.K, args.num_siso, tail_len)

    final_extrinsic = None
    if args.decoder == "radix4":
        final_extrinsic, intrinsic = turbo_decode_radix4(sys1, par1, sys2, par2, cfg, return_extrinsic=True)
    else:
        intrinsic = decode_frame(sys1, par1, sys2, par2, cfg, args.decoder)

    if cfg.quantized:
        intrinsic_to_save = np.rint(intrinsic).astype(np.int32)
        intrinsic_fmt = "%d"
    else:
        intrinsic_to_save = intrinsic
        intrinsic_fmt = "%.10g"

    tag = args.decoder.replace("-", "_")
    model_hard = hard_bits(intrinsic)
    true_bits = load_true_bits(data_dir, args.K)

    input_fmt = "%d" if cfg.quantized else "%.10g"
    np.savetxt(os.path.join(data_dir, "windowed_input_sys1.txt"), sys1, fmt=input_fmt)
    np.savetxt(os.path.join(data_dir, "windowed_input_par1.txt"), par1, fmt=input_fmt)
    np.savetxt(os.path.join(data_dir, "windowed_input_sys2.txt"), sys2, fmt=input_fmt)
    np.savetxt(os.path.join(data_dir, "windowed_input_par2.txt"), par2, fmt=input_fmt)
    intrinsic_name = f"{tag}_final_intrinsic.txt"
    hard_name = f"{tag}_final_hard_bits.txt"
    np.savetxt(os.path.join(data_dir, intrinsic_name), intrinsic_to_save, fmt=intrinsic_fmt)
    np.savetxt(os.path.join(data_dir, hard_name), model_hard, fmt="%d")
    if args.decoder == "windowed":
        np.savetxt(os.path.join(data_dir, "windowed_final_intrinsic.txt"), intrinsic_to_save, fmt=intrinsic_fmt)
        np.savetxt(os.path.join(data_dir, "windowed_final_hard_bits.txt"), model_hard, fmt="%d")
    if final_extrinsic is not None:
        np.savetxt(os.path.join(data_dir, f"{tag}_final_extrinsic.txt"), final_extrinsic, fmt="%d")

    lines = [
        "Windowed Parallel Deterministic BRAM Comparison",
        "=" * 52,
        f"Data directory: {data_dir}",
        f"K: {args.K}",
        f"NUM_SISO: {args.num_siso}",
        f"Segment length: {cfg.segment_len}",
        f"QPP f1/f2: {f1}/{f2}",
        f"Tail trellis steps: {tail_len}",
        f"Decoder: {args.decoder}",
        f"Window size: {args.window_size}",
        f"Half iterations: {args.half_iters}",
        f"Quantized: {cfg.quantized}",
        f"Extrinsic scale: {args.scale}",
        f"Boundary mode: {args.boundary_mode}",
        "",
        "Input source",
        "  sys1: sys_even_ram.hex + sys_odd_ram.hex",
        "  par1: par1_even_ram.hex + par1_odd_ram.hex",
        "  sys2/interleaved systematic: sys_ilv_even_ram.hex + sys_ilv_odd_ram.hex",
        "  par2: par2_even_ram.hex + par2_odd_ram.hex",
        "",
        "Saved deterministic artifacts",
        "  windowed_input_sys1.txt",
        "  windowed_input_par1.txt",
        "  windowed_input_sys2.txt",
        "  windowed_input_par2.txt",
        f"  {intrinsic_name}",
        f"  {hard_name}",
    ]
    if final_extrinsic is not None:
        lines.append(f"  {tag}_final_extrinsic.txt")
    lines.extend(["", "BER"])
    append_ber_report(lines, "channel hard decisions from sys1", sys1[:args.K], true_bits)
    append_ber_report(lines, f"windowed_parallel_ber.py {args.decoder} final L_D", intrinsic, true_bits)
    lines.append("")

    metadata_path = os.path.join(data_dir, "vector_metadata.txt")
    if os.path.exists(metadata_path):
        lines.append("Vector metadata")
        with open(metadata_path) as f:
            for line in f:
                line = line.rstrip()
                if line:
                    lines.append(f"  {line}")
        lines.append("")

    lines.append("Comparison note")
    if args.decoder == "radix4":
        lines.extend([
            "  This is the RTL bit-accurate radix-4 fixed-point mode in scripts/windowed_parallel_ber.py.",
            "  It reuses the exact RTL BRAM LLR inputs and should match RTL/turbo_ref_model.py for the N=2 target.",
            "",
        ])
    else:
        lines.extend([
            "  This is the paper-style algorithm model in scripts/windowed_parallel_ber.py.",
            "  It reuses the exact RTL BRAM LLR inputs, but it is still a scalar trellis model.",
            "  Use --decoder radix4 for the RTL bit-accurate fixed-point path.",
            "",
        ])

    rtl_path = os.path.join(data_dir, "rtl_final_intrinsic.txt")
    if os.path.exists(rtl_path):
        rtl_intrinsic = load_signed_vector(rtl_path, args.K)
        compare_intrinsic_vectors(lines, "RTL final intrinsic comparison", intrinsic_to_save, rtl_intrinsic, true_bits)
    else:
        lines.extend(["RTL final intrinsic comparison", "  skipped: rtl_final_intrinsic.txt not found", ""])

    ref_path = os.path.join(data_dir, "ref_final_intrinsic.txt")
    if os.path.exists(ref_path):
        ref_intrinsic = load_signed_vector(ref_path, args.K)
        compare_intrinsic_vectors(lines, "turbo_ref_model.py final intrinsic comparison", intrinsic_to_save, ref_intrinsic, true_bits)
    else:
        lines.extend(["turbo_ref_model.py final intrinsic comparison", "  skipped: ref_final_intrinsic.txt not found", ""])

    if final_extrinsic is not None:
        rtl_extr_path = os.path.join(data_dir, "ld_ram_output.hex")
        if os.path.exists(rtl_extr_path):
            rtl_extrinsic = load_folded_bram_6_from_ld(rtl_extr_path, args.K, args.num_siso)
            compare_value_vectors(lines, "RTL final extrinsic comparison", final_extrinsic, rtl_extrinsic)
        else:
            lines.extend(["RTL final extrinsic comparison", "  skipped: ld_ram_output.hex not found", ""])

        ref_extr_path = os.path.join(data_dir, "ref_final_extrinsic.hex")
        if os.path.exists(ref_extr_path):
            ref_extrinsic = load_folded_bram_6_from_ld(ref_extr_path, args.K, args.num_siso)
            compare_value_vectors(lines, "turbo_ref_model.py final extrinsic comparison", final_extrinsic, ref_extrinsic)
        else:
            lines.extend(["turbo_ref_model.py final extrinsic comparison", "  skipped: ref_final_extrinsic.hex not found", ""])

    report_path = os.path.join(data_dir, "windowed_bram_compare_results.txt")
    with open(report_path, "w") as f:
        f.write("\n".join(lines))
        f.write("\n")

    print("\n".join(lines))
    print(f"Saved: {report_path}")
    return intrinsic


def run_one_frame(cfg, ebn0_db, rng, quant_scale):
    info = rng.integers(0, 2, cfg.k).astype(np.int32)
    sys1_bits, par1_bits, sys2_bits, par2_bits = turbo_encode(info, cfg.pi)
    tail = 3 if cfg.use_tail else 0

    sys1 = awgn_channel(sys1_bits[:cfg.k + tail], ebn0_db, cfg.channel_rate, rng)
    par1 = awgn_channel(par1_bits[:cfg.k + tail], ebn0_db, cfg.channel_rate, rng)
    par2 = awgn_channel(par2_bits[:cfg.k + tail], ebn0_db, cfg.channel_rate, rng)

    sys2 = np.zeros(cfg.k + tail, dtype=np.float64)
    sys2[:cfg.k] = sys1[:cfg.k][cfg.pi]
    if tail:
        sys2[cfg.k:] = awgn_channel(sys2_bits[cfg.k:cfg.k + tail], ebn0_db, cfg.channel_rate, rng)

    if cfg.quantized:
        sys1 = quantize_llr(sys1, 5, quant_scale)
        par1 = quantize_llr(par1, 5, quant_scale)
        sys2 = quantize_llr(sys2, 5, quant_scale)
        par2 = quantize_llr(par2, 5, quant_scale)

    return info, sys1, par1, sys2, par2


def ber_for_frame(info, intrinsic):
    hard = hard_bits(intrinsic)
    errors = int(np.count_nonzero(hard != info))
    return errors


def run_frame_task(task):
    cfg, decoder, ebn0_db, seed, quant_scale = task
    rng = np.random.default_rng(seed)
    info, sys1, par1, sys2, par2 = run_one_frame(cfg, ebn0_db, rng, quant_scale)
    intrinsic = decode_frame(sys1, par1, sys2, par2, cfg, decoder)
    return ber_for_frame(info, intrinsic), cfg.k


def run_point_serial(args, cfg, ebn0, rng, quant_scale):
    total_bits = 0
    total_errors = 0
    total_frames = 0
    t0 = time.time()

    while total_errors < args.min_errors and total_frames < args.max_frames:
        info, sys1, par1, sys2, par2 = run_one_frame(cfg, ebn0, rng, quant_scale)
        intrinsic = decode_frame(sys1, par1, sys2, par2, cfg, args.decoder)

        errors = ber_for_frame(info, intrinsic)
        total_errors += errors
        total_bits += args.K
        total_frames += 1

        if total_frames % args.progress_interval == 0:
            ber = total_errors / total_bits if total_bits else 0.0
            elapsed = time.time() - t0
            print(
                f"\r  Eb/N0={ebn0:4.2f} dB: frames={total_frames:4d}, "
                f"errors={total_errors:6d}, BER~{ber:.3e}, {elapsed:.1f}s",
                end="",
                flush=True,
            )

    return total_errors, total_bits, total_frames, time.time() - t0


def run_point_parallel(args, cfg, ebn0, seed_seq, quant_scale, executor):
    total_bits = 0
    total_errors = 0
    total_frames = 0
    submitted = 0
    t0 = time.time()
    futures = set()

    def submit_one():
        nonlocal submitted
        child = seed_seq.spawn(1)[0]
        seed = int(child.generate_state(1, dtype=np.uint32)[0])
        task = (cfg, args.decoder, float(ebn0), seed, quant_scale)
        futures.add(executor.submit(run_frame_task, task))
        submitted += 1

    initial = min(args.jobs, args.max_frames)
    for _ in range(initial):
        submit_one()

    while futures:
        for fut in as_completed(futures):
            futures.remove(fut)
            errors, bits = fut.result()
            total_errors += errors
            total_bits += bits
            total_frames += 1

            if total_frames % args.progress_interval == 0:
                ber = total_errors / total_bits if total_bits else 0.0
                elapsed = time.time() - t0
                print(
                    f"\r  Eb/N0={ebn0:4.2f} dB: frames={total_frames:4d}, "
                    f"errors={total_errors:6d}, BER~{ber:.3e}, {elapsed:.1f}s",
                    end="",
                    flush=True,
                )

            if total_errors < args.min_errors and submitted < args.max_frames:
                submit_one()
            break

        if total_errors >= args.min_errors:
            for fut in futures:
                fut.cancel()
            for fut in as_completed(futures):
                if not fut.cancelled():
                    errors, bits = fut.result()
                    total_errors += errors
                    total_bits += bits
                    total_frames += 1
            break

    return total_errors, total_bits, total_frames, time.time() - t0


def run_sweep(args):
    if args.K % args.num_siso != 0:
        raise ValueError("K must be divisible by --num-siso for equal SISO segmentation")
    if args.window_size % 2 != 0:
        raise ValueError("--window-size must be even for the radix-4 RTL schedule")

    pi = make_qpp(args.K)
    quant_scale = args.quant_scale if args.quant_scale > 0 else None
    info_rate = args.K / (3 * args.K + (12 if not args.no_tail else 0))
    if args.snr_rate_mode == "information":
        channel_rate = info_rate
    elif args.snr_rate_mode == "coded":
        channel_rate = 1.0
    else:
        channel_rate = args.channel_rate
        if channel_rate <= 0:
            raise ValueError("--channel-rate must be positive when --snr-rate-mode custom is used")

    cfg = DecoderConfig(
        k=args.K,
        pi=pi,
        num_siso=args.num_siso,
        window_size=args.window_size,
        half_iters=args.half_iters,
        scale=args.scale,
        quantized=args.quantize,
        boundary_mode=args.boundary_mode,
        use_tail=not args.no_tail,
        channel_rate=channel_rate,
    )
    if args.decoder == "radix4" and not cfg.quantized:
        raise ValueError("--decoder radix4 models the fixed-point RTL and requires --quantize")
    if args.decoder == "radix4" and cfg.num_siso != 2:
        raise ValueError("--decoder radix4 is bit-accurate for the current N=2 RTL target only")

    f1, f2 = QPP_TABLE[args.K]
    ebn0_points = np.arange(args.ebn0_start, args.ebn0_stop + 1e-9, args.ebn0_step)
    rng = np.random.default_rng(args.seed)
    results = []

    print("=" * 78)
    print("Windowed/parallel LTE turbo BER model")
    print("=" * 78)
    print(f"K={args.K}, f1={f1}, f2={f2}, N={args.num_siso}, S={cfg.segment_len}")
    print(f"decoder={args.decoder}, M={args.window_size}, half-iters={args.half_iters}")
    print(f"quantized={args.quantize}, scale={args.scale}, boundary={args.boundary_mode}, tail={cfg.use_tail}")
    print(f"snr-rate-mode={args.snr_rate_mode}, channel_rate={cfg.channel_rate:.9f}, LTE info_rate={info_rate:.9f}")
    print(f"jobs={args.jobs}, min-errors={args.min_errors}, max-frames={args.max_frames}")
    print(f"Eb/N0 points: {ebn0_points}")

    executor = ProcessPoolExecutor(max_workers=args.jobs) if args.jobs > 1 else None
    try:
        for ebn0 in ebn0_points:
            ebn0_seed = int(round(float(ebn0) * 1000.0))
            if executor is None:
                rng = np.random.default_rng(np.random.SeedSequence([args.seed, ebn0_seed]))
                total_errors, total_bits, total_frames, elapsed = run_point_serial(
                    args, cfg, ebn0, rng, quant_scale
                )
            else:
                seed_seq = np.random.SeedSequence([args.seed, ebn0_seed])
                total_errors, total_bits, total_frames, elapsed = run_point_parallel(
                    args, cfg, ebn0, seed_seq, quant_scale, executor
                )

            ber = total_errors / total_bits if total_bits else 0.0
            print(
                f"\r  Eb/N0={ebn0:4.2f} dB: BER={ber:.6e} "
                f"({total_errors}/{total_bits}, frames={total_frames}, {elapsed:.1f}s)"
            )
            results.append((float(ebn0), ber, total_errors, total_bits, total_frames))
    finally:
        if executor is not None:
            executor.shutdown(cancel_futures=True)

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)
    qtag = "fixed" if args.quantize else "float"
    ttag = "tail" if cfg.use_tail else "notail"
    rtag = args.snr_rate_mode if args.snr_rate_mode != "custom" else f"R{cfg.channel_rate:g}"
    out_path = os.path.join(
        out_dir,
        f"ber_curve_K{args.K}_{args.decoder}_N{args.num_siso}_{qtag}_{args.boundary_mode}_{ttag}_{rtag}.csv",
    )
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["ebn0_db", "ber", "errors", "bits", "frames"])
        writer.writerows(results)
    print(f"Saved: {out_path}")
    return results


def main():
    parser = argparse.ArgumentParser(description="BER for windowed parallel LTE turbo decoder model")
    parser.add_argument(
        "--decoder",
        choices=["full", "windowed", "radix4"],
        default="windowed",
        help=(
            "full/windowed are algorithm models; radix4 is the N=2 fixed-point RTL-bit-accurate path "
            "and requires --quantize"
        ),
    )
    parser.add_argument("--K", type=int, default=3200)
    parser.add_argument("--num-siso", type=int, default=2)
    parser.add_argument("--window-size", type=int, default=30)
    parser.add_argument("--half-iters", type=int, default=11, help="11 half-iterations = 5.5 full iterations")
    parser.add_argument("--scale", type=float, default=0.6875)
    parser.add_argument("--boundary-mode", choices=["paper", "rtl"], default="paper")
    parser.add_argument("--no-tail", action="store_true", help="drop LTE tail trellis steps for legacy no-tail experiments")
    parser.add_argument("--quantize", action="store_true", help="use fixed-point hardware widths")
    parser.add_argument("--quant-scale", type=float, default=-1.0)
    parser.add_argument(
        "--snr-rate-mode",
        choices=["information", "coded", "custom"],
        default="information",
        help=(
            "AWGN normalization. 'information' uses K/transmitted_bits, while "
            "'coded' uses R=1 coded-bit BPSK SNR, matching common hardware-paper plots."
        ),
    )
    parser.add_argument("--channel-rate", type=float, default=1.0, help="custom AWGN rate used with --snr-rate-mode custom")
    parser.add_argument("--ebn0-start", type=float, default=0.0)
    parser.add_argument("--ebn0-stop", type=float, default=1.0)
    parser.add_argument("--ebn0-step", type=float, default=0.25)
    parser.add_argument("--min-errors", type=int, default=100)
    parser.add_argument("--max-frames", type=int, default=100)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--jobs", type=int, default=1, help="parallel worker processes for Monte Carlo frames")
    parser.add_argument("--progress-interval", type=int, default=5)
    parser.add_argument("--out-dir", default=os.path.join(os.path.dirname(__file__), "..", "data"))
    parser.add_argument(
        "--from-bram-dir",
        default=None,
        help=(
            "decode one deterministic frame from RTL BRAM hex files in this directory, "
            "save unfolded inputs/output LLRs, and compare against available RTL/reference outputs"
        ),
    )
    args = parser.parse_args()
    if args.from_bram_dir:
        run_bram_frame(args)
    else:
        run_sweep(args)


if __name__ == "__main__":
    main()
