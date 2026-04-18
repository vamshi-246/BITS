# Implementation Report — Parallel Radix-4 Max-Log M-BCJR LTE Turbo Decoder

**Reference:** Studer et al., "ASIC Implementation of Soft-Input Soft-Output MIMO Detection Using MMSE Parallel Interference Cancellation", IEEE JSSC 2011  
**Target:** Zynq-7010 FPGA / ASIC  
**HDL:** Verilog-2001  
**Current Config:** 2 parallel SISO modules, block length 6144, window size W=30

---

## 1. System-Level Architecture

The decoder splits the LTE codeword (up to 6144 bits) equally across **NUM_SISO** parallel BCJR (SISO) cores. Each core independently processes its segment using the sliding-window Max-Log MAP algorithm and outputs extrinsic LLRs.

| Parameter          | Value | Rationale                                               |
|--------------------|-------|---------------------------------------------------------|
| NUM_SISO           | 2     | Demonstrates parallelism; scalable to 8                 |
| Segment per core   | 3072  | 6144 / 2 = 3072 trellis steps each                     |
| Window size (W)    | 30    | 30 trellis steps = 15 radix-4 cycles per window        |
| NUM_WINDOWS        | 103   | ceil(3072 / 30) = 103 (102 full + 1 partial of 12)     |
| Trellis states     | 8     | LTE 3GPP constraint-length 4 convolutional code        |

**Partial-window handling:** Every window always runs the full 15 R4 cycles. For the last partial window, out-of-range memory addresses return zero LLRs (handled by external controller), and output addresses ≥ `frame_len` are filtered out (`llr_out_valid` suppressed inside `bcjr_core`). This eliminates the synchronization bug where FR, BR, and DBR—running on different windows in parallel—would be desynchronized by a shared shortened window counter.

---

## 2. Module Hierarchy

```
bcjr_core (top-level FSM + datapath)
├── forward_recursion_unit (FR)
│   ├── bm_preproc         — 4 BPSK-substituted PRE values (odd + even)
│   ├── bm_radix2          — 16 R2 branch metrics per step (odd + even)
│   ├── bm_radix4          — 32 R4 branch metrics from R2 pair
│   └── acs_r4 × 8         — Radix-4 ACS with modulo-normalized compare
├── backward_recursion_unit (BR)
│   ├── bm_radix4          — Reconstructs R4 BMs from stored R2 gammas
│   └── acs_r4 × 8         — Backward (transposed) ACS wiring
├── dummy_backward_recursion_unit (DBR)
│   ├── bm_preproc
│   ├── bm_radix2
│   ├── bm_radix4
│   └── acs_r4 × 8         — Backward ACS, all-zero init
├── llr_compute             — 1-stage pipelined LLR from α, β, γ
├── alpha_mem × 2           — Double-buffered, 15-deep × 8×10-bit
└── gamma_mem × 2           — Double-buffered, 15-deep × 32×7-bit
```

**Total ACS instances per core:** 8 (FR) + 8 (BR) + 8 (DBR) = **24 acs_r4 units**.

---

## 3. Bit-Width Budget

| Signal              | Width  | Range          | Rationale                                                    |
|----------------------|--------|----------------|--------------------------------------------------------------|
| Input LLRs (sys, par, apr) | 5-bit signed | [-16, +15]     | Quantized channel LLRs                                      |
| PRE (preprocessed)   | 7-bit signed | [-64, +63]     | Max |L_s + L_A + L_p| = 16+15+15 = 46, fits in 7-bit       |
| BM_R2 (radix-2 BM)  | 7-bit signed | [-64, +63]     | Direct selection from PRE — same width                       |
| BM_R4 (radix-4 BM)  | 8-bit signed | [-128, +127]   | Sum of two 7-bit R2 BMs: max = 2×63 = 126                   |
| State Metric (α, β)  | 10-bit signed | [-512, +511]   | Accumulates BMs; modulo arithmetic prevents overflow         |
| NEG_INF              | -256   | —              | Initial gap |0 − (−256)| = 256 < 512 = 2^(10−1) for modulo correctness |
| Extrinsic LLR out   | 6-bit signed | [-32, +31]     | Saturated, scaled by 0.6875                                  |
| LLR addresses       | 12-bit | [0, 4095]      | Supports segments up to 4096 trellis steps                   |
| Window indices       | 7-bit  | [0, 127]       | Supports up to 128 windows per core                          |

---

## 4. Branch Metric Pipeline (Combinational)

### 4.1 bm_preproc — BPSK-Substituted Preprocessing

Maps the BPSK bit hypotheses (x_s, x_p) into 4 pre-computed sums:

```
sa     = L_sys + L_apr                          (6-bit signed)
PRE[0] = +(sa) + L_par  =  L_s + L_A + L_p     (x_s=0, x_p=0)
PRE[1] = +(sa) - L_par  =  L_s + L_A - L_p     (x_s=0, x_p=1)
PRE[2] = -(sa) + L_par  = -L_s - L_A + L_p     (x_s=1, x_p=0)  = -PRE[1]
PRE[3] = -(sa) - L_par  = -L_s - L_A - L_p     (x_s=1, x_p=1)  = -PRE[0]
```

Computed for both **odd** and **even** trellis steps simultaneously (8 outputs total). The symmetry PRE[2]=−PRE[1], PRE[3]=−PRE[0] is exploited implicitly by the synthesis tool.

### 4.2 bm_radix2 — Radix-2 Branch Metric Selection

Pure wiring — selects from 4 PRE values to produce 16 R2 branch metrics per step, following the radix-2 predecessor table. Index mapping: `bm_r2[dest*2 + pred_idx]`, where each destination state has exactly 2 predecessors.

**Key table (odd step):**

| Dest | Pred_0 | PRE idx | Pred_1 | PRE idx |
|------|--------|---------|--------|---------|
| 000  | 000    | 0       | 001    | 3       |
| 001  | 010    | 1       | 011    | 2       |
| 010  | 100    | 2       | 101    | 1       |
| 011  | 110    | 3       | 111    | 0       |
| 100  | 000    | 3       | 001    | 0       |
| 101  | 010    | 2       | 011    | 1       |
| 110  | 100    | 1       | 101    | 2       |
| 111  | 110    | 0       | 111    | 3       |

Even step uses identical structure with `pre_even` values.

### 4.3 bm_radix4 — Radix-4 Branch Metric Composition

Combines two consecutive R2 steps into one R4 step: `bm_r4 = bm_r2_odd[s'→s'] + bm_r2_even[s'→s]`. This produces 32 R4 branch metrics (4 per destination state × 8 states). Each R4 BM is 8-bit signed.

---

## 5. Add-Compare-Select (acs_r4)

**One registered ACS unit per trellis state per recursion direction.** Each selects the maximum of 4 candidates using **modulo-normalized comparison**.

### 5.1 Candidate Computation
```
cand[i] = sm_in[i] + sign_extend(bm_r4[i])     (11-bit intermediate)
c[i]    = cand[i][9:0]                          (truncated to 10-bit — intentional modulo)
```

### 5.2 Modulo-Normalized Max
Six pairwise differences computed: `diff_ij = c[i] - c[j]`. Comparison rule:
```
c[i] > c[j]  iff  (diff_ij ≠ 0) AND (diff_ij[MSB] == 0)
```
This works correctly as long as the true difference is < 2^(SM_W−1) = 512, guaranteed by the NEG_INF = −256 initialization.

### 5.3 Winner Selection via 6-bit LUT
The 6 comparison bits `{cmp5..cmp0}` form a 6-bit key into a 24-entry LUT covering all valid orderings of 4 elements. The LUT outputs a 2-bit selector choosing the winner. Default maps to candidate 0.

### 5.4 Load-Init Mechanism
The `load_init` port takes priority over `enable`, allowing the FSM to inject custom initial state metrics without a reset. This is essential for parallel window processing where each core needs different starting conditions.

---

## 6. Forward Recursion Unit (FR)

Contains the full BM pipeline (preproc → R2 → R4) and 8 forward-direction ACS units.

### ACS Wiring (Forward)
Each destination state selects its 4 radix-4 predecessors:
- **Even-indexed states** (0,2,4,6): predecessors from states {0,1,2,3}
- **Odd-indexed states** (1,3,5,7): predecessors from states {4,5,6,7}

### Initialization (CORE_ID-dependent)
| CORE_ID | α(S0) | α(S1..S7) | Rationale                                    |
|---------|-------|-----------|----------------------------------------------|
| 0       | 0     | −256      | Known encoder start state (all-zero)         |
| Others  | 0     | 0         | Equal prob — dummy FR will converge to truth  |

### Memory Writes
On each active cycle (non-dummy): writes α values to `alpha_mem` and 32 R2 BMs to `gamma_mem` at address = `step_cnt`. During dummy forward pass, writes are suppressed (`alpha_wr_en = 0, gamma_wr_en = 0`).

---

## 7. Backward Recursion Unit (BR)

Processes one window in **reverse** trellis order. Does NOT contain its own BM pipeline — instead reconstructs R4 BMs from stored R2 gammas read from `gamma_mem`.

### ACS Wiring (Backward = Transposed Forward)
For backward β(s''), we collect all forward destination states s that have s'' as a predecessor:
- **s'' ∈ {0,1,2,3}**: successors β from states {0,2,4,6}, using `bm_r4[s*4 + pred_idx_of_s''_in_s]`
- **s'' ∈ {4,5,6,7}**: successors β from states {1,3,5,7}

### Beta Initialization
Loaded from `br_beta_init_reg` via `load_beta_init` pulse. Source:
- **Normal:** DBR's `final_beta` from the next window
- **Last core, last window:** Known terminal state β(S0)=0, β(others)=−256

### Pipeline
```
Cycle 0: load_beta_init → ACS loads init. Issue memory read at addr = win_len_r4 − 1.
Cycle 1: Wait for synchronous memory read (1-cycle latency).
Cycle 2: Memory data valid. Enable ACS. Output β + β_valid. Issue next read (addr−1).
...repeat, decrementing address to 0.
```

Each cycle outputs `{beta_out[0..7], cur_step, beta_valid}` to the LLR compute unit.

---

## 8. Dummy Backward Recursion Unit (DBR)

Traverses one window backward to generate **converged beta initial values** for BR. Contains its own full BM pipeline (preproc → R2 → R4). Does **not** access alpha/gamma memory. Does **not** trigger LLR computation.

### Initialization
All 8 beta state metrics initialized to **0** (equal probability) on the rising edge of `active`. The backward recursion converges toward the true state distribution over W=30 trellis steps.

### Output
After `win_len_r4` active cycles, captures `final_beta[0..7]` and asserts `window_done`. These values are stored in `br_beta_init_reg` inside `bcjr_core` for use by BR on the corresponding data window.

---

## 9. LLR Computation Unit (llr_compute)

**1-stage pipelined**, producing two extrinsic LLRs per cycle (odd and even trellis steps).

### Step 1–2: Derive Intermediate α and β
Since radix-4 processes two trellis steps at once, the LLR unit needs α_{k-1} and β_{k-1} at the odd trellis step. These are derived on-the-fly:
- **α_{k-1}(s'):** 8 parallel max-of-2 computations from α_{k-2} + R2_odd BMs (using radix-2 predecessor table)
- **β_{k-1}(s'):** 8 parallel max-of-2 computations from β_k + R2_even BMs (using radix-2 successor table)

### Step 3–4: Path Metric Trees
For each step (even and odd), 16 path metrics are computed (8 for xs=0, 8 for xs=1):
```
path(s', s) = α(s') + γ_R2(s', s) + β(s)
```

### Step 5: Max-of-8 Binary Trees
Two binary reduction trees per step compute:
```
max0 = max over all xs=0 paths    (3-level tree: 4→2→1)
max1 = max over all xs=1 paths
L_D  = max0 − max1
```
All max operations use modulo-normalized comparison (same function as ACS).

### Step 6: Extrinsic LLR
```
L_extr = L_D − (L_sys + L_apr)              — subtract intrinsic
L_scaled = L_extr × 0.6875                  — scaling factor
         = L_extr − (L_extr >>> 3) − (L_extr >>> 4)  — shift-add approximation
L_out = saturate(L_scaled, [-32, +31])       — clamp to 6-bit signed
```

Output registered with 1-cycle latency, gated by `beta_valid`.

---

## 10. Memory Architecture

### 10.1 Alpha Memory (alpha_mem)
- **Structure:** Register-array, 15 entries × 8 states × 10 bits = **1200 bits per bank**
- **Banks:** 2 (double-buffered), selected by `fr_win_idx[0]` (write) and `br_win_idx[0]` (read)
- **Interface:** Synchronous write + synchronous read (1-cycle latency)
- **Purpose:** FR writes α forward; BR reads α backward for LLR computation

### 10.2 Gamma Memory (gamma_mem)
- **Structure:** Register-array, 15 entries × 32 BMs × 7 bits = **3360 bits per bank**
- **Banks:** 2 (double-buffered), same bank selection as alpha
- **Purpose:** FR writes 32 R2 BMs; BR reads them back to reconstruct R4 BMs + feed LLR compute

### Double-Buffering
Bank selection via LSB of window index ensures FR writes to one bank while BR reads from the other. When FR advances to the next window, the banks swap roles.

---

## 11. Top-Level FSM (bcjr_core)

### 11.1 6-State FSM

| State       | Function                                                            |
|-------------|---------------------------------------------------------------------|
| ST_IDLE     | Wait for `start`. Initialize window indices, load initial state metrics. |
| ST_LLR_REQ  | Assert `llr_req`. Compute FR/BR/DBR memory addresses. |
| ST_LLR_WAIT | Wait for `llr_valid` from external memory controller. Latch LLR inputs. |
| ST_COMPUTE  | Drive FR/BR/DBR with latched LLRs. Increment `step_cnt`. |
| ST_LLR_OUT  | BR backward pass active. Load beta_init, wait for `br_window_done`. |
| ST_WIN_DONE | Advance all window indices. Update activation flags. Check completion. |

### 11.2 Window Scheduling Pipeline

```
Slot 0:    FR=W1,  BR=idle, DBR=W2'     (CORE_ID=0 skips dummy FR)
Slot 1:    FR=W2,  BR=W1,   DBR=W3'
Slot 2:    FR=W3,  BR=W2,   DBR=W4'
...
Slot N-1:  FR=WN,  BR=W(N-1), DBR=W(N+1)'  (last core: DBR uses known β)
Slot N:    FR=idle, BR=WN,   DBR=idle
```

FR and DBR run **in parallel** during ST_LLR_REQ → ST_COMPUTE (the forward pass). BR runs afterwards during ST_LLR_OUT (the backward pass), reading from the alpha/gamma memory that FR wrote.

### 11.3 Address Generation

| Unit | Direction | Address Formula                                                |
|------|-----------|----------------------------------------------------------------|
| FR   | Forward   | `(fr_win_idx − 1) × 30 + step_cnt × 2`                       |
| BR   | Backward  | `(br_win_idx − 1) × 30 + (14 − step_cnt) × 2`                |
| DBR  | Backward  | `(dbr_win_idx − 1) × 30 + (14 − step_cnt) × 2`               |

Each address points to an odd-step LLR; the even-step is at `addr + 1`.

### 11.4 Activation Logic

| Unit | Active when                                                        | Rationale                                  |
|------|--------------------------------------------------------------------|--------------------------------------------|
| FR   | `fr_win_idx ≤ NUM_WINDOWS`                                        | Processes data windows 1..N                |
| BR   | `br_win_idx ≥ 1 && br_win_idx ≤ NUM_WINDOWS`                     | Lags FR by 1 slot                          |
| DBR  | `dbr_win_idx ≥ 2 && dbr_win_idx ≤ NUM_WINDOWS+1`                 | Runs 1 window ahead of BR                  |
|      | Exception: last core deactivates DBR at `dbr_win_idx ≥ NUM_WINDOWS` | Known terminal β — no DBR needed     |

### 11.5 CORE_ID-Specific Behaviour

| CORE_ID | Start Behaviour                           | End Behaviour                     |
|---------|-------------------------------------------|-----------------------------------|
| 0       | Skip dummy FR; FR starts at W1 directly. α(S0)=0, α(others)=−256. | Normal (DBR provides β for last window's BR) |
| Last    | Dummy FR runs on prev segment's last window. All α init to 0. | Last BR loads known terminal β(S0)=0, β(others)=−256. DBR suppressed. |

### 11.6 Output Filtering

The LLR output path computes: `llr_out_addr = (br_win_idx − 1) × 30 + llr_step_pipe × 2`. Before asserting `llr_out_valid`, the core checks:
```
if (llr_out_addr + 1 ≤ frame_len)  →  llr_out_valid = 1
else                                →  llr_out_valid = 0  (suppress padding outputs)
```
This handles the partial last window without requiring a shortened step counter — the architectural key to preventing the FR/BR/DBR desynchronization bug.

---

## 12. Verification Infrastructure

| Component              | File                    | Function                                     |
|------------------------|-------------------------|----------------------------------------------|
| Test data generator    | `generate_test_data.py` | Random LLRs + idealized Max-Log-MAP golden reference |
| Testbench              | `tb_bcjr_core.v`        | Loads `input_llr.hex`, drives DUT, captures `rtl_extrinsic.hex` |
| Comparison script      | `compare_results.py`    | Bit-level comparison of RTL vs golden hex     |
| Human-readable dump    | `input_llr_readable.txt`| Decimal table of all LLR inputs               |

**Testbench memory model:** `llr_mem[0:4095]` with 1-cycle read latency. Out-of-range addresses (≥3072) return zero — simulating the external controller's zero-padding behaviour for the partial last window.

---

## 13. Design Decisions Summary

| Decision                              | Rationale                                                              |
|---------------------------------------|------------------------------------------------------------------------|
| Radix-4 processing                    | Halves clock cycles vs radix-2; doubles throughput for same frequency  |
| Modulo-normalization (not subtraction) | Avoids costly subtraction across all states every cycle                |
| NEG_INF = −256                        | Ensures initial gap stays within modulo comparison range (< 512)       |
| BPSK substitution in bm_preproc       | Eliminates multipliers; branch metrics become ±sums of LLRs           |
| Store R2 BMs in gamma_mem (not R4)    | BR needs R2 BMs for LLR path metric computation, not just R4          |
| Double-buffered memories              | FR writes bank A while BR reads bank B; swap on window boundary       |
| Uniform window length (always 15 R4)  | Prevents BR/DBR desync when FR hits a partial last window              |
| Output address filtering              | Cleanly handles partial windows without FSM complexity                 |
| `load_init` in ACS (not `rst_n`)      | Allows per-window re-initialization without global reset               |
| 0.6875 scaling via shift-add          | Approximates LTE correction factor without a multiplier                |
