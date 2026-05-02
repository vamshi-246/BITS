# Turbo Decoder Debug Notes

This note records what was changed during the BER/debug pass and how the final
results were produced.

## Current Status After Cleanup

The active RTL/testbench target is now:

- `K=3200`
- `NUM_SISO=2`
- `SEG_LEN=1600`
- QPP `f1=111`, `f2=240`
- QPP ROM `data/qpp_3200.hex`
- LTE tail enabled with 3 tail trellis steps on the last core
- paper boundary mode enabled
- 11 half-iterations

The active deterministic vector set in `data/` is `R=0.375`, `Eb/N0=1.0 dB`,
`seed=57`. Current measured results:

```text
Channel hard decisions: 544/3200 = 0.170000
RTL final L_D:            1/3200 = 0.0003125
RTL vs turbo_ref_model.py intrinsic/hard/extrinsic mismatches: 0
RTL vs windowed_parallel_ber.py --decoder radix4 mismatches:   0
```

Historical K=6144 vectors, plots, VCD/VVP builds, old saved runs, and the old
`qpp_6144` data were moved to `archive/cleanup_20260429/`.

Use these current commands for a fresh deterministic check:

```powershell
python scripts\qpp_lut_gen.py --K 3200
python scripts\gen_encoded_test_vectors.py --K 3200 --num-siso 2 --ebn0-db 1.0 --channel-rate 0.375 --seed 57
iverilog -g2012 -o tb\tb_turbo_decoder_clean_default.vvp tb\tb_turbo_decoder.v rtl\*.v
vvp tb\tb_turbo_decoder_clean_default.vvp
python scripts\turbo_ref_model.py
python scripts\windowed_parallel_ber.py --decoder radix4 --K 3200 --num-siso 2 --window-size 30 --half-iters 11 --quantize --boundary-mode paper --from-bram-dir data
```

The older notes below are retained as debug history. Treat any K=6144 result
below this point as historical, not the current repository default.

## Algorithm Interpreted From The Paper

The implementation was aligned to the LTE parallel turbo decoder architecture
described by Studer et al.:

- 6144-bit LTE block, split across 2 SISO cores.
- Segment length per SISO: 3072 bits.
- Radix-4 Max-Log M-BCJR.
- Window length: 30 trellis steps, processed as 15 radix-4 pairs.
- Dummy backward recursion initializes beta for each window.
- QPP interleaver exchange between natural and interleaved half-iterations.
- Input and a-priori LLRs are 5-bit signed.
- Extrinsics are 6-bit signed.
- State metrics are 10-bit signed.
- Extrinsic scale factor is 0.6875, implemented as `x - (x >>> 2) - (x >>> 4)`.

## Main RTL Fixes

### Branch Metric Mapping

Files:

- `rtl/bm_radix2.v`
- `rtl/bm_radix4.v`
- `rtl/llr_compute.v`

Fixes:

- Corrected the radix-2 branch metric PRE index mapping for the LTE trellis.
- Corrected radix-4 pair order to use even address first, then odd address.
- Updated LLR path grouping so xs=0 and xs=1 paths match the same trellis used
  by the branch metric generator.

### LLR/Extrinsic Alignment

File:

- `rtl/llr_compute.v`

Fixes:

- During backward recursion, the sys/apr input registers are not aligned with
  the stored gamma memory entry being decoded.
- The extrinsic subtraction term `L_s + L_A` is now derived from stored branch
  metrics for the exact decoded step:
  - `PRE[0] = (L_s + L_A) + L_p`
  - `PRE[1] = (L_s + L_A) - L_p`
  - `L_s + L_A = (PRE[0] + PRE[1]) / 2`
- Added registered intrinsic decision outputs `llr_intr_even` and
  `llr_intr_odd` so the testbench can compute BER from actual RTL final
  decision LLRs.

### Alpha Memory Timing

File:

- `rtl/forward_recursion_unit.v`

Fix:

- The alpha memory must store the alpha metrics before the radix-4 transition
  corresponding to the stored gamma entry. The write path now registers and
  writes those pre-transition metrics so backward LLR computation receives the
  correct alpha/gamma pair.

### Core Scheduling

File:

- `rtl/bcjr_core.v`

Fixes:

- The two SISO cores are kept cycle-aligned so the top-level folded BRAM write
  can use one shared output address/valid.
- Core 0 starts from the known initial state.
- Core 1 starts from equal alpha metrics, matching the current lockstep
  sub-block approximation.
- Added registered final intrinsic outputs at the core boundary.

### Top-Level Interleaved APR Handling

File:

- `rtl/turbo_decoder.v`

Fixes:

- Added parameter `NUM_HALF_ITER`.
- In interleaved half-iterations, out-of-range padded symbols in the final
  partial window now get zero a-priori LLRs. Previously, their sys/par values
  were zeroed, but their a-priori values could be read through clamped QPP
  address 0 and pollute final-window beta recursion.
- Exposed per-core final intrinsic wires for the simulation testbench.

### Testbench Updates

Files:

- `tb/tb_turbo_decoder.v`
- `tb/tb_bcjr_core.v`

Fixes:

- VCD dumping is opt-in with `+dump_vcd`.
- The number of expected done pulses follows `TEST_NUM_HALF_ITER`.
- LD RAM dump samples after nonblocking read updates.
- Data file paths are relative to the repository root.
- The full-system testbench now captures 6144 final RTL intrinsic decision LLRs
  during the last half-iteration and writes:
  - `data/rtl_final_intrinsic.txt`
  - `data/rtl_final_hard_bits.txt`

## Python Tools Added Or Updated

### `scripts/gen_encoded_test_vectors.py`

Purpose:

- Generate reproducible encoded/channel test vectors for the RTL BRAMs.

Important fixes:

- Saves original transmitted bits to `data/true_info_bits.txt`.
- Forces generated information bits to terminate both constituent encoders in
  state zero, matching the RTL known-terminal-beta assumption.
- Corrected parity-2 BRAM layout. `par2` is already in interleaved encoder time,
  so it is written directly as `L_p2[2r]`, `L_p2[2r+S]`, etc.

Current vector settings:

- `K = 6144`
- Random seed: 42
- `Eb/N0 = 1.0 dB`
- Code rate used for AWGN generation: 1/3

### `scripts/turbo_ref_model.py`

Purpose:

- Fixed-point reference model matching the RTL architecture.
- Computes BER from original bits versus final intrinsic decision LLR.
- Compares RTL final extrinsics against reference final extrinsics.
- Compares RTL final intrinsic decision LLRs against reference final intrinsic
  LLRs when `data/rtl_final_intrinsic.txt` exists.

Output files:

- `data/ref_final_extrinsic.hex`
- `data/ref_final_intrinsic.txt`
- `data/ref_final_hard_bits.txt`
- `data/rtl_final_hard_bits.txt`
- `data/ber_results.txt`

### `scripts/ber_input_vs_output.py`

Status:

- Deprecated.
- It used to compare noisy input hard decisions against decoder output, which is
  not a valid BER measurement.
- It now delegates to `turbo_ref_model.py`.

## Final Verification Commands

From the repository root:

```powershell
iverilog -g2012 -o tb\tb_turbo_decoder_check.vvp tb\tb_turbo_decoder.v rtl\*.v
vvp tb\tb_turbo_decoder_check.vvp
python scripts\turbo_ref_model.py --show-mismatches 5
```

## Final Results

These historical results are for the old `K=6144`, seed 42, `Eb/N0=1.0 dB` vector set:

```text
Channel hard decisions: 1160/6144 = 0.188802
Python reference final L_D: 99/6144 = 0.016113
RTL final L_D: 99/6144 = 0.016113

RTL-vs-reference final intrinsic mismatches: 0/6144
RTL-vs-reference final hard-bit mismatches: 0/6144
RTL-vs-reference final extrinsic value mismatches: 0/6144
RTL-vs-reference final extrinsic sign mismatches: 0/6144
```

The RTL BER above is computed over the whole `K=6144` block as:

```text
original transmitted bit != hard_decision(RTL final intrinsic L_D)
```

with hard decision rule:

```text
L_D < 0  -> bit 1
L_D >= 0 -> bit 0
```

## Why These Results Do Not Yet Reproduce Fig. 9 In The Paper

The current RTL/reference regression is useful for finding implementation bugs,
but it is not yet an apples-to-apples reproduction of the paper's BER curve.

Important differences:

- Fig. 9 in the paper reports a Monte Carlo BER curve in AWGN for code-block
  length `K = 3200`; the old regression used one deterministic `K = 6144`
  block at one SNR point.
- The paper's M-BCJR description uses a dummy forward recursion to initialize
  forward state metrics for arbitrary trellis starts. The current two-core RTL
  keeps both cores in lockstep and initializes nonzero cores with equal alpha
  metrics, which is an approximation made to preserve one shared folded BRAM
  write address/valid.
- The current vector generator does not append explicit LTE tail bits. Instead,
  it flips a small number of information-bit positions so both constituent
  encoders naturally end in state 0. This is valid for testing the RTL's known
  terminal-state assumption, but it is not the same as a full LTE tail-bit
  conformance simulation.
- The input LLR quantization scale is fixed in the generator (`15/4.0`) rather
  than swept or optimized against the paper's implementation. With the current
  `Eb/N0 = 1.0 dB` vectors, roughly 10% of the 5-bit channel LLRs saturate.

To reproduce the paper-level BER curve, the next debug stage should compare:

1. Floating-point full-block BCJR.
2. Fixed-point full-block Max-Log BCJR.
3. Fixed-point windowed `M = 30` BCJR with one SISO.
4. Fixed-point parallel M-BCJR with proper dummy-forward boundary
   initialization.
5. The current RTL architecture.

That sequence will separate algorithm loss, quantization loss, windowing loss,
parallel-boundary loss, and RTL implementation bugs.

## Full-Block Floating-Point Baseline Added

Added `scripts/full_block_float_turbo.py`.

This simulator removes the current RTL architectural approximations:

- no SISO segmentation;
- no `M = 30` windowing;
- no fixed-point state metrics;
- optional exact `logmap` or `maxlog` recursion;
- optional extrinsic/a-priori clipping.

The first tests use the same current 5-bit quantized BRAM LLR files, not a new
paper-style `K = 3200` Monte Carlo setup.

Commands run:

```powershell
python scripts\full_block_float_turbo.py --mode maxlog
python scripts\full_block_float_turbo.py --mode logmap
python scripts\full_block_float_turbo.py --mode maxlog --extrinsic-clip-bits 6 --apr-clip-bits 5
python scripts\full_block_float_turbo.py --mode logmap --extrinsic-clip-bits 6 --apr-clip-bits 5
```

Historical results on the old `K = 6144` block:

```text
Channel hard decisions: 1160/6144 = 0.188802
RTL/fixed-point windowed parallel final L_D: 99/6144 = 0.016113

Full-block Max-Log, no clipping:         281/6144 = 0.045736
Full-block Log-MAP, no clipping:         269/6144 = 0.043783
Full-block Max-Log, extr6/apr5 clipping:  61/6144 = 0.009928
Full-block Log-MAP, extr6/apr5 clipping:  63/6144 = 0.010254
```

The clipped full-block floating model is now the best baseline on this same
received frame. The remaining gap from 61 errors to the RTL's 99 errors is the
loss to investigate next: windowing, segmentation, fixed-point state metrics,
and current parallel-boundary initialization.

## Phase 3 Paper-Boundary RTL Mode Added

Added an optional `PAPER_BOUNDARY` mode to the N=2 RTL path:

- `bcjr_core.v` now supports a one-window prologue. Core 1 runs dummy forward
  over core 0's last window; core 0 idles for the same slot to keep folded
  write-back aligned.
- `turbo_decoder.v` routes core 1 dummy-forward reads through column 0 and core
  0 cross-segment DBR reads through column 1.
- The three LTE tail samples are shadowed in the top level so the last core can
  still receive tail DBR samples when the shared DBR BRAM port is serving a
  core 0 cross-segment read.
- `tb_turbo_decoder.v` exposes `TEST_PAPER_BOUNDARY` and can emit an optional
  `+trace_boundary` debug trace.
- `scripts/turbo_ref_model.py` now accepts `--boundary-mode rtl|paper`, matching
  the RTL regression mode.

Phase 3 deterministic regression:

```powershell
iverilog -g2012 -P tb_turbo_decoder.TEST_TAIL_LEN=3 -P tb_turbo_decoder.TEST_PAPER_BOUNDARY=1 -o tb\tb_turbo_decoder_paper_check.vvp tb\tb_turbo_decoder.v rtl\*.v
vvp tb\tb_turbo_decoder_paper_check.vvp
python scripts\turbo_ref_model.py --tail-len 3 --boundary-mode paper --show-mismatches 10
```

Result:

```text
Python reference final L_D BER: 157/6144 = 0.025553
RTL final L_D BER:             157/6144 = 0.025553
Intrinsic mismatches:          0/6144
Hard-bit mismatches:           0/6144
Final-extrinsic mismatches:    0/6144
```

Saved artifacts:

```text
data/saved_n2_tail_paper_boundary_regression/
```

## Revised Phase 4: K=3200 With Two SISO Cores

The hardware constraint is now two SISO cores, so Phase 4 was redirected from
an `N=8` RTL network replacement to a paper-code-block, two-core target:

- `K=3200`
- `NUM_SISO=2`
- `SEG_LEN=1600`
- QPP `f1=111`, `f2=240`
- LTE tail enabled
- paper boundary mode enabled

Changes made:

- `scripts/qpp_lut_gen.py` emits
  `data/qpp_3200.hex`.
- `scripts/gen_encoded_test_vectors.py` now accepts `--K`, `--num-siso`, and
  `--ebn0-db` for deterministic N=2 BRAM vectors.
- `scripts/turbo_ref_model.py` now accepts `--K` and `--num-siso` while still
  mirroring the folded N=2 RTL architecture.
- `tb/tb_turbo_decoder.v` exposes `TEST_QPP_LUT_FILE`.
- `rtl/turbo_decoder.v` now ping-pongs the extrinsic BRAM by half-iteration.
  This is required for K=3200 because an in-place interleaved half-iteration can
  overwrite a-priori rows before later DBR/FR reads consume them.

Regression commands:

```powershell
python scripts\qpp_lut_gen.py --K 3200
python scripts\gen_encoded_test_vectors.py --K 3200 --num-siso 2 --include-tail --seed 42
iverilog -g2012 -P tb_turbo_decoder.TEST_K=3200 -P tb_turbo_decoder.TEST_TAIL_LEN=3 -P tb_turbo_decoder.TEST_PAPER_BOUNDARY=1 -P tb_turbo_decoder.TEST_QPP_LUT_FILE='"data/qpp_3200.hex"' -o tb\tb_turbo_decoder_k3200_paper_check.vvp tb\tb_turbo_decoder.v rtl\*.v
vvp tb\tb_turbo_decoder_k3200_paper_check.vvp
python scripts\turbo_ref_model.py --K 3200 --num-siso 2 --tail-len 3 --boundary-mode paper --show-mismatches 10
```

Result:

```text
Channel hard decisions:        577/3200 = 0.180312
Python reference final L_D:     90/3200 = 0.028125
RTL final L_D:                  90/3200 = 0.028125
Intrinsic mismatches:            0/3200
Hard-bit mismatches:             0/3200
Final-extrinsic mismatches:      0/3200
```

Saved artifacts:

```text
data/saved_k3200_n2_tail_paper_boundary_regression/
```

## Phase 5: Deterministic Paper-Model vs RTL BER Bridge

Added a deterministic BRAM-vector mode to `scripts/windowed_parallel_ber.py`.
This mode keeps the Monte Carlo sweep path intact, but also allows the
paper-style algorithm model to consume the exact same fixed-point input LLR
BRAM files used by the RTL testbench.

New command:

```powershell
python scripts\windowed_parallel_ber.py --decoder windowed --K 3200 --num-siso 2 --window-size 30 --half-iters 11 --quantize --boundary-mode paper --from-bram-dir data
```

Files now written by the deterministic mode:

```text
data/windowed_input_sys1.txt
data/windowed_input_par1.txt
data/windowed_input_sys2.txt
data/windowed_input_par2.txt
data/windowed_final_intrinsic.txt
data/windowed_final_hard_bits.txt
data/windowed_bram_compare_results.txt
```

Result on the active `K=3200`, `N=2`, tail-enabled, paper-boundary vector:

```text
Channel hard decisions from sys1:      577/3200 = 0.180312
windowed_parallel_ber.py final L_D:    105/3200 = 0.032813
RTL final L_D:                          90/3200 = 0.028125
turbo_ref_model.py final L_D:            90/3200 = 0.028125
RTL vs windowed model hard mismatches:   35/3200
RTL vs turbo_ref_model hard mismatches:   0/3200
```

Interpretation:

- `windowed_parallel_ber.py` is still the paper-style algorithm/BER model. The
  new mode proves it is using the same deterministic LTE input LLRs as the RTL
  for spot checks and saves the model outputs for inspection.
- `turbo_ref_model.py` remains the exact RTL fixed-point/radix-4 bridge. It is
  the reference for bit-exact RTL regressions.
- The `35/3200` hard-bit delta is documented rather than hidden. It comes from
  the fact that `windowed_parallel_ber.py` is a scalar trellis model and does
  not yet duplicate every RTL radix-4 ordering/truncation/tie-break detail.

Saved artifacts:

```text
data/saved_k3200_n2_windowed_model_vs_rtl/
```

## Phase 5 Update: Radix-4 Bit-Accurate Mode Added

Added `--decoder radix4` to `scripts/windowed_parallel_ber.py`. This path ports
the fixed-point radix-4 RTL mirror from `scripts/turbo_ref_model.py` into the
main BER script, so BER sweeps can be run directly from the script that also
contains the paper-style scalar model.

This mode is intentionally limited to the current RTL target:

- `NUM_SISO=2`
- quantized fixed-point mode only (`--quantize`)
- radix-4, two trellis steps per ACS/LLR cycle
- paper/rtl boundary selection retained

Deterministic RTL-match command:

```powershell
python scripts\windowed_parallel_ber.py --decoder radix4 --K 3200 --num-siso 2 --window-size 30 --half-iters 11 --quantize --boundary-mode paper --from-bram-dir data
```

Result on the active `K=3200`, `N=2`, tail-enabled, paper-boundary vector:

```text
Channel hard decisions from sys1:       577/3200 = 0.180312
windowed_parallel_ber.py radix4 L_D:     90/3200 = 0.028125
RTL intrinsic mismatches:                 0/3200
RTL hard-bit mismatches:                  0/3200
RTL final-extrinsic mismatches:           0/3200
turbo_ref_model intrinsic mismatches:     0/3200
turbo_ref_model final-extrinsic mismatch: 0/3200
```

The previous `35/3200` mismatch is eliminated by using `--decoder radix4`.
The older `--decoder windowed` mode is still useful as the scalar paper-style
algorithm model, but it is no longer the mode to use for RTL-bit-accurate BER.

Smoke BER sweep command:

```powershell
python scripts\windowed_parallel_ber.py --decoder radix4 --K 3200 --num-siso 2 --window-size 30 --half-iters 11 --quantize --boundary-mode paper --ebn0-start 1.0 --ebn0-stop 1.0 --ebn0-step 1.0 --max-frames 1 --min-errors 100 --jobs 1 --out-dir data\radix4_smoke
```

Smoke result:

```text
Eb/N0=1.00 dB: BER=6.250000e-04 (2/3200, frames=1)
```

The plot script now accepts `--label` and `--title`, so the N=2/radix-4 curve
does not inherit the old hardcoded N=8 title.

Saved artifacts:

```text
data/saved_k3200_n2_radix4_bitaccurate_regression/
```
