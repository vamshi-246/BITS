# RTL Gap Analysis: Python Windowed Parallel Turbo Decoder vs Current RTL

## Purpose

This document records the status of the Verilog RTL against the Python models
used for the K=3200 windowed Max-Log-MAP work. It now treats the final hardware
constraint as **two SISO cores**, not the paper's eight-core throughput point.

The short version:

- The core fixed-point BCJR math is already close to the Python/RTL reference model.
- The active RTL target is `K=3200`, `N=2`, `S=1600`, `M=30`, 11 half-iterations, LTE tail enabled, paper boundary mode enabled, and QPP `f1=111`, `f2=240`.
- The active QPP ROM is `data/qpp_3200.hex`; the old `qpp_6144` data has been archived.
- `scripts/windowed_parallel_ber.py --decoder radix4` is the RTL-bit-accurate BER model. The scalar `--decoder windowed` mode remains useful for algorithm experiments.
- N=8 routing is now an optional throughput extension, not the current implementation target.

## Current RTL Baseline

Main files:

- `rtl/turbo_decoder.v`
- `rtl/bcjr_core.v`
- `rtl/forward_recursion_unit.v`
- `rtl/backward_recursion_unit.v`
- `rtl/dummy_backward_recursion_unit.v`
- `rtl/llr_compute.v`
- `rtl/qpp_lut.v`
- `rtl/master_net.v`
- `rtl/slave_net.v`
- `rtl/input_bram.v`
- `rtl/extrinsic_bram.v`
- `rtl/ld_bram.v`

Current architectural constants:

| Item | Current RTL |
|---|---:|
| Code-block length | `K=3200` |
| Parallel SISO cores | `N=2` |
| Segment length | `S=1600` |
| Window size | `M=30` trellis steps |
| Radix-4 cycles per window | `15` |
| Windows per core | `54` for `(1600 + 3 tail)` |
| Half-iterations | `11` by default |
| QPP parameters | `f1=111`, `f2=240` |
| QPP storage | `data/qpp_3200.hex`, 1600 entries |
| Input memory format | even/odd folded 2-column words |
| Explicit LTE tail in trellis | yes, 3 steps on the last core |
| Boundary mode | paper-style dummy forward and cross-segment DBR |

Current RTL matches both `scripts/turbo_ref_model.py` and
`scripts/windowed_parallel_ber.py --decoder radix4` for the active deterministic
K=3200/N=2 vector set.

## Python Target Model

The current algorithm authority for BER behavior is
`scripts/windowed_parallel_ber.py`.

Important settings for the active RTL-equivalent runs:

| Item | Python target |
|---|---:|
| Code-block length | `K=3200` |
| Parallel SISO cores | `N=2` |
| Segment length | `S=1600` |
| Window size | `M=30` trellis steps |
| Windows per core | `ceil((1600+3)/30)=54`; last core includes 3 tail steps |
| Half-iterations | `11` |
| QPP parameters | `f1=111`, `f2=240` |
| Quantization | 5-bit channel/a-priori, 6-bit extrinsic, 10-bit metrics |
| Extrinsic scaling | `Le - (Le >> 2) - (Le >> 4)` = `0.6875 * Le` |
| Tail mode | enabled by default |
| Boundary mode | `paper` by default |

The Python model has two boundary modes:

- `boundary_mode=rtl`: legacy approximation mode.
- `boundary_mode=paper`: uses dummy forward warm-up for nonzero SISO cores and dummy backward windows that may read across segment boundaries.

## What Already Matches

These parts should be preserved unless a later bit-accurate comparison finds a defect:

1. **Radix-4 Max-Log trellis math**
   - The RTL uses radix-2 branch metrics, composes radix-4 metrics, and uses 8-state ACS units.

2. **Modulo-normalized state metric comparison**
   - `acs_r4.v`, recursion units, and `llr_compute.v` use modulo-style comparison with 10-bit state metrics.

3. **Fixed-point widths**
   - 5-bit channel/a-priori LLRs.
   - 6-bit extrinsic LLRs.
   - 10-bit alpha/beta/intrinsic state metrics.
   - `NEG_INF=-256`.

4. **Extrinsic scaling**
   - `rtl/llr_compute.v` implements `0.6875` scaling as `Le - (Le >>> 2) - (Le >>> 4)`.
   - Python fixed-point model uses the same shift approximation.

5. **Natural/interleaved half-iteration exchange**
   - The current N=2 top-level stores extrinsics in natural order after natural half-iterations and deinterleaves interleaved outputs back to natural order.

6. **Final intrinsic capture in testbench**
   - `tb/tb_turbo_decoder.v` captures final intrinsic decision LLRs internally for BER. This is the correct BER signal.

## Major Gaps

### 1. Completed: RTL Retargeted from K=6144 to K=3200/N=2

The original RTL contained many constants tied to `K=6144`, `N=2`:

- `FRAME_LEN = 3072` in `rtl/turbo_decoder.v`.
- Two explicit `bcjr_core` instances.
- `NUM_WINDOWS=103`.
- 1536-row memories.
- Two-column folded BRAM words.
- `qpp_lut.v` used to load `data/qpp_6144.hex`.
- `master_net.v` derives only the second SISO address using `+/-3072`.
- Testbenches expect 6144 intrinsic outputs and 1536 LD rows.

The active implementation now uses the two-core hardware constraint with the
paper code-block length:

- `rtl/turbo_decoder.v` defaults to `K=3200`, `NUM_SISO=2`, `SEG_LEN=1600`.
- `rtl/bcjr_core.v` defaults to `NUM_WINDOWS=54`.
- `rtl/qpp_lut.v` defaults to `data/qpp_3200.hex`.
- `tb/tb_turbo_decoder.v` defaults to the same K=3200/N=2/tail/paper setup.

The old K=6144/QPP files were moved to `archive/cleanup_20260429/`.

Remaining note: `NUM_SISO=2` is intentionally retained. A generated N-way
top-level is not part of the current target.

### 2. Completed: Explicit LTE Tail Trellis for the Current Target

The Python BER model processes 3 tail trellis steps for the last SISO segment.
BER is counted only over the original K information bits.

The current RTL and vector generator now load and process those 3 tail trellis
steps on the last SISO core.

Implemented behavior:

- Separate **decision length** from **trellis length**:
  - `INFO_SEG_LEN = K / NUM_SISO`
  - `TRELLIS_SEG_LEN = INFO_SEG_LEN` for non-last cores
  - `TRELLIS_SEG_LEN = INFO_SEG_LEN + 3` for the last core
- Continue suppressing output for non-information tail positions.
- Keep the known terminal beta condition on the last core's last window.
- Load tail systematic/parity pairs for:
  - constituent encoder 1 during natural half-iterations;
  - constituent encoder 2 during interleaved half-iterations.
- Ensure the final odd/even tail pair is zero-padded correctly. For the current
  `K=3200`, `N=2` target, the last core has local trellis positions `0..1602`;
  position `1603` is padding.

Important RTL implication:

`bcjr_core` currently uses one `frame_len` both to decide whether an input pair is out of range and whether to emit LLR output. Tail support needs two lengths:

- input/recursion valid length: includes tail;
- output valid length: excludes tail.

### 3. Completed: Paper Boundary Mode for N=2

The Python `paper` boundary mode does two things:

1. Nonzero SISO cores run a dummy forward window before their first real segment window.
2. Dummy backward recursion may read one window beyond the local segment boundary into the next SISO segment.

The current RTL supports this mode for the N=2 folded architecture:

- Add a real dummy-forward prologue for nonzero cores.
- Do not write alpha/gamma during dummy forward.
- After dummy forward, start each nonzero core's real `W1` with the warmed alpha metrics.
- Allow DBR windows at the end of a non-last segment to fetch the next segment's channel/a-priori LLRs instead of zero padding.
- Use the K=3200/N=2 deterministic regression to compare final intrinsic,
  hard-bit, and extrinsic outputs against Python.

### 4. Optional Extension: Replace N=2 Interleaver Network for N=8

The current interleaver network is deliberately specialized for `N=2`.

Current N=2 behavior:

- `qpp_lut.v` stores only `pi(k)` for SISO 0 local addresses.
- `master_net.v` derives the SISO 1 address from `pi(k) +/- 3072`.
- `slave_net.v` is a 2-input switch controlled by one permutation bit.

For a future `K=3200`, `N=8` throughput build, this is not sufficient. It is
not required for the current two-core hardware target.

Required RTL changes:

- Generate or compute the 8 interleaved addresses for each local trellis address:
  - `pi(local + j*S)` for `j=0..7`.
- Map the 8 addresses into 8 memory banks/columns with no conflict.
- Generate a permutation vector, not a single `perm_bit`.
- Replace `slave_net` with an 8-way inverse routing network.
- Do this for all simultaneous FR/BR/DBR even/odd accesses.
- Do the same routing for interleaved write-back.

Practical implementation choices:

1. **Functional/simple first:** direct combinational 8-address generation plus small sort/crossbar.
2. **Paper-style:** master-slave Batcher network as in the paper.
3. **QPP-specialized:** exploit the QPP maximum-contention-free property. For `K=3200`, `S=400`, `f1=111`, `f2=240`, addresses across the eight cores differ by multiples of 400 modulo 3200, so the bank mapping can be derived from one base QPP value plus a rotation.

For confidence and maintainability, implement the simple functional network first, then optimize if timing/area requires it.

### 5. Memory Organization Is Still Two-Column

Current memories are built around two SISO columns:

- input words are `{core1[4:0], core0[4:0]}`.
- extrinsic words are `{core1[5:0], core0[5:0]}`.
- even and odd trellis positions are split into separate RAMs.

For N=8:

- input memory words need 8 columns of 5-bit LLRs, or 8 banked memories;
- extrinsic memory words need 8 columns of 6-bit LLRs, or 8 banked memories;
- read ports still need to serve FR, BR, and DBR simultaneously;
- write-back must update all 8 columns each radix-4 output cycle, routed through the interleaver network during interleaved half-iterations.

Required RTL changes:

- Parameterize `input_bram.v`, `extrinsic_bram.v`, and `ld_bram.v` by `NUM_SISO`.
- Expand `load_data` width or change the load interface to stream one core column at a time.
- Expand `load_bram_sel` or add a `load_core_sel` field if per-column loading is used.
- Rework the LD/output memory so it can expose final decision data for all 8 cores.

### 6. Output Interface Currently Exposes Final Extrinsics, Not BER Decisions

`rtl/turbo_decoder.v` copies final extrinsic BRAM contents into `ld_bram`. The testbench separately captures final intrinsic `L_D` from internal wires during the last half-iteration.

For an RTL decoder intended to produce decoded bits or reliable BER numbers, the production output should expose one of:

- final intrinsic LLRs `L_D`;
- final hard decisions;
- both final intrinsic and final extrinsic.

Required RTL changes:

- Add a final decision RAM or output stream for `llr_intr_even/odd`.
- Suppress tail positions from the output.
- Update testbench and comparison scripts to use this explicit output instead of internal hierarchical probes.

### 7. Completed: Current Test Vector and BER Pipeline

The active test vector scripts and testbench now target the current
K=3200/N=2/tail/paper setup:

- `scripts/gen_encoded_test_vectors.py` defaults to `K=3200`, `NUM_SISO=2`,
  `f1=111`, `f2=240`, `R=0.375`, and tail rows enabled.
- `scripts/qpp_lut_gen.py` emits `data/qpp_3200.hex` by default.
- `tb/tb_turbo_decoder.v` expects 3200 information outputs and calculates BER
  from final intrinsic `L_D` hard decisions.
- `scripts/turbo_ref_model.py` mirrors the folded N=2 RTL.
- `scripts/windowed_parallel_ber.py --decoder radix4 --from-bram-dir data`
  consumes the same BRAM files and saves RTL-bit-accurate outputs.

Current active deterministic frame:

- `Eb/N0=1.0 dB`, `R=0.375`, `seed=57`.
- Channel hard BER: `544/3200 = 0.170000`.
- RTL final intrinsic BER: `1/3200 = 0.0003125`.
- RTL vs `turbo_ref_model.py`: 0 final intrinsic, hard-bit, and final
  extrinsic mismatches.
- RTL vs `windowed_parallel_ber.py --decoder radix4`: 0 final intrinsic,
  hard-bit, and final extrinsic mismatches.

## Completed RTL Change Order

### Phase 1: Make the Existing RTL Configurable Without Changing Behavior

Original goal: preserve the old K=6144/N=2 regression while removing hardcoded
constants.

1. Parameterize constants in `turbo_decoder.v`, BRAMs, and testbench.
2. Keep `NUM_SISO=2` while making K configurable.
3. Confirm the existing RTL still matches `scripts/turbo_ref_model.py`.

This phase is complete; active defaults have since moved to K=3200.

### Phase 2: Add Explicit Tail Support

Goal: support `K+3` trellis processing for the final core while outputting only K decisions.

1. Add separate recursion length and output length to `bcjr_core`.
2. Extend input memory depth for the tail rows.
3. Load natural and interleaved tail pairs.
4. Add tests around the last window and padding pair.

This is the most important BER-correctness change after parameterization.

### Phase 3: Add Paper Boundary Mode

Goal: make RTL match `scripts/windowed_parallel_ber.py --boundary-mode paper`.

1. Implement nonzero-core dummy forward warm-up.
2. Allow cross-segment DBR reads for non-last cores.
3. Stop relying on only core 0's address/valid when cores are in different warm-up states.
4. Compare boundary-window alpha/beta/extrinsic values against Python.

### Phase 4: Retarget Paper-Style RTL to K=3200/N=2

Goal: keep the final hardware constraint of two SISO cores while moving to the
paper code-block length and QPP.

1. Generate `qpp_3200.hex` for `K=3200`, `f1=111`, `f2=240`.
2. Generate deterministic `K=3200`, `N=2`, tail-enabled BRAM vectors.
3. Compile RTL with `TEST_K=3200`, `TEST_TAIL_LEN=3`, and
   `TEST_PAPER_BOUNDARY=1`.
4. Use ping-pong extrinsic memories so interleaved half-iterations read the
   previous half-iteration while writing the next one.
5. Validate bit-accurate RTL vs Python for a deterministic frame.

The N=8 network remains the paper throughput architecture, but it is no longer
the RTL implementation target under the two-SISO-core constraint.

### Phase 5: Add RTL-Level BER Workflow

Goal: use RTL as a credible hardware model for selected BER spot checks.

1. Run Python Monte Carlo for full sweeps with `scripts/windowed_parallel_ber.py`.
2. Generate deterministic `K=3200`, `N=2`, tail-enabled BRAM frames at selected
   SNRs/seeds.
3. Run the same BRAM frame through RTL and through `scripts/turbo_ref_model.py`
   for bit-exact hardware regression.
4. Run `scripts/windowed_parallel_ber.py --decoder radix4 --from-bram-dir ...`
   on the same BRAM frame to save RTL-bit-accurate inputs, outputs, and BER.
5. Use `--decoder windowed` only when the scalar paper-style algorithm model is
   specifically wanted. Use `--decoder radix4` for RTL-equivalent BER sweeps.

## High-Risk Areas

1. **Interleaved write-back under N=8**
   - The current N=2 stall detector is not enough for 8 columns/banks.
   - Read-after-write hazards must be handled per bank/row.

2. **Tail plus radix-4 pairing**
   - Tail length is 3, so the last pair includes one real tail trellis step and one zero-padded trellis step.
   - Recursion must use tail metrics, but output must suppress all tail decisions.

3. **Cross-segment DBR reads**
   - Paper boundary mode needs DBR to read beyond the local segment for non-last cores.
   - Current OOR logic converts those addresses to zero.

4. **Core lockstep assumptions**
   - Current top-level writes `{core1, core0}` using core 0's valid/address.
   - Dummy-forward warm-up and any future N=8 routing weaken that assumption.

5. **QPP address width and bank derivation**
   - K=3200 fits in 12 bits; current modules default to `PI_W=12`.
   - Any future K=6144 regression needs the wider legacy address settings restored explicitly.

## Minimal Definition of Done

The RTL should not be considered aligned with the current two-SISO target until
all of these pass:

1. Deterministic K=3200/N=2/tail frame generated from Python.
2. RTL final hard bits match `scripts/turbo_ref_model.py`.
3. RTL final intrinsic LLRs match `scripts/turbo_ref_model.py`.
4. Final extrinsics match `scripts/turbo_ref_model.py` or have documented,
   bounded differences caused only by accepted pipeline/tie-breaking behavior.
5. Boundary-window traces match for:
   - core 1 first real window after dummy forward;
   - last window of a non-last core using cross-segment DBR;
   - last core tail window using known terminal beta.
6. `scripts/windowed_parallel_ber.py --decoder radix4 --from-bram-dir ...`
   saves the same input LLRs and matches RTL final intrinsic, hard bits, and
   final extrinsics.
7. Any hard-bit difference between the scalar `--decoder windowed` model and
   RTL is treated as an algorithm-model versus radix-4-implementation
   difference, not an RTL regression failure.

## Bottom Line

The current RTL is now a K=3200/N=2 fixed-point, LTE-tail-aware prototype under
the final two-SISO-core constraint. It is not the paper-throughput K=3200/N=8
architecture. The datapath math should mostly be retained. The remaining work
should concentrate on:

1. broadening deterministic spot checks across SNR/seed points,
2. using `scripts/windowed_parallel_ber.py --decoder radix4` for RTL-equivalent
   BER sweeps and plots,
3. adding a cleaner non-testbench final intrinsic/hard-decision output interface,
4. keeping the optional N=8 memory/interleaver routing as a separate throughput
   extension, not the current implementation target.

Those changes will make the RTL BER workflow credible while preserving the
two-core hardware constraint.
