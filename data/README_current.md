# Current K=3200/N=2 RTL Vector Set

This directory is intentionally kept small after the workspace cleanup.

## Active Target

- `K=3200`
- `NUM_SISO=2`, `SEG_LEN=1600`
- LTE QPP: `f1=111`, `f2=240`
- QPP LUT file: `qpp_3200.hex`
- LTE tail enabled: `TAIL_LEN=3`
- Paper boundary mode enabled
- Half iterations: `11`
- Fixed-point mode: 5-bit channel/apriori LLR, 6-bit extrinsic, 10-bit state metrics
- Channel/code rate used for this saved frame: `R=0.375`
- `Eb/N0=1.0 dB`, `seed=57`

## Files Kept Here

- `sys_*_ram.hex`, `par*_ram.hex`: RTL BRAM input LLRs.
- `initial_extrinsic.hex`: zero initial extrinsic memory image.
- `qpp_3200.hex`: active interleaver LUT for the RTL target.
- `true_info_bits.txt`, `tail_bits.txt`, `vector_metadata.txt`: transmitted bits and vector metadata.
- `ld_ram_output.hex`, `rtl_final_intrinsic.txt`, `rtl_final_hard_bits.txt`: RTL output artifacts from `tb/tb_turbo_decoder.v`.
- `ref_final_*`: `scripts/turbo_ref_model.py` deterministic reference outputs.
- `radix4_final_*`: `scripts/windowed_parallel_ber.py --decoder radix4` RTL-bit-accurate reference outputs.
- `windowed_input_*`: unfolded signed LLR inputs saved by the radix-4 deterministic BRAM comparison.
- `rtl_ber_results.txt`, `ber_results.txt`, `windowed_bram_compare_results.txt`: current BER and mismatch summaries.

## Current Result

- Channel hard BER: `544/3200 = 0.170000`
- RTL final intrinsic `L_D` BER: `1/3200 = 0.0003125`
- RTL vs `turbo_ref_model.py`: `0` final intrinsic mismatches, `0` hard-bit mismatches, `0` final extrinsic mismatches.
- RTL vs `windowed_parallel_ber.py --decoder radix4`: `0` final intrinsic mismatches, `0` hard-bit mismatches, `0` final extrinsic mismatches.

Historical plots, sweeps, old K=6144/QPP files, VCD/VVP builds, and saved run bundles were moved to `archive/cleanup_20260429/`.
