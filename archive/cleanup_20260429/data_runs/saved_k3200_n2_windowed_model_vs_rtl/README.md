# K=3200 N=2 Windowed Model vs RTL Deterministic Comparison

This bundle captures the Phase 5 deterministic comparison between:

- `scripts/windowed_parallel_ber.py` in BRAM-vector mode
- the RTL simulation outputs in `rtl_final_intrinsic.txt`
- the exact RTL mirror in `scripts/turbo_ref_model.py`

Configuration:

- K=3200
- NUM_SISO=2
- Segment length=1600
- QPP f1=111, f2=240
- LTE tail enabled, 3 trellis steps
- Window size=30
- Half iterations=11
- Quantized fixed-point mode enabled
- Boundary mode=paper
- Input vector seed=42
- Eb/N0=1.0 dB, code rate=1/3

Command used:

```powershell
python scripts\windowed_parallel_ber.py --decoder windowed --K 3200 --num-siso 2 --window-size 30 --half-iters 11 --quantize --boundary-mode paper --from-bram-dir data
```

Result summary:

- Channel hard-decision BER from quantized systematic LLRs: `577/3200 = 0.180312`
- `windowed_parallel_ber.py` final L_D BER: `105/3200 = 0.032813`
- RTL final L_D BER: `90/3200 = 0.028125`
- `turbo_ref_model.py` final L_D BER: `90/3200 = 0.028125`
- RTL vs `windowed_parallel_ber.py` hard-bit mismatches: `35/3200`
- RTL vs `turbo_ref_model.py` hard-bit mismatches: `0/3200`

Interpretation:

`windowed_parallel_ber.py` is the paper-style algorithm model. The new BRAM-vector mode proves it can consume the exact same fixed-point LTE input LLRs used by RTL and produces saved BER/output files for inspection. It is not the radix-4 RTL bridge, so exact intrinsic-value equality is not expected. `turbo_ref_model.py` remains the bit-order and fixed-point reference for exact RTL regression.

Primary files:

- `windowed_bram_compare_results.txt`
- `windowed_input_sys1.txt`, `windowed_input_par1.txt`, `windowed_input_sys2.txt`, `windowed_input_par2.txt`
- `windowed_final_intrinsic.txt`, `windowed_final_hard_bits.txt`
- `rtl_final_intrinsic.txt`, `rtl_final_hard_bits.txt`
- `ref_final_intrinsic.txt`, `ref_final_hard_bits.txt`
- `ber_results.txt`
