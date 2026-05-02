# K=3200 N=2 Radix-4 Bit-Accurate Regression

This bundle captures the new `scripts/windowed_parallel_ber.py --decoder radix4`
mode.

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

Deterministic BRAM command:

```powershell
python scripts\windowed_parallel_ber.py --decoder radix4 --K 3200 --num-siso 2 --window-size 30 --half-iters 11 --quantize --boundary-mode paper --from-bram-dir data
```

Deterministic result:

- Channel hard-decision BER: `577/3200 = 0.180312`
- `windowed_parallel_ber.py --decoder radix4` final L_D BER: `90/3200 = 0.028125`
- RTL final intrinsic mismatches: `0/3200`
- RTL hard-bit mismatches: `0/3200`
- RTL final extrinsic mismatches: `0/3200`
- `turbo_ref_model.py` final intrinsic mismatches: `0/3200`
- `turbo_ref_model.py` final extrinsic mismatches: `0/3200`

BER sweep smoke command:

```powershell
python scripts\windowed_parallel_ber.py --decoder radix4 --K 3200 --num-siso 2 --window-size 30 --half-iters 11 --quantize --boundary-mode paper --ebn0-start 1.0 --ebn0-stop 1.0 --ebn0-step 1.0 --max-frames 1 --min-errors 100 --jobs 1 --out-dir data\radix4_smoke
```

Smoke result:

- `1.00 dB`: `2/3200 = 6.25e-4`

Plot command:

```powershell
python scripts\plot_windowed_parallel_ber.py --csv data\radix4_smoke\ber_curve_K3200_radix4_N2_fixed_paper_tail_information.csv --out data\radix4_smoke\ber_plot_K3200_radix4_N2_fixed_paper_tail_information_smoke.png --label "RTL-bit-accurate radix-4 N=2 model (I=5.5)" --title "K=3200, N=2, M=30, Radix-4 Fixed-Point, 5.5 iterations"
```

Primary files:

- `windowed_bram_compare_results.txt`
- `radix4_final_intrinsic.txt`
- `radix4_final_hard_bits.txt`
- `radix4_final_extrinsic.txt`
- `rtl_final_intrinsic.txt`
- `rtl_final_hard_bits.txt`
- `ld_ram_output.hex`
- `ber_curve_K3200_radix4_N2_fixed_paper_tail_information.csv`
- `ber_plot_K3200_radix4_N2_fixed_paper_tail_information_smoke.png`
