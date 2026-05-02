# High-Grade Radix-4 RTL-Equivalent BER Sweep, R=0.375

Command:

```powershell
python scripts\windowed_parallel_ber.py --decoder radix4 --K 3200 --num-siso 2 --window-size 30 --half-iters 11 --quantize --boundary-mode paper --snr-rate-mode custom --channel-rate 0.375 --ebn0-start 0.0 --ebn0-stop 1.0 --ebn0-step 0.25 --max-frames 100 --min-errors 500 --jobs 8 --progress-interval 10 --out-dir data\radix4_runs_R0375_highgrade_100framecap
```

Configuration:

- RTL-equivalent model: `scripts/windowed_parallel_ber.py --decoder radix4`
- K=3200
- NUM_SISO=2
- Window size=30
- Half iterations=11
- Quantized fixed-point mode
- LTE tail enabled
- Boundary mode=paper
- SNR normalization: `custom`
- Channel rate: 0.375
- Eb/N0 points: 0.00, 0.25, 0.50, 0.75, 1.00 dB
- Stop rule: at least 500 errors or 100 frames per point

Results:

| Eb/N0 dB | Channel rate | Errors | Bits | Frames | BER |
|---:|---:|---:|---:|---:|---:|
| 0.00 | 0.375 | 2745 | 28800 | 9 | 0.0953125000 |
| 0.25 | 0.375 | 1947 | 32000 | 10 | 0.0608437500 |
| 0.50 | 0.375 | 653 | 86400 | 27 | 0.0075578704 |
| 0.75 | 0.375 | 573 | 300800 | 94 | 0.0019049202 |
| 1.00 | 0.375 | 68 | 320000 | 100 | 0.0002125000 |

Files:

- `ber_curve_K3200_radix4_N2_fixed_paper_tail_R0.375.csv`
- `ber_plot_K3200_radix4_N2_fixed_paper_tail_R0375_highgrade_100framecap.png`
