# Radix-4 RTL-Equivalent BER Sweep, 30 Frames Per SNR

Command:

```powershell
python scripts\windowed_parallel_ber.py --decoder radix4 --K 3200 --num-siso 2 --window-size 30 --half-iters 11 --quantize --boundary-mode paper --ebn0-start 0.0 --ebn0-stop 1.0 --ebn0-step 0.25 --max-frames 30 --min-errors 100000000 --jobs 8 --progress-interval 5 --out-dir data\radix4_runs_30frames
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
- SNR normalization: `information`
- Frames per SNR: 30
- Bits per SNR: 96000

Results:

| Eb/N0 dB | Errors | Bits | Frames | BER |
|---:|---:|---:|---:|---:|
| 0.00 | 16082 | 96000 | 30 | 0.1675208333 |
| 0.25 | 13827 | 96000 | 30 | 0.1440312500 |
| 0.50 | 8947 | 96000 | 30 | 0.0931979167 |
| 0.75 | 4954 | 96000 | 30 | 0.0516041667 |
| 1.00 | 791 | 96000 | 30 | 0.0082395833 |

Files:

- `ber_curve_K3200_radix4_N2_fixed_paper_tail_information.csv`
- `ber_plot_K3200_radix4_N2_fixed_paper_tail_information_30frames.png`
