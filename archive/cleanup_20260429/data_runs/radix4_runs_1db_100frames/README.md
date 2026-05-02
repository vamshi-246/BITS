# Radix-4 RTL-Equivalent BER Run, 1.0 dB, 100 Frames

Command:

```powershell
python scripts\windowed_parallel_ber.py --decoder radix4 --K 3200 --num-siso 2 --window-size 30 --half-iters 11 --quantize --boundary-mode paper --ebn0-start 1.0 --ebn0-stop 1.0 --ebn0-step 1.0 --max-frames 100 --min-errors 100000000 --jobs 8 --progress-interval 10 --out-dir data\radix4_runs_1db_100frames
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
- Eb/N0=1.0 dB
- Frames: 100
- Bits: 320000

Result:

| Eb/N0 dB | Errors | Bits | Frames | BER |
|---:|---:|---:|---:|---:|
| 1.00 | 3240 | 320000 | 100 | 0.010125 |

File:

- `ber_curve_K3200_radix4_N2_fixed_paper_tail_information.csv`
