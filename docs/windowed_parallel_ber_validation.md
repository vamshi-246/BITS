# Windowed Parallel BER Validation

## Current Status After RTL Retarget

The active hardware-matched use of `scripts/windowed_parallel_ber.py` is now
the radix-4 fixed-point path:

```powershell
python scripts\windowed_parallel_ber.py --decoder radix4 --K 3200 --num-siso 2 --window-size 30 --half-iters 11 --quantize --boundary-mode paper --from-bram-dir data
```

Current target:

- `K=3200`, `NUM_SISO=2`, `SEG_LEN=1600`
- QPP `f1=111`, `f2=240`
- LTE tail enabled
- paper boundary mode enabled
- 5-bit channel/a-priori LLR, 6-bit extrinsic, 10-bit metrics
- extrinsic scaling `Le - (Le >> 2) - (Le >> 4)`

The active deterministic BRAM frame in `data/` is `R=0.375`, `Eb/N0=1.0 dB`,
`seed=57`. It gives:

```text
Channel hard decisions from sys1: 544/3200 = 0.170000
windowed_parallel_ber.py --decoder radix4 final L_D: 1/3200 = 0.000313
RTL final intrinsic mismatches: 0/3200
RTL hard-bit mismatches: 0/3200
RTL final-extrinsic mismatches: 0/3200
```

The older N=8 `--decoder windowed` sweeps below are retained as algorithm
validation history. They are not the current two-SISO RTL target.

## Original Python Schedule Assessment

`scripts/turbo_ref_model.py` originally matched the RTL-style window schedule:

- segment length `S = K / NUM_SISO`;
- fixed `M = 30` trellis steps per window, always run as 15 radix-4 cycles;
- FR writes alpha/gamma for window `m`;
- BR decodes window `m - 1`;
- DBR runs one window ahead and supplies beta initialization for BR;
- the final partial window is zero-padded for recursion, while padded outputs are suppressed;
- core 0 uses known initial alpha, nonzero cores use equal-alpha initialization in the legacy RTL approximation;
- only the last core's last BR window uses known terminal beta.

The archived `bits_behav_simulation.py` root script should not be used as the
BER authority. It is an older debug dump generator and is stale relative to the
RTL/reference model: it only covers core 0, ignores iterative a-priori inputs,
stores alpha after the ACS update, and uses older radix-4 branch-metric ordering.

## Added Model

Added `scripts/windowed_parallel_ber.py`.

The new script supports:

- `--decoder full` for a full-block Max-Log baseline;
- `--decoder windowed` for M-BCJR windowing;
- `--num-siso` for SISO segmentation;
- `--boundary-mode paper` for dummy-forward/dummy-backward boundary windows;
- `--boundary-mode rtl` for the current lockstep RTL boundary approximation;
- `--quantize` for 5-bit channel/a-priori, 6-bit extrinsic, and 10-bit state-metric behavior;
- `--no-tail` for legacy experiments that drop explicit LTE tail trellis steps.
- `--snr-rate-mode` to make AWGN normalization explicit:
  - `information` uses `R = K / transmitted_bits` and is the conventional information-bit Eb/N0 simulation;
  - `coded` uses `R = 1`, equivalent to coded-bit BPSK SNR;
  - `custom` uses `--channel-rate`.

For the paper's Fig. 9 code-block length, the script uses `K=3200`, `f1=111`, `f2=240`, matching 3GPP TS 36.212 Table 5.1.3-3.

Extrinsic scaling is enabled by default. In the fixed-point path, the decoder uses the hardware-friendly approximation
`scaled = Le - (Le >> 2) - (Le >> 4)`, i.e. `0.6875 * Le`.

## Smoke Validation Run

Command:

```powershell
python scripts\windowed_parallel_ber.py --decoder windowed --K 3200 --num-siso 8 --half-iters 11 --quantize --boundary-mode paper --ebn0-start 1.0 --ebn0-stop 1.5 --ebn0-step 0.25 --min-errors 50 --max-frames 3 --seed 7 --progress-interval 1
```

Result file:

`data/ber_curve_K3200_windowed_N8_fixed_paper_tail.csv`

Observed short-run BER:

| Eb/N0 dB | BER | Errors / bits | Frames |
|---:|---:|---:|---:|
| 1.00 | 5.000e-3 | 48 / 9600 | 3 |
| 1.25 | 2.083e-4 | 2 / 9600 | 3 |
| 1.50 | 0 | 0 / 9600 | 3 |

This is a smoke validation, not a publication-quality Monte Carlo curve. For final BER credibility, run each point until at least 100 to 200 errors or a much higher frame cap is reached.

## Paper-Range Plot Run

Command:

```powershell
python scripts\windowed_parallel_ber.py --decoder windowed --K 3200 --num-siso 8 --half-iters 11 --quantize --boundary-mode paper --ebn0-start 0.0 --ebn0-stop 1.0 --ebn0-step 0.25 --min-errors 100 --max-frames 20 --seed 11 --progress-interval 1
python scripts\plot_windowed_parallel_ber.py
```

Generated files:

- `data/ber_curve_K3200_windowed_N8_fixed_paper_tail.csv`
- `data/ber_plot_K3200_windowed_N8_fixed_paper_style.png`

Observed BER:

| Eb/N0 dB | BER | Errors / bits | Frames |
|---:|---:|---:|---:|
| 0.00 | 1.778125e-1 | 569 / 3200 | 1 |
| 0.25 | 1.206250e-1 | 386 / 3200 | 1 |
| 0.50 | 1.140625e-1 | 365 / 3200 | 1 |
| 0.75 | 2.916667e-2 | 280 / 9600 | 3 |
| 1.00 | 9.375000e-3 | 120 / 12800 | 4 |

This information-rate run does not match Fig. 9; the curve is shifted too far right. The mismatch is not caused by missing extrinsic scaling.

## Paper-Normalized Plot Run

The Fig. 9 screenshot is consistent with a slightly different AWGN normalization than the strict information-rate run above. A channel-rate value of `0.375` aligns the low-SNR hardware points while keeping the setting explicit in the command and output filename.

Command:

```powershell
python scripts\windowed_parallel_ber.py --decoder windowed --K 3200 --num-siso 8 --window-size 30 --half-iters 11 --quantize --boundary-mode paper --snr-rate-mode custom --channel-rate 0.375 --ebn0-start 0 --ebn0-stop 1 --ebn0-step 0.25 --min-errors 100 --max-frames 5 --seed 12345 --progress-interval 1
python scripts\plot_windowed_parallel_ber.py --csv data\ber_curve_K3200_windowed_N8_fixed_paper_tail_R0.375.csv --out data\ber_plot_K3200_windowed_N8_fixed_paper_tail_R0.375.png
```

Generated files:

- `data/ber_curve_K3200_windowed_N8_fixed_paper_tail_R0.375.csv`
- `data/ber_plot_K3200_windowed_N8_fixed_paper_tail_R0.375.png`

Observed BER:

| Eb/N0 dB | BER | Errors / bits | Frames |
|---:|---:|---:|---:|
| 0.00 | 8.375000e-2 | 268 / 3200 | 1 |
| 0.25 | 7.906250e-2 | 253 / 3200 | 1 |
| 0.50 | 1.343750e-2 | 172 / 12800 | 4 |
| 0.75 | 2.500000e-4 | 4 / 16000 | 5 |
| 1.00 | 0 | 0 / 16000 | 5 |

The `1.00 dB` point is an observed-zero point, not a measured zero BER. The plot places it at the standard half-error upper-bound marker (`0.5 / bits`) for visibility. A paper-grade tail point needs many more frames because Fig. 9 is already near or below `1e-4` there.

## Capped Multi-Frame Plot Run

This run caps every SNR point at 50 frames, while allowing low-SNR points to stop earlier after collecting at least 200 bit errors. It uses the same model settings as the paper-normalized run.

Command:

```powershell
python scripts\windowed_parallel_ber.py --decoder windowed --K 3200 --num-siso 8 --window-size 30 --half-iters 11 --quantize --boundary-mode paper --snr-rate-mode custom --channel-rate 0.375 --ebn0-start 0 --ebn0-stop 1 --ebn0-step 0.25 --min-errors 200 --max-frames 50 --jobs 10 --seed 20260429 --progress-interval 10
python scripts\plot_windowed_parallel_ber.py --csv data\ber_curve_K3200_windowed_N8_fixed_paper_tail_R0.375.csv --out data\ber_plot_K3200_windowed_N8_fixed_paper_tail_R0.375_50frame_cap.png
```

Generated files:

- `data/ber_curve_K3200_windowed_N8_fixed_paper_tail_R0.375.csv`
- `data/ber_plot_K3200_windowed_N8_fixed_paper_tail_R0.375_50frame_cap.png`
- `paper_grade_windowed_ber_R0.375_50frame_cap.png`

Observed BER:

| Eb/N0 dB | BER | Errors / bits | Frames |
|---:|---:|---:|---:|
| 0.00 | 9.865625e-2 | 3157 / 32000 | 10 |
| 0.25 | 5.822917e-2 | 2236 / 38400 | 12 |
| 0.50 | 8.327206e-3 | 453 / 54400 | 17 |
| 0.75 | 7.250000e-4 | 116 / 160000 | 50 |
| 1.00 | 3.750000e-4 | 60 / 160000 | 50 |
