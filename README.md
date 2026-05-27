# Parallel Radix-4 LTE Turbo Decoder — FPGA Implementation

![Verilog](https://img.shields.io/badge/HDL-Verilog--2001-blue?style=flat-square)
![FPGA](https://img.shields.io/badge/FPGA-Zynq--7010-green?style=flat-square)
![Vivado](https://img.shields.io/badge/Tool-Vivado%202024.1-orange?style=flat-square)
![Clock](https://img.shields.io/badge/Clock-100%20MHz-brightgreen?style=flat-square)
![Status](https://img.shields.io/badge/Status-Verified%20%E2%9C%93-success?style=flat-square)

---

## Summary

A fully verified, FPGA-synthesizable **Parallel Radix-4 Max-Log M-BCJR LTE Turbo Decoder** implemented in Verilog-2001, targeting the Xilinx Zynq-7010 SoC. The decoder processes LTE-standard turbo-coded frames (K=3200 information bits) using **2 parallel SISO cores** with a **windowed sliding-window BCJR** schedule, achieving **bit-exact match** with the Python reference model (0/3200 mismatches) and meeting timing at **100 MHz** post-place-and-route. The design reduces channel BER from **17.0% down to 0.031%** — a coding gain exceeding **27 dB** — in a single 3200-bit frame at E_b/N_0 = 1.0 dB. The project includes a complete verification pipeline: bit-accurate Python reference models, Monte Carlo BER simulations, RTL testbenches, post-synthesis functional simulation, and on-FPGA hardware bring-up with ILA-based debug.

---

## Problem Statement

4G LTE wireless systems rely on **turbo codes** — a class of near-Shannon-limit error-correcting codes — to reliably transmit data over noisy wireless channels. The turbo decoder is the most computationally intensive block in the LTE receiver, requiring iterative probabilistic decoding using the BCJR (MAP) algorithm. The standard BCJR algorithm processes the entire trellis sequentially, creating a throughput bottleneck.

This project addresses the challenge of implementing a **high-throughput, area-efficient LTE turbo decoder** in hardware by:

1. Using **radix-4 trellis processing** to halve the number of clock cycles per trellis traversal
2. Deploying **parallel SISO cores** that process independent segments of the codeblock simultaneously
3. Employing a **windowed sliding-window schedule** with dummy backward recursion for memory-efficient beta state initialization
4. Implementing **fixed-point arithmetic** with carefully optimized bit widths to minimize area while preserving decoding performance

---

## Motivation

| Aspect | Relevance |
|--------|-----------|
| **Industry** | Turbo decoders are deployed in every 4G LTE base station and handset; efficient hardware implementations directly impact wireless infrastructure cost and power |
| **Research** | This project implements and validates key architectural ideas from Studer et al., IEEE JSSC 2011 — a foundational paper in high-throughput decoder design |
| **Engineering** | The design exercises every major VLSI/FPGA skill: FSM design, fixed-point DSP, BRAM memory architecture, multi-core synchronization, AXI integration, and hardware-software co-verification |
| **Academic** | Completed as part of the Digital VLSI & Architecture Design (DVAD) course at IIT Mandi, demonstrating graduate-level digital design competence |

---

## Key Features

- **Radix-4 Max-Log M-BCJR Algorithm** — Processes 2 trellis steps per clock cycle, halving latency compared to radix-2 implementations
- **2 Parallel SISO Cores** — Block is segmented into 2 equal segments (1600 bits each), decoded in parallel with QPP interleaver-based extrinsic exchange
- **Windowed Sliding-Window Schedule** — 30-step windows (15 radix-4 cycles) with overlapping Forward/Backward/Dummy-Backward recursion for memory-efficient operation
- **Paper Boundary Initialization** — Implements the Studer et al. cross-segment dummy-forward warm-up for accurate beta state initialization at segment boundaries
- **Proper LTE Tail Trellis** — 3-step tail termination following the 3GPP LTE specification, not a simplified flush
- **Bit-Exact RTL-to-Python Match** — 0/3200 mismatches across intrinsic LLRs, extrinsic LLRs, and hard-decision bits verified against two independent Python models
- **100 MHz Timing Closure** — All timing constraints met post-place-and-route on Zynq-7010 (WNS = +0.230 ns)
- **AXI4-Lite SoC Integration** — Complete IP wrapper with register map for Zynq PS ↔ PL communication
- **On-FPGA Hardware Bring-Up** — Verified on Zybo Z7-10 board with ILA-based waveform capture and automated comparison scripts
- **Complete Verification Stack** — Python reference model → Monte Carlo BER simulation → RTL testbench → Post-synthesis simulation → FPGA hardware validation

---

## Tech Stack / Tools Used

| Category | Technology |
|----------|------------|
| **HDL** | Verilog-2001 (IEEE 1364-2001) |
| **Target FPGA** | Xilinx Zynq-7010 (xc7z010clg400-1) on Digilent Zybo Z7-10 |
| **EDA Tools** | AMD Vivado 2024.1/2024.2 (Synthesis, Implementation, ILA Debug) |
| **Simulation** | Icarus Verilog (iverilog + vvp) for behavioral simulation |
| **Software IDE** | AMD Vitis 2024.x (bare-metal Zynq application) |
| **Reference Models** | Python 3 with NumPy (turbo encoder/decoder, BER analysis) |
| **Verification** | Custom Python ↔ Verilog test vector pipeline, Monte Carlo BER |
| **Waveform Tools** | WaveDrom (timing diagram generation) |
| **Version Control** | Git |
| **Board Interface** | UART (115200 baud), JTAG (ILA), GPIO (buttons/LEDs) |

---

## System Architecture / Workflow

```
                          ┌─────────────────────────────────────────────────────┐
                          │              turbo_decoder (Top Level)               │
                          │                                                     │
  Host / AXI  ───────►   │  ┌──────────┐    ┌──────────┐    ┌──────────┐       │
  Load 8 BRAMs           │  │ Group-A   │    │ Group-B   │    │ Group-C  │       │
  (sys, par1,             │  │ Input     │    │ Extrinsic │    │ LD Output│       │
   silv, par2)            │  │ BRAMs (8) │    │ BRAMs (4) │    │ BRAM (2) │       │
                          │  │ ×3 ports  │    │ ping-pong │    │          │       │
                          │  └─────┬─────┘    └─────┬─────┘    └─────┬────┘      │
                          │        │                │                │            │
                          │  ┌─────▼────────────────▼──────┐        │            │
                          │  │      Fetch Pipeline FSM      │        │            │
                          │  │  (F_IDLE→F_C1→F_C2→F_STALL) │        │            │
                          │  └─────┬───────────────┬───────┘        │            │
                          │        │               │                │            │
                          │  ┌─────▼─────┐   ┌─────▼─────┐         │            │
                          │  │ bcjr_core │   │ bcjr_core │         │            │
                          │  │  (Core 0)  │   │  (Core 1)  │         │            │
                          │  │            │   │            │         │            │
                          │  │ ┌────────┐ │   │ ┌────────┐ │         │            │
                          │  │ │   FR   │ │   │ │   FR   │ │         │            │
                          │  │ │(8 ACS) │ │   │ │(8 ACS) │ │         │            │
                          │  │ ├────────┤ │   │ ├────────┤ │         │            │
                          │  │ │   BR   │ │   │ │   BR   │ │         │            │
                          │  │ │(8 ACS) │ │   │ │(8 ACS) │ │         │            │
                          │  │ ├────────┤ │   │ ├────────┤ │         │            │
                          │  │ │  DBR   │ │   │ │  DBR   │ │         │            │
                          │  │ │(8 ACS) │ │   │ │(8 ACS) │ │         │            │
                          │  │ ├────────┤ │   │ ├────────┤ │         │            │
                          │  │ │  LLR   │ │   │ │  LLR   │ │         │            │
                          │  │ │Compute │ │   │ │Compute │ │         │            │
                          │  │ └────────┘ │   │ └────────┘ │         │            │
                          │  └─────┬──────┘   └──────┬─────┘         │            │
                          │        │                 │               │            │
                          │  ┌─────▼─────────────────▼──────┐        │            │
                          │  │   Write-Back + QPP Interleave │        │            │
                          │  │   (Master/Slave Networks ×12) ├───────►│            │
                          │  └───────────────────────────────┘        │            │
                          │                                           │   ───►  Hard
                          │  ┌──────────────┐                         │         Decision
                          │  │ QPP LUT ROM  │                         │         Output
                          │  │  (1600 × 12) │                         │            │
                          │  └──────────────┘                         │            │
                          └─────────────────────────────────────────────────────────┘
```

### Decoding Flow (Per Half-Iteration)

```
Start ──► Load Channel LLRs ──► For each window (54 windows/core):
              │
              ├──► Forward Recursion (FR)  ──► Stores α metrics + γ metrics
              ├──► Dummy Backward (DBR)    ──► Estimates β initialization
              └──► Backward Recursion (BR) ──► Reads α,γ → Produces extrinsic LLRs
              │
              └──► Write extrinsics to ping-pong BRAM (natural or QPP-interleaved)
              │
          ──► Repeat for 11 half-iterations (alternating natural/interleaved)
              │
          ──► Hard decision from final intrinsic L_D  ──► Output
```

---

## Implementation Details

### BCJR Core Architecture (`bcjr_core.v`)

Each SISO core implements a **6-state FSM** (`IDLE → LLR_REQ → LLR_WAIT → COMPUTE → LLR_OUT → WIN_DONE`) that orchestrates three concurrent recursion units within each window slot:

| Unit | Function | ACS Instances |
|------|----------|:------------:|
| **Forward Recursion (FR)** | Computes α state metrics, stores to double-buffered alpha/gamma memory | 8 |
| **Backward Recursion (BR)** | Reads stored α/γ, computes β metrics, produces extrinsic LLRs | 8 |
| **Dummy Backward (DBR)** | Runs on the *next* window to generate warm-up β initialization for BR | 8 |
| **Total per core** | | **24** |

### Radix-4 Processing

The radix-4 approach processes **2 trellis steps per clock cycle** by combining pairs of radix-2 branch metrics:

- **`bm_preproc.v`**: Preprocesses 5-bit input LLRs into 7-bit radix-2 branch metrics using the BPSK substitution trick (no multipliers needed)
- **`bm_radix2.v`**: Computes 16 radix-2 branch metrics (2 predecessors × 8 states) from preprocessed inputs
- **`bm_radix4.v`**: Combines radix-2 pairs into 32 radix-4 branch metrics (4 predecessors × 8 states) following the Section 3.4 predecessor table
- **`acs_r4.v`**: Radix-4 Add-Compare-Select with **modulo-normalized 6-comparator parallel comparison** and 24-entry LUT winner selection — avoids costly per-cycle subtraction normalization

### Fixed-Point Bit-Width Budget

| Signal | Width | Range | Rationale |
|--------|:-----:|-------|-----------|
| Channel/A-priori LLR | 5-bit | [-16, +15] | Input quantization matching SNR dynamic range |
| Preprocessed R2 BMs | 7-bit | [-64, +63] | Sum of two 5-bit + 1 sign-extended systematic |
| Radix-4 BMs | 8-bit | [-128, +127] | Sum of two 7-bit R2 BMs |
| State Metrics (α, β) | 10-bit | [-512, +511] | NEG_INF = −256 ensures modulo comparison correctness |
| Extrinsic LLR output | 6-bit | [-32, +31] | Scaled by 0.6875 via shift-add: `L - (L>>>3) - (L>>>4)` |
| Intrinsic L_D (decision) | 10-bit | Full precision | Used for final hard decision (sign bit) |

### Memory Architecture

- **8 Group-A Input BRAMs**: Triple-port (FR, BR, DBR simultaneous read) for systematic, parity1, interleaved-systematic, parity2 — each split into even/odd banks for radix-4 dual-column access
- **4 Group-B Extrinsic BRAMs**: Ping-pong double-buffered (2 banks × even/odd) for concurrent read of previous iteration's extrinsics while writing current iteration
- **2 Group-C LD BRAMs**: Final extrinsic output readback
- **Double-Buffered Alpha Memory**: 2 banks × 15 entries × 80 bits per core — FR writes to one bank while BR reads the other
- **Double-Buffered Gamma Memory**: 2 banks × 15 entries × 224 bits per core — stores radix-2 BMs (not R4) so BR can reconstruct individual-step LLRs

### QPP Interleaver

The **Quadratic Permutation Polynomial** interleaver follows the LTE standard with parameters f₁ = 111, f₂ = 240 for K = 3200. The permutation table is stored in a 6-read-port ROM (`qpp_lut.v`) initialized from `data/qpp_3200.hex` at synthesis time.

### Interleaver Network

- **6 Master Networks** (`master_net.v`): Convert QPP-permuted frame addresses into BRAM row addresses and permutation bits for the folded N=2 memory layout
- **6 Slave Networks** (`slave_net.v`): Route extrinsic data from the folded BRAM word to the correct core based on the permutation bit

### Window Scheduling

54 windows per core, each 30 trellis steps (15 radix-4 cycles). The schedule is pipelined:

| Slot | FR | BR | DBR |
|:----:|:--:|:--:|:---:|
| 0 (prologue) | Dummy warm-up | — | — |
| 1 | W1 | — | W2' |
| 2 | W2 | W1 | W3' |
| ... | ... | ... | ... |
| N | WN | W(N-1) | — |
| N+1 | — | WN | — |

---

## Results / Outputs

### Single-Frame Deterministic Verification (K=3200, E_b/N_0 = 1.0 dB)

| Metric | Value |
|--------|-------|
| Block length (K) | 3,200 information bits |
| Code rate | R = 0.375 (rate-1/3 with LTE tail) |
| Channel hard-decision BER (before decoding) | **544 / 3,200 = 17.00%** |
| Python reference decoded BER (after decoding) | **1 / 3,200 = 0.031%** |
| RTL simulation decoded BER | **1 / 3,200 = 0.031%** |
| RTL vs. Python intrinsic LLR mismatches | **0 / 3,200** |
| RTL vs. Python hard-bit mismatches | **0 / 3,200** |
| RTL vs. Python extrinsic value mismatches | **0 / 3,200** |
| RTL vs. Python max |difference| | **0** |
| Half-iterations | 11 (5.5 full iterations) |

> The decoder achieves **bit-exact match** with both Python reference models (`turbo_ref_model.py` and `windowed_parallel_ber.py --decoder radix4`), confirming complete correctness of the Verilog implementation.

### Monte Carlo BER Curve (Windowed Parallel Model, K=3200, R=0.375)

| E_b/N_0 (dB) | Decoded BER | Bit Errors | Total Bits | Frames |
|:---:|:---:|:---:|:---:|:---:|
| 0.00 | 1.08 × 10⁻¹ | 1,729 | 16,000 | 5 |
| 0.25 | 7.02 × 10⁻² | 1,123 | 16,000 | 5 |
| 0.50 | 5.19 × 10⁻³ | 166 | 32,000 | 10 |
| 0.75 | 2.88 × 10⁻³ | 92 | 32,000 | 10 |
| 1.00 | 8.44 × 10⁻⁴ | 27 | 32,000 | 10 |

### FPGA Resource Utilization (Zynq-7010, xc7z010clg400-1)

| Resource | Used | Available | Utilization |
|----------|:----:|:---------:|:-----------:|
| **Slice LUTs** | 15,477 | 17,600 | **87.9%** |
| LUT as Logic | 14,385 | 17,600 | 81.7% |
| LUT as Distributed RAM | 1,020 | 6,000 | 18.2% |
| **Slice Registers** | 8,190 | 35,200 | **23.3%** |
| **Block RAM Tiles** | 18.5 | 60 | **30.8%** |
| RAMB18E1 | 33 | 120 | 27.5% |
| RAMB36E1 | 2 | 60 | 3.3% |
| **DSP48E1 Slices** | 6 | 80 | **7.5%** |
| **CARRY4** | 3,051 | — | — |
| **Slices** | 4,397 | 4,400 | **99.9%** |

### Timing Summary (Post-Place-and-Route, 100 MHz)

| Metric | Value |
|--------|-------|
| Target Clock Period | 10.000 ns (100 MHz) |
| Worst Negative Slack (WNS) | **+0.230 ns** (MET) |
| Worst Hold Slack (WHS) | **+0.030 ns** (MET) |
| Total Negative Slack (TNS) | 0.000 ns |
| Timing Endpoints | 29,427 |
| **All timing constraints met** | ✅ |

### Hardware Bring-Up (Zybo Z7-10)

- FPGA on-board decode result: **1/3200 errors** (matches RTL simulation exactly)
- Hard-decision RAM readback vs RTL reference: **0/3200 mismatches**
- Verified via Vivado ILA waveform capture + automated Python comparison scripts

---

## Repository Structure

```
BITS_LTE_Parallel_Turbo_Decoder/
│
├── rtl/                              # Synthesizable Verilog RTL
│   ├── turbo_decoder.v               # Top-level: 2-core decoder, memory, interleaver
│   ├── bcjr_core.v                   # SISO core: FSM + FR/BR/DBR + LLR compute
│   ├── forward_recursion_unit.v      # Forward alpha recursion (8× acs_r4)
│   ├── backward_recursion_unit.v     # Backward beta recursion + LLR generation
│   ├── dummy_backward_recursion_unit.v  # Beta warm-up for next window
│   ├── llr_compute.v                 # Extrinsic LLR calculation from α, β, γ
│   ├── acs_r4.v                      # Radix-4 Add-Compare-Select with modulo norm
│   ├── bm_radix4.v                   # Radix-4 branch metric combiner (32 BMs)
│   ├── bm_radix2.v                   # Radix-2 branch metric calculator (16 BMs)
│   ├── bm_preproc.v                  # LLR preprocessing (BPSK substitution)
│   ├── alpha_mem.v                   # Double-buffered alpha state metric memory
│   ├── gamma_mem.v                   # Double-buffered gamma branch metric memory
│   ├── input_bram.v                  # Triple-read-port input BRAM wrapper
│   ├── extrinsic_bram.v              # Triple-read-port extrinsic BRAM wrapper
│   ├── ld_bram.v                     # Final extrinsic output BRAM
│   ├── qpp_lut.v                     # 6-port QPP interleaver ROM
│   ├── master_net.v                  # Interleaver address → BRAM row mapper
│   ├── slave_net.v                   # Extrinsic data routing network
│   ├── turbo_decoder_axi_lite.v      # AXI4-Lite IP wrapper for Zynq integration
│   └── turbo_decoder_button_bringup.v  # PL-only bringup top (ROM vectors, GPIO)
│
├── tb/                               # Testbenches
│   ├── tb_turbo_decoder.v            # Full-system testbench: load→decode→BER check
│   ├── tb_bcjr_core.v               # Unit testbench for single BCJR core
│   ├── tb_button_bringup.v          # Button bringup testbench
│   ├── tb_dbr_standalone.v          # Standalone DBR unit test
│   └── tb_turbo_decoder_post_synth.v # Post-synthesis functional simulation TB
│
├── scripts/                          # Python & TCL automation
│   ├── turbo_ref_model.py            # Full-block fixed-point turbo reference model
│   ├── windowed_parallel_ber.py      # Windowed parallel decoder + BER sweep
│   ├── monte_carlo_ber.py            # Monte Carlo BER simulation framework
│   ├── gen_encoded_test_vectors.py   # LTE turbo encoder + AWGN channel → hex vectors
│   ├── full_block_float_turbo.py     # Floating-point reference decoder
│   ├── plot_windowed_parallel_ber.py # BER curve plotting utility
│   ├── qpp_lut_gen.py               # QPP interleaver table generator
│   ├── configure_bits_project_absolute_data.tcl  # Vivado project setup
│   ├── run_bits_impl_timing_check.tcl  # Implementation timing check
│   ├── synth_timing_100mhz.tcl      # 100 MHz synthesis constraint script
│   └── post_synth_funcsim_turbo_decoder.tcl  # Post-synth simulation setup
│
├── constraints/                      # FPGA constraint files
│   ├── Zybo-Z7-Master.xdc           # Full Zybo Z7 pin mapping
│   ├── turbo_decoder_button_bringup.xdc  # Button bringup constraints
│   └── turbo_decoder_axi_ooc.xdc    # Out-of-context AXI timing
│
├── data/                             # Test vectors and results
│   ├── *_ram.hex                     # 8 input BRAM hex files (sys/par/ilv)
│   ├── qpp_3200.hex                  # QPP interleaver LUT (1600 entries)
│   ├── true_info_bits.txt            # Ground-truth transmitted bits
│   ├── rtl_ber_results.txt           # RTL simulation BER report
│   ├── ber_results.txt               # Python vs RTL comparison report
│   ├── windowed_bram_compare_results.txt  # Deterministic match verification
│   ├── ber_curve_*.csv               # Monte Carlo BER curve data
│   ├── ber_curve_*.png               # BER curve plot
│   └── vector_metadata.txt           # Test vector parameters
│
├── docs/                             # Technical documentation
│   ├── implementation_report.md      # Detailed RTL architecture specification
│   ├── debug_changes.md              # Comprehensive bug fix log (5 phases)
│   ├── rtl_gap_analysis_windowed_parallel_decoder.md  # RTL vs model gap tracker
│   ├── windowed_parallel_ber_validation.md  # BER validation methodology
│   ├── zybo_axi_integration.md       # AXI4-Lite register map & SW guide
│   └── windowing_radix4_wavedrom.*   # Timing diagrams (JSON/PNG/SVG)
│
├── fpga_bringup/                     # Hardware bring-up infrastructure
│   ├── README.md                     # Staged bring-up test guide
│   ├── scripts/                      # ILA capture + Vitis vector generation
│   │   ├── generate_vitis_vectors.py
│   │   ├── compare_ila_hard_bits.py
│   │   └── ila_capture_to_vectors.py
│   └── vitis_app_src/               # Bare-metal Zynq C application
│       ├── main.c                    # 8-stage AXI test sequence
│       ├── turbo_decoder_axi.c       # AXI driver library
│       ├── turbo_decoder_axi.h       # Register map definitions
│       └── turbo_test_vectors.h      # Compiled test vectors
│
├── reports/                          # Vivado synthesis/implementation reports
│   ├── bits_opt_util_hier.rpt        # Hierarchical utilization breakdown
│   └── bits_opt_control_sets.rpt     # Control set analysis
│
├── final_utilization_report.txt      # Post-route FPGA utilization (full report)
├── implementation_timing_report.txt  # Post-route timing analysis (full report)
├── reproduce_paper_ber_plan.md       # Historical: plan to reproduce paper results
│
├── *.pdf                             # Reference research papers
│   ├── main-working-paper-11JSSC-turbo-research-paper.pdf  # Studer et al. JSSC 2011
│   └── map-decoder-architectures.pdf # MAP decoder survey
│
└── .gitignore                        # Excludes build artifacts, VCD, Vivado temps
```

---

## How to Run the Project

### Prerequisites

| Tool | Purpose |
|------|---------|
| **Icarus Verilog** (`iverilog`, `vvp`) | Behavioral RTL simulation |
| **Python 3** + NumPy | Reference models, test vector generation, BER analysis |
| **AMD Vivado 2024.x** | Synthesis, implementation, bitstream generation |
| **AMD Vitis 2024.x** | Bare-metal Zynq application (optional, for AXI bring-up) |
| **Digilent Zybo Z7-10** | Hardware validation (optional) |

### 1. Generate Test Vectors

```bash
cd scripts
python gen_encoded_test_vectors.py --K 3200 --num-siso 2 --ebn0 1.0 --seed 57 --out-dir ../data
```

This runs the LTE turbo encoder, applies AWGN noise, quantizes channel LLRs, and writes 8 input BRAM hex files + metadata to `data/`.

### 2. Run Python Reference Model

```bash
python turbo_ref_model.py --data-dir ../data
```

Produces `ref_final_intrinsic.txt`, `ref_final_hard_bits.txt`, and `ber_results.txt`.

### 3. Run RTL Simulation

```bash
# Compile
iverilog -o build/tb_turbo_decoder.vvp -I rtl tb/tb_turbo_decoder.v rtl/*.v

# Simulate
cd build
vvp tb_turbo_decoder.vvp
```

Expected output:
```
PASS: Correct number of done pulses (11).
RTL Turbo Decoder BER Results
  Eb/N0              = 1.000 dB
  Channel hard BER   = 544/3200 = 0.170000000000
  RTL final L_D BER  = 1/3200 = 0.000312500000
PASS: Hard-decision RAM matches final intrinsic signs.
```

### 4. Cross-Verify RTL vs Python

```bash
python scripts/windowed_parallel_ber.py --decoder radix4 --from-bram-dir data/ --compare-rtl data/
```

Expected: `intrinsic value mismatches: 0/3200`, `hard-bit mismatches: 0/3200`.

### 5. Monte Carlo BER Sweep

```bash
python scripts/windowed_parallel_ber.py --decoder radix4 --sweep --ebn0-start 0.0 --ebn0-stop 1.0 --ebn0-step 0.25
```

### 6. FPGA Synthesis & Implementation (Vivado)

```bash
# Open Vivado, source the project setup script
source scripts/configure_bits_project_absolute_data.tcl

# Or run OOC synthesis for timing check
source scripts/synth_timing_100mhz.tcl
```

### 7. Hardware Bring-Up (Zybo Z7-10)

See [`fpga_bringup/README.md`](fpga_bringup/README.md) for the complete staged bring-up procedure, including both the button-triggered PL-only flow and the Vitis AXI bare-metal application flow.

---

## Challenges Faced

### 1. Radix-4 Branch Metric Mapping Correctness
Correctly mapping the 32 radix-4 branch metrics from the 16 even-step and 16 odd-step radix-2 metrics required careful cross-referencing of the trellis predecessor table from the paper. An incorrect predecessor index silently produced plausible but wrong LLRs, requiring a bit-exact Python `--decoder radix4` mode to isolate the issue.

### 2. Alpha/Gamma Memory Timing Alignment
The backward recursion unit must read the alpha metric corresponding to **two** trellis steps earlier (α_{k-2}) while computing β_k. An off-by-one in the alpha write timing caused the BR to read stale or future values. The fix was to write the *pre-transition* alpha metric so the read-during-backward naturally obtains the correct α/γ pair.

### 3. Cross-Segment Boundary Initialization
In paper-boundary mode, core 0's dummy backward recursion must cross into core 1's segment to generate an accurate initial β estimate. This required a separate address mapping path, validity checks for out-of-range trellis indices, and tail-sample shadow registers to avoid BRAM port contention.

### 4. Extrinsic Ping-Pong BRAM Coherency
During interleaved half-iterations, the QPP-permuted write-back address may alias with a concurrent read address. The ping-pong double-buffering scheme with bank selection tied to `half_iter_cnt[0]` ensures reads always see the *previous* half-iteration's data, avoiding read-during-write hazards.

### 5. Near-Maximum FPGA Utilization
At 87.9% LUT utilization on the Zynq-7010, the design operates near the device's physical limit. Adding a full 19-probe ILA for debug exceeded device capacity. The solution was a minimal 1-probe packed ILA (5-bit: 1 valid + 4 hard bits, 1024 samples) combined with Vivado's area-optimized synthesis strategy (`Flow_AreaOptimized_high`, `flatten_hierarchy full`).

### 6. LTE Tail Trellis Handling with Radix-4
The 3 LTE tail trellis steps are an odd number, but radix-4 processes pairs. The solution zero-pads the 4th position and marks it with validity flags, ensuring the tail is correctly processed without corrupting the final window's metrics. The last core (core 1) processes the tail while core 0 stops at the segment boundary.

---

## What I Learned

- **VLSI Architecture Translation**: Converting a research paper's block diagram into cycle-accurate Verilog requires understanding every implicit assumption — memory port timing, pipeline latency, normalization strategies — that papers leave to the reader.
- **Fixed-Point Design Trade-offs**: Selecting the optimal bit width at each pipeline stage is an exercise in balancing area vs. BER performance; a single extra bit in the state metrics can double memory cost, while one fewer can cause modulo normalization failure.
- **Hardware-Software Co-Verification**: Building a Python reference model that is *bit-identical* to the RTL at every intermediate node — not just the final output — is essential for isolating bugs in pipelined, multi-core architectures. The "golden model" approach caught subtle issues that output-only comparison would miss.
- **FPGA Bring-Up Methodology**: Systematic staged validation (simulation → post-synthesis simulation → ILA on hardware) dramatically reduced debug time. Knowing which signals to expose via ILA before taping out to the board is a critical skill.
- **Parallel Decoder Scheduling**: Coordinating forward, backward, and dummy-backward recursions across multiple cores with shared memory access requires careful FSM design. Lockstep scheduling simplifies the write-back muxing but demands equal-length windows even when the last window is partial.

---

## Future Improvements

| Improvement | Impact |
|-------------|--------|
| **Scale to N=8 parallel SISO cores** | Directly increases throughput by 4× with Batcher/crossbar interleaver network |
| **Configurable K via runtime parameter** | Support all 188 LTE block sizes (40–6144) with runtime QPP table loading |
| **Early termination** | Monitor extrinsic convergence across iterations; skip remaining half-iterations when BER floor is reached |
| **AXI4-Stream DMA interface** | Replace register-based BRAM loading with burst DMA for realistic base station integration |
| **Log-MAP correction** | Add max* correction term (lookup table) to close the ~0.1 dB gap vs. optimal Log-MAP |
| **ASIC synthesis** | Port to a standard-cell flow (e.g., TSMC 28nm) for power and area characterization |
| **LLR quantization sweep** | Optimize the channel LLR scale factor per SNR operating point for minimum implementation loss |

---

## Conclusion

This project demonstrates a complete, verified implementation of a parallel radix-4 LTE turbo decoder — from the BCJR algorithm's mathematical formulation through RTL design, fixed-point optimization, FPGA synthesis, and on-hardware validation. The decoder achieves **bit-exact correctness** against independent Python reference models, meets **100 MHz timing** on a resource-constrained Zynq-7010, and reduces channel BER from 17% to 0.03% at E_b/N_0 = 1.0 dB. The project exercises the full digital IC design lifecycle: algorithm study, architecture design, RTL coding, verification, synthesis, timing closure, and hardware bring-up — skills directly applicable to ASIC/FPGA roles in wireless communications, signal processing, and digital design.

---

## References

1. C. Studer, C. Benkeser, S. Belfanti, and Q. Huang, "Design and Implementation of a Parallel Turbo-Decoder ASIC for 3GPP-LTE," *IEEE Journal of Solid-State Circuits (JSSC)*, vol. 46, no. 1, pp. 8–17, Jan. 2011.
2. 3GPP TS 36.212, "Multiplexing and Channel Coding," LTE standard specification.

<!-- ## Manual Edits Needed

> Items below may require your manual input to finalize the README:

- [ ] **Add your name and affiliation**: Consider adding `## Author` section with your name, IIT Mandi roll number, and course info (DVAD, 6th Semester)
- [ ] **Add team members**: If this was a group project (the repo name mentions "grp_10"), add team member names and contribution breakdown
- [ ] **BER curve image**: The BER plot `data/ber_curve_K3200_radix4_N2_fixed_paper_tail_R0.375.png` exists — embed it in the Results section with `![BER Curve](data/ber_curve_K3200_radix4_N2_fixed_paper_tail_R0.375.png)` if desired
- [ ] **Timing diagram images**: `docs/windowing_radix4_wavedrom.png` and `docs/window_schedule_c0_c1_wavedrom.png` can be embedded in the Architecture section
- [ ] **Add ILA hardware screenshot**: If you have Vivado ILA waveform captures from the FPGA run, add them to the Results section
- [ ] **Update Monte Carlo BER results**: Current results use a small number of frames (5–10). If you run longer sweeps, update the BER table
- [ ] **Add demo video/GIF**: A short recording of the Zybo board running the decoder (LEDs showing decode progress) would strengthen the hardware validation narrative
- [ ] **LinkedIn / portfolio link**: Add a link to your portfolio or LinkedIn if you want recruiters to contact you
- [ ] **License**: Consider adding an open-source license (MIT, Apache 2.0) if you want the project to be reusable -->
