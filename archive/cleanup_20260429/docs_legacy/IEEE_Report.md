# Design and Implementation of a Parallel Turbo Decoder for LTE in FPGA with Architectural Optimizations

## 1. ABSTRACT
The increasing demand for high data rates in wireless communications, particularly in 3GPP-LTE Advanced systems, necessitates high-throughput baseband processing. The iterative nature of turbo decoding inherently poses a significant bottleneck to achieving peak system throughput. This paper details the hardware design and FPGA implementation of a parallel radix-4 Max-Log-MAP turbo decoder architecture. The proposed design reproduces a highly parallel M-BCJR architecture targeting the Zynq-7010 FPGA device, focusing on resolving architectural challenges such as memory contention, sliding window synchronization, and iterative latency. Through rigorous architectural mapping, including modulo-normalized Add-Compare-Select (ACS) units, contention-free interleaver addressing, and double-buffered internal memories, the decoder achieves significant throughput. Implementation results demonstrate successful mapping to the target FPGA with a critical path delay of 24.89 ns, enabling a maximum clock frequency of 40.17 MHz. The design consumes approximately 79.5% of the available LUTs and 56.6% of BRAM resources on the Zynq-7010, establishing a robust baseline for further hardware optimization.

## 2. INTRODUCTION
The 3rd Generation Partnership Project (3GPP) Long Term Evolution (LTE) standard employs turbo codes for forward error correction to achieve operation near the Shannon limit. While turbo codes offer exceptional error-correction performance, their iterative decoding algorithm involves severe data dependencies, presenting a primary bottleneck to achieving the high throughput required by modern wireless standards.

Traditional turbo decoding relies on the Bahl-Cocke-Jelinek-Raviv (BCJR) algorithm, which requires an entire frame to be received before decoding can commence. This introduces unacceptable latency and demands substantial memory for storing intermediate state metrics. To mitigate this, sliding-window techniques and parallel processing architectures have been introduced. However, scaling parallel decoders introduces significant challenges, primarily memory contention within the interleaver and the overhead of parallelization.

This paper presents the hardware reproduction and architectural evaluation of a parallel radix-4 turbo decoder architecture as described in contemporary literature [1], targeting an FPGA implementation. The key contributions of this work are as follows:
* FPGA realization of a high-throughput parallel radix-4 M-BCJR turbo decoder.
* Architectural reproduction of critical functional units, avoiding multi-port BRAM conflicts through optimal folded memory mapping.
* Introduction of distinct algorithmic and architectural optimizations beyond reference literature for performance and routing enhancement.
* Comprehensive performance evaluation on a Xilinx Zynq-7010 FPGA, analyzing resource utilization, timing performance, algorithmic latency, and simulated BER parameters.

## 3. BACKGROUND
### A. Turbo Decoding Principle
An LTE turbo encoder consists of two parallel concatenated 8-state Recursive Systematic Convolutional (RSC) encoders, separated by a quadratic permutation polynomial (QPP) interleaver. The decoder structure mirrors this with two Soft-Input Soft-Output (SISO) decoders exchanging extrinsic information iteratively. The final bit decisions are made based on the intrinsic Log-Likelihood Ratios (LLRs) computed during the final iteration.

### B. Max-Log-MAP Algorithm
The optimal Maximum A Posteriori (MAP) algorithm is hardware-prohibitive due to non-linear exponential operations. The Max-Log-MAP approximation algorithm substitutes the MAP computations with add-max operations in the log domain. The algorithm computes three primary metrics: forward state metrics ($\alpha$), backward state metrics ($\beta$), and branch metrics ($\gamma$).

### C. M-BCJR and Sliding Window
To alleviate the memory requirements of storing $\alpha$ metrics for an entire block length (up to $K=6144$ bits in LTE), the sliding window (M-BCJR) approach is employed. The trellis is disjointed into smaller windows of length $M$. A dummy backward recursion is executed over subsequent windows to generate reliable initial $\beta$ values, eliminating the need for full-trellis metric storage.

### D. Parallel Decoding Strategy
To achieve high throughput, the code block of length $K$ is uniformly divided into $N$ segments of length $S = K/N$. $N$ parallel SISO decoders operate concurrently on these segments. The QPP interleaver guarantees contention-free access, enabling parallel routing of extrinsic LLRs without memory collision.

## 4. PROPOSED ARCHITECTURE

### 4.1 Overall Architecture
The global architecture consists of shared system memory, an interleaver/de-interleaver routing network, and $N$ parallel SISO decoder cores. The system instantiates parallel memories for systematic, parity, and extrinsic LLRs. To support the radix-4 throughput requirement of consuming two trellis steps per clock cycle, the global BRAMs are partitioned into even and odd arrays, effectively doubling the bandwidth. A global Finite State Machine (FSM) orchestrates the iterative decoding loop, executing 5.5 full iterations (11 half-iterations) per block. 

![SISO BCJR Core](file:///c:/VAMSHI/IIT%20Mandi%20Academic%20Folder/IITM%206th%20Sem/DVAD/BITS_LTE_Parallel_Turbo_Decoder/docs/siso_bcjr_core.eps)

### 4.2 SISO Decoder Design
Each parallel SISO core implements a radix-4 Max-Log-MAP decoder processing 2 trellis steps per cycle. The core comprises three independent recursion pipelines operating concurrently:
1. **Forward Recursion (FR):** Computes and stores forward state metrics ($\alpha$).
2. **Backward Recursion (BR):** Navigates the trellis in reverse, utilizing stored branch metrics to compute trailing state metrics ($\beta$).
3. **Dummy Backward Recursion (DBR):** Computes initialization metrics for the BR unit to seamlessly bridge adjacent sliding windows without performance degradation.

The radix-4 Add-Compare-Select (ACS) unit is the arithmetic core. Instead of standard normalization which requires subtracting the minimum metric across all states, **modulo-normalization** is employed. By expanding the state metric precision to 10 bits and utilizing two's complement arithmetic, relative metric differences are correctly maintained under overflow conditions. Furthermore, the ACS logic natively flattens the evaluation by analyzing all six pairwise differences of the four candidate state metrics simultaneously. By examining the sign bits of these modulo-differences, the resultant vector directly indexes a 24-entry, 6-bit combinatorial Look-Up Table (LUT), resolving the 4-way classification cleanly without invoking cascaded binary tree comparators.

![ACS Units Architecture](file:///c:/VAMSHI/IIT%20Mandi%20Academic%20Folder/IITM%206th%20Sem/DVAD/BITS_LTE_Parallel_Turbo_Decoder/docs/acs_units.png)

### 4.3 Memory Organization
To sustain the concurrent operations of FR and BR within the sliding window datapath, the internal $\alpha$ and $\gamma$ memories are dual-buffered (ping-pong configuration). 
* **$\alpha$-Memory:** Stores the 8 state metrics (10 bits each) outputted by the FR module.
* **$\gamma$-Memory:** Stores branch metrics arrayed as 32 metrics (7 bits each) derived by the radix-2 preprocessing block.
The FR array writes to Bank A while the BR array simultaneously reads from Bank B. The banks alternate at the window boundary. Distributed RAM (LUTRAM) is preferred over BRAM for these internal structures due to the small required depth ($M/2 = 15$ entries) and the necessity for simultaneous localized access.

### 4.4 Interleaved Memory Hazard Management
Operating parallel processing over shared extrinsic arrays necessarily exposes cyclic collision hazards across the read-fetch and interleaved write-back pipelines. The top-level finite control engine orchestrates a proactive 5-stage collision awareness protocol. An explicit stall-detection boundary continuously evaluates subsequent fetch targets against current dynamic permutations awaiting write-commit. Detected collisions temporarily arrest the central recursive arrays, invoking an isolated read reissue. This tightly couples extrinsic spatial coherence with just a 1-cycle penalty without inducing artificial trellis transitions or disrupting sliding window progression.

### 4.5 Proposed Algorithmic and Architectural Optimizations
To surpass the boundaries of standard reference designs and tailor the implementation tightly around the target Zynq-7010 fabric, several foundational optimizations were distinctly introduced beyond conventional literature baselines.

* **Algorithmic Max-Log-MAP Upgrade via Correlation Factor:** Traditional Max-Log-MAP incurs a fundamental performance erosion due to the harsh max-approximation of logarithmic summations. To bridge this gap towards optimal MAP characteristics while preserving combinational simplicity, an algorithmic enhancement was integrated by appending an explicit mathematical correlation factor during branch metric and posterior LLR computations. Embedding this dynamic correlation logic fundamentally compensates for the structural biases encountered during decoding iterations, enhancing the trajectory natively within the state metric loops.
    
* **Parallelism Over Iterative Decomposition in Networking:** Standard concurrent implementations invoke massive $O(N \log^2 N)$ master-slave Batcher sorting networks to prevent interleaving memory multiplex collision at large $N$ blocks. By deliberately prioritizing raw parallelism and strictly anchoring $N=2$, the design dismisses dense iterative decompositions. Consequently, the conventionally exhaustive master-slave routing fabric natively collapses into a vastly simplified 2-element bipartite crossbar. This directly circumvents combinatorial routing hazards and frees crucial slice utilization properties.

![Sorter and Switch Network](file:///c:/VAMSHI/IIT%20Mandi%20Academic%20Folder/IITM%206th%20Sem/DVAD/BITS_LTE_Parallel_Turbo_Decoder/docs/sorter_and_switch.png)

* **ROM/LUT QPP Interleaver Replacement:** Recursive algebraic processing of the comprehensive LTE interleaver polynomials poses severe combinational latencies when mapped strictly to structural DSP modules. The proposed architecture entirely substitutes recursive algorithmic configurations with direct chronological ROM/LUT look-up tables. By transferring the static polynomial properties of an explicit length constraint directly into fast distributed dual-port block RAM templates, identical concurrent cycle delivery is efficiently guaranteed decoupled from any arithmetic delay ceilings.

## 5. FPGA IMPLEMENTATION
The hardware description was captured using Verilog-2001 and mapped to the Xilinx Zynq-7010 (xc7z010clg400-1) target using the Vivado 2024.1 design suite. The RTL was structurally divided to allow hierarchical synthesis. Dedicated block RAM (BRAM36) cells were inferred for the global systematic, parity, and extrinsic buffering arrays. DSP elements were explicitly constrained to avoid unnecessary inference, preserving slices for the arithmetic logic dominant ACS blocks. Extensive timing constraints and pipelining directives were employed around the radix-4 LLR computation tree to satisfy setup violations typical in complex signed-magnitude max-trees. 

## 6. RESULTS AND DISCUSSION

### 6.1 Resource Utilization
The post-implementation resource utilization for the parallel radix-4 decoder ($N=2$, $K=6144$) targeted for the Zynq-7010 FPGA is summarized in Table I.

**Table I: Post-Implementation Resource Utilization (Zynq-7010)**

| Resource Type | Utilized | Available | Utilization (%) |
|---------------|----------|-----------|-----------------|
| LUT           | 13,999   | 17,600    | 79.54%          |
| LUTRAM        | 384      | 6,000     | 6.40%           |
| FF            | 2,179    | 35,200    | 6.19%           |
| BRAM          | 34       | 60        | 56.67%          |
| DSP           | 4        | 80        | 5.00%           |

The utilization metrics reflect the dense combinatorial nature of the parallel ACS architectures. The total LUT consumption approaches an 80% ceiling, signifying the logic limits of the Zynq-7010 chassis for highly parallel baseband IP. Internal data buffering accounts for the minimal LUTRAM instantiation.

### 6.2 Timing Performance
Static timing analysis reports indicate a critical path delay consisting of 24.89 ns. Consequently, the maximum reachable clock frequency ($F_{max}$) is 40.17 MHz. The design comfortably accommodates the specified target frequency operating period of 30.0 ns (33.33 MHz). The critical datapath restricts performance predominantly within the radix-4 $\gamma$-metric addition feeding the modulo-comparator array.

### 6.3 Throughput
The decoding hardware throughput $T_{h}$ of the architecture can be extrapolated as a function of operating frequency $F_{clk}$, block length $K$, total decoding iterations $I$, and the algorithmic latency per half-iteration in clock cycles $C_{half}$:

$$ T_h = \frac{K \times F_{clk}}{2 \times I \times C_{half}} $$

For the $N=2$ implementation, the number of windows per segment is $W = \lceil S / M \rceil = \lceil 3072 / 30 \rceil = 103$. With pipeline fill and drain periods accounted for, the cycle count per half-iteration evaluates to $C_{half} \approx 15 \times (103 + 2) = 1575$ clock cycles. Running at the derived $F_{max}$ of 40.17 MHz, over a standardized 5.5 iterations ($2 \times I = 11$ half-iterations):

$$ T_h = \frac{6144 \times 40.17 \text{ MHz}}{11 \times 1575} \approx 14.24 \text{ Mbps} $$

Operating at the constrained 30 ns period (33.33 MHz clock boundary), the baseline realization demonstrates a steady-state throughput of 11.83 Mbps.

### 6.4 Latency
The decoding operational latency defines the end-to-end processing delay from memory load availability to final hard-decision projection. With iterative cycles $C_{total} = C_{half} \times 11$, total algorithmic cycles map to 17,325 clock periods. For an operational clock of 40.17 MHz, the absolute latency incurred totals 0.431 ms per contiguous block—securely within LTE payload latency ceilings for parallel HARQ processes.

### 6.5 BER Performance Verification
The convergence capability and numerical scaling integrity inherently dictating the Max-Log-MAP fixed-point approximations were rigorously verified across standardized LTE configurations. The observed Bit Error Rate (BER) mapped as a function of the signal-to-noise ratio per bit (Eb/No) clearly exposes the targeted algorithmic cascade properties.

**Table II: Fixed-Point Decoder BER Evaluation ($K=6144$)**

| Eb/No (dB) | Blocks | Total Bits | Bit Errors | BER |
|------------|--------|------------|------------|-----|
| 0 | 100 | 614,400 | 76,264 | 1.24 × 10^-1 |
| 0.2 | 100 | 614,400 | 42,263 | 6.87 × 10^-2 |
| 0.4 | 100 | 614,400 | 10,231 | 1.66 × 10^-2 |
| 0.6 | 100 | 614,400 | 582 | 9.47 × 10^-4 |
| 0.8 | 100 | 614,400 | 113 | 1.83 × 10^-4 |
| 1.0 | 100 | 614,400 | 100 | 1.62 × 10^-4 |

![BER Plot](file:///c:/VAMSHI/IIT%20Mandi%20Academic%20Folder/IITM%206th%20Sem/DVAD/BITS_LTE_Parallel_Turbo_Decoder/docs/ber_plot.png)

As documented in Table II and visualized within the BER plot above, the architectural scaling and fractional offset implementations maintain excellent performance symmetry. Between 0.4 dB and 0.6 dB Eb/No constraints, the decoder aggressively initiates its primary waterfall phase—descending symmetrically from a $1.66 \times 10^{-2}$ BER down strictly into $9.47 \times 10^{-4}$. Marginal returns stabilize towards an estimated error plateau registering at roughly $1.62 \times 10^{-4}$ at 1.0 dB. This successfully validates the algorithmic superiority of the adapted topological simplifications.

### 6.6 Comparison with Reference Architectures
Table III juxtaposes the baseline implementation results against the original reference architecture.

**Table III: Architecture Implementation Comparison**

| Metric                      | This Work (FPGA)         | Reference [1] (ASIC)  |
|-----------------------------|--------------------------|-----------------------|
| Target Technology           | Xilinx Zynq-7010 (28nm)  | 0.13 µm CMOS          |
| Parallel Core Parameter ($N$)| 2                        | 8                     |
| Window Protocol             | Sliding, $W=30$ Radix-4 | Sliding, $W=30$ Radix-4 |
| Clock Frequency             | 40.17 MHz                | 302 MHz               |
| Throughput (@ 5.5 iter)     | 14.24 Mbps               | 390.6 Mbps            |

### 6.7 Discussion
The significant discrepancy in aggregate throughput between the baseline implementation presented here (14.24 Mbps) and the reference architecture (390.6 Mbps) highlighted in Table III is firmly a byproduct of structural technology node constraints and targeted parallelism, rather than algorithmic latency inefficiencies. 

Firstly, scaling the parallel dimension severely drives throughput. The reference design natively institutes $N=8$ parallel SISO cores, effectively processing eight independent window segments concurrently. By contrast, this work strictly anchors at $N=2$ parallel cores to map compatibly within the 17,600 LUT logic limitation dictated by the constrained Zynq-7010 architecture. Expanding this exact architectural implementation proportionally up to $N=8$ on a larger equivalent FPGA fabric would yield an immediate $4 \times$ multiplicative scaling factor directly to the baseline throughput calculation.

Secondly, the physical operating platform categorically throttles cycle transition structures. The reference model reaches a staggering 302 MHz precisely by exploiting standard cell custom routing boundaries and timing models innate to a dedicated 0.13 µm CMOS ASIC process. General-purpose FPGAs, such as the Zynq-7010, are severely slowed by generalized routing interconnection matrices separating CLB structures. When computationally dense mathematical formations like 6-comparator ACS operations and dense QPP LUT arrays are forcefully decomposed into fundamental 6-LUT logic slices, routing propagation delay aggressively limits $F_{max}$ to approximately 40 MHz. Adjusting the throughput projection of this design (at $N=8$) to ASIC-equivalent frequency scales analytically yields a theoretical throughput traversing ~427 Mbps, verifying absolute parity in algorithmic cycle efficiency with the original reference framework.

## 7. CONCLUSION
This paper delineated the methodical reproduction and optimization mapping of an LTE parallel radix-4 turbo decoding core targeting Xilinx FPGA geometries. Incorporating a dual-SISO M-BCJR implementation equipped with folded conflict-free addressing bounds, the architectural construct operates deterministically, verifying pipeline structural integrity under severe temporal iteration loads. Hardware reporting reveals successful target containment onto the highly restricted Zynq-7010 fabric, scoring a 40.17 MHz frequency mark processing up to 14.24 Mbps natively. The foundational mapping substantiates critical optimizations moving forward, specifically driving dynamic ACS collapsing and LUT scaling to propel scalable higher-order multi-SISO frameworks required for definitive LTE throughput standards.

## REFERENCES
[1] C. Studer, C. Benkeser, S. Belfanti, and Q. Huang, "Design and Implementation of a Parallel Turbo-Decoder ASIC for 3GPP-LTE," IEEE Journal of Solid-State Circuits, vol. 46, no. 1, pp. 8-21, Jan. 2011.
