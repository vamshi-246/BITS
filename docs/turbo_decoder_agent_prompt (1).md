# RTL Implementation Prompt — Parallel LTE Turbo Decoder (Top-Level Integration)

## CONTEXT: WHAT ALREADY EXISTS

You are working in a repository that already contains a fully verified, synthesizable Verilog-2001
implementation of a radix-4 Max-Log M-BCJR SISO decoder core. **Do NOT touch, rewrite, or
restructure any existing file.** Your job is to build all surrounding infrastructure and wire
everything into a complete, FPGA-synthesizable parallel turbo decoder.

The existing module is `bcjr_core`. Its interface and behaviour are described precisely below.
Every new module you write must integrate cleanly with it without modifying a single line of it.

---

## SYSTEM PARAMETERS — FIXED, DO NOT CHANGE

| Parameter       | Value            | Derivation / Notes                                       |
|-----------------|------------------|----------------------------------------------------------|
| Standard        | 3GPP-LTE         |                                                          |
| Block length K  | 6144             | Maximum LTE codeword                                     |
| Parallelism N   | 2                | Two SISO cores operating simultaneously                  |
| Segment S       | 3072             | K / N = 3072 trellis steps per core                     |
| Window size W   | 30               | Trellis steps per M-BCJR sliding window                  |
| NUM_WINDOWS     | 103              | ceil(3072/30); last window has only 12 valid steps       |
| Half-iterations | 11               | Indices 0..10; even=natural phase, odd=interleaved phase |
| Trellis states  | 8                | LTE constraint-length-4 convolutional code               |
| Recursion       | Radix-4          | Processes 2 trellis steps per clock cycle                |
| QPP params      | f1=263, f2=480   | For K=6144: π(k) = (263k + 480k²) mod 6144              |
| Target          | Zynq-7010 FPGA   | Synthesizable Verilog-2001                               |
| HDL             | Verilog-2001     | No SystemVerilog constructs whatsoever                   |

---

## SIGNAL BIT-WIDTH REFERENCE TABLE

Every signal in every module must conform exactly to these widths. Violations are silent RTL errors.

| Signal                                | Width              | Range        | Notes                                       |
|---------------------------------------|--------------------|--------------|---------------------------------------------|
| Channel LLRs (L_s, L_p1, L_p2)       | 5-bit signed       | [−16, +15]   | Input to decoder                            |
| Extrinsic / A-priori LLRs (L_E, L_A) | 6-bit signed       | [−32, +31]   | Produced and consumed by bcjr_core          |
| Local trellis address (fr/br/dbr_addr)| 12-bit unsigned    | [0, 3070]    | Always even; odd step = addr+1              |
| BRAM row index                        | 11-bit unsigned    | [0, 1535]    | = local_addr >> 1                           |
| LUT index k                           | 12-bit unsigned    | [0, 3071]    | SISO-0 local step fed into LUT              |
| **LUT output π(k)**                   | **13-bit unsigned**| **[0, 6143]**| **12 bits is WRONG — π reaches 6143 > 4095**|
| SISO-1 derived address π(k+3072)      | 13-bit unsigned    | [0, 6143]    | Computed combinatorially from π(k)          |
| sorted_lo (master_net internal)       | 13-bit unsigned    | [0, 3071]    | Always < 3072 after sort                    |
| bram_row (master_net output)          | 11-bit unsigned    | [0, 1535]    | = sorted_lo[11:1]                           |
| perm_bit (master_net output)          | 1-bit              | {0,1}        | 0=no swap, 1=SISO-0 is in col[1]            |
| ld_ram output                         | 6-bit signed       | [−32, +31]   | L_extr from final half-iteration            |

**The 13-bit rule for π(k) is non-negotiable.** π(k) ∈ [0..6143]. 2¹²=4096 < 6143, so a
12-bit declaration silently truncates all addresses above 4095. This corrupts roughly a third
of all interleaved memory accesses with no synthesis warning. Every port, register, and wire
carrying π(k) or π(k+3072) must be `[12:0]`.

---

## EXISTING MODULE: `bcjr_core` — COMPLETE INTERFACE

Two instances: CORE_ID=0 handles global trellis steps [0..3071]; CORE_ID=1 handles [3072..6143].
Both output **local** addresses in [0..3071]. The top controller maps these to global/memory indices.

### Parameters
```verilog
parameter CORE_ID   = 0;    // 0 = first segment, 1 = last segment
parameter FRAME_LEN = 3072; // trellis steps in this segment (fixed at 3072)
```

### Ports
```verilog
input  wire        clk,
input  wire        rst_n,
input  wire        start,          // pulse high exactly 1 cycle to start a half-iteration

// LLR request handshake
output wire        llr_req,        // high when core is stalled waiting for LLRs
output wire [11:0] fr_addr,        // FR unit: current even-step local address [0..3070]
output wire [11:0] br_addr,        // BR unit: current even-step local address [0..3070]
output wire [11:0] dbr_addr,       // DBR unit: current even-step local address [0..3070]

input  wire        llr_valid,      // top controller asserts after all 18 LLRs below are ready
// FR inputs (5-bit sys/par, 6-bit apriori)
input  wire signed [4:0]  fr_sys_odd,   fr_sys_even,
input  wire signed [4:0]  fr_par_odd,   fr_par_even,
input  wire signed [5:0]  fr_apr_odd,   fr_apr_even,
// BR inputs
input  wire signed [4:0]  br_sys_odd,   br_sys_even,
input  wire signed [4:0]  br_par_odd,   br_par_even,
input  wire signed [5:0]  br_apr_odd,   br_apr_even,
// DBR inputs
input  wire signed [4:0]  dbr_sys_odd,  dbr_sys_even,
input  wire signed [4:0]  dbr_par_odd,  dbr_par_even,
input  wire signed [5:0]  dbr_apr_odd,  dbr_apr_even,

// Outputs
output wire        llr_out_valid,  // high when extrinsic output is valid (during BR backward pass)
output wire [11:0] llr_out_addr,   // local even-step address [0..3070], always even
output wire signed [5:0]  l_extr_odd,   // extrinsic LLR at step llr_out_addr+1 (odd)
output wire signed [5:0]  l_extr_even,  // extrinsic LLR at step llr_out_addr   (even)

output wire        done            // pulses 1 cycle after all 103 windows finish
```

### Behavioural contract — every point here is load-bearing

1. `fr_addr`, `br_addr`, `dbr_addr` are always even, always in [0..3070]. The odd-step partner
   is always `addr + 1`. This is guaranteed by radix-4 processing.

2. The core stalls indefinitely in `ST_LLR_WAIT` after asserting `llr_req`, until the controller
   asserts `llr_valid`. **The controller may take any number of cycles to fetch LLRs before
   asserting `llr_valid`.** Use this slack to pipeline multi-cycle memory fetches.

3. `llr_req` fires **once per radix-4 cycle** (one R4 cycle = 2 trellis steps). For 3072 steps
   / 2 = 1536 R4 steps per half-iteration, the controller sees exactly **1536 `llr_req` pulses**
   per core per half-iteration.

4. Both cores are structurally identical and started with the same `start` pulse at the same
   time. They therefore assert `llr_req`, `llr_out_valid`, and `done` **simultaneously**, in
   exactly the same clock cycle. The controller treats them as a single synchronised unit.

5. `llr_out_valid` fires during the BR backward pass — up to 15 cycles per window ×103 windows
   = up to 1545 write events per half-iteration. These **can and do overlap** with the fetch
   pipeline for the next forward step. The controller must continuously handle writes.

6. `done` pulses exactly 1 cycle, after the final window's backward pass completes.

7. `llr_out_addr` from both cores is always the same value (since they're synchronised). Use it
   directly to derive write addresses.

### Window scheduling pipeline (per core)
```
Slot:  FR           BR          DBR           Notes
  0    dummy_FR     —           —             CORE_ID=0: α init to known start state
  1    W1           —           W2'           W2' = dummy backward for W2 β-init
  2    W2           W1          W3'
  3    W3           W2          W4'
  ...
102    W102         W101        W103'
103    W103         W102        —             CORE_ID=1: last BR uses known terminal β; no DBR
104    —            W103        —
```
FR and DBR run simultaneously (forward pass, needs `llr_req` service).
BR runs afterward (backward pass, produces `llr_out_valid` outputs).
At any instant, FR/BR/DBR occupy **different** windows, so all 6 local addresses
{fr_addr, fr_addr+1, br_addr, br_addr+1, dbr_addr, dbr_addr+1} are always distinct.
Across both cores: **12 distinct trellis steps** live simultaneously.

---

## FOLDED MEMORY ARCHITECTURE — THE FOUNDATION OF EVERYTHING

**Every memory in this design uses the folded memory structure. Understand this before implementing
anything.**

A folded memory for N=2 stores K=6144 values as 1536 rows × 2 columns (after radix-4 odd/even split):

| BRAM suffix | Covers         | Rows | Row r stores                                         |
|-------------|----------------|------|------------------------------------------------------|
| `_even`     | even local steps | 1536 | col[0]=SISO-0 at local step 2r, col[1]=SISO-1 at local step 2r |
| `_odd`      | odd local steps  | 1536 | col[0]=SISO-0 at local step 2r+1, col[1]=SISO-1 at local step 2r+1 |

**Universal row-address formula** — used identically for ALL BRAMs in ALL phases:
```
row = even_local_addr >> 1

_even_ram[row]  → both SISOs' values at even step (even_local_addr)
_odd_ram[row]   → both SISOs' values at odd step  (even_local_addr + 1)
```
Example with fr_addr = 60: row = 30.
`sys_even_ram[30]` = {SISO-1 L_s at step 60, SISO-0 L_s at step 60}
`sys_odd_ram[30]`  = {SISO-1 L_s at step 61, SISO-0 L_s at step 61}

col[0] is always SISO-0 (core0). col[1] is always SISO-1 (core1). This is invariant.

---

## MEMORY INVENTORY — ALL 13 MEMORIES

### Group A — Channel Input LLRs (pre-loaded before decoding, read-only during decoding)

These 8 BRAMs are written once by the host through the load interface and never written again.
All 8 are accessed with the same row-address formula: `fr_addr>>1`, `br_addr>>1`, `dbr_addr>>1`.

| BRAM name           | Contains                              | Word width  | Phase used   |
|---------------------|---------------------------------------|-------------|--------------|
| `sys_odd_ram`       | L_s (natural order) for odd steps     | 2×5-bit     | Natural only |
| `sys_even_ram`      | L_s (natural order) for even steps    | 2×5-bit     | Natural only |
| `par1_odd_ram`      | L_p1 for odd steps                    | 2×5-bit     | Natural only |
| `par1_even_ram`     | L_p1 for even steps                   | 2×5-bit     | Natural only |
| `sys_ilv_odd_ram`   | L_s pre-permuted by π, odd steps      | 2×5-bit     | Interleaved only |
| `sys_ilv_even_ram`  | L_s pre-permuted by π, even steps     | 2×5-bit     | Interleaved only |
| `par2_odd_ram`      | L_p2 pre-permuted by π, odd steps     | 2×5-bit     | Interleaved only |
| `par2_even_ram`     | L_p2 pre-permuted by π, even steps    | 2×5-bit     | Interleaved only |

**Why sys_ilv and par2 are pre-permuted and why this removes all interleaver complexity for them:**
In the interleaved phase, the SISO needs L_s[π(k)] for its local step k. If we pre-store
L_s[π(k)] at position k in `sys_ilv`, then the SISO can read it using the same local address k
(i.e. `fr_addr>>1`) with no LUT lookup at all. This is the entire point of these four BRAMs.
Parity-2 is treated identically: L_p2[π(k)] is stored at position k in `par2_*_ram`.

**Exact memory content for host loading** (for sys_ilv; par2 is analogous):
```
For r in 0..1535:
  sys_ilv_even_ram[r][col0] = L_s[ π(2r)               ]   // SISO-0's π address for even step 2r
  sys_ilv_even_ram[r][col1] = L_s[ (π(2r) + 3072) % 6144 ] // SISO-1's π address
  sys_ilv_odd_ram[r][col0]  = L_s[ π(2r+1)              ]
  sys_ilv_odd_ram[r][col1]  = L_s[ (π(2r+1) + 3072) % 6144 ]
```
The host pre-computes these using the QPP formula and writes during the load phase.

**Port requirement:** Each Group-A BRAM needs **3 simultaneous read ports** (FR, BR, DBR)
plus **1 write port** (load phase). Implement as **2× true dual-port BRAM** per logical BRAM
(4 ports available; use 3 for decode reads, 1 for load writes).

### Group B — Extrinsic / A-priori RAM (read and written every half-iteration)

| BRAM name              | Contains                     | Word width | Notes                             |
|------------------------|------------------------------|------------|-----------------------------------|
| `extrinsic_odd_ram`    | L_E for odd steps            | 2×6-bit    | col[0]=SISO-0, col[1]=SISO-1      |
| `extrinsic_even_ram`   | L_E for even steps           | 2×6-bit    | col[0]=SISO-0, col[1]=SISO-1      |

These are the **only** BRAMs with non-trivial addressing during the interleaved phase.

**Natural phase access:** rows `fr_addr>>1`, `br_addr>>1`, `dbr_addr>>1` — exactly like Group A.
**Interleaved phase read:** rows from `master_net.bram_row` (after LUT lookup and sort).
**Write-back:** address depends on phase; see the write-back section below.

**Port requirement:** 3 simultaneous read ports + 1 write port.
Implement as **2× true dual-port BRAM** per logical BRAM.

### Group C — Final Output RAM

| BRAM name | Rows | Word width  | Written when               | Content                                    |
|-----------|------|-------------|----------------------------|--------------------------------------------|
| `ld_ram`  | 1536 | 2×6-bit     | After half-iteration 10    | L_extr from last half-iteration            |

Single-port, 1 write / 1 read. The sign of L_extr gives the hard bit decision (sign > 0 → bit 1).
The controller writes this BRAM in ST_WRITE_LD by copying from the final extrinsic BRAM state.

---

## QPP INTERLEAVER LUT MODULE

### Module: `qpp_lut`

**Used only during the interleaved phase and during interleaved write-back.**
Not used during natural phase at all.

**Content:** 3072 entries. Entry k = π(k) = (263k + 480k²) mod 6144. Read-only ROM.
Output width: **13 bits** (π(k) ∈ [0..6143]). Input width: 12 bits (k ∈ [0..3071]).

**6 simultaneous read ports required** — one per active address slot:
{fr_addr, fr_addr+1, br_addr, br_addr+1, dbr_addr, dbr_addr+1}.
All 6 addresses are distinct (proven by window scheduling — FR/BR/DBR always on different windows).

**Physical implementation:** 3 identical copies of the LUT ROM, each as a true dual-port BRAM:
- Copy 0, ports {A,B}: reads at `fr_addr` and `fr_addr+1`
- Copy 1, ports {A,B}: reads at `br_addr` and `br_addr+1`
- Copy 2, ports {A,B}: reads at `dbr_addr` and `dbr_addr+1`

All 3 copies store identical data; they differ only in which addresses are presented.
Initialise all 3 from `qpp_6144.hex` using `$readmemh` in an `initial` block (Vivado-legal for ROM).

**Port reuse during backward pass:** When `llr_out_valid` is active (backward pass), the FR and
DBR ports on copies 0 and 2 are idle (no forward fetch in progress). The controller reuses them
to look up π(llr_out_addr) and π(llr_out_addr+1) for write-back address generation. Account for
the 1-cycle LUT latency: issue the LUT read 1 cycle before `llr_out_valid` is expected.

```verilog
module qpp_lut (
    input  wire        clk,
    // 6 read address inputs (LUT index k, 12-bit, range [0..3071])
    input  wire [11:0] addr_fr_even,   // = fr_addr
    input  wire [11:0] addr_fr_odd,    // = fr_addr + 12'd1
    input  wire [11:0] addr_br_even,   // = br_addr
    input  wire [11:0] addr_br_odd,    // = br_addr + 12'd1
    input  wire [11:0] addr_dbr_even,  // = dbr_addr
    input  wire [11:0] addr_dbr_odd,   // = dbr_addr + 12'd1
    // 6 outputs — 13-bit (π(k) ∈ [0..6143], requires 13 bits)
    output reg  [12:0] pi_fr_even,
    output reg  [12:0] pi_fr_odd,
    output reg  [12:0] pi_br_even,
    output reg  [12:0] pi_br_odd,
    output reg  [12:0] pi_dbr_even,
    output reg  [12:0] pi_dbr_odd
);
// Implementation: three reg [12:0] lut_copy[0:3071] arrays
// initial begin $readmemh("qpp_6144.hex", lut_copy0); ... end
// Three always @(posedge clk) blocks, one per copy, each reading 2 addresses
```

---

## MASTER NETWORK MODULE

### Module: `master_net` — instantiated 6 times

Instances: `master_fr_even`, `master_fr_odd`, `master_br_even`, `master_br_odd`,
           `master_dbr_even`, `master_dbr_odd`.

**Purely combinatorial — no clock, no registers.**

**Input:** `addr_siso0` = π(k) from the LUT output (13-bit, [0..6143]).
**Function:** Derive SISO-1's address, sort the pair, output the BRAM row and permutation bit.

**SISO-1 address derivation — the ONLY correct formula:**
```verilog
wire [12:0] pi_siso1 = (addr_siso0 < 13'd3072) ? (addr_siso0 + 13'd3072)
                                                : (addr_siso0 - 13'd3072);
```
Mathematical basis: π(k+3072) ≡ π(k) + 3072 (mod 6144) for the LTE QPP with K=6144.
Proof: π(k+3072) − π(k) = 263·3072 + 2·480·k·3072 + 480·3072² mod 6144.
The k and k² terms contain factor 2·3072 = 6144 → vanish. 263·3072 mod 6144 = 3072 (263 is odd).

**NEVER use XOR:** `addr_siso0 ^ 13'h0C00` is WRONG for addr_siso0 ≥ 3072.
Verification: π(k)=5109 → ternary gives 5109−3072=2037 ✓; XOR gives 5109^3072=8181 ✗ (out of range).
Second example: π(k)=743 → ternary gives 743+3072=3815 ✓; XOR gives 743^3072=3815 ✓ (coincidence).
Since XOR fails for the ≥3072 case, it is prohibited.

**Sorting:** since exactly one of {pi_siso0, pi_siso1} is < 3072 and the other is ≥ 3072:
- `sorted_lo = min(addr_siso0, pi_siso1)` — always in [0..3071], guaranteed
- `perm_bit = (addr_siso0 >= 13'd3072)` — 1 if SISO-0's address was the larger one

```verilog
module master_net (
    input  wire [12:0] addr_siso0,  // π(k) from LUT — 13-bit [0..6143]
    output wire [12:0] pi_siso1,    // π(k+3072) — 13-bit [0..6143]
    output wire [10:0] bram_row,    // BRAM row for extrinsic_odd or _even — 11-bit [0..1535]
    output wire        perm_bit     // 0: SISO-0 value is in col[0]; 1: in col[1]
);
assign pi_siso1  = (addr_siso0 < 13'd3072) ? (addr_siso0 + 13'd3072)
                                            : (addr_siso0 - 13'd3072);
assign perm_bit  = (addr_siso0 >= 13'd3072);
assign bram_row  = perm_bit ? pi_siso1[11:1] : addr_siso0[11:1];
// Explanation: sorted_lo is the one that is < 3072.
// If perm_bit=0, addr_siso0 < 3072, so sorted_lo = addr_siso0; bram_row = addr_siso0[11:1]
// If perm_bit=1, pi_siso1 < 3072,   so sorted_lo = pi_siso1;   bram_row = pi_siso1[11:1]
endmodule
```

**perm_bit semantics** (memorise this — it drives both slave_net and write-back):
- `perm_bit=0`: SISO-0's π(k) fell in [0..3071] → SISO-0's extrinsic is in col[0] of the BRAM.
- `perm_bit=1`: SISO-0's π(k) fell in [3072..6143] → SISO-0's extrinsic is in col[1] of the BRAM.

---

## SLAVE NETWORK MODULE

### Module: `slave_net` — instantiated 6 times

One-to-one correspondence with master_net instances. Purely combinatorial.

**Function:** Given BRAM read data and `perm_bit`, route the correct values to each core.

```verilog
module slave_net (
    input  wire        perm_bit,
    input  wire signed [5:0] col0,      // extrinsic BRAM col[0] — SISO-0 physical slot
    input  wire signed [5:0] col1,      // extrinsic BRAM col[1] — SISO-1 physical slot
    output wire signed [5:0] out_siso0, // a-priori for core0
    output wire signed [5:0] out_siso1  // a-priori for core1
);
assign out_siso0 = perm_bit ? col1 : col0;
assign out_siso1 = perm_bit ? col0 : col1;
endmodule
```

The 6 slave outputs drive 12 a-priori registers in the top-level:

| Instance        | Step parity | out_siso0 feeds          | out_siso1 feeds          |
|-----------------|-------------|--------------------------|--------------------------|
| slave_fr_even   | even        | core0 fr_apr_even_reg    | core1 fr_apr_even_reg    |
| slave_fr_odd    | odd         | core0 fr_apr_odd_reg     | core1 fr_apr_odd_reg     |
| slave_br_even   | even        | core0 br_apr_even_reg    | core1 br_apr_even_reg    |
| slave_br_odd    | odd         | core0 br_apr_odd_reg     | core1 br_apr_odd_reg     |
| slave_dbr_even  | even        | core0 dbr_apr_even_reg   | core1 dbr_apr_even_reg   |
| slave_dbr_odd   | odd         | core0 dbr_apr_odd_reg    | core1 dbr_apr_odd_reg    |

---

## 36-REGISTER INPUT FILE

Before asserting `llr_valid`, all 36 LLR input registers must be loaded and stable.
The registers are held through the duration of `llr_valid=1`.

```
Per core (×2), per recursion unit ({FR,BR,DBR}), per step parity ({odd,even}):

  Sys registers  (5-bit signed, ×12 total):
    core{0,1}_fr_sys_{odd,even}_reg   [4:0]
    core{0,1}_br_sys_{odd,even}_reg   [4:0]
    core{0,1}_dbr_sys_{odd,even}_reg  [4:0]

  Par registers  (5-bit signed, ×12 total):
    core{0,1}_fr_par_{odd,even}_reg   [4:0]
    core{0,1}_br_par_{odd,even}_reg   [4:0]
    core{0,1}_dbr_par_{odd,even}_reg  [4:0]

  Apriori registers (6-bit signed, ×12 total):
    core{0,1}_fr_apr_{odd,even}_reg   [5:0]
    core{0,1}_br_apr_{odd,even}_reg   [5:0]
    core{0,1}_dbr_apr_{odd,even}_reg  [5:0]
```

**Column-to-core mapping (invariant):** col[0] → core0's register; col[1] → core1's register.
This holds for ALL 13 memories in ALL phases. It never changes.

**Half-iteration 0 special case:** All 12 apriori registers are forced to `6'sh0`.
Do NOT read the extrinsic BRAM during half-iteration 0.
The extrinsic BRAM IS still written (to capture L_E1 from the cores for use in half-iteration 1).

---

## LLR FETCH PIPELINES — CYCLE-BY-CYCLE

### Natural phase fetch (half_iter_cnt[0] == 0)

Triggered by `llr_req=1`. Latch `fr_addr`, `br_addr`, `dbr_addr` on the same cycle.

```
Cycle +0 (llr_req cycle):
  Issue simultaneous reads to all 6 Group-A natural BRAMs and both extrinsic BRAMs:
    sys_even_ram  at rows {fr_addr>>1,  br_addr>>1,  dbr_addr>>1}
    sys_odd_ram   at rows {fr_addr>>1,  br_addr>>1,  dbr_addr>>1}
    par1_even_ram at rows {fr_addr>>1,  br_addr>>1,  dbr_addr>>1}
    par1_odd_ram  at rows {fr_addr>>1,  br_addr>>1,  dbr_addr>>1}
    extrinsic_even_ram at rows {fr_addr>>1,  br_addr>>1,  dbr_addr>>1}  [skip if half_iter==0]
    extrinsic_odd_ram  at rows {fr_addr>>1,  br_addr>>1,  dbr_addr>>1}  [skip if half_iter==0]

Cycle +1 (1-cycle BRAM latency):
  All 36 data values arrive.
  Load 36 registers from BRAM outputs:
    core0_fr_sys_even_reg  ← sys_even_ram[fr_addr>>1][col0]  ... etc.
  If half_iter_cnt==0: override all 12 apr_regs to 6'sh0.
  Assert llr_valid=1.
```

**N_FETCH_NATURAL = 1 cycle.** llr_valid is asserted on cycle +1.

### Interleaved phase fetch (half_iter_cnt[0] == 1)

Triggered by `llr_req=1`. Latch `fr_addr`, `br_addr`, `dbr_addr` on the same cycle.

```
Cycle +0 (llr_req cycle):
  Issue reads to 4 interleaved Group-A BRAMs:
    sys_ilv_even_ram at rows {fr_addr>>1,  br_addr>>1,  dbr_addr>>1}
    sys_ilv_odd_ram  at rows {fr_addr>>1,  br_addr>>1,  dbr_addr>>1}
    par2_even_ram    at rows {fr_addr>>1,  br_addr>>1,  dbr_addr>>1}
    par2_odd_ram     at rows {fr_addr>>1,  br_addr>>1,  dbr_addr>>1}
  Issue reads to LUT ROM (qpp_lut):
    addr_fr_even=fr_addr,    addr_fr_odd=fr_addr+1
    addr_br_even=br_addr,    addr_br_odd=br_addr+1
    addr_dbr_even=dbr_addr,  addr_dbr_odd=dbr_addr+1

Cycle +1 (BRAM and LUT outputs arrive):
  Load 24 sys/par registers from interleaved BRAM col[0] and col[1] outputs.
  Feed 6 LUT outputs (pi_fr_even, ..., pi_dbr_odd) — 13-bit each — into 6× master_net.
  master_net is combinatorial: bram_row and perm_bit available immediately.
  Register the 6 perm_bits (perm_fr_even, perm_fr_odd, ..., perm_dbr_odd).
  Issue reads to both extrinsic BRAMs:
    extrinsic_even_ram at rows {master_fr_even.bram_row, master_br_even.bram_row, master_dbr_even.bram_row}
    extrinsic_odd_ram  at rows {master_fr_odd.bram_row,  master_br_odd.bram_row,  master_dbr_odd.bram_row}

Cycle +2 (extrinsic BRAM outputs arrive):
  Feed {col0, col1} from each of the 6 extrinsic BRAM ports into 6× slave_net.
  Use the 6 registered perm_bits to control each slave_net.
  slave_net is combinatorial: all 12 a-priori values available immediately.
  Load 12 apriori registers.
  Assert llr_valid=1.
```

**N_FETCH_INTERLEAVED = 2 cycles.** llr_valid is asserted on cycle +2.

**Critical:** The sys_ilv and par2 BRAMs use `fr_addr>>1`, `br_addr>>1`, `dbr_addr>>1` as
row addresses — **the same formula as natural phase, just different BRAMs.** There is no
separate counter, no LUT lookup, no permutation network for these 4 BRAMs. The pre-permuted
storage arrangement makes the access pattern completely sequential.

---

## EXTRINSIC WRITE-BACK — NATURAL PHASE

On every cycle `llr_out_valid=1` (both cores fire simultaneously with the same `llr_out_addr`):

```verilog
write_row = llr_out_addr[11:1];   // = llr_out_addr >> 1, 11-bit

// Write even step:
extrinsic_even_ram[write_row] <= {core1.l_extr_even, core0.l_extr_even};  // col1=core1, col0=core0

// Write odd step:
extrinsic_odd_ram[write_row]  <= {core1.l_extr_odd,  core0.l_extr_odd};
```

The write row equals the read row used for a-priori in this same half-iteration step
("write where you read"). This invariant ensures the natural sequential read in the
next natural-phase half-iteration finds the correct a-priori values without any permutation.

---

## EXTRINSIC WRITE-BACK — INTERLEAVED PHASE

This is the most subtle part of the design. During the BR backward pass (`llr_out_valid=1`),
the write address is NOT `llr_out_addr>>1`. It is the interleaved BRAM row that was used to
READ the a-priori for this exact step.

**Why "write where you read" works across phase boundaries:**
After an interleaved write-back placing L_E2[k] at row sorted_lo>>1 (where sorted_lo=min(π(k),π(k+3072))),
the next natural-phase sequential read at row r retrieves:
- col[0]: L_E2 for whichever step k had its sorted_lo == 2r → = L_E2[π⁻¹(2r)] = L_A1[2r] ✓

This is automatically correct without an explicit inverse interleaver.

**Mechanism — LUT reuse during backward pass:**
During the backward pass, forward fetch ports on LUT copies 0 (FR) and 2 (DBR) are idle.
The controller reuses these to look up write addresses:

```
1 cycle BEFORE llr_out_valid is expected (pipeline the 1-cycle LUT latency):
  Issue LUT copy-0 port-A read at addr = llr_out_addr[11:0]     → gives π(llr_out_addr)
  Issue LUT copy-0 port-B read at addr = llr_out_addr[11:0] + 1 → gives π(llr_out_addr+1)
  (Use the known 1-cycle pipeline of bcjr_core to align: when llr_out_addr is known from
   the step counter, issue the LUT read immediately.)

On the cycle llr_out_valid=1 (LUT data has arrived):
  wire [12:0] pi_even = lut_out_even_writeback;   // π(llr_out_addr)
  wire [12:0] pi_odd  = lut_out_odd_writeback;    // π(llr_out_addr+1)

  // Derive write row and column assignment (combinatorial, same logic as master_net):
  wire [10:0] wr_row_even  = (pi_even < 13'd3072) ? pi_even[11:1]
                                                   : (pi_even - 13'd3072)[11:1];
  wire [10:0] wr_row_odd   = (pi_odd  < 13'd3072) ? pi_odd[11:1]
                                                   : (pi_odd  - 13'd3072)[11:1];
  wire        wr_perm_even = (pi_even >= 13'd3072);
  wire        wr_perm_odd  = (pi_odd  >= 13'd3072);
```

**Column assignment for the write (mirror of slave_net):**
```verilog
// EVEN step:
if (wr_perm_even == 1'b0)
    extrinsic_even_ram[wr_row_even] <= {core1.l_extr_even, core0.l_extr_even}; // normal
else
    extrinsic_even_ram[wr_row_even] <= {core0.l_extr_even, core1.l_extr_even}; // swapped

// ODD step: identical using wr_row_odd and wr_perm_odd
```

Column swap logic: when perm_bit=1, SISO-0's data came from col[1] during the read, so
it must go back to col[1] during the write. SISO-1's data swaps to col[0] accordingly.
If perm_bit=0, both cores go to their natural slots (core0→col[0], core1→col[1]).

---

## EXTRINSIC BRAM WRITE-READ CONFLICT HANDLING

During the interleaved phase, `llr_out_valid` (write) and the extrinsic read (for the next
a-priori fetch, cycle +1 of the next fetch pipeline) can target the same BRAM row simultaneously.

**Detection:** On cycle +1 of the next fetch pipeline, compare the 3 incoming extrinsic read rows
(master_fr_even.bram_row, master_br_even.bram_row, master_dbr_even.bram_row for the even BRAM,
and analogously for odd) against the current write row (wr_row_even / wr_row_odd).

**Resolution:** If any match, insert a 1-cycle stall: delay the `llr_valid` assertion by 1 cycle.
During the stall, complete the write first, then re-issue the extrinsic BRAM read with correct data.
A 2-bit stall counter handles the extended pipeline.

This edge case occurs with probability ≈ 3/1536 per step (~0.2%) but must be implemented.

---

## TOP-LEVEL FSM

### States

```
ST_IDLE
  Reset half_iter_cnt=0.
  On start=1: assert start_cores=1 for 1 cycle. Go to ST_RUNNING.

ST_RUNNING   [inner loop — entire half-iteration]
  Sub-FSM with states {F_IDLE, F_C0, F_C1, F_C2, F_PROVIDE}:

  F_IDLE:
    Wait for llr_req (from both cores, fires simultaneously).
    On llr_req=1: latch fr/br/dbr_addr. Issue BRAM reads and/or LUT reads per phase.
    Natural: go to F_C0.  Interleaved: go to F_C0.

  F_C0  (natural: BRAM reads issued; interleaved: BRAM + LUT reads issued):
    Natural: go to F_C1 (data arrives next cycle).
    Interleaved: go to F_C1.

  F_C1  (natural: data arrived; interleaved: LUT+BRAM data arrived):
    Natural: load 36 registers. Go to F_PROVIDE.
    Interleaved: load 24 sys/par registers. Compute master_net (comb). Issue extrinsic reads.
                 Register 6 perm_bits. Go to F_C2.

  F_C2  (interleaved only: extrinsic data arrived):
    Extrinsic data → slave_net (comb) → load 12 apr registers.
    Check conflict: if write-row matches any read-row: stall (go to F_STALL).
    Else: go to F_PROVIDE.

  F_STALL (interleaved only, rare):
    Complete the pending write. Re-issue extrinsic read. Go to F_C2B.

  F_C2B:
    Load 12 apr registers from re-issued read. Go to F_PROVIDE.

  F_PROVIDE:
    Assert llr_valid=1 for 1 cycle. Go to F_IDLE.

  Throughout ST_RUNNING, on EVERY cycle:
    If llr_out_valid=1: execute write-back (natural or interleaved based on half_iter_cnt[0]).
    If done=1 (both cores): go to ST_HALF_DONE.

ST_HALF_DONE:
  Increment half_iter_cnt.
  If half_iter_cnt == 4'd11: go to ST_WRITE_LD.
  Else: assert start_cores=1 for 1 cycle. Go to ST_RUNNING.

ST_WRITE_LD:
  Sequentially for row 0..1535:
    ld_ram[row] <= extrinsic_even_ram[row]  (or odd — see note)
  After 1536 cycles: assert decode_done=1. Go to ST_IDLE.

  Note: L_D RAM stores the extrinsic from the final half-iteration (half-iter 10, natural).
  The sign of L_extr for each bit gives the hard decision. Copy both even and odd extrinsic
  into ld_ram — use two sequential passes or a 2-cycle write loop (row → write even on addr 2r,
  write odd on addr 2r+1 within a 3072-cycle loop).
  Alternatively, keep ld_ram as a 1536-row × 2×6-bit BRAM (same structure as extrinsic BRAM)
  and copy directly row-by-row in 1536 cycles.
```

### start_cores timing
`start_cores` is high exactly 1 clock cycle per half-iteration.
It is asserted in ST_IDLE (first half-iteration) and in ST_HALF_DONE (subsequent half-iterations).
It is NEVER asserted while in ST_RUNNING.

### done detection
Both cores assert `done` simultaneously (they are synchronised). The controller checks `done`
combinatorially during every ST_RUNNING cycle. When `done=1`, it transitions to ST_HALF_DONE
in the same clock edge that captures the last `llr_out_valid` write (if any — handle both
occurring together).

---

## TOP-LEVEL MODULE: `turbo_decoder`

```verilog
module turbo_decoder (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,           // pulse 1 cycle to begin decoding

    // Host load interface — write all 8 Group-A BRAMs before asserting start
    input  wire        load_en,
    input  wire [2:0]  load_bram_sel,   // encoding: see table below
    input  wire [10:0] load_addr,       // BRAM row [0..1535]
    input  wire [9:0]  load_data,       // {col1[4:0], col0[4:0]} = {SISO-1_val, SISO-0_val}

    // Result interface
    output wire        decode_done,     // pulses 1 cycle when decoding complete
    input  wire [10:0] ld_rd_addr,      // row to read from ld_ram (external, after decode_done)
    output wire [11:0] ld_rd_data       // {col1[5:0], col0[5:0]} = {SISO-1_L_extr, SISO-0_L_extr}
);
```

### load_bram_sel Encoding Table

| load_bram_sel | BRAM written         |
|---------------|----------------------|
| 3'b000        | sys_odd_ram          |
| 3'b001        | sys_even_ram         |
| 3'b010        | par1_odd_ram         |
| 3'b011        | par1_even_ram        |
| 3'b100        | sys_ilv_odd_ram      |
| 3'b101        | sys_ilv_even_ram     |
| 3'b110        | par2_odd_ram         |
| 3'b111        | par2_even_ram        |

During `load_en=1`, the decoder must be in ST_IDLE (not decoding). The 8 BRAMs share the
load_addr and load_data buses; load_bram_sel steers the write enable.

---

## MODULE FILE LIST

| # | File name              | Purpose                                                         |
|---|------------------------|-----------------------------------------------------------------|
| 1 | `qpp_lut.v`            | 3-copy LUT ROM, 6×13-bit read ports, 3072 entries              |
| 2 | `master_net.v`         | Combinatorial sorter: 13-bit inputs, 11-bit bram_row, perm_bit |
| 3 | `slave_net.v`          | Combinatorial switch: col0/col1 → out_siso0/out_siso1          |
| 4 | `input_bram.v`         | Parameterised Group-A BRAM (3R+1W, 2×5-bit words, 1536 rows)  |
| 5 | `extrinsic_bram.v`     | Parameterised Group-B BRAM (3R+1W, 2×6-bit words, 1536 rows)  |
| 6 | `ld_bram.v`            | Single-port 1536×12-bit output BRAM                            |
| 7 | `turbo_decoder.v`      | Top-level: instantiate all modules, FSM, all wiring            |
| 8 | `qpp_6144_gen.py`      | Generate qpp_6144.hex: 3072 lines, each 4 hex digits           |
| 9 | `gen_test_vectors.py`  | Generate hex files for all 8 Group-A BRAMs                     |
|10 | `tb_turbo_decoder.v`   | Full-system testbench                                          |
|11 | `integration_notes.md` | Timing, write-back proof, conflict analysis, BRAM estimate     |

---

## IMPLEMENTATION REQUIREMENTS

### Verilog-2001 (strictly enforced)
- `always @(posedge clk or negedge rst_n)` for all clocked logic.
- `always @(*)` for all combinatorial logic. Never `always @(a or b)` explicit sensitivity.
- No latches. Every `case` in a combinatorial block must have a `default` branch.
- `parameter` for module-level constants. `localparam` for derived constants inside modules.
- All port directions explicitly typed: `input wire`, `output reg`, `output wire`.
- No SystemVerilog: no `logic`, `always_ff`, `always_comb`, interfaces, structs, or packages.

### BRAM inference for Zynq-7010 / Vivado
- Synchronous reads only — 1-cycle registered output. No async/transparent reads.
- True dual-port: two independent `always @(posedge clk)` read/write blocks on shared array.
- `(* ram_style = "block" *)` attribute on all arrays intended as block RAM.
- ROM initialisation via `$readmemh` in `initial` blocks (legal for Vivado ROM inference).
- Do NOT use `initial` blocks to initialise read/write BRAMs.

### Reset
- All FSM state, counters, perm_bit registers, and LLR registers reset synchronously on `rst_n=0`.
- BRAM content arrays are NOT reset (undefined until loaded, which is fine).
- `half_iter_cnt [3:0]` resets to 0. Sub-FSM `fetch_state` resets to F_IDLE.

---

## CORRECTNESS CHECKLIST — INSPECT EVERY ITEM BEFORE FINALISING

1. **13-bit LUT outputs:** Search entire codebase for `[11:0]` on any wire/reg that carries π(k).
   Every such signal must be `[12:0]`. There must be NO 12-bit signals in the LUT output path.

2. **SISO-1 formula:** Verify the ternary: π(743)=743 → SISO-1=3815; π(5109)=5109 → SISO-1=2037.
   No XOR anywhere in the code path from LUT output to master_net output.

3. **Interleaved sys/par addressing:** The reads to sys_ilv_* and par2_* BRAMs use
   `fr_addr>>1`, `br_addr>>1`, `dbr_addr>>1` as row addresses. If any separate counter
   appears as the row address for these BRAMs, it is wrong.

4. **bram_row bounds:** For any valid 13-bit π(k), `bram_row = sorted_lo[11:1]` ∈ [0..1535].
   sorted_lo < 3072, so sorted_lo[12]=0 and sorted_lo[11:1] ≤ 1535. Verified.

5. **Natural write-back:** After any natural half-iteration, confirm:
   extrinsic_even_ram[k>>1][col0] = L_E produced by core0 for local step k.
   The next natural read at row k>>1 retrieves this as a-priori. Trivially correct.

6. **Interleaved write-back correctness:** For step k=1, π(1)=743, SISO-0's perm_bit=0
   (743 < 3072). Write lands at row 371 (743>>1), col[0]. Next natural-phase read at row 371
   col[0] retrieves this value as a-priori for SISO-0 step 2·371=742... wait, that's wrong.
   
   Correction: row 371 holds step 2·371=742 in _even and step 743 in _odd.
   π⁻¹(743) = 1 (since π(1)=743). Natural-phase step 743 = local step 743 of SISO-0.
   Natural-phase read of row 371 for step 743 (odd) uses `extrinsic_odd_ram[371][col0]`.
   This is where L_E2[1] was written (since π(1)=743, perm_bit=0 → col[0], bram_row=371).
   Next natural half-iteration: apriori for SISO-0 local step 743 = extrinsic_odd_ram[371][col0] = L_E2[1] = L_A1[743] ✓

7. **Interleaved write column swap:** When wr_perm=1, core0's L_extr goes to col[1] and
   core1's goes to col[0]. Verify one example: π(k)=5109 (perm_bit=1), SISO-0 data → col[1].
   Next natural read at that row: col[1] = L_E_core0 = L_A for SISO-1's segment. But wait —
   this requires careful analysis. Trace: sorted_lo=5109-3072=2037, bram_row=1018.
   Since perm_bit=1, SISO-0 was actually SISO-1's physical segment. So core0's value crosses
   into col[1], which in the next natural phase is read as SISO-1's a-priori. The step mapping
   confirms: SISO-1's local step for global π address 2037+3072=5109 is step k, decoded
   correctly as a-priori for SISO-1 in the next half-iteration. Trace this fully in integration_notes.md.

8. **Half-iteration 0:** All apriori registers are 6'sh0. Extrinsic BRAM is NOT read.
   But extrinsic BRAM IS written from `llr_out_valid` outputs of the cores.

9. **done handling:** Both cores assert `done` on the same cycle. Controller goes to ST_HALF_DONE
   without losing the final `llr_out_valid` write if both occur simultaneously.

10. **start_cores:** Asserted for exactly 1 cycle. Never high during ST_RUNNING.

11. **Stall logic completeness:** When `llr_out_valid=1` during interleaved phase fetch,
    compare wr_row_even against all 3 even-BRAM read rows AND wr_row_odd against all 3
    odd-BRAM read rows. That is 6 comparisons. Any match triggers stall.

12. **ld_ram content:** Written AFTER half-iteration 10 completes (ST_WRITE_LD). Contains
    L_extr values from half-iteration 10. The sign gives the hard bit decision.

13. **load_bram_sel coverage:** All 8 decode values (3'b000..3'b111) map to unique BRAMs.
    No default case activates during decoding (load_en must be gated by ST_IDLE).

---

## VERIFICATION ARTIFACTS

### `qpp_6144_gen.py`
```python
K, f1, f2 = 6144, 263, 480
with open("qpp_6144.hex", "w") as f:
    for k in range(K // 2):   # 3072 entries for SISO-0 segment
        pi_k = (f1 * k + f2 * k * k) % K
        f.write(f"{pi_k:04x}\n")   # 4 hex digits, 13-bit value fits in 4 hex digits
# Verify: pi[1]=743 (0x02E7), pi[3072//2-1] = check manually
```

### `gen_test_vectors.py`
Generates 9 hex files. Each file has 1536 lines. Each line is one row in hex.
For Group-A 5-bit BRAMs: row = 10-bit value = `{col1[4:0], col0[4:0]}`.
```python
import random
K, S, f1, f2 = 6144, 3072, 263, 480

def qpp(k): return (f1*k + f2*k*k) % K

pi = [qpp(k) for k in range(K)]

def rand_llr(): return random.randint(-16, 15) & 0x1F  # 5-bit signed as unsigned

L_s  = [rand_llr() for _ in range(K)]
L_p1 = [rand_llr() for _ in range(K)]
L_p2 = [rand_llr() for _ in range(K)]

def write_bram(fname, rows):
    with open(fname, "w") as f:
        for c0, c1 in rows:
            f.write(f"{((c1 & 0x1F) << 5) | (c0 & 0x1F):03x}\n")

# Natural BRAMs: SISO-0=global step r, SISO-1=global step r+S
write_bram("sys_even_ram.hex",  [(L_s[2*r],     L_s[2*r+S])     for r in range(S//2)])
write_bram("sys_odd_ram.hex",   [(L_s[2*r+1],   L_s[2*r+1+S])   for r in range(S//2)])
write_bram("par1_even_ram.hex", [(L_p1[2*r],    L_p1[2*r+S])    for r in range(S//2)])
write_bram("par1_odd_ram.hex",  [(L_p1[2*r+1],  L_p1[2*r+1+S])  for r in range(S//2)])

# Interleaved BRAMs: col0=L_s[pi(2r)], col1=L_s[(pi(2r)+S)%K]
write_bram("sys_ilv_even_ram.hex", [(L_s[pi[2*r]],           L_s[(pi[2*r]+S)%K])           for r in range(S//2)])
write_bram("sys_ilv_odd_ram.hex",  [(L_s[pi[2*r+1]],         L_s[(pi[2*r+1]+S)%K])         for r in range(S//2)])
write_bram("par2_even_ram.hex",    [(L_p2[pi[2*r]],           L_p2[(pi[2*r]+S)%K])           for r in range(S//2)])
write_bram("par2_odd_ram.hex",     [(L_p2[pi[2*r+1]],         L_p2[(pi[2*r+1]+S)%K])         for r in range(S//2)])
```

### `tb_turbo_decoder.v`
- Preload all 8 Group-A BRAMs using `$readmemh` on the generated hex files.
- Assert `start` for 1 cycle. Simulate until `decode_done`.
- Assertions (use `$display` + comparison):
  - Both cores' `done` fires exactly 11 times total.
  - `half_iter_cnt` inside turbo_decoder increments from 0 to 11.
  - `decode_done` fires exactly once.
  - After `decode_done`, read all 1536 ld_ram rows and write to file.
- Enable `$dumpvars` scoped to at minimum: FSM state, perm_bits, write-back addresses,
  extrinsic BRAM write enables — to verify the interleaved pipeline in waveform.

### `integration_notes.md`
Required sections:
1. Fetch pipeline timing diagram (ASCII art, cycle-numbered) for both phases.
2. Concrete write-back trace for k=1 (π(1)=743) through natural and interleaved half-iterations.
3. Concrete write-back trace for a case where perm_bit=1 (e.g. k where π(k)≥3072).
4. Port conflict analysis: enumerate all 6 possible conflict pairs and show the stall detects each.
5. BRAM resource estimate: count RAMB36E1 primitives for all 13 logical memories.

---

## SUMMARY DATA FLOW — QUICK REFERENCE

### Natural phase:
```
llr_req → latch {fr,br,dbr}_addr
  +0: issue reads: sys_odd/even, par1_odd/even, extrinsic_odd/even at {fr,br,dbr}_addr>>1
  +1: data arrives → load 36 registers → assert llr_valid
      (half_iter==0: force apr regs to 0)

llr_out_valid: write extrinsic_{even,odd}_ram[llr_out_addr>>1]
               ← {core1.l_extr_{even,odd}, core0.l_extr_{even,odd}}
```

### Interleaved phase:
```
llr_req → latch {fr,br,dbr}_addr
  +0: issue reads: sys_ilv_odd/even, par2_odd/even at {fr,br,dbr}_addr>>1  [same address formula]
      issue LUT reads: {fr,fr+1,br,br+1,dbr,dbr+1}_addr
  +1: sys/par data → 24 regs
      LUT data → 6× master_net (comb) → {6×bram_row, 6×perm_bit}
      register 6 perm_bits
      issue extrinsic reads at 6 master_net bram_rows
  +2: extrinsic data → 6× slave_net (comb, uses registered perm_bits) → 12 apr regs
      assert llr_valid (unless stall)

llr_out_valid:
  (issued 1 cycle prior) LUT read at {llr_out_addr, llr_out_addr+1} on free LUT ports
  (same cycle as valid) derive wr_row_even, wr_row_odd, wr_perm_even, wr_perm_odd
  write extrinsic_{even,odd}_ram[wr_row] with col swap per wr_perm
```

---

## DO NOT — ABSOLUTE PROHIBITIONS

1. **Do NOT** modify, rewrite, or restructure any existing file. `bcjr_core` is frozen.
2. **Do NOT** declare any π(k) wire or register as `[11:0]`. All must be `[12:0]` (13-bit).
3. **Do NOT** use XOR to derive SISO-1's interleaved address. Use the ternary ±3072 formula.
4. **Do NOT** implement more than a single 2-input sorter in each master_net. The 8-parallel
   Batcher network from the paper is for N=8 and is NOT needed here.
5. **Do NOT** route sys_ilv or par2 LLRs through master_net or slave_net. They access the BRAM
   directly using `fr/br/dbr_addr>>1`. Only the extrinsic / a-priori path uses master-slave.
6. **Do NOT** use a separate counter as the row address for sys_ilv or par2 BRAMs.
   The row address is always derived from the core's `fr_addr`, `br_addr`, or `dbr_addr`.
7. **Do NOT** implement the QPP address recursion (Eq. 6 from the paper) in hardware.
   The LUT ROM is the only interleaver hardware needed.
8. **Do NOT** use asynchronous (combinatorial) BRAM reads anywhere. All reads are registered.
9. **Do NOT** assert `start_cores` during ST_RUNNING. One pulse per half-iteration, precisely.
10. **Do NOT** skip the write-back during the final half-iteration. ld_ram is populated from
    the extrinsic BRAM state after all 11 write-backs have completed.
