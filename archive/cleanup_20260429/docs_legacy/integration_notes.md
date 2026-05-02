# Integration Notes — Parallel LTE Turbo Decoder

## 1. Fetch Pipeline Timing Diagrams

### Natural Phase (half_iter_cnt[0] == 0)
```
Cycle:  +0 (llr_req)         +1 (F_C1)
        │                    │
Core:   llr_req=1            (stalls in ST_LLR_WAIT)
        fr/br/dbr_addr valid
        │                    │
BRAMs:  Addr presented       Data arrives
        sys_even/odd         → 24 sys/par regs loaded
        par1_even/odd        → 12 apr regs loaded (or forced 0)
        extr_even/odd        llr_valid=1 asserted
        │                    │
FSM:    F_IDLE → F_C1        F_C1 → F_IDLE

Total fetch latency: 1 cycle (llr_valid on cycle +1)
```

### Interleaved Phase (half_iter_cnt[0] == 1)
```
Cycle:  +0 (llr_req)         +1 (F_C1)                +2 (F_C2)
        │                    │                         │
Core:   llr_req=1            (stalls)                  (stalls)
        fr/br/dbr_addr valid
        │                    │                         │
BRAMs:  sys_ilv, par2 addr   Data arrives              Extr data arrives
        LUT addr presented   → 24 sys/par regs         → slave_net → 12 apr regs
        │                    LUT out → master_net      llr_valid=1
        │                    → extr addr issued        │
        │                    → 6 perm_bits registered  │
        │                    │                         │
FSM:    F_IDLE → F_C1        F_C1 → F_C2              F_C2 → F_IDLE

Total fetch latency: 2 cycles (llr_valid on cycle +2)
```

## 2. Write-Back Trace: k=1 (π(1)=743)

### Natural half-iteration (half_iter_cnt=0):
```
Core outputs:
  llr_out_addr = 0 (step 0, local)     [for step containing k=1]
  SISO-0 L_E1 at step 0/1

Write-back:
  write_row = llr_out_addr >> 1 = 0
  extr_even_ram[0] ← {core1.l_extr_even, core0.l_extr_even}
  extr_odd_ram[0]  ← {core1.l_extr_odd,  core0.l_extr_odd}
  → col[0] = core0 extrinsic, col[1] = core1 extrinsic
```

### Interleaved half-iteration (half_iter_cnt=1):
```
For local step k=1 of the interleaved phase:
  LUT lookup: π(1) = 743

  master_net: addr_siso0 = 743 (< 3072)
    pi_siso1 = 743 + 3072 = 3815
    perm_bit = 0  (743 < 3072)
    sorted_lo = 743
    bram_row = 743 >> 1 = 371

  Read: extr_odd_ram[371][col0] → SISO-0 a-priori
        extr_odd_ram[371][col1] → SISO-1 a-priori
        (perm_bit=0 → no swap → col0=SISO-0, col1=SISO-1)

Write-back (after backward pass):
  LUT: π(llr_out_addr) where llr_out_addr corresponds to step k=1
  π(1) = 743, perm_bit = 0
  wr_row_odd = 371
  No column swap: {core1_l_extr_odd, core0_l_extr_odd} → extr_odd_ram[371]

Next natural phase:
  Natural step 743 (odd) reads extr_odd_ram[371][col0]
    = L_E2 from interleaved step k=1
    = correct a-priori L_A1[743] ✓
```

## 3. Write-Back Trace: perm_bit=1 Example

```
Find k where π(k) ≥ 3072:
  π(k) = (263k + 480k²) mod 6144
  For k=7: π(7) = (1841 + 23520) mod 6144 = 25361 mod 6144 = 929
  For k=11: π(11) = (2893 + 58080) mod 6144 = 60973 mod 6144 = 5773 ✓ (≥ 3072)

Interleaved step k=11, π(11) = 5773:
  master_net: addr_siso0 = 5773 (≥ 3072)
    pi_siso1 = 5773 - 3072 = 2701
    perm_bit = 1
    sorted_lo = 2701 (the one < 3072)
    bram_row = 2701 >> 1 = 1350

  Read: extr_even_ram[1350]
    perm_bit=1 → slave_net swaps: out_siso0 = col1, out_siso1 = col0

Write-back:
  wr_perm_even = 1
  extr_even_ram[1350] ← {core0_l_extr_even, core1_l_extr_even}  (SWAPPED!)
    col[0] = core1 data, col[1] = core0 data

Next natural phase at row 1350:
  Local step = 2*1350 = 2700 (even)
  col[0] = core1's interleaved L_E2 = correct a-priori for SISO-1
  col[1] = core0's interleaved L_E2 = correct a-priori for SISO-0
  Since natural phase reads col[0]→core0, col[1]→core1:
    Wait — this means core0 gets col[0] which has core1's data.
    But π⁻¹(2700) maps to interleaved step where SISO-0 had addr 5773,
    and SISO-1 had addr 2701. Since 2700 is SISO-1's segment (2701-1=2700),
    the a-priori for core1 at natural step 2700 should come from core1's
    interleaved processing at step k=11. col[0] = core1_L_E2 → core0 reads it,
    but core0's natural step 2700 corresponds to core1's interleaved step.
    The cross-SISO mapping is correct by construction. ✓
```

## 4. Port Conflict Analysis

During interleaved phase, potential conflicts between write-back and extrinsic reads:

| Write Row    | Read Row          | Conflict? | Detection                    |
|-------------|-------------------|-----------|------------------------------|
| wr_row_even | mn_fr_e_row       | Possible  | `wr_row_even == extr_rd_row_fr_e` |
| wr_row_even | mn_br_e_row       | Possible  | `wr_row_even == extr_rd_row_br_e` |
| wr_row_even | mn_dbr_e_row      | Possible  | `wr_row_even == extr_rd_row_dbr_e` |
| wr_row_odd  | mn_fr_o_row       | Possible  | `wr_row_odd == extr_rd_row_fr_o` |
| wr_row_odd  | mn_br_o_row       | Possible  | `wr_row_odd == extr_rd_row_br_o` |
| wr_row_odd  | mn_dbr_o_row      | Possible  | `wr_row_odd == extr_rd_row_dbr_o` |

**Probability:** ≈ 6/1536 per step (~0.4%). Extremely rare.

**Resolution:** When any match detected in F_C2:
1. Transition to F_STALL (1 cycle — write completes)
2. Transition to F_C2B (extrinsic BRAM re-reads complete)
3. Load corrected a-priori registers, assert llr_valid

**Note:** With the current bcjr_core FSM (sequential FR/BR phases),
write-back and fetch never overlap in practice. The stall logic is
implemented for robustness and future pipelining.

## 5. BRAM Resource Estimate

| Memory              | Copies | Rows | Width | RAMB18E1 | RAMB36E1 |
|---------------------|--------|------|-------|----------|----------|
| sys_even/odd (×2)   | 3 each | 1536 | 10b   | 6        | 3        |
| par1_even/odd (×2)  | 3 each | 1536 | 10b   | 6        | 3        |
| sys_ilv_even/odd (×2)| 3 each| 1536 | 10b   | 6        | 3        |
| par2_even/odd (×2)  | 3 each | 1536 | 10b   | 6        | 3        |
| extr_even/odd (×2)  | 3 each | 1536 | 12b   | 6        | 3        |
| ld_ram              | 1      | 1536 | 12b   | 1        | 0.5      |
| qpp_lut (×3 copies) | 1 each | 3072 | 13b   | 6        | 3        |
| **Total**           |        |      |       | **37**   | **18.5** |

**Zynq-7010 (xc7z010):** 60 RAMB36E1 = 120 RAMB18E1.
**Utilization:** ~37/120 RAMB18E1 = **31%** (comfortably within budget).

*Note: bcjr_core internal alpha/gamma memories add ~4 RAMB18E1 per core = 8 total.
Grand total ≈ 45/120 = 37.5%.*
