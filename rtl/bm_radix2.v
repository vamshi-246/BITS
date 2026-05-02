//==============================================================================
// Module: bm_radix2
// Description: Combinational. Selects from the 4 pre-processed values to form
//              16 radix-2 branch metrics per trellis step (32 total for both
//              odd and even steps). Wiring hardcoded from Section 3.3.
//
// With BPSK substitution: PREPROC_W = 7 = BM_R2_W, so no sign extension needed.
// Each BM_R2 is simply a direct selection from the 4 PRE values.
//
// Reference: Studer et al., IEEE JSSC 2011
//==============================================================================
module bm_radix2 (
    input  wire signed [6:0] pre_odd_0,
    input  wire signed [6:0] pre_odd_1,
    input  wire signed [6:0] pre_odd_2,
    input  wire signed [6:0] pre_odd_3,
    input  wire signed [6:0] pre_even_0,
    input  wire signed [6:0] pre_even_1,
    input  wire signed [6:0] pre_even_2,
    input  wire signed [6:0] pre_even_3,
    // Odd step: bm_r2_odd[dest][pred_idx], flattened as dest*2+pred_idx (0..15)
    output wire signed [6:0] bm_r2_odd_0,
    output wire signed [6:0] bm_r2_odd_1,
    output wire signed [6:0] bm_r2_odd_2,
    output wire signed [6:0] bm_r2_odd_3,
    output wire signed [6:0] bm_r2_odd_4,
    output wire signed [6:0] bm_r2_odd_5,
    output wire signed [6:0] bm_r2_odd_6,
    output wire signed [6:0] bm_r2_odd_7,
    output wire signed [6:0] bm_r2_odd_8,
    output wire signed [6:0] bm_r2_odd_9,
    output wire signed [6:0] bm_r2_odd_10,
    output wire signed [6:0] bm_r2_odd_11,
    output wire signed [6:0] bm_r2_odd_12,
    output wire signed [6:0] bm_r2_odd_13,
    output wire signed [6:0] bm_r2_odd_14,
    output wire signed [6:0] bm_r2_odd_15,
    // Even step: identical mapping with pre_even values
    output wire signed [6:0] bm_r2_even_0,
    output wire signed [6:0] bm_r2_even_1,
    output wire signed [6:0] bm_r2_even_2,
    output wire signed [6:0] bm_r2_even_3,
    output wire signed [6:0] bm_r2_even_4,
    output wire signed [6:0] bm_r2_even_5,
    output wire signed [6:0] bm_r2_even_6,
    output wire signed [6:0] bm_r2_even_7,
    output wire signed [6:0] bm_r2_even_8,
    output wire signed [6:0] bm_r2_even_9,
    output wire signed [6:0] bm_r2_even_10,
    output wire signed [6:0] bm_r2_even_11,
    output wire signed [6:0] bm_r2_even_12,
    output wire signed [6:0] bm_r2_even_13,
    output wire signed [6:0] bm_r2_even_14,
    output wire signed [6:0] bm_r2_even_15
);

    // -------------------------------------------------------------------------
    // Local parameters
    // -------------------------------------------------------------------------
    localparam PREPROC_W = 7;   // BPSK: 7 bits
    localparam BM_R2_W   = 7;  // Same as PREPROC_W — direct assignment, no extension

    // -------------------------------------------------------------------------
    // ODD STEP — Direct selection from PRE values (no sign extension needed)
    //
    // From Section 3.3 Radix-2 Predecessor Table:
    //   Dest | Pred_0  PRE_idx_0 | Pred_1  PRE_idx_1
    //   000  |   000      0      |   001      3
    //   001  |   010      1      |   011      2
    //   010  |   100      2      |   101      1
    //   011  |   110      3      |   111      0
    //   100  |   000      3      |   001      0
    //   101  |   010      2      |   011      1
    //   110  |   100      1      |   101      2
    //   111  |   110      0      |   111      3
    // -------------------------------------------------------------------------
    assign bm_r2_odd_0  = pre_odd_0;  // dest=S0, S0→S0: u=0,p=0 → PRE[0]
    assign bm_r2_odd_1  = pre_odd_3;  // dest=S0, S1→S0: u=1,p=1 → PRE[3]
    assign bm_r2_odd_2  = pre_odd_2;  // dest=S1, S2→S1: u=1,p=0 → PRE[2]
    assign bm_r2_odd_3  = pre_odd_1;  // dest=S1, S3→S1: u=0,p=1 → PRE[1]
    assign bm_r2_odd_4  = pre_odd_1;  // dest=S2, S4→S2: u=0,p=1 → PRE[1]
    assign bm_r2_odd_5  = pre_odd_2;  // dest=S2, S5→S2: u=1,p=0 → PRE[2]
    assign bm_r2_odd_6  = pre_odd_3;  // dest=S3, S6→S3: u=1,p=1 → PRE[3]
    assign bm_r2_odd_7  = pre_odd_0;  // dest=S3, S7→S3: u=0,p=0 → PRE[0]
    assign bm_r2_odd_8  = pre_odd_3;  // dest=S4, S0→S4: u=1,p=1 → PRE[3]
    assign bm_r2_odd_9  = pre_odd_0;  // dest=S4, S1→S4: u=0,p=0 → PRE[0]
    assign bm_r2_odd_10 = pre_odd_1;  // dest=S5, S2→S5: u=0,p=1 → PRE[1]
    assign bm_r2_odd_11 = pre_odd_2;  // dest=S5, S3→S5: u=1,p=0 → PRE[2]
    assign bm_r2_odd_12 = pre_odd_2;  // dest=S6, S4→S6: u=1,p=0 → PRE[2]
    assign bm_r2_odd_13 = pre_odd_1;  // dest=S6, S5→S6: u=0,p=1 → PRE[1]
    assign bm_r2_odd_14 = pre_odd_0;  // dest=S7, S6→S7: u=0,p=0 → PRE[0]
    assign bm_r2_odd_15 = pre_odd_3;  // dest=S7, S7→S7: u=1,p=1 → PRE[3]

    // -------------------------------------------------------------------------
    // EVEN STEP — Identical structure, using pre_even values
    // -------------------------------------------------------------------------
    assign bm_r2_even_0  = pre_even_0;  // dest=S0, S0→S0: u=0,p=0 → PRE[0]
    assign bm_r2_even_1  = pre_even_3;  // dest=S0, S1→S0: u=1,p=1 → PRE[3]
    assign bm_r2_even_2  = pre_even_2;  // dest=S1, S2→S1: u=1,p=0 → PRE[2]
    assign bm_r2_even_3  = pre_even_1;  // dest=S1, S3→S1: u=0,p=1 → PRE[1]
    assign bm_r2_even_4  = pre_even_1;  // dest=S2, S4→S2: u=0,p=1 → PRE[1]
    assign bm_r2_even_5  = pre_even_2;  // dest=S2, S5→S2: u=1,p=0 → PRE[2]
    assign bm_r2_even_6  = pre_even_3;  // dest=S3, S6→S3: u=1,p=1 → PRE[3]
    assign bm_r2_even_7  = pre_even_0;  // dest=S3, S7→S3: u=0,p=0 → PRE[0]
    assign bm_r2_even_8  = pre_even_3;  // dest=S4, S0→S4: u=1,p=1 → PRE[3]
    assign bm_r2_even_9  = pre_even_0;  // dest=S4, S1→S4: u=0,p=0 → PRE[0]
    assign bm_r2_even_10 = pre_even_1;  // dest=S5, S2→S5: u=0,p=1 → PRE[1]
    assign bm_r2_even_11 = pre_even_2;  // dest=S5, S3→S5: u=1,p=0 → PRE[2]
    assign bm_r2_even_12 = pre_even_2;  // dest=S6, S4→S6: u=1,p=0 → PRE[2]
    assign bm_r2_even_13 = pre_even_1;  // dest=S6, S5→S6: u=0,p=1 → PRE[1]
    assign bm_r2_even_14 = pre_even_0;  // dest=S7, S6→S7: u=0,p=0 → PRE[0]
    assign bm_r2_even_15 = pre_even_3;  // dest=S7, S7→S7: u=1,p=1 → PRE[3]

endmodule
