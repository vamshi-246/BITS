//==============================================================================
// Module: bm_radix4
// Description: Combinational. Combines pairs of radix-2 branch metrics to form
//              32 radix-4 branch metrics (4 per destination state).
//              Wiring directly from Section 3.4 radix-4 predecessor table.
// Reference: Studer et al., IEEE JSSC 2011
//==============================================================================
module bm_radix4 (
    // Radix-2 BMs flattened: index = dest*2 + pred_idx (0..15)
    input  wire signed [6:0] bm_r2_odd_0,   bm_r2_odd_1,
    input  wire signed [6:0] bm_r2_odd_2,   bm_r2_odd_3,
    input  wire signed [6:0] bm_r2_odd_4,   bm_r2_odd_5,
    input  wire signed [6:0] bm_r2_odd_6,   bm_r2_odd_7,
    input  wire signed [6:0] bm_r2_odd_8,   bm_r2_odd_9,
    input  wire signed [6:0] bm_r2_odd_10,  bm_r2_odd_11,
    input  wire signed [6:0] bm_r2_odd_12,  bm_r2_odd_13,
    input  wire signed [6:0] bm_r2_odd_14,  bm_r2_odd_15,
    input  wire signed [6:0] bm_r2_even_0,  bm_r2_even_1,
    input  wire signed [6:0] bm_r2_even_2,  bm_r2_even_3,
    input  wire signed [6:0] bm_r2_even_4,  bm_r2_even_5,
    input  wire signed [6:0] bm_r2_even_6,  bm_r2_even_7,
    input  wire signed [6:0] bm_r2_even_8,  bm_r2_even_9,
    input  wire signed [6:0] bm_r2_even_10, bm_r2_even_11,
    input  wire signed [6:0] bm_r2_even_12, bm_r2_even_13,
    input  wire signed [6:0] bm_r2_even_14, bm_r2_even_15,
    // Radix-4 BMs: bm_r4[dest][i], flattened as dest*4+i (0..31)
    output wire signed [7:0] bm_r4_0,  bm_r4_1,  bm_r4_2,  bm_r4_3,   // dest=000
    output wire signed [7:0] bm_r4_4,  bm_r4_5,  bm_r4_6,  bm_r4_7,   // dest=001
    output wire signed [7:0] bm_r4_8,  bm_r4_9,  bm_r4_10, bm_r4_11,  // dest=010
    output wire signed [7:0] bm_r4_12, bm_r4_13, bm_r4_14, bm_r4_15,  // dest=011
    output wire signed [7:0] bm_r4_16, bm_r4_17, bm_r4_18, bm_r4_19,  // dest=100
    output wire signed [7:0] bm_r4_20, bm_r4_21, bm_r4_22, bm_r4_23,  // dest=101
    output wire signed [7:0] bm_r4_24, bm_r4_25, bm_r4_26, bm_r4_27,  // dest=110
    output wire signed [7:0] bm_r4_28, bm_r4_29, bm_r4_30, bm_r4_31   // dest=111
);

    // -------------------------------------------------------------------------
    // Local parameters
    // -------------------------------------------------------------------------
    localparam BM_R2_W = 7;
    localparam BM_R4_W = 8;

    // -------------------------------------------------------------------------
    // Radix-4 BM = bm_r2_odd[s'][pred_of_s''_in_s'] + bm_r2_even[dest][pred_of_s'_in_dest]
    //
    // From Section 3.4 table:
    // bm_r2_odd index  = s'*2 + pred_idx(s'' in s')
    // bm_r2_even index = dest*2 + pred_idx(s' in dest)
    //
    // Cross-referencing Section 3.3 radix-2 predecessor table to get pred_idx:
    //   For odd step, dest=s': check which pred_idx s'' occupies
    //   For even step, dest=s: check which pred_idx s' occupies
    // -------------------------------------------------------------------------

    // --- State 000 (dest=0) ---
    // s''=000, s'=000: odd[s'=0][pred_idx(000 in 000)] + even[dest=0][pred_idx(000 in 000)]
    //   In R2 table: dest=000, pred_0=000 → pred_idx=0; dest=000, pred_0=000 → pred_idx=0
    assign bm_r4_0  = $signed(bm_r2_odd_0)  + $signed(bm_r2_even_0);  // odd[0][0] + even[0][0]
    // s''=001, s'=000: odd[0][pred_idx(001 in 000)=1] + even[0][0]
    assign bm_r4_1  = $signed(bm_r2_odd_1)  + $signed(bm_r2_even_0);  // odd[0][1] + even[0][0]
    // s''=010, s'=001: odd[1][pred_idx(010 in 001)=0] + even[0][pred_idx(001 in 000)=1]
    assign bm_r4_2  = $signed(bm_r2_odd_2)  + $signed(bm_r2_even_1);  // odd[1][0] + even[0][1]
    // s''=011, s'=001: odd[1][pred_idx(011 in 001)=1] + even[0][1]
    assign bm_r4_3  = $signed(bm_r2_odd_3)  + $signed(bm_r2_even_1);  // odd[1][1] + even[0][1]

    // --- State 001 (dest=1) ---
    // s''=100, s'=010: odd[2][pred_idx(100 in 010)=0] + even[1][pred_idx(010 in 001)=0]
    assign bm_r4_4  = $signed(bm_r2_odd_4)  + $signed(bm_r2_even_2);  // odd[2][0] + even[1][0]
    // s''=101, s'=010: odd[2][pred_idx(101 in 010)=1] + even[1][0]
    assign bm_r4_5  = $signed(bm_r2_odd_5)  + $signed(bm_r2_even_2);  // odd[2][1] + even[1][0]
    // s''=110, s'=011: odd[3][pred_idx(110 in 011)=0] + even[1][pred_idx(011 in 001)=1]
    assign bm_r4_6  = $signed(bm_r2_odd_6)  + $signed(bm_r2_even_3);  // odd[3][0] + even[1][1]
    // s''=111, s'=011: odd[3][pred_idx(111 in 011)=1] + even[1][1]
    assign bm_r4_7  = $signed(bm_r2_odd_7)  + $signed(bm_r2_even_3);  // odd[3][1] + even[1][1]

    // --- State 010 (dest=2) ---
    // s''=000, s'=100: odd[4][pred_idx(000 in 100)=0] + even[2][pred_idx(100 in 010)=0]
    assign bm_r4_8  = $signed(bm_r2_odd_8)  + $signed(bm_r2_even_4);  // odd[4][0] + even[2][0]
    // s''=001, s'=100: odd[4][pred_idx(001 in 100)=1] + even[2][0]
    assign bm_r4_9  = $signed(bm_r2_odd_9)  + $signed(bm_r2_even_4);  // odd[4][1] + even[2][0]
    // s''=010, s'=101: odd[5][pred_idx(010 in 101)=0] + even[2][pred_idx(101 in 010)=1]
    assign bm_r4_10 = $signed(bm_r2_odd_10) + $signed(bm_r2_even_5);  // odd[5][0] + even[2][1]
    // s''=011, s'=101: odd[5][pred_idx(011 in 101)=1] + even[2][1]
    assign bm_r4_11 = $signed(bm_r2_odd_11) + $signed(bm_r2_even_5);  // odd[5][1] + even[2][1]

    // --- State 011 (dest=3) ---
    // s''=100, s'=110: odd[6][pred_idx(100 in 110)=0] + even[3][pred_idx(110 in 011)=0]
    assign bm_r4_12 = $signed(bm_r2_odd_12) + $signed(bm_r2_even_6);  // odd[6][0] + even[3][0]
    // s''=101, s'=110: odd[6][pred_idx(101 in 110)=1] + even[3][0]
    assign bm_r4_13 = $signed(bm_r2_odd_13) + $signed(bm_r2_even_6);  // odd[6][1] + even[3][0]
    // s''=110, s'=111: odd[7][pred_idx(110 in 111)=0] + even[3][pred_idx(111 in 011)=1]
    assign bm_r4_14 = $signed(bm_r2_odd_14) + $signed(bm_r2_even_7);  // odd[7][0] + even[3][1]
    // s''=111, s'=111: odd[7][pred_idx(111 in 111)=1] + even[3][1]
    assign bm_r4_15 = $signed(bm_r2_odd_15) + $signed(bm_r2_even_7);  // odd[7][1] + even[3][1]

    // --- State 100 (dest=4) ---
    // s''=000, s'=000: odd[0][0] + even[4][pred_idx(000 in 100)=0]
    assign bm_r4_16 = $signed(bm_r2_odd_0)  + $signed(bm_r2_even_8);  // odd[0][0] + even[4][0]
    // s''=001, s'=000: odd[0][1] + even[4][0]
    assign bm_r4_17 = $signed(bm_r2_odd_1)  + $signed(bm_r2_even_8);  // odd[0][1] + even[4][0]
    // s''=010, s'=001: odd[1][0] + even[4][pred_idx(001 in 100)=1]
    assign bm_r4_18 = $signed(bm_r2_odd_2)  + $signed(bm_r2_even_9);  // odd[1][0] + even[4][1]
    // s''=011, s'=001: odd[1][1] + even[4][1]
    assign bm_r4_19 = $signed(bm_r2_odd_3)  + $signed(bm_r2_even_9);  // odd[1][1] + even[4][1]

    // --- State 101 (dest=5) ---
    // s''=100, s'=010: odd[2][0] + even[5][pred_idx(010 in 101)=0]
    assign bm_r4_20 = $signed(bm_r2_odd_4)  + $signed(bm_r2_even_10); // odd[2][0] + even[5][0]
    // s''=101, s'=010: odd[2][1] + even[5][0]
    assign bm_r4_21 = $signed(bm_r2_odd_5)  + $signed(bm_r2_even_10); // odd[2][1] + even[5][0]
    // s''=110, s'=011: odd[3][0] + even[5][pred_idx(011 in 101)=1]
    assign bm_r4_22 = $signed(bm_r2_odd_6)  + $signed(bm_r2_even_11); // odd[3][0] + even[5][1]
    // s''=111, s'=011: odd[3][1] + even[5][1]
    assign bm_r4_23 = $signed(bm_r2_odd_7)  + $signed(bm_r2_even_11); // odd[3][1] + even[5][1]

    // --- State 110 (dest=6) ---
    // s''=000, s'=100: odd[4][0] + even[6][pred_idx(100 in 110)=0]
    assign bm_r4_24 = $signed(bm_r2_odd_8)  + $signed(bm_r2_even_12); // odd[4][0] + even[6][0]
    // s''=001, s'=100: odd[4][1] + even[6][0]
    assign bm_r4_25 = $signed(bm_r2_odd_9)  + $signed(bm_r2_even_12); // odd[4][1] + even[6][0]
    // s''=010, s'=101: odd[5][0] + even[6][pred_idx(101 in 110)=1]
    assign bm_r4_26 = $signed(bm_r2_odd_10) + $signed(bm_r2_even_13); // odd[5][0] + even[6][1]
    // s''=011, s'=101: odd[5][1] + even[6][1]
    assign bm_r4_27 = $signed(bm_r2_odd_11) + $signed(bm_r2_even_13); // odd[5][1] + even[6][1]

    // --- State 111 (dest=7) ---
    // s''=100, s'=110: odd[6][0] + even[7][pred_idx(110 in 111)=0]
    assign bm_r4_28 = $signed(bm_r2_odd_12) + $signed(bm_r2_even_14); // odd[6][0] + even[7][0]
    // s''=101, s'=110: odd[6][1] + even[7][0]
    assign bm_r4_29 = $signed(bm_r2_odd_13) + $signed(bm_r2_even_14); // odd[6][1] + even[7][0]
    // s''=110, s'=111: odd[7][0] + even[7][pred_idx(111 in 111)=1]
    assign bm_r4_30 = $signed(bm_r2_odd_14) + $signed(bm_r2_even_15); // odd[7][0] + even[7][1]
    // s''=111, s'=111: odd[7][1] + even[7][1]
    assign bm_r4_31 = $signed(bm_r2_odd_15) + $signed(bm_r2_even_15); // odd[7][1] + even[7][1]

endmodule
