//==============================================================================
// Module: bm_preproc
// Description: Combinational branch metric preprocessing with BPSK substitution.
//              Computes 4 pre-processed values for both odd and even trellis
//              steps from input systematic, parity, and a-priori LLRs.
//
// BPSK mapping: bit 0 → +1, bit 1 → -1
// Branch metric: γ = ũ·(L_s + L_A) + x̃_p·L_p
//   PRE[0] = {x_s=0, x_p=0}: +(L_s + L_A) + L_p  =  L_s + L_A + L_p
//   PRE[1] = {x_s=0, x_p=1}: +(L_s + L_A) - L_p  =  L_s + L_A - L_p
//   PRE[2] = {x_s=1, x_p=0}: -(L_s + L_A) + L_p  = -L_s - L_A + L_p
//   PRE[3] = {x_s=1, x_p=1}: -(L_s + L_A) - L_p  = -L_s - L_A - L_p
//
// Note: PRE[2] = -PRE[1], PRE[3] = -PRE[0]  (symmetry property)
//
// Reference: Studer et al., IEEE JSSC 2011, Section 4.1
//==============================================================================
module bm_preproc (
    input  wire signed [4:0] sys_odd,
    input  wire signed [4:0] par_odd,
    input  wire signed [4:0] apr_odd,
    input  wire signed [4:0] sys_even,
    input  wire signed [4:0] par_even,
    input  wire signed [4:0] apr_even,
    output wire signed [6:0] pre_odd_0,
    output wire signed [6:0] pre_odd_1,
    output wire signed [6:0] pre_odd_2,
    output wire signed [6:0] pre_odd_3,
    output wire signed [6:0] pre_even_0,
    output wire signed [6:0] pre_even_1,
    output wire signed [6:0] pre_even_2,
    output wire signed [6:0] pre_even_3
);

    // -------------------------------------------------------------------------
    // Local parameters (re-declared per prompt Section 16 constraint 10)
    // -------------------------------------------------------------------------
    localparam LLR_W     = 5;   // Input LLR width
    localparam PREPROC_W = 7;   // Pre-processed BM width (7 bits for BPSK)
                                // Max |L_s+L_A+L_p| = 48, fits in 7-bit signed [-64,63]

    // -------------------------------------------------------------------------
    // Intermediate: L_s + L_A (6-bit signed, fits exactly)
    // -------------------------------------------------------------------------
    wire signed [5:0] sa_odd  = $signed(sys_odd)  + $signed(apr_odd);
    wire signed [5:0] sa_even = $signed(sys_even) + $signed(apr_even);

    // -------------------------------------------------------------------------
    // Odd step pre-processing (BPSK substituted)
    // PRE[0] = +(L_s + L_A) + L_p =  (sa + par)
    // PRE[1] = +(L_s + L_A) - L_p =  (sa - par)
    // PRE[2] = -(L_s + L_A) + L_p = -(sa) + par = -PRE[1]
    // PRE[3] = -(L_s + L_A) - L_p = -(sa + par) = -PRE[0]
    // -------------------------------------------------------------------------
    assign pre_odd_0 =  $signed(sa_odd) + $signed(par_odd);   // L_s + L_A + L_p
    assign pre_odd_1 =  $signed(sa_odd) - $signed(par_odd);   // L_s + L_A - L_p
    assign pre_odd_2 = -$signed(sa_odd) + $signed(par_odd);   // -L_s - L_A + L_p
    assign pre_odd_3 = -$signed(sa_odd) - $signed(par_odd);   // -L_s - L_A - L_p

    // -------------------------------------------------------------------------
    // Even step pre-processing (identical structure, even-step LLRs)
    // -------------------------------------------------------------------------
    assign pre_even_0 =  $signed(sa_even) + $signed(par_even);
    assign pre_even_1 =  $signed(sa_even) - $signed(par_even);
    assign pre_even_2 = -$signed(sa_even) + $signed(par_even);
    assign pre_even_3 = -$signed(sa_even) - $signed(par_even);

endmodule
