//==============================================================================
// Module: llr_compute
// Description: 1-stage pipelined LLR computation unit. Receives beta, alpha,
//              and gamma each cycle and produces two extrinsic LLRs per cycle
//              (one for odd step, one for even step).
//
// Steps:
//   1. Derive odd-step alpha α_{k-1}(s') from α_{k-2} + γ_r2_odd
//   2. Derive odd-step beta β_{k-1}(s') from β_k(s) + γ_r2_even
//   3. Compute even-step LLR from α_{k-1}, γ_r2_even, β_k
//   4. Compute odd-step LLR from α_{k-2}, γ_r2_odd, β_{k-1}
//   5. Max-of-8 trees for max0/max1 (xs=0/1 partitions)
//   6. Extrinsic = intrinsic - (L_s + L_A), scaled by 0.6875, saturated
//
// Reference: Studer et al., IEEE JSSC 2011, Section 13
//==============================================================================
module llr_compute (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       beta_valid,
    // Beta at even step k (from backward recursion)
    input  wire signed [9:0]          beta_k_0, beta_k_1, beta_k_2, beta_k_3,
    input  wire signed [9:0]          beta_k_4, beta_k_5, beta_k_6, beta_k_7,
    // Alpha at even step k-2 (from alpha memory)
    input  wire signed [9:0]          alpha_km2_0, alpha_km2_1, alpha_km2_2, alpha_km2_3,
    input  wire signed [9:0]          alpha_km2_4, alpha_km2_5, alpha_km2_6, alpha_km2_7,
    // Radix-2 BMs from gamma memory (indices 0..15 = odd, 16..31 = even)
    input  wire signed [6:0]          bm_r2_odd_0,  bm_r2_odd_1,  bm_r2_odd_2,  bm_r2_odd_3,
    input  wire signed [6:0]          bm_r2_odd_4,  bm_r2_odd_5,  bm_r2_odd_6,  bm_r2_odd_7,
    input  wire signed [6:0]          bm_r2_odd_8,  bm_r2_odd_9,  bm_r2_odd_10, bm_r2_odd_11,
    input  wire signed [6:0]          bm_r2_odd_12, bm_r2_odd_13, bm_r2_odd_14, bm_r2_odd_15,
    input  wire signed [6:0]          bm_r2_even_0,  bm_r2_even_1,  bm_r2_even_2,  bm_r2_even_3,
    input  wire signed [6:0]          bm_r2_even_4,  bm_r2_even_5,  bm_r2_even_6,  bm_r2_even_7,
    input  wire signed [6:0]          bm_r2_even_8,  bm_r2_even_9,  bm_r2_even_10, bm_r2_even_11,
    input  wire signed [6:0]          bm_r2_even_12, bm_r2_even_13, bm_r2_even_14, bm_r2_even_15,
    // Systematic and a-priori LLRs for extrinsic computation
    input  wire signed [4:0]          sys_odd_k, apr_odd_k,
    input  wire signed [4:0]          sys_even_k, apr_even_k,
    // Outputs
    output reg  signed [5:0]          llr_extr_odd,
    output reg  signed [5:0]          llr_extr_even,
    output reg                        llr_valid
);

    localparam SM_W    = 10;
    localparam EXTR_W  = 6;
    localparam LLR_W   = 5;

    // =========================================================================
    // Modulo-greater-than comparison function (combinational)
    // a > b iff (a-b) != 0 AND (a-b)[MSB] == 0
    // =========================================================================
    function signed [SM_W-1:0] modulo_max;
        input signed [SM_W-1:0] a, b;
        reg signed [SM_W-1:0] diff;
        begin
            diff = a - b;
            if ((diff != {SM_W{1'b0}}) && (diff[SM_W-1] == 1'b0))
                modulo_max = a;
            else
                modulo_max = b;
        end
    endfunction

    // =========================================================================
    // Step 1: Derive odd-step alpha α_{k-1}(s') from α_{k-2}(s'') + γ_r2_odd
    //
    // Using radix-2 predecessor table (Section 3.3):
    //   s'=000: preds s'' ∈ {000,001}, BMs: bm_r2_odd[0][0], bm_r2_odd[0][1]
    //   s'=001: preds s'' ∈ {010,011}, BMs: bm_r2_odd[1][0], bm_r2_odd[1][1]
    //   ...
    // =========================================================================
    wire signed [SM_W-1:0] alpha_km1_cand_0_0, alpha_km1_cand_0_1;
    wire signed [SM_W-1:0] alpha_km1_cand_1_0, alpha_km1_cand_1_1;
    wire signed [SM_W-1:0] alpha_km1_cand_2_0, alpha_km1_cand_2_1;
    wire signed [SM_W-1:0] alpha_km1_cand_3_0, alpha_km1_cand_3_1;
    wire signed [SM_W-1:0] alpha_km1_cand_4_0, alpha_km1_cand_4_1;
    wire signed [SM_W-1:0] alpha_km1_cand_5_0, alpha_km1_cand_5_1;
    wire signed [SM_W-1:0] alpha_km1_cand_6_0, alpha_km1_cand_6_1;
    wire signed [SM_W-1:0] alpha_km1_cand_7_0, alpha_km1_cand_7_1;

    // s'=000: preds = {000=0, 001=1}, BM indices = {0, 1}
    assign alpha_km1_cand_0_0 = (alpha_km2_0 + $signed({{3{bm_r2_odd_0[6]}}, bm_r2_odd_0}));
    assign alpha_km1_cand_0_1 = (alpha_km2_1 + $signed({{3{bm_r2_odd_1[6]}}, bm_r2_odd_1}));
    // s'=001: preds = {010=2, 011=3}, BM indices = {2, 3}
    assign alpha_km1_cand_1_0 = (alpha_km2_2 + $signed({{3{bm_r2_odd_2[6]}}, bm_r2_odd_2}));
    assign alpha_km1_cand_1_1 = (alpha_km2_3 + $signed({{3{bm_r2_odd_3[6]}}, bm_r2_odd_3}));
    // s'=010: preds = {100=4, 101=5}, BM indices = {4, 5}
    assign alpha_km1_cand_2_0 = (alpha_km2_4 + $signed({{3{bm_r2_odd_4[6]}}, bm_r2_odd_4}));
    assign alpha_km1_cand_2_1 = (alpha_km2_5 + $signed({{3{bm_r2_odd_5[6]}}, bm_r2_odd_5}));
    // s'=011: preds = {110=6, 111=7}, BM indices = {6, 7}
    assign alpha_km1_cand_3_0 = (alpha_km2_6 + $signed({{3{bm_r2_odd_6[6]}}, bm_r2_odd_6}));
    assign alpha_km1_cand_3_1 = (alpha_km2_7 + $signed({{3{bm_r2_odd_7[6]}}, bm_r2_odd_7}));
    // s'=100: preds = {000=0, 001=1}, BM indices = {8, 9}
    assign alpha_km1_cand_4_0 = (alpha_km2_0 + $signed({{3{bm_r2_odd_8[6]}}, bm_r2_odd_8}));
    assign alpha_km1_cand_4_1 = (alpha_km2_1 + $signed({{3{bm_r2_odd_9[6]}}, bm_r2_odd_9}));
    // s'=101: preds = {010=2, 011=3}, BM indices = {10, 11}
    assign alpha_km1_cand_5_0 = (alpha_km2_2 + $signed({{3{bm_r2_odd_10[6]}}, bm_r2_odd_10}));
    assign alpha_km1_cand_5_1 = (alpha_km2_3 + $signed({{3{bm_r2_odd_11[6]}}, bm_r2_odd_11}));
    // s'=110: preds = {100=4, 101=5}, BM indices = {12, 13}
    assign alpha_km1_cand_6_0 = (alpha_km2_4 + $signed({{3{bm_r2_odd_12[6]}}, bm_r2_odd_12}));
    assign alpha_km1_cand_6_1 = (alpha_km2_5 + $signed({{3{bm_r2_odd_13[6]}}, bm_r2_odd_13}));
    // s'=111: preds = {110=6, 111=7}, BM indices = {14, 15}
    assign alpha_km1_cand_7_0 = (alpha_km2_6 + $signed({{3{bm_r2_odd_14[6]}}, bm_r2_odd_14}));
    assign alpha_km1_cand_7_1 = (alpha_km2_7 + $signed({{3{bm_r2_odd_15[6]}}, bm_r2_odd_15}));

    // Truncate candidates to SM_W for modulo comparison
    wire signed [SM_W-1:0] akm1_c00 = alpha_km1_cand_0_0[SM_W-1:0];
    wire signed [SM_W-1:0] akm1_c01 = alpha_km1_cand_0_1[SM_W-1:0];
    wire signed [SM_W-1:0] akm1_c10 = alpha_km1_cand_1_0[SM_W-1:0];
    wire signed [SM_W-1:0] akm1_c11 = alpha_km1_cand_1_1[SM_W-1:0];
    wire signed [SM_W-1:0] akm1_c20 = alpha_km1_cand_2_0[SM_W-1:0];
    wire signed [SM_W-1:0] akm1_c21 = alpha_km1_cand_2_1[SM_W-1:0];
    wire signed [SM_W-1:0] akm1_c30 = alpha_km1_cand_3_0[SM_W-1:0];
    wire signed [SM_W-1:0] akm1_c31 = alpha_km1_cand_3_1[SM_W-1:0];
    wire signed [SM_W-1:0] akm1_c40 = alpha_km1_cand_4_0[SM_W-1:0];
    wire signed [SM_W-1:0] akm1_c41 = alpha_km1_cand_4_1[SM_W-1:0];
    wire signed [SM_W-1:0] akm1_c50 = alpha_km1_cand_5_0[SM_W-1:0];
    wire signed [SM_W-1:0] akm1_c51 = alpha_km1_cand_5_1[SM_W-1:0];
    wire signed [SM_W-1:0] akm1_c60 = alpha_km1_cand_6_0[SM_W-1:0];
    wire signed [SM_W-1:0] akm1_c61 = alpha_km1_cand_6_1[SM_W-1:0];
    wire signed [SM_W-1:0] akm1_c70 = alpha_km1_cand_7_0[SM_W-1:0];
    wire signed [SM_W-1:0] akm1_c71 = alpha_km1_cand_7_1[SM_W-1:0];

    // α_{k-1}(s') = max of 2 candidates (modulo comparison)
    wire signed [SM_W-1:0] alpha_km1_0 = modulo_max(akm1_c00, akm1_c01);
    wire signed [SM_W-1:0] alpha_km1_1 = modulo_max(akm1_c10, akm1_c11);
    wire signed [SM_W-1:0] alpha_km1_2 = modulo_max(akm1_c20, akm1_c21);
    wire signed [SM_W-1:0] alpha_km1_3 = modulo_max(akm1_c30, akm1_c31);
    wire signed [SM_W-1:0] alpha_km1_4 = modulo_max(akm1_c40, akm1_c41);
    wire signed [SM_W-1:0] alpha_km1_5 = modulo_max(akm1_c50, akm1_c51);
    wire signed [SM_W-1:0] alpha_km1_6 = modulo_max(akm1_c60, akm1_c61);
    wire signed [SM_W-1:0] alpha_km1_7 = modulo_max(akm1_c70, akm1_c71);

    // =========================================================================
    // Step 2: Derive odd-step beta β_{k-1}(s') from β_k(s) + γ_r2_even
    //
    // For each s', find its 2 successors in the even-step transition table:
    // Using "successor table" (transpose of R2 predecessor table for even step):
    //   s'=000: succesors s ∈ {000, 100} → BMs: bm_r2_even[0][0], bm_r2_even[4][0]
    //     (dest=000, pred_0=000: bm_r2_even_0; dest=100, pred_0=000: bm_r2_even_8)
    //   s'=001: successors s ∈ {000, 100} → bm_r2_even[0][1], bm_r2_even[4][1]
    //     (dest=000, pred_1=001: bm_r2_even_1; dest=100, pred_1=001: bm_r2_even_9)
    //   s'=010: successors s ∈ {001, 101}
    //     (dest=001, pred_0=010: bm_r2_even_2; dest=101, pred_0=010: bm_r2_even_10)
    //   s'=011: successors s ∈ {001, 101}
    //     (dest=001, pred_1=011: bm_r2_even_3; dest=101, pred_1=011: bm_r2_even_11)
    //   s'=100: successors s ∈ {010, 110}
    //     (dest=010, pred_0=100: bm_r2_even_4; dest=110, pred_0=100: bm_r2_even_12)
    //   s'=101: successors s ∈ {010, 110}
    //     (dest=010, pred_1=101: bm_r2_even_5; dest=110, pred_1=101: bm_r2_even_13)
    //   s'=110: successors s ∈ {011, 111}
    //     (dest=011, pred_0=110: bm_r2_even_6; dest=111, pred_0=110: bm_r2_even_14)
    //   s'=111: successors s ∈ {011, 111}
    //     (dest=011, pred_1=111: bm_r2_even_7; dest=111, pred_1=111: bm_r2_even_15)
    // =========================================================================
    wire signed [SM_W-1:0] beta_km1_cand_0_0, beta_km1_cand_0_1;
    wire signed [SM_W-1:0] beta_km1_cand_1_0, beta_km1_cand_1_1;
    wire signed [SM_W-1:0] beta_km1_cand_2_0, beta_km1_cand_2_1;
    wire signed [SM_W-1:0] beta_km1_cand_3_0, beta_km1_cand_3_1;
    wire signed [SM_W-1:0] beta_km1_cand_4_0, beta_km1_cand_4_1;
    wire signed [SM_W-1:0] beta_km1_cand_5_0, beta_km1_cand_5_1;
    wire signed [SM_W-1:0] beta_km1_cand_6_0, beta_km1_cand_6_1;
    wire signed [SM_W-1:0] beta_km1_cand_7_0, beta_km1_cand_7_1;

    // s'=000: succ s=000(β0), s=100(β4)
    assign beta_km1_cand_0_0 = beta_k_0 + $signed({{3{bm_r2_even_0[6]}}, bm_r2_even_0});
    assign beta_km1_cand_0_1 = beta_k_4 + $signed({{3{bm_r2_even_8[6]}}, bm_r2_even_8});
    // s'=001: succ s=000(β0), s=100(β4)
    assign beta_km1_cand_1_0 = beta_k_0 + $signed({{3{bm_r2_even_1[6]}}, bm_r2_even_1});
    assign beta_km1_cand_1_1 = beta_k_4 + $signed({{3{bm_r2_even_9[6]}}, bm_r2_even_9});
    // s'=010: succ s=001(β1), s=101(β5)
    assign beta_km1_cand_2_0 = beta_k_1 + $signed({{3{bm_r2_even_2[6]}}, bm_r2_even_2});
    assign beta_km1_cand_2_1 = beta_k_5 + $signed({{3{bm_r2_even_10[6]}}, bm_r2_even_10});
    // s'=011: succ s=001(β1), s=101(β5)
    assign beta_km1_cand_3_0 = beta_k_1 + $signed({{3{bm_r2_even_3[6]}}, bm_r2_even_3});
    assign beta_km1_cand_3_1 = beta_k_5 + $signed({{3{bm_r2_even_11[6]}}, bm_r2_even_11});
    // s'=100: succ s=010(β2), s=110(β6)
    assign beta_km1_cand_4_0 = beta_k_2 + $signed({{3{bm_r2_even_4[6]}}, bm_r2_even_4});
    assign beta_km1_cand_4_1 = beta_k_6 + $signed({{3{bm_r2_even_12[6]}}, bm_r2_even_12});
    // s'=101: succ s=010(β2), s=110(β6)
    assign beta_km1_cand_5_0 = beta_k_2 + $signed({{3{bm_r2_even_5[6]}}, bm_r2_even_5});
    assign beta_km1_cand_5_1 = beta_k_6 + $signed({{3{bm_r2_even_13[6]}}, bm_r2_even_13});
    // s'=110: succ s=011(β3), s=111(β7)
    assign beta_km1_cand_6_0 = beta_k_3 + $signed({{3{bm_r2_even_6[6]}}, bm_r2_even_6});
    assign beta_km1_cand_6_1 = beta_k_7 + $signed({{3{bm_r2_even_14[6]}}, bm_r2_even_14});
    // s'=111: succ s=011(β3), s=111(β7)
    assign beta_km1_cand_7_0 = beta_k_3 + $signed({{3{bm_r2_even_7[6]}}, bm_r2_even_7});
    assign beta_km1_cand_7_1 = beta_k_7 + $signed({{3{bm_r2_even_15[6]}}, bm_r2_even_15});

    wire signed [SM_W-1:0] beta_km1_0 = modulo_max(beta_km1_cand_0_0, beta_km1_cand_0_1);
    wire signed [SM_W-1:0] beta_km1_1 = modulo_max(beta_km1_cand_1_0, beta_km1_cand_1_1);
    wire signed [SM_W-1:0] beta_km1_2 = modulo_max(beta_km1_cand_2_0, beta_km1_cand_2_1);
    wire signed [SM_W-1:0] beta_km1_3 = modulo_max(beta_km1_cand_3_0, beta_km1_cand_3_1);
    wire signed [SM_W-1:0] beta_km1_4 = modulo_max(beta_km1_cand_4_0, beta_km1_cand_4_1);
    wire signed [SM_W-1:0] beta_km1_5 = modulo_max(beta_km1_cand_5_0, beta_km1_cand_5_1);
    wire signed [SM_W-1:0] beta_km1_6 = modulo_max(beta_km1_cand_6_0, beta_km1_cand_6_1);
    wire signed [SM_W-1:0] beta_km1_7 = modulo_max(beta_km1_cand_7_0, beta_km1_cand_7_1);

    // =========================================================================
    // Steps 3-4: Path metric computation for even and odd steps
    //
    // Even step: path_metric(s',s) = α_{k-1}(s') + γ_r2_even(s',s) + β_k(s)
    // Odd step:  path_metric(s'',s') = α_{k-2}(s'') + γ_r2_odd(s'',s') + β_{k-1}(s')
    //
    // Even step xs=0 paths (from Section 3.5):
    //   (000→000), (001→100), (010→001), (011→101),
    //   (100→110), (101→010), (110→111), (111→011)
    //
    // Even step xs=1 paths:
    //   (000→100), (001→000), (010→101), (011→001),
    //   (100→010), (101→110), (110→011), (111→111)
    // =========================================================================

    // --- Even step path metrics ---
    // xs=0 paths:
    wire signed [SM_W-1:0] ep0_0 = (alpha_km1_0 + $signed({{3{bm_r2_even_0[6]}},  bm_r2_even_0})  + beta_k_0); // 000→000
    wire signed [SM_W-1:0] ep0_1 = (alpha_km1_1 + $signed({{3{bm_r2_even_9[6]}},  bm_r2_even_9})  + beta_k_4); // 001→100
    wire signed [SM_W-1:0] ep0_2 = (alpha_km1_2 + $signed({{3{bm_r2_even_2[6]}},  bm_r2_even_2})  + beta_k_1); // 010→001
    wire signed [SM_W-1:0] ep0_3 = (alpha_km1_3 + $signed({{3{bm_r2_even_11[6]}}, bm_r2_even_11}) + beta_k_5); // 011→101
    wire signed [SM_W-1:0] ep0_4 = (alpha_km1_4 + $signed({{3{bm_r2_even_12[6]}}, bm_r2_even_12}) + beta_k_6); // 100→110
    wire signed [SM_W-1:0] ep0_5 = (alpha_km1_5 + $signed({{3{bm_r2_even_4[6]}},  bm_r2_even_4})  + beta_k_2); // 101→010
    wire signed [SM_W-1:0] ep0_6 = (alpha_km1_6 + $signed({{3{bm_r2_even_14[6]}}, bm_r2_even_14}) + beta_k_7); // 110→111
    wire signed [SM_W-1:0] ep0_7 = (alpha_km1_7 + $signed({{3{bm_r2_even_6[6]}},  bm_r2_even_6})  + beta_k_3); // 111→011

    // xs=1 paths:
    wire signed [SM_W-1:0] ep1_0 = (alpha_km1_0 + $signed({{3{bm_r2_even_8[6]}},  bm_r2_even_8})  + beta_k_4); // 000→100
    wire signed [SM_W-1:0] ep1_1 = (alpha_km1_1 + $signed({{3{bm_r2_even_1[6]}},  bm_r2_even_1})  + beta_k_0); // 001→000
    wire signed [SM_W-1:0] ep1_2 = (alpha_km1_2 + $signed({{3{bm_r2_even_10[6]}}, bm_r2_even_10}) + beta_k_5); // 010→101
    wire signed [SM_W-1:0] ep1_3 = (alpha_km1_3 + $signed({{3{bm_r2_even_3[6]}},  bm_r2_even_3})  + beta_k_1); // 011→001
    wire signed [SM_W-1:0] ep1_4 = (alpha_km1_4 + $signed({{3{bm_r2_even_4[6]}},  bm_r2_even_4})  + beta_k_2); // 100→010
    wire signed [SM_W-1:0] ep1_5 = (alpha_km1_5 + $signed({{3{bm_r2_even_12[6]}}, bm_r2_even_12}) + beta_k_6); // 101→110
    wire signed [SM_W-1:0] ep1_6 = (alpha_km1_6 + $signed({{3{bm_r2_even_6[6]}},  bm_r2_even_6})  + beta_k_3); // 110→011
    wire signed [SM_W-1:0] ep1_7 = (alpha_km1_7 + $signed({{3{bm_r2_even_14[6]}}, bm_r2_even_14}) + beta_k_7); // 111→111

    // --- Odd step path metrics ---
    // xs=0 paths (same transition patterns):
    wire signed [SM_W-1:0] op0_0 = (alpha_km2_0 + $signed({{3{bm_r2_odd_0[6]}},  bm_r2_odd_0})  + beta_km1_0); // 000→000
    wire signed [SM_W-1:0] op0_1 = (alpha_km2_1 + $signed({{3{bm_r2_odd_9[6]}},  bm_r2_odd_9})  + beta_km1_4); // 001→100
    wire signed [SM_W-1:0] op0_2 = (alpha_km2_2 + $signed({{3{bm_r2_odd_2[6]}},  bm_r2_odd_2})  + beta_km1_1); // 010→001
    wire signed [SM_W-1:0] op0_3 = (alpha_km2_3 + $signed({{3{bm_r2_odd_11[6]}}, bm_r2_odd_11}) + beta_km1_5); // 011→101
    wire signed [SM_W-1:0] op0_4 = (alpha_km2_4 + $signed({{3{bm_r2_odd_12[6]}}, bm_r2_odd_12}) + beta_km1_6); // 100→110
    wire signed [SM_W-1:0] op0_5 = (alpha_km2_5 + $signed({{3{bm_r2_odd_4[6]}},  bm_r2_odd_4})  + beta_km1_2); // 101→010
    wire signed [SM_W-1:0] op0_6 = (alpha_km2_6 + $signed({{3{bm_r2_odd_14[6]}}, bm_r2_odd_14}) + beta_km1_7); // 110→111
    wire signed [SM_W-1:0] op0_7 = (alpha_km2_7 + $signed({{3{bm_r2_odd_6[6]}},  bm_r2_odd_6})  + beta_km1_3); // 111→011

    // xs=1 paths:
    wire signed [SM_W-1:0] op1_0 = (alpha_km2_0 + $signed({{3{bm_r2_odd_8[6]}},  bm_r2_odd_8})  + beta_km1_4); // 000→100
    wire signed [SM_W-1:0] op1_1 = (alpha_km2_1 + $signed({{3{bm_r2_odd_1[6]}},  bm_r2_odd_1})  + beta_km1_0); // 001→000
    wire signed [SM_W-1:0] op1_2 = (alpha_km2_2 + $signed({{3{bm_r2_odd_10[6]}}, bm_r2_odd_10}) + beta_km1_5); // 010→101
    wire signed [SM_W-1:0] op1_3 = (alpha_km2_3 + $signed({{3{bm_r2_odd_3[6]}},  bm_r2_odd_3})  + beta_km1_1); // 011→001
    wire signed [SM_W-1:0] op1_4 = (alpha_km2_4 + $signed({{3{bm_r2_odd_4[6]}},  bm_r2_odd_4})  + beta_km1_2); // 100→010
    wire signed [SM_W-1:0] op1_5 = (alpha_km2_5 + $signed({{3{bm_r2_odd_12[6]}}, bm_r2_odd_12}) + beta_km1_6); // 101→110
    wire signed [SM_W-1:0] op1_6 = (alpha_km2_6 + $signed({{3{bm_r2_odd_6[6]}},  bm_r2_odd_6})  + beta_km1_3); // 110→011
    wire signed [SM_W-1:0] op1_7 = (alpha_km2_7 + $signed({{3{bm_r2_odd_14[6]}}, bm_r2_odd_14}) + beta_km1_7); // 111→111

    // =========================================================================
    // Step 5: Max-of-8 trees (binary tree using modulo_max)
    // =========================================================================

    // --- Even step max trees ---
    // max0_even: max of ep0_0 through ep0_7
    wire signed [SM_W-1:0] emax0_l0 = modulo_max(ep0_0, ep0_1);
    wire signed [SM_W-1:0] emax0_l1 = modulo_max(ep0_2, ep0_3);
    wire signed [SM_W-1:0] emax0_l2 = modulo_max(ep0_4, ep0_5);
    wire signed [SM_W-1:0] emax0_l3 = modulo_max(ep0_6, ep0_7);
    wire signed [SM_W-1:0] emax0_m0 = modulo_max(emax0_l0, emax0_l1);
    wire signed [SM_W-1:0] emax0_m1 = modulo_max(emax0_l2, emax0_l3);
    wire signed [SM_W-1:0] max0_even = modulo_max(emax0_m0, emax0_m1);

    // max1_even: max of ep1_0 through ep1_7
    wire signed [SM_W-1:0] emax1_l0 = modulo_max(ep1_0, ep1_1);
    wire signed [SM_W-1:0] emax1_l1 = modulo_max(ep1_2, ep1_3);
    wire signed [SM_W-1:0] emax1_l2 = modulo_max(ep1_4, ep1_5);
    wire signed [SM_W-1:0] emax1_l3 = modulo_max(ep1_6, ep1_7);
    wire signed [SM_W-1:0] emax1_m0 = modulo_max(emax1_l0, emax1_l1);
    wire signed [SM_W-1:0] emax1_m1 = modulo_max(emax1_l2, emax1_l3);
    wire signed [SM_W-1:0] max1_even = modulo_max(emax1_m0, emax1_m1);

    // L_D_even = max0_even - max1_even
    wire signed [SM_W-1:0] ld_even = max0_even - max1_even;

    // --- Odd step max trees ---
    wire signed [SM_W-1:0] omax0_l0 = modulo_max(op0_0, op0_1);
    wire signed [SM_W-1:0] omax0_l1 = modulo_max(op0_2, op0_3);
    wire signed [SM_W-1:0] omax0_l2 = modulo_max(op0_4, op0_5);
    wire signed [SM_W-1:0] omax0_l3 = modulo_max(op0_6, op0_7);
    wire signed [SM_W-1:0] omax0_m0 = modulo_max(omax0_l0, omax0_l1);
    wire signed [SM_W-1:0] omax0_m1 = modulo_max(omax0_l2, omax0_l3);
    wire signed [SM_W-1:0] max0_odd = modulo_max(omax0_m0, omax0_m1);

    wire signed [SM_W-1:0] omax1_l0 = modulo_max(op1_0, op1_1);
    wire signed [SM_W-1:0] omax1_l1 = modulo_max(op1_2, op1_3);
    wire signed [SM_W-1:0] omax1_l2 = modulo_max(op1_4, op1_5);
    wire signed [SM_W-1:0] omax1_l3 = modulo_max(op1_6, op1_7);
    wire signed [SM_W-1:0] omax1_m0 = modulo_max(omax1_l0, omax1_l1);
    wire signed [SM_W-1:0] omax1_m1 = modulo_max(omax1_l2, omax1_l3);
    wire signed [SM_W-1:0] max1_odd = modulo_max(omax1_m0, omax1_m1);

    wire signed [SM_W-1:0] ld_odd = max0_odd - max1_odd;

    // =========================================================================
    // Step 6: Extrinsic LLR = L_D - (L_s + L_A), then scale by 0.6875
    // 0.6875 = 1 - 1/8 - 1/16 = value - (value>>>3) - (value>>>4)
    // Saturate result to [-32, 31] (6-bit signed)
    // =========================================================================
    wire signed [SM_W-1:0] la_even = $signed({{(SM_W-LLR_W){sys_even_k[LLR_W-1]}}, sys_even_k})
                                   + $signed({{(SM_W-LLR_W){apr_even_k[LLR_W-1]}}, apr_even_k});
    wire signed [SM_W-1:0] le_even = ld_even - la_even;
    wire signed [SM_W-1:0] le_even_scaled = le_even - (le_even >>> 3) - (le_even >>> 4);

    wire signed [SM_W-1:0] la_odd = $signed({{(SM_W-LLR_W){sys_odd_k[LLR_W-1]}}, sys_odd_k})
                                  + $signed({{(SM_W-LLR_W){apr_odd_k[LLR_W-1]}}, apr_odd_k});
    wire signed [SM_W-1:0] le_odd = ld_odd - la_odd;
    wire signed [SM_W-1:0] le_odd_scaled = le_odd - (le_odd >>> 3) - (le_odd >>> 4);

    // Saturation to 6-bit signed [-32, 31]
    function signed [EXTR_W-1:0] saturate;
        input signed [SM_W-1:0] val;
        begin
            if (val > $signed(10'sd31))
                saturate = 6'sd31;
            else if (val < $signed(-10'sd32))
                saturate = -6'sd32;
            else
                saturate = val[EXTR_W-1:0];
        end
    endfunction

    // =========================================================================
    // Output registers (1-cycle pipeline)
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            llr_extr_odd  <= 6'sd0;
            llr_extr_even <= 6'sd0;
            llr_valid     <= 1'b0;
        end else begin
            llr_valid <= beta_valid;
            if (beta_valid) begin
                llr_extr_even <= saturate(le_even_scaled);
                llr_extr_odd  <= saturate(le_odd_scaled);
            end
        end
    end

endmodule
