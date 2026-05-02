//==============================================================================
// Module: llr_compute
// Description: 5-stage pipelined LLR computation unit. Receives beta, alpha,
//              and gamma each cycle and produces two extrinsic LLRs per cycle
//              after the pipeline latency.
//
// Pipeline:
//   S1: mid-step alpha/beta metrics
//   S2: path metrics and first max-tree level
//   S3: second max-tree level
//   S4: final max-tree level and intrinsic LLRs
//   S5: extrinsic scaling/saturation and output registers
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
    output reg  signed [9:0]          llr_intr_odd,
    output reg  signed [9:0]          llr_intr_even,
    output reg                        llr_valid
);

    localparam SM_W    = 10;
    localparam EXTR_W  = 6;
    localparam BM_R2_W = 7;

    // Keep these input ports intentionally consumed so older elaboration flows
    // do not warn about disconnected systematic/a-priori pins. The actual
    // L_s+L_A term is recovered from stored branch metrics below.
    wire unused_sys_apr = ^{sys_odd_k, apr_odd_k, sys_even_k, apr_even_k};

    // =========================================================================
    // Small arithmetic helpers
    // =========================================================================
    function signed [SM_W-1:0] bm_ext;
        input signed [BM_R2_W-1:0] bm;
        begin
            bm_ext = {{(SM_W-BM_R2_W){bm[BM_R2_W-1]}}, bm};
        end
    endfunction

    function signed [SM_W-1:0] add_bm;
        input signed [SM_W-1:0]    metric;
        input signed [BM_R2_W-1:0] bm;
        begin
            add_bm = metric + bm_ext(bm);
        end
    endfunction

    function signed [SM_W-1:0] path_metric;
        input signed [SM_W-1:0]    metric_a;
        input signed [BM_R2_W-1:0] bm;
        input signed [SM_W-1:0]    metric_b;
        begin
            path_metric = metric_a + bm_ext(bm) + metric_b;
        end
    endfunction

    // a > b iff (a-b) != 0 AND (a-b)[MSB] == 0
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
    // Flattened inputs into local arrays for the pipelined datapath
    // =========================================================================
    wire signed [SM_W-1:0] alpha_in [0:7];
    wire signed [SM_W-1:0] beta_in  [0:7];
    wire signed [BM_R2_W-1:0] bm_odd_in  [0:15];
    wire signed [BM_R2_W-1:0] bm_even_in [0:15];

    assign alpha_in[0] = alpha_km2_0; assign alpha_in[1] = alpha_km2_1;
    assign alpha_in[2] = alpha_km2_2; assign alpha_in[3] = alpha_km2_3;
    assign alpha_in[4] = alpha_km2_4; assign alpha_in[5] = alpha_km2_5;
    assign alpha_in[6] = alpha_km2_6; assign alpha_in[7] = alpha_km2_7;

    assign beta_in[0] = beta_k_0; assign beta_in[1] = beta_k_1;
    assign beta_in[2] = beta_k_2; assign beta_in[3] = beta_k_3;
    assign beta_in[4] = beta_k_4; assign beta_in[5] = beta_k_5;
    assign beta_in[6] = beta_k_6; assign beta_in[7] = beta_k_7;

    assign bm_odd_in[0]  = bm_r2_odd_0;  assign bm_odd_in[1]  = bm_r2_odd_1;
    assign bm_odd_in[2]  = bm_r2_odd_2;  assign bm_odd_in[3]  = bm_r2_odd_3;
    assign bm_odd_in[4]  = bm_r2_odd_4;  assign bm_odd_in[5]  = bm_r2_odd_5;
    assign bm_odd_in[6]  = bm_r2_odd_6;  assign bm_odd_in[7]  = bm_r2_odd_7;
    assign bm_odd_in[8]  = bm_r2_odd_8;  assign bm_odd_in[9]  = bm_r2_odd_9;
    assign bm_odd_in[10] = bm_r2_odd_10; assign bm_odd_in[11] = bm_r2_odd_11;
    assign bm_odd_in[12] = bm_r2_odd_12; assign bm_odd_in[13] = bm_r2_odd_13;
    assign bm_odd_in[14] = bm_r2_odd_14; assign bm_odd_in[15] = bm_r2_odd_15;

    assign bm_even_in[0]  = bm_r2_even_0;  assign bm_even_in[1]  = bm_r2_even_1;
    assign bm_even_in[2]  = bm_r2_even_2;  assign bm_even_in[3]  = bm_r2_even_3;
    assign bm_even_in[4]  = bm_r2_even_4;  assign bm_even_in[5]  = bm_r2_even_5;
    assign bm_even_in[6]  = bm_r2_even_6;  assign bm_even_in[7]  = bm_r2_even_7;
    assign bm_even_in[8]  = bm_r2_even_8;  assign bm_even_in[9]  = bm_r2_even_9;
    assign bm_even_in[10] = bm_r2_even_10; assign bm_even_in[11] = bm_r2_even_11;
    assign bm_even_in[12] = bm_r2_even_12; assign bm_even_in[13] = bm_r2_even_13;
    assign bm_even_in[14] = bm_r2_even_14; assign bm_even_in[15] = bm_r2_even_15;

    // =========================================================================
    // Pipeline registers
    // =========================================================================
    reg s1_valid, s2_valid, s3_valid, s4_valid;

    reg signed [SM_W-1:0] alpha_km2_s1 [0:7];
    reg signed [SM_W-1:0] beta_k_s1    [0:7];
    reg signed [SM_W-1:0] alpha_km1_s1 [0:7];
    reg signed [SM_W-1:0] beta_km1_s1  [0:7];
    reg signed [BM_R2_W-1:0] bm_odd_s1  [0:15];
    reg signed [BM_R2_W-1:0] bm_even_s1 [0:15];

    reg signed [SM_W-1:0] emax0_l_s2 [0:3];
    reg signed [SM_W-1:0] emax1_l_s2 [0:3];
    reg signed [SM_W-1:0] omax0_l_s2 [0:3];
    reg signed [SM_W-1:0] omax1_l_s2 [0:3];
    reg signed [SM_W-1:0] la_even_s2, la_odd_s2;

    reg signed [SM_W-1:0] emax0_m_s3 [0:1];
    reg signed [SM_W-1:0] emax1_m_s3 [0:1];
    reg signed [SM_W-1:0] omax0_m_s3 [0:1];
    reg signed [SM_W-1:0] omax1_m_s3 [0:1];
    reg signed [SM_W-1:0] la_even_s3, la_odd_s3;

    reg signed [SM_W-1:0] ld_even_s4, ld_odd_s4;
    reg signed [SM_W-1:0] la_even_s4, la_odd_s4;

    wire signed [SM_W-1:0] max0_even_s4 = modulo_max(emax0_m_s3[0], emax0_m_s3[1]);
    wire signed [SM_W-1:0] max1_even_s4 = modulo_max(emax1_m_s3[0], emax1_m_s3[1]);
    wire signed [SM_W-1:0] max0_odd_s4  = modulo_max(omax0_m_s3[0], omax0_m_s3[1]);
    wire signed [SM_W-1:0] max1_odd_s4  = modulo_max(omax1_m_s3[0], omax1_m_s3[1]);

    wire signed [SM_W-1:0] le_even_s5 = ld_even_s4 - la_even_s4;
    wire signed [SM_W-1:0] le_odd_s5  = ld_odd_s4  - la_odd_s4;
    wire signed [SM_W-1:0] le_even_scaled_s5 = le_even_s5 - (le_even_s5 >>> 2) - (le_even_s5 >>> 4);
    wire signed [SM_W-1:0] le_odd_scaled_s5  = le_odd_s5  - (le_odd_s5  >>> 2) - (le_odd_s5  >>> 4);

    integer i;

    always @(posedge clk) begin
        if (!rst_n) begin
            s1_valid    <= 1'b0;
            s2_valid    <= 1'b0;
            s3_valid    <= 1'b0;
            s4_valid    <= 1'b0;
            llr_valid   <= 1'b0;
            llr_extr_odd  <= 6'sd0;
            llr_extr_even <= 6'sd0;
            llr_intr_odd  <= 10'sd0;
            llr_intr_even <= 10'sd0;
        end else begin
            s1_valid  <= beta_valid;
            s2_valid  <= s1_valid;
            s3_valid  <= s2_valid;
            s4_valid  <= s3_valid;
            llr_valid <= s4_valid;

            // S1: derive mid-step alpha and beta metrics.
            if (beta_valid) begin
                for (i = 0; i < 8; i = i + 1) begin
                    alpha_km2_s1[i] <= alpha_in[i];
                    beta_k_s1[i]    <= beta_in[i];
                end
                for (i = 0; i < 16; i = i + 1) begin
                    bm_odd_s1[i]  <= bm_odd_in[i];
                    bm_even_s1[i] <= bm_even_in[i];
                end

                alpha_km1_s1[0] <= modulo_max(add_bm(alpha_in[0], bm_even_in[0]),  add_bm(alpha_in[1], bm_even_in[1]));
                alpha_km1_s1[1] <= modulo_max(add_bm(alpha_in[2], bm_even_in[2]),  add_bm(alpha_in[3], bm_even_in[3]));
                alpha_km1_s1[2] <= modulo_max(add_bm(alpha_in[4], bm_even_in[4]),  add_bm(alpha_in[5], bm_even_in[5]));
                alpha_km1_s1[3] <= modulo_max(add_bm(alpha_in[6], bm_even_in[6]),  add_bm(alpha_in[7], bm_even_in[7]));
                alpha_km1_s1[4] <= modulo_max(add_bm(alpha_in[0], bm_even_in[8]),  add_bm(alpha_in[1], bm_even_in[9]));
                alpha_km1_s1[5] <= modulo_max(add_bm(alpha_in[2], bm_even_in[10]), add_bm(alpha_in[3], bm_even_in[11]));
                alpha_km1_s1[6] <= modulo_max(add_bm(alpha_in[4], bm_even_in[12]), add_bm(alpha_in[5], bm_even_in[13]));
                alpha_km1_s1[7] <= modulo_max(add_bm(alpha_in[6], bm_even_in[14]), add_bm(alpha_in[7], bm_even_in[15]));

                beta_km1_s1[0] <= modulo_max(add_bm(beta_in[0], bm_odd_in[0]),  add_bm(beta_in[4], bm_odd_in[8]));
                beta_km1_s1[1] <= modulo_max(add_bm(beta_in[0], bm_odd_in[1]),  add_bm(beta_in[4], bm_odd_in[9]));
                beta_km1_s1[2] <= modulo_max(add_bm(beta_in[1], bm_odd_in[2]),  add_bm(beta_in[5], bm_odd_in[10]));
                beta_km1_s1[3] <= modulo_max(add_bm(beta_in[1], bm_odd_in[3]),  add_bm(beta_in[5], bm_odd_in[11]));
                beta_km1_s1[4] <= modulo_max(add_bm(beta_in[2], bm_odd_in[4]),  add_bm(beta_in[6], bm_odd_in[12]));
                beta_km1_s1[5] <= modulo_max(add_bm(beta_in[2], bm_odd_in[5]),  add_bm(beta_in[6], bm_odd_in[13]));
                beta_km1_s1[6] <= modulo_max(add_bm(beta_in[3], bm_odd_in[6]),  add_bm(beta_in[7], bm_odd_in[14]));
                beta_km1_s1[7] <= modulo_max(add_bm(beta_in[3], bm_odd_in[7]),  add_bm(beta_in[7], bm_odd_in[15]));
            end

            // S2: compute path metrics and first max-tree level.
            if (s1_valid) begin
                emax0_l_s2[0] <= modulo_max(path_metric(alpha_km2_s1[0], bm_even_s1[0],  beta_km1_s1[0]),
                                             path_metric(alpha_km2_s1[1], bm_even_s1[9],  beta_km1_s1[4]));
                emax0_l_s2[1] <= modulo_max(path_metric(alpha_km2_s1[2], bm_even_s1[10], beta_km1_s1[5]),
                                             path_metric(alpha_km2_s1[3], bm_even_s1[3],  beta_km1_s1[1]));
                emax0_l_s2[2] <= modulo_max(path_metric(alpha_km2_s1[4], bm_even_s1[4],  beta_km1_s1[2]),
                                             path_metric(alpha_km2_s1[5], bm_even_s1[13], beta_km1_s1[6]));
                emax0_l_s2[3] <= modulo_max(path_metric(alpha_km2_s1[6], bm_even_s1[14], beta_km1_s1[7]),
                                             path_metric(alpha_km2_s1[7], bm_even_s1[7],  beta_km1_s1[3]));

                emax1_l_s2[0] <= modulo_max(path_metric(alpha_km2_s1[0], bm_even_s1[8],  beta_km1_s1[4]),
                                             path_metric(alpha_km2_s1[1], bm_even_s1[1],  beta_km1_s1[0]));
                emax1_l_s2[1] <= modulo_max(path_metric(alpha_km2_s1[2], bm_even_s1[2],  beta_km1_s1[1]),
                                             path_metric(alpha_km2_s1[3], bm_even_s1[11], beta_km1_s1[5]));
                emax1_l_s2[2] <= modulo_max(path_metric(alpha_km2_s1[4], bm_even_s1[12], beta_km1_s1[6]),
                                             path_metric(alpha_km2_s1[5], bm_even_s1[5],  beta_km1_s1[2]));
                emax1_l_s2[3] <= modulo_max(path_metric(alpha_km2_s1[6], bm_even_s1[6],  beta_km1_s1[3]),
                                             path_metric(alpha_km2_s1[7], bm_even_s1[15], beta_km1_s1[7]));

                omax0_l_s2[0] <= modulo_max(path_metric(alpha_km1_s1[0], bm_odd_s1[0],  beta_k_s1[0]),
                                             path_metric(alpha_km1_s1[1], bm_odd_s1[9],  beta_k_s1[4]));
                omax0_l_s2[1] <= modulo_max(path_metric(alpha_km1_s1[2], bm_odd_s1[10], beta_k_s1[5]),
                                             path_metric(alpha_km1_s1[3], bm_odd_s1[3],  beta_k_s1[1]));
                omax0_l_s2[2] <= modulo_max(path_metric(alpha_km1_s1[4], bm_odd_s1[4],  beta_k_s1[2]),
                                             path_metric(alpha_km1_s1[5], bm_odd_s1[13], beta_k_s1[6]));
                omax0_l_s2[3] <= modulo_max(path_metric(alpha_km1_s1[6], bm_odd_s1[14], beta_k_s1[7]),
                                             path_metric(alpha_km1_s1[7], bm_odd_s1[7],  beta_k_s1[3]));

                omax1_l_s2[0] <= modulo_max(path_metric(alpha_km1_s1[0], bm_odd_s1[8],  beta_k_s1[4]),
                                             path_metric(alpha_km1_s1[1], bm_odd_s1[1],  beta_k_s1[0]));
                omax1_l_s2[1] <= modulo_max(path_metric(alpha_km1_s1[2], bm_odd_s1[2],  beta_k_s1[1]),
                                             path_metric(alpha_km1_s1[3], bm_odd_s1[11], beta_k_s1[5]));
                omax1_l_s2[2] <= modulo_max(path_metric(alpha_km1_s1[4], bm_odd_s1[12], beta_k_s1[6]),
                                             path_metric(alpha_km1_s1[5], bm_odd_s1[5],  beta_k_s1[2]));
                omax1_l_s2[3] <= modulo_max(path_metric(alpha_km1_s1[6], bm_odd_s1[6],  beta_k_s1[3]),
                                             path_metric(alpha_km1_s1[7], bm_odd_s1[15], beta_k_s1[7]));

                la_even_s2 <= (bm_ext(bm_even_s1[0]) + bm_ext(bm_even_s1[3])) >>> 1;
                la_odd_s2  <= (bm_ext(bm_odd_s1[0])  + bm_ext(bm_odd_s1[3]))  >>> 1;
            end

            // S3: second max-tree level.
            if (s2_valid) begin
                emax0_m_s3[0] <= modulo_max(emax0_l_s2[0], emax0_l_s2[1]);
                emax0_m_s3[1] <= modulo_max(emax0_l_s2[2], emax0_l_s2[3]);
                emax1_m_s3[0] <= modulo_max(emax1_l_s2[0], emax1_l_s2[1]);
                emax1_m_s3[1] <= modulo_max(emax1_l_s2[2], emax1_l_s2[3]);
                omax0_m_s3[0] <= modulo_max(omax0_l_s2[0], omax0_l_s2[1]);
                omax0_m_s3[1] <= modulo_max(omax0_l_s2[2], omax0_l_s2[3]);
                omax1_m_s3[0] <= modulo_max(omax1_l_s2[0], omax1_l_s2[1]);
                omax1_m_s3[1] <= modulo_max(omax1_l_s2[2], omax1_l_s2[3]);
                la_even_s3    <= la_even_s2;
                la_odd_s3     <= la_odd_s2;
            end

            // S4: final max-tree level and intrinsic LLRs.
            if (s3_valid) begin
                ld_even_s4 <= max0_even_s4 - max1_even_s4;
                ld_odd_s4  <= max0_odd_s4  - max1_odd_s4;
                la_even_s4 <= la_even_s3;
                la_odd_s4  <= la_odd_s3;
            end

            // S5: extrinsic computation and output registers.
            if (s4_valid) begin
                llr_extr_even <= saturate(le_even_scaled_s5);
                llr_extr_odd  <= saturate(le_odd_scaled_s5);
                llr_intr_even <= ld_even_s4;
                llr_intr_odd  <= ld_odd_s4;
            end

        end
    end

endmodule
