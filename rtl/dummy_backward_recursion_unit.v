//==============================================================================
// Module: dummy_backward_recursion_unit
// Description: Traverses one window in backward direction to generate stable
//              beta initial values for BR. Does NOT access alpha/gamma memory.
//              Does NOT trigger LLR computation. Initializes all SMs to 0 at
//              the start of each window via load_init.
//
// Backward ACS wiring (TRANSPOSED from forward):
// For each s'', collect all s such that s'' is a predecessor of s (forward).
// This means: for backward β(s''), the 4 successors are the dest states s
// that have s'' in their forward predecessor list.
//
// Transposed table:
//   s''=000: appears as pred i=0 of dest {0,2,4,6} → succ β from {0,2,4,6}
//   s''=001: appears as pred i=1 of dest {0,2,4,6} → succ β from {0,2,4,6}
//   s''=010: appears as pred i=2 of dest {0,2,4,6} → succ β from {0,2,4,6}
//   s''=011: appears as pred i=3 of dest {0,2,4,6} → succ β from {0,2,4,6}
//   s''=100: appears as pred i=0 of dest {1,3,5,7} → succ β from {1,3,5,7}
//   s''=101: appears as pred i=1 of dest {1,3,5,7} → succ β from {1,3,5,7}
//   s''=110: appears as pred i=2 of dest {1,3,5,7} → succ β from {1,3,5,7}
//   s''=111: appears as pred i=3 of dest {1,3,5,7} → succ β from {1,3,5,7}
//
// Reference: Studer et al., IEEE JSSC 2011, Section 12
//==============================================================================
module dummy_backward_recursion_unit (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       active,
    input  wire                       init_sm,       // pulse to initialize beta SMs to 0
    input  wire [3:0]                 win_len_r4,
    input  wire signed [4:0]          sys_odd, sys_even,
    input  wire signed [4:0]          par_odd, par_even,
    input  wire signed [4:0]          apr_odd, apr_even,
    output reg  signed [9:0]          final_beta_0, final_beta_1,
    output reg  signed [9:0]          final_beta_2, final_beta_3,
    output reg  signed [9:0]          final_beta_4, final_beta_5,
    output reg  signed [9:0]          final_beta_6, final_beta_7,
    output reg                        window_done
);

    localparam SM_W      = 10;
    localparam PREPROC_W = 7;
    localparam BM_R2_W   = 7;
    localparam BM_R4_W   = 8;

    // =========================================================================
    // BM pipeline (identical to forward)
    // =========================================================================
    wire signed [PREPROC_W-1:0] pre_odd_0, pre_odd_1, pre_odd_2, pre_odd_3;
    wire signed [PREPROC_W-1:0] pre_even_0, pre_even_1, pre_even_2, pre_even_3;

    bm_preproc u_preproc (
        .sys_odd(sys_odd),   .par_odd(par_odd),   .apr_odd(apr_odd),
        .sys_even(sys_even), .par_even(par_even), .apr_even(apr_even),
        .pre_odd_0(pre_odd_0),   .pre_odd_1(pre_odd_1),
        .pre_odd_2(pre_odd_2),   .pre_odd_3(pre_odd_3),
        .pre_even_0(pre_even_0), .pre_even_1(pre_even_1),
        .pre_even_2(pre_even_2), .pre_even_3(pre_even_3)
    );

    wire signed [BM_R2_W-1:0] bm_r2_odd_0,  bm_r2_odd_1,  bm_r2_odd_2,  bm_r2_odd_3;
    wire signed [BM_R2_W-1:0] bm_r2_odd_4,  bm_r2_odd_5,  bm_r2_odd_6,  bm_r2_odd_7;
    wire signed [BM_R2_W-1:0] bm_r2_odd_8,  bm_r2_odd_9,  bm_r2_odd_10, bm_r2_odd_11;
    wire signed [BM_R2_W-1:0] bm_r2_odd_12, bm_r2_odd_13, bm_r2_odd_14, bm_r2_odd_15;
    wire signed [BM_R2_W-1:0] bm_r2_even_0,  bm_r2_even_1,  bm_r2_even_2,  bm_r2_even_3;
    wire signed [BM_R2_W-1:0] bm_r2_even_4,  bm_r2_even_5,  bm_r2_even_6,  bm_r2_even_7;
    wire signed [BM_R2_W-1:0] bm_r2_even_8,  bm_r2_even_9,  bm_r2_even_10, bm_r2_even_11;
    wire signed [BM_R2_W-1:0] bm_r2_even_12, bm_r2_even_13, bm_r2_even_14, bm_r2_even_15;

    bm_radix2 u_r2 (
        .pre_odd_0(pre_odd_0), .pre_odd_1(pre_odd_1),
        .pre_odd_2(pre_odd_2), .pre_odd_3(pre_odd_3),
        .pre_even_0(pre_even_0), .pre_even_1(pre_even_1),
        .pre_even_2(pre_even_2), .pre_even_3(pre_even_3),
        .bm_r2_odd_0(bm_r2_odd_0),   .bm_r2_odd_1(bm_r2_odd_1),
        .bm_r2_odd_2(bm_r2_odd_2),   .bm_r2_odd_3(bm_r2_odd_3),
        .bm_r2_odd_4(bm_r2_odd_4),   .bm_r2_odd_5(bm_r2_odd_5),
        .bm_r2_odd_6(bm_r2_odd_6),   .bm_r2_odd_7(bm_r2_odd_7),
        .bm_r2_odd_8(bm_r2_odd_8),   .bm_r2_odd_9(bm_r2_odd_9),
        .bm_r2_odd_10(bm_r2_odd_10), .bm_r2_odd_11(bm_r2_odd_11),
        .bm_r2_odd_12(bm_r2_odd_12), .bm_r2_odd_13(bm_r2_odd_13),
        .bm_r2_odd_14(bm_r2_odd_14), .bm_r2_odd_15(bm_r2_odd_15),
        .bm_r2_even_0(bm_r2_even_0),   .bm_r2_even_1(bm_r2_even_1),
        .bm_r2_even_2(bm_r2_even_2),   .bm_r2_even_3(bm_r2_even_3),
        .bm_r2_even_4(bm_r2_even_4),   .bm_r2_even_5(bm_r2_even_5),
        .bm_r2_even_6(bm_r2_even_6),   .bm_r2_even_7(bm_r2_even_7),
        .bm_r2_even_8(bm_r2_even_8),   .bm_r2_even_9(bm_r2_even_9),
        .bm_r2_even_10(bm_r2_even_10), .bm_r2_even_11(bm_r2_even_11),
        .bm_r2_even_12(bm_r2_even_12), .bm_r2_even_13(bm_r2_even_13),
        .bm_r2_even_14(bm_r2_even_14), .bm_r2_even_15(bm_r2_even_15)
    );

    wire signed [BM_R4_W-1:0] bm_r4_0,  bm_r4_1,  bm_r4_2,  bm_r4_3;
    wire signed [BM_R4_W-1:0] bm_r4_4,  bm_r4_5,  bm_r4_6,  bm_r4_7;
    wire signed [BM_R4_W-1:0] bm_r4_8,  bm_r4_9,  bm_r4_10, bm_r4_11;
    wire signed [BM_R4_W-1:0] bm_r4_12, bm_r4_13, bm_r4_14, bm_r4_15;
    wire signed [BM_R4_W-1:0] bm_r4_16, bm_r4_17, bm_r4_18, bm_r4_19;
    wire signed [BM_R4_W-1:0] bm_r4_20, bm_r4_21, bm_r4_22, bm_r4_23;
    wire signed [BM_R4_W-1:0] bm_r4_24, bm_r4_25, bm_r4_26, bm_r4_27;
    wire signed [BM_R4_W-1:0] bm_r4_28, bm_r4_29, bm_r4_30, bm_r4_31;

    bm_radix4 u_r4 (
        .bm_r2_odd_0(bm_r2_odd_0),   .bm_r2_odd_1(bm_r2_odd_1),
        .bm_r2_odd_2(bm_r2_odd_2),   .bm_r2_odd_3(bm_r2_odd_3),
        .bm_r2_odd_4(bm_r2_odd_4),   .bm_r2_odd_5(bm_r2_odd_5),
        .bm_r2_odd_6(bm_r2_odd_6),   .bm_r2_odd_7(bm_r2_odd_7),
        .bm_r2_odd_8(bm_r2_odd_8),   .bm_r2_odd_9(bm_r2_odd_9),
        .bm_r2_odd_10(bm_r2_odd_10), .bm_r2_odd_11(bm_r2_odd_11),
        .bm_r2_odd_12(bm_r2_odd_12), .bm_r2_odd_13(bm_r2_odd_13),
        .bm_r2_odd_14(bm_r2_odd_14), .bm_r2_odd_15(bm_r2_odd_15),
        .bm_r2_even_0(bm_r2_even_0),   .bm_r2_even_1(bm_r2_even_1),
        .bm_r2_even_2(bm_r2_even_2),   .bm_r2_even_3(bm_r2_even_3),
        .bm_r2_even_4(bm_r2_even_4),   .bm_r2_even_5(bm_r2_even_5),
        .bm_r2_even_6(bm_r2_even_6),   .bm_r2_even_7(bm_r2_even_7),
        .bm_r2_even_8(bm_r2_even_8),   .bm_r2_even_9(bm_r2_even_9),
        .bm_r2_even_10(bm_r2_even_10), .bm_r2_even_11(bm_r2_even_11),
        .bm_r2_even_12(bm_r2_even_12), .bm_r2_even_13(bm_r2_even_13),
        .bm_r2_even_14(bm_r2_even_14), .bm_r2_even_15(bm_r2_even_15),
        .bm_r4_0(bm_r4_0),   .bm_r4_1(bm_r4_1),   .bm_r4_2(bm_r4_2),   .bm_r4_3(bm_r4_3),
        .bm_r4_4(bm_r4_4),   .bm_r4_5(bm_r4_5),   .bm_r4_6(bm_r4_6),   .bm_r4_7(bm_r4_7),
        .bm_r4_8(bm_r4_8),   .bm_r4_9(bm_r4_9),   .bm_r4_10(bm_r4_10), .bm_r4_11(bm_r4_11),
        .bm_r4_12(bm_r4_12), .bm_r4_13(bm_r4_13), .bm_r4_14(bm_r4_14), .bm_r4_15(bm_r4_15),
        .bm_r4_16(bm_r4_16), .bm_r4_17(bm_r4_17), .bm_r4_18(bm_r4_18), .bm_r4_19(bm_r4_19),
        .bm_r4_20(bm_r4_20), .bm_r4_21(bm_r4_21), .bm_r4_22(bm_r4_22), .bm_r4_23(bm_r4_23),
        .bm_r4_24(bm_r4_24), .bm_r4_25(bm_r4_25), .bm_r4_26(bm_r4_26), .bm_r4_27(bm_r4_27),
        .bm_r4_28(bm_r4_28), .bm_r4_29(bm_r4_29), .bm_r4_30(bm_r4_30), .bm_r4_31(bm_r4_31)
    );

    wire signed [BM_R4_W-1:0] bm_r4_w [0:31];
    reg  signed [BM_R4_W-1:0] bm_r4_r [0:31];
    reg                       bm_valid_r;

    assign bm_r4_w[0]  = bm_r4_0;  assign bm_r4_w[1]  = bm_r4_1;
    assign bm_r4_w[2]  = bm_r4_2;  assign bm_r4_w[3]  = bm_r4_3;
    assign bm_r4_w[4]  = bm_r4_4;  assign bm_r4_w[5]  = bm_r4_5;
    assign bm_r4_w[6]  = bm_r4_6;  assign bm_r4_w[7]  = bm_r4_7;
    assign bm_r4_w[8]  = bm_r4_8;  assign bm_r4_w[9]  = bm_r4_9;
    assign bm_r4_w[10] = bm_r4_10; assign bm_r4_w[11] = bm_r4_11;
    assign bm_r4_w[12] = bm_r4_12; assign bm_r4_w[13] = bm_r4_13;
    assign bm_r4_w[14] = bm_r4_14; assign bm_r4_w[15] = bm_r4_15;
    assign bm_r4_w[16] = bm_r4_16; assign bm_r4_w[17] = bm_r4_17;
    assign bm_r4_w[18] = bm_r4_18; assign bm_r4_w[19] = bm_r4_19;
    assign bm_r4_w[20] = bm_r4_20; assign bm_r4_w[21] = bm_r4_21;
    assign bm_r4_w[22] = bm_r4_22; assign bm_r4_w[23] = bm_r4_23;
    assign bm_r4_w[24] = bm_r4_24; assign bm_r4_w[25] = bm_r4_25;
    assign bm_r4_w[26] = bm_r4_26; assign bm_r4_w[27] = bm_r4_27;
    assign bm_r4_w[28] = bm_r4_28; assign bm_r4_w[29] = bm_r4_29;
    assign bm_r4_w[30] = bm_r4_30; assign bm_r4_w[31] = bm_r4_31;

    // =========================================================================
    // 8× ACS units (BACKWARD direction — transposed wiring)
    // Uses load_init to initialize all SM to 0 at start of each window.
    // =========================================================================
    wire signed [SM_W-1:0] sm [0:7]; // beta state metrics
    reg running;           // latched high from init_sm until window completes
    reg capture_pending;   // delays final beta capture by 1 cycle

    // All DBR states init to 0 (equal probability — no state knowledge)
    wire signed [SM_W-1:0] dbr_init_val = 10'sd0;

    // ACS enable is delayed by one cycle so radix-4 BMs are registered first.
    wire bm_capture = active & running;
    wire acs_en     = bm_valid_r;

    // ACS for s''=000: successors s ∈ {0,2,4,6}, using bm_r4[s][0]
    acs_r4 u_acs_0 (
        .clk(clk), .rst_n(rst_n), .enable(acs_en),
        .bm_0(bm_r4_r[0]),  .bm_1(bm_r4_r[8]),  .bm_2(bm_r4_r[16]), .bm_3(bm_r4_r[24]),
        .sm_in_0(sm[0]), .sm_in_1(sm[2]), .sm_in_2(sm[4]), .sm_in_3(sm[6]),
        .init_val(dbr_init_val), .load_init(init_sm),
        .sm_out(sm[0])
    );
    // ACS for s''=001: using bm_r4[s][1] for s ∈ {0,2,4,6}
    acs_r4 u_acs_1 (
        .clk(clk), .rst_n(rst_n), .enable(acs_en),
        .bm_0(bm_r4_r[1]),  .bm_1(bm_r4_r[9]),  .bm_2(bm_r4_r[17]), .bm_3(bm_r4_r[25]),
        .sm_in_0(sm[0]), .sm_in_1(sm[2]), .sm_in_2(sm[4]), .sm_in_3(sm[6]),
        .init_val(dbr_init_val), .load_init(init_sm),
        .sm_out(sm[1])
    );
    // ACS for s''=010
    acs_r4 u_acs_2 (
        .clk(clk), .rst_n(rst_n), .enable(acs_en),
        .bm_0(bm_r4_r[2]),  .bm_1(bm_r4_r[10]), .bm_2(bm_r4_r[18]), .bm_3(bm_r4_r[26]),
        .sm_in_0(sm[0]), .sm_in_1(sm[2]), .sm_in_2(sm[4]), .sm_in_3(sm[6]),
        .init_val(dbr_init_val), .load_init(init_sm),
        .sm_out(sm[2])
    );
    // ACS for s''=011
    acs_r4 u_acs_3 (
        .clk(clk), .rst_n(rst_n), .enable(acs_en),
        .bm_0(bm_r4_r[3]),  .bm_1(bm_r4_r[11]), .bm_2(bm_r4_r[19]), .bm_3(bm_r4_r[27]),
        .sm_in_0(sm[0]), .sm_in_1(sm[2]), .sm_in_2(sm[4]), .sm_in_3(sm[6]),
        .init_val(dbr_init_val), .load_init(init_sm),
        .sm_out(sm[3])
    );
    // ACS for s''=100: successors s ∈ {1,3,5,7}, using bm_r4[s][0]
    acs_r4 u_acs_4 (
        .clk(clk), .rst_n(rst_n), .enable(acs_en),
        .bm_0(bm_r4_r[4]),  .bm_1(bm_r4_r[12]), .bm_2(bm_r4_r[20]), .bm_3(bm_r4_r[28]),
        .sm_in_0(sm[1]), .sm_in_1(sm[3]), .sm_in_2(sm[5]), .sm_in_3(sm[7]),
        .init_val(dbr_init_val), .load_init(init_sm),
        .sm_out(sm[4])
    );
    // ACS for s''=101: using bm_r4[s][1] for s ∈ {1,3,5,7}
    acs_r4 u_acs_5 (
        .clk(clk), .rst_n(rst_n), .enable(acs_en),
        .bm_0(bm_r4_r[5]),  .bm_1(bm_r4_r[13]), .bm_2(bm_r4_r[21]), .bm_3(bm_r4_r[29]),
        .sm_in_0(sm[1]), .sm_in_1(sm[3]), .sm_in_2(sm[5]), .sm_in_3(sm[7]),
        .init_val(dbr_init_val), .load_init(init_sm),
        .sm_out(sm[5])
    );
    // ACS for s''=110
    acs_r4 u_acs_6 (
        .clk(clk), .rst_n(rst_n), .enable(acs_en),
        .bm_0(bm_r4_r[6]),  .bm_1(bm_r4_r[14]), .bm_2(bm_r4_r[22]), .bm_3(bm_r4_r[30]),
        .sm_in_0(sm[1]), .sm_in_1(sm[3]), .sm_in_2(sm[5]), .sm_in_3(sm[7]),
        .init_val(dbr_init_val), .load_init(init_sm),
        .sm_out(sm[6])
    );
    // ACS for s''=111
    acs_r4 u_acs_7 (
        .clk(clk), .rst_n(rst_n), .enable(acs_en),
        .bm_0(bm_r4_r[7]),  .bm_1(bm_r4_r[15]), .bm_2(bm_r4_r[23]), .bm_3(bm_r4_r[31]),
        .sm_in_0(sm[1]), .sm_in_1(sm[3]), .sm_in_2(sm[5]), .sm_in_3(sm[7]),
        .init_val(dbr_init_val), .load_init(init_sm),
        .sm_out(sm[7])
    );

    // =========================================================================
    // Control FSM
    //
    // running:          latched HIGH by init_sm, stays HIGH until window done
    // capture_pending:  delays final beta capture by 1 cycle so sm[] reflects
    //                   the result of the LAST ACS update (not the pre-update)
    // =========================================================================
    reg [3:0] step_cnt;
    integer bm_i;

    always @(posedge clk) begin
        if (!rst_n) begin
            step_cnt        <= 4'd0;
            window_done     <= 1'b0;
            running         <= 1'b0;
            capture_pending <= 1'b0;
            bm_valid_r      <= 1'b0;
            for (bm_i = 0; bm_i < 32; bm_i = bm_i + 1)
                bm_r4_r[bm_i] <= {BM_R4_W{1'b0}};
            final_beta_0    <= 10'sd0; final_beta_1 <= 10'sd0;
            final_beta_2    <= 10'sd0; final_beta_3 <= 10'sd0;
            final_beta_4    <= 10'sd0; final_beta_5 <= 10'sd0;
            final_beta_6    <= 10'sd0; final_beta_7 <= 10'sd0;
        end else begin
            window_done <= 1'b0;
            bm_valid_r  <= 1'b0;

            if (bm_capture) begin
                bm_valid_r <= 1'b1;
                for (bm_i = 0; bm_i < 32; bm_i = bm_i + 1)
                    bm_r4_r[bm_i] <= bm_r4_w[bm_i];
            end

            // ----- Delayed capture: 1 cycle after last ACS step -----
            // sm[] now holds the result of all 15 ACS updates
            if (capture_pending) begin
                capture_pending <= 1'b0;
                window_done     <= 1'b1;
                running         <= 1'b0;
                final_beta_0    <= sm[0]; final_beta_1 <= sm[1];
                final_beta_2    <= sm[2]; final_beta_3 <= sm[3];
                final_beta_4    <= sm[4]; final_beta_5 <= sm[5];
                final_beta_6    <= sm[6]; final_beta_7 <= sm[7];
            end

            // ----- Initialization: load all SMs to 0, begin window -----
            if (init_sm) begin
                running         <= 1'b1;
                step_cnt        <= 4'd0;
                capture_pending <= 1'b0;
                bm_valid_r      <= 1'b0;
            end

            // ----- Step counter: counts delayed ACS updates, not BM captures -----
            if (bm_valid_r) begin
                if (step_cnt == win_len_r4 - 4'd1) begin
                    capture_pending <= 1'b1;
                    step_cnt        <= 4'd0;
                end else begin
                    step_cnt <= step_cnt + 4'd1;
                end
            end
        end
    end

endmodule
