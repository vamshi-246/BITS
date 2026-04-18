//==============================================================================
// Module: forward_recursion_unit
// Description: Computes forward state metrics (alpha) using the radix-4
//              Max-Log M-BCJR algorithm. Contains:
//              - BM preprocessing pipeline (preproc → R2 → R4)
//              - 8× ACS units (one per trellis state)
//              - FSM to write alpha and gamma to memory banks
//
//              Supports CORE_ID parameter for initial state metric values:
//              - CORE_ID=0: known initial state (α[0]=0, α[1..7]=-∞)
//              - Others:    all states initialized to 0 (equal probability)
//
// Reference: Studer et al., IEEE JSSC 2011, Section 10
//==============================================================================
module forward_recursion_unit #(
    parameter CORE_ID = 0  // SISO core index (0-7)
) (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       active,
    input  wire [3:0]                 win_len_r4,
    input  wire                       is_dummy,
    input  wire                       init_sm,
    input  wire signed [4:0]          sys_odd, sys_even,
    input  wire signed [4:0]          par_odd, par_even,
    input  wire signed [4:0]          apr_odd, apr_even,
    // Alpha memory write port
    output reg  [3:0]                 alpha_wr_addr,
    output wire signed [9:0]          alpha_wr_data_0, alpha_wr_data_1,
    output wire signed [9:0]          alpha_wr_data_2, alpha_wr_data_3,
    output wire signed [9:0]          alpha_wr_data_4, alpha_wr_data_5,
    output wire signed [9:0]          alpha_wr_data_6, alpha_wr_data_7,
    output reg                        alpha_wr_en,
    // Gamma memory write port (stores 32 R2 BMs)
    output reg  [3:0]                 gamma_wr_addr,
    output wire signed [6:0]          gamma_wr_data_0,  gamma_wr_data_1,
    output wire signed [6:0]          gamma_wr_data_2,  gamma_wr_data_3,
    output wire signed [6:0]          gamma_wr_data_4,  gamma_wr_data_5,
    output wire signed [6:0]          gamma_wr_data_6,  gamma_wr_data_7,
    output wire signed [6:0]          gamma_wr_data_8,  gamma_wr_data_9,
    output wire signed [6:0]          gamma_wr_data_10, gamma_wr_data_11,
    output wire signed [6:0]          gamma_wr_data_12, gamma_wr_data_13,
    output wire signed [6:0]          gamma_wr_data_14, gamma_wr_data_15,
    output wire signed [6:0]          gamma_wr_data_16, gamma_wr_data_17,
    output wire signed [6:0]          gamma_wr_data_18, gamma_wr_data_19,
    output wire signed [6:0]          gamma_wr_data_20, gamma_wr_data_21,
    output wire signed [6:0]          gamma_wr_data_22, gamma_wr_data_23,
    output wire signed [6:0]          gamma_wr_data_24, gamma_wr_data_25,
    output wire signed [6:0]          gamma_wr_data_26, gamma_wr_data_27,
    output wire signed [6:0]          gamma_wr_data_28, gamma_wr_data_29,
    output wire signed [6:0]          gamma_wr_data_30, gamma_wr_data_31,
    output reg                        gamma_wr_en,
    output reg  [3:0]                 step_cnt,
    output reg                        window_done
);

    localparam SM_W        = 10;
    localparam PREPROC_W   = 7;  // BPSK: 7-bit PRE values
    localparam BM_R2_W     = 7;
    localparam BM_R4_W     = 8;
    localparam NUM_STATES  = 8;

    // NEG_INF for modulo arithmetic: must satisfy |0 - NEG_INF| < 2^(SM_W-1)
    // -256 gives initial gap of 256 < 512 = 2^9, ensuring correct modulo comparison
    localparam signed [SM_W-1:0] NEG_INF = -10'sd256;

    // =========================================================================
    // BM pipeline: bm_preproc → bm_radix2 → bm_radix4
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

    // =========================================================================
    // 8× ACS units — FORWARD direction
    //
    // From Section 3.4: R4 predecessor table
    //   dest=000: preds={000, 001, 010, 011}, BMs: bm_r4[0..3]
    //   dest=001: preds={100, 101, 110, 111}, BMs: bm_r4[4..7]
    //   dest=010: preds={000, 001, 010, 011}, BMs: bm_r4[8..11]
    //   dest=011: preds={100, 101, 110, 111}, BMs: bm_r4[12..15]
    //   dest=100: preds={000, 001, 010, 011}, BMs: bm_r4[16..19]
    //   dest=101: preds={100, 101, 110, 111}, BMs: bm_r4[20..23]
    //   dest=110: preds={000, 001, 010, 011}, BMs: bm_r4[24..27]
    //   dest=111: preds={100, 101, 110, 111}, BMs: bm_r4[28..31]
    // =========================================================================
    wire signed [SM_W-1:0] sm [0:7]; // alpha state metrics (read from ACS outputs)
    reg acs_load_init; // asserted during init_sm cycle

    // Init values: CORE_ID=0 gets known initial state, others get all-zero
    wire signed [SM_W-1:0] acs_init_0 = (CORE_ID == 0) ? 10'sd0   : 10'sd0;
    wire signed [SM_W-1:0] acs_init_1 = (CORE_ID == 0) ? NEG_INF  : 10'sd0;
    wire signed [SM_W-1:0] acs_init_2 = (CORE_ID == 0) ? NEG_INF  : 10'sd0;
    wire signed [SM_W-1:0] acs_init_3 = (CORE_ID == 0) ? NEG_INF  : 10'sd0;
    wire signed [SM_W-1:0] acs_init_4 = (CORE_ID == 0) ? NEG_INF  : 10'sd0;
    wire signed [SM_W-1:0] acs_init_5 = (CORE_ID == 0) ? NEG_INF  : 10'sd0;
    wire signed [SM_W-1:0] acs_init_6 = (CORE_ID == 0) ? NEG_INF  : 10'sd0;
    wire signed [SM_W-1:0] acs_init_7 = (CORE_ID == 0) ? NEG_INF  : 10'sd0;

    // ACS for dest=000: preds={000,001,010,011}
    acs_r4 u_acs_0 (
        .clk(clk), .rst_n(rst_n), .enable(active),
        .bm_0(bm_r4_0),  .bm_1(bm_r4_1),  .bm_2(bm_r4_2),  .bm_3(bm_r4_3),
        .sm_in_0(sm[0]), .sm_in_1(sm[1]), .sm_in_2(sm[2]), .sm_in_3(sm[3]),
        .init_val(acs_init_0), .load_init(acs_load_init),
        .sm_out(sm[0])
    );
    // ACS for dest=001: preds={100,101,110,111}
    acs_r4 u_acs_1 (
        .clk(clk), .rst_n(rst_n), .enable(active),
        .bm_0(bm_r4_4),  .bm_1(bm_r4_5),  .bm_2(bm_r4_6),  .bm_3(bm_r4_7),
        .sm_in_0(sm[4]), .sm_in_1(sm[5]), .sm_in_2(sm[6]), .sm_in_3(sm[7]),
        .init_val(acs_init_1), .load_init(acs_load_init),
        .sm_out(sm[1])
    );
    // ACS for dest=010: preds={000,001,010,011}
    acs_r4 u_acs_2 (
        .clk(clk), .rst_n(rst_n), .enable(active),
        .bm_0(bm_r4_8),  .bm_1(bm_r4_9),  .bm_2(bm_r4_10), .bm_3(bm_r4_11),
        .sm_in_0(sm[0]), .sm_in_1(sm[1]), .sm_in_2(sm[2]), .sm_in_3(sm[3]),
        .init_val(acs_init_2), .load_init(acs_load_init),
        .sm_out(sm[2])
    );
    // ACS for dest=011: preds={100,101,110,111}
    acs_r4 u_acs_3 (
        .clk(clk), .rst_n(rst_n), .enable(active),
        .bm_0(bm_r4_12), .bm_1(bm_r4_13), .bm_2(bm_r4_14), .bm_3(bm_r4_15),
        .sm_in_0(sm[4]), .sm_in_1(sm[5]), .sm_in_2(sm[6]), .sm_in_3(sm[7]),
        .init_val(acs_init_3), .load_init(acs_load_init),
        .sm_out(sm[3])
    );
    // ACS for dest=100: preds={000,001,010,011}
    acs_r4 u_acs_4 (
        .clk(clk), .rst_n(rst_n), .enable(active),
        .bm_0(bm_r4_16), .bm_1(bm_r4_17), .bm_2(bm_r4_18), .bm_3(bm_r4_19),
        .sm_in_0(sm[0]), .sm_in_1(sm[1]), .sm_in_2(sm[2]), .sm_in_3(sm[3]),
        .init_val(acs_init_4), .load_init(acs_load_init),
        .sm_out(sm[4])
    );
    // ACS for dest=101: preds={100,101,110,111}
    acs_r4 u_acs_5 (
        .clk(clk), .rst_n(rst_n), .enable(active),
        .bm_0(bm_r4_20), .bm_1(bm_r4_21), .bm_2(bm_r4_22), .bm_3(bm_r4_23),
        .sm_in_0(sm[4]), .sm_in_1(sm[5]), .sm_in_2(sm[6]), .sm_in_3(sm[7]),
        .init_val(acs_init_5), .load_init(acs_load_init),
        .sm_out(sm[5])
    );
    // ACS for dest=110: preds={000,001,010,011}
    acs_r4 u_acs_6 (
        .clk(clk), .rst_n(rst_n), .enable(active),
        .bm_0(bm_r4_24), .bm_1(bm_r4_25), .bm_2(bm_r4_26), .bm_3(bm_r4_27),
        .sm_in_0(sm[0]), .sm_in_1(sm[1]), .sm_in_2(sm[2]), .sm_in_3(sm[3]),
        .init_val(acs_init_6), .load_init(acs_load_init),
        .sm_out(sm[6])
    );
    // ACS for dest=111: preds={100,101,110,111}
    acs_r4 u_acs_7 (
        .clk(clk), .rst_n(rst_n), .enable(active),
        .bm_0(bm_r4_28), .bm_1(bm_r4_29), .bm_2(bm_r4_30), .bm_3(bm_r4_31),
        .sm_in_0(sm[4]), .sm_in_1(sm[5]), .sm_in_2(sm[6]), .sm_in_3(sm[7]),
        .init_val(acs_init_7), .load_init(acs_load_init),
        .sm_out(sm[7])
    );

    // =========================================================================
    // Alpha & gamma write data (directly from ACS outputs and R2 BMs)
    // =========================================================================
    assign alpha_wr_data_0 = sm[0]; assign alpha_wr_data_1 = sm[1];
    assign alpha_wr_data_2 = sm[2]; assign alpha_wr_data_3 = sm[3];
    assign alpha_wr_data_4 = sm[4]; assign alpha_wr_data_5 = sm[5];
    assign alpha_wr_data_6 = sm[6]; assign alpha_wr_data_7 = sm[7];

    assign gamma_wr_data_0  = bm_r2_odd_0;  assign gamma_wr_data_1  = bm_r2_odd_1;
    assign gamma_wr_data_2  = bm_r2_odd_2;  assign gamma_wr_data_3  = bm_r2_odd_3;
    assign gamma_wr_data_4  = bm_r2_odd_4;  assign gamma_wr_data_5  = bm_r2_odd_5;
    assign gamma_wr_data_6  = bm_r2_odd_6;  assign gamma_wr_data_7  = bm_r2_odd_7;
    assign gamma_wr_data_8  = bm_r2_odd_8;  assign gamma_wr_data_9  = bm_r2_odd_9;
    assign gamma_wr_data_10 = bm_r2_odd_10; assign gamma_wr_data_11 = bm_r2_odd_11;
    assign gamma_wr_data_12 = bm_r2_odd_12; assign gamma_wr_data_13 = bm_r2_odd_13;
    assign gamma_wr_data_14 = bm_r2_odd_14; assign gamma_wr_data_15 = bm_r2_odd_15;
    assign gamma_wr_data_16 = bm_r2_even_0;  assign gamma_wr_data_17 = bm_r2_even_1;
    assign gamma_wr_data_18 = bm_r2_even_2;  assign gamma_wr_data_19 = bm_r2_even_3;
    assign gamma_wr_data_20 = bm_r2_even_4;  assign gamma_wr_data_21 = bm_r2_even_5;
    assign gamma_wr_data_22 = bm_r2_even_6;  assign gamma_wr_data_23 = bm_r2_even_7;
    assign gamma_wr_data_24 = bm_r2_even_8;  assign gamma_wr_data_25 = bm_r2_even_9;
    assign gamma_wr_data_26 = bm_r2_even_10; assign gamma_wr_data_27 = bm_r2_even_11;
    assign gamma_wr_data_28 = bm_r2_even_12; assign gamma_wr_data_29 = bm_r2_even_13;
    assign gamma_wr_data_30 = bm_r2_even_14; assign gamma_wr_data_31 = bm_r2_even_15;

    // =========================================================================
    // Control FSM
    // =========================================================================
    reg active_prev;

    always @(posedge clk) begin
        if (!rst_n) begin
            step_cnt      <= 4'd0;
            alpha_wr_addr <= 4'd0;
            gamma_wr_addr <= 4'd0;
            alpha_wr_en   <= 1'b0;
            gamma_wr_en   <= 1'b0;
            window_done   <= 1'b0;
            acs_load_init <= 1'b0;
            active_prev   <= 1'b0;
        end else begin
            window_done   <= 1'b0;
            acs_load_init <= 1'b0;
            active_prev   <= active;

            // Initialize ACS state metrics on init_sm pulse
            if (init_sm) begin
                acs_load_init <= 1'b1;
                step_cnt      <= 4'd0;
            end

            if (active) begin
                // Write alpha and gamma on every active cycle (unless dummy pass)
                if (!is_dummy) begin
                    alpha_wr_en   <= 1'b1;
                    alpha_wr_addr <= step_cnt;
                    gamma_wr_en   <= 1'b1;
                    gamma_wr_addr <= step_cnt;
                end else begin
                    alpha_wr_en <= 1'b0;
                    gamma_wr_en <= 1'b0;
                end

                // Step counter
                if (step_cnt == win_len_r4 - 4'd1) begin
                    window_done <= 1'b1;
                    step_cnt    <= 4'd0;
                end else begin
                    step_cnt <= step_cnt + 4'd1;
                end
            end else begin
                alpha_wr_en <= 1'b0;
                gamma_wr_en <= 1'b0;
            end
        end
    end

endmodule
