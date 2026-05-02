//==============================================================================
// Module: bcjr_core 
// Description: Top-level SISO decoder core implementing the Radix-4 Max-Log
//              M-BCJR algorithm. Contains:
//              - Forward recursion unit (FR)
//              - Backward recursion unit (BR)
//              - Dummy backward recursion unit (DBR)
//              - LLR computation unit
//              - Double-buffered alpha and gamma memories (2 banks each)
//              - 6-state FSM for window scheduling
//
// Parallel segmentation strategy:
//   Block length is split equally across NUM_SISO modules.
//   Each core processes ceil(BLOCK_LEN / NUM_SISO) / WIN_LEN windows.
//   ALL windows always run the full 15 R4 cycles. For the last window
//   (which may be partial), out-of-range addresses get zero LLRs from
//   the external memory controller, and output addresses beyond
//   output_len are filtered (llr_out_valid suppressed).
//   This ensures FR, BR, and DBR running on different windows in
//   parallel never get desynchronized by a shortened window.
//
// CORE_ID-specific behavior (2 parallel SISO modules for current config):
//   All cores are kept in lockstep so the top level can write core0/core1
//   outputs into one folded BRAM word using a shared valid/address.
//   CORE_ID=0 starts with known initial state α(S0)=0, α(S1..S7)=-∞.
//   In paper-boundary mode, nonzero cores run a dummy-forward warm-up before
//   their first real window. In RTL-boundary mode, nonzero cores start with
//   equal alpha metrics.
//   CORE_ID=NUM_SISO-1 skips the last DBR and uses known terminal state
//   β(S0)=0, β(S1..S7)=-∞.
//
// Window schedule (for NUM_WINDOWS=N):
//   Slot 1:  FR=W1,     BR=idle,  DBR=W2'
//   Slot 2:  FR=W2,     BR=W1,    DBR=W3'
//   ...
//   Slot N:  FR=WN,     BR=W(N-1),DBR=W(N+1)'
//   Slot N+1:FR=idle,   BR=WN,    DBR=idle
//
// Reference: Studer et al., IEEE JSSC 2011, Section 15
//==============================================================================
module bcjr_core #(
    parameter CORE_ID      = 0,
    parameter NUM_SISO     = 2,    // Total number of parallel SISO modules
    parameter ADDR_W       = 12,
    parameter WIN_LEN      = 30,
    parameter PAPER_BOUNDARY = 1,
    parameter NUM_WINDOWS  = 54    // ceil((1600 + 3 tail)/30) for K=3200/N=2
) (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       start,
    input  wire [ADDR_W-1:0]          output_len,
    output reg                        done,

    // LLR Memory Interface
    output reg                        llr_req,
    output reg  [ADDR_W-1:0]          fr_llr_addr,
    output reg  [ADDR_W-1:0]          br_llr_addr,
    output reg  [ADDR_W-1:0]          dbr_llr_addr,
    output wire                       fr_dummy_active,
    input  wire                       llr_valid,
    // FR LLR inputs
    input  wire signed [4:0]          fr_sys_odd,  fr_sys_even,
    input  wire signed [4:0]          fr_par_odd,  fr_par_even,
    input  wire signed [4:0]          fr_apr_odd,  fr_apr_even,
    // BR LLR inputs
    input  wire signed [4:0]          br_sys_odd,  br_sys_even,
    input  wire signed [4:0]          br_par_odd,  br_par_even,
    input  wire signed [4:0]          br_apr_odd,  br_apr_even,
    // DBR LLR inputs
    input  wire signed [4:0]          dbr_sys_odd, dbr_sys_even,
    input  wire signed [4:0]          dbr_par_odd, dbr_par_even,
    input  wire signed [4:0]          dbr_apr_odd, dbr_apr_even,

    // Extrinsic LLR output
    output reg  signed [5:0]          llr_extr_odd_out,
    output reg  signed [5:0]          llr_extr_even_out,
    output reg  signed [9:0]          llr_intr_odd_out,
    output reg  signed [9:0]          llr_intr_even_out,
    output reg  [ADDR_W-1:0]          llr_out_addr,
    output reg                        llr_out_valid
);

    // =========================================================================
    // Local parameters
    // =========================================================================
    localparam LLR_W       = 5;
    localparam EXTR_W      = 6;
    localparam SM_W        = 10;
    localparam BM_R2_W     = 7;
    localparam BM_R4_W     = 8;
    localparam [3:0] WIN_LEN_R4 = WIN_LEN / 2;
    localparam NUM_STATES  = 8;
    localparam signed [SM_W-1:0] NEG_INF = -10'sd256;
    localparam integer LLR_PIPE_LATENCY = 5;
    localparam integer LLR_PIPE_LAST    = LLR_PIPE_LATENCY - 1;

    // =========================================================================
    // FSM States
    // =========================================================================
    localparam ST_IDLE     = 3'd0;
    localparam ST_LLR_REQ  = 3'd1;
    localparam ST_LLR_WAIT = 3'd2;
    localparam ST_COMPUTE  = 3'd3;
    localparam ST_LLR_OUT  = 3'd4;
    localparam ST_WIN_DONE = 3'd5;

    // =========================================================================
    // Internal registers
    // =========================================================================
    reg [2:0] state;
    reg [6:0] fr_win_idx;
    reg [6:0] br_win_idx;
    reg [6:0] dbr_win_idx;
    reg [3:0] step_cnt;

    reg fr_active_r, br_active_r, dbr_active_r;

    // LLR input registers
    reg signed [LLR_W-1:0] fr_sys_odd_r,  fr_sys_even_r;
    reg signed [LLR_W-1:0] fr_par_odd_r,  fr_par_even_r;
    reg signed [LLR_W-1:0] fr_apr_odd_r,  fr_apr_even_r;
    reg signed [LLR_W-1:0] br_sys_odd_r,  br_sys_even_r;
    reg signed [LLR_W-1:0] br_par_odd_r,  br_par_even_r;
    reg signed [LLR_W-1:0] br_apr_odd_r,  br_apr_even_r;
    reg signed [LLR_W-1:0] dbr_sys_odd_r, dbr_sys_even_r;
    reg signed [LLR_W-1:0] dbr_par_odd_r, dbr_par_even_r;
    reg signed [LLR_W-1:0] dbr_apr_odd_r, dbr_apr_even_r;

    // DBR → BR beta_init pipeline register
    reg signed [SM_W-1:0] br_beta_init_reg_0, br_beta_init_reg_1;
    reg signed [SM_W-1:0] br_beta_init_reg_2, br_beta_init_reg_3;
    reg signed [SM_W-1:0] br_beta_init_reg_4, br_beta_init_reg_5;
    reg signed [SM_W-1:0] br_beta_init_reg_6, br_beta_init_reg_7;

    reg fr_init_sm;
    reg dbr_init_sm;
    reg br_load_beta;
    reg br_phase_active;
    reg br_drain_pending;

    // =========================================================================
    // Forward Recursion Unit
    // =========================================================================
    wire [3:0]  fr_alpha_wr_addr;
    wire signed [SM_W-1:0] fr_alpha_wr_data_0, fr_alpha_wr_data_1;
    wire signed [SM_W-1:0] fr_alpha_wr_data_2, fr_alpha_wr_data_3;
    wire signed [SM_W-1:0] fr_alpha_wr_data_4, fr_alpha_wr_data_5;
    wire signed [SM_W-1:0] fr_alpha_wr_data_6, fr_alpha_wr_data_7;
    wire        fr_alpha_wr_en;
    wire [3:0]  fr_gamma_wr_addr;
    wire signed [BM_R2_W-1:0] fr_gamma_wr_data_0,  fr_gamma_wr_data_1;
    wire signed [BM_R2_W-1:0] fr_gamma_wr_data_2,  fr_gamma_wr_data_3;
    wire signed [BM_R2_W-1:0] fr_gamma_wr_data_4,  fr_gamma_wr_data_5;
    wire signed [BM_R2_W-1:0] fr_gamma_wr_data_6,  fr_gamma_wr_data_7;
    wire signed [BM_R2_W-1:0] fr_gamma_wr_data_8,  fr_gamma_wr_data_9;
    wire signed [BM_R2_W-1:0] fr_gamma_wr_data_10, fr_gamma_wr_data_11;
    wire signed [BM_R2_W-1:0] fr_gamma_wr_data_12, fr_gamma_wr_data_13;
    wire signed [BM_R2_W-1:0] fr_gamma_wr_data_14, fr_gamma_wr_data_15;
    wire signed [BM_R2_W-1:0] fr_gamma_wr_data_16, fr_gamma_wr_data_17;
    wire signed [BM_R2_W-1:0] fr_gamma_wr_data_18, fr_gamma_wr_data_19;
    wire signed [BM_R2_W-1:0] fr_gamma_wr_data_20, fr_gamma_wr_data_21;
    wire signed [BM_R2_W-1:0] fr_gamma_wr_data_22, fr_gamma_wr_data_23;
    wire signed [BM_R2_W-1:0] fr_gamma_wr_data_24, fr_gamma_wr_data_25;
    wire signed [BM_R2_W-1:0] fr_gamma_wr_data_26, fr_gamma_wr_data_27;
    wire signed [BM_R2_W-1:0] fr_gamma_wr_data_28, fr_gamma_wr_data_29;
    wire signed [BM_R2_W-1:0] fr_gamma_wr_data_30, fr_gamma_wr_data_31;
    wire        fr_gamma_wr_en;
    wire [3:0]  fr_step_cnt;
    wire        fr_window_done;

    wire fr_compute_active = fr_active_r && (state == ST_COMPUTE);

    forward_recursion_unit #(.CORE_ID(CORE_ID)) u_fr (
        .clk(clk), .rst_n(rst_n),
        .active(fr_compute_active),
        .win_len_r4(WIN_LEN_R4),
        .is_dummy(fr_win_idx == 7'd0),
        .init_sm(fr_init_sm),
        .sys_odd(fr_sys_odd_r), .sys_even(fr_sys_even_r),
        .par_odd(fr_par_odd_r), .par_even(fr_par_even_r),
        .apr_odd(fr_apr_odd_r), .apr_even(fr_apr_even_r),
        .alpha_wr_addr(fr_alpha_wr_addr),
        .alpha_wr_data_0(fr_alpha_wr_data_0), .alpha_wr_data_1(fr_alpha_wr_data_1),
        .alpha_wr_data_2(fr_alpha_wr_data_2), .alpha_wr_data_3(fr_alpha_wr_data_3),
        .alpha_wr_data_4(fr_alpha_wr_data_4), .alpha_wr_data_5(fr_alpha_wr_data_5),
        .alpha_wr_data_6(fr_alpha_wr_data_6), .alpha_wr_data_7(fr_alpha_wr_data_7),
        .alpha_wr_en(fr_alpha_wr_en),
        .gamma_wr_addr(fr_gamma_wr_addr),
        .gamma_wr_data_0(fr_gamma_wr_data_0),   .gamma_wr_data_1(fr_gamma_wr_data_1),
        .gamma_wr_data_2(fr_gamma_wr_data_2),   .gamma_wr_data_3(fr_gamma_wr_data_3),
        .gamma_wr_data_4(fr_gamma_wr_data_4),   .gamma_wr_data_5(fr_gamma_wr_data_5),
        .gamma_wr_data_6(fr_gamma_wr_data_6),   .gamma_wr_data_7(fr_gamma_wr_data_7),
        .gamma_wr_data_8(fr_gamma_wr_data_8),   .gamma_wr_data_9(fr_gamma_wr_data_9),
        .gamma_wr_data_10(fr_gamma_wr_data_10), .gamma_wr_data_11(fr_gamma_wr_data_11),
        .gamma_wr_data_12(fr_gamma_wr_data_12), .gamma_wr_data_13(fr_gamma_wr_data_13),
        .gamma_wr_data_14(fr_gamma_wr_data_14), .gamma_wr_data_15(fr_gamma_wr_data_15),
        .gamma_wr_data_16(fr_gamma_wr_data_16), .gamma_wr_data_17(fr_gamma_wr_data_17),
        .gamma_wr_data_18(fr_gamma_wr_data_18), .gamma_wr_data_19(fr_gamma_wr_data_19),
        .gamma_wr_data_20(fr_gamma_wr_data_20), .gamma_wr_data_21(fr_gamma_wr_data_21),
        .gamma_wr_data_22(fr_gamma_wr_data_22), .gamma_wr_data_23(fr_gamma_wr_data_23),
        .gamma_wr_data_24(fr_gamma_wr_data_24), .gamma_wr_data_25(fr_gamma_wr_data_25),
        .gamma_wr_data_26(fr_gamma_wr_data_26), .gamma_wr_data_27(fr_gamma_wr_data_27),
        .gamma_wr_data_28(fr_gamma_wr_data_28), .gamma_wr_data_29(fr_gamma_wr_data_29),
        .gamma_wr_data_30(fr_gamma_wr_data_30), .gamma_wr_data_31(fr_gamma_wr_data_31),
        .gamma_wr_en(fr_gamma_wr_en),
        .step_cnt(fr_step_cnt),
        .window_done(fr_window_done)
    );

    // =========================================================================
    // Dummy Backward Recursion Unit
    // =========================================================================
    wire signed [SM_W-1:0] dbr_final_beta_0, dbr_final_beta_1;
    wire signed [SM_W-1:0] dbr_final_beta_2, dbr_final_beta_3;
    wire signed [SM_W-1:0] dbr_final_beta_4, dbr_final_beta_5;
    wire signed [SM_W-1:0] dbr_final_beta_6, dbr_final_beta_7;
    wire dbr_window_done;

    wire dbr_compute_active = dbr_active_r && (state == ST_COMPUTE);

    dummy_backward_recursion_unit u_dbr (
        .clk(clk), .rst_n(rst_n),
        .active(dbr_compute_active),
        .init_sm(dbr_init_sm),
        .win_len_r4(WIN_LEN_R4),
        .sys_odd(dbr_sys_odd_r), .sys_even(dbr_sys_even_r),
        .par_odd(dbr_par_odd_r), .par_even(dbr_par_even_r),
        .apr_odd(dbr_apr_odd_r), .apr_even(dbr_apr_even_r),
        .final_beta_0(dbr_final_beta_0), .final_beta_1(dbr_final_beta_1),
        .final_beta_2(dbr_final_beta_2), .final_beta_3(dbr_final_beta_3),
        .final_beta_4(dbr_final_beta_4), .final_beta_5(dbr_final_beta_5),
        .final_beta_6(dbr_final_beta_6), .final_beta_7(dbr_final_beta_7),
        .window_done(dbr_window_done)
    );

    // =========================================================================
    // Double-Buffered Alpha Memory (2 banks)
    // =========================================================================
    wire alpha_wr_bank_sel = fr_win_idx[0];
    wire alpha_rd_bank_sel = br_win_idx[0];

    wire        a0_wr_en = fr_alpha_wr_en && (alpha_wr_bank_sel == 1'b0);
    wire [3:0]  a0_rd_addr;
    wire signed [SM_W-1:0] a0_rd_data_0, a0_rd_data_1, a0_rd_data_2, a0_rd_data_3;
    wire signed [SM_W-1:0] a0_rd_data_4, a0_rd_data_5, a0_rd_data_6, a0_rd_data_7;

    alpha_mem u_alpha0 (
        .clk(clk), .wr_en(a0_wr_en),
        .wr_addr(fr_alpha_wr_addr),
        .wr_data_0(fr_alpha_wr_data_0), .wr_data_1(fr_alpha_wr_data_1),
        .wr_data_2(fr_alpha_wr_data_2), .wr_data_3(fr_alpha_wr_data_3),
        .wr_data_4(fr_alpha_wr_data_4), .wr_data_5(fr_alpha_wr_data_5),
        .wr_data_6(fr_alpha_wr_data_6), .wr_data_7(fr_alpha_wr_data_7),
        .rd_addr(a0_rd_addr),
        .rd_data_0(a0_rd_data_0), .rd_data_1(a0_rd_data_1),
        .rd_data_2(a0_rd_data_2), .rd_data_3(a0_rd_data_3),
        .rd_data_4(a0_rd_data_4), .rd_data_5(a0_rd_data_5),
        .rd_data_6(a0_rd_data_6), .rd_data_7(a0_rd_data_7)
    );

    wire        a1_wr_en = fr_alpha_wr_en && (alpha_wr_bank_sel == 1'b1);
    wire [3:0]  a1_rd_addr;
    wire signed [SM_W-1:0] a1_rd_data_0, a1_rd_data_1, a1_rd_data_2, a1_rd_data_3;
    wire signed [SM_W-1:0] a1_rd_data_4, a1_rd_data_5, a1_rd_data_6, a1_rd_data_7;

    alpha_mem u_alpha1 (
        .clk(clk), .wr_en(a1_wr_en),
        .wr_addr(fr_alpha_wr_addr),
        .wr_data_0(fr_alpha_wr_data_0), .wr_data_1(fr_alpha_wr_data_1),
        .wr_data_2(fr_alpha_wr_data_2), .wr_data_3(fr_alpha_wr_data_3),
        .wr_data_4(fr_alpha_wr_data_4), .wr_data_5(fr_alpha_wr_data_5),
        .wr_data_6(fr_alpha_wr_data_6), .wr_data_7(fr_alpha_wr_data_7),
        .rd_addr(a1_rd_addr),
        .rd_data_0(a1_rd_data_0), .rd_data_1(a1_rd_data_1),
        .rd_data_2(a1_rd_data_2), .rd_data_3(a1_rd_data_3),
        .rd_data_4(a1_rd_data_4), .rd_data_5(a1_rd_data_5),
        .rd_data_6(a1_rd_data_6), .rd_data_7(a1_rd_data_7)
    );

    wire [3:0]  br_alpha_rd_addr;
    wire signed [SM_W-1:0] alpha_rd_data_0, alpha_rd_data_1, alpha_rd_data_2, alpha_rd_data_3;
    wire signed [SM_W-1:0] alpha_rd_data_4, alpha_rd_data_5, alpha_rd_data_6, alpha_rd_data_7;

    assign a0_rd_addr = br_alpha_rd_addr;
    assign a1_rd_addr = br_alpha_rd_addr;

    assign alpha_rd_data_0 = (alpha_rd_bank_sel == 1'b0) ? a0_rd_data_0 : a1_rd_data_0;
    assign alpha_rd_data_1 = (alpha_rd_bank_sel == 1'b0) ? a0_rd_data_1 : a1_rd_data_1;
    assign alpha_rd_data_2 = (alpha_rd_bank_sel == 1'b0) ? a0_rd_data_2 : a1_rd_data_2;
    assign alpha_rd_data_3 = (alpha_rd_bank_sel == 1'b0) ? a0_rd_data_3 : a1_rd_data_3;
    assign alpha_rd_data_4 = (alpha_rd_bank_sel == 1'b0) ? a0_rd_data_4 : a1_rd_data_4;
    assign alpha_rd_data_5 = (alpha_rd_bank_sel == 1'b0) ? a0_rd_data_5 : a1_rd_data_5;
    assign alpha_rd_data_6 = (alpha_rd_bank_sel == 1'b0) ? a0_rd_data_6 : a1_rd_data_6;
    assign alpha_rd_data_7 = (alpha_rd_bank_sel == 1'b0) ? a0_rd_data_7 : a1_rd_data_7;

    // =========================================================================
    // Double-Buffered Gamma Memory (2 banks)
    // =========================================================================
    wire gamma_wr_bank_sel = fr_win_idx[0];
    wire gamma_rd_bank_sel = br_win_idx[0];

    wire g0_wr_en = fr_gamma_wr_en && (gamma_wr_bank_sel == 1'b0);
    wire [3:0] g0_rd_addr;
    wire signed [BM_R2_W-1:0] g0_rd_0,  g0_rd_1,  g0_rd_2,  g0_rd_3;
    wire signed [BM_R2_W-1:0] g0_rd_4,  g0_rd_5,  g0_rd_6,  g0_rd_7;
    wire signed [BM_R2_W-1:0] g0_rd_8,  g0_rd_9,  g0_rd_10, g0_rd_11;
    wire signed [BM_R2_W-1:0] g0_rd_12, g0_rd_13, g0_rd_14, g0_rd_15;
    wire signed [BM_R2_W-1:0] g0_rd_16, g0_rd_17, g0_rd_18, g0_rd_19;
    wire signed [BM_R2_W-1:0] g0_rd_20, g0_rd_21, g0_rd_22, g0_rd_23;
    wire signed [BM_R2_W-1:0] g0_rd_24, g0_rd_25, g0_rd_26, g0_rd_27;
    wire signed [BM_R2_W-1:0] g0_rd_28, g0_rd_29, g0_rd_30, g0_rd_31;

    gamma_mem u_gamma0 (
        .clk(clk), .wr_en(g0_wr_en),
        .wr_addr(fr_gamma_wr_addr),
        .wr_data_0(fr_gamma_wr_data_0),   .wr_data_1(fr_gamma_wr_data_1),
        .wr_data_2(fr_gamma_wr_data_2),   .wr_data_3(fr_gamma_wr_data_3),
        .wr_data_4(fr_gamma_wr_data_4),   .wr_data_5(fr_gamma_wr_data_5),
        .wr_data_6(fr_gamma_wr_data_6),   .wr_data_7(fr_gamma_wr_data_7),
        .wr_data_8(fr_gamma_wr_data_8),   .wr_data_9(fr_gamma_wr_data_9),
        .wr_data_10(fr_gamma_wr_data_10), .wr_data_11(fr_gamma_wr_data_11),
        .wr_data_12(fr_gamma_wr_data_12), .wr_data_13(fr_gamma_wr_data_13),
        .wr_data_14(fr_gamma_wr_data_14), .wr_data_15(fr_gamma_wr_data_15),
        .wr_data_16(fr_gamma_wr_data_16), .wr_data_17(fr_gamma_wr_data_17),
        .wr_data_18(fr_gamma_wr_data_18), .wr_data_19(fr_gamma_wr_data_19),
        .wr_data_20(fr_gamma_wr_data_20), .wr_data_21(fr_gamma_wr_data_21),
        .wr_data_22(fr_gamma_wr_data_22), .wr_data_23(fr_gamma_wr_data_23),
        .wr_data_24(fr_gamma_wr_data_24), .wr_data_25(fr_gamma_wr_data_25),
        .wr_data_26(fr_gamma_wr_data_26), .wr_data_27(fr_gamma_wr_data_27),
        .wr_data_28(fr_gamma_wr_data_28), .wr_data_29(fr_gamma_wr_data_29),
        .wr_data_30(fr_gamma_wr_data_30), .wr_data_31(fr_gamma_wr_data_31),
        .rd_addr(g0_rd_addr),
        .rd_data_0(g0_rd_0),   .rd_data_1(g0_rd_1),   .rd_data_2(g0_rd_2),   .rd_data_3(g0_rd_3),
        .rd_data_4(g0_rd_4),   .rd_data_5(g0_rd_5),   .rd_data_6(g0_rd_6),   .rd_data_7(g0_rd_7),
        .rd_data_8(g0_rd_8),   .rd_data_9(g0_rd_9),   .rd_data_10(g0_rd_10), .rd_data_11(g0_rd_11),
        .rd_data_12(g0_rd_12), .rd_data_13(g0_rd_13), .rd_data_14(g0_rd_14), .rd_data_15(g0_rd_15),
        .rd_data_16(g0_rd_16), .rd_data_17(g0_rd_17), .rd_data_18(g0_rd_18), .rd_data_19(g0_rd_19),
        .rd_data_20(g0_rd_20), .rd_data_21(g0_rd_21), .rd_data_22(g0_rd_22), .rd_data_23(g0_rd_23),
        .rd_data_24(g0_rd_24), .rd_data_25(g0_rd_25), .rd_data_26(g0_rd_26), .rd_data_27(g0_rd_27),
        .rd_data_28(g0_rd_28), .rd_data_29(g0_rd_29), .rd_data_30(g0_rd_30), .rd_data_31(g0_rd_31)
    );

    wire g1_wr_en = fr_gamma_wr_en && (gamma_wr_bank_sel == 1'b1);
    wire [3:0] g1_rd_addr;
    wire signed [BM_R2_W-1:0] g1_rd_0,  g1_rd_1,  g1_rd_2,  g1_rd_3;
    wire signed [BM_R2_W-1:0] g1_rd_4,  g1_rd_5,  g1_rd_6,  g1_rd_7;
    wire signed [BM_R2_W-1:0] g1_rd_8,  g1_rd_9,  g1_rd_10, g1_rd_11;
    wire signed [BM_R2_W-1:0] g1_rd_12, g1_rd_13, g1_rd_14, g1_rd_15;
    wire signed [BM_R2_W-1:0] g1_rd_16, g1_rd_17, g1_rd_18, g1_rd_19;
    wire signed [BM_R2_W-1:0] g1_rd_20, g1_rd_21, g1_rd_22, g1_rd_23;
    wire signed [BM_R2_W-1:0] g1_rd_24, g1_rd_25, g1_rd_26, g1_rd_27;
    wire signed [BM_R2_W-1:0] g1_rd_28, g1_rd_29, g1_rd_30, g1_rd_31;

    gamma_mem u_gamma1 (
        .clk(clk), .wr_en(g1_wr_en),
        .wr_addr(fr_gamma_wr_addr),
        .wr_data_0(fr_gamma_wr_data_0),   .wr_data_1(fr_gamma_wr_data_1),
        .wr_data_2(fr_gamma_wr_data_2),   .wr_data_3(fr_gamma_wr_data_3),
        .wr_data_4(fr_gamma_wr_data_4),   .wr_data_5(fr_gamma_wr_data_5),
        .wr_data_6(fr_gamma_wr_data_6),   .wr_data_7(fr_gamma_wr_data_7),
        .wr_data_8(fr_gamma_wr_data_8),   .wr_data_9(fr_gamma_wr_data_9),
        .wr_data_10(fr_gamma_wr_data_10), .wr_data_11(fr_gamma_wr_data_11),
        .wr_data_12(fr_gamma_wr_data_12), .wr_data_13(fr_gamma_wr_data_13),
        .wr_data_14(fr_gamma_wr_data_14), .wr_data_15(fr_gamma_wr_data_15),
        .wr_data_16(fr_gamma_wr_data_16), .wr_data_17(fr_gamma_wr_data_17),
        .wr_data_18(fr_gamma_wr_data_18), .wr_data_19(fr_gamma_wr_data_19),
        .wr_data_20(fr_gamma_wr_data_20), .wr_data_21(fr_gamma_wr_data_21),
        .wr_data_22(fr_gamma_wr_data_22), .wr_data_23(fr_gamma_wr_data_23),
        .wr_data_24(fr_gamma_wr_data_24), .wr_data_25(fr_gamma_wr_data_25),
        .wr_data_26(fr_gamma_wr_data_26), .wr_data_27(fr_gamma_wr_data_27),
        .wr_data_28(fr_gamma_wr_data_28), .wr_data_29(fr_gamma_wr_data_29),
        .wr_data_30(fr_gamma_wr_data_30), .wr_data_31(fr_gamma_wr_data_31),
        .rd_addr(g1_rd_addr),
        .rd_data_0(g1_rd_0),   .rd_data_1(g1_rd_1),   .rd_data_2(g1_rd_2),   .rd_data_3(g1_rd_3),
        .rd_data_4(g1_rd_4),   .rd_data_5(g1_rd_5),   .rd_data_6(g1_rd_6),   .rd_data_7(g1_rd_7),
        .rd_data_8(g1_rd_8),   .rd_data_9(g1_rd_9),   .rd_data_10(g1_rd_10), .rd_data_11(g1_rd_11),
        .rd_data_12(g1_rd_12), .rd_data_13(g1_rd_13), .rd_data_14(g1_rd_14), .rd_data_15(g1_rd_15),
        .rd_data_16(g1_rd_16), .rd_data_17(g1_rd_17), .rd_data_18(g1_rd_18), .rd_data_19(g1_rd_19),
        .rd_data_20(g1_rd_20), .rd_data_21(g1_rd_21), .rd_data_22(g1_rd_22), .rd_data_23(g1_rd_23),
        .rd_data_24(g1_rd_24), .rd_data_25(g1_rd_25), .rd_data_26(g1_rd_26), .rd_data_27(g1_rd_27),
        .rd_data_28(g1_rd_28), .rd_data_29(g1_rd_29), .rd_data_30(g1_rd_30), .rd_data_31(g1_rd_31)
    );

    // Gamma read mux
    wire [3:0] br_gamma_rd_addr;
    wire signed [BM_R2_W-1:0] gamma_rd_0,  gamma_rd_1,  gamma_rd_2,  gamma_rd_3;
    wire signed [BM_R2_W-1:0] gamma_rd_4,  gamma_rd_5,  gamma_rd_6,  gamma_rd_7;
    wire signed [BM_R2_W-1:0] gamma_rd_8,  gamma_rd_9,  gamma_rd_10, gamma_rd_11;
    wire signed [BM_R2_W-1:0] gamma_rd_12, gamma_rd_13, gamma_rd_14, gamma_rd_15;
    wire signed [BM_R2_W-1:0] gamma_rd_16, gamma_rd_17, gamma_rd_18, gamma_rd_19;
    wire signed [BM_R2_W-1:0] gamma_rd_20, gamma_rd_21, gamma_rd_22, gamma_rd_23;
    wire signed [BM_R2_W-1:0] gamma_rd_24, gamma_rd_25, gamma_rd_26, gamma_rd_27;
    wire signed [BM_R2_W-1:0] gamma_rd_28, gamma_rd_29, gamma_rd_30, gamma_rd_31;

    assign g0_rd_addr = br_gamma_rd_addr;
    assign g1_rd_addr = br_gamma_rd_addr;

    assign gamma_rd_0  = (gamma_rd_bank_sel == 1'b0) ? g0_rd_0  : g1_rd_0;
    assign gamma_rd_1  = (gamma_rd_bank_sel == 1'b0) ? g0_rd_1  : g1_rd_1;
    assign gamma_rd_2  = (gamma_rd_bank_sel == 1'b0) ? g0_rd_2  : g1_rd_2;
    assign gamma_rd_3  = (gamma_rd_bank_sel == 1'b0) ? g0_rd_3  : g1_rd_3;
    assign gamma_rd_4  = (gamma_rd_bank_sel == 1'b0) ? g0_rd_4  : g1_rd_4;
    assign gamma_rd_5  = (gamma_rd_bank_sel == 1'b0) ? g0_rd_5  : g1_rd_5;
    assign gamma_rd_6  = (gamma_rd_bank_sel == 1'b0) ? g0_rd_6  : g1_rd_6;
    assign gamma_rd_7  = (gamma_rd_bank_sel == 1'b0) ? g0_rd_7  : g1_rd_7;
    assign gamma_rd_8  = (gamma_rd_bank_sel == 1'b0) ? g0_rd_8  : g1_rd_8;
    assign gamma_rd_9  = (gamma_rd_bank_sel == 1'b0) ? g0_rd_9  : g1_rd_9;
    assign gamma_rd_10 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_10 : g1_rd_10;
    assign gamma_rd_11 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_11 : g1_rd_11;
    assign gamma_rd_12 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_12 : g1_rd_12;
    assign gamma_rd_13 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_13 : g1_rd_13;
    assign gamma_rd_14 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_14 : g1_rd_14;
    assign gamma_rd_15 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_15 : g1_rd_15;
    assign gamma_rd_16 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_16 : g1_rd_16;
    assign gamma_rd_17 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_17 : g1_rd_17;
    assign gamma_rd_18 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_18 : g1_rd_18;
    assign gamma_rd_19 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_19 : g1_rd_19;
    assign gamma_rd_20 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_20 : g1_rd_20;
    assign gamma_rd_21 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_21 : g1_rd_21;
    assign gamma_rd_22 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_22 : g1_rd_22;
    assign gamma_rd_23 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_23 : g1_rd_23;
    assign gamma_rd_24 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_24 : g1_rd_24;
    assign gamma_rd_25 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_25 : g1_rd_25;
    assign gamma_rd_26 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_26 : g1_rd_26;
    assign gamma_rd_27 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_27 : g1_rd_27;
    assign gamma_rd_28 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_28 : g1_rd_28;
    assign gamma_rd_29 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_29 : g1_rd_29;
    assign gamma_rd_30 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_30 : g1_rd_30;
    assign gamma_rd_31 = (gamma_rd_bank_sel == 1'b0) ? g0_rd_31 : g1_rd_31;

    // =========================================================================
    // Backward Recursion Unit
    // =========================================================================
    wire signed [SM_W-1:0] br_beta_out_0, br_beta_out_1, br_beta_out_2, br_beta_out_3;
    wire signed [SM_W-1:0] br_beta_out_4, br_beta_out_5, br_beta_out_6, br_beta_out_7;
    wire [3:0]  br_cur_step;
    wire        br_beta_valid;
    wire        br_window_done;

    backward_recursion_unit u_br (
        .clk(clk), .rst_n(rst_n),
        .active(br_phase_active),
        .win_len_r4(WIN_LEN_R4),
        .sys_odd(br_sys_odd_r), .sys_even(br_sys_even_r),
        .par_odd(br_par_odd_r), .par_even(br_par_even_r),
        .apr_odd(br_apr_odd_r), .apr_even(br_apr_even_r),
        .beta_init_0(br_beta_init_reg_0), .beta_init_1(br_beta_init_reg_1),
        .beta_init_2(br_beta_init_reg_2), .beta_init_3(br_beta_init_reg_3),
        .beta_init_4(br_beta_init_reg_4), .beta_init_5(br_beta_init_reg_5),
        .beta_init_6(br_beta_init_reg_6), .beta_init_7(br_beta_init_reg_7),
        .load_beta_init(br_load_beta),
        .alpha_rd_addr(br_alpha_rd_addr),
        .alpha_rd_data_0(alpha_rd_data_0), .alpha_rd_data_1(alpha_rd_data_1),
        .alpha_rd_data_2(alpha_rd_data_2), .alpha_rd_data_3(alpha_rd_data_3),
        .alpha_rd_data_4(alpha_rd_data_4), .alpha_rd_data_5(alpha_rd_data_5),
        .alpha_rd_data_6(alpha_rd_data_6), .alpha_rd_data_7(alpha_rd_data_7),
        .gamma_rd_addr(br_gamma_rd_addr),
        .gamma_rd_data_0(gamma_rd_0),   .gamma_rd_data_1(gamma_rd_1),
        .gamma_rd_data_2(gamma_rd_2),   .gamma_rd_data_3(gamma_rd_3),
        .gamma_rd_data_4(gamma_rd_4),   .gamma_rd_data_5(gamma_rd_5),
        .gamma_rd_data_6(gamma_rd_6),   .gamma_rd_data_7(gamma_rd_7),
        .gamma_rd_data_8(gamma_rd_8),   .gamma_rd_data_9(gamma_rd_9),
        .gamma_rd_data_10(gamma_rd_10), .gamma_rd_data_11(gamma_rd_11),
        .gamma_rd_data_12(gamma_rd_12), .gamma_rd_data_13(gamma_rd_13),
        .gamma_rd_data_14(gamma_rd_14), .gamma_rd_data_15(gamma_rd_15),
        .gamma_rd_data_16(gamma_rd_16), .gamma_rd_data_17(gamma_rd_17),
        .gamma_rd_data_18(gamma_rd_18), .gamma_rd_data_19(gamma_rd_19),
        .gamma_rd_data_20(gamma_rd_20), .gamma_rd_data_21(gamma_rd_21),
        .gamma_rd_data_22(gamma_rd_22), .gamma_rd_data_23(gamma_rd_23),
        .gamma_rd_data_24(gamma_rd_24), .gamma_rd_data_25(gamma_rd_25),
        .gamma_rd_data_26(gamma_rd_26), .gamma_rd_data_27(gamma_rd_27),
        .gamma_rd_data_28(gamma_rd_28), .gamma_rd_data_29(gamma_rd_29),
        .gamma_rd_data_30(gamma_rd_30), .gamma_rd_data_31(gamma_rd_31),
        .beta_out_0(br_beta_out_0), .beta_out_1(br_beta_out_1),
        .beta_out_2(br_beta_out_2), .beta_out_3(br_beta_out_3),
        .beta_out_4(br_beta_out_4), .beta_out_5(br_beta_out_5),
        .beta_out_6(br_beta_out_6), .beta_out_7(br_beta_out_7),
        .cur_step(br_cur_step),
        .beta_valid(br_beta_valid),
        .window_done(br_window_done)
    );

    // =========================================================================
    // LLR Compute Unit
    // =========================================================================
    wire signed [EXTR_W-1:0] llr_extr_odd_w, llr_extr_even_w;
    wire signed [SM_W-1:0]   llr_intr_odd_w, llr_intr_even_w;
    wire llr_valid_w;

    llr_compute u_llr (
        .clk(clk), .rst_n(rst_n),
        .beta_valid(br_beta_valid),
        .beta_k_0(br_beta_out_0), .beta_k_1(br_beta_out_1),
        .beta_k_2(br_beta_out_2), .beta_k_3(br_beta_out_3),
        .beta_k_4(br_beta_out_4), .beta_k_5(br_beta_out_5),
        .beta_k_6(br_beta_out_6), .beta_k_7(br_beta_out_7),
        .alpha_km2_0(alpha_rd_data_0), .alpha_km2_1(alpha_rd_data_1),
        .alpha_km2_2(alpha_rd_data_2), .alpha_km2_3(alpha_rd_data_3),
        .alpha_km2_4(alpha_rd_data_4), .alpha_km2_5(alpha_rd_data_5),
        .alpha_km2_6(alpha_rd_data_6), .alpha_km2_7(alpha_rd_data_7),
        .bm_r2_odd_0(gamma_rd_0),   .bm_r2_odd_1(gamma_rd_1),
        .bm_r2_odd_2(gamma_rd_2),   .bm_r2_odd_3(gamma_rd_3),
        .bm_r2_odd_4(gamma_rd_4),   .bm_r2_odd_5(gamma_rd_5),
        .bm_r2_odd_6(gamma_rd_6),   .bm_r2_odd_7(gamma_rd_7),
        .bm_r2_odd_8(gamma_rd_8),   .bm_r2_odd_9(gamma_rd_9),
        .bm_r2_odd_10(gamma_rd_10), .bm_r2_odd_11(gamma_rd_11),
        .bm_r2_odd_12(gamma_rd_12), .bm_r2_odd_13(gamma_rd_13),
        .bm_r2_odd_14(gamma_rd_14), .bm_r2_odd_15(gamma_rd_15),
        .bm_r2_even_0(gamma_rd_16),  .bm_r2_even_1(gamma_rd_17),
        .bm_r2_even_2(gamma_rd_18),  .bm_r2_even_3(gamma_rd_19),
        .bm_r2_even_4(gamma_rd_20),  .bm_r2_even_5(gamma_rd_21),
        .bm_r2_even_6(gamma_rd_22),  .bm_r2_even_7(gamma_rd_23),
        .bm_r2_even_8(gamma_rd_24),  .bm_r2_even_9(gamma_rd_25),
        .bm_r2_even_10(gamma_rd_26), .bm_r2_even_11(gamma_rd_27),
        .bm_r2_even_12(gamma_rd_28), .bm_r2_even_13(gamma_rd_29),
        .bm_r2_even_14(gamma_rd_30), .bm_r2_even_15(gamma_rd_31),
        .sys_odd_k(br_sys_odd_r),   .apr_odd_k(br_apr_odd_r),
        .sys_even_k(br_sys_even_r), .apr_even_k(br_apr_even_r),
        .llr_extr_odd(llr_extr_odd_w),
        .llr_extr_even(llr_extr_even_w),
        .llr_intr_odd(llr_intr_odd_w),
        .llr_intr_even(llr_intr_even_w),
        .llr_valid(llr_valid_w)
    );

    // =========================================================================
    // LLR output address pipeline
    // =========================================================================
    reg [3:0] llr_step_pipe [0:LLR_PIPE_LAST];
    reg [6:0] llr_win_idx_pipe [0:LLR_PIPE_LAST];
    reg       llr_active_pipe [0:LLR_PIPE_LAST];
    integer   llr_pipe_i;

    reg                      llr_emit_candidate_r;
    reg [ADDR_W-1:0]         llr_emit_addr_r;
    reg signed [EXTR_W-1:0]  llr_emit_extr_odd_r;
    reg signed [EXTR_W-1:0]  llr_emit_extr_even_r;
    reg signed [SM_W-1:0]    llr_emit_intr_odd_r;
    reg signed [SM_W-1:0]    llr_emit_intr_even_r;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (llr_pipe_i = 0; llr_pipe_i < LLR_PIPE_LATENCY; llr_pipe_i = llr_pipe_i + 1) begin
                llr_step_pipe[llr_pipe_i]    <= 4'd0;
                llr_win_idx_pipe[llr_pipe_i] <= 7'd0;
                llr_active_pipe[llr_pipe_i]  <= 1'b0;
            end
        end else begin
            llr_step_pipe[0]    <= br_cur_step;
            llr_win_idx_pipe[0] <= br_win_idx;
            llr_active_pipe[0]  <= br_beta_valid && br_active_r;
            for (llr_pipe_i = 1; llr_pipe_i < LLR_PIPE_LATENCY; llr_pipe_i = llr_pipe_i + 1) begin
                llr_step_pipe[llr_pipe_i]    <= llr_step_pipe[llr_pipe_i-1];
                llr_win_idx_pipe[llr_pipe_i] <= llr_win_idx_pipe[llr_pipe_i-1];
                llr_active_pipe[llr_pipe_i]  <= llr_active_pipe[llr_pipe_i-1];
            end
        end
    end

    wire [ADDR_W-1:0] llr_pipe_addr =
        (llr_win_idx_pipe[LLR_PIPE_LAST] - 7'd1) * WIN_LEN
      + llr_step_pipe[LLR_PIPE_LAST] * 2;

    wire llr_pipe_busy =
        br_beta_valid | llr_valid_w |
        llr_active_pipe[0] | llr_active_pipe[1] | llr_active_pipe[2] |
        llr_active_pipe[3] | llr_active_pipe[4] |
        llr_emit_candidate_r;

    // =========================================================================
    // Top-Level FSM
    //
    // CORE_ID=0: FR init sets α(S0)=0, α(others)=NEG_INF.
    // Last core: last window's BR uses known terminal β(S0)=0,
    //            β(others)=NEG_INF instead of DBR output.
    // =========================================================================

    // Determine whether the last BR window needs known terminal beta
    // Last core is CORE_ID == NUM_SISO-1
    wire is_last_core = (CORE_ID == NUM_SISO - 1);
    wire use_known_terminal_beta = is_last_core && (br_win_idx == NUM_WINDOWS);
    wire is_paper_boundary = (PAPER_BOUNDARY != 0);
    wire is_prologue_slot = is_paper_boundary && (fr_win_idx == 7'd0);
    assign fr_dummy_active = is_prologue_slot && fr_active_r;

    always @(posedge clk) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            fr_win_idx     <= 7'd0;
            br_win_idx     <= 7'd0;
            dbr_win_idx    <= 7'd0;
            step_cnt       <= 4'd0;
            fr_active_r    <= 1'b0;
            br_active_r    <= 1'b0;
            dbr_active_r   <= 1'b0;
            done           <= 1'b0;
            llr_req        <= 1'b0;
            fr_llr_addr    <= {ADDR_W{1'b0}};
            br_llr_addr    <= {ADDR_W{1'b0}};
            dbr_llr_addr   <= {ADDR_W{1'b0}};
            llr_out_valid  <= 1'b0;
            llr_out_addr   <= {ADDR_W{1'b0}};
            llr_extr_odd_out  <= 6'sd0;
            llr_extr_even_out <= 6'sd0;
            llr_intr_odd_out  <= 10'sd0;
            llr_intr_even_out <= 10'sd0;
            llr_emit_candidate_r <= 1'b0;
            llr_emit_addr_r      <= {ADDR_W{1'b0}};
            llr_emit_extr_odd_r  <= {EXTR_W{1'b0}};
            llr_emit_extr_even_r <= {EXTR_W{1'b0}};
            llr_emit_intr_odd_r  <= {SM_W{1'b0}};
            llr_emit_intr_even_r <= {SM_W{1'b0}};
            fr_init_sm     <= 1'b0;
            dbr_init_sm    <= 1'b0;
            br_load_beta   <= 1'b0;
            br_phase_active <= 1'b0;
            br_drain_pending <= 1'b0;
            br_beta_init_reg_0 <= 10'sd0; br_beta_init_reg_1 <= 10'sd0;
            br_beta_init_reg_2 <= 10'sd0; br_beta_init_reg_3 <= 10'sd0;
            br_beta_init_reg_4 <= 10'sd0; br_beta_init_reg_5 <= 10'sd0;
            br_beta_init_reg_6 <= 10'sd0; br_beta_init_reg_7 <= 10'sd0;
            fr_sys_odd_r  <= 5'sd0; fr_sys_even_r <= 5'sd0;
            fr_par_odd_r  <= 5'sd0; fr_par_even_r <= 5'sd0;
            fr_apr_odd_r  <= 5'sd0; fr_apr_even_r <= 5'sd0;
            br_sys_odd_r  <= 5'sd0; br_sys_even_r <= 5'sd0;
            br_par_odd_r  <= 5'sd0; br_par_even_r <= 5'sd0;
            br_apr_odd_r  <= 5'sd0; br_apr_even_r <= 5'sd0;
            dbr_sys_odd_r <= 5'sd0; dbr_sys_even_r <= 5'sd0;
            dbr_par_odd_r <= 5'sd0; dbr_par_even_r <= 5'sd0;
            dbr_apr_odd_r <= 5'sd0; dbr_apr_even_r <= 5'sd0;
        end else begin
            // Default deassertions
            done          <= 1'b0;
            llr_out_valid <= 1'b0;
            llr_emit_candidate_r <= 1'b0;
            fr_init_sm    <= 1'b0;
            dbr_init_sm   <= 1'b0;
            br_load_beta  <= 1'b0;

            // Capture DBR final_beta into BR beta_init register
            if (dbr_window_done) begin
                br_beta_init_reg_0 <= dbr_final_beta_0;
                br_beta_init_reg_1 <= dbr_final_beta_1;
                br_beta_init_reg_2 <= dbr_final_beta_2;
                br_beta_init_reg_3 <= dbr_final_beta_3;
                br_beta_init_reg_4 <= dbr_final_beta_4;
                br_beta_init_reg_5 <= dbr_final_beta_5;
                br_beta_init_reg_6 <= dbr_final_beta_6;
                br_beta_init_reg_7 <= dbr_final_beta_7;
            end

            // LLR output from pipeline — filter addresses beyond output_len.
            // The top level may still feed valid tail trellis metrics beyond
            // output_len, but tail positions are not emitted as decisions.
            if (llr_emit_candidate_r) begin
                llr_out_addr <= llr_emit_addr_r;
                // Only emit valid output for addresses within this core's frame
                if ((llr_emit_addr_r + 1'b1) <= output_len) begin
                    llr_out_valid     <= 1'b1;
                    llr_extr_odd_out  <= llr_emit_extr_odd_r;
                    llr_extr_even_out <= llr_emit_extr_even_r;
                    llr_intr_odd_out  <= llr_emit_intr_odd_r;
                    llr_intr_even_out <= llr_emit_intr_even_r;
                end
            end

            if (llr_valid_w && llr_active_pipe[LLR_PIPE_LAST]) begin
                llr_emit_candidate_r <= 1'b1;
                llr_emit_addr_r      <= llr_pipe_addr;
                llr_emit_extr_odd_r  <= llr_extr_odd_w;
                llr_emit_extr_even_r <= llr_extr_even_w;
                llr_emit_intr_odd_r  <= llr_intr_odd_w;
                llr_emit_intr_even_r <= llr_intr_even_w;
            end

            case (state)
                // =============================================================
                ST_IDLE: begin
                    br_phase_active <= 1'b0;
                    if (start) begin
                        if (is_paper_boundary) begin
                            // One timing prologue keeps the folded top-level
                            // write-back aligned. Nonzero cores run dummy FR
                            // over the previous segment; core 0 idles with its
                            // known alpha state loaded.
                            fr_win_idx   <= 7'd0;
                            br_win_idx   <= 7'd0;
                            dbr_win_idx  <= 7'd0;
                            fr_active_r  <= (CORE_ID != 0);
                            br_active_r  <= 1'b0;
                            dbr_active_r <= 1'b0;
                            fr_init_sm   <= 1'b1;
                            dbr_init_sm  <= 1'b0;
                        end else begin
                            // Existing lockstep approximation.
                            fr_win_idx   <= 7'd1;
                            br_win_idx   <= 7'd0;  // inactive
                            dbr_win_idx  <= 7'd2;  // DBR starts at W2'
                            fr_active_r  <= 1'b1;
                            br_active_r  <= 1'b0;
                            dbr_active_r <= 1'b1;
                            fr_init_sm   <= 1'b1;  // Load initial state metrics
                            dbr_init_sm  <= 1'b1;  // DBR initialized at start of first window
                        end
                        step_cnt       <= 4'd0;
                        state          <= ST_LLR_REQ;
                    end
                end

                // =============================================================
                ST_LLR_REQ: begin
                    llr_req <= 1'b1;

                    // FR address computation
                    if (fr_active_r) begin
                        if (fr_win_idx == 7'd0) begin
                            // Dummy forward: previous segment's last window.
                            fr_llr_addr <= output_len - WIN_LEN + step_cnt * 2;
                        end else begin
                            fr_llr_addr <= (fr_win_idx - 7'd1) * WIN_LEN + step_cnt * 2;
                        end
                    end

                    if (br_active_r) begin
                        br_llr_addr <= (br_win_idx - 7'd1) * WIN_LEN
                                     + (WIN_LEN_R4 - 4'd1 - step_cnt) * 2;
                    end

                    if (dbr_active_r) begin
                        // DBR window N' processes data from window N (which is the NEXT window)
                        dbr_llr_addr <= (dbr_win_idx - 7'd1) * WIN_LEN
                                      + (WIN_LEN_R4 - 4'd1 - step_cnt) * 2;
                    end

                    state <= ST_LLR_WAIT;
                end

                // =============================================================
                ST_LLR_WAIT: begin
                    llr_req <= 1'b0;
                    if (llr_valid) begin
                        fr_sys_odd_r  <= fr_sys_odd;  fr_sys_even_r <= fr_sys_even;
                        fr_par_odd_r  <= fr_par_odd;  fr_par_even_r <= fr_par_even;
                        fr_apr_odd_r  <= fr_apr_odd;  fr_apr_even_r <= fr_apr_even;
                        br_sys_odd_r  <= br_sys_odd;  br_sys_even_r <= br_sys_even;
                        br_par_odd_r  <= br_par_odd;  br_par_even_r <= br_par_even;
                        br_apr_odd_r  <= br_apr_odd;  br_apr_even_r <= br_apr_even;
                        dbr_sys_odd_r <= dbr_sys_odd; dbr_sys_even_r <= dbr_sys_even;
                        dbr_par_odd_r <= dbr_par_odd; dbr_par_even_r <= dbr_par_even;
                        dbr_apr_odd_r <= dbr_apr_odd; dbr_apr_even_r <= dbr_apr_even;
                        state <= ST_COMPUTE;
                    end
                end

                // =============================================================
                ST_COMPUTE: begin
                    if (step_cnt == WIN_LEN_R4 - 4'd1) begin
                        // Window computation complete
                        if (br_active_r) begin
                            // For CORE_ID=7 last window: use known terminal beta
                            if (use_known_terminal_beta) begin
                                br_beta_init_reg_0 <= 10'sd0;
                                br_beta_init_reg_1 <= NEG_INF;
                                br_beta_init_reg_2 <= NEG_INF;
                                br_beta_init_reg_3 <= NEG_INF;
                                br_beta_init_reg_4 <= NEG_INF;
                                br_beta_init_reg_5 <= NEG_INF;
                                br_beta_init_reg_6 <= NEG_INF;
                                br_beta_init_reg_7 <= NEG_INF;
                            end
                            br_load_beta    <= 1'b1;
                            br_phase_active <= 1'b1;
                            br_drain_pending <= 1'b0;
                            state           <= ST_LLR_OUT;
                        end else begin
                            state <= ST_WIN_DONE;
                        end
                        step_cnt <= 4'd0;
                    end else begin
                        step_cnt <= step_cnt + 4'd1;
                        state    <= ST_LLR_REQ;
                    end
                end

                // =============================================================
                ST_LLR_OUT: begin
                    br_load_beta <= 1'b0;
                    if (br_window_done) begin
                        br_phase_active <= 1'b0;
                        br_drain_pending <= 1'b1;
                    end

                    if (br_drain_pending && !llr_pipe_busy) begin
                        br_drain_pending <= 1'b0;
                        state <= ST_WIN_DONE;
                    end
                end

                // =============================================================
                ST_WIN_DONE: begin
                    br_phase_active <= 1'b0;

                    // Nonzero cores spend the first slot on dummy forward warm-up.
                    // The next slot must run FR=W1 and DBR=W2 while BR stays idle;
                    // BR=W1 can only start after W1 alpha/gamma have been written.
                    if (is_paper_boundary && (fr_win_idx == 7'd0)) begin
                        fr_win_idx   <= 7'd1;
                        br_win_idx   <= 7'd0;
                        dbr_win_idx  <= 7'd2;
                        fr_active_r  <= 1'b1;
                        br_active_r  <= 1'b0;
                        dbr_active_r <= (NUM_WINDOWS >= 2);
                        step_cnt     <= 4'd0;
                        dbr_init_sm  <= 1'b1;
                        state        <= ST_LLR_REQ;
                    end else begin
                        // Advance window indices
                        fr_win_idx  <= fr_win_idx + 7'd1;
                        br_win_idx  <= br_win_idx + 7'd1;

                        // DBR scheduling
                        if (CORE_ID == 0) begin
                            // CORE_ID 0: DBR started at W2', advance normally
                            if (dbr_win_idx < NUM_WINDOWS + 1)
                                dbr_win_idx <= dbr_win_idx + 7'd1;
                            else
                                dbr_win_idx <= 7'd0;
                        end else begin
                            if (dbr_win_idx < NUM_WINDOWS + 1)
                                dbr_win_idx <= dbr_win_idx + 7'd1;
                            else
                                dbr_win_idx <= 7'd0;
                        end

                        // Update activation flags
                        fr_active_r  <= (fr_win_idx + 7'd1 <= NUM_WINDOWS);
                        br_active_r  <= (br_win_idx + 7'd1 >= 7'd1) && (br_win_idx + 7'd1 <= NUM_WINDOWS);

                        // DBR activation: active for windows 2..NUM_WINDOWS+1
                        // Last core: last window doesn't need DBR (known terminal state)
                        if (is_last_core && (dbr_win_idx >= NUM_WINDOWS))
                            dbr_active_r <= 1'b0;
                        else if (dbr_win_idx + 7'd1 >= 7'd2 && dbr_win_idx + 7'd1 <= NUM_WINDOWS + 1)
                            dbr_active_r <= 1'b1;
                        else
                            dbr_active_r <= 1'b0;

                        step_cnt <= 4'd0;

                        // Check if all done
                        if (br_win_idx >= NUM_WINDOWS) begin
                            done  <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            dbr_init_sm <= 1'b1;
                            state <= ST_LLR_REQ;
                        end
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
