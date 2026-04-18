//==============================================================================
// Module: backward_recursion_unit
// Description: Processes one window in reverse trellis order. Uses externally
//              provided beta_init values loaded via load_init mechanism.
//              Reads alpha and gamma from memory. Drives LLR computation unit
//              with beta, alpha, and gamma data.
//              Uses backward (transposed) ACS wiring.
// Reference: Studer et al., IEEE JSSC 2011, Section 11
//==============================================================================
module backward_recursion_unit (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       active,
    input  wire [3:0]                 win_len_r4,
    input  wire signed [4:0]          sys_odd, sys_even,
    input  wire signed [4:0]          par_odd, par_even,
    input  wire signed [4:0]          apr_odd, apr_even,
    // Beta initialization (from DBR or known terminal state)
    input  wire signed [9:0]          beta_init_0, beta_init_1,
    input  wire signed [9:0]          beta_init_2, beta_init_3,
    input  wire signed [9:0]          beta_init_4, beta_init_5,
    input  wire signed [9:0]          beta_init_6, beta_init_7,
    input  wire                       load_beta_init,  // pulse to load beta_init
    // Alpha memory read port
    output reg  [3:0]                 alpha_rd_addr,
    input  wire signed [9:0]          alpha_rd_data_0, alpha_rd_data_1,
    input  wire signed [9:0]          alpha_rd_data_2, alpha_rd_data_3,
    input  wire signed [9:0]          alpha_rd_data_4, alpha_rd_data_5,
    input  wire signed [9:0]          alpha_rd_data_6, alpha_rd_data_7,
    // Gamma memory read port
    output reg  [3:0]                 gamma_rd_addr,
    input  wire signed [6:0]          gamma_rd_data_0,  gamma_rd_data_1,
    input  wire signed [6:0]          gamma_rd_data_2,  gamma_rd_data_3,
    input  wire signed [6:0]          gamma_rd_data_4,  gamma_rd_data_5,
    input  wire signed [6:0]          gamma_rd_data_6,  gamma_rd_data_7,
    input  wire signed [6:0]          gamma_rd_data_8,  gamma_rd_data_9,
    input  wire signed [6:0]          gamma_rd_data_10, gamma_rd_data_11,
    input  wire signed [6:0]          gamma_rd_data_12, gamma_rd_data_13,
    input  wire signed [6:0]          gamma_rd_data_14, gamma_rd_data_15,
    input  wire signed [6:0]          gamma_rd_data_16, gamma_rd_data_17,
    input  wire signed [6:0]          gamma_rd_data_18, gamma_rd_data_19,
    input  wire signed [6:0]          gamma_rd_data_20, gamma_rd_data_21,
    input  wire signed [6:0]          gamma_rd_data_22, gamma_rd_data_23,
    input  wire signed [6:0]          gamma_rd_data_24, gamma_rd_data_25,
    input  wire signed [6:0]          gamma_rd_data_26, gamma_rd_data_27,
    input  wire signed [6:0]          gamma_rd_data_28, gamma_rd_data_29,
    input  wire signed [6:0]          gamma_rd_data_30, gamma_rd_data_31,
    // Outputs to LLR compute
    output reg  signed [9:0]          beta_out_0, beta_out_1,
    output reg  signed [9:0]          beta_out_2, beta_out_3,
    output reg  signed [9:0]          beta_out_4, beta_out_5,
    output reg  signed [9:0]          beta_out_6, beta_out_7,
    output reg  [3:0]                 cur_step,
    output reg                        beta_valid,
    output reg                        window_done
);

    localparam SM_W = 10;

    // =========================================================================
    // Reconstruct radix-4 BMs from gamma memory's stored radix-2 BMs
    // gamma_rd_data[0..15] = odd R2 BMs, gamma_rd_data[16..31] = even R2 BMs
    // =========================================================================
    wire signed [7:0] bm_r4_0,  bm_r4_1,  bm_r4_2,  bm_r4_3;
    wire signed [7:0] bm_r4_4,  bm_r4_5,  bm_r4_6,  bm_r4_7;
    wire signed [7:0] bm_r4_8,  bm_r4_9,  bm_r4_10, bm_r4_11;
    wire signed [7:0] bm_r4_12, bm_r4_13, bm_r4_14, bm_r4_15;
    wire signed [7:0] bm_r4_16, bm_r4_17, bm_r4_18, bm_r4_19;
    wire signed [7:0] bm_r4_20, bm_r4_21, bm_r4_22, bm_r4_23;
    wire signed [7:0] bm_r4_24, bm_r4_25, bm_r4_26, bm_r4_27;
    wire signed [7:0] bm_r4_28, bm_r4_29, bm_r4_30, bm_r4_31;

    bm_radix4 u_r4_from_gamma (
        .bm_r2_odd_0(gamma_rd_data_0),   .bm_r2_odd_1(gamma_rd_data_1),
        .bm_r2_odd_2(gamma_rd_data_2),   .bm_r2_odd_3(gamma_rd_data_3),
        .bm_r2_odd_4(gamma_rd_data_4),   .bm_r2_odd_5(gamma_rd_data_5),
        .bm_r2_odd_6(gamma_rd_data_6),   .bm_r2_odd_7(gamma_rd_data_7),
        .bm_r2_odd_8(gamma_rd_data_8),   .bm_r2_odd_9(gamma_rd_data_9),
        .bm_r2_odd_10(gamma_rd_data_10), .bm_r2_odd_11(gamma_rd_data_11),
        .bm_r2_odd_12(gamma_rd_data_12), .bm_r2_odd_13(gamma_rd_data_13),
        .bm_r2_odd_14(gamma_rd_data_14), .bm_r2_odd_15(gamma_rd_data_15),
        .bm_r2_even_0(gamma_rd_data_16),  .bm_r2_even_1(gamma_rd_data_17),
        .bm_r2_even_2(gamma_rd_data_18),  .bm_r2_even_3(gamma_rd_data_19),
        .bm_r2_even_4(gamma_rd_data_20),  .bm_r2_even_5(gamma_rd_data_21),
        .bm_r2_even_6(gamma_rd_data_22),  .bm_r2_even_7(gamma_rd_data_23),
        .bm_r2_even_8(gamma_rd_data_24),  .bm_r2_even_9(gamma_rd_data_25),
        .bm_r2_even_10(gamma_rd_data_26), .bm_r2_even_11(gamma_rd_data_27),
        .bm_r2_even_12(gamma_rd_data_28), .bm_r2_even_13(gamma_rd_data_29),
        .bm_r2_even_14(gamma_rd_data_30), .bm_r2_even_15(gamma_rd_data_31),
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
    // 8× ACS units (BACKWARD direction — transposed wiring)
    // Same wiring as dummy_backward_recursion_unit
    // Uses load_init to load beta_init from DBR/known terminal state
    // =========================================================================
    wire signed [SM_W-1:0] sm [0:7]; // beta state metrics
    reg  acs_enable_reg;
    reg  acs_load_init_reg;

    // ACS for s''=000
    acs_r4 u_acs_0 (
        .clk(clk), .rst_n(rst_n), .enable(acs_enable_reg),
        .bm_0(bm_r4_0),  .bm_1(bm_r4_8),  .bm_2(bm_r4_16), .bm_3(bm_r4_24),
        .sm_in_0(sm[0]), .sm_in_1(sm[2]), .sm_in_2(sm[4]), .sm_in_3(sm[6]),
        .init_val(beta_init_0), .load_init(acs_load_init_reg),
        .sm_out(sm[0])
    );
    // ACS for s''=001
    acs_r4 u_acs_1 (
        .clk(clk), .rst_n(rst_n), .enable(acs_enable_reg),
        .bm_0(bm_r4_1),  .bm_1(bm_r4_9),  .bm_2(bm_r4_17), .bm_3(bm_r4_25),
        .sm_in_0(sm[0]), .sm_in_1(sm[2]), .sm_in_2(sm[4]), .sm_in_3(sm[6]),
        .init_val(beta_init_1), .load_init(acs_load_init_reg),
        .sm_out(sm[1])
    );
    // ACS for s''=010
    acs_r4 u_acs_2 (
        .clk(clk), .rst_n(rst_n), .enable(acs_enable_reg),
        .bm_0(bm_r4_2),  .bm_1(bm_r4_10), .bm_2(bm_r4_18), .bm_3(bm_r4_26),
        .sm_in_0(sm[0]), .sm_in_1(sm[2]), .sm_in_2(sm[4]), .sm_in_3(sm[6]),
        .init_val(beta_init_2), .load_init(acs_load_init_reg),
        .sm_out(sm[2])
    );
    // ACS for s''=011
    acs_r4 u_acs_3 (
        .clk(clk), .rst_n(rst_n), .enable(acs_enable_reg),
        .bm_0(bm_r4_3),  .bm_1(bm_r4_11), .bm_2(bm_r4_19), .bm_3(bm_r4_27),
        .sm_in_0(sm[0]), .sm_in_1(sm[2]), .sm_in_2(sm[4]), .sm_in_3(sm[6]),
        .init_val(beta_init_3), .load_init(acs_load_init_reg),
        .sm_out(sm[3])
    );
    // ACS for s''=100
    acs_r4 u_acs_4 (
        .clk(clk), .rst_n(rst_n), .enable(acs_enable_reg),
        .bm_0(bm_r4_4),  .bm_1(bm_r4_12), .bm_2(bm_r4_20), .bm_3(bm_r4_28),
        .sm_in_0(sm[1]), .sm_in_1(sm[3]), .sm_in_2(sm[5]), .sm_in_3(sm[7]),
        .init_val(beta_init_4), .load_init(acs_load_init_reg),
        .sm_out(sm[4])
    );
    // ACS for s''=101
    acs_r4 u_acs_5 (
        .clk(clk), .rst_n(rst_n), .enable(acs_enable_reg),
        .bm_0(bm_r4_5),  .bm_1(bm_r4_13), .bm_2(bm_r4_21), .bm_3(bm_r4_29),
        .sm_in_0(sm[1]), .sm_in_1(sm[3]), .sm_in_2(sm[5]), .sm_in_3(sm[7]),
        .init_val(beta_init_5), .load_init(acs_load_init_reg),
        .sm_out(sm[5])
    );
    // ACS for s''=110
    acs_r4 u_acs_6 (
        .clk(clk), .rst_n(rst_n), .enable(acs_enable_reg),
        .bm_0(bm_r4_6),  .bm_1(bm_r4_14), .bm_2(bm_r4_22), .bm_3(bm_r4_30),
        .sm_in_0(sm[1]), .sm_in_1(sm[3]), .sm_in_2(sm[5]), .sm_in_3(sm[7]),
        .init_val(beta_init_6), .load_init(acs_load_init_reg),
        .sm_out(sm[6])
    );
    // ACS for s''=111
    acs_r4 u_acs_7 (
        .clk(clk), .rst_n(rst_n), .enable(acs_enable_reg),
        .bm_0(bm_r4_7),  .bm_1(bm_r4_15), .bm_2(bm_r4_23), .bm_3(bm_r4_31),
        .sm_in_0(sm[1]), .sm_in_1(sm[3]), .sm_in_2(sm[5]), .sm_in_3(sm[7]),
        .init_val(beta_init_7), .load_init(acs_load_init_reg),
        .sm_out(sm[7])
    );

    // =========================================================================
    // Memory read pipeline & control FSM
    //
    // Pipeline:
    //   Cycle 0: Load beta_init into ACS SMs. Issue read for addr=win_len_r4-1.
    //   Cycle 1: Memory data arrives (wait cycle).
    //   Cycle 2: Gamma R4 BMs ready. Enable ACS. Output beta + beta_valid.
    //            Issue next read addr. Continue.
    //   ...decrement until step 0.
    // =========================================================================
    reg [1:0] pipe_state;
    localparam PIPE_IDLE      = 2'd0;
    localparam PIPE_INIT      = 2'd1;
    localparam PIPE_WAIT_MEM  = 2'd2;
    localparam PIPE_RUNNING   = 2'd3;

    reg [3:0] rd_step;

    always @(posedge clk) begin
        if (!rst_n) begin
            pipe_state       <= PIPE_IDLE;
            rd_step          <= 4'd0;
            alpha_rd_addr    <= 4'd0;
            gamma_rd_addr    <= 4'd0;
            beta_valid       <= 1'b0;
            window_done      <= 1'b0;
            cur_step         <= 4'd0;
            acs_enable_reg   <= 1'b0;
            acs_load_init_reg <= 1'b0;
            beta_out_0 <= 10'sd0; beta_out_1 <= 10'sd0;
            beta_out_2 <= 10'sd0; beta_out_3 <= 10'sd0;
            beta_out_4 <= 10'sd0; beta_out_5 <= 10'sd0;
            beta_out_6 <= 10'sd0; beta_out_7 <= 10'sd0;
        end else begin
            beta_valid         <= 1'b0;
            window_done        <= 1'b0;
            acs_enable_reg     <= 1'b0;
            acs_load_init_reg  <= 1'b0;

            case (pipe_state)
                PIPE_IDLE: begin
                    if (active && load_beta_init) begin
                        // Load beta_init into ACS units
                        acs_load_init_reg <= 1'b1;
                        // Issue first read: address = win_len_r4 - 1
                        rd_step       <= win_len_r4 - 4'd1;
                        alpha_rd_addr <= win_len_r4 - 4'd1;
                        gamma_rd_addr <= win_len_r4 - 4'd1;
                        pipe_state    <= PIPE_WAIT_MEM;
                    end
                end

                PIPE_WAIT_MEM: begin
                    // Memory data will be available next cycle
                    pipe_state <= PIPE_RUNNING;
                end

                PIPE_RUNNING: begin
                    // Memory data is now valid for rd_step
                    // Enable ACS to compute backward update using these BMs
                    acs_enable_reg <= 1'b1;

                    // Output current beta to LLR compute
                    beta_out_0 <= sm[0]; beta_out_1 <= sm[1];
                    beta_out_2 <= sm[2]; beta_out_3 <= sm[3];
                    beta_out_4 <= sm[4]; beta_out_5 <= sm[5];
                    beta_out_6 <= sm[6]; beta_out_7 <= sm[7];
                    cur_step   <= rd_step;
                    beta_valid <= 1'b1;

                    if (rd_step == 4'd0) begin
                        // All steps processed
                        window_done <= 1'b1;
                        pipe_state  <= PIPE_IDLE;
                    end else begin
                        // Issue next read
                        rd_step       <= rd_step - 4'd1;
                        alpha_rd_addr <= rd_step - 4'd1;
                        gamma_rd_addr <= rd_step - 4'd1;
                        pipe_state    <= PIPE_WAIT_MEM;
                    end
                end

                default: pipe_state <= PIPE_IDLE;
            endcase
        end
    end

endmodule
