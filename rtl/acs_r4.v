//==============================================================================
// Module: acs_r4
// Description: Registered Radix-4 Add-Compare-Select unit with
//              modulo-normalized comparison. One instance per trellis state
//              per recursion unit (24 total in bcjr_core).
//              Uses parallel 6-comparator + 24-entry LUT for winner selection.
//              Supports explicit initial value loading via load_init/init_val.
// Reference: Studer et al., IEEE JSSC 2011, Section 9
//==============================================================================
module acs_r4 (
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      enable,
    input  wire signed [7:0]         bm_0, bm_1, bm_2, bm_3,
    input  wire signed [9:0]         sm_in_0, sm_in_1, sm_in_2, sm_in_3,
    // Init mechanism: load_init takes priority over enable
    input  wire signed [9:0]         init_val,
    input  wire                      load_init,
    output reg  signed [9:0]         sm_out
);

    localparam SM_W    = 10;
    localparam BM_R4_W = 8;

    // -------------------------------------------------------------------------
    // Candidate computation
    // -------------------------------------------------------------------------
    wire signed [SM_W:0] cand_0, cand_1, cand_2, cand_3;
    assign cand_0 = $signed(sm_in_0) + $signed({{(SM_W-BM_R4_W){bm_0[BM_R4_W-1]}}, bm_0});
    assign cand_1 = $signed(sm_in_1) + $signed({{(SM_W-BM_R4_W){bm_1[BM_R4_W-1]}}, bm_1});
    assign cand_2 = $signed(sm_in_2) + $signed({{(SM_W-BM_R4_W){bm_2[BM_R4_W-1]}}, bm_2});
    assign cand_3 = $signed(sm_in_3) + $signed({{(SM_W-BM_R4_W){bm_3[BM_R4_W-1]}}, bm_3});
    
    // Truncate to SM_W — intentional modulo normalization
    wire signed [SM_W-1:0] c0, c1, c2, c3;
    assign c0 = cand_0[SM_W-1:0];
    assign c1 = cand_1[SM_W-1:0];
    assign c2 = cand_2[SM_W-1:0];
    assign c3 = cand_3[SM_W-1:0];

    // -------------------------------------------------------------------------
    // Modulo-normalization comparison
    // a > b iff (a - b) != 0 AND sign_bit(a - b) == 0
    // -------------------------------------------------------------------------
    wire signed [SM_W-1:0] diff_01, diff_02, diff_03, diff_12, diff_13, diff_23;
    assign diff_01 = c0 - c1;
    assign diff_02 = c0 - c2;
    assign diff_03 = c0 - c3;
    assign diff_12 = c1 - c2;
    assign diff_13 = c1 - c3;
    assign diff_23 = c2 - c3;

    wire cmp0, cmp1, cmp2, cmp3, cmp4, cmp5;
    assign cmp0 = (diff_01 != {SM_W{1'b0}}) && (diff_01[SM_W-1] == 1'b0); // A > B
    assign cmp1 = (diff_02 != {SM_W{1'b0}}) && (diff_02[SM_W-1] == 1'b0); // A > C
    assign cmp2 = (diff_03 != {SM_W{1'b0}}) && (diff_03[SM_W-1] == 1'b0); // A > D
    assign cmp3 = (diff_12 != {SM_W{1'b0}}) && (diff_12[SM_W-1] == 1'b0); // B > C
    assign cmp4 = (diff_13 != {SM_W{1'b0}}) && (diff_13[SM_W-1] == 1'b0); // B > D
    assign cmp5 = (diff_23 != {SM_W{1'b0}}) && (diff_23[SM_W-1] == 1'b0); // C > D

    // -------------------------------------------------------------------------
    // 6-bit LUT: all 24 valid orderings of {A,B,C,D}
    // -------------------------------------------------------------------------
    reg [1:0] sel;
    always @(*) begin
        case ({cmp5, cmp4, cmp3, cmp2, cmp1, cmp0})
            // A wins (6 orderings)
            6'b111111: sel = 2'd0;
            6'b011111: sel = 2'd0;
            6'b110111: sel = 2'd0;
            6'b100111: sel = 2'd0;
            6'b001111: sel = 2'd0;
            6'b000111: sel = 2'd0;
            // B wins (6 orderings)
            6'b111110: sel = 2'd1;
            6'b011110: sel = 2'd1;
            6'b111100: sel = 2'd1;
            6'b111000: sel = 2'd1;
            6'b011010: sel = 2'd1;
            6'b011000: sel = 2'd1;
            // C wins (6 orderings)
            6'b110101: sel = 2'd2;
            6'b100101: sel = 2'd2;
            6'b110100: sel = 2'd2;
            6'b110000: sel = 2'd2;
            6'b100001: sel = 2'd2;
            6'b100000: sel = 2'd2;
            // D wins (6 orderings)
            6'b001011: sel = 2'd3;
            6'b000011: sel = 2'd3;
            6'b001010: sel = 2'd3;
            6'b001000: sel = 2'd3;
            6'b000001: sel = 2'd3;
            6'b000000: sel = 2'd3;
            default:   sel = 2'd0;
        endcase
    end

    // -------------------------------------------------------------------------
    // Select winner
    // -------------------------------------------------------------------------
    reg signed [SM_W-1:0] winner;
    always @(*) begin
        case (sel)
            2'd0: winner = c0;
            2'd1: winner = c1;
            2'd2: winner = c2;
            2'd3: winner = c3;
            default: winner = c0;
        endcase
    end

    // -------------------------------------------------------------------------
    // Register update: load_init takes priority over enable
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n)
            sm_out <= {SM_W{1'b0}};
        else if (load_init)
            sm_out <= init_val;
        else if (enable)
            sm_out <= winner;
    end

endmodule
