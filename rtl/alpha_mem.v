//==============================================================================
// Module: alpha_mem
// Description: Register-array based alpha state metric memory bank.
//              Synchronous write, synchronous read (1-cycle read latency).
//              Stores 8 state metrics (each SM_W=10 bits) per address.
//              Depth: WIN_LEN_R4 = 15 entries.
// Reference: Studer et al., IEEE JSSC 2011, Section 14
//==============================================================================
module alpha_mem (
    input  wire         clk,
    input  wire         wr_en,
    input  wire [3:0]   wr_addr,               // 0..14
    input  wire signed [9:0] wr_data_0,
    input  wire signed [9:0] wr_data_1,
    input  wire signed [9:0] wr_data_2,
    input  wire signed [9:0] wr_data_3,
    input  wire signed [9:0] wr_data_4,
    input  wire signed [9:0] wr_data_5,
    input  wire signed [9:0] wr_data_6,
    input  wire signed [9:0] wr_data_7,
    input  wire [3:0]   rd_addr,
    output reg  signed [9:0] rd_data_0,
    output reg  signed [9:0] rd_data_1,
    output reg  signed [9:0] rd_data_2,
    output reg  signed [9:0] rd_data_3,
    output reg  signed [9:0] rd_data_4,
    output reg  signed [9:0] rd_data_5,
    output reg  signed [9:0] rd_data_6,
    output reg  signed [9:0] rd_data_7
);

    localparam SM_W       = 10;
    localparam WIN_LEN_R4 = 15;

    // Register array: 15 entries × 8 states × 10 bits = 1200 bits
    reg signed [SM_W-1:0] mem_s0 [0:WIN_LEN_R4-1];
    reg signed [SM_W-1:0] mem_s1 [0:WIN_LEN_R4-1];
    reg signed [SM_W-1:0] mem_s2 [0:WIN_LEN_R4-1];
    reg signed [SM_W-1:0] mem_s3 [0:WIN_LEN_R4-1];
    reg signed [SM_W-1:0] mem_s4 [0:WIN_LEN_R4-1];
    reg signed [SM_W-1:0] mem_s5 [0:WIN_LEN_R4-1];
    reg signed [SM_W-1:0] mem_s6 [0:WIN_LEN_R4-1];
    reg signed [SM_W-1:0] mem_s7 [0:WIN_LEN_R4-1];

    always @(posedge clk) begin
        if (wr_en) begin
            mem_s0[wr_addr] <= wr_data_0;
            mem_s1[wr_addr] <= wr_data_1;
            mem_s2[wr_addr] <= wr_data_2;
            mem_s3[wr_addr] <= wr_data_3;
            mem_s4[wr_addr] <= wr_data_4;
            mem_s5[wr_addr] <= wr_data_5;
            mem_s6[wr_addr] <= wr_data_6;
            mem_s7[wr_addr] <= wr_data_7;
        end
        // Synchronous read (1-cycle latency)
        rd_data_0 <= mem_s0[rd_addr];
        rd_data_1 <= mem_s1[rd_addr];
        rd_data_2 <= mem_s2[rd_addr];
        rd_data_3 <= mem_s3[rd_addr];
        rd_data_4 <= mem_s4[rd_addr];
        rd_data_5 <= mem_s5[rd_addr];
        rd_data_6 <= mem_s6[rd_addr];
        rd_data_7 <= mem_s7[rd_addr];
    end

endmodule
