//==============================================================================
// Module: gamma_mem
// Description: Register-array based gamma (branch metric) memory bank.
//              Synchronous write, synchronous read (1-cycle read latency).
//              Stores 32 radix-2 BMs per address (16 odd + 16 even).
//              Depth: WIN_LEN_R4 = 15 entries.
//              Width: 32 × BM_R2_W = 32 × 7 = 224 bits per entry.
// Convention: Indices 0..15 = odd BMs, indices 16..31 = even BMs.
// Reference: Studer et al., IEEE JSSC 2011, Section 14
//==============================================================================
module gamma_mem (
    input  wire         clk,
    input  wire         wr_en,
    input  wire [3:0]   wr_addr,                    // 0..14
    // Write data: 32 BMs (flattened)
    input  wire signed [6:0] wr_data_0,  wr_data_1,  wr_data_2,  wr_data_3,
    input  wire signed [6:0] wr_data_4,  wr_data_5,  wr_data_6,  wr_data_7,
    input  wire signed [6:0] wr_data_8,  wr_data_9,  wr_data_10, wr_data_11,
    input  wire signed [6:0] wr_data_12, wr_data_13, wr_data_14, wr_data_15,
    input  wire signed [6:0] wr_data_16, wr_data_17, wr_data_18, wr_data_19,
    input  wire signed [6:0] wr_data_20, wr_data_21, wr_data_22, wr_data_23,
    input  wire signed [6:0] wr_data_24, wr_data_25, wr_data_26, wr_data_27,
    input  wire signed [6:0] wr_data_28, wr_data_29, wr_data_30, wr_data_31,
    input  wire [3:0]   rd_addr,
    // Read data: 32 BMs (flattened), 1-cycle latency
    output reg  signed [6:0] rd_data_0,  rd_data_1,  rd_data_2,  rd_data_3,
    output reg  signed [6:0] rd_data_4,  rd_data_5,  rd_data_6,  rd_data_7,
    output reg  signed [6:0] rd_data_8,  rd_data_9,  rd_data_10, rd_data_11,
    output reg  signed [6:0] rd_data_12, rd_data_13, rd_data_14, rd_data_15,
    output reg  signed [6:0] rd_data_16, rd_data_17, rd_data_18, rd_data_19,
    output reg  signed [6:0] rd_data_20, rd_data_21, rd_data_22, rd_data_23,
    output reg  signed [6:0] rd_data_24, rd_data_25, rd_data_26, rd_data_27,
    output reg  signed [6:0] rd_data_28, rd_data_29, rd_data_30, rd_data_31
);

    localparam BM_R2_W    = 7;
    localparam WIN_LEN_R4 = 15;
    localparam NUM_BMS    = 32;

    // Register array: 15 entries × 32 BMs × 7 bits = 3360 bits
    // Split into 32 separate arrays for Verilog-2001 compatibility
    reg signed [BM_R2_W-1:0] mem_0  [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_1  [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_2  [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_3  [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_4  [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_5  [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_6  [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_7  [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_8  [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_9  [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_10 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_11 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_12 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_13 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_14 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_15 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_16 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_17 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_18 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_19 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_20 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_21 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_22 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_23 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_24 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_25 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_26 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_27 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_28 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_29 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_30 [0:WIN_LEN_R4-1];
    reg signed [BM_R2_W-1:0] mem_31 [0:WIN_LEN_R4-1];

    always @(posedge clk) begin
        if (wr_en) begin
            mem_0[wr_addr]  <= wr_data_0;  mem_1[wr_addr]  <= wr_data_1;
            mem_2[wr_addr]  <= wr_data_2;  mem_3[wr_addr]  <= wr_data_3;
            mem_4[wr_addr]  <= wr_data_4;  mem_5[wr_addr]  <= wr_data_5;
            mem_6[wr_addr]  <= wr_data_6;  mem_7[wr_addr]  <= wr_data_7;
            mem_8[wr_addr]  <= wr_data_8;  mem_9[wr_addr]  <= wr_data_9;
            mem_10[wr_addr] <= wr_data_10; mem_11[wr_addr] <= wr_data_11;
            mem_12[wr_addr] <= wr_data_12; mem_13[wr_addr] <= wr_data_13;
            mem_14[wr_addr] <= wr_data_14; mem_15[wr_addr] <= wr_data_15;
            mem_16[wr_addr] <= wr_data_16; mem_17[wr_addr] <= wr_data_17;
            mem_18[wr_addr] <= wr_data_18; mem_19[wr_addr] <= wr_data_19;
            mem_20[wr_addr] <= wr_data_20; mem_21[wr_addr] <= wr_data_21;
            mem_22[wr_addr] <= wr_data_22; mem_23[wr_addr] <= wr_data_23;
            mem_24[wr_addr] <= wr_data_24; mem_25[wr_addr] <= wr_data_25;
            mem_26[wr_addr] <= wr_data_26; mem_27[wr_addr] <= wr_data_27;
            mem_28[wr_addr] <= wr_data_28; mem_29[wr_addr] <= wr_data_29;
            mem_30[wr_addr] <= wr_data_30; mem_31[wr_addr] <= wr_data_31;
        end
        // Synchronous read (1-cycle latency)
        rd_data_0  <= mem_0[rd_addr];  rd_data_1  <= mem_1[rd_addr];
        rd_data_2  <= mem_2[rd_addr];  rd_data_3  <= mem_3[rd_addr];
        rd_data_4  <= mem_4[rd_addr];  rd_data_5  <= mem_5[rd_addr];
        rd_data_6  <= mem_6[rd_addr];  rd_data_7  <= mem_7[rd_addr];
        rd_data_8  <= mem_8[rd_addr];  rd_data_9  <= mem_9[rd_addr];
        rd_data_10 <= mem_10[rd_addr]; rd_data_11 <= mem_11[rd_addr];
        rd_data_12 <= mem_12[rd_addr]; rd_data_13 <= mem_13[rd_addr];
        rd_data_14 <= mem_14[rd_addr]; rd_data_15 <= mem_15[rd_addr];
        rd_data_16 <= mem_16[rd_addr]; rd_data_17 <= mem_17[rd_addr];
        rd_data_18 <= mem_18[rd_addr]; rd_data_19 <= mem_19[rd_addr];
        rd_data_20 <= mem_20[rd_addr]; rd_data_21 <= mem_21[rd_addr];
        rd_data_22 <= mem_22[rd_addr]; rd_data_23 <= mem_23[rd_addr];
        rd_data_24 <= mem_24[rd_addr]; rd_data_25 <= mem_25[rd_addr];
        rd_data_26 <= mem_26[rd_addr]; rd_data_27 <= mem_27[rd_addr];
        rd_data_28 <= mem_28[rd_addr]; rd_data_29 <= mem_29[rd_addr];
        rd_data_30 <= mem_30[rd_addr]; rd_data_31 <= mem_31[rd_addr];
    end

endmodule
