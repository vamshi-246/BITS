//==============================================================================
// Module: extrinsic_bram
// Description: Group-B BRAM for the LTE Turbo Decoder.
//              Stores extrinsic / a-priori LLRs in folded memory format:
//                1536 rows × 12 bits per row.
//              Provides 3 simultaneous read ports (FR, BR, DBR) and
//              1 write port (write-back during decoding).
//
//              Implemented as 3 simple dual-port BRAM copies.
//              Word format: {col1[5:0], col0[5:0]} = 12-bit
//              col[0] = SISO-0 (core0), col[1] = SISO-1 (core1)
//==============================================================================
module extrinsic_bram #(
    parameter WORD_W = 12,       // {col1[5:0], col0[5:0]}
    parameter DEPTH  = 1536,
    parameter ADDR_W = 11        // ceil(log2(1536)) = 11
) (
    input  wire              clk,

    // Write port (write-back during decoding, or copy to ld_ram)
    input  wire              wr_en,
    input  wire [ADDR_W-1:0] wr_addr,
    input  wire [WORD_W-1:0] wr_data,   // {col1[5:0], col0[5:0]}

    // Read port 0 — FR
    input  wire [ADDR_W-1:0] rd_addr_0,
    output reg  [WORD_W-1:0] rd_data_0,

    // Read port 1 — BR
    input  wire [ADDR_W-1:0] rd_addr_1,
    output reg  [WORD_W-1:0] rd_data_1,

    // Read port 2 — DBR
    input  wire [ADDR_W-1:0] rd_addr_2,
    output reg  [WORD_W-1:0] rd_data_2
);

    // =========================================================================
    // Three SDP BRAM copies — all hold identical data
    // =========================================================================
    (* ram_style = "block" *) reg [WORD_W-1:0] mem0 [0:DEPTH-1];
    (* ram_style = "block" *) reg [WORD_W-1:0] mem1 [0:DEPTH-1];
    (* ram_style = "block" *) reg [WORD_W-1:0] mem2 [0:DEPTH-1];

    // Copy 0 — write + FR read
    always @(posedge clk) begin
        if (wr_en)
            mem0[wr_addr] <= wr_data;
    end
    always @(posedge clk) begin
        rd_data_0 <= mem0[rd_addr_0];
    end

    // Copy 1 — write (mirror) + BR read
    always @(posedge clk) begin
        if (wr_en)
            mem1[wr_addr] <= wr_data;
    end
    always @(posedge clk) begin
        rd_data_1 <= mem1[rd_addr_1];
    end

    // Copy 2 — write (mirror) + DBR read
    always @(posedge clk) begin
        if (wr_en)
            mem2[wr_addr] <= wr_data;
    end
    always @(posedge clk) begin
        rd_data_2 <= mem2[rd_addr_2];
    end

endmodule
