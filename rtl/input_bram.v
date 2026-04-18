//==============================================================================
// Module: input_bram
// Description: Parameterised Group-A BRAM for the LTE Turbo Decoder.
//              Stores channel LLRs in folded memory format:
//                1536 rows × WORD_W bits per row.
//              Provides 3 simultaneous read ports (FR, BR, DBR) and
//              1 write port (host load phase).
//
//              Implemented as 3 simple dual-port BRAM copies:
//                Copy 0: write + FR read
//                Copy 1: write (mirror) + BR read
//                Copy 2: write (mirror) + DBR read
//              All copies always hold identical data.
//
//              Word format: {col1[4:0], col0[4:0]} = 10-bit
//              col[0] = SISO-0 (core0), col[1] = SISO-1 (core1)
//
// BRAM inference: (* ram_style = "block" *), synchronous reads,
//                 no async/transparent reads.
//==============================================================================
module input_bram #(
    parameter WORD_W = 10,       // {col1[4:0], col0[4:0]}
    parameter DEPTH  = 1536,
    parameter ADDR_W = 11        // ceil(log2(1536)) = 11
) (
    input  wire              clk,

    // Write port (load phase — all 3 copies written simultaneously)
    input  wire              wr_en,
    input  wire [ADDR_W-1:0] wr_addr,   // row [0..1535]
    input  wire [WORD_W-1:0] wr_data,   // {col1, col0}

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

    // Copy 0 — write port
    always @(posedge clk) begin
        if (wr_en)
            mem0[wr_addr] <= wr_data;
    end
    // Copy 0 — FR read port (synchronous, 1-cycle latency)
    always @(posedge clk) begin
        rd_data_0 <= mem0[rd_addr_0];
    end

    // Copy 1 — write port (mirror)
    always @(posedge clk) begin
        if (wr_en)
            mem1[wr_addr] <= wr_data;
    end
    // Copy 1 — BR read port
    always @(posedge clk) begin
        rd_data_1 <= mem1[rd_addr_1];
    end

    // Copy 2 — write port (mirror)
    always @(posedge clk) begin
        if (wr_en)
            mem2[wr_addr] <= wr_data;
    end
    // Copy 2 — DBR read port
    always @(posedge clk) begin
        rd_data_2 <= mem2[rd_addr_2];
    end

endmodule
