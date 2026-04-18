//==============================================================================
// Module: ld_bram
// Description: Group-C output BRAM for the LTE Turbo Decoder.
//              Single-port 1536×12-bit BRAM storing the final extrinsic
//              LLR values after the last half-iteration.
//              Written by the controller in ST_WRITE_LD.
//              Read by the host after decode_done.
//              Word format: {col1[5:0], col0[5:0]} = 12-bit
//==============================================================================
module ld_bram #(
    parameter WORD_W = 12,
    parameter DEPTH  = 1536,
    parameter ADDR_W = 11
) (
    input  wire              clk,

    // Write port (controller writes during ST_WRITE_LD)
    input  wire              wr_en,
    input  wire [ADDR_W-1:0] wr_addr,
    input  wire [WORD_W-1:0] wr_data,

    // Read port (host reads after decode_done)
    input  wire [ADDR_W-1:0] rd_addr,
    output reg  [WORD_W-1:0] rd_data
);

    (* ram_style = "block" *) reg [WORD_W-1:0] mem [0:DEPTH-1];

    // Port A: write
    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    // Port B: read (synchronous, 1-cycle latency)
    always @(posedge clk) begin
        rd_data <= mem[rd_addr];
    end

endmodule
