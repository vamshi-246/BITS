//==============================================================================
// Module: qpp_lut
// Description: QPP interleaver LUT ROM for the LTE Turbo Decoder.
//              Default build has 1600 entries:
//              pi(k) = (111k + 240k^2) mod 3200.
//              6 simultaneous read ports provided via 3 identical ROM copies,
//              each as a true dual-port BRAM (2 ports per copy).
//
//              Copy 0, ports {A,B}: reads at fr_addr and fr_addr+1
//              Copy 1, ports {A,B}: reads at br_addr and br_addr+1
//              Copy 2, ports {A,B}: reads at dbr_addr and dbr_addr+1
//
//              During backward pass, FR ports (copy 0) are reused for
//              write-back address generation.
//
//              Initialised from QPP_LUT_FILE via $readmemh.
//              Output width defaults to 12 bits (pi(k) in [0..3199]).
//              Input width defaults to 11 bits (k in [0..1599]).
//==============================================================================
`ifndef TURBO_DATA_DIR
`define TURBO_DATA_DIR "data/"
`endif

module qpp_lut #(
    parameter ADDR_W       = 11,
    parameter PI_W         = 12,
    parameter DEPTH        = 1600,
    parameter QPP_LUT_FILE = {`TURBO_DATA_DIR, "qpp_3200.hex"}
) (
    input  wire        clk,

    // 6 read address inputs (LUT index k, 12-bit, range [0..3071])
    input  wire [ADDR_W-1:0] addr_fr_even,   // = fr_addr
    input  wire [ADDR_W-1:0] addr_fr_odd,    // = fr_addr + 1
    input  wire [ADDR_W-1:0] addr_br_even,   // = br_addr
    input  wire [ADDR_W-1:0] addr_br_odd,    // = br_addr + 1
    input  wire [ADDR_W-1:0] addr_dbr_even,  // = dbr_addr
    input  wire [ADDR_W-1:0] addr_dbr_odd,   // = dbr_addr + 1

    // 6 outputs
    output reg  [PI_W-1:0] pi_fr_even,
    output reg  [PI_W-1:0] pi_fr_odd,
    output reg  [PI_W-1:0] pi_br_even,
    output reg  [PI_W-1:0] pi_br_odd,
    output reg  [PI_W-1:0] pi_dbr_even,
    output reg  [PI_W-1:0] pi_dbr_odd
);

    // =========================================================================
    // Three identical ROM copies — initialised from the same hex file
    // =========================================================================
    (* ram_style = "block" *) reg [PI_W-1:0] lut_copy0 [0:DEPTH-1];
    (* ram_style = "block" *) reg [PI_W-1:0] lut_copy1 [0:DEPTH-1];
    (* ram_style = "block" *) reg [PI_W-1:0] lut_copy2 [0:DEPTH-1];

    initial begin
        $readmemh(QPP_LUT_FILE, lut_copy0);
        $readmemh(QPP_LUT_FILE, lut_copy1);
        $readmemh(QPP_LUT_FILE, lut_copy2);
    end

    // Copy 0 — FR even (port A) and FR odd (port B)
    always @(posedge clk) begin
        pi_fr_even <= lut_copy0[addr_fr_even];
    end
    always @(posedge clk) begin
        pi_fr_odd  <= lut_copy0[addr_fr_odd];
    end

    // Copy 1 — BR even (port A) and BR odd (port B)
    always @(posedge clk) begin
        pi_br_even <= lut_copy1[addr_br_even];
    end
    always @(posedge clk) begin
        pi_br_odd  <= lut_copy1[addr_br_odd];
    end

    // Copy 2 — DBR even (port A) and DBR odd (port B)
    always @(posedge clk) begin
        pi_dbr_even <= lut_copy2[addr_dbr_even];
    end
    always @(posedge clk) begin
        pi_dbr_odd  <= lut_copy2[addr_dbr_odd];
    end

endmodule
