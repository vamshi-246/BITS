//==============================================================================
// Module: qpp_lut
// Description: QPP interleaver LUT ROM for the LTE Turbo Decoder.
//              3072 entries, each 13-bit: π(k) = (263k + 480k²) mod 6144.
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
//              Initialised from qpp_6144.hex via $readmemh.
//              Output width: 13 bits (π(k) ∈ [0..6143]).
//              Input width: 12 bits (k ∈ [0..3071]).
//==============================================================================
module qpp_lut (
    input  wire        clk,

    // 6 read address inputs (LUT index k, 12-bit, range [0..3071])
    input  wire [11:0] addr_fr_even,   // = fr_addr
    input  wire [11:0] addr_fr_odd,    // = fr_addr + 1
    input  wire [11:0] addr_br_even,   // = br_addr
    input  wire [11:0] addr_br_odd,    // = br_addr + 1
    input  wire [11:0] addr_dbr_even,  // = dbr_addr
    input  wire [11:0] addr_dbr_odd,   // = dbr_addr + 1

    // 6 outputs — 13-bit (π(k) ∈ [0..6143])
    output reg  [12:0] pi_fr_even,
    output reg  [12:0] pi_fr_odd,
    output reg  [12:0] pi_br_even,
    output reg  [12:0] pi_br_odd,
    output reg  [12:0] pi_dbr_even,
    output reg  [12:0] pi_dbr_odd
);

    // =========================================================================
    // Three identical ROM copies — initialised from the same hex file
    // =========================================================================
    (* ram_style = "block" *) reg [12:0] lut_copy0 [0:3071];
    (* ram_style = "block" *) reg [12:0] lut_copy1 [0:3071];
    (* ram_style = "block" *) reg [12:0] lut_copy2 [0:3071];

    initial begin
        $readmemh("../data/qpp_6144.hex", lut_copy0);
        $readmemh("../data/qpp_6144.hex", lut_copy1);
        $readmemh("../data/qpp_6144.hex", lut_copy2);
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
