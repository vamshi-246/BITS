//==============================================================================
// Module: slave_net
// Description: Combinatorial switch for the parallel turbo decoder's
//              contention-free interleaver network.
//              Routes extrinsic BRAM column data to the correct SISO core
//              based on the permutation bit from master_net.
//
//              Instantiated 6 times (one-to-one with master_net):
//                slave_fr_even, slave_fr_odd,
//                slave_br_even, slave_br_odd,
//                slave_dbr_even, slave_dbr_odd
//
// perm_bit=0: SISO-0 data is in col[0] → out_siso0 = col0, out_siso1 = col1
// perm_bit=1: SISO-0 data is in col[1] → out_siso0 = col1, out_siso1 = col0
//==============================================================================
module slave_net (
    input  wire        perm_bit,
    input  wire signed [5:0] col0,      // extrinsic BRAM col[0] value
    input  wire signed [5:0] col1,      // extrinsic BRAM col[1] value
    output wire signed [5:0] out_siso0, // a-priori for core 0
    output wire signed [5:0] out_siso1  // a-priori for core 1
);

    assign out_siso0 = perm_bit ? col1 : col0;
    assign out_siso1 = perm_bit ? col0 : col1;

endmodule
