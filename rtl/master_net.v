//==============================================================================
// Module: master_net
// Description: Combinatorial sorter for the parallel turbo decoder's
//              contention-free interleaver network.
//              Given SISO-0's interleaved address pi(k) (13-bit),
//              derives SISO-1's address, sorts the pair, and outputs
//              the BRAM row address and permutation bit.
//
//              Instantiated 6 times:
//                master_fr_even, master_fr_odd,
//                master_br_even, master_br_odd,
//                master_dbr_even, master_dbr_odd
//
// SISO-1 derivation: pi(k+SEG_LEN) == pi(k) +/- SEG_LEN (mod K)
//   - If pi(k) < SEG_LEN: pi(k+SEG_LEN) = pi(k) + SEG_LEN
//   - If pi(k) >= SEG_LEN: pi(k+SEG_LEN) = pi(k) - SEG_LEN
//   XOR is PROHIBITED (fails for upper-segment addresses).
//
// perm_bit semantics for the current N=2 folded memory:
//   0: SISO-0's pi(k) in the lower segment, data is in col[0]
//   1: SISO-0's pi(k) in the upper segment, data is in col[1]
//==============================================================================
module master_net #(
    parameter K          = 3200,
    parameter SEG_LEN    = 1600,
    parameter PI_W       = 12,
    parameter ROW_ADDR_W = 10
) (
    input  wire [PI_W-1:0]       addr_siso0,  // pi(k) from LUT
    output wire [PI_W-1:0]       pi_siso1,    // pi(k+SEG_LEN)
    output wire [ROW_ADDR_W-1:0] bram_row,    // row in folded extrinsic BRAM
    output wire                  perm_bit     // 0: SISO-0 in col[0]; 1: SISO-0 in col[1]
);

    // Derive SISO-1 address using ternary (NEVER XOR)
    assign pi_siso1 = (addr_siso0 < SEG_LEN) ? (addr_siso0 + SEG_LEN)
                                              : (addr_siso0 - SEG_LEN);

    // Permutation bit: 1 if SISO-0's address is in the upper segment.
    assign perm_bit = (addr_siso0 >= SEG_LEN);

    // BRAM row = sorted_lo / 2 where sorted_lo = min(addr_siso0, pi_siso1)
    assign bram_row = (perm_bit ? pi_siso1 : addr_siso0) >> 1;

endmodule
