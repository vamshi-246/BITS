//==============================================================================
// Module: master_net
// Description: Combinatorial sorter for the parallel turbo decoder's
//              contention-free interleaver network.
//              Given SISO-0's interleaved address π(k) (13-bit),
//              derives SISO-1's address, sorts the pair, and outputs
//              the BRAM row address and permutation bit.
//
//              Instantiated 6 times:
//                master_fr_even, master_fr_odd,
//                master_br_even, master_br_odd,
//                master_dbr_even, master_dbr_odd
//
// SISO-1 derivation: π(k+3072) ≡ π(k) ± 3072 (mod 6144)
//   - If π(k) < 3072: π(k+3072) = π(k) + 3072
//   - If π(k) >= 3072: π(k+3072) = π(k) - 3072
//   XOR is PROHIBITED (fails for π(k) >= 3072).
//
// perm_bit semantics:
//   0 → SISO-0's π(k) ∈ [0..3071], SISO-0 data is in col[0]
//   1 → SISO-0's π(k) ∈ [3072..6143], SISO-0 data is in col[1]
//==============================================================================
module master_net (
    input  wire [12:0] addr_siso0,  // π(k) from LUT — 13-bit [0..6143]
    output wire [12:0] pi_siso1,    // π(k+3072) — 13-bit [0..6143]
    output wire [10:0] bram_row,    // BRAM row for extrinsic BRAM — 11-bit [0..1535]
    output wire        perm_bit     // 0: SISO-0 in col[0]; 1: SISO-0 in col[1]
);

    // Derive SISO-1 address using ternary (NEVER XOR)
    assign pi_siso1 = (addr_siso0 < 13'd3072) ? (addr_siso0 + 13'd3072)
                                               : (addr_siso0 - 13'd3072);

    // Permutation bit: 1 if SISO-0's address is the larger one (>= 3072)
    assign perm_bit = (addr_siso0 >= 13'd3072);

    // BRAM row = sorted_lo[11:1] where sorted_lo = min(addr_siso0, pi_siso1)
    // If perm_bit=0: addr_siso0 < 3072, sorted_lo = addr_siso0
    // If perm_bit=1: pi_siso1 < 3072, sorted_lo = pi_siso1
    assign bram_row = perm_bit ? pi_siso1[11:1] : addr_siso0[11:1];

endmodule
