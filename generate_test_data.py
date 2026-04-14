import numpy as np
import os

# ==============================================================================
# BITS_LTE_Parallel_Turbo_Decoder Test Data Generator
# Generates input LLRs and corresponding Golden Output LLRs
# ==============================================================================

def generate_random_llrs(frame_len=768):
    """
    Generates realistic 5-bit signed LLR values for a channel simulation.
    Range: [-16, 15] 2's complement. We use [-7, +7] for realistic channel swings.
    """
    sys = np.random.randint(-7, 8, frame_len)
    par = np.random.randint(-7, 8, frame_len)
    apr = np.random.randint(-2, 3, frame_len) # Apr is usually small initially
    return sys, par, apr

def run_golden_siso_model(sys, par, apr):
    """
    Algorithmic Reference Model for Max-Log-MAP SISO Decoder.
    For bit-true RTL comparison, this needs to incorporate the sliding window,
    modulo-normalization, and extrinsic shift-add scaling.
    
    Here we provide an algorithmic placeholder that computes the correct BPSK 
    preprocessed branch metrics and simulates the trellis logic structurally.
    """
    frame_len = len(sys)
    extr_out = np.zeros(frame_len, dtype=int)
    
    # --------------------------------------------------------------------------
    # Trellis Predecessor Table (from Section 3.3)
    # dest -> [(pred_state, input_u, par_x), ...]
    # --------------------------------------------------------------------------
    r2_preds = {
        0: [(0, 0, 0), (1, 1, 1)],
        1: [(2, 0, 1), (3, 1, 0)],
        2: [(4, 0, 0), (5, 1, 1)],
        3: [(6, 1, 0), (7, 0, 1)],
        4: [(0, 1, 1), (1, 0, 0)],
        5: [(2, 1, 0), (3, 0, 1)],
        6: [(4, 1, 1), (5, 0, 0)],
        7: [(6, 0, 1), (7, 1, 0)]
    }
    
    # Floating point Max-Log-MAP for simplicity across the whole frame
    # Note: RTL uses sliding window and dummy backward, which causes small deviations
    # from this idealized full-frame Max-Log-MAP at window boundaries.
    
    # 1. Branch Metrics
    # gammas: [trellis_time][pred_state][dest_state]
    gammas = np.full((frame_len, 8, 8), -np.inf)
    
    for k in range(frame_len):
        L_s = sys[k]
        L_p = par[k]
        L_a = apr[k]
        
        # BPSK values: (u=0 -> +1, u=1 -> -1), same for parity
        # Gamma = u_bpsk * (L_s + L_a) + p_bpsk * L_p
        # Corresponding to PRE[0] .. PRE[3] in RTL
        G = {
            (0, 0):  (L_s + L_a) + L_p,  # u=0, p=0
            (0, 1):  (L_s + L_a) - L_p,  # u=0, p=1
            (1, 0): -(L_s + L_a) + L_p,  # u=1, p=0
            (1, 1): -(L_s + L_a) - L_p   # u=1, p=1
        }
        
        for dest, preds in r2_preds.items():
            for (pred, u, p) in preds:
                gammas[k][pred][dest] = G[(u, p)]

    # 2. Forward Recursion (Alpha)
    alphas = np.full((frame_len + 1, 8), -np.inf)
    alphas[0][0] = 0 # RTL CORE_ID=0 assumption
    
    for k in range(frame_len):
        for dest in range(8):
            cands = []
            for pred in range(8):
                if gammas[k][pred][dest] != -np.inf:
                    cands.append(alphas[k][pred] + gammas[k][pred][dest])
            alphas[k+1][dest] = max(cands) if cands else -np.inf

    # 3. Backward Recursion (Beta)
    betas = np.full((frame_len + 1, 8), -np.inf)
    betas[frame_len][:] = 0 # Assume equal probability terminal or dummy window
    
    for k in range(frame_len - 1, -1, -1):
        for pred in range(8):
            cands = []
            for dest in range(8):
                if gammas[k][pred][dest] != -np.inf:
                    cands.append(betas[k+1][dest] + gammas[k][pred][dest])
            betas[k][pred] = max(cands) if cands else -np.inf

    # 4. LLR Computation
    for k in range(frame_len):
        llr_0_cands = []
        llr_1_cands = []
        
        for dest, preds in r2_preds.items():
            for (pred, u, p) in preds:
                val = alphas[k][pred] + gammas[k][pred][dest] + betas[k+1][dest]
                if u == 0:
                    llr_0_cands.append(val)
                else:
                    llr_1_cands.append(val)
                    
        L_D = max(llr_0_cands) - max(llr_1_cands)
        L_E_raw = L_D - (sys[k] + apr[k])
        
        # Extrinsic scaling trick (RTL equivalent: L_E - L_E>>>3 - L_E>>>4) ~ 0.6875
        L_E_scaled = L_E_raw - (int(L_E_raw) >> 3) - (int(L_E_raw) >> 4)
        
        # 6-bit signed saturation
        extr_out[k] = np.clip(L_E_scaled, -32, 31)
        
    return extr_out

def write_hex_files(sys, par, apr, extr_golden):
    frame_len = len(sys)
    
    with open('input_llr.hex', 'w') as fin:
        for i in range(frame_len):
            # Format: {apr(5), par(5), sys(5)} => 15 bits
            s = int(sys[i]) & 0x1F
            p = int(par[i]) & 0x1F
            a = int(apr[i]) & 0x1F
            val = (a << 10) | (p << 5) | s
            fin.write(f"{val:04X}\n")

    # Generate human-readable decimal table
    with open('input_llr_readable.txt', 'w') as f_read:
        f_read.write(f"{'ADDR':>4} | {'SYS':>4} | {'PAR':>4} | {'APR':>4}\n")
        f_read.write("-" * 27 + "\n")
        for i in range(frame_len):
            f_read.write(f"{i:03X}  | {int(sys[i]):>4} | {int(par[i]):>4} | {int(apr[i]):>4}\n")

    with open('ref_extrinsic.hex', 'w') as fout:
        # We output in pairs (odd, even) just like the RTL will output
        for i in range(0, frame_len, 2):
            if i+1 < frame_len:
                e_odd = int(extr_golden[i]) & 0x3F
                e_even = int(extr_golden[i+1]) & 0x3F
            else:
                e_odd = int(extr_golden[i]) & 0x3F
                e_even = 0
            
            # Match the format of TB output: "ADDR: EXTR_ODD EXTR_EVEN"
            fout.write(f"{i:03x}: {e_odd:02x} {e_even:02x}\n")

if __name__ == "__main__":
    np.random.seed(42) # Deterministic for repeatability
    FRAME_LEN = 780  # 26 full windows * 30 for CORE_ID=0
    
    print(f"Generating Random Test Vectors for FRAME_LEN = {FRAME_LEN}...")
    s, p, a = generate_random_llrs(FRAME_LEN)
    
    print("Running Golden Python Max-Log-MAP Model...")
    extr_golden = run_golden_siso_model(s, p, a)
    
    print("Writing Test Vectors to 'input_llr.hex' and 'ref_extrinsic.hex'...")
    write_hex_files(s, p, a, extr_golden)
    
    print("Done. You can now run the Verilog testbench.")
