import numpy as np
import sys, io

# =============================================================================
# Constants -- must match RTL (bcjr_core.v, acs_r4.v)
# =============================================================================
SM_W       = 10
SM_HALF    = 1 << (SM_W - 1)   # 512
SM_FULL    = 1 << SM_W          # 1024
BM_R2_W    = 7
BM_R4_W    = 8
NEG_INF    = -256               # RTL: -10'sd256
WIN_LEN    = 30                 # Trellis steps per window
WIN_LEN_R4 = 15                 # Radix-4 steps per window
NUM_WINDOWS = 103               # ceil(3072 / 30)
CORE_ID     = 0
NUM_SISO    = 2

# =============================================================================
# Data loading
# =============================================================================
def load_hex_ram(filepath):
    vals = np.loadtxt(filepath, dtype=str)
    ints = np.array([int(v, 16) for v in vals], dtype=np.int32)
    d1 = ints & 0x1F; d2 = (ints >> 5) & 0x1F
    d1 = np.where(d1 & 0x10, d1 - 32, d1)
    d2 = np.where(d2 & 0x10, d2 - 32, d2)
    return np.column_stack((d1, d2))

sys_odd_ram   = load_hex_ram("data/sys_odd_ram.hex")
sys_even_ram  = load_hex_ram("data/sys_even_ram.hex")
par1_odd_ram  = load_hex_ram("data/par1_odd_ram.hex")
par1_even_ram = load_hex_ram("data/par1_even_ram.hex")
silv_odd_ram  = load_hex_ram("data/sys_ilv_odd_ram.hex")
silv_even_ram = load_hex_ram("data/sys_ilv_even_ram.hex")
par2_odd_ram  = load_hex_ram("data/par2_odd_ram.hex")
par2_even_ram = load_hex_ram("data/par2_even_ram.hex")

# =============================================================================
# Branch Metric helpers
# =============================================================================
def _trunc(val, width):
    half = 1 << (width - 1); full = 1 << width
    return ((val + half) % full) - half

def prep_data(o4, ls, lp, la):
    if o4 == 0: return  ls + la + lp
    if o4 == 1: return  ls + la - lp
    if o4 == 2: return -ls - la + lp
    return -ls - la - lp

def compute_radix_2(lse, lso, lpe, lpo, lae, lao):
    """Returns shape (2, 16): row 0 = odd (first step), row 1 = even (second step)."""
    lse, lso, lpe, lpo = int(lse), int(lso), int(lpe), int(lpo)
    lae, lao = int(lae), int(lao)
    pre_e = [_trunc(prep_data(i, lse, lpe, lae), BM_R2_W) for i in range(4)]
    pre_o = [_trunc(prep_data(i, lso, lpo, lao), BM_R2_W) for i in range(4)]
    idx = [0, 3, 2, 1, 1, 2, 3, 0, 3, 0, 1, 2, 2, 1, 0, 3]
    bm_odd  = np.array([pre_o[i] for i in idx], dtype=np.int32)
    bm_even = np.array([pre_e[i] for i in idx], dtype=np.int32)
    return np.stack([bm_odd, bm_even])

def compute_radix_4(bm_r2):
    """Input (2,16) -> output (32,) radix-4 BMs."""
    first_idx  = [i % 16 for i in range(32)]
    second_idx = [i // 2 for i in range(32)]
    bm_r4 = bm_r2[0][first_idx] + bm_r2[1][second_idx]
    return _trunc(bm_r4, BM_R4_W)

# =============================================================================
# ACS helpers
# =============================================================================
def _acs_max4(cands):
    """Modulo-normalized max of 4 candidates."""
    winner = cands[0]
    for i in range(1, 4):
        diff = _trunc(cands[i] - winner, SM_W)
        if diff > 0: winner = cands[i]
    return winner

def acs_forward(bm_r4, prev_sm):
    """Forward ACS: dest even -> preds {0,1,2,3}, dest odd -> preds {4,5,6,7}."""
    next_sm = np.zeros(8, dtype=np.int32)
    for d in range(8):
        bms = bm_r4[d*4 : d*4+4]
        preds = [0,1,2,3] if d % 2 == 0 else [4,5,6,7]
        cands = [_trunc(int(prev_sm[preds[i]]) + int(bms[i]), SM_W) for i in range(4)]
        next_sm[d] = _acs_max4(cands)
    return next_sm

def acs_backward(bm_r4, prev_sm):
    """Backward (transposed) ACS from dummy_backward_recursion_unit.v:
    s''=0: succ={0,2,4,6}, BMs={r4[0], r4[8], r4[16], r4[24]}
    s''=1: succ={0,2,4,6}, BMs={r4[1], r4[9], r4[17], r4[25]}
    s''=2: succ={0,2,4,6}, BMs={r4[2], r4[10],r4[18], r4[26]}
    s''=3: succ={0,2,4,6}, BMs={r4[3], r4[11],r4[19], r4[27]}
    s''=4: succ={1,3,5,7}, BMs={r4[4], r4[12],r4[20], r4[28]}
    s''=5: succ={1,3,5,7}, BMs={r4[5], r4[13],r4[21], r4[29]}
    s''=6: succ={1,3,5,7}, BMs={r4[6], r4[14],r4[22], r4[30]}
    s''=7: succ={1,3,5,7}, BMs={r4[7], r4[15],r4[23], r4[31]}
    """
    next_sm = np.zeros(8, dtype=np.int32)
    # BM index for s'' = bm_r4[succ*4 + (s'' % 4)]
    for spp in range(8):
        if spp < 4:
            succs = [0, 2, 4, 6]
        else:
            succs = [1, 3, 5, 7]
        bm_indices = [s * 4 + (spp % 4) for s in succs]
        cands = [_trunc(int(prev_sm[succs[i]]) + int(bm_r4[bm_indices[i]]), SM_W)
                 for i in range(4)]
        next_sm[spp] = _acs_max4(cands)
    return next_sm

# =============================================================================
# Read LLRs helper
# =============================================================================
def read_llrs(addr, sys_even, sys_odd, par_even, par_odd):
    if addr < len(sys_even):
        return sys_even[addr, 0], sys_odd[addr, 0], par_even[addr, 0], par_odd[addr, 0]
    return 0, 0, 0, 0

# =============================================================================
# Forward Recursion -- all windows
# =============================================================================
def forward_recursion_all_windows(sys_even, sys_odd, par_even, par_odd,
                                   core_id=0, num_windows=NUM_WINDOWS):
    alpha_mem    = np.zeros((num_windows, WIN_LEN_R4, 8),  dtype=np.int32)
    gamma_r2_mem = np.zeros((num_windows, WIN_LEN_R4, 32), dtype=np.int32)
    gamma_r4_mem = np.zeros((num_windows, WIN_LEN_R4, 32), dtype=np.int32)

    sm = np.array([0] + [NEG_INF]*7, dtype=np.int32) if core_id == 0 \
         else np.zeros(8, dtype=np.int32)

    for w in range(num_windows):
        base = w * WIN_LEN_R4
        for step in range(WIN_LEN_R4):
            lse, lso, lpe, lpo = read_llrs(base + step, sys_even, sys_odd, par_even, par_odd)
            bm_r2 = compute_radix_2(lse, lso, lpe, lpo, 0, 0)
            bm_r4 = compute_radix_4(bm_r2)
            sm = acs_forward(bm_r4, sm)
            alpha_mem[w, step]    = sm
            gamma_r2_mem[w, step] = bm_r2.flatten()
            gamma_r4_mem[w, step] = bm_r4
    return alpha_mem, gamma_r2_mem, gamma_r4_mem

# =============================================================================
# Dummy Backward Recursion -- one window (backward through data)
# =============================================================================
def dbr_one_window(win_data_idx, sys_even, sys_odd, par_even, par_odd):
    """Run DBR for one window. Reads data in REVERSE order.
    win_data_idx: 1-based window index for data (DBR window N' reads data of window N).
    Returns: final_beta (8,), and per-step beta trace (WIN_LEN_R4, 8).
    """
    sm = np.zeros(8, dtype=np.int32)  # DBR always inits to 0
    beta_trace = np.zeros((WIN_LEN_R4, 8), dtype=np.int32)
    bm_r4_trace = np.zeros((WIN_LEN_R4, 32), dtype=np.int32)

    base = (win_data_idx - 1) * WIN_LEN_R4

    for step in range(WIN_LEN_R4):
        # DBR reads in reverse: addr = base + (WIN_LEN_R4-1-step)
        addr = base + (WIN_LEN_R4 - 1 - step)
        lse, lso, lpe, lpo = read_llrs(addr, sys_even, sys_odd, par_even, par_odd)
        bm_r2 = compute_radix_2(lse, lso, lpe, lpo, 0, 0)
        bm_r4 = compute_radix_4(bm_r2)
        sm = acs_backward(bm_r4, sm)
        beta_trace[step] = sm
        bm_r4_trace[step] = bm_r4

    return sm.copy(), beta_trace, bm_r4_trace

# =============================================================================
# Backward Recursion -- one window (uses stored alpha & gamma from FR)
# =============================================================================
def br_one_window(win_data_idx, beta_init, alpha_mem_win, gamma_r2_win,
                  sys_even, sys_odd, par_even, par_odd):
    """Run BR for one window. Reads stored alpha/gamma in REVERSE order.
    win_data_idx: 1-based window index.
    beta_init: 8-element initial beta from DBR (or known terminal).
    alpha_mem_win: (WIN_LEN_R4, 8) stored alphas for this window.
    gamma_r2_win: (WIN_LEN_R4, 32) stored R2 gammas for this window.
    Returns: per-step beta trace (WIN_LEN_R4, 8), reconstructed R4 BMs.

    Note: In RTL, BR reads gamma from memory and re-computes R4 BMs from stored R2 BMs.
    The BR outputs beta_out BEFORE the ACS update (pre-update beta for LLR compute).
    """
    sm = beta_init.copy()
    beta_trace = np.zeros((WIN_LEN_R4, 8), dtype=np.int32)
    bm_r4_trace = np.zeros((WIN_LEN_R4, 32), dtype=np.int32)

    for step in range(WIN_LEN_R4):
        rd_step = WIN_LEN_R4 - 1 - step  # read in reverse

        # Reconstruct R4 BMs from stored R2 gammas (same as BR RTL)
        stored_r2 = gamma_r2_win[rd_step]  # (32,): [odd_0..15, even_0..15]
        bm_r2_for_r4 = stored_r2.reshape(2, 16)  # (2, 16)
        bm_r4 = compute_radix_4(bm_r2_for_r4)

        # Output beta BEFORE ACS update (this is what LLR compute uses)
        beta_trace[step] = sm.copy()
        bm_r4_trace[step] = bm_r4

        # Backward ACS update
        sm = acs_backward(bm_r4, sm)

    return beta_trace, bm_r4_trace

# =============================================================================
# Full SISO core simulation with window scheduling
# =============================================================================
def simulate_siso_core(sys_even, sys_odd, par_even, par_odd,
                       core_id=0, num_windows=NUM_WINDOWS):
    """Simulate one SISO core with the full window schedule from bcjr_core.v.

    Window schedule for CORE_ID=0:
      Slot 0: FR=W1,    BR=idle,   DBR=W2'
      Slot 1: FR=W2,    BR=W1,     DBR=W3'
      ...
      Slot N-1: FR=WN,  BR=W(N-1), DBR=W(N+1)'  [last core: DBR=idle]
      Slot N:   FR=idle, BR=WN,    DBR=idle

    Returns dict with all stored data.
    """
    is_last_core = (core_id == NUM_SISO - 1)

    # --- Phase 1: Forward Recursion (all windows) ---
    alpha_mem, gamma_r2_mem, gamma_r4_mem = forward_recursion_all_windows(
        sys_even, sys_odd, par_even, par_odd, core_id, num_windows)

    # --- Phase 2: DBR for all required windows ---
    # For CORE_ID=0: DBR runs on windows 2..NUM_WINDOWS+1 (data indices)
    # But last core skips the very last DBR (known terminal state)
    dbr_results = {}  # dbr_results[data_win_idx] = (final_beta, beta_trace, bm_r4_trace)

    for slot in range(num_windows + 1):
        if core_id == 0:
            dbr_data_win = slot + 2  # Slot 0 -> DBR=W2', Slot 1 -> DBR=W3'
        else:
            if slot == 0:
                continue  # No DBR on first slot for non-zero cores
            dbr_data_win = slot + 2

        # Skip if beyond range or last core's terminal window
        if is_last_core and dbr_data_win > num_windows:
            continue
        if dbr_data_win > num_windows + 1:
            continue

        final_beta, bt, r4t = dbr_one_window(dbr_data_win, sys_even, sys_odd, par_even, par_odd)
        dbr_results[dbr_data_win] = (final_beta, bt, r4t)

    # --- Phase 3: Backward Recursion (all windows) ---
    br_results = {}  # br_results[win_idx] = (beta_trace, bm_r4_trace, beta_init_used)

    for w in range(1, num_windows + 1):
        # Determine beta_init for this BR window
        if is_last_core and w == num_windows:
            # Known terminal state
            beta_init = np.array([0] + [NEG_INF]*7, dtype=np.int32)
        else:
            dbr_key = w + 1  # DBR window (w+1)' provides beta_init for BR window w
            if dbr_key in dbr_results:
                beta_init = dbr_results[dbr_key][0]
            else:
                beta_init = np.zeros(8, dtype=np.int32)

        # BR uses stored alpha/gamma from window w (0-indexed: w-1)
        alpha_win = alpha_mem[w - 1]
        gamma_r2_win = gamma_r2_mem[w - 1]

        beta_trace, bm_r4_trace = br_one_window(
            w, beta_init, alpha_win, gamma_r2_win,
            sys_even, sys_odd, par_even, par_odd)

        br_results[w] = (beta_trace, bm_r4_trace, beta_init)

    return {
        'alpha_mem': alpha_mem,
        'gamma_r2_mem': gamma_r2_mem,
        'gamma_r4_mem': gamma_r4_mem,
        'dbr_results': dbr_results,
        'br_results': br_results,
    }

# =============================================================================
# Output formatting
# =============================================================================
def format_sm_row(vals):
    return ' '.join(f'{v:6d}' for v in vals)

def format_bm_row(vals):
    return ' '.join(f'{v:4d}' for v in vals)

def print_slot(f, slot_idx, fr_win, br_win, dbr_data_win, results):
    alpha_mem = results['alpha_mem']
    gamma_r2_mem = results['gamma_r2_mem']
    gamma_r4_mem = results['gamma_r4_mem']
    dbr_results = results['dbr_results']
    br_results = results['br_results']

    hdr = f"SLOT {slot_idx}"
    units = []
    if fr_win is not None:  units.append(f"FR=W{fr_win}")
    else:                   units.append("FR=idle")
    if br_win is not None:  units.append(f"BR=W{br_win}")
    else:                   units.append("BR=idle")
    if dbr_data_win is not None: units.append(f"DBR=W{dbr_data_win}'")
    else:                        units.append("DBR=idle")

    f.write(f"\n{'#'*78}\n")
    f.write(f"# {hdr}  |  {' , '.join(units)}\n")
    f.write(f"{'#'*78}\n")

    SM_HDR = f"{'Step':>4}  {'S0':>6} {'S1':>6} {'S2':>6} {'S3':>6} {'S4':>6} {'S5':>6} {'S6':>6} {'S7':>6}\n"

    # --- FR ---
    if fr_win is not None:
        w = fr_win - 1  # 0-indexed
        f.write(f"\n  --- FR Window {fr_win} : Alpha (state metrics after ACS) ---\n")
        f.write(f"  {SM_HDR}")
        for step in range(WIN_LEN_R4):
            f.write(f"  {step:4d}  {format_sm_row(alpha_mem[w, step])}\n")

        f.write(f"\n  --- FR Window {fr_win} : Gamma R4 BMs ---\n")
        for step in range(WIN_LEN_R4):
            f.write(f"  {step:4d}  {format_bm_row(gamma_r4_mem[w, step])}\n")

    # --- DBR ---
    if dbr_data_win is not None and dbr_data_win in dbr_results:
        final_beta, beta_trace, bm_r4_trace = dbr_results[dbr_data_win]
        f.write(f"\n  --- DBR Window {dbr_data_win}' : Beta trace (backward, data read in reverse) ---\n")
        f.write(f"  {SM_HDR}")
        for step in range(WIN_LEN_R4):
            f.write(f"  {step:4d}  {format_sm_row(beta_trace[step])}\n")
        f.write(f"  Final beta_init for BR: {format_sm_row(final_beta)}\n")

    # --- BR ---
    if br_win is not None and br_win in br_results:
        beta_trace, bm_r4_trace, beta_init = br_results[br_win]
        f.write(f"\n  --- BR Window {br_win} : Beta init ---\n")
        f.write(f"  {format_sm_row(beta_init)}\n")
        f.write(f"\n  --- BR Window {br_win} : Beta trace (pre-ACS, for LLR compute) ---\n")
        f.write(f"  Read order: step {WIN_LEN_R4-1} down to 0\n")
        f.write(f"  {'Iter':>4} {'RdStep':>6}  {'S0':>6} {'S1':>6} {'S2':>6} {'S3':>6} {'S4':>6} {'S5':>6} {'S6':>6} {'S7':>6}\n")
        for step in range(WIN_LEN_R4):
            rd_step = WIN_LEN_R4 - 1 - step
            f.write(f"  {step:4d} {rd_step:6d}  {format_sm_row(beta_trace[step])}\n")

# =============================================================================
# Run simulation
# =============================================================================
print("=" * 70)
print("BITS Behavioral Simulation -- Full SISO Core (CORE_ID=0)")
print("=" * 70)

results = simulate_siso_core(
    sys_even_ram, sys_odd_ram, par1_even_ram, par1_odd_ram,
    core_id=CORE_ID, num_windows=NUM_WINDOWS)

print(f"Forward recursion: {NUM_WINDOWS} windows x {WIN_LEN_R4} R4-steps")
print(f"DBR windows computed: {len(results['dbr_results'])}")
print(f"BR windows computed:  {len(results['br_results'])}")

# =============================================================================
# Build window schedule and output
# =============================================================================
def build_schedule(core_id, num_windows):
    """Build the slot schedule matching bcjr_core.v FSM."""
    schedule = []
    if core_id == 0:
        # Slot 0: FR=W1, BR=idle, DBR=W2'
        fr_win = 1; br_win = None; dbr_win = 2
        schedule.append((0, fr_win, br_win, dbr_win))
        for slot in range(1, num_windows):
            fr_w = slot + 1 if slot + 1 <= num_windows else None
            br_w = slot
            dbr_w = slot + 2 if slot + 2 <= num_windows + 1 else None
            if CORE_ID == NUM_SISO - 1 and dbr_w is not None and dbr_w > num_windows:
                dbr_w = None
            schedule.append((slot, fr_w, br_w, dbr_w))
        # Final slot: FR=idle, BR=WN
        schedule.append((num_windows, None, num_windows, None))
    return schedule

schedule = build_schedule(CORE_ID, NUM_WINDOWS)

# Output to file and terminal (first 3 + last slot for brevity on terminal)
OUTPUT_FILE = "data/siso_core_simulation.txt"
TERMINAL_SLOTS = [0, 1, 2, NUM_WINDOWS]  # slots to show on terminal

with open(OUTPUT_FILE, 'w') as f:
    f.write("=" * 78 + "\n")
    f.write("BITS Behavioral Simulation -- Full SISO Core Window Schedule (CORE_ID=0)\n")
    f.write(f"NUM_WINDOWS={NUM_WINDOWS}, WIN_LEN_R4={WIN_LEN_R4}, NEG_INF={NEG_INF}\n")
    f.write("=" * 78 + "\n")

    for slot_idx, fr_win, br_win, dbr_data_win in schedule:
        print_slot(f, slot_idx, fr_win, br_win, dbr_data_win, results)

print(f"\nFull schedule written to: {OUTPUT_FILE}")
print(f"Showing slots {TERMINAL_SLOTS} on terminal:\n")

# Print selected slots to terminal
buf = io.StringIO()
for slot_idx, fr_win, br_win, dbr_data_win in schedule:
    if slot_idx in TERMINAL_SLOTS:
        print_slot(buf, slot_idx, fr_win, br_win, dbr_data_win, results)
print(buf.getvalue())

# Quick verification
print("=" * 70)
print("Quick Verification")
print("=" * 70)
a = results['alpha_mem']
print(f"Window 0, Step 0 alpha: {a[0, 0, :]}")
print(f"Expected (RTL):         [-27, -235, 9, -239, -17, -221, 35, -229]")
match = np.array_equal(a[0, 0], [-27, -235, 9, -239, -17, -221, 35, -229])
print(f"Match: {'YES' if match else 'NO'}")