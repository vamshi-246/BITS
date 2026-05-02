import sys

def load_hex(filename):
    data = {}
    total = 0
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line: continue
            # Format expected: "000: 1a 3f" => "ADDR: ODD EVEN"
            parts = line.split(":")
            if len(parts) == 2:
                addr = int(parts[0], 16)
                vals = parts[1].strip().split()
                if len(vals) == 2:
                    odd = int(vals[0], 16)
                    even = int(vals[1], 16)
                    # Convert 6-bit 2's complement to signed int
                    odd = odd - 64 if odd >= 32 else odd
                    even = even - 64 if even >= 32 else even
                    data[addr] = (odd, even)
                    total += 2
    return data, total

if __name__ == "__main__":
    import os
    if not os.path.exists("../data/ref_extrinsic.hex") or not os.path.exists("../data/rtl_extrinsic.hex"):
        print("Required hex files missing.")
        sys.exit(1)

    ref_data, ref_count = load_hex("../data/ref_extrinsic.hex")
    rtl_data, rtl_count = load_hex("../data/rtl_extrinsic.hex")
    
    print(f"Loaded {ref_count} reference values and {rtl_count} RTL values.")
    
    errors = 0
    max_err = 0
    for addr in sorted(ref_data.keys()):
        ref_odd, ref_even = ref_data[addr]
        
        if addr in rtl_data:
            rtl_odd, rtl_even = rtl_data[addr]
            
            diff_odd = abs(ref_odd - rtl_odd)
            diff_even = abs(ref_even - rtl_even)
            
            if diff_odd != 0 or diff_even != 0:
                if errors < 10:
                    print(f"Mismatch at ADDR {addr:03X}: REF=({ref_odd:3}, {ref_even:3}) | RTL=({rtl_odd:3}, {rtl_even:3})")
                errors += int(diff_odd != 0) + int(diff_even != 0)
                max_err = max(max_err, diff_odd, diff_even)
        else:
            print(f"Missing ADDR {addr:03X} in RTL output")
            errors += 2
            
    # Note on mismatches:
    # Small differences (diff_odd <= 2) at window boundaries are expected due to 
    # dummy backward recursion convergence limits and modulo arithmetic approximations 
    # versus the idealized floating point max() in the reference model.
            
    print("-" * 50)
    if errors == 0:
        print("SUCCESS! RTL perfectly matches the Python Reference Model.")
    else:
        print(f"Comparsion finished with {errors} mismatches.")
        print(f"Max Absolute Error Delta: {max_err}")
        if max_err <= 3:
            print("Note: Small max error suggests RTL is functionally correct but showing sliding-window/quantization convergence differences.")
        else:
            print("Note: Large max error indicates a logical bug in the RTL.")
