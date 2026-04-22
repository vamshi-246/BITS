import os
import re
import glob

def unsigned_to_signed_2s_complement(val, bit_width):
    if (val & (1 << (bit_width - 1))) != 0:
        val = val - (1 << bit_width)
    return val

def format_sys_par(hex_str, row_idx):
    val = int(hex_str, 16)
    data_1 = unsigned_to_signed_2s_complement(val & 0x1F, 5)
    data_2 = unsigned_to_signed_2s_complement((val >> 5) & 0x1F, 5)
    return f"{row_idx:03X} | {data_1:5d} | {data_2:5d}"

def format_input_llr(hex_str, row_idx):
    val = int(hex_str, 16)
    sys = unsigned_to_signed_2s_complement(val & 0x1F, 5)
    par1 = unsigned_to_signed_2s_complement((val >> 5) & 0x1F, 5)
    par2 = unsigned_to_signed_2s_complement((val >> 10) & 0x1F, 5)
    return f"{row_idx:03X} | {sys:5d} | {par1:5d} | {par2:5d}"

def format_extrinsic(line):
    # e.g. "01c: 01 00"
    m = re.match(r"^([0-9a-fA-F]+):\s*([0-9a-fA-F]+)\s+([0-9a-fA-F]+)$", line.strip())
    if m:
        addr = int(m.group(1), 16)
        val1 = unsigned_to_signed_2s_complement(int(m.group(2), 16) & 0x3F, 6)
        val2 = unsigned_to_signed_2s_complement(int(m.group(3), 16) & 0x3F, 6)
        return f"{addr:03X} | {val1:5d} | {val2:5d}"
    return line

def format_ld_ram(line, row_idx):
    # e.g. "bdf 097"
    parts = line.strip().split()
    if len(parts) == 2:
        v1 = int(parts[0], 16)
        v2 = int(parts[1], 16)
        ext_1 = unsigned_to_signed_2s_complement(v1 & 0x3F, 6)
        ext_2 = unsigned_to_signed_2s_complement((v1 >> 6) & 0x3F, 6)
        ext_3 = unsigned_to_signed_2s_complement(v2 & 0x3F, 6)
        ext_4 = unsigned_to_signed_2s_complement((v2 >> 6) & 0x3F, 6)
        return f"{row_idx:03X} | {ext_1:5d} | {ext_2:5d} | {ext_3:5d} | {ext_4:5d}"
    return line

def format_qpp(hex_str, row_idx):
    val = int(hex_str, 16)
    return f"{row_idx:03X} | {val:5d}"

def process_files(data_dir):
    hex_files = glob.glob(os.path.join(data_dir, "*.hex"))
    for fpath in hex_files:
        fname = os.path.basename(fpath)
        out_path = fpath.replace(".hex", "_readable.txt")
        
        with open(fpath, 'r') as f:
            lines = [l.strip() for l in f.readlines() if l.strip()]
            
        out_lines = []
        is_formatted = False
        
        if fname in ['sys_even_ram.hex', 'sys_odd_ram.hex', 'sys_ilv_even_ram.hex', 'sys_ilv_odd_ram.hex',
                     'par1_even_ram.hex', 'par1_odd_ram.hex', 'par2_even_ram.hex', 'par2_odd_ram.hex']:
            out_lines.append("ROW | DATA_1 | DATA_2")
            out_lines.append("---------------------")
            for i, l in enumerate(lines):
                out_lines.append(format_sys_par(l, i))
            is_formatted = True
            
        elif fname == 'input_llr.hex':
            out_lines.append("ROW |   SYS |  PAR1 |  PAR2")
            out_lines.append("---------------------------")
            for i, l in enumerate(lines):
                out_lines.append(format_input_llr(l, i))
            is_formatted = True
            
        elif fname in ['initial_extrinsic.hex', 'ref_extrinsic.hex', 'rtl_extrinsic.hex']:
            out_lines.append("ROW | EXT_1 | EXT_2")
            out_lines.append("-------------------")
            for l in lines:
                out_lines.append(format_extrinsic(l))
            is_formatted = True
            
        elif fname == 'ld_ram_output.hex':
            out_lines.append("ROW | EXT_1 | EXT_2 | EXT_3 | EXT_4")
            out_lines.append("-----------------------------------")
            for i, l in enumerate(lines):
                out_lines.append(format_ld_ram(l, i))
            is_formatted = True
            
        elif fname == 'qpp_6144.hex':
            out_lines.append("ROW | ADDR ")
            out_lines.append("-----------")
            for i, l in enumerate(lines):
                out_lines.append(format_qpp(l, i))
            is_formatted = True
            
        if is_formatted:
            with open(out_path, 'w') as f:
                f.write("\n".join(out_lines) + "\n")
            print(f"Processed {fname} -> {os.path.basename(out_path)}")

if __name__ == "__main__":
    process_files('C:\\Users\\USER\\Documents\\Digital_VLSI_grp_10_BITS\\BITS\\data')
