# Zybo FPGA Bring-up Tests for `turbo_decoder_axi_lite`

This folder contains a staged Vitis bare-metal test application for the Zybo/Zynq implementation. It follows this test order:

1. AXI read/write smoke test.
2. Load one BRAM row and check that AXI does not hang.
3. Load all 8 input BRAMs.
4. Start the decoder and verify `STATUS[1]` becomes busy.
5. Wait for `STATUS[0]` done.
6. Read the first 10 hard-decision rows.
7. Read all 3200 decoded bits.
8. Compare against `true_info_bits` and the RTL simulation hard bits.

## Faster PL-only Button Bring-up

If Vitis is slowing you down, use the separate top-level:

```text
rtl/turbo_decoder_button_bringup.v
constraints/turbo_decoder_button_bringup.xdc
```

This path does not use AXI or ARM software. The current fixed input vectors are
embedded into bitstream ROMs from:

- `data/sys_odd_ram.hex`
- `data/sys_even_ram.hex`
- `data/par1_odd_ram.hex`
- `data/par1_even_ram.hex`
- `data/sys_ilv_odd_ram.hex`
- `data/sys_ilv_even_ram.hex`
- `data/par2_odd_ram.hex`
- `data/par2_even_ram.hex`

The hardware copies those ROM rows into the decoder through the existing
`load_en/load_bram_sel/load_addr/load_data` interface, so the decoder RTL does
not need a new data path.

Button mapping:

| Control | Function |
|---|---|
| `BTN0` | Load all embedded input vectors into decoder BRAMs. |
| `BTN1` | Start decode after `LED0` is on. |
| `BTN2` | Reset while held. |
| `BTN3` | Replay the output sweep after decode is done. |

LED mapping:

| LED | Meaning |
|---|---|
| `LED0` | Input vectors loaded. |
| `LED1` | Loader or decoder busy. |
| `LED2` | Decode done latched. |
| `LED3` | Output sweep active. |

Suggested manual flow:

1. Create/open a Zybo Z7-10 Vivado project.
2. Add all current repository `rtl/*.v` sources. If Vivado asks whether to
   copy sources into the project, that is fine, but copy from this repository's
   `rtl` directory, not from an older `*.srcs/imports/rtl` project copy.
3. Set top module to `turbo_decoder_button_bringup`.
4. Add `constraints/turbo_decoder_button_bringup.xdc`.
5. Add the `data/*.hex` files above plus `data/qpp_3200.hex` as memory
   initialization files.
6. Create an ILA IP named `ila_0` from **IP Catalog**. Do not use a block
   design for this simple RTL top.
7. For Zybo Z7-10, use the minimal 1-probe packed ILA below, then add the
   `USE_MIN_ILA_IP` Verilog define.
8. Run synthesis, implementation, and bitstream generation.
9. Program the FPGA with the generated `.bit` and matching `.ltx`, arm the ILA,
   press `BTN0`, wait for `LED0`, then press `BTN1`.
   The sweep runs automatically after done. If the ILA was not armed in time,
   press `BTN3` to replay the sweep.

Quick synthesis sanity check from the repository root:

```powershell
vivado -mode batch -source scripts\synth_button_bringup_ooc.tcl
```

Behavioral verification of the button wrapper:

```powershell
iverilog -g2012 -o build\button_bringup_sim.vvp tb\tb_button_bringup.v rtl\*.v
vvp build\button_bringup_sim.vvp
```

Expected result:

```text
Button bring-up captured rows: 800/800
Errors vs true_info_bits: 1/3200
Mismatches vs rtl_final_hard_bits: 0/3200
PASS: button bring-up output matches expected RTL vector ordering.
```

The Z7-10 build is tight, so the minimal ILA packs the output into one probe:

```verilog
dbg_packed_output = {dbg_sweep_valid, dbg_hard_bits[3:0]}
```

Packed probe bits:

| Probe bit | Meaning |
|---:|---|
| `probe0[4]` | `dbg_sweep_valid`; trigger on this rising/high. |
| `probe0[3:0]` | `dbg_hard_bits`; four hard-decision bits for one output row. |

There is no row-index probe in the minimal build. The sweep emits 800 valid
rows on consecutive clocks, so the row number is the count of valid samples:
first valid sample is row `0`, last valid sample is row `799`.

Minimal ILA settings for Zybo Z7-10:

| ILA setting | Value |
|---|---|
| Component name | `ila_0` |
| Monitor type | `Native` |
| Number of probes | `1` |
| Sample data depth | `1024` |
| Comparators | `1` |
| Input pipe stages | `0` |
| Capture Control | Disabled |
| Advanced Trigger | Disabled |
| Trigger | `probe0[4] == 1` |
| Hardware Manager trigger position | `0` or as low as possible |

Probe widths:

| Port | Signal | Width |
|---|---|---:|
| `probe0` | `dbg_packed_output` | 5 |

Make `probe0` `DATA_AND_TRIGGER`. Keep the number of comparators at `1`. Use a
trigger position near `0`; if the default trigger position is in the middle of
the capture window, a 1024-sample ILA may capture only part of the 800-row
output sweep after the trigger.

Enable the minimal direct RTL instantiation with:

```tcl
set_property verilog_define {USE_MIN_ILA_IP} [get_filesets sources_1]
update_compile_order -fileset sources_1
set_property STRATEGY Flow_AreaOptimized_high [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY full [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.RESOURCE_SHARING on [get_runs synth_1]
set_property STRATEGY Performance_ExplorePostRoutePhysOpt [get_runs impl_1]
reset_run synth_1
reset_run impl_1
launch_runs impl_1 -to_step write_bitstream
```

The repository also includes helpers for the existing projects:

```tcl
source C:/VAMSHI/IIT\ Mandi\ Academic\ Folder/IITM\ 6th\ Sem/DVAD/BITS_LTE_Parallel_Turbo_Decoder/scripts/configure_min_ila_project.tcl
source C:/VAMSHI/IIT\ Mandi\ Academic\ Folder/IITM\ 6th\ Sem/DVAD/BITS_LTE_Parallel_Turbo_Decoder/scripts/configure_bits_project_absolute_data.tcl
```

Do not also define `USE_ILA_IP` or `USE_MARK_DEBUG` for the Z7-10 minimal ILA
flow.

For a fresh project, the required source set is:

- all files in `rtl/*.v`
- `constraints/turbo_decoder_button_bringup.xdc`
- the 8 input memory files listed above
- `data/qpp_3200.hex`
- a newly created `ila_0` IP configured as the 1-probe, 5-bit ILA shown above

The top module must be `turbo_decoder_button_bringup`.

Vivado resolves `$readmemh` files from the synthesis run directory, not always
from the project root. If synthesis reports `could not open $readmem data file`,
install the pre-synthesis copy hook:

```tcl
source C:/VAMSHI/IIT\ Mandi\ Academic\ Folder/IITM\ 6th\ Sem/DVAD/BITS_LTE_Parallel_Turbo_Decoder/scripts/install_readmem_data_hook.tcl
reset_run synth_1
launch_runs synth_1
```

The hook copies the required `.hex` files into both:

```text
<project>.runs/synth_1/
<project>.runs/synth_1/data/
```

so both `sys_odd_ram.hex` and `data/sys_odd_ram.hex` RTL paths work.

If you previously used **Set Up Debug**, remove or comment the old generated
debug commands from the active XDC before rebuilding. In particular, remove old
lines such as:

```tcl
create_debug_core ...
connect_debug_port ...
set_property port_width ...
set_property PROBE_TYPE ...
```

Keep only the board pin/clock constraints plus the 1-probe `ila_0.xci` direct
RTL instantiation. Stale debug XDC commands can silently keep the large ILA in
the design and cause the same placement failure again.

Resource note: this design is close to the `xc7z010clg400-1` limit. The working
timing-clean run used `15473 / 17600` Slice LUTs, `8190 / 35200` registers,
`18.5 / 60` BRAM tiles, and `6 / 80` DSPs. A full 19-probe ILA plus marked
debug nets pushes the design over the device limit. If implementation reports
something like `Luts: 20493 (total), available capacity: 17600`, the fix is to
reduce debug probes/comparators and use the area/timing strategies above, not to
rerun placement unchanged.

For the current `C:/VAMSHI/BITS` project, the timing-clean files produced by the
checked run are:

```text
C:/VAMSHI/BITS/turbo_decoder_button_bringup_timing_clean.bit
C:/VAMSHI/BITS/turbo_decoder_button_bringup_timing_clean.ltx
```

That run met timing at the 100 MHz clock constraint with final routed
`WNS=0.300 ns`, `TNS=0.000 ns`, `WHS=0.023 ns`, and `THS=0.000 ns`.

The older full ILA option is still available for larger devices or experiments.
If you manually create an ILA IP named `ila_0` with all 19 probe ports listed
above, enable it with this define:

```tcl
set_property verilog_define {USE_ILA_IP} [get_filesets sources_1]
update_compile_order -fileset sources_1
```

Then rerun synthesis and implementation. After implementation, these commands
must show a debug core:

```tcl
open_run impl_1
get_debug_cores
get_debug_ports
get_cells -hier *ila*
get_cells -hier *dbg_hub*
```

If they still say there are no ChipScope debug cores, the ILA was not connected
to the implemented design.

For row `r`, `dbg_hard_bits` has the same bit order as the AXI readback path:

| `dbg_hard_bits` bit | Frame bit index |
|---:|---:|
| 0 | `2*r` |
| 1 | `1600 + 2*r` |
| 2 | `2*r + 1` |
| 3 | `1600 + 2*r + 1` |

To save the captured vectors on the laptop, use Vivado Hardware Manager:

```tcl
set ila [lindex [get_hw_ilas] 0]
run_hw_ila $ila
# Press BTN3 now if decode already finished, or press BTN1 to start a new run.
wait_on_hw_ila $ila
set cap [upload_hw_ila_data $ila]
write_hw_ila_data -csv_file C:/VAMSHI/turbo_button_ila_capture.csv $cap
```

In the exported CSV, keep only samples where packed `probe0[4]` is `1`. Those
samples are the saved output vector rows. The converter below understands both
the current packed `probe0` CSV and the older separate-probe CSV.

To convert the ILA CSV into a 3200-line hard-bit vector:

```powershell
python fpga_bringup\scripts\ila_capture_to_vectors.py C:\VAMSHI\turbo_button_ila_capture.csv --out-prefix fpga_bringup\fpga_button_ila
python fpga_bringup\scripts\compare_ila_hard_bits.py
```
   
This writes:

- `fpga_bringup/fpga_button_ila_hard_bits.txt`
- `fpga_bringup/fpga_button_ila_rows.csv`

## Folder Contents

| Path | Purpose |
|---|---|
| `vitis_app_src/main.c` | The sequential bring-up test program. |
| `vitis_app_src/turbo_decoder_axi.h` | Register map, base-address selection, and constants. |
| `vitis_app_src/turbo_decoder_axi.c` | AXI helper functions for loading inputs, starting decode, polling, and reading hard bits. |
| `vitis_app_src/turbo_test_vectors.h` | Generated header containing the 8 input BRAM arrays and reference bits. |
| `scripts/generate_vitis_vectors.py` | Regenerates `turbo_test_vectors.h` from the repo `data/` folder. |
| `scripts/prepare_vitis_app.ps1` | Regenerates vectors and copies all Vitis source files into an existing Vitis app `src` folder. |
| `scripts/ila_capture_to_vectors.py` | Converts exported ILA CSV rows into saved hard-bit/output-row files. |
| `scripts/compare_ila_hard_bits.py` | Compares converted ILA hard bits against RTL hard bits and truth bits. |
| `../scripts/synth_button_bringup_ooc.tcl` | Synthesizes the PL-only button top for a quick Vivado sanity check. |
| `../scripts/configure_min_ila_project.tcl` | Reconfigures the Vivado project `ila_0` IP to the 1-probe packed output-capture setup. |
| `../scripts/configure_bits_project_absolute_data.tcl` | Configures `C:/VAMSHI/BITS` with absolute `$readmemh` data paths and the minimal ILA define. |
| `../scripts/run_bits_impl_check.tcl` | Runs area-optimized implementation for `C:/VAMSHI/BITS`. |
| `../scripts/run_bits_impl_timing_check.tcl` | Runs the timing-focused implementation variant that met 100 MHz timing. |
| `../scripts/install_readmem_data_hook.tcl` | Installs the Vivado pre-synthesis hook for `$readmemh` data files. |
| `../scripts/copy_readmem_data_to_run.tcl` | Copies the `.hex` files into the active synthesis run directory. |
| `../scripts/report_button_debug_nets.tcl` | Opens the synthesized checkpoint and lists `MARK_DEBUG` nets. |
| `../rtl/turbo_decoder_button_bringup.v` | PL-only button/ILA bring-up top with embedded input vectors. |
| `../tb/tb_button_bringup.v` | Behavioral testbench for the PL-only button top. |
| `../constraints/turbo_decoder_button_bringup.xdc` | Zybo clock, button, and LED constraints for the PL-only top. |

## Hardware Assumptions

The Vitis app assumes this hardware design:

- Zynq PS `M_AXI_GP0` connected to `turbo_decoder_axi_lite/s_axi`.
- `FCLK_CLK0` is 100 MHz.
- `s_axi_aresetn` comes from the matching processor-system reset block.
- `data/qpp_3200.hex` was added to Vivado as a memory initialization file, so the QPP LUT is already in the bitstream.
- The decoder AXI address is the Address Editor base address. Your Vivado log showed `0x40000000`; check this again in Address Editor.

The software does not load the QPP LUT. It only loads the frame-dependent 8 input BRAMs through AXI.

## Regenerate Test Vectors

From the repository root:

```powershell
python fpga_bringup\scripts\generate_vitis_vectors.py
```

Expected output:

```text
Generated ...\fpga_bringup\vitis_app_src\turbo_test_vectors.h
Input memories: 8 x 802 rows
Truth bits: 3200
RTL expected hard bits: 3200
```

The generated header embeds:

- `sys_odd_ram.hex`
- `sys_even_ram.hex`
- `par1_odd_ram.hex`
- `par1_even_ram.hex`
- `sys_ilv_odd_ram.hex`
- `sys_ilv_even_ram.hex`
- `par2_odd_ram.hex`
- `par2_even_ram.hex`

- `rtl_final_hard_bits.txt`

## Copy Sources Into Vitis

If your Vitis app already exists, use the helper script. Change the path to your actual app `src` folder:

```powershell
powershell -ExecutionPolicy Bypass -File fpga_bringup\scripts\prepare_vitis_app.ps1 -VitisSrc C:\VAMSHI\BITS_FINAL\turbo_decoder_test\src
```

This copies:

- `main.c`
- `turbo_decoder_axi.c`
- `turbo_decoder_axi.h`
- `turbo_test_vectors.h`

You can also copy these four files manually from `fpga_bringup\vitis_app_src`.

## Vitis Setup

Use **Vitis Software**.

1. Export hardware from Vivado with bitstream included:

   - In Vivado: `File > Export > Export Hardware`.
   - Select `Include bitstream`.
   - Export the `.xsa`.

2. Create or update the Vitis platform from that `.xsa`.

3. Create a bare-metal application for `ps7_cortexa9_0`.

   - OS: `standalone`
   - Language: `C`
   - Template: any template is fine, because you will replace the source files. If the template list fails because of the Windows user path with a space, create an empty application and copy the files into `src`.

4. Put the four files from `vitis_app_src` into the application `src` directory.

5. Check the base address.

   Open generated `xparameters.h` in Vitis and search for:

   - `TURBO`
   - `DECODER`
   - `BASEADDR`

   In your current exported platform this macro exists:

   ```c
   #define XPAR_TURBO_DECODER_AXI_LI_0_BASEADDR 0x40000000
   ```

   `turbo_decoder_axi.h` will use that automatically. If not, either keep the default `0x40000000` or add this compiler symbol:

   ```text
   TURBO_DECODER_BASEADDR=0x40000000
   ```

   Use your actual Vivado Address Editor value if it differs.

6. Build the application.

   Your Vitis 2024 CMake template uses `aux_source_directory(${CMAKE_SOURCE_DIR} _sources)`, so both `main.c` and `turbo_decoder_axi.c` are picked up automatically after they are copied into `src`.

7. Connect the Zybo board:

   - JTAG USB connected.
   - UART terminal open, usually 115200 baud, 8 data bits, no parity, 1 stop bit.
   - Board powered and boot mode suitable for JTAG programming.

8. Program FPGA and run:

   - In Vitis, program the FPGA with the bitstream from the platform.
   - Run the application on `ps7_cortexa9_0`.

## XSDB and UART Notes

Seeing this in the XSDB console means the ELF was downloaded and started:

```text
Successfully downloaded .../turbo_decoder_test.elf
Info: ARM Cortex-A9 MPCore #0 ... Running
```

The bring-up program does not print to the `xsdb%` prompt. All `xil_printf`
output goes to the board UART selected in the standalone BSP/domain settings.
Open a serial terminal on the Zybo USB-UART COM port before running the app:

```text
115200 baud, 8 data bits, no parity, 1 stop bit
```

If typing `hi` at `xsdb%` prints numbered `hi` lines, that is XSDB command
history, not input sent to the ARM program. This application does not read
stdin.

The messages below are Vitis/XSDB environment setup noise and are not the
reason the ARM application fails to print:

```text
'\gnu\microblaze\lin\bin\' is not recognized as an internal or external command
'\gnu\microblaze\nt\bin\' is not recognized as an internal or external command
'\gnuwin\bin\' is not recognized as an internal or external command
```

This project runs on `ps7_cortexa9_0`, not MicroBlaze. To avoid those warnings,
start tools from the Vitis 2024.1 command prompt, or from a normal `cmd.exe`
session after:

```cmd
call C:\Xilinx\Vitis\2024.1\settings64.bat
```

If the UART terminal shows no banner at all, check the BSP/domain `stdin` and
`stdout` device. On a Zybo design this is usually the PS UART connected to the
USB-UART bridge, often `ps7_uart_1`. If the banner appears but the app hangs
after `1. AXI read/write smoke test`, then debug the AXI base address,
interconnect, clock, and reset wiring.

## Expected Console Output

The important expected results are:

```text
[PASS] AXI register smoke
[PASS] single sys_odd row load
[PASS] all input BRAM load
[PASS] busy observed
[PASS] decode done
[PASS] read all hard bits
Errors vs true_info_bits: 1/3200
Mismatches vs rtl_final_hard_bits: 0/3200
[PASS] FPGA output matches RTL simulation vector ordering
```

For this current vector set, `1/3200` BER error is expected from the existing RTL simulation report. The stronger hardware integration check is `Mismatches vs rtl_final_hard_bits: 0/3200`; if that is nonzero, suspect load ordering, base address, stale bitstream, or hard-bit readback ordering.

## What Each Test Proves

`AXI read/write smoke test` only writes readable control registers. It proves the PS can access the decoder slave without using the input memories.

`Load one BRAM row` performs the first real input-memory write through `LOAD_SEL`, `LOAD_ADDR`, `LOAD_DATA`, and `LOAD_COMMIT`. There is no input-memory readback port, so the check is that the AXI write completes and the design stays responsive.

`Load all 8 BRAMs` writes all 6416 input rows. The BRAM select order must match the RTL wrapper:

| Select | Memory |
|---:|---|
| 0 | `sys_odd` |
| 1 | `sys_even` |
| 2 | `par1_odd` |
| 3 | `par1_even` |
| 4 | `sys_ilv_odd` |
| 5 | `sys_ilv_even` |
| 6 | `par2_odd` |
| 7 | `par2_even` |

`Start decoder` writes `CONTROL[0]`. The app then checks that `STATUS[1]` busy is observed before waiting for final done.

`Read hard bits` uses `HARD_RD_ADDR` and `HARD_BITS`. Each output row packs four decoded bits:

| `HARD_BITS` bit | Frame bit index |
|---:|---:|
| 0 | `2*r` |
| 1 | `1600 + 2*r` |
| 2 | `2*r + 1` |
| 3 | `1600 + 2*r + 1` |

## If Something Fails

If the first smoke test fails, check the AXI base address and that the app is running on `ps7_cortexa9_0`, not MicroBlaze or a different domain.

If loading hangs, check reset wiring in the block design. `s_axi_aresetn` must be active high during software execution.

If busy never asserts and done never asserts, check that `FCLK_CLK0` is connected to both the AXI interconnect and the decoder `s_axi_aclk`.

If done never asserts but busy asserted, use Vivado ILA or read `STATUS` repeatedly. This points to a decoder runtime/control issue, not an AXI address issue.

If RTL mismatches are nonzero but BER looks close, the most likely problem is BRAM select ordering or hard-bit row unpacking.
