# Zybo AXI Integration Notes

## Hardware Shape

Use `turbo_decoder_axi_lite` as a custom AXI4-Lite IP in a Zynq block design.
Do not bring the raw AXI signals out to package pins. In Vivado, connect:

- `S_AXI` to the Zynq PS `M_AXI_GP0` interface.
- `s_axi_aclk` to PS `FCLK_CLK0` configured for 100 MHz.
- `s_axi_aresetn` to the matching active-low peripheral reset.

The board top should be the Vivado block-design wrapper with DDR and FIXED_IO
ports. The decoder AXI ports remain internal PL nets.

## Register Map

All accesses are 32-bit words.

| Address | Name | Access | Description |
|---:|---|---|---|
| `0x00` | `CONTROL` | W | bit0 starts one decode, bit1 clears done sticky |
| `0x04` | `STATUS` | R | bit0 done sticky, bit1 busy, bit2 one-cycle done pulse |
| `0x08` | `LOAD_ADDR` | W | input BRAM row address |
| `0x0C` | `LOAD_DATA` | W | bits `[9:0] = {core1[4:0], core0[4:0]}` |
| `0x10` | `LOAD_SEL` | W | input BRAM select |
| `0x14` | `LOAD_COMMIT` | W | any write pulses one input BRAM write |
| `0x18` | `LD_RD_ADDR` | W | final extrinsic row read address |
| `0x1C` | `LD_EVEN` | R | final copied even extrinsic row |
| `0x20` | `LD_ODD` | R | final copied odd extrinsic row |
| `0x24` | `HARD_RD_ADDR` | W | hard-decision row read address |
| `0x28` | `HARD_BITS` | R | bits `[0]=core0 even`, `[1]=core1 even`, `[2]=core0 odd`, `[3]=core1 odd` |

`LOAD_SEL` values:

| Value | Input BRAM |
|---:|---|
| `0` | `sys_odd` |
| `1` | `sys_even` |
| `2` | `par1_odd` |
| `3` | `par1_even` |
| `4` | `sys_ilv_odd` |
| `5` | `sys_ilv_even` |
| `6` | `par2_odd` |
| `7` | `par2_even` |

## Software Sequence

1. For each of the 8 input BRAMs, write `LOAD_SEL`.
2. For every row, write `LOAD_ADDR`, write `LOAD_DATA`, then write any value to `LOAD_COMMIT`.
3. Write `CONTROL = 1` to start decoding.
4. Poll `STATUS[0]` until it is `1`.
5. Read hard decisions:
   - write row index to `HARD_RD_ADDR`;
   - read `HARD_BITS`.
6. Optional debug readback of copied final extrinsics:
   - write row index to `LD_RD_ADDR`;
   - read `LD_EVEN` and `LD_ODD`.

For row `r`, `HARD_BITS` maps to frame bit indices:

- bit `[0]`: `2*r`
- bit `[1]`: `SEG_LEN + 2*r`
- bit `[2]`: `2*r + 1`
- bit `[3]`: `SEG_LEN + 2*r + 1`

## Memory Loading

The QPP LUT is initialized by Vivado from `data/qpp_3200.hex` through the
`$readmemh` call in `qpp_lut.v`. This becomes part of the bitstream and is not
loaded by software at runtime for the fixed `K=3200` build.

The input BRAMs are frame-dependent channel LLR memories. They are not
bitstream-initialized in the production wrapper; software loads them through
the AXI register sequence above before each decode.

## Vivado Implementation Flow

1. Create/open a Zybo Z7-10 Vivado project for part `xc7z010clg400-1`.
2. Add all `rtl/*.v` sources and add `data/qpp_3200.hex` as a memory init file.
3. Package `turbo_decoder_axi_lite` as custom IP, or add it directly to a block design as RTL module.
4. Add Zynq7 Processing System, run block automation, enable `M_AXI_GP0`, and set `FCLK_CLK0` to 100 MHz.
5. Connect the decoder AXI-Lite slave to `M_AXI_GP0` through AXI interconnect/SmartConnect.
6. Run `synth_design`, then `opt_design`, `place_design`, `phys_opt_design`, and `route_design`.
7. Check post-route timing at 10 ns before generating the bitstream.
