# Saved N=2 RTL/Python Regression

Date: 2026-04-29

This bundle is the current validated RTL regression, not an LTE-tail regression.
It uses the existing `K=6144`, `NUM_SISO=2` vector format and does not include
the extra 3 LTE tail trellis steps in the RTL decode.

Commands used for the saved run:

```powershell
iverilog -g2012 -o tb\tb_turbo_decoder_param_check.vvp tb\tb_turbo_decoder.v rtl\*.v
vvp tb\tb_turbo_decoder_param_check.vvp
python scripts\turbo_ref_model.py
```

Observed result:

- RTL completed 11 half-iterations.
- RTL captured 6144 final intrinsic LLRs.
- Python reference final BER: `78/6144 = 0.012695`.
- RTL final intrinsic BER: `78/6144 = 0.012695`.
- RTL vs Python final intrinsic mismatches: `0/6144`.
- RTL vs Python final hard-bit mismatches: `0/6144`.
- RTL vs Python final extrinsic value mismatches: `0/6144`.

Saved inputs:

- `sys_even_ram.hex`
- `sys_odd_ram.hex`
- `par1_even_ram.hex`
- `par1_odd_ram.hex`
- `sys_ilv_even_ram.hex`
- `sys_ilv_odd_ram.hex`
- `par2_even_ram.hex`
- `par2_odd_ram.hex`
- `input_llr.hex`
- `true_info_bits.txt`

Saved RTL outputs:

- `ld_ram_output.hex`
- `rtl_final_intrinsic.txt`
- `rtl_final_hard_bits.txt`

Saved Python reference outputs:

- `ref_final_intrinsic.txt`
- `ref_final_hard_bits.txt`
- `ref_final_extrinsic.hex`

Saved BER/compare report:

- `ber_results.txt`

LTE tail status:

The requested "same LLRs with extra tail bits" comparison has not been run
because the RTL still lacks Phase 2 tail-trellis support. The current RTL
`bcjr_core` still uses a single `frame_len` for both recursion and output
filtering, and the top-level memories/load format still only cover the
information segment length. A valid tail comparison needs separate recursion
valid length and output valid length, plus tail rows in the input memories.
