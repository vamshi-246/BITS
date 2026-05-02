# Saved N=2 LTE-Tail RTL/Python Regression

Date: 2026-04-29

This bundle is the Phase 2 deterministic tail-enabled regression:

- `K = 6144`
- `NUM_SISO = 2`
- `SEG_LEN = 3072`
- `TAIL_LEN = 3` on the last SISO core
- `WIN_LEN = 30`
- `NUM_WINDOWS = 103`
- `NUM_HALF_ITER = 11`
- 5-bit channel/a-priori LLRs, 6-bit extrinsics, 10-bit metrics

Commands used for the saved run:

```powershell
python scripts\gen_encoded_test_vectors.py --include-tail --seed 42
iverilog -g2012 -P tb_turbo_decoder.TEST_TAIL_LEN=3 -o tb\tb_turbo_decoder_tail_check.vvp tb\tb_turbo_decoder.v rtl\*.v
vvp tb\tb_turbo_decoder_tail_check.vvp
python scripts\turbo_ref_model.py --tail-len 3 --show-mismatches 10
```

Observed result:

- RTL completed 11 half-iterations.
- RTL captured 6144 final intrinsic LLRs.
- Tail trellis metrics were loaded through the extra folded input rows.
- Tail positions were suppressed from final outputs.
- Python reference final BER: `164/6144 = 0.026693`.
- RTL final intrinsic BER: `164/6144 = 0.026693`.
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
- `initial_extrinsic.hex`
- `tail_bits.txt`
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

Remaining architecture gaps:

- This is still the N=2 folded memory/interleaver architecture.
- Paper boundary mode is still not implemented.
- N=8 banked routing/interleaver support is still pending.
