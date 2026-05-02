# K=3200 N=2 R=0.375 1 dB Nonzero-BER RTL Vector Set

This bundle contains a deterministic RTL input set whose decoded BER is
nonzero. It is useful for verifying that the RTL testbench BER calculation is
actually exercising the error-count path.

Vector generation command:

```powershell
python scripts\qpp_6144_gen.py --K 3200
python scripts\gen_encoded_test_vectors.py --K 3200 --num-siso 2 --include-tail --seed 57 --ebn0-db 1.0 --channel-rate 0.375
```

RTL simulation command:

```powershell
iverilog -g2012 -P tb_turbo_decoder.TEST_K=3200 -P tb_turbo_decoder.TEST_NUM_SISO=2 -P tb_turbo_decoder.TEST_SEG_LEN=1600 -P tb_turbo_decoder.TEST_TAIL_LEN=3 -P tb_turbo_decoder.TEST_ROW_DEPTH=800 -P tb_turbo_decoder.TEST_INPUT_ROW_DEPTH=802 -P tb_turbo_decoder.TEST_ROW_ADDR_W=10 -P tb_turbo_decoder.TEST_INPUT_WORD_W=10 -P tb_turbo_decoder.TEST_EXTR_WORD_W=12 -P tb_turbo_decoder.TEST_NUM_HALF_ITER=11 -P tb_turbo_decoder.TEST_PAPER_BOUNDARY=1 -P tb_turbo_decoder.TEST_QPP_LUT_FILE='"data/qpp_3200.hex"' -o tb\tb_turbo_decoder_k3200_n2_R0375_seed57_nonzero.vvp tb\tb_turbo_decoder.v rtl\*.v
vvp tb\tb_turbo_decoder_k3200_n2_R0375_seed57_nonzero.vvp
```

Configuration:

- K=3200
- NUM_SISO=2
- QPP f1=111, f2=240
- Eb/N0=1.0 dB
- Channel rate=0.375
- Seed=57
- LTE tail enabled
- Boundary mode=paper
- Half iterations=11

Generator channel statistics:

- Float channel hard-decision errors: `544/3200 = 0.17`
- Quantized systematic hard-decision errors: `544/3200 = 0.17`

Decoded result:

- RTL testbench final L_D BER: `1/3200 = 0.0003125`
- `turbo_ref_model.py` final L_D BER: `1/3200 = 0.0003125`
- `windowed_parallel_ber.py --decoder radix4` final L_D BER: `1/3200 = 0.0003125`
- RTL intrinsic mismatches: `0/3200`
- RTL hard-bit mismatches: `0/3200`
- RTL final-extrinsic mismatches: `0/3200`

Primary files:

- `sys_even_ram.hex`, `sys_odd_ram.hex`
- `par1_even_ram.hex`, `par1_odd_ram.hex`
- `sys_ilv_even_ram.hex`, `sys_ilv_odd_ram.hex`
- `par2_even_ram.hex`, `par2_odd_ram.hex`
- `initial_extrinsic.hex`
- `qpp_3200.hex`
- `rtl_ber_results.txt`
- `rtl_final_intrinsic.txt`
- `rtl_final_hard_bits.txt`
- `ld_ram_output.hex`
