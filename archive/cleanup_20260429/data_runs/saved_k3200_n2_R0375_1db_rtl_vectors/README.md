# K=3200 N=2 R=0.375 1 dB RTL Vector Set

This bundle contains RTL BRAM hex files generated with paper-normalized
`channel_rate=0.375`, plus RTL and Python comparison outputs.

Vector generation command:

```powershell
python scripts\qpp_6144_gen.py --K 3200
python scripts\gen_encoded_test_vectors.py --K 3200 --num-siso 2 --include-tail --seed 42 --ebn0-db 1.0 --channel-rate 0.375
```

RTL simulation command:

```powershell
iverilog -g2012 -P tb_turbo_decoder.TEST_K=3200 -P tb_turbo_decoder.TEST_TAIL_LEN=3 -P tb_turbo_decoder.TEST_PAPER_BOUNDARY=1 -P tb_turbo_decoder.TEST_QPP_LUT_FILE='"data/qpp_3200.hex"' -o tb\tb_turbo_decoder_k3200_R0375_paper_check.vvp tb\tb_turbo_decoder.v rtl\*.v
vvp tb\tb_turbo_decoder_k3200_R0375_paper_check.vvp
```

Comparison commands:

```powershell
python scripts\turbo_ref_model.py --K 3200 --num-siso 2 --tail-len 3 --boundary-mode paper --show-mismatches 10
python scripts\windowed_parallel_ber.py --decoder radix4 --K 3200 --num-siso 2 --window-size 30 --half-iters 11 --quantize --boundary-mode paper --from-bram-dir data
```

The RTL testbench now also writes:

```text
data/rtl_ber_results.txt
```

Configuration:

- K=3200
- NUM_SISO=2
- QPP f1=111, f2=240
- Eb/N0=1.0 dB
- Channel rate=0.375
- Seed=42
- LTE tail enabled
- Boundary mode=paper

Generator channel statistics:

- Float channel hard-decision errors: `534/3200 = 0.166875`
- Quantized systematic hard-decision errors: `527/3200 = 0.1646875`

Decoded result:

- `turbo_ref_model.py` final L_D BER: `0/3200 = 0`
- RTL final L_D BER: `0/3200 = 0`
- `windowed_parallel_ber.py --decoder radix4` final L_D BER: `0/3200 = 0`
- RTL intrinsic mismatches: `0/3200`
- RTL hard-bit mismatches: `0/3200`
- RTL final-extrinsic mismatches: `0/3200`
- RTL testbench BER report: `rtl_ber_results.txt`

Primary RTL input files:

- `sys_even_ram.hex`, `sys_odd_ram.hex`
- `par1_even_ram.hex`, `par1_odd_ram.hex`
- `sys_ilv_even_ram.hex`, `sys_ilv_odd_ram.hex`
- `par2_even_ram.hex`, `par2_odd_ram.hex`
- `initial_extrinsic.hex`
- `qpp_3200.hex`
