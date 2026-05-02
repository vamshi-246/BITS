set_param general.maxThreads 4

read_verilog [glob rtl/*.v]
read_xdc constraints/turbo_decoder_axi_ooc.xdc

synth_design -top turbo_decoder_axi_lite -part xc7z010clg400-1 -mode out_of_context

report_timing_summary \
    -delay_type min_max \
    -report_unconstrained \
    -check_timing_verbose \
    -max_paths 10 \
    -input_pins \
    -routable_nets \
    -file timing_report_axi_wrapper_ooc.txt

report_utilization -file utilization_axi_wrapper_ooc.txt
write_checkpoint -force turbo_decoder_axi_wrapper_ooc.dcp
