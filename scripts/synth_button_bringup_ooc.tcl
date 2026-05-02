set_param general.maxThreads 4

read_verilog [glob rtl/*.v]
read_xdc constraints/turbo_decoder_button_bringup.xdc

synth_design -top turbo_decoder_button_bringup -part xc7z010clg400-1

report_timing_summary \
    -delay_type min_max \
    -report_unconstrained \
    -check_timing_verbose \
    -max_paths 10 \
    -input_pins \
    -routable_nets \
    -file timing_report_button_bringup_synth.txt

report_utilization -file utilization_button_bringup_synth.txt
write_checkpoint -force turbo_decoder_button_bringup_synth.dcp
