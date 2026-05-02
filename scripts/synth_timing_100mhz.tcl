set_param general.maxThreads 4

read_verilog [glob rtl/*.v]
read_xdc constraints/Zybo-Z7-Master.xdc

synth_design -top turbo_decoder -part xc7z010clg400-1

report_timing_summary \
    -delay_type min_max \
    -report_unconstrained \
    -check_timing_verbose \
    -max_paths 10 \
    -input_pins \
    -routable_nets \
    -file timing_report_after_pipeline.txt

report_utilization -file utilization_after_pipeline.txt
