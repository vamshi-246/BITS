set_param general.maxThreads 4

set repo_root [file normalize [file join [file dirname [info script]] ..]]
cd $repo_root

set out_dir_rel [file join build post_synth_funcsim]
set out_dir [file join $repo_root $out_dir_rel]
file mkdir $out_dir

set netlist_file [file join $out_dir_rel turbo_decoder_post_synth_funcsim.v]
set dcp_file     [file join $out_dir_rel turbo_decoder_post_synth.dcp]
set timing_file  [file join $out_dir_rel turbo_decoder_post_synth_timing_summary.txt]
set util_file    [file join $out_dir_rel turbo_decoder_post_synth_utilization.txt]
set tb_file      [file join tb tb_turbo_decoder_post_synth.v]

proc run_external {cmd_list} {
    puts "Running: [join $cmd_list { }]"
    if {[catch {exec {*}$cmd_list} result]} {
        puts $result
        error "Command failed: [join $cmd_list { }]"
    }
    if {$result ne ""} {
        puts $result
    }
}

puts "Repository root: $repo_root"
puts "Output directory: $out_dir"

read_verilog [glob rtl/*.v]
read_xdc constraints/Zybo-Z7-Master.xdc

synth_design -top turbo_decoder -part xc7z010clg400-1

write_checkpoint -force $dcp_file
write_verilog -force -mode funcsim $netlist_file

report_timing_summary \
    -delay_type min_max \
    -report_unconstrained \
    -check_timing_verbose \
    -max_paths 10 \
    -input_pins \
    -file $timing_file

report_utilization -file $util_file

run_external [list xvlog -work work $netlist_file]
run_external [list xvlog -work work $tb_file]
run_external [list xelab -debug typical -L unisims_ver -L unimacro_ver -L secureip tb_turbo_decoder_post_synth glbl -s turbo_decoder_post_synth_funcsim]
run_external [list xsim turbo_decoder_post_synth_funcsim -runall]

puts "Post-synthesis functional simulation complete."
puts "Netlist: $netlist_file"
puts "Timing summary: $timing_file"
puts "Utilization: $util_file"
