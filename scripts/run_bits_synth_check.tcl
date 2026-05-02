open_project C:/VAMSHI/BITS/BITS.xpr
reset_run synth_1
launch_runs synth_1
wait_on_run synth_1
open_run synth_1
puts "synth_1 status: [get_property STATUS [get_runs synth_1]]"
