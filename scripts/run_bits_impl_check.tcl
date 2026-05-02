open_project C:/VAMSHI/BITS/BITS.xpr
update_compile_order -fileset sources_1

proc try_set_run_property {run property value} {
    if {[catch {set_property $property $value $run} result]} {
        puts "WARN: could not set $property=$value on [get_property NAME $run]: $result"
    } else {
        puts "Set [get_property NAME $run] $property=$value"
    }
}

set ip_run [get_runs -quiet ila_0_synth_1]
if {[llength $ip_run] == 1} {
    reset_run $ip_run
    launch_runs $ip_run -jobs 4
    wait_on_run $ip_run
    set ip_status [get_property STATUS $ip_run]
    puts "ila_0_synth_1 status: $ip_status"
    if {![string match "*Complete*" $ip_status] && ![string match "*cached*" $ip_status]} {
        puts "ila_0_synth_1 failed."
        exit 1
    }
}

set synth_run [get_runs synth_1]
set impl_run [get_runs impl_1]

# The Zynq-7010 build is close to the slice/LUT limit once the ILA is present.
# Keep synthesis area-focused, then use a post-route phys-opt implementation
# strategy to recover timing.
try_set_run_property $synth_run STRATEGY Flow_AreaOptimized_high
try_set_run_property $synth_run STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY full
try_set_run_property $synth_run STEPS.SYNTH_DESIGN.ARGS.RESOURCE_SHARING on
try_set_run_property $impl_run STRATEGY Performance_ExplorePostRoutePhysOpt

reset_run $synth_run
reset_run $impl_run

launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

set impl_status [get_property STATUS $impl_run]
puts "impl_1 status: $impl_status"

if {![string match "*Complete*" $impl_status]} {
    puts "impl_1 failed. Main log:"
    puts "  C:/VAMSHI/BITS/BITS.runs/impl_1/runme.log"
    exit 1
}

open_run impl_1
puts "debug cores:"
puts "  [get_debug_cores -quiet]"
puts "ila cells:"
puts "  [get_cells -hier -quiet *ila*]"
puts "dbg_hub cells:"
puts "  [get_cells -hier -quiet *dbg_hub*]"

report_utilization -file C:/VAMSHI/BITS/impl_util_after_button_ila.rpt -force
