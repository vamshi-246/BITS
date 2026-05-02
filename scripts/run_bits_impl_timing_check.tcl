open_project C:/VAMSHI/BITS/BITS.xpr
update_compile_order -fileset sources_1

proc try_set_run_property {run property value} {
    if {[catch {set_property $property $value $run} result]} {
        puts "WARN: could not set $property=$value on [get_property NAME $run]: $result"
        return 0
    }
    puts "Set [get_property NAME $run] $property=$value"
    return 1
}

set synth_run [get_runs synth_1]
try_set_run_property $synth_run STRATEGY Flow_AreaOptimized_high
try_set_run_property $synth_run STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY full
try_set_run_property $synth_run STEPS.SYNTH_DESIGN.ARGS.RESOURCE_SHARING on

set timing_run [get_runs -quiet impl_timing_1]
if {[llength $timing_run] == 0} {
    set base_impl [get_runs impl_1]
    set impl_flow [get_property FLOW $base_impl]
    if {$impl_flow eq ""} {
        set impl_flow "Vivado Implementation 2024"
    }
    create_run impl_timing_1 -parent_run synth_1 -flow $impl_flow -strategy Performance_ExplorePostRoutePhysOpt
    set timing_run [get_runs impl_timing_1]
} else {
    set timing_run [lindex $timing_run 0]
    try_set_run_property $timing_run STRATEGY Performance_ExplorePostRoutePhysOpt
}

reset_run $timing_run
launch_runs $timing_run -to_step write_bitstream -jobs 4
wait_on_run $timing_run

set timing_status [get_property STATUS $timing_run]
puts "impl_timing_1 status: $timing_status"

if {![string match "*Complete*" $timing_status]} {
    puts "impl_timing_1 failed. Main log:"
    puts "  C:/VAMSHI/BITS/BITS.runs/impl_timing_1/runme.log"
    exit 1
}

open_run impl_timing_1
report_timing_summary -file C:/VAMSHI/BITS/impl_timing_1_timing_summary.rpt
report_utilization -file C:/VAMSHI/BITS/impl_timing_1_util.rpt -force
puts "debug cores:"
puts "  [get_debug_cores -quiet]"
puts "ila cells:"
puts "  [get_cells -hier -quiet *ila*]"
puts "dbg_hub cells:"
puts "  [get_cells -hier -quiet *dbg_hub*]"
