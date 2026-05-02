# Install the pre-synthesis hook that copies $readmemh files into synth_1.
#
# Source this from a Vivado Tcl Console after opening the project, or run:
#   vivado -mode batch -source scripts/install_readmem_data_hook.tcl

if {[llength [get_projects -quiet]] == 0} {
    set default_project "C:/VAMSHI/BITS/BITS.xpr"
    if {![file exists $default_project]} {
        error "No project is open and default project was not found: $default_project"
    }
    open_project $default_project
}

set script_dir [file normalize [file dirname [info script]]]
set hook_script [file normalize [file join $script_dir copy_readmem_data_to_run.tcl]]

if {![file exists $hook_script]} {
    error "Missing hook script: $hook_script"
}

set synth_run [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.TCL.PRE $hook_script $synth_run

puts "Installed synth_1 pre-hook:"
puts "  $hook_script"
puts "Project:"
puts "  [get_property DIRECTORY [current_project]]/[get_property NAME [current_project]].xpr"
puts ""
puts "Now reset and rerun synthesis:"
puts "  reset_run synth_1"
puts "  launch_runs synth_1"
