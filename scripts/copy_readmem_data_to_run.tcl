# Copy $readmemh files into the active synthesis run directory.
#
# Vivado project runs often execute synth_design from <project>.runs/synth_1,
# while $readmemh relative paths are resolved from that run directory. This hook
# copies the required files both flat and under data/ so either
# "sys_odd_ram.hex" or "data/sys_odd_ram.hex" works.

set required_hex_files [list \
    sys_odd_ram.hex \
    sys_even_ram.hex \
    par1_odd_ram.hex \
    par1_even_ram.hex \
    sys_ilv_odd_ram.hex \
    sys_ilv_even_ram.hex \
    par2_odd_ram.hex \
    par2_even_ram.hex \
    qpp_3200.hex \
]

set run_dir [file normalize [pwd]]
set candidates {}

if {[llength [get_projects -quiet]] > 0} {
    set project_dir [file normalize [get_property DIRECTORY [current_project]]]
    set project_name [get_property NAME [current_project]]
    lappend candidates \
        [file join $project_dir data] \
        [file join $project_dir ${project_name}.srcs sources_1 imports data]
}

if {[info script] ne ""} {
    lappend candidates [file normalize [file join [file dirname [info script]] .. data]]
}

set data_dir ""
foreach candidate $candidates {
    set ok 1
    foreach f $required_hex_files {
        if {![file exists [file join $candidate $f]]} {
            set ok 0
            break
        }
    }
    if {$ok} {
        set data_dir [file normalize $candidate]
        break
    }
}

if {$data_dir eq ""} {
    puts "Checked these candidate data directories:"
    foreach candidate $candidates {
        puts "  $candidate"
    }
    error {Could not find a data directory containing all required $readmemh files}
}

file mkdir [file join $run_dir data]
foreach f $required_hex_files {
    file copy -force [file join $data_dir $f] [file join $run_dir $f]
    file copy -force [file join $data_dir $f] [file join $run_dir data $f]
}

puts {Copied $readmemh files from:}
puts "  $data_dir"
puts "to synthesis run directory:"
puts "  $run_dir"
puts "and:"
puts "  [file join $run_dir data]"
