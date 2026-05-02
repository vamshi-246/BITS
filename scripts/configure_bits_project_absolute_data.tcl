# Configure C:/VAMSHI/BITS to use absolute $readmemh paths.
#
# This avoids Vivado run-directory ambiguity. The RTL uses TURBO_DATA_DIR to
# build all INIT_FILE/QPP_LUT_FILE strings.

set project_path "C:/VAMSHI/BITS/BITS.xpr"
set data_dir "C:/VAMSHI/BITS/data/"

if {[llength [get_projects -quiet]] == 0} {
    open_project $project_path
}

if {![file isdirectory $data_dir]} {
    error "Missing data directory: $data_dir"
}

set fs [get_filesets sources_1]
set defs [get_property verilog_define $fs]
set new_defs {}

foreach d $defs {
    if {$d eq "USE_ILA_IP"} {
        continue
    }
    if {$d eq "USE_MARK_DEBUG"} {
        continue
    }
    if {[string match "TURBO_DATA_DIR=*" $d]} {
        continue
    }
    lappend new_defs $d
}

if {[lsearch -exact $new_defs "USE_MIN_ILA_IP"] < 0} {
    lappend new_defs "USE_MIN_ILA_IP"
}
lappend new_defs "TURBO_DATA_DIR=\"${data_dir}\""

set_property verilog_define $new_defs $fs

set ip [get_ips -quiet ila_0]
if {[llength $ip] == 1} {
    set_property -dict [list \
        CONFIG.C_NUM_OF_PROBES {1} \
        CONFIG.C_DATA_DEPTH {1024} \
        CONFIG.C_PROBE0_WIDTH {5} \
        CONFIG.C_PROBE0_MU_CNT {1} \
    ] $ip

    set xci [get_files -quiet */ila_0.xci]
    if {[llength $xci] == 0} {
        set xci [get_files -quiet *ila_0.xci]
    }
    if {[llength $xci] > 0} {
        generate_target all $xci
        export_ip_user_files -of_objects $xci -no_script -sync -force -quiet
    }
}

set synth_run [get_runs synth_1]
set hook_script "C:/VAMSHI/IIT Mandi Academic Folder/IITM 6th Sem/DVAD/BITS_LTE_Parallel_Turbo_Decoder/scripts/copy_readmem_data_to_run.tcl"
if {[file exists $hook_script]} {
    set_property STEPS.SYNTH_DESIGN.TCL.PRE $hook_script $synth_run
}

update_compile_order -fileset sources_1

puts "Configured absolute data directory:"
puts "  $data_dir"
puts "Verilog defines:"
puts "  [get_property verilog_define $fs]"
