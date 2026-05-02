# Reconfigure the Vivado project for the small output-capture ILA.
# Run from Vivado Tcl Console or in batch:
#   vivado -mode batch -source scripts/configure_min_ila_project.tcl

set project_path "C:/VAMSHI/fpga_bits/fpga_bits.xpr"

if {[llength [get_projects -quiet]] == 0} {
    open_project $project_path
}

set ip [get_ips -quiet ila_0]
if {[llength $ip] != 1} {
    error "Expected exactly one IP named ila_0, found [llength $ip]"
}

set_property -dict [list \
    CONFIG.C_NUM_OF_PROBES {1} \
    CONFIG.C_DATA_DEPTH {1024} \
    CONFIG.C_PROBE0_WIDTH {5} \
    CONFIG.C_PROBE0_MU_CNT {1} \
] $ip

set fs [get_filesets sources_1]
set defs [get_property verilog_define $fs]
set new_defs {}
foreach d $defs {
    if {$d ne "USE_ILA_IP" && $d ne "USE_MARK_DEBUG"} {
        lappend new_defs $d
    }
}
if {[lsearch -exact $new_defs "USE_MIN_ILA_IP"] < 0} {
    lappend new_defs "USE_MIN_ILA_IP"
}
set_property verilog_define $new_defs $fs

set xci [get_files -quiet */ila_0.xci]
if {[llength $xci] == 0} {
    set xci [get_files -quiet *ila_0.xci]
}
if {[llength $xci] == 0} {
    error "Could not find ila_0.xci in the project fileset"
}

generate_target all $xci
export_ip_user_files -of_objects $xci -no_script -sync -force -quiet
update_compile_order -fileset sources_1

puts "ila_0 configured:"
puts "  probes      = [get_property CONFIG.C_NUM_OF_PROBES $ip]"
puts "  depth       = [get_property CONFIG.C_DATA_DEPTH $ip]"
puts "  probe0 width= [get_property CONFIG.C_PROBE0_WIDTH $ip]"
puts "  defines     = [get_property verilog_define $fs]"
