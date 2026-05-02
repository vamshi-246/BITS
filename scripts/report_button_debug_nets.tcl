open_checkpoint turbo_decoder_button_bringup_synth.dcp

set dbg_nets [get_nets -hier -filter {MARK_DEBUG == TRUE}]
puts "MARK_DEBUG net count: [llength $dbg_nets]"

foreach net [lsort [get_property NAME $dbg_nets]] {
    puts $net
}

close_design
