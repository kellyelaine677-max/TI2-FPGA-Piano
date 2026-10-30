set_device -name GW2A-18 -device_version C GW2A-LV18PG256C8/I7
set_option -top_module top_synth
set_option -verilog_std v2001
foreach file [glob -directory src -- *.v] { add_file $file }
foreach file [glob -directory constraints -- *.cst] { add_file $file }
run syn
