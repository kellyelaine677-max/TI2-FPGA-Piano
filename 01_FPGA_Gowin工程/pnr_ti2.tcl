set_device -name GW2A-18 -device_version C GW2A-LV18PG256C8/I7
set_option -top_module top_synth
set_option -verilog_std v2001
set_option -use_mspi_as_gpio 1
set_option -use_sspi_as_gpio 1
set_option -use_ready_as_gpio 1
set_option -use_done_as_gpio 1
foreach file [glob -directory src -- *.v] { add_file $file }
foreach file [glob -directory constraints -- *.cst] { add_file $file }
foreach file [glob -directory constraints -- *.sdc] { add_file $file }
run pnr

# ---------------------------------------------------------------------------
# Mirror the bitstream to the name the Gowin IDE programmer actually uses.
#
# gowin_ti2.gprj.user points at impl/pnr/gowin_ti2.fs, while "run pnr" writes
# impl/pnr/project.fs. Without this copy every CLI build is invisible to the
# IDE, and programming from the IDE quietly re-flashes whatever gowin_ti2.fs
# held last - which sat three days stale while a dozen fixes went into
# project.fs. The symptom is "I rebuilt but nothing changed".
# gowin_ti2.fs is delivered read-only after a build, hence the delete first.
# ---------------------------------------------------------------------------
catch { file delete -force impl/pnr/gowin_ti2.fs }
if {[catch { file copy -force impl/pnr/project.fs impl/pnr/gowin_ti2.fs } err]} {
    puts "WARNING: could not refresh impl/pnr/gowin_ti2.fs: $err"
} else {
    puts "bitstream mirrored to impl/pnr/gowin_ti2.fs (IDE output name)"
}
