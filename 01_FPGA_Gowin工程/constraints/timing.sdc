// Timing constraints for the realtime synthesizer.
//
// Two clocks matter and BOTH must be constrained, otherwise the reported
// Fmax is meaningless:
//   1. clk27  - the 27 MHz onboard oscillator (H11) feeding the rPLL input
//   2. clk40  - the rPLL output that actually clocks the whole design
//               (clk_gen.v: IDIV=1, FBDIV=2, ODIV=16 -> 40.5 MHz)
//
// Constraining only the input port is the trap: Gowin then reports a single
// "clk27" domain, the number looks fine, and the real 40.5 MHz domain is
// never analysed. The design appears to pass while it is actually running
// out of time on every path.
//
// 40.5 MHz period = 24.691 ns.
//   Measured post-route Fmax on this design: 46 ~ 55 MHz depending on the
//   P&R run (Gowin's placer is seeded randomly - do not read a single Fmax
//   as an absolute). Worst case observed 46.1 MHz, i.e. ~14 % slack over the
//   40.501 MHz constraint. If a run reports below ~44 MHz, treat it as a
//   placement outlier and re-run rather than as a regression.

create_clock -name clk27 -period 37.037 -waveform {0.000 18.518} [get_ports {clk27}]

// The generated clock. -add allows it to coexist with the input clock, and
// the period is what Gowin checks the internal logic against. NOTE: the whole
// command must stay on ONE line - Gowin's SDC parser does not accept a
// backslash continuation and fails with TA2000/TA2003.
create_clock -name clk40 -period 24.691 -waveform {0.000 12.346} -add [get_pins {u_clk/rpll_inst/CLKOUT}]
