`timescale 1ns/1ps
// ============================================================================
// clk_gen.v - system clock generator: 27 MHz onboard oscillator -> 40.5 MHz
//
// Why: the synthesis engine is time-multiplexed, one clock cycle per pipeline
// step. With 32 voices and the 8-harmonic additive timbre the worst-case
// sample costs 32 x 14 + 5 = 453 cycles, which does NOT fit in the 384 cycles
// of a 70.3125 kHz sample at 27 MHz. At 40.5 MHz a sample period is 576
// cycles, so 32 voices (256 simultaneous oscillators) fit with ~21 % headroom.
// The audio sample rate itself is UNCHANGED (70.3125 kHz = 40.5e6 / 576), so
// every sample-rate-derived constant (vibrato LFO increment, Goertzel
// coefficients, echo tap depths) stays exactly as it was.
//
// 40.5 MHz rather than 54 MHz on purpose: post-route STA reports
// Fmax = 52.1 MHz on the current design, so a 54 MHz core would fail timing.
// (P&R passing is NOT timing passing - read impl/pnr/project.tr section 2.3.)
// 40.5 MHz leaves ~22 % slack against the measured Fmax.
//
// rPLL frequency formulas (Gowin UG286, Clock User Guide):
//     fCLKOUT = fCLKIN * (FBDIV_SEL + 1) / (IDIV_SEL + 1)
//     fVCO    = fCLKOUT * ODIV_SEL          (ODIV_SEL in {2,4,8,16,32,...128})
//     fPFD    = fCLKIN  / (IDIV_SEL + 1)
//
//   fCLKOUT = 27 * 3 / 2      = 40.5 MHz  <- system clock
//   fVCO    = 40.5 * 16       = 648 MHz   <- must be 500..1250 MHz on GW2A-18
//   fPFD    = 27 / 2          = 13.5 MHz
//
// Two traps, both already hit the hard way once:
//   * ODIV_SEL must be 16. With ODIV_SEL = 8 the VCO would be 324 MHz, below
//     the 500 MHz floor - Gowin reports "WARN (PA1019) Invalid VCO frequency"
//     and silently discards the PLL, so the core clock collapses to 0.
//   * IDIV_SEL must be 1. With IDIV_SEL = 0 the only integer (FBDIV+1) that
//     lands on a legal VCO gives 27 MHz (x1) or 54 MHz (x2); 54 MHz is
//     unachievable at the current Fmax, so 40.5 MHz needs the /2.
//
// 'lock' gates the system reset so nothing runs on an unstable clock.
// ============================================================================
`default_nettype none

module clk_gen (
    input  wire clk27,      // H11 onboard oscillator
    output wire clk40,      // system clock (40.5 MHz)
    output wire lock        // 1 = PLL locked
);

    wire clkoutp_o, clkoutd_o, clkoutd3_o;
    wire gnd;
    assign gnd = 1'b0;

    rPLL rpll_inst (
        .CLKOUT   (clk40),
        .LOCK     (lock),
        .CLKOUTP  (clkoutp_o),
        .CLKOUTD  (clkoutd_o),
        .CLKOUTD3 (clkoutd3_o),
        .RESET    (gnd),
        .RESET_P  (gnd),
        .CLKIN    (clk27),
        .CLKFB    (gnd),
        .FBDSEL   ({gnd,gnd,gnd,gnd,gnd,gnd}),
        .IDSEL    ({gnd,gnd,gnd,gnd,gnd,gnd}),
        .ODSEL    ({gnd,gnd,gnd,gnd,gnd,gnd}),
        .PSDA     ({gnd,gnd,gnd,gnd}),
        .DUTYDA   ({gnd,gnd,gnd,gnd}),
        .FDLY     ({gnd,gnd,gnd,gnd})
    );

    defparam rpll_inst.FCLKIN        = "27";
    defparam rpll_inst.DYN_IDIV_SEL  = "false";
    defparam rpll_inst.IDIV_SEL      = 1;      // /2 -> fPFD = 13.5 MHz
    defparam rpll_inst.DYN_FBDIV_SEL = "false";
    defparam rpll_inst.FBDIV_SEL     = 2;      // x3 -> fVCO = 13.5 * 3 = 40.5
    defparam rpll_inst.DYN_ODIV_SEL  = "false";
    defparam rpll_inst.ODIV_SEL      = 16;     // VCO = 40.5 * 16 = 648 MHz
    defparam rpll_inst.PSDA_SEL      = "0000";
    defparam rpll_inst.DYN_DA_EN     = "true";
    defparam rpll_inst.DUTYDA_SEL    = "1000";
    defparam rpll_inst.CLKOUT_FT_DIR = 1'b1;
    defparam rpll_inst.CLKOUTP_FT_DIR= 1'b1;
    defparam rpll_inst.CLKOUT_DLY_STEP = 0;
    defparam rpll_inst.CLKOUTP_DLY_STEP = 0;
    defparam rpll_inst.CLKFB_SEL     = "internal";
    defparam rpll_inst.CLKOUT_BYPASS = "false";
    defparam rpll_inst.CLKOUTP_BYPASS= "false";
    defparam rpll_inst.CLKOUTD_BYPASS= "false";
    defparam rpll_inst.DYN_SDIV_SEL  = 2;
    defparam rpll_inst.CLKOUTD_SRC   = "CLKOUT";
    defparam rpll_inst.CLKOUTD3_SRC  = "CLKOUT";
    defparam rpll_inst.DEVICE        = "GW2A-18C";

endmodule
`default_nettype wire
