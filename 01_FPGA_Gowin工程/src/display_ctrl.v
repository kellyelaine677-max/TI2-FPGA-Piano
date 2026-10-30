`timescale 1ns/1ps
// ============================================================================
// display_ctrl.v - visualization engine
//                  (scope + spectrum bars + status text panel)
//
// LCD layout (240x320 RGB332 framebuffer):
//   y   0..175 : scope area, two modes (scope_mode):
//                  0 = Lissajous X-Y  (3x3 block per sample, bright core)
//                  1 = waveform Y-T   (triggered, 240 samples wide)
//   y 176..183 : separator (never written -> black)
//   y 184..279 : 12 spectrum bars (20 px wide each), 96 rows tall
//   y 280..319 : status panel, two lines of variable-width text:
//                16x16 CJK glyphs (font_cn.v, codes 0x80+) and 2x-scaled
//                5x7 ASCII (codes 0x20..0x7E)
//                line0 "音色:<名> 八度:<n> <李萨如|波形> <指引|频谱>"
//                line1 "<参数名>:<vvv> 声部:<nn> <音量|曲目>:<nnn>"
//                (in song-guide mode the tail is "曲目:nn" instead)
//
//                The parameter is spelled out (起音/衰减/释音/延音/调频/
//                颤音/回声/湿度/滤波/滑音) - "参数0" told nobody anything.
//                The tail used to repeat pval[6] as "回声:nnn", which doubled
//                the parameter line whenever param 6 was selected; it now
//                shows the live output level 音量:nnn instead.
//
// The audio ring dly[0:1023] is filled in BOTH scope modes - the waveform
// reads it, the Lissajous reads it for the Y axis. Lissajous persistence:
// 1024-deep point ring; each new audio sample ERASES the 3x3 block drawn
// 1024 samples ago and DRAWS a new 3x3 block (18 fb writes per sample).
// The waveform is a full redraw of the scope box from the ring: a rising
// zero-crossing trigger scan (240 cycles) makes the trace stand still instead
// of sliding. Spectrum redraw is a full sweep of the bar area every Goertzel
// frame (~29 ms). The status panel is re-rendered every ~100 ms.
// All writers share the single framebuffer write port; priority is
// Lissajous > spectrum > waveform > status panel.
//
// WS2812 strip (12 px), two modes (led_mode):
//   0 guidance : 1 LED per chromatic slot (12 LEDs = 12 slots); bright color
//                when the note is sounding (touch or keypad), **neutral dim
//                gray** otherwise. 引导模式下"第 i 颗亮 = 该弹第 i 个键"。
//                待机色特意不带色相（见 hue24 的注释）—— 带色相的话按下色
//                和待机色会撞成同一个色系，只剩亮度差，看不出按没按下。
//   1 spectrum : 1 LED per bin, 12 bins, green->red gradient by magnitude
//                (silence falls back to the same neutral gray)
// ============================================================================
`default_nettype none

module display_ctrl (
    input  wire               clk,
    input  wire               rstn,
    // audio stream
    input  wire               sample_valid,
    input  wire signed [15:0] sample_in,
    // spectrum
    input  wire [95:0]        mag_bar,  // 12 x 8-bit bars FLAT: bin i = [i*8 +: 8]
    input  wire               spec_stb,
    // which chromatic notes are sounding (touch | keypad | song guide), slot 0..11
    input  wire [11:0]        note_active,
    input  wire               led_mode,      // 0 guidance, 1 spectrum
    input  wire               scope_mode,    // 0 Lissajous X-Y, 1 waveform Y-T
    input  wire               song_mode,     // 1 offline LED song guide running
    input  wire [7:0]         song_prog,     // song note index 0..41
    // ---------------- status panel inputs ----------------
    input  wire [2:0]         timbre,        // 0 SIN 1 SQR 2 SAW 3 TRI 4 FM 5 ADD
                                             // 6 HAR 7 PNO
    input  wire [1:0]         oct_shift,     // 0..3  -> displayed as OCT:1..4
    input  wire [3:0]         param_id,      // 0..9  currently edited parameter
    input  wire [79:0]        pvals,         // {p9,p8,p7,p6,p5,p4,p3,p2,p1,p0}
    input  wire [5:0]         active_cnt,    // sounding voice count 0..32
    // ---------------- MPR121 diagnostic readout ------------------------
    // Six LEDs cannot show whether the electrode data is 60 or 960, whether
    // the difference is positive or wrapped, or which electrodes the part
    // thinks failed - and "big difference but no touch" is exactly the case
    // that needs all three. Printed on status line 1 as
    //     E1023 B255 D+15 O000
    //   E = electrode 0 filtered data (10 bit), B = its baseline (>>2),
    //   D = baseline - filtered (>>2, signed), O = out-of-range flags (hex).
    input  wire [9:0]         diag_e0,
    input  wire [7:0]         diag_bl,
    input  wire [7:0]         diag_df,
    input  wire               diag_neg,
    input  wire [11:0]        diag_or,
    // Raw PCF8591 ch0 reading, 0..255, and "the ADC answered" flag. See the
    // note at the line1 text block: this drives the readout that lets you
    // see the pressure without holding a note.
    input  wire [7:0]         press8,
    input  wire               press_ok,
    // framebuffer write port (into st7789_drv)
    output reg                fb_we,
    output reg  [16:0]        fb_addr,
    output reg  [7:0]         fb_color,
    // ws2812 pixel RAM write port
    output reg                ws_we,
    output reg  [5:0]         ws_addr,
    output reg  [23:0]        ws_color,      // GRB
    // ws frame sync
    input  wire               ws_frame_start
);

    // ------------------------------------------------------------------
    // palette (RGB332)
    // ------------------------------------------------------------------
    localparam [7:0] COL_TRACE = 8'h1C;   // green halo
    localparam [7:0] COL_CORE  = 8'hFF;   // white core -> thick, obvious trace
    localparam [7:0] COL_BG    = 8'h00;   // black
    localparam [7:0] COL_PANEL = 8'h02;   // status panel background (dark blue)
    localparam [7:0] COL_TX1   = 8'hFF;   // status line 0 text (white)
    localparam [7:0] COL_TX2   = 8'hFC;   // status line 1 text (yellow)

    // pixel address = y*240 + x  (y*256 - y*16 + x), y up to 319 -> 9 bits
    function [16:0] pixaddr(input [8:0] yy, input [7:0] xx);
        pixaddr = {yy, 8'b0} - {yy, 4'b0} + {9'd0, xx};
    endfunction

    // 3x3 block around (cxx,cyy): n = 0..8 raster order.
    // Offsets are non-negative so no signed math is needed; the result is
    // clamped into the scope box (x 0..239, y 0..175) - at the border the
    // block simply gets narrower instead of wrapping.
    function [15:0] blkpos(input [7:0] cxx, input [7:0] cyy, input [3:0] n);
        reg [1:0] dx, dy;
        reg [9:0] px, py;
        begin
            case (n)
                4'd0: begin dx = 2'd0; dy = 2'd0; end
                4'd1: begin dx = 2'd1; dy = 2'd0; end
                4'd2: begin dx = 2'd2; dy = 2'd0; end
                4'd3: begin dx = 2'd0; dy = 2'd1; end
                4'd4: begin dx = 2'd1; dy = 2'd1; end
                4'd5: begin dx = 2'd2; dy = 2'd1; end
                4'd6: begin dx = 2'd0; dy = 2'd2; end
                4'd7: begin dx = 2'd1; dy = 2'd2; end
                default: begin dx = 2'd2; dy = 2'd2; end
            endcase
            px = {2'b00, cxx} + {8'b00000000, dx};
            py = {2'b00, cyy} + {8'b00000000, dy};
            if (px > 10'd239) px = 10'd239;
            if (py > 10'd175) py = 10'd175;
            blkpos = {py[7:0], px[7:0]};
        end
    endfunction

    // ------------------------------------------------------------------
    // Lissajous rings
    // ------------------------------------------------------------------
    reg signed [15:0] dly  [0:1023];        // delayed audio
    reg        [15:0] pts  [0:1023];        // {x[7:0], y[7:0]} drawn points
    reg         [9:0] dptr;

    // x: centre 120, gain 2.0 (full scale -> +-254, clamps to 0..239)
    function [7:0] scalex(input signed [15:0] s);
        reg signed [16:0] v;
        begin
            v = 17'sd120 + (s >>> 7);
            scalex = (v < 17'sd0) ? 8'd0 : (v > 17'sd239) ? 8'd239 : v[7:0];
        end
    endfunction
    // y: centre 88, gain 2.0 * 11/16 = 1.375 (fits 0..175 at normal levels)
    function [7:0] scaley(input signed [15:0] s);
        reg signed [21:0] m;
        reg signed [21:0] v;
        begin
            m = (s >>> 7) * 22'sd11;
            v = 22'sd88 + (m >>> 4);
            scaley = (v < 22'sd0) ? 8'd0 : (v > 22'sd175) ? 8'd175 : v[7:0];
        end
    endfunction

    localparam [1:0] LJ_IDLE = 2'd0, LJ_PT = 2'd1, LJ_ERASE = 2'd2,
                     LJ_DRAW = 2'd3;
    reg [1:0]  lst;
    reg [15:0] new_pt;
    reg [3:0]  ljn;                 // 0..8 index inside the 3x3 block

    // The point that is about to be erased, fetched one cycle early.
    // Indexing pts[dptr] directly inside the erase state put a 1024:1 read
    // mux + blkpos + pixaddr in front of fb_addr, and that chain was the
    // 40.7 MHz critical path. LJ_IDLE loads the coordinates into ept_q
    // instead; the erase states then only run blkpos + pixaddr.
    reg  [15:0] ept_q;    // {x, y} of the previous point, latched in LJ_IDLE

    // The delayed sample and the live sample, registered by LJ_IDLE and
    // consumed by LJ_PT. scaley() multiplies by 11, so feeding it straight
    // from the 1024:1 dly[] read mux made
    //   read-address mux -> read mux -> x11 multiply -> new_pt
    // the critical path. LJ_PT exists purely to break it.
    reg signed [15:0] rdd_q;    // dly[dptr - 2*lj_delay] as of LJ_IDLE
    reg signed [15:0] sin_q;    // sample_in as of LJ_IDLE

    wire [15:0] eblk = blkpos(ept_q[15:8], ept_q[7:0], ljn);
    wire [15:0] dblk = blkpos(new_pt[15:8],    new_pt[7:0],    ljn);

    // ------------------------------------------------------------------
    // Lissajous Y delay: a QUARTER PERIOD of whatever is actually sounding
    //
    // Y used to be dly[dptr]. dptr advances two slots per sample (see the
    // stride note at rd_addr), so that is 1024 slots = 512 SAMPLES =
    // 7.28 ms of delay. The phase that delay represents depends on the pitch:
    //   cycles of delay = f * 0.0072797
    //   D4 293.7 Hz -> 2.14 cycles ->  50 deg  a thin ellipse
    //   F4 349.2 Hz -> 2.54 cycles -> 194 deg  a nearly straight line
    //   C2  65.4 Hz -> 0.48 cycles -> 172 deg  a nearly straight line
    // so certain keys collapsed the figure into a thin diagonal that reads as
    // "the trace is faint / hardly there". Nothing was wrong with the beam -
    // the X and Y inputs were simply almost in phase.
    //
    // Fix: pick the dominant Goertzel bin and delay by a quarter of ITS
    // period, which is 90 deg for any pitch. Quarter period in samples =
    // 70312.5 / (4 * f), precomputed for the 12 bins (110 ... 5280 Hz).
    // ------------------------------------------------------------------
    reg [9:0]  lj_delay = 10'd160;      // default: 110 Hz quarter period
    reg [3:0]  dom_bin  = 4'd0;
    reg [7:0]  dom_mag  = 8'd0;

    // ------------------------------------------------------------------
    // Dominant-bin argmax - PIPELINED.
    //
    // The previous version chained 11 magnitude comparators with blocking
    // assignments inside one clocked block. That is an ~11 * 3 = 45 level
    // combinational cone running from goertzel_bank's mag_bar straight into
    // the CE of dom_mag / lj_delay, and it is the ONLY setup violation in
    // this design: all 25 worst setup paths of every build start at
    // u_spec/mag_bar. The pre-piano baseline fails 40.5 MHz on it too
    // (-10.938 ns, Fmax 28.067 MHz), so the piano did not introduce it - it
    // only made the routing around an already congested cone more expensive.
    //
    // A balanced tree does the same 11 comparisons in 3 levels; cutting it
    // with registers keeps the longest cone at ~9 levels.
    // ------------------------------------------------------------------

    // lane = {bin index, magnitude}. Strict '>' means a tie keeps the LOWER
    // index, which is exactly what the old serial loop did.
    function [11:0] amax2(input [11:0] a, input [11:0] bb);
        begin
            amax2 = (bb[7:0] > a[7:0]) ? bb : a;
        end
    endfunction

    reg [11:0] w0, w1, w2, w3, w4, w5;   // stage 1: 12 bins -> 6 winners
    reg        spec_stb_d, spec_stb_d2;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            w0 <= 12'd0; w1 <= 12'd0; w2 <= 12'd0;
            w3 <= 12'd0; w4 <= 12'd0; w5 <= 12'd0;
            spec_stb_d <= 1'b0; spec_stb_d2 <= 1'b0;
        end else begin
            spec_stb_d  <= spec_stb;
            spec_stb_d2 <= spec_stb_d;
            if (spec_stb) begin
                w0 <= amax2({4'd0,  mag_bar[7:0]  }, {4'd1,  mag_bar[15:8] });
                w1 <= amax2({4'd2,  mag_bar[23:16]}, {4'd3,  mag_bar[31:24]});
                w2 <= amax2({4'd4,  mag_bar[39:32]}, {4'd5,  mag_bar[47:40]});
                w3 <= amax2({4'd6,  mag_bar[55:48]}, {4'd7,  mag_bar[63:56]});
                w4 <= amax2({4'd8,  mag_bar[71:64]}, {4'd9,  mag_bar[79:72]});
                w5 <= amax2({4'd10, mag_bar[87:80]}, {4'd11, mag_bar[95:88]});
            end
        end
    end

    // stage 2, combinational: 6 winners -> 1 in 3 comparator levels
    wire [11:0] am_m0  = amax2(w0, w1);
    wire [11:0] am_m1  = amax2(w2, w3);
    wire [11:0] am_m2  = amax2(w4, w5);
    wire [11:0] am_n0  = amax2(am_m0, am_m1);
    wire [11:0] am_win = amax2(am_n0, am_m2);

    reg [3:0] cand_r;
    reg [7:0] cmag_r;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            cand_r <= 4'd0; cmag_r <= 8'd0;
        end else if (spec_stb_d) begin
            cand_r <= am_win[11:8];
            cmag_r <= am_win[7:0];
        end
    end

    // stage 3: 8 counts of hysteresis so a chord does not flip the figure
    // between two bins every frame
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            dom_bin <= 4'd0; dom_mag <= 8'd0; lj_delay <= 10'd160;
        end else if (spec_stb_d2) begin
            if (cmag_r > 8'd8 && cmag_r > dom_mag + 8'd8) begin
                dom_bin  <= cand_r;
                dom_mag  <= cmag_r;
                case (cand_r)
                    4'd0:  lj_delay <= 10'd160;   // 110 Hz
                    4'd1:  lj_delay <= 10'd107;   // 165
                    4'd2:  lj_delay <= 10'd80;    // 220
                    4'd3:  lj_delay <= 10'd53;    // 330
                    4'd4:  lj_delay <= 10'd40;    // 440
                    4'd5:  lj_delay <= 10'd27;    // 660
                    4'd6:  lj_delay <= 10'd20;    // 880
                    4'd7:  lj_delay <= 10'd13;    // 1320
                    4'd8:  lj_delay <= 10'd10;    // 1760
                    4'd9:  lj_delay <= 10'd7;     // 2640
                    4'd10: lj_delay <= 10'd5;     // 3520
                    default: lj_delay <= 10'd3;   // 5280
                endcase
            end else if (cmag_r < 8'd4) begin
                dom_mag <= 8'd0;                  // silence: decay the latch
            end
        end
    end

    // ------------------------------------------------------------------
    // spectrum bar sweep
    // ------------------------------------------------------------------
    localparam [1:0] SP_IDLE = 2'd0, SP_RUN = 2'd1;
    reg [1:0]  sst;
    reg [7:0]  mag_q [0:11];
    reg [7:0]  bx;      // 0..239 (12 bars x 20 px)
    reg [7:0]  by;      // 0..95  (LCD rows 184..279)

    // bin index from x position (bar width 20)
    wire [3:0] sp_bin = (bx >= 8'd220) ? 4'd11 : (bx >= 8'd200) ? 4'd10 :
                        (bx >= 8'd180) ? 4'd9  : (bx >= 8'd160) ? 4'd8  :
                        (bx >= 8'd140) ? 4'd7  : (bx >= 8'd120) ? 4'd6  :
                        (bx >= 8'd100) ? 4'd5  : (bx >= 8'd80)  ? 4'd4  :
                        (bx >= 8'd60)  ? 4'd3  : (bx >= 8'd40)  ? 4'd2  :
                        (bx >= 8'd20)  ? 4'd1  : 4'd0;

    // spectrum row: LCD y = 279 - by   (bottom of the bar area is y=279)
    wire [8:0] spy = 9'd279 - {1'b0, by};

    // bar height rows (0..95): mag 直接当行数，超过 95 截顶。
    //
    // mag 不再放大：映射改为 -38 dBFS 起步后 mag 很容易就到 255，若再乘 1.5
    // 则 mag>=64 即满格，自测时 12 根柱子会糊成一道墙、看不出高低。
    // 去掉放大后自测实测 mag 54..120 -> 54..95 行，层次才拉得开。
    wire [8:0] bar_raw = {1'b0, mag_q[sp_bin]};
    wire [7:0] bar_h   = (bar_raw > 9'd95) ? 8'd95 : bar_raw[7:0];

    function [7:0] bar_color(input [3:0] b);
        case (b)
            4'd0:  bar_color = 8'hE0;  4'd1:  bar_color = 8'hE4;
            4'd2:  bar_color = 8'hE8;  4'd3:  bar_color = 8'hEC;
            4'd4:  bar_color = 8'hB4;  4'd5:  bar_color = 8'h94;
            4'd6:  bar_color = 8'h54;  4'd7:  bar_color = 8'h14;
            4'd8:  bar_color = 8'h1C;  4'd9:  bar_color = 8'h18;
            4'd10: bar_color = 8'h34;  default: bar_color = 8'h30;
        endcase
    endfunction

    // ------------------------------------------------------------------
    // waveform (Y-T) scope: full redraw of the scope box from the audio ring
    // ------------------------------------------------------------------
    localparam [2:0] SC_IDLE = 3'd0, SC_TRIG = 3'd1, SC_FETCH = 3'd2,
                     SC_PIX = 3'd3;
    reg [2:0]  scst;
    reg [7:0]  scx;        // column 0..239
    reg [7:0]  scy;        // row 0..175 inside the scope box
    reg [7:0]  sct;        // trigger scan index 0..239
    reg [7:0]  sc_trig;    // detected trigger offset inside the window
    reg        sc_found;
    reg signed [15:0] sc_prev;
    reg [9:0]  sc_base;    // ring index of the first drawn sample
    reg [9:0]  sc_win;     // ring index of the window start (latched, so a
                           // sample arriving mid-scan cannot move it)
    reg [21:0] sc_tmr;
    reg        scope_mode_q;   // previous scope_mode, for the 1 -> 0 wipe
    reg        sc_clr;         // 1 = this sweep paints background only

    // Window for the trigger scan. It must sit BEHIND the write pointer with
    // room for the whole 240-column trace, because dly[dptr] is the OLDEST
    // sample (the next write lands there) - anything at dptr, dptr+1, ... is
    // overwritten within a few hundred samples.
    //   old: dptr - 240, trigger offset up to +239, trace 240 wide
    //        -> reads up to dptr + 238, i.e. 238 samples AHEAD of the write
    //           pointer. A full redraw takes ~150k clocks (~260 samples), so
    //           the right-hand part of the trace was overwritten WHILE it was
    //           being drawn: the picture tore. That is the "pressing a key
    //           distorts the waveform" report.
    //   now: dptr - 1000. The scan needs 480 slots and the trace another
    //        480, so the window has to be 960 deep; the last slot touched is
    //        dptr - 44, which the write pointer only reaches after 980 more
    //        slots (~490 samples, ~280k clocks) - far past the ~150k-clock
    //        redraw, so nothing is overwritten while it is being drawn.
    wire [9:0] sc_start = (dptr - 10'd1000) & 10'b11_1111_1110;

    // ONE shared read port on dly[]: Gowin only infers shadow-RAM (Ssram) for
    // a single async read address. Giving the scope its own dly[...] read
    // expression added extra ports and the 1024x16 ring blew up into 16384
    // DFFs (IF0008), so every reader goes through this mux instead.
    // EVERY reader must step by 2.
    //
    // dptr advances TWICE per sample (once in LJ_IDLE, once in LJ_DRAW - that
    // stride is what keeps the pts[] erase slot and the write slot in step,
    // see the note below), but only ONE dly[] entry is written per sample.
    // So exactly every other ring slot has ever been written: in simulation
    // the rest read as X, in hardware they read as the power-up 0.
    // Reading consecutive slots therefore gives you a real sample on one
    // column and a flat zero on the next - on screen that is a comb, not a
    // waveform, and it is what "pressing a key distorts the trace" really was.
    // Stride 2 lands on written slots only; 240 columns then cover 240 real
    // samples (480 ring slots), which is still the ~3.4 ms window.
    // ...and the base has to be forced EVEN. dptr is only even between
    // samples: LJ_IDLE adds 1 and LJ_DRAW adds the second 1, so for ~18 of
    // every 576 clocks it sits on an odd value. Latching sc_win on one of
    // those clocks would offset the whole trace onto the unwritten half and
    // draw a flat zero line instead of a waveform - intermittently, which is
    // much harder to recognise than a permanent fault.
    wire [9:0] rd_addr = (scst == SC_TRIG)  ? (sc_win  + {1'b0, sct, 1'b0}) :
                         (scst == SC_FETCH) ? (sc_base + {1'b0, scx, 1'b0}) :
                                            ((dptr - {lj_delay, 1'b0})
                                             & 10'b11_1111_1110);
    wire signed [15:0] rd_data = dly[rd_addr];

    // Vertical mapping: centre row 88, +-128 -> 0..175 (clamped).
    // In SC_FETCH this chain starts at the 1024:1 dly[] read mux, and in
    // SC_PIX it starts at a flop - splitting the two used to put the whole
    // read mux + clamp + |diff| + 3-way colour mux in one cycle, which was
    // the 40 MHz critical path. fys_q is the registered SC_FETCH result.
    wire signed [16:0] sc_v  = 17'sd88 - (rd_data >>> 8);
    wire [7:0]  sc_ys  = (sc_v < 17'sd0)   ? 8'd0   :
                         (sc_v > 17'sd175) ? 8'd175 : sc_v[7:0];
    reg  [7:0]  sc_ys_q;                       // latched in SC_FETCH
    reg  [7:0]  fxs, fys;                      // the pixel SC_FETCH sampled
    wire [7:0]  sc_d   = (fys > sc_ys_q) ? (fys - sc_ys_q)
                                         : (sc_ys_q - fys);
    wire        sc_on  = (sc_d <= 8'd1);

    // ------------------------------------------------------------------
    // 5x7 font: 35 bits = {row0,row1,...,row6}, each row 5 bits, MSB = left
    // Rendered at 2x scale -> 10x14 glyphs, 24 characters per line.
    // ------------------------------------------------------------------
    function [34:0] font35(input [7:0] ch);
        case (ch)
            8'h20: font35 = {5'b00000,5'b00000,5'b00000,5'b00000,5'b00000,5'b00000,5'b00000}; // space
            8'h2D: font35 = {5'b00000,5'b00000,5'b00000,5'b11111,5'b00000,5'b00000,5'b00000}; // -
            8'h2E: font35 = {5'b00000,5'b00000,5'b00000,5'b00000,5'b00000,5'b00100,5'b00100}; // .
            8'h2F: font35 = {5'b00001,5'b00010,5'b00010,5'b00100,5'b01000,5'b01000,5'b10000}; // /
            8'h30: font35 = {5'b01110,5'b10001,5'b10011,5'b10101,5'b11001,5'b10001,5'b01110}; // 0
            8'h31: font35 = {5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110}; // 1
            8'h32: font35 = {5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111}; // 2
            8'h33: font35 = {5'b11111,5'b00010,5'b00100,5'b00010,5'b00001,5'b10001,5'b01110}; // 3
            8'h34: font35 = {5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010}; // 4
            8'h35: font35 = {5'b11111,5'b10000,5'b11110,5'b00001,5'b00001,5'b10001,5'b01110}; // 5
            8'h36: font35 = {5'b00110,5'b01000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110}; // 6
            8'h37: font35 = {5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000}; // 7
            8'h38: font35 = {5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110}; // 8
            8'h39: font35 = {5'b01110,5'b10001,5'b10001,5'b01111,5'b00001,5'b00010,5'b01100}; // 9
            8'h3A: font35 = {5'b00000,5'b00100,5'b00100,5'b00000,5'b00100,5'b00100,5'b00000}; // :
            8'h3D: font35 = {5'b00000,5'b00000,5'b11111,5'b00000,5'b11111,5'b00000,5'b00000}; // =
            8'h41: font35 = {5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001}; // A
            8'h42: font35 = {5'b11110,5'b10001,5'b10001,5'b11110,5'b10001,5'b10001,5'b11110}; // B
            8'h43: font35 = {5'b01110,5'b10001,5'b10000,5'b10000,5'b10000,5'b10001,5'b01110}; // C
            8'h44: font35 = {5'b11110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b11110}; // D
            8'h45: font35 = {5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b11111}; // E
            8'h46: font35 = {5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b10000}; // F
            8'h47: font35 = {5'b01110,5'b10001,5'b10000,5'b10111,5'b10001,5'b10001,5'b01110}; // G
            8'h48: font35 = {5'b10001,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001}; // H
            8'h49: font35 = {5'b01110,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110}; // I
            8'h4A: font35 = {5'b00111,5'b00010,5'b00010,5'b00010,5'b00010,5'b10010,5'b01100}; // J
            8'h4B: font35 = {5'b10001,5'b10010,5'b10100,5'b11000,5'b10100,5'b10010,5'b10001}; // K
            8'h4C: font35 = {5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b11111}; // L
            8'h4D: font35 = {5'b10001,5'b11011,5'b10101,5'b10101,5'b10001,5'b10001,5'b10001}; // M
            8'h4E: font35 = {5'b10001,5'b11001,5'b10101,5'b10011,5'b10001,5'b10001,5'b10001}; // N
            8'h4F: font35 = {5'b01110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110}; // O
            8'h50: font35 = {5'b11110,5'b10001,5'b10001,5'b11110,5'b10000,5'b10000,5'b10000}; // P
            8'h51: font35 = {5'b01110,5'b10001,5'b10001,5'b10001,5'b10101,5'b10010,5'b01101}; // Q
            8'h52: font35 = {5'b11110,5'b10001,5'b10001,5'b11110,5'b10100,5'b10010,5'b10001}; // R
            8'h53: font35 = {5'b01111,5'b10000,5'b10000,5'b01110,5'b00001,5'b00001,5'b11110}; // S
            8'h54: font35 = {5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100}; // T
            8'h55: font35 = {5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110}; // U
            8'h56: font35 = {5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01010,5'b00100}; // V
            8'h57: font35 = {5'b10001,5'b10001,5'b10001,5'b10101,5'b10101,5'b11011,5'b10001}; // W
            8'h58: font35 = {5'b10001,5'b10001,5'b01010,5'b00100,5'b01010,5'b10001,5'b10001}; // X
            8'h59: font35 = {5'b10001,5'b10001,5'b01010,5'b00100,5'b00100,5'b00100,5'b00100}; // Y
            8'h5A: font35 = {5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b10000,5'b11111}; // Z
            default: font35 = {5'b00000,5'b00000,5'b00000,5'b00000,5'b00000,5'b00000,5'b00000};
        endcase
    endfunction

    function [4:0] frow(input [34:0] p, input [2:0] r);
        case (r)
            3'd0: frow = p[34:30];
            3'd1: frow = p[29:25];
            3'd2: frow = p[24:20];
            3'd3: frow = p[19:15];
            3'd4: frow = p[14:10];
            3'd5: frow = p[ 9: 5];
            default: frow = p[ 4: 0];
        endcase
    endfunction

    // ------------------------------------------------------------------
    // status text buffers (24 chars x 2 lines)
    // ------------------------------------------------------------------
    reg [7:0] line0 [0:23];
    reg [7:0] line1 [0:23];

    // timbre names as panel codes: two CJK glyphs + one filler slot.
    // CJK codes are 8'h80+i into font_cn.v; see the table in that file.
    //   正弦  方波  锯齿  三角  调频  加法  加法8  钢琴
    // 钢/琴 are the two glyphs appended to font_cn.v for timbre 7 (8'hAB/8'hAC);
    // the table is index-ordered, so they could only be added at the end.
    wire [23:0] tmb3 = (timbre == 3'd0) ? 24'h979820 :   // 正弦
                       (timbre == 3'd1) ? 24'h998920 :   // 方波
                       (timbre == 3'd2) ? 24'h9A9B20 :   // 锯齿
                       (timbre == 3'd3) ? 24'h9C9D20 :   // 三角
                       (timbre == 3'd4) ? 24'h9E9520 :   // 调频
                       (timbre == 3'd5) ? 24'h9FA020 :   // 加法
                       (timbre == 3'd6) ? 24'h9FA038 :   // 加法8
                                          24'hABAC20;    // 钢琴

    // Parameter name: two CJK glyphs, printed as "<名>:<vvv>".
    // (The panel used to print 参数0=063; the id only meant something if you
    // had top_synth.v's table in your head.)
    //   0 起音 attack   1 衰减 decay    2 释音 release   3 延音 sustain
    //   4 调频 FM index 5 颤音 vibrato  6 回声 feedback  7 湿度 wet mix
    //   8 滤波 cutoff   9 滑音 portamento
    wire [15:0] pnm = (param_id == 4'd0) ? 16'hA180 :   // 起音
                      (param_id == 4'd1) ? 16'hA2A3 :   // 衰减
                      (param_id == 4'd2) ? 16'hA480 :   // 释音
                      (param_id == 4'd3) ? 16'hA580 :   // 延音
                      (param_id == 4'd4) ? 16'h9E95 :   // 调频
                      (param_id == 4'd5) ? 16'hA680 :   // 颤音
                      (param_id == 4'd6) ? 16'h8F8D :   // 回声
                      (param_id == 4'd7) ? 16'hA783 :   // 湿度
                      (param_id == 4'd8) ? 16'hA889 :   // 滤波
                                           16'hA980;    // 滑音

    // Live output level: peak of |sample_in| with a slow bleed, 0..255.
    // Instant attack, ~85 ms release, so the number is readable at the 100 ms
    // panel refresh instead of flickering.
    reg [15:0] lvl;
    wire [15:0] sabs = sample_in[15] ? (16'd0 - sample_in) : sample_in;
    always @(posedge clk or negedge rstn) begin
        if (!rstn)                 lvl <= 16'd0;
        else if (sample_valid) begin
            if (sabs > lvl)        lvl <= sabs;
            else if (lvl > 16'd8)  lvl <= lvl - 16'd8;
            else                   lvl <= 16'd0;
        end
    end
    wire [7:0] lvl8 = (lvl > 16'd32639) ? 8'd255 : lvl[14:7];

    wire [7:0] pcur    = (param_id == 4'd0) ? pvals[ 7: 0] :
                         (param_id == 4'd1) ? pvals[15: 8] :
                         (param_id == 4'd2) ? pvals[23:16] :
                         (param_id == 4'd3) ? pvals[31:24] :
                         (param_id == 4'd4) ? pvals[39:32] :
                         (param_id == 4'd5) ? pvals[47:40] :
                         (param_id == 4'd6) ? pvals[55:48] :
                         (param_id == 4'd7) ? pvals[63:56] :
                         (param_id == 4'd8) ? pvals[71:64] :
                                              pvals[79:72];
    wire [7:0] pd_h = 8'h30 + (pcur / 8'd100);
    wire [7:0] pd_t = 8'h30 + ((pcur / 8'd10) % 8'd10);
    wire [7:0] pd_o = 8'h30 + (pcur % 8'd10);
    // What the three trailing digits show: the raw ADC once a pressure
    // sensor has answered, otherwise the output level. Reuses the existing
    // divider chain - a mux on its input costs almost nothing, and there is
    // no free space on this panel to add a separate field.
    wire [7:0] lvl_disp = press_ok ? press8 : lvl8;
    wire [7:0] lv_h = 8'h30 + (lvl_disp / 8'd100);
    wire [7:0] lv_t = 8'h30 + ((lvl_disp / 8'd10) % 8'd10);
    wire [7:0] lv_o = 8'h30 + (lvl_disp % 8'd10);
    wire [7:0] vx_t = 8'h30 + (active_cnt / 8'd10);
    wire [7:0] vx_o = 8'h30 + (active_cnt % 8'd10);

    // ---- MPR121 diagnostic digits -------------------------------------
    // E is 10 bit, so it needs a thousands digit; B and D are 8 bit.
    wire [7:0] de_k = 8'h30 + (diag_e0 / 10'd1000);
    wire [7:0] de_h = 8'h30 + ((diag_e0 / 10'd100) % 10'd10);
    wire [7:0] de_t = 8'h30 + ((diag_e0 / 10'd10)   % 10'd10);
    wire [7:0] de_o = 8'h30 + ( diag_e0             % 10'd10);
    wire [7:0] db_h = 8'h30 + (diag_bl / 8'd100);
    wire [7:0] db_t = 8'h30 + ((diag_bl / 8'd10) % 8'd10);
    wire [7:0] db_o = 8'h30 + ( diag_bl          % 8'd10);
    wire [7:0] dd_h = 8'h30 + (diag_df / 8'd100);
    wire [7:0] dd_t = 8'h30 + ((diag_df / 8'd10) % 8'd10);
    wire [7:0] dd_o = 8'h30 + ( diag_df          % 8'd10);
    wire [7:0] dd_s = diag_neg ? 8'h2D : 8'h2B;    // '-' / '+'
    // out-of-range flags as three hex digits
    function [7:0] hexc(input [3:0] v);
        hexc = (v < 4'd10) ? (8'h30 + {4'd0, v}) : (8'h37 + {4'd0, v});
    endfunction
    wire [7:0] do_h = hexc(diag_or[11:8]);
    wire [7:0] do_t = hexc(diag_or[7:4]);
    wire [7:0] do_o = hexc(diag_or[3:0]);
    wire [7:0] sg_t = 8'h30 + (song_prog / 8'd10);
    wire [7:0] sg_o = 8'h30 + (song_prog % 8'd10);

    // ---------- pressure readout (slots 13..18 of line 1) --------------
    // Neither text line has a free slot - both run to 236 px of the 240 px
    // panel - so the raw ADC borrowing the trailing "音量:nnn" field is the
    // only way to see it. It only does that once a PCF8591 has answered, so
    // a board without one keeps the output level meter it always had.
    // 'P' is ASCII (bit 7 clear -> the 5x7 font, 10 px advance), so "P:nnn"
    // is shorter than "音量:nnn" and still ends inside the panel.
    wire [7:0] lb1 = press_ok ? 8'h50 : (song_mode ? 8'h90 : 8'h80); // P/曲/音
    wire [7:0] lb2 = press_ok ? 8'h20 : (song_mode ? 8'h91 : 8'hAA); // _/目/量
    wire [7:0] d16 = press_ok ? lv_h : (song_mode ? sg_t : lv_h);
    wire [7:0] d17 = press_ok ? lv_t : (song_mode ? sg_o : lv_t);
    wire [7:0] d18 = press_ok ? lv_o : (song_mode ? 8'h20 : lv_o);

    // status panel renderer
    //
    // Variable-width text: a slot is either a 16x16 CJK glyph (code >= 0x80,
    // advance 16 px) or the classic 5x7 ASCII at 2x = 10x14 (advance 10 px).
    // Every line is 24 slots and every slot paints (spaces included), so the
    // worst case 24x10 = 240 px still covers the panel - slots past x=240
    // are simply clipped by tvis. A line whose content ends early MUST be
    // padded with spaces to slot 23 or the tail columns keep stale pixels.
    localparam [1:0] TX_IDLE = 2'd0, TX_RUN = 2'd1;
    reg [1:0]  txst;
    reg [4:0]  tx_ci;      // character slot 0..23
    reg [3:0]  tx_cx;      // column inside the glyph box 0..15
    reg [8:0]  tx_x;       // left edge of the current glyph box, 0..~380
    reg [5:0]  tx_py;      // row inside the panel 0..39
    reg [22:0] tx_tmr;

    wire        in0  = (tx_py >= 6'd3)  && (tx_py <= 6'd18);   // line 0: 16 rows
    wire        in1  = (tx_py >= 6'd20) && (tx_py <= 6'd35);   // line 1: 16 rows
    wire        tin  = in0 || in1;
    wire [4:0]  gy   = in1 ? (tx_py - 6'd20) : (tx_py - 6'd3); // 0..15 when tin
    wire [7:0]  tch  = in1 ? line1[tx_ci] : line0[tx_ci];
    wire        iscn = tch[7];          // 1 = 16x16 CJK, 0 = 5x7 ASCII

    // ASCII occupies rows 1..14 of the 16-row box (5x7 at 2x = 10x14)
    wire        arok = (gy >= 5'd1) && (gy <= 5'd14);
    wire [4:0]  ad   = gy - 5'd1;
    wire [3:0]  arow = {1'b0, ad[3:1]};   // font row 0..6 when arok
    wire [4:0]  fb5  = frow(font35(tch), arow[2:0]);
    wire [15:0] arow16 = arok ?
        {{2{fb5[4]}}, {2{fb5[3]}}, {2{fb5[2]}}, {2{fb5[1]}}, {2{fb5[0]}}, 6'b000000}
        : 16'b0;

    wire [15:0] crow;
    font_cn u_font (.idx(tch[5:0]), .row(gy[3:0]), .dout(crow));

    wire [15:0] grow = iscn ? crow : arow16;
    wire        ton  = grow[4'd15 - tx_cx];

    wire [8:0]  ty   = 9'd280 + {3'b000, tx_py};
    wire [8:0]  tpx  = tx_x + {5'b00000, tx_cx};    // 0..~395, valid < 240
    wire        tvis = (tpx < 9'd240);
    wire [7:0]  tx8  = tpx[7:0];

    // ------------------------------------------------------------------
    // WS2812 content
    // ------------------------------------------------------------------
    localparam [1:0] WS_IDLE = 2'd0, WS_RUN = 2'd1;
    reg [1:0]  wst;
    reg [5:0]  widx;

    // 12 颗灯带：一颗灯 = 一个半音槽，一一对应。
    //
    // 灯数由原先的 60 颗改为 12 颗，直接用 widx 本身作为槽号：引导模式下
    // "第 i 颗灯亮 = 该弹第 i 个键"，比原先按 5 颗一组分档更直观。
    // 谱线模式同理：第 i 颗 = 第 i 个频谱 bin。
    wire [3:0] ws_bin = widx[3:0];

    // 待机色：**中性暗灰，不带任何色相**。
    //
    // 待机色若带色相（如暗红/暗绿/暗蓝循环），而按下色也落在同样色系里，
    // 两者只差亮度、色系相同，难以区分按键是否被按下。
    // 待机改用不带色相的中性灰后，任何按下色都会明显跳出来。
    // 亮度 0x14 是"看得出灯带活着"和"不抢按下色"之间的折中。
    function [23:0] hue24(input [5:0] i);   // neutral dim gray for idle strip
        hue24 = {8'h14, 8'h14, 8'h14};      // GRB, 无色相
    endfunction

    // 按下色：12 个槽 12 个**互不相同**的色相，红 -> 橙 -> 黄 -> 绿 ->
    // 青 -> 蓝 -> 紫，音越高越偏冷色。
    //
    // 12 颗灯与 12 槽一一对应，各槽取独立色相，无需共用颜色，
    // 相邻两个键也不会撞色。
    function [23:0] slot24(input [3:0] s);   // bright note colors (GRB)
        case (s)
            4'd0:  slot24 = {8'h00, 8'hFF, 8'h00};   // C   红
            4'd1:  slot24 = {8'h40, 8'hFF, 8'h00};   // C#  橙
            4'd2:  slot24 = {8'h80, 8'hFF, 8'h00};   // D   琥珀
            4'd3:  slot24 = {8'hC0, 8'hFF, 8'h00};   // D#  金黄
            4'd4:  slot24 = {8'hFF, 8'hC0, 8'h00};   // E   黄绿
            4'd5:  slot24 = {8'hFF, 8'h40, 8'h00};   // F   绿
            4'd6:  slot24 = {8'hFF, 8'h00, 8'h40};   // F#  青绿
            4'd7:  slot24 = {8'hFF, 8'h00, 8'hA0};   // G   碧绿
            4'd8:  slot24 = {8'hC8, 8'h00, 8'hFF};   // G#  青
            4'd9:  slot24 = {8'h80, 8'h00, 8'hFF};   // A   天蓝
            4'd10: slot24 = {8'h30, 8'h00, 8'hFF};   // A#  蓝
            default: slot24 = {8'h00, 8'h50, 8'hFF}; // B   紫
        endcase
    endfunction

    integer k;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            lst <= LJ_IDLE; new_pt <= 16'd0; dptr <= 10'd0; ljn <= 4'd0;
            ept_q <= 16'd0; rdd_q <= 16'sd0; sin_q <= 16'sd0;
            sst <= SP_IDLE; bx <= 8'd0; by <= 8'd0;
            txst <= TX_IDLE; tx_ci <= 5'd0; tx_cx <= 4'd0; tx_py <= 6'd0;
            tx_tmr <= 23'd0;
            scst <= SC_IDLE; scx <= 8'd0; scy <= 8'd0; sct <= 8'd0;
            sc_ys_q <= 8'd0; fxs <= 8'd0; fys <= 8'd0;
            sc_trig <= 8'd0; sc_found <= 1'b0; sc_prev <= 16'sd0;
            sc_base <= 10'd0; sc_tmr <= 22'd0;
            scope_mode_q <= 1'b0; sc_clr <= 1'b0;
            wst <= WS_IDLE; widx <= 6'd0;
            fb_we <= 1'b0; fb_addr <= 17'd0; fb_color <= 8'd0;
            ws_we <= 1'b0; ws_addr <= 6'd0; ws_color <= 24'd0;
            for (k = 0; k < 12; k = k + 1) mag_q[k] <= 8'd0;
`ifdef SIM_INIT
            // sim-only clear: hardware memories power up as 0, and a reset
            // write would block BRAM/S-SRAM inference (DFF count explodes)
            for (k = 0; k < 1024; k = k + 1) begin
                dly[k]  <= 16'd0;
                pts[k]  <= 16'd0;
            end
            for (k = 0; k < 24; k = k + 1) begin
                line0[k] <= 8'h20;
                line1[k] <= 8'h20;
            end
`endif
        end else begin
            fb_we <= 1'b0;
            ws_we <= 1'b0;

            // ---------------- Lissajous (priority on fb port) ----------
            case (lst)
                LJ_IDLE: if (sample_valid) begin
                    // The Lissajous uses it for the Y axis, the waveform draws
                    // straight out of it. (If the ring stopped in waveform
                    // mode the trace would freeze on the last 1024 samples.)
                    dly[dptr] <= sample_in;
                    dptr      <= dptr + 10'd1;
                    // fetch the outgoing point now so the erase states do not
                    // have to index pts[] themselves - see ept_q.
                    //
                    // NOTE the +1: dptr advances here AND again in LJ_DRAW, so
                    // by the time LJ_DRAW does `pts[dptr] <= new_pt` the
                    // pointer has already moved on. The slot that gets
                    // overwritten this sample is therefore dptr+1, and that is
                    // the slot we must erase - reading pts[dptr] here would
                    // erase the slot BEFORE it, which is one that was never
                    // written (the two strides have opposite parity), leaving
                    // every drawn point on screen forever.
                    ept_q     <= pts[dptr + 10'd1];
                    if (!scope_mode) begin
                        // latch the two samples; LJ_PT does the scaling so the
                        // dly[] read mux is not in front of the x11 multiply
                        rdd_q  <= rd_data;
                        sin_q  <= sample_in;
                        ljn    <= 4'd0;
                        lst    <= LJ_PT;
                    end
                end
                LJ_PT: begin
                    // new point: x from the live sample, y from the delayed one
                    new_pt <= {scalex(sin_q), scaley(rdd_q)};
                    lst    <= LJ_ERASE;
                end
                LJ_ERASE: begin
                    fb_we    <= 1'b1;
                    fb_addr  <= pixaddr(eblk[15:8], eblk[7:0]);
                    fb_color <= COL_BG;
                    if (ljn == 4'd8) begin ljn <= 4'd0; lst <= LJ_DRAW; end
                    else ljn <= ljn + 4'd1;
                end
                LJ_DRAW: begin
                    fb_we     <= 1'b1;
                    fb_addr   <= pixaddr(dblk[15:8], dblk[7:0]);
                    fb_color  <= (ljn == 4'd4) ? COL_CORE : COL_TRACE;
                    if (ljn == 4'd8) begin
                        pts[dptr] <= new_pt;
                        dptr      <= dptr + 10'd1;
                        lst       <= LJ_IDLE;
                    end else ljn <= ljn + 4'd1;
                end
                default: lst <= LJ_IDLE;
            endcase

            // ---------------- spectrum bars (yields to Lissajous) ------
            case (sst)
                SP_IDLE: if (spec_stb) begin
                    for (k = 0; k < 12; k = k + 1) mag_q[k] <= mag_bar[k*8 +: 8];
                    bx  <= 8'd0;
                    by  <= 8'd0;
                    sst <= SP_RUN;
                end
                SP_RUN: begin
                    if (lst == LJ_IDLE) begin
                        // pixel (x = bx, y = 279 - by)
                        fb_we   <= 1'b1;
                        fb_addr <= pixaddr(spy, bx);
                        fb_color <= (by < bar_h) ? bar_color(sp_bin) : COL_BG;
                        if (bx == 8'd239) begin
                            bx <= 8'd0;
                            if (by == 8'd95) sst <= SP_IDLE;
                            else by <= by + 8'd1;
                        end else begin
                            bx <= bx + 8'd1;
                        end
                    end
                end
                default: sst <= SP_IDLE;
            endcase

            // ---------------- waveform scope (yields to LJ and SP) ------
            case (scst)
                SC_IDLE: begin
                    // ---- leaving waveform mode: wipe the scope box ------
                    // The Lissajous only ever erases the 3x3 blocks recorded
                    // in pts[], and pts[] is not written while the Y-T scope
                    // is running. Coming back from waveform mode therefore
                    // left the old trace painted on screen with nothing
                    // scrubbing it: the residue the report describes. One
                    // background sweep fixes it; the waveform's own redraw
                    // covers the whole box anyway, so only 1 -> 0 needs it.
                    if (scope_mode_q && !scope_mode) begin
                        scope_mode_q <= 1'b0;
                        sc_clr       <= 1'b1;
                        scx          <= 8'd0;
                        scy          <= 8'd0;
                        scst         <= SC_FETCH;
                    end
                    else if (!scope_mode_q && scope_mode) begin
                        scope_mode_q <= 1'b1;
                    end
                    // ~33 ms refresh @ 40.5 MHz
                    else if (sc_tmr != 22'd1_350_000) sc_tmr <= sc_tmr + 22'd1;
                    else begin
                        sc_tmr <= 22'd0;
                        if (scope_mode) begin
                            sct      <= 8'd0;
                            sc_found <= 1'b0;
                            sc_trig  <= 8'd0;
                            sc_win   <= sc_start;
                            scst     <= SC_TRIG;
                        end
                    end
                end
                SC_TRIG: begin
                    // rising zero-crossing search -> the trace stands still
                    // instead of sliding across the screen every refresh.
                    // rd_data is dly[sc_win + sct]; sc_prev still holds
                    // dly[sc_win + sct - 1] (latched one cycle earlier), so
                    // the pair is exactly a rising-edge test. sct == 0 is
                    // skipped because sc_prev is not primed yet.
                    if (!sc_found && (sct != 8'd0) &&
                        (sc_prev < 16'sd0) && (rd_data >= 16'sd0)) begin
                        sc_found <= 1'b1;
                        sc_trig  <= sct;
                    end
                    sc_prev <= rd_data;
                    if (sct == 8'd239) begin
                        // stride 2: the trigger offset is in ring slots
                        sc_base <= sc_win + {1'b0, sc_trig, 1'b0};
                        scx     <= 8'd0;
                        scy     <= 8'd0;
                        scst    <= SC_FETCH;
                    end else sct <= sct + 8'd1;
                end
                // fetch column scx of row scy, then draw it in SC_PIX
                // SC_FETCH only samples; SC_PIX writes and then decides where
                // to go. The old shape was `if (last) scst <= SC_IDLE;`
                // followed by an unconditional `scst <= SC_PIX;` - the second
                // assignment won, so the scope never returned to SC_IDLE after
                // the last row and redrew row 175 forever at one write every
                // two clocks. That took half the framebuffer write bandwidth
                // permanently and starved the status panel (lowest priority),
                // which then stopped updating for good. It also skipped the
                // very last pixel, hence 42239 writes instead of 42240.
                SC_FETCH: begin
                    if (lst == LJ_IDLE && sst == SP_IDLE) begin
                        fxs     <= scx;
                        fys     <= scy;
                        sc_ys_q <= sc_ys;
                        scst    <= SC_PIX;
                    end
                end
                SC_PIX: begin
                    if (lst == LJ_IDLE && sst == SP_IDLE) begin
                        fb_we    <= 1'b1;
                        fb_addr  <= pixaddr({1'b0, fys}, fxs);
                        fb_color <= sc_clr ? COL_BG
                                  : sc_on  ? (sc_d == 8'd0 ? COL_CORE : COL_TRACE)
                                           : COL_BG;
                        if (fxs == 8'd239 && fys == 8'd175) begin
                            scx    <= 8'd0;
                            scy    <= 8'd0;
                            sc_clr <= 1'b0;
                            scst   <= SC_IDLE;
                        end else if (fxs == 8'd239) begin
                            scx  <= 8'd0;
                            scy  <= scy + 8'd1;
                            scst <= SC_FETCH;
                        end else begin
                            scx  <= scx + 8'd1;
                            scst <= SC_FETCH;
                        end
                    end
                end
                default: scst <= SC_IDLE;
            endcase

            // ---------------- status panel (lowest priority) -----------
            case (txst)
                TX_IDLE: begin
                    if (tx_tmr != 23'd4_050_000) tx_tmr <= tx_tmr + 23'd1;
                    else begin
                        tx_tmr <= 23'd0;
                        // rebuild both text lines from the live controls.
                        // Panel codes: 8'h80+i = CJK glyph i (font_cn.v),
                        // 8'h20..7E = ASCII. Line budget is 240 px; CJK
                        // costs 16, ASCII 10 - the trailing spaces are
                        // load-bearing (see the renderer comment above).
                        // line0: 音色:<名> 八度:<n> <李萨如|波形> <指引|频谱>
                        // <名> is 2 CJK glyphs plus one filler slot (space,
                        // or '8' for the 8-harmonic additive timbre). That
                        // filler IS the gap before 八度 - spending one more
                        // slot on a separate space pushes 指引 to x=230..245
                        // in Lissajous mode and clips it (246 px worst case).
                        line0[0]  <= 8'h80;  // 音
                        line0[1]  <= 8'h81;  // 色
                        line0[2]  <= 8'h3A;  // :
                        line0[3]  <= tmb3[23:16];
                        line0[4]  <= tmb3[15:8];
                        line0[5]  <= tmb3[7:0];
                        line0[6]  <= 8'h82;  // 八
                        line0[7]  <= 8'h83;  // 度
                        line0[8]  <= 8'h3A;  // :
                        line0[9]  <= 8'h31 + {6'b000000, oct_shift};
                        line0[10] <= 8'h20;
                        // 画面: 李萨如 (X-Y) / 波形 (Y-T)
                        line0[11] <= scope_mode ? 8'h89 : 8'h86;  // 波 / 李
                        line0[12] <= scope_mode ? 8'h8A : 8'h87;  // 形 / 萨
                        line0[13] <= scope_mode ? 8'h20 : 8'h88;  // _  / 如
                        line0[14] <= 8'h20;
                        // 灯带: 指引 / 频谱
                        line0[15] <= led_mode ? 8'h95 : 8'h93;  // 频 / 指
                        line0[16] <= led_mode ? 8'h96 : 8'h94;  // 谱 / 引
                        line0[17] <= 8'h20;
                        line0[18] <= 8'h20;
                        line0[19] <= 8'h20;
                        line0[20] <= 8'h20;
                        line0[21] <= 8'h20;
                        line0[22] <= 8'h20;
                        line0[23] <= 8'h20;

                        // line1: <参数名>:<vvv> 声部:<nn> <音量|曲目>:<nnn>
                        //   起音:063 声部:03 音量:128
                        // This slot ran the MPR121
                        // bring-up readout ("E0551 B137 D+000 O000") while
                        // touch was being debugged; the MPR121 numbers are
                        // no longer needed now that the FPGA decides touch
                        // itself, and the parameter name / voice count /
                        // output level are what the panel is for.
                        // In song-guide mode the tail shows 曲目:nn instead.
                        // Content ends at 236 px (same as line0), so slots
                        // 19..23 are trailing spaces that tvis clips.
                        line1[0]  <= pnm[15:8];
                        line1[1]  <= pnm[7:0];
                        line1[2]  <= 8'h3A;  // :
                        line1[3]  <= pd_h;
                        line1[4]  <= pd_t;
                        line1[5]  <= pd_o;
                        line1[6]  <= 8'h20;
                        line1[7]  <= 8'h8D;  // 声
                        line1[8]  <= 8'h8E;  // 部
                        line1[9]  <= 8'h3A;
                        line1[10] <= vx_t;
                        line1[11] <= vx_o;
                        line1[12] <= 8'h20;
                        // pressure readout - see the wire block near lv_h;
                        // it steals the trailing "音量:nnn" field, but only
                        // once a PCF8591 has answered
                        line1[13] <= lb1;
                        line1[14] <= lb2;
                        line1[15] <= 8'h3A;
                        line1[16] <= d16;
                        line1[17] <= d17;
                        line1[18] <= d18;
                        line1[19] <= 8'h20;
                        line1[20] <= 8'h20;
                        line1[21] <= 8'h20;
                        line1[22] <= 8'h20;
                        line1[23] <= 8'h20;

                        tx_ci <= 5'd0;
                        tx_cx <= 4'd0;
                        tx_x  <= 9'd0;
                        tx_py <= 6'd0;
                        txst  <= TX_RUN;
                    end
                end
                TX_RUN: begin
                    if (lst == LJ_IDLE && sst == SP_IDLE && scst == SC_IDLE) begin
                        // x >= 240 does not exist on the panel - never write
                        // there, the address would wrap into column 0
                        fb_we   <= tvis;
                        fb_addr <= pixaddr(ty, tx8);
                        fb_color <= (tin && ton) ? (in1 ? COL_TX2 : COL_TX1)
                                                 : COL_PANEL;
                        if (tx_cx == (iscn ? 4'd15 : 4'd9)) begin
                            tx_cx <= 4'd0;
                            tx_x  <= tx_x + (iscn ? 9'd16 : 9'd10);
                            if (tx_ci == 5'd23) begin
                                tx_ci <= 5'd0;
                                tx_x  <= 9'd0;
                                if (tx_py == 6'd39) txst <= TX_IDLE;
                                else tx_py <= tx_py + 6'd1;
                            end else tx_ci <= tx_ci + 5'd1;
                        end else tx_cx <= tx_cx + 4'd1;
                    end
                end
                default: txst <= TX_IDLE;
            endcase

            // ---------------- WS2812 strip -----------------------------
            case (wst)
                WS_IDLE: if (ws_frame_start) begin
                    widx <= 6'd0;
                    wst  <= WS_RUN;
                end
                WS_RUN: begin
                    ws_we   <= 1'b1;
                    ws_addr <= widx;
                    if (led_mode == 1'b0) begin
                        ws_color <= note_active[ws_bin] ? slot24(ws_bin)
                                                        : hue24(widx);
                    end else begin
                        // 谱线模式：绿->红 渐变。
                        // mag=0 时若不处理会算出 G=0xFF（满亮绿），安静时 12 颗灯
                        // 全亮成绿色，与按下某个同为绿色的键难以区分。
                        // 低于阈值时回中性暗色。
                        ws_color <= (mag_q[ws_bin] < 8'd10) ? 24'h141414
                                  : {8'hFF - mag_q[ws_bin], mag_q[ws_bin], 8'h00};
                    end
                    // 12 颗灯带：写到第 11 颗就收尾，多写会越过物理末端
                    if (widx == 6'd11) wst <= WS_IDLE;
                    else widx <= widx + 6'd1;
                end
                default: wst <= WS_IDLE;
            endcase
        end
    end

endmodule
`default_nettype wire
