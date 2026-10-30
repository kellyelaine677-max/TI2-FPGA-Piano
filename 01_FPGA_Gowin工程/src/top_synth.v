`timescale 1ns/1ps
// ============================================================================
// top_synth.v (v5) - FPGA realtime synthesizer, milestones M1..M6
//
// Target: Sipeed Tang Primer 20K (GW2A-LV18PG256C8/I7) + Dock baseboard,
//         27 MHz onboard clock. Audio out: Dock headphone jack (LPA4809MSF
//         amp) - also compatible with PCM5102 / WM8978 by pin re-assign.
//
// v6 (this build) - 32 voices on a 40.5 MHz core clock
//
// The v5 build sat at 27 MHz / 24 voices with Fmax = 34.4 MHz, which left the
// 32-voice target (256 simultaneous oscillators, the scoring metric) out of
// reach. v6 is a pure timing-closure pass - no feature was removed:
//   1. voice_engine: the allocation/retrigger scan now walks only the next 8
//      slots from the round-robin pointer and steals rr_ptr if all 8 are
//      busy; note_off became a broadcast (every voice holding that note goes
//      to release) so a stolen voice can never drone.
//   2. voice_engine: v_age shrank to 8 bits and is written from the register
//      ST_S0 already loaded. Re-reading v_age[cur] in the write cycle had put
//      a 24:1 read mux + saturating compare + increment behind `cur`.
//   3. voice_engine: the voice-slot address is taken apart - a registered
//      one-hot write enable plus ph_raw_q / ph_q / incv_q. `cur` had 753
//      fanout and a single 9.6 ns net; nothing re-reads v_phase[cur] or
//      v_inc[cur] after ST_S0 any more.
//   4. voice_engine: the portamento step (a 32-bit variable shift, a 5-level
//      LUT tree) is computed in ST_S1 into pstep_q instead of feeding v_inc
//      straight from the parameter register.
//   5. display_ctrl: the outgoing trace point is fetched in LJ_IDLE (ept_q)
//      instead of indexing the 1024-entry pts[] inside the erase states.
//   6. display_ctrl: the scope pixel loop is split into SC_FETCH / SC_PIX,
//      and the Lissajous point moved into its own LJ_PT register stage, so
//      the dly[] read mux no longer drives scaley's x11 multiplier.
//   7. clk_gen: rPLL now generates 40.5 MHz (IDIV=1, FBDIV=2, ODIV=16) and
//      is actually instantiated. timing.sdc constrains the clk40 domain, not
//      just the 27 MHz input - constraining only the input is what made the
//      earlier "54 MHz passes P&R" result meaningless.
//
// Measured result: Fmax 34.385 -> 55.477 MHz, TNS 0 on every clock domain.
// 32 voices x 14 cycles + 5 = 453 of the 576 cycles per sample (21 % headroom).
// The audio sample rate is UNCHANGED at 70.3125 kHz (40.5 MHz / 576), so no
// sample-rate-derived constant in the engine or the analysers moved.
//
// v5 (previous build) added the full P0/P1/P2 improvement set:
//   P0-1  burst self-test: 8 independent notes one after another, then all
//         32 sounding at once -> 32 x 8 = 256 oscillators (was 24 / 192)
//   P0-2  portamento (glide): exponential slew of the phase increment, so
//         every octave takes the same time (param 9)
//   P0-3  per-partial independent amplitude decay: high harmonics fade
//         faster than the fundamental, so each oscillator has its own
//         envelope behaviour
//   P1-4  32 voices (was 24, originally 16) - 256 simultaneous oscillators
//   P1-5  offline LED song guide (拓展 4): a built-in melody is synthesised
//         live and the strip lights the next key (key13+key14 long press)
//   P2-6  waveform (Y-T) scope mode on the LCD, rising-edge triggered so the
//         trace stands still (key14 long press)
//   P2-7  bus tone filter: one-pole IIR low-pass on the mix bus, param 8
//
// Clock: 40.5 MHz from the rPLL in clk_gen.v (27 MHz oscillator, IDIV=1,
// FBDIV=2, ODIV=16). Post-route STA reports Fmax = 55.5 MHz on the clk40
// domain with zero negative slack, so 40.5 MHz closes with ~27 % margin.
// 32 voices x 14 cycles + 5 = 453 cycles fit in the 576-cycle sample period.
//
//
// M5 additions:
//   * 8-harmonic additive timbre (#6): 16 voices x 8 = 128 simultaneous
//     oscillators at 70.3 kHz sample rate
//   * fx_echo: multi-tap feedback delay (echo/simple reverb) in BRAM,
//     feedback + wet-mix runtime adjustable (params 6/7)
//
// M6 additions:
//   * DIP-switch test mode: auto-plays 8 sustained A/E notes whose
//     fundamentals land on the Goertzel bins - live spectrum verification
//     of independent oscillator peaks (scoring "现场验证" requirement)
//
// M4: Goertzel spectrum, ST7789V Lissajous + bars, WS2812 guidance strip.
// M2/M3: MPR121 touch + PCF8591 velocity, EC11 expression controls,
// FM + additive timbres, vibrato LFO, adjustable ADSR.
//
// Inputs (>=3 expression dimensions for the scoring rubric):
//   * 4x4 matrix keypad   - 12 chromatic keys + octave -/+ + timbre/param
//   * MPR121 capacitive touch (12 electrodes, I2C) - chromatic keyboard
//   * PCF8591 ADC ch0 + RP-S40-ST pressure sensor - NOTE VELOCITY + pressure
//   * EC11 encoder 1      - timbre select (rotate), param select (button)
//   * EC11 encoder 2      - parameter value +/- (rotate)
//   * vibrato LFO depth   - runtime adjustable expression modulation
//
// Chain: keypad/touch -> note events (+velocity) -> voice_engine v2
//        (16 voices, sine/sq/saw/tri/FM/additive, ADSR, vibrato)
//        -> mixer -> i2s_tx -> headphone amp -> speaker
//
// Timbre & parameters:
//   timbre    : 0 sine | 1 square | 2 saw | 3 triangle | 4 FM | 5 additive
//   param id  : 0 attack | 1 decay | 2 release | 3 sustain | 4 FM index
//               5 vibrato depth
//
// Sample rate 70.3125 kHz; latency one sample (~14 us) << 10 ms requirement.
// ============================================================================
`default_nettype none

module top_synth (
    input  wire       clk27,     // H11, 27 MHz onboard oscillator
    input  wire       rst_btn,   // T10 = Dock S0 key, pressed = low
    // 4x4 matrix keypad (rows need pull-ups, set PULL_MODE=UP in cst)
    output wire [3:0] kb_col,
    input  wire [3:0] kb_row,
    // I2C sensor bus (MPR121 touch + PCF8591 ADC, external pull-ups)
    inout  wire       sda,
    inout  wire       scl,
    // EC11 rotary encoders (A/B + push button, idle high)
    input  wire       enc1_a, enc1_b, enc1_btn,
    input  wire       enc2_a, enc2_b, enc2_btn,
    // diagnostic build: the old dip_test jumper pin (J6 pin12, ball E15) is
    // repurposed as the LCD SDO read-back input for the panel ID probe
    input  wire       lcd_sdo,
    // Bluetooth link: MIDI bytes in from a transparent-UART BLE module.
    // This is the ONLY free IO left - J5/J6/J12 each have 8 IOs and all 24
    // are taken, so this has to come from the J14 header (pin5..12).
    input  wire       bt_rxd,
    // ... and the same link back out: note events streamed as MIDI bytes for
    // the app to record (拓展 4: 演奏录音回传)
    output wire       bt_txd,
    // onboard headphone amp
    output wire       hp_bck,
    output wire       hp_ws,
    output wire       hp_din,
    output wire       pa_en,
    // WS2812B strip (5V supply, series 470R on DIN, common ground)
    output wire       ws_din,
    // 板上自带的那颗 WS2812（J14 座 WS2812-DAT，球 T9）与外接灯带同源、
    // 送同一串数据，所以它显示的是外接灯带"第 0 颗"的颜色。
    // 这颗灯由板子自己供电，不需要任何外接连线 ——
    // 外接灯带全不亮时，看它亮不亮就能一次分清：
    //   板上这颗亮  = FPGA 和 WS2812 驱动逻辑都是好的，问题在外接灯带/接线
    //   板上这颗不亮 = 问题在板子这一侧
    output wire       ws_din_ob,
    // ST7789V SPI LCD
    output wire       lcd_scl,
    output wire       lcd_sda,
    output wire       lcd_dc,
    output wire       lcd_cs,
    output wire       lcd_rst,
    output wire       lcd_bl,
    // status LEDs
    output wire [5:0] led
);

    // ------------------------------------------------------------------
    // system clock: 27 MHz oscillator -> 40.5 MHz via rPLL (clk_gen.v)
    //
    // The core needs to be fast enough that one audio sample period can hold
    // 32 voices x 14 pipeline cycles + 5 overhead = 453 core cycles.
    //   at 27 MHz : 384 cycles/sample -> 24 voices max
    //   at 40.5 MHz: 576 cycles/sample -> 32 voices, 21 % headroom
    //
    // 54 MHz was rejected: post-route STA measures Fmax = 52.1 MHz on this
    // design, so a 54 MHz core misses timing. (P&R passing is NOT timing
    // passing - the number to read is impl/pnr/project.tr section 2.3.)
    // Getting Fmax up from 34.4 to 52.1 MHz took: shortening the allocation
    // scan, taking the voice-slot address apart into a registered one-hot
    // write enable, and hoisting three wide reads/variable shifts out of the
    // voice accumulate stage. See voice_engine.v and display_ctrl.v.
    // ------------------------------------------------------------------
    wire       clk;
    wire       pll_lock;
    clk_gen u_clk (.clk27 (clk27), .clk40 (clk), .lock (pll_lock));

    // ------------------------------------------------------------------
    // reset release (button held = reset; auto-release after 255 cycles)
    // The PLL lock gates the release so nothing runs on an unstable clock.
    // ------------------------------------------------------------------
    reg [7:0] rstn_cnt;
    always @(posedge clk or negedge rst_btn) begin
        if (!rst_btn) rstn_cnt <= 8'd0;
        else if (rstn_cnt != 8'hFF) rstn_cnt <= rstn_cnt + 8'd1;
    end
    wire rstn = (rstn_cnt == 8'hFF) && pll_lock;

    // The M6 self-test is now triggered by the encoder-2 button long-press
    // (see the e2b_cnt block below); the old J6-pin12 jumper is gone because
    // that pin carries the LCD SDO read-back. lcd_sdo stays an input (it is
    // wired in the cst) but its ID-probe consumer is optimised away.

    // ------------------------------------------------------------------
    // sample tick: one audio sample per 576 core cycles
    //              40.5 MHz / 576 = 70.3125 kHz  (SAME audio rate as the
    //              27 MHz build, so no sample-rate-derived constant changes)
    // ------------------------------------------------------------------
    reg [9:0] tick_cnt;
    wire sample_tick = (tick_cnt == 10'd575);
    always @(posedge clk or negedge rstn) begin
        if (!rstn) tick_cnt <= 10'd0;
        else tick_cnt <= sample_tick ? 10'd0 : tick_cnt + 10'd1;
    end

    // ------------------------------------------------------------------
    // keypad scan
    // ------------------------------------------------------------------
    wire [15:0] key_state;
    wire        key_on, key_off;
    wire [3:0]  key_idx;
    key_scan u_keys (
        .clk (clk), .rstn (rstn),
        .col (kb_col), .row (kb_row),
        .key_state (key_state),
        .key_on (key_on), .key_off (key_off), .key_idx (key_idx)
    );

    // ------------------------------------------------------------------
    // sensor hub: MPR121 touch + PCF8591 pressure/velocity
    // ------------------------------------------------------------------
    wire [11:0] touch12;
    wire        touch_stb;
    wire [7:0]  press8;
    wire        press_ok;   // 1 = a pressure sensor answered the ADC poll
    wire [1:0]  hub_addr;
    wire        hub_found;
    wire [9:0]  elec0;
    wire [7:0]  elec_diff;
    wire [7:0]  bl0;
    wire        elec_neg;
    wire [11:0] oor12;
    sensor_hub u_hub (
        .clk (clk), .rstn (rstn),
        .sda (sda), .scl (scl),
        .touch12 (touch12), .touch_stb (touch_stb),
        .press8 (press8), .press_ok (press_ok), .hub_ready (),
        .hub_addr (hub_addr), .hub_found (hub_found),
        .elec0 (elec0), .elec_diff (elec_diff), .bl0 (bl0),
        .elec_neg (elec_neg), .oor12 (oor12)
    );

    // ------------------------------------------------------------------
    // Resting-value calibration: how hard, not which way
    //
    // press8 is whatever the divider puts on AIN0, and WHICH WAY that moves
    // depends on how the two resistors are wired. This board reads 3.26 V
    // with nothing on the pad and 1.80 V with a cup on it, i.e. the ADC
    // FALLS as you press - so the raw byte put every modulation listed below
    // at MAXIMUM with the pad untouched and reduced it as you squeezed,
    // which is backwards.
    //
    // Remember the reading once, right after the ADC first answers, and
    // drive everything from the deviation from it. Then the pad behaves the
    // same whichever way round the resistors end up, and the range is the
    // same too (~170 counts full-scale either way).
    //
    // Cost: one byte, one subtract, one compare. Caveat: power up with
    // nothing resting on the pad - that first reading is taken as zero.
    // ------------------------------------------------------------------
    reg  [7:0] press_rest;
    reg        press_cal;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            press_rest <= 8'd0;
            press_cal  <= 1'b0;
        end else if (press_ok && !press_cal) begin
            // press8 and press_ok land in the same cycle, so this is a
            // genuine reading, not a stale one
            press_rest <= press8;
            press_cal  <= 1'b1;
        end
    end
    wire [7:0] press_amt = press_cal
                         ? ((press8 > press_rest) ? (press8 - press_rest)
                                                  : (press_rest - press8))
                         : 8'd0;

    // ------------------------------------------------------------------
    // EC11 encoders
    // ------------------------------------------------------------------
    wire e1_inc, e1_dec, e1_press;
    wire e2_inc, e2_dec, e2_press;
    ec11_ctrl u_enc1 (
        .clk (clk), .rstn (rstn),
        .a (enc1_a), .b (enc1_b), .btn (enc1_btn),
        .inc (e1_inc), .dec (e1_dec), .btn_press (e1_press)
    );
    ec11_ctrl u_enc2 (
        .clk (clk), .rstn (rstn),
        .a (enc2_a), .b (enc2_b), .btn (enc2_btn),
        .inc (e2_inc), .dec (e2_dec), .btn_press (e2_press)
    );

    // ------------------------------------------------------------------
    // Function-key long presses (keypad only - NO extra wiring, and a floating
    // pin can never fire these, unlike the encoder-2 button which turned out
    // to read low and auto-started the self-test right after programming):
    //   key12 + key13 held together >=0.5 s -> M6 self-test (8 sustained notes
    //        whose fundamentals land on the Goertzel bins) + strip forced to
    //        spectrum mode.  Their short-press side effects (octave -1 / +1)
    //        cancel each other out, so nothing else moves.
    //   key15 held >=0.5 s -> toggle the strip mode (guidance <=> spectrum);
    //        the param cycle its short press already did is undone by the
    //        undo_param pulse consumed in the control block below.
    //   key15 short press -> cycle param_id (unchanged)
    // ------------------------------------------------------------------
    reg        test_start;        // 1-clk pulse
    reg        undo_param;        // 1-clk pulse: undo key15's short-press cycle
    reg        undo_timbre;       // 1-clk pulse: undo key14's short-press cycle
    reg        undo_oct;          // 1-clk pulse: undo key13's short-press octave+
    reg [24:0] fn_cnt;            // 0.5 s long-press threshold @ 40.5 MHz
                                  // (20_250_000 / 40.5e6 = 0.5 s)
    reg        fn_done;           // long press already fired for this hold
    reg        led_mode;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            test_start <= 1'b0; undo_param <= 1'b0;
            undo_timbre <= 1'b0; undo_oct <= 1'b0;
            fn_cnt     <= 25'd0; fn_done <= 1'b0;
            led_mode   <= 1'b0;
            song_mode  <= 1'b0;
            scope_mode <= 1'b0;
        end else begin
            test_start  <= 1'b0;
            undo_param  <= 1'b0;
            undo_timbre <= 1'b0;
            undo_oct    <= 1'b0;
            if (key_state[12] && key_state[13]) begin
                // M6 self-test: 12 notes in sequence, then ALL 32 sounding
                if (fn_cnt != 25'd20_250_000) fn_cnt <= fn_cnt + 25'd1;
                else if (!fn_done) begin
                    fn_done    <= 1'b1;
                    test_start <= 1'b1;
                    led_mode   <= 1'b1;      // spectrum mode shows the peaks
                end
            end else if (key_state[13] && key_state[14]) begin
                // offline LED song guide on/off
                if (fn_cnt != 25'd20_250_000) fn_cnt <= fn_cnt + 25'd1;
                else if (!fn_done) begin
                    fn_done    <= 1'b1;
                    song_mode  <= ~song_mode;
                    undo_oct   <= 1'b1;      // key13 short press moved the octave
                    undo_timbre<= 1'b1;      // key14 short press moved the timbre
                end
            end else if (key_state[14]) begin
                // LCD scope: Lissajous <=> waveform
                if (fn_cnt != 25'd20_250_000) fn_cnt <= fn_cnt + 25'd1;
                else if (!fn_done) begin
                    fn_done    <= 1'b1;
                    scope_mode <= ~scope_mode;
                    undo_timbre<= 1'b1;
                end
            end else if (key_state[15]) begin
                if (fn_cnt != 25'd20_250_000) fn_cnt <= fn_cnt + 25'd1;
                else if (!fn_done) begin
                    fn_done    <= 1'b1;
                    led_mode   <= ~led_mode;
                    undo_param <= 1'b1;
                end
            end else begin
                fn_cnt  <= 25'd0;
                fn_done <= 1'b0;
            end

            // 曲谱下载完成（SysEx 收尾的 F7）-> 自动开始播放。
            // 直接把 song_mode 置 1，不另建 sc_autoplay 标志：另建标志会让
            // CLS 冲到 10171/10368（99%），布线 DesRoute failed 布不开。
            // 想停就长按 key13+key14 切一次曲目模式。
            // 开始下载就先停下并复位（song_player 在 enable=0 时把 idx 归零、
            // pend 置 1，还会顺手把残音 note_off 掉），收到 F7 再从第一个音
            // 开始播新谱。不这么做的话下载期间会继续按旧谱走，指针和内容对不上。
            if      (sc_start) song_mode <= 1'b0;
            else if (sc_end)   song_mode <= 1'b1;
            // CC 34（手机上的"停止播放"按钮）：0 = 停，1 = 播。
            // 放在 sc_start/sc_end 之后，同一拍同时来的话以手机指令为准。
            if (cc_wr && cc_idx == 4'd14) song_mode <= cc_val[0];

            // 编码器 2 的按键：切 LCD 显示模式（李萨如 ↔ 波形）。
            // 等价的键盘操作是长按 key14 约 0.5 秒（fn 块中已实现）。
            // 若 enc2_btn 引脚读到的电平固定为低，则 btn_lvl 恒为 0、
            // 无法捕获下降沿，此时需使用 key14。scope_mode 只有 0/1 两态，
            // 频谱区常驻显示。
            if (e2_press) scope_mode <= ~scope_mode;
        end
    end


    // ------------------------------------------------------------------
    // global controls: octave, timbre, parameter table
    // ------------------------------------------------------------------
    reg [1:0] oct_shift;   // 0..3, base 48/60/72/84
    reg [2:0] timbre;      // 0..7  (7 = piano)
    reg [3:0] param_id;    // 0..9
    reg [7:0] pval [0:9];  // parameter values, see defaults below
    reg       song_mode;   // 1 = offline LED song guide
    reg       scope_mode;  // 0 = Lissajous, 1 = waveform (YT) scope

    // LCD init variant - LOCKED. The user proved on an STM32 that this panel
    // is an ST7789V, so chip_sel is pinned to 1 (the ST7789 init script, which
    // already carries INVON 0x21). No auto-cycling, no ILI9341 trials.
    //   lcd_var = {chip_sel, inv_sel}  -> fixed at 10 = ST7789
    reg [1:0] lcd_var;
    reg       lcd_reinit;
    wire      chip_sel = lcd_var[1];
    wire      inv_sel  = lcd_var[0];

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            oct_shift <= 2'd1;   // default base C4
            timbre    <= 3'd0;
            param_id  <= 3'd0;
            pval[0]   <= 8'd63;  // attack  : ~1.8 ms
            pval[1]   <= 8'd31;  // decay   : ~2.4 ms
            pval[2]   <= 8'd47;  // release : ~4.8 ms
            pval[3]   <= 8'd176; // sustain : 69 %
            pval[4]   <= 8'd40;  // FM index: moderate
            pval[5]   <= 8'd46;  // vibrato : ~19 cents
            pval[6]   <= 8'd100; // echo feedback ~0.39
            pval[7]   <= 8'd76;  // echo wet mix ~0.30
            pval[8]   <= 8'd235; // tone filter cutoff (235 = nearly open)
            pval[9]   <= 8'd11;  // portamento: inc >> 11, ~20 ms per octave
            lcd_var   <= 2'd2;   // locked: ST7789 (chip_sel = 1)
            lcd_reinit<= 1'b0;
        end else begin
            // octave from keypad function keys
            if (key_on) begin
                case (key_idx)
                    4'd12: if (oct_shift != 2'd0) oct_shift <= oct_shift - 2'd1;
                    4'd13: if (oct_shift != 2'd3) oct_shift <= oct_shift + 2'd1;
                    4'd14: timbre <= (timbre == 3'd7) ? 3'd0 : timbre + 3'd1;
                    4'd15: param_id <= (param_id == 4'd9) ? 4'd0 : param_id + 4'd1;
                    default: ;
                endcase
            end
            // key15 was held: undo the param cycle its short press already did
            if (undo_param) param_id <= (param_id == 4'd0) ? 4'd9 : param_id - 4'd1;
            // key14 held: LCD scope mode toggle, undo the timbre cycle
            if (undo_timbre) timbre <= (timbre == 3'd0) ? 3'd7 : timbre - 3'd1;
            // key13+key14 held: song guide toggle, undo octave+ / timbre cycle
            if (undo_oct)   oct_shift <= (oct_shift == 2'd0) ? 2'd3 : oct_shift - 2'd1;
            // encoder 1: timbre select (rotate), param select (button)
            if (e1_inc) timbre <= (timbre == 3'd7) ? 3'd0 : timbre + 3'd1;
            if (e1_dec) timbre <= (timbre == 3'd0) ? 3'd7 : timbre - 3'd1;
            if (e1_press) param_id <= (param_id == 4'd9) ? 4'd0 : param_id + 4'd1;
            // 编码器 2 的按键切 LCD 显示模式 —— 必须写在下面 fn 块里，
            // 不能写在这里：scope_mode 已经在 fn 块（长按 key14）被驱动过，
            // 两个 always 块同时写同一个 reg 是 EX2000 多驱动错误。
            // 蓝牙音色选择（CC 33 -> cc_idx 13）。和编码器/键盘共用 timbre
            // 一个寄存器，手机和板载旋钮改的是同一个值，LCD 会同步显示。
            if (cc_wr && cc_idx == 4'd13) timbre <= cc_val[2:0];
            // (LCD init variant is locked to ST7789 - no more auto-cycling)
            // encoder 2: parameter value +/- (saturating)
            if (e2_inc && pval[param_id] != 8'd255) pval[param_id] <= pval[param_id] + 8'd1;
            if (e2_dec && pval[param_id] != 8'd0)   pval[param_id] <= pval[param_id] - 8'd1;
        end
    end

    wire [6:0] note_base = 7'd48 + oct_shift * 7'd12;

    // parameter value -> engine rates / fx levels
    wire [15:0] a_step    = {pval[0], 3'b000} + 8'd8;              // 8*(v+1)
    wire [15:0] d_step    = {pval[1], 3'b000} + 8'd8;              // 8*(v+1)
    wire [15:0] r_step    = {pval[2], 2'b00} + 8'd4;               // 4*(v+1)
    // sustain fraction: 0..32640 (pval<<7) = 0..99.4% of 32768.
    // MUST stay below the signed-16 ceiling: sus_lvl as signed m_b with
    // pval<<8 (up to 65280) wraps negative and breaks the envelope.
    // ------------------------------------------------------------------
    // PCF8591 pressure expression (RP-S40-ST on a separate squeeze pad)
    //
    // The pad is NOT under the keypad, so sampling it at note-on is not how
    // it gets played - you squeeze it while notes are already sounding.
    // sus_lvl and vib_depth are the two engine inputs that are re-read
    // EVERY sample, so a squeeze is heard at once instead of only changing
    // the next note.
    //
    //   sus_lvl   raises the sustain level of every sounding note: a volume
    //             swell. pval[3] defaults to 176 (22528) and the pad adds up
    //             to 63<<7 = 8064. Those add up past the signed-16 ceiling
    //             if the user also turns the sustain param up, and sus_lvl
    //             is an m_b operand - a value with bit 15 set wraps
    //             negative and breaks the envelope. Hence the clamp.
    //   vib_depth deepens the vibrato. Measured on a MIDI 60 note with
    //   pval[5] at its default 46: 2.57 cents at rest to 16.72 cents when
    //   the pad is fully squeezed, i.e. a 14.2 cent swing. (An earlier
    //   comment here claimed 19 -> 32 cents; that was wrong by ~2x and it
    //   is how a >>3 slice that only moved the vibrato 1.7 cents got
    //   shipped - nobody can hear 1.7 cents.)
    //
    // press_amt is 0 until the ADC answers AND the resting value has been
    // captured, so with no PCF8591 fitted both terms are 0 and nothing
    // changes at all - and with one fitted the pad starts from zero however
    // the divider happens to be wired.
    // ------------------------------------------------------------------
    // Add and clamp BEFORE the shift, on the 9-bit sum. 32640 is exactly
    // 255<<7, so clamping the 9-bit sum to 255 and then shifting is the same
    // number as shifting and then clamping to 32640 - and a 9-bit adder plus
    // a 9-bit compare cost far less than their 16-bit equivalents. At 97 %
    // CLS that difference is worth several MHz of Fmax.
    wire [8:0]  sus_n     = {1'b0, pval[3]} + {1'b0, press_amt[7:1]};
    wire [7:0]  sus_c     = (sus_n > 9'd255) ? 8'd255 : sus_n[7:0];
    wire [15:0] sus_lvl   = {sus_c, 7'b0000000};                // v/256 x peak
    wire [15:0] fm_index  = {pval[4], 4'b0000} + {pval[4], 3'b000} + 8'd24; // 24v+24
    // The FULL 8-bit pressure, doubled. The vibrato offset is 32767 x
    // vib_depth added straight into the 32-bit phase accumulator, which works
    // out to only ~1.7 cents per 1000 of vib_depth - an early version used a
    // >>3 slice and moved it 2.6 -> 4.3 cents, which nobody can hear.
    // This sensor also tops out around 8k with a modest load (ADC ~114 of
    // 255), so the x2 gain is what makes a gentle squeeze audible at all:
    //   2.6 cents at rest, 15.3 cents with a cup on it, 30.7 at full press.
    // (255 + 510) << 5 = 24480, safely positive as a signed 16-bit m_b.
    // 3x, not 2x: the pad only reaches ~170 counts even squeezed hard, so a
    // bigger gain is what makes a gentle squeeze register at all.
    // HARD CEILING: vib_depth is a SIGNED 16-bit m_b operand, so vib_n must
    // stay below 32768>>5 = 1023. Worst case here is pval[5]=255 plus 3*255
    // = 1020 -> 32640, which fits. 4x would overflow into a negative depth.
    wire [9:0]  vib_n     = {2'b00, pval[5]}
                          + {1'b0, press_amt, 1'b0}     // 2x
                          + {2'b00, press_amt};         // +1x
    wire [15:0] vib_depth = {vib_n, 5'b00000};                  // 32v
    // param 8 = tone filter cutoff (0 = darkest, 255 = bypass)
    wire [7:0]  tone_cut  = pval[8];
    // param 9 = portamento rate. 0 = off; the increment slews by inc>>n per
    // sample, so bigger n means a slower glide. Clamp to 1..15.
    wire [3:0]  porta_sh  = (pval[9] == 8'd0)  ? 4'd0 :
                            (pval[9]  > 8'd15) ? 4'd11 : pval[9][3:0];

    // ------------------------------------------------------------------
    // Bluetooth control-change outputs (declared here because the remap
    // table below consumes them, and it sits above the midi_rx instance)
    // ------------------------------------------------------------------
    wire       cc_wr;
    wire [3:0] cc_idx;
    wire [3:0] cc_val;
    // score download from the Bluetooth link (拓展 4: 在线曲谱下载)
    wire       sc_start, sc_byte, sc_end;
    wire [6:0] sc_data;

    // 曲谱传到琴上就自动播放：见下面键盘块里的 `if (sc_end) song_mode <= 1'b1;`。
    // 这里刻意不再新增 sc_autoplay 寄存器 —— 试过，多出来的状态机让 CLS 冲到
    // 10171/10368（99%），布线直接 DesRoute failed。复用 song_mode 本身零成本。

    // ------------------------------------------------------------------
    // touch release watchdog - fixes the "lift a finger and it drones until
    // you touch the same spot again" report.
    //
    // The scanner above is sound: t_diff is recomputed every cycle and t_prev
    // only clears once the release has been issued. So a note that stays on
    // means the release was lost AFTER the scanner - and the one place that
    // can happen is event arbitration: note_on and note_off are separate
    // if-blocks feeding an engine that accepts one event per cycle, so an
    // off that lands on the same clock as an on never takes effect, and
    // nothing ever re-issues it.
    //
    // Rather than guess which race it is, reconcile on every MPR121 poll:
    // walk the twelve electrodes and re-send a note-off for any we still
    // believe is sounding while the pad reports it clear. Repeated offs on a
    // voice that is already releasing are harmless - the engine skips voices
    // already in ENV_RELEASE.
    // ------------------------------------------------------------------
    reg [11:0] t_held;      // electrodes we think are currently sounding
    reg [3:0]  wd_idx;
    reg        wd_run;
    reg        wd_off;
    reg [6:0]  wd_num;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            t_held <= 12'd0; wd_run <= 1'b0; wd_idx <= 4'd0;
            wd_off <= 1'b0;  wd_num <= 7'd0;
        end else begin
            wd_off <= 1'b0;
            // the scanner's own view wins; the watchdog only mops up
            if (t_go) begin
                if (t_val) t_held[t_idx] <= 1'b1;
                else       t_held[t_idx] <= 1'b0;
            end
            if (!wd_run) begin
                if (touch_stb && !t_go) begin
                    wd_run <= 1'b1;
                    wd_idx <= 4'd0;
                end
            end else begin
                if (t_held[wd_idx] && !touch12[wd_idx] && !t_go) begin
                    wd_off          <= 1'b1;
                    wd_num          <= t_note[wd_idx];
                    t_held[wd_idx]  <= 1'b0;      // one retry per poll
                end
                if (wd_idx == 4'd11) wd_run <= 1'b0;
                else                 wd_idx <= wd_idx + 4'd1;
            end
        end
    end

    // ------------------------------------------------------------------
    // slot -> note offset remap (拓展 4: 演奏键位自定义)
    //
    // Identity by default, so nothing changes until an app says otherwise.
    // CC 20..31 from the Bluetooth link rewrite one entry each, which lets a
    // phone rearrange the 12 slots for a different scale or playing style
    // without touching the keypad wiring.
    //
    // Deliberately does NOT affect note-off: key_note[] / t_note[] latch the
    // note a slot actually started, so remapping while keys are held can
    // never leave a note stuck on.
    // ------------------------------------------------------------------
    reg [3:0] kmap [0:11];
    // CC 32 -> cc_idx 12：曲谱播放方式。1 = 引导（只亮 LED 等你弹），
    // 0 = 直接播放（板子自己弹出来）。默认直接播放。
    // CC 33 -> cc_idx 13：音色，交给下面 global controls 里的 timbre 处理。
    reg       guide_only;
    integer i1;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            for (i1 = 0; i1 < 12; i1 = i1 + 1) kmap[i1] <= i1[3:0];
            guide_only <= 1'b0;
        end else if (cc_wr) begin
            // 必须显式排除 12/13：kmap 只有 0..11，直接写 kmap[13] 会把
            // 越界下标喂给数组，综合出一片废逻辑还找不到。
            if      (cc_idx == 4'd12) guide_only <= (cc_val == 4'd1);
            else if (cc_idx <  4'd12) kmap[cc_idx] <= cc_val;
        end
    end

    // ------------------------------------------------------------------
    // per-key note latch (octave/timbre changes do not corrupt held notes)
    // ------------------------------------------------------------------
    reg [6:0] key_note [0:11];
    integer i0;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            for (i0 = 0; i0 < 12; i0 = i0 + 1) key_note[i0] <= 7'd60;
        end else if (key_on && (key_idx < 4'd12)) begin
            key_note[key_idx] <= note_base + {3'b000, kmap[key_idx]};
        end
    end

    // ------------------------------------------------------------------
    // touch event scanner (one event per clk, priority to lowest electrode)
    // ------------------------------------------------------------------
    reg [11:0] t_prev;
    reg [6:0]  t_note [0:11];   // note number latched at touch-on
    wire [11:0] t_diff = t_prev ^ touch12;
    integer ti;
    reg       t_found;
    reg [3:0] t_idx;
    reg       t_val;
    always @* begin
        t_found = 1'b0; t_idx = 4'd0; t_val = 1'b0;
        for (ti = 0; ti < 12; ti = ti + 1) begin
            if (!t_found && t_diff[ti]) begin
                t_found = 1'b1;
                t_idx   = ti[3:0];
                t_val   = touch12[ti];
            end
        end
    end
    // One event per cycle at most, then a 3-cycle quiet window: the engine's
    // allocation is pipelined (pick slot -> write slot), and a note_on that
    // arrives while the previous one is only just being applied would still
    // see the old v_active and grab the same slot (silently losing a note).
    reg [1:0] t_gap;
    wire      t_go = t_found && (t_gap == 2'd0);
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            t_prev <= 12'd0;
            t_gap  <= 2'd0;
            for (ti = 0; ti < 12; ti = ti + 1) t_note[ti] <= 7'd60;
        end else begin
            if (t_go)          t_gap <= 2'd3;
            else if (t_gap != 2'd0) t_gap <= t_gap - 2'd1;
            if (t_go) begin
                t_prev[t_idx] <= t_val;
                if (t_val) t_note[t_idx] <= note_base + {3'b000, kmap[t_idx]};
            end
        end
    end

    // ------------------------------------------------------------------
    // M6: self-test sequencer
    // Triggered by holding key12 and key13 together for >=0.5 s (test_start).
    // ------------------------------------------------------------------
    //  自测音序分为两个阶段，两阶段之间必须留出保持时间。
    //
    // 若 Phase A 发完最后一个音后立刻进入 Phase B 的 32 音齐发，会出现：
    //   1) 多音同时发声的状态仅持续 32x4 = 128 个时钟（约 3 us），
    //      无法观察到各频点柱高同时建立；
    //   2) 两阶段音符总数超过 32 个发声槽，voice_engine 在槽满时按 rr_ptr
    //      抢占，后发的音会顶掉先发、已对齐频点的音。
    // 两者叠加会导致频谱上只能看到零星几根柱子。
    //
    // 目前的顺序：12 个音逐个起（间隔 0.6 s）-> 一起保持 4 s（该时段用于观察
    // 12 根柱子）-> 全部松开 -> 再发 32 音齐奏 -> 保持 -> 松开。
    //
    // 音数取 12 以覆盖全部 12 个频点：若只取低 8 个频点，1760/2640/3520/5280
    // 只能依靠谐波分量（实测 power 仅 9~20，而基频可达 40~83），
    // 高 4 根柱子会明显偏低。每个频点都有一个基频对应，柱高才一致。
    function [6:0] test_note(input [3:0] i);
        case (i)
            4'd0:  test_note = 7'd45;   // A2  110 Hz
            4'd1:  test_note = 7'd52;   // E3  165 Hz
            4'd2:  test_note = 7'd57;   // A3  220 Hz
            4'd3:  test_note = 7'd64;   // E4  330 Hz
            4'd4:  test_note = 7'd69;   // A4  440 Hz
            4'd5:  test_note = 7'd76;   // E5  660 Hz
            4'd6:  test_note = 7'd81;   // A5  880 Hz
            4'd7:  test_note = 7'd88;   // E6 1320 Hz
            4'd8:  test_note = 7'd93;   // A6 1760 Hz
            4'd9:  test_note = 7'd100;  // E7 2637 Hz -> 2640 频点
            4'd10: test_note = 7'd105;  // A7 3520 Hz
            4'd11: test_note = 7'd112;  // E8 5274 Hz -> 5280 频点
            default: test_note = 7'd69;
        endcase
    endfunction
    localparam [3:0] SEQ_N = 4'd12;     // 覆盖全部 12 个频点

    localparam [2:0] TS_IDLE = 3'd0, TS_SEQ = 3'd1, TS_BURST = 3'd2,
                     TS_HOLD = 3'd3, TS_REL = 3'd4,
                     TS_CHORD = 3'd5, TS_CHORD_REL = 3'd6;

    reg [28:0] tst_cnt;      // up to ~13.5 s @ 40.5 MHz (needs 29 bits)
    reg [3:0]  seq_idx;      // 0..11 sequential-note phase (note-on index)
    reg [3:0]  rel_idx;      // 0..11 sequential-note release index
    reg [5:0]  bcnt;         // 0..31 burst note index (32 voices)
    reg [1:0]  bwait;        // spacing between the burst note-on pulses
    reg [2:0]  ts;
    reg        test_fire;    // 1-clk pulse: one sequential note
    reg        seq_rel;      // 1-clk pulse: release one sequential note
    reg        burst_fire;   // 1-clk pulse: one of the 32 simultaneous notes
    reg        burst_rel;    // 1-clk pulse: release one of the 32
    reg        test_run;     // high while the demo is playing
    reg        rel_all;      // 1-clk panic: release every voice at once

    // All wall-clock constants below are in 40.5 MHz cycles (27 MHz value
    // x 1.5). The audio sample rate is unchanged at 70.3125 kHz, so nothing
    // inside the engine or the analysers had to change - only this sequencer.
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            tst_cnt <= 27'd0; seq_idx <= 4'd0; rel_idx <= 4'd0;
            bcnt <= 6'd0; bwait <= 2'd0;
            ts <= TS_IDLE;
            test_fire <= 1'b0; seq_rel <= 1'b0;
            burst_fire <= 1'b0; burst_rel <= 1'b0;
            test_run <= 1'b0; rel_all <= 1'b0;
        end else begin
            test_fire  <= 1'b0;
            seq_rel    <= 1'b0;
            burst_fire <= 1'b0;
            burst_rel  <= 1'b0;
            rel_all    <= 1'b0;
            case (ts)
                TS_IDLE: if (test_start) begin
                    ts        <= TS_SEQ;
                    tst_cnt   <= 29'd60_750_000;   // 1.5 s warm-up
                    seq_idx   <= 4'd0;
                    test_run  <= 1'b1;
                end
                TS_SEQ: begin                       // 12 notes, 0.6 s apart
                    //  索引需延后一拍推进。
                    // test_fire 为单周期脉冲，下游 eng_on_num 在该脉冲周期
                    // 采样音号；若在同一周期写入 seq_idx+1，则采样到的是
                    // 下一个音号，导致音序整体错位。
                    if (test_fire) seq_idx <= seq_idx + 4'd1;
                    if (tst_cnt != 29'd0) tst_cnt <= tst_cnt - 29'd1;
                    else begin
                        test_fire <= 1'b1;
                        tst_cnt   <= 29'd24_300_000;   // 0.6 s
                        if (seq_idx == SEQ_N - 4'd1) begin
                            // 12 个音全部在响 —— 先稳稳保持住让人看
                            ts      <= TS_CHORD;
                            tst_cnt <= 29'd162_000_000; // 4 s
                        end
                    end
                end
                // 12 音齐响保持段：这 4 秒才是"12 根柱子"给评委看的时刻
                TS_CHORD: begin
                    if (tst_cnt != 29'd0) tst_cnt <= tst_cnt - 29'd1;
                    else begin
                        ts      <= TS_CHORD_REL;
                        rel_idx <= 4'd0;
                    end
                end
                // 进入 Phase B 之前先释放这 12 个音。
                // 否则音符总数超过 32 个发声槽，voice_engine 会按 rr_ptr
                // 抢占，顶掉已对齐频点的音，频谱柱数随之减少。
                TS_CHORD_REL: begin                 // 12 note-off, 1 per clock
                    // 同上：索引延后一拍，保证松的是刚才起的那个音号
                    if (seq_rel) rel_idx <= rel_idx + 4'd1;
                    if (rel_idx != SEQ_N) begin
                        seq_rel <= 1'b1;
                    end else begin
                        ts    <= TS_BURST;
                        bcnt  <= 6'd0;
                        bwait <= 2'd0;
                    end
                end
                TS_BURST: begin                     // 32 note-on, 1 per 4 clk
                    // 4-clk spacing: the engine's allocation is pipelined and
                    // needs a free cycle to see the updated v_active, or the
                    // second note would land on the same voice slot.
                    bwait <= bwait + 2'd1;
                    if (bcnt != 6'd32) begin
                        if (bwait == 2'd0) begin
                            burst_fire <= 1'b1;
                            bcnt       <= bcnt + 6'd1;
                        end
                    end else begin
                        ts      <= TS_HOLD;
                        tst_cnt <= 29'd162_000_000;  // hold 4 s
                    end
                end
                TS_HOLD: begin
                    if (tst_cnt != 29'd0) tst_cnt <= tst_cnt - 29'd1;
                    else begin
                        ts   <= TS_REL;
                        bcnt  <= 6'd0;
                    end
                end
                TS_REL: begin                       // 32 note-off, 1 per clock
                    if (bcnt != 6'd32) begin
                        burst_rel <= 1'b1;
                        bcnt      <= bcnt + 6'd1;
                    end else begin
                        // Phase A 的 12 个音已经在 TS_CHORD_REL 里松掉了，
                        // 这里的 rel_all 只是兜底：万一哪一拍漏发，别让音
                        // 在演示结束后一直拖着。
                        rel_all   <= 1'b1;
                        ts        <= TS_IDLE;
                        test_run  <= 1'b0;
                    end
                end
                default: ts <= TS_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // offline LED song guide (拓展 4)
    // ------------------------------------------------------------------
    wire        song_on, song_off;
    wire [6:0]  song_num;
    wire [11:0] song_slot;
    wire [7:0]  song_prog;

    // 引导模式的推进脉冲：用户在琴上按下去了（触摸条或矩阵键盘都算）。
    // t_val=1 表示"按下"（0 是松手）；key_idx<12 排除八度/音色这些功能键。
    // 半音槽取 kmap[] —— 用户在键位页改过映射的话，要按改完的映射比对，
    // 否则灯亮第 3 盏、用户按第 3 个键却因为映射被改过而对不上。
    // （key_is_note 在下面才声明，这里内联避免前向引用）
    wire        g_step = (t_go && t_val) || (key_on && (key_idx < 4'd12));
    wire [3:0]  g_semi = (t_go && t_val) ? kmap[t_idx[3:0]] : kmap[key_idx];

    song_player u_song (
        .clk (clk), .rstn (rstn),
        .enable (song_mode), .guide_only (guide_only),
        .key_step (g_step), .key_semi (g_semi),
        .note_base (note_base),
        .note_on (song_on), .note_off (song_off), .note_num (song_num),
        .cur_slot (song_slot), .progress (song_prog),
        .sc_start (sc_start), .sc_byte (sc_byte), .sc_end (sc_end),
        .sc_data (sc_data)
    );

    // ------------------------------------------------------------------
    // Bluetooth MIDI in (拓展 4: 蓝牙模块与手机 App 联动)
    //
    // A transparent-UART BLE module hands over raw MIDI bytes; the BLE stack
    // itself stays inside the module, which is the only reason this fits in
    // the little logic that is left. uart_rx turns bits into bytes, midi_rx
    // turns bytes into note on/off.
    // ------------------------------------------------------------------
    // HC-06 ships at 9600 baud, so that is the default here:
    // 40.5 MHz / 9600 = 4218.75 -> 4219, i.e. 9599.4 baud, -0.006 % off.
    // To run the module at 115200 instead, send AT+BAUD8 to the HC-06 while
    // it is NOT paired, then change this to 352.
    localparam BT_CPB = 4219;

    wire [7:0] bt_byte;
    wire       bt_byte_v;
    uart_rx #(.CLKS_PER_BIT(BT_CPB)) u_uart (
        .clk (clk), .rstn (rstn), .rxd (bt_rxd),
        .dout (bt_byte), .dout_valid (bt_byte_v)
    );

    wire       midi_on, midi_off;
    wire [6:0] midi_num;
    midi_rx u_midi (
        .clk (clk), .rstn (rstn),
        .din (bt_byte), .din_valid (bt_byte_v),
        .note_on (midi_on), .note_off (midi_off), .note_num (midi_num),
        .cc_wr (cc_wr), .cc_idx (cc_idx), .cc_val (cc_val),
        .sc_start (sc_start), .sc_byte (sc_byte), .sc_end (sc_end),
        .sc_data (sc_data)
    );

    // ------------------------------------------------------------------
    // note event mux: bluetooth > self-test > song guide > touch > keypad
    //
    // MIDI carries an ABSOLUTE note number, so unlike the keypad and the
    // touch strip it does NOT get note_base added - the app decides the
    // octave, not the octave buttons on the board.
    // ------------------------------------------------------------------
    wire       key_is_note = (key_idx < 4'd12);
    wire       eng_on      = midi_on    ? 1'b1 :
                             test_fire  ? 1'b1 :
                             burst_fire ? 1'b1 :
                             song_on    ? 1'b1 :
                             (t_go    ? t_val   : (key_on  && key_is_note));
    // seq_rel 在最后一个音松开后的那一拍还会是高电平（那时 rel_idx 已经
    // 等于 SEQ_N，会取到 test_note 的 default 分支），用 seq_rel_v 把它挡掉，
    // 否则会多发一个 69 号(A4)的松手事件，误伤 Phase B 里的同音号 voice。
    wire       seq_rel_v   = seq_rel && (rel_idx != SEQ_N);

    wire       eng_off     = wd_off     ? 1'b1 :   // watchdog retry first
                             midi_off   ? 1'b1 :
                             seq_rel_v  ? 1'b1 :
                             burst_rel  ? 1'b1 :
                             song_off   ? 1'b1 :
                             (t_go    ? ~t_val  : (key_off && key_is_note));
    wire [6:0] eng_on_num  = midi_on    ? midi_num :
                             test_fire  ? test_note(seq_idx) :
                             burst_fire ? (7'd48 + {1'b0, bcnt}) :
                             song_on    ? song_num :
                              //  起音与停音必须使用同一算式。
                              // eng_off_num 使用 t_note[]（含 kmap 映射）。
                              // 默认恒等映射时两者相等，一旦在键位页修改过
                              // 映射，松手发出的音号就会与按下时不一致，
                             // voice_engine 找不到那个 voice，音一直响。
                             (t_go    ? (note_base + {3'b000, kmap[t_idx[3:0]]})
                                      : (note_base + {3'b000, kmap[key_idx]}));
    wire [6:0] eng_off_num = wd_off     ? wd_num :
                             midi_off   ? midi_num :
                             seq_rel_v  ? test_note(rel_idx) :
                             burst_rel  ? (7'd48 + {1'b0, bcnt}) :
                             song_off   ? song_num :
                             (t_go    ? t_note[t_idx] : key_note[key_idx]);

    // ------------------------------------------------------------------
    // note events -> MIDI bytes -> Bluetooth (拓展 4: 演奏录音回传)
    //
    // The app records; the board only streams. Gated off during the
    // self-test, otherwise the 12-note demo plus the 32-note burst would
    // flood the link with noise nobody asked for.
    // ------------------------------------------------------------------
    wire       bt_ev_on  = eng_on  && !test_run;
    wire       bt_ev_off = eng_off && !test_run;
    wire [6:0] bt_ev_num = bt_ev_on ? eng_on_num : eng_off_num;
    bt_tx u_bttx (
        .clk (clk), .rstn (rstn),
        .ev_on (bt_ev_on), .ev_off (bt_ev_off), .ev_num (bt_ev_num),
        .txd (bt_txd)
    );
    // ------------------------------------------------------------------
    // Note velocity is NOT taken from the pressure pad any more.
    //
    // It used to be, and that was a mistake: v_peak is written once at
    // note-on and never revisited, so the pad only ever mattered for the
    // few milliseconds around the moment you touched - hold a note and the
    // pad did nothing. Worse, with the divider wired the way this board is
    // wired the reading is HIGH at rest and FALLS as you press, so the note
    // came out quieter the harder you squeezed. The pad is a separate
    // squeeze pad, not a key bed, so it belongs on the continuous
    // expression inputs below, not on the attack.
    // ------------------------------------------------------------------
    // 自测音给满力度：演示信号本身离满刻度很远（ADD8 谐波系数只给 1/4 量程，
    // 再乘复音归一化，12 音齐奏混音峰值才约 -19 dBFS），能给的一点都别省。
    wire [7:0] eng_vel     = (test_fire || burst_fire) ? 8'd255 : 8'd232;

    // Event pipeline stage. Without it the touch/keypad scan, the source mux
    // and the engine's 32-voice allocation scan all sit in ONE combinational
    // chain that ends on every voice-state register - that path alone capped
    // Fmax at 26 MHz. Latching the event here splits it in two.
    reg        ev_on, ev_off;
    reg [6:0]  ev_on_num, ev_off_num;
    reg [7:0]  ev_vel;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            ev_on <= 1'b0; ev_off <= 1'b0;
            ev_on_num <= 7'd60; ev_off_num <= 7'd60; ev_vel <= 8'd200;
        end else begin
            ev_on      <= eng_on;
            ev_off     <= eng_off;
            ev_on_num  <= eng_on_num;
            ev_off_num <= eng_off_num;
            ev_vel     <= eng_vel;
        end
    end

    // in self-test force the 8-harmonic additive timbre (clean peak series)
    wire [2:0] timbre_eff  = test_run ? 3'd6 : timbre;

    // ------------------------------------------------------------------
    // synthesis engine
    // ------------------------------------------------------------------
    wire signed [15:0] eng_sample;
    wire               eng_valid;
    wire [5:0]         active_cnt;
    // 32 voices x 8 harmonics = 256 simultaneously sounding oscillators.
    // Budget: 32 x 14 + 5 = 453 of the 576 cycles per sample at 40.5 MHz
    // (21 % headroom). 24 voices would only reach 192 oscillators.
    voice_engine #(.NVOICES(32)) u_engine (
        .clk (clk), .rstn (rstn),
        .sample_tick (sample_tick),
        .note_on (ev_on), .note_on_num (ev_on_num), .note_on_vel (ev_vel),
        .note_off (ev_off), .note_off_num (ev_off_num), .rel_all (rel_all),
        .timbre (timbre_eff),
        .a_step (a_step), .d_step (d_step), .r_step (r_step),
        .sus_lvl (sus_lvl), .fm_index (fm_index), .vib_depth (vib_depth),
        .porta_sh (porta_sh),
        .sample_out (eng_sample), .sample_valid (eng_valid),
        .active_cnt (active_cnt)
    );

    // ------------------------------------------------------------------
    // M5: multi-tap echo / simple reverb (BRAM delay line, 116 ms)
    //     -> then the P2 tone filter, then the DAC + analysers
    // ------------------------------------------------------------------
    wire signed [15:0] echo_sample;
    wire               echo_valid;
    wire signed [15:0] sample_out;
    wire               sample_valid;

`ifdef SIM_MINI
    // fast-sim / bisect config: bypass FX + all M4 peripherals
    assign echo_sample = eng_sample;
    assign echo_valid  = eng_valid;
    assign lcd_scl = 1'b1;  assign lcd_sda = 1'b1;  assign lcd_dc = 1'b1;
    assign lcd_cs = 1'b0;  assign lcd_rst = 1'b1;  assign lcd_bl = 1'b1;
    assign ws_din  = 1'b0;
    assign ws_din_ob = 1'b0;
`else
    fx_echo u_fx (
        .clk (clk), .rstn (rstn),
        .in_valid (eng_valid), .in_data (eng_sample),
        .fb_level (pval[6]), .wet_level (pval[7]),
        .out_valid (echo_valid), .out_data (echo_sample)
    );
`endif

    // ------------------------------------------------------------------
    // P2: bus tone filter (one-pole IIR low-pass, param 8 = cutoff)
    // Sits AFTER the echo and BEFORE the DAC + analysers, so what you hear
    // is also what the spectrum / scope show.
    // ------------------------------------------------------------------
    fx_tone u_tone (
        .clk (clk), .rstn (rstn),
        .in_valid (echo_valid), .in_data (echo_sample),
        .cutoff (tone_cut),
        .out_valid (sample_valid), .out_data (sample_out)
    );

    // ------------------------------------------------------------------
    // I2S output -> headphone amp (PA_EN high = amplifier enabled)
    // ------------------------------------------------------------------
    assign pa_en = 1'b1;
    i2s_tx u_i2s (
        .clk (clk), .rstn (rstn),
        .sample_valid (sample_valid),
        .sample_data (sample_out),
        .bck (hp_bck), .lrck (hp_ws), .sdout (hp_din)
    );

    // ------------------------------------------------------------------
    // M4: Goertzel spectrum analyzer on the mixed output
    // ------------------------------------------------------------------
    wire [95:0] mag_bar;    // 12 x 8-bit bars FLAT: bin i = mag_bar[i*8 +: 8]
    wire       spec_stb;
`ifndef SIM_MINI
    goertzel_bank u_spec (
        .clk (clk), .rstn (rstn),
        .sample_valid (sample_valid), .sample_in (sample_out),
        .mag_bar (mag_bar), .frame_stb (spec_stb)
    );

    // ------------------------------------------------------------------
    // M4: ST7789V LCD with RGB332 framebuffer
    // ------------------------------------------------------------------
    wire        fb_we;
    wire [16:0] fb_addr;
    wire [7:0]  fb_color;
    wire        id_st7789, id_ili9341, id_any;
    // LCD driver. The init variant is LOCKED to ST7789 (lcd_var = 10,
    // chip_sel = 1) because the panel was proven to be an ST7789V on an STM32.
    // self_test = 0 -> the real visualisation (Lissajous + spectrum bars) is
    // shown instead of the colour wheel.
    st7789_drv u_lcd (
        .clk (clk), .rstn (rstn),
        .chip_sel (chip_sel), .inv_sel (inv_sel), .reinit (lcd_reinit),
        .wr_en (fb_we), .wr_addr (fb_addr), .wr_data (fb_color),
        .lcd_scl (lcd_scl), .lcd_sda (lcd_sda), .lcd_dc (lcd_dc),
        .lcd_cs (lcd_cs), .lcd_rst (lcd_rst), .lcd_bl (lcd_bl),
        .disp_ready (),
        .self_test (1'b0),      // real display: Lissajous + spectrum bars
        .lcd_sdo (lcd_sdo),
        .id_st7789 (id_st7789), .id_ili9341 (id_ili9341), .id_any (id_any)
    );

    // Status LEDs (active LOW on this board, and the silkscreen number runs
    // the opposite way to the bit index: what is printed LED0 is led[5]).
    // So a LED that is OFF is a 1 bit.
    //
    //   led[5:3] (printed LED0..2) = current timbre, 0..7
    //   led[2:0] (printed LED3..5) = active voice count, 0..7
    //
    // This is the documented encoding (README section 4). It was replaced on
    // by an MPR121 readout - touch flag / out-of-range flag /
    // electrode-0 difference - while the touch front-end was being brought
    // up. Touch now decides itself inside sensor_hub and the numbers are no
    // longer needed, so the LEDs go back to reporting the synth state.
    // active_cnt is 6 bits (up to 32 voices); the low three are shown, which
    // is enough to see "is anything sounding at all" and to count a chord.
    // ------------------------------------------------------------------
    // 蓝牙诊断灯：收到任何蓝牙字节就让丝印 LED0（led[5]）亮 ~25ms。
    // 用来二分定位"App 连上了但没反应"：
    //   按 App 琴键 LED0 规律地闪 3 下  = 模块→FPGA 通，问题在解析/音频
    //   LED0 完全不亮                  = 字节根本没到 FPGA（接线/波特率/bitstream）
    //   LED0 乱闪个不停                = 波特率对不上（收到的是乱码）
    // 只覆盖 led[5] 这一位，timbre / 复音数显示不受影响。
    // ------------------------------------------------------------------
    // 两位诊断灯，用来一眼分清"字节到了没"和"MIDI 解出来没"：
    //   丝印 LED0（led[5]）：收到任何字节就翻一下，持续 ~150 ms
    //   丝印 LED5（led[0]）：解出 note on / off 就翻一下，持续 ~400 ms
    // 组合判读：
    //   LED0 翻、LED5 也翻 -> 链路和解析都通，音符已经进引擎了
    //   LED0 翻、LED5 不翻 -> 字节是乱码（波特率不对），音符没解出来
    //   两个都不翻     -> 字节根本没到 FPGA（接线 / bitstream / 波特率）
    // 150 ms 而不是 25 ms：短到一眨眼就没了，用户根本来不及看。
    reg [22:0] bt_blink;
    always @(posedge clk or negedge rstn) begin
        if (!rstn)              bt_blink <= 23'd0;
        else if (bt_byte_v)     bt_blink <= 23'd6_000_000;   // ~148 ms @40.5 MHz
        else if (bt_blink != 0) bt_blink <= bt_blink - 23'd1;
    end
    reg [23:0] midi_blink;
    always @(posedge clk or negedge rstn) begin
        if (!rstn)                        midi_blink <= 24'd0;
        else if (midi_on || midi_off)     midi_blink <= 24'd16_000_000; // ~395 ms
        else if (midi_blink != 0)         midi_blink <= midi_blink - 24'd1;
    end

    // 关键：这里必须**取反**而不是固定写 0 或 1。
    // LED0 平时显示 timbre[2]，音色 0~3 时它本来就是亮的、音色 4~7 时本来是灭的。
    // 第一版固定写 0（亮），用户音色是 0~3 时就"一直常亮"、完全看不出变化。
    // 取反之后不管当前音色是多少，收到字节都会明显翻一下。
    wire [5:0] led_base = { timbre, active_cnt[2:0] };
    assign led[5]   = (bt_blink   != 0) ? ~led_base[5] : led_base[5];
    assign led[4:1] = led_base[4:1];
    assign led[0]   = (midi_blink != 0) ? ~led_base[0] : led_base[0];

    // ------------------------------------------------------------------
    // M4: WS2812B strip
    // ------------------------------------------------------------------
    wire        ws_we;
    wire [5:0]  ws_addr;
    wire [23:0] ws_color;
    wire        ws_frame_start;
    // 12 颗灯带，一颗灯 = 一个半音槽，
    // 引导模式下"第 i 颗亮 = 弹第 i 个键"，正好一一对应。
    ws2812_drv #(.NPIX(12)) u_ws (
        .clk (clk), .rstn (rstn),
        .wr_en (ws_we), .wr_addr (ws_addr), .wr_data (ws_color),
        .frame_start (ws_frame_start),
        .dout (ws_din)
    );
    // 板上自带的那颗 WS2812 跟外接灯带同源同数据 —— 它显示的是"第 0 颗"的颜色。
    // 零逻辑成本（只是把同一个信号再引到一个脚上），但排障价值很大：
    // 外接灯带全黑时，这一颗亮不亮就是"板子好 / 灯带或接线有问题"的分界线。
    assign ws_din_ob = ws_din;

    // ------------------------------------------------------------------
    // M4: visualization controller (scope + bars + LED content)
    // scope_mode / song_mode / led_mode are all toggled from the keypad.
    // ------------------------------------------------------------------
    // In song-guide mode the LED strip also lights the slot of the note the
    // melody is currently playing ("play this key next"), on top of whatever
    // the player is actually holding down.
    wire [11:0] note_active = touch12 | key_state[11:0] |
                              (song_mode ? song_slot : 12'd0);

    // packed parameter table for the LCD status panel: {p9..p0}
    wire [79:0] pvals = {pval[9], pval[8], pval[7], pval[6], pval[5],
                         pval[4], pval[3], pval[2], pval[1], pval[0]};

    display_ctrl u_disp (
        .clk (clk), .rstn (rstn),
        .sample_valid (sample_valid), .sample_in (sample_out),
        .mag_bar (mag_bar), .spec_stb (spec_stb),
        .note_active (note_active), .led_mode (led_mode),
        .scope_mode (scope_mode), .song_mode (song_mode),
        .song_prog (song_prog),
        .timbre (timbre), .oct_shift (oct_shift), .param_id (param_id),
        .pvals (pvals), .active_cnt (active_cnt),
        .diag_e0 (elec0), .diag_bl (bl0), .diag_df (elec_diff),
        .diag_neg (elec_neg), .diag_or (oor12),
        // the readout shows the calibrated deviation, so "P:000" really
        // means "not pressed" whichever way round the divider is wired
        .press8 (press_amt), .press_ok (press_cal),
        .fb_we (fb_we), .fb_addr (fb_addr), .fb_color (fb_color),
        .ws_we (ws_we), .ws_addr (ws_addr), .ws_color (ws_color),
        .ws_frame_start (ws_frame_start)
    );
    `endif

endmodule
`default_nettype wire
