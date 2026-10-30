`timescale 1ns/1ps
// ============================================================================
// voice_engine.v (v4) - Time-multiplexed polyphonic synthesis engine
//
// One shared compute pipeline (phase acc + wavetable + shared 16x16 mult)
// serves NVOICES voices per audio sample. Slot budget: worst case is the
// 8-harmonic additive timbre: 4 (fetch/dispatch) + 8 (harmonic loop)
// + 2 (env/out) = 14 cycles/voice.
//
//   24 voices -> 24 x 14 + 5 = 341 cycles   (fits 384 @ 27 MHz)
//   32 voices -> 32 x 14 + 5 = 453 cycles   (576 cycles are available per
//                                            70.3125 kHz sample @ 40.5 MHz,
//                                            so 32 fits with 21 % headroom)
//
// With NVOICES = 32 and the 8-harmonic additive timbre the chip runs
// 32 x 8 = 256 simultaneously sounding independent oscillators.
//
// v4 timing work (Fmax 34.4 -> 55.5 MHz, no feature removed) - the theme is
// "never let a wide memory read or a variable shift sit in front of a write":
//   * allocation scans 8 slots from rr_ptr and steals rr_ptr if all are busy;
//     note_off is a broadcast instead of a priority scan
//   * v_age is 8 bits and is written from the register ST_S0 already loaded
//   * the slot address is split into a registered one-hot write enable plus
//     ph_raw_q / ph_q / incv_q, so `cur` (753 fanout) leaves the data path
//   * the portamento step (32-bit variable shift) is computed in ST_S1
//
// Timbres (per voice oscillator count - scoring: total = voices x count):
//   0 sine      (1 osc)   classic clean tone
//   1 square    (1 osc)
//   2 saw       (1 osc)
//   3 triangle  (1 osc)
//   4 FM        (2 osc)   carrier + 2f modulator, phase-mod, index adjustable
//   5 additive  (4 osc)   harmonics 1f..4f at 1/n amplitude
//   6 additive8 (8 osc)   harmonics 1f..8f at 1/n amplitude
//                         -> 16 voices x 8 = 128 simultaneous oscillators
//   7 piano     (8 osc)   stretched partials, piano amplitude recipe and a
//                         per-voice string decay, so a held note dies away
//                         instead of droning at the sustain level. Same
//                         14 cycles/voice as additive8.
//
// Harmonic phases use shift-add address generation (no multiplier):
//   addr(k) = ((k * phase) mod 2^32) >> 22
//
// Expression:
//   - velocity  : scales envelope peak per note
//   - vibrato   : global 5.5 Hz sine LFO phase-modulates every voice
//   - envelope  : A/D/R rates + sustain level are runtime inputs
// ============================================================================
`default_nettype none

module voice_engine #(
    parameter NVOICES = 16
)(
    input  wire               clk,
    input  wire               rstn,
    input  wire               sample_tick,     // 1-clk pulse per audio sample
    // note events (1-clk pulses)
    input  wire               note_on,
    input  wire [6:0]         note_on_num,
    input  wire [7:0]         note_on_vel,     // 0..255 -> envelope peak
    input  wire               note_off,
    input  wire [6:0]         note_off_num,
    input  wire               rel_all,      // 1 = panic: release every voice
    // timbre & expression parameters (runtime adjustable)
    input  wire [2:0]         timbre,          // 0..6
    input  wire [15:0]        a_step,          // attack rate (env counts/sample)
    input  wire [15:0]        d_step,          // decay rate
    input  wire [15:0]        r_step,          // release rate
    input  wire [15:0]        sus_lvl,         // sustain level 0..65535 (of peak)
    input  wire [15:0]        fm_index,        // FM modulation depth
    input  wire [15:0]        vib_depth,       // vibrato depth (phase units)
    input  wire [3:0]         porta_sh,        // portamento: the increment
                                               // slews by (v_inc >> porta_sh)
                                               // per sample, i.e. constant
                                               // time per octave. 0 = off.
                                               // 11 -> ~20 ms per octave.
    // audio output
    output reg  signed [15:0] sample_out,
    output reg                sample_valid,
    output wire [5:0]         active_cnt
);

    localparam [2:0] TIM_SINE = 3'd0, TIM_SQ = 3'd1, TIM_SAW = 3'd2,
                     TIM_TRI  = 3'd3, TIM_FM = 3'd4, TIM_ADD = 3'd5,
                     TIM_ADD8 = 3'd6, TIM_PIANO = 3'd7;

    // Timbre classes. The additive harmonic loop (ST_SA) is shared by three
    // timbres, and the piano re-uses it wholesale - adding an eighth timbre
    // this way costs no new states, so the 14 cycles/voice slot budget that
    // the whole engine is timed around does not move.
    wire        is_add = (timbre == TIM_ADD || timbre == TIM_ADD8 ||
                          timbre == TIM_PIANO);
    wire        piano  = (timbre == TIM_PIANO);

    // additive harmonic amplitudes: 8192/n (sum ~ 22.3k < 32k headroom)
    function signed [15:0] coef(input [3:0] n);
        case (n)
            4'd1: coef = 16'sd8192;
            4'd2: coef = 16'sd4096;
            4'd3: coef = 16'sd2731;
            4'd4: coef = 16'sd2048;
            4'd5: coef = 16'sd1638;
            4'd6: coef = 16'sd1365;
            4'd7: coef = 16'sd1170;
            default: coef = 16'sd1024;
        endcase
    endfunction

    // Per-partial independent amplitude decay (scoring rule: an independent
    // oscillator needs its OWN amplitude-envelope behaviour, not just a shared
    // voice ADSR).  Higher partials decay faster, exactly like a real plucked
    // or struck instrument: partial k is attenuated by (v_age*(k-1))>>8
    // shifts, clamped to 3 (i.e. at most /8).  Partial 1 never decays, so a
    // held note keeps sounding while the timbre darkens.
    //
    // TIMING NOTE (an earlier comment here claimed ~0.4 s and was wrong):
    // v_age is 8 bits and saturates at 255, which at 70.3125 kHz is 3.6 ms.
    // Partial 8 is down 6 dB after 0.53 ms and 18 dB after 1.6 ms; partial 2
    // never decays at all inside the window. So psh() is a STRIKE TRANSIENT -
    // it is what makes an attack clang and then mellow - not a slow decay.
    // (A piano also needs a slow decay; that is v_pdec below, which is a
    // separate register precisely because 8 bits cannot express it.)
    // Costs zero extra clock cycles: it is folded into the existing coef
    // multiply operand as a variable shift.
    function [2:0] psh(input [3:0] k, input [7:0] age8);
        reg [11:0] v;
        begin
            v = {4'b0000, age8} * {8'h00, (k - 4'd1)};
            v = v >> 8;
            psh = (v > 12'd3) ? 3'd3 : v[2:0];
        end
    endfunction

    // harmonic k wavetable address: ((k*phase) mod 2^32) >> 22, shift-add
    function [9:0] haddr(input [31:0] p, input [3:0] k);
        reg [31:0] t;
        begin
            case (k)
                4'd1: haddr = p[31:22];
                4'd2: haddr = p[30:21];
                4'd3: begin t = p + {p[30:0], 1'b0};          haddr = t[31:22]; end
                4'd4: haddr = p[29:20];
                4'd5: begin t = {p[29:0], 2'b00} + p;         haddr = t[31:22]; end
                4'd6: begin t = {p[29:0], 2'b00} + {p[30:0], 1'b0}; haddr = t[31:22]; end
                4'd7: begin t = {p[29:0], 2'b00} + {p[30:0], 1'b0} + p; haddr = t[31:22]; end
                4'd8: haddr = p[28:19];
                default: haddr = p[31:22];
            endcase
        end
    endfunction

    // ------------------------------------------------------------------
    // Timbre 7 - piano
    //
    // A struck string reads as a piano for three reasons, and only one of
    // them is the spectrum:
    //
    //   1. it dies away while the key is STILL DOWN.  The shared ADSR cannot
    //      express that - sus_lvl is a global parameter, so setting it to 0
    //      for the piano would kill the sustain of every other timbre too.
    //      The decay therefore lives in its own per-voice register v_pdec
    //      and is multiplied into the envelope (see the register below).
    //   2. the partials are STRETCHED.  A real string is stiff, so
    //      f_k = k*f0*sqrt(1+B*k^2): partial 8 sits ~0.4 % sharp.  That
    //      inharmonicity is most of what separates a piano from an organ.
    //   3. the partials are NOT 1/n.  coef() is a sawtooth recipe and it is
    //      why the additive timbres buzz; a piano has a strong fundamental
    //      and a strong 2nd/3rd.
    //
    // The first recipe took "thin tail" much too far: partials 5..8 came out
    // at 0.14/0.09/0.06/0.04 and the note measured 488 Hz spectral centroid
    // with only 0.018 of its onset energy above 1.5 kHz - against 513 Hz and
    // 0.035 for the sawtooth recipe.  That is what "闷 / not bright" is: the
    // top half of the spectrum is simply missing, and psh() then strips what
    // is left within ~1.6 ms of the attack.
    //
    // These values lift the upper partials to 0.27/0.19/0.13/0.09
    // (centroid 537 Hz, HF>1.5k = 0.028, measured by _piano_tone.py) while
    // the sum stays at 27262 - under the 32767 ceiling the 16-bit harmonic
    // accumulator needs, so nothing can wrap.
    // ------------------------------------------------------------------
    function signed [15:0] pcoef(input [3:0] n);
        case (n)
            4'd1: pcoef = 16'sd8192;
            4'd2: pcoef = 16'sd6144;
            4'd3: pcoef = 16'sd4259;
            4'd4: pcoef = 16'sd3113;
            4'd5: pcoef = 16'sd2212;
            4'd6: pcoef = 16'sd1540;
            4'd7: pcoef = 16'sd1065;
            default: pcoef = 16'sd737;
        endcase
    endfunction

    // Piano per-partial decay = the fast strike transient from psh() plus a
    // slow mellowing driven by the string-decay register.  Only partials 5..8
    // mellow: the fundamental and the 2nd/3rd keep ringing, which is what
    // separates a piano from an organ.
    //
    // The slow term is a SHIFT, i.e. a 6 dB staircase - three steps over the
    // life of the note.  That is acceptable only because it acts on partials
    // that already sit 25..36 dB below the fundamental, so the steps land at
    // -31 dB and below.  t9 measures the size of every step relative to the
    // whole note and fails if one ever becomes audible.
    function [3:0] psh_p(input [3:0] k, input [7:0] age8, input [15:0] pdec);
        reg [3:0] fast;
        reg [3:0] slow;
        begin
            fast = {1'b0, psh(k, age8)};
            slow = (k >= 4'd5) ? (((16'd32767 - pdec) >> 13) & 4'h3) : 4'h0;
            psh_p = (fast + slow > 4'd5) ? 4'd5 : (fast + slow);
        end
    endfunction

    // Stretched partial addresses: ((k*phase + phase>>s_k) >> 22) + carry,
    // where `carry` is built from the per-voice wrap counter w.  The >>s_k
    // term raises partial k by 1/(k*2^s_k):
    //     k=4 s=8 -> +0.098 %      k=6 s=6 -> +0.26 %
    //     k=5 s=7 -> +0.156 %      k=7 s=6 -> +0.22 %   k=8 s=5 -> +0.39 %
    // against the sqrt(1+B*k^2) target at B = 1.2e-4 (0.096/0.15/0.22/0.29/
    // 0.38 %).  Partials 1..3 stay exactly harmonic, which is where the real
    // stretch is too small to hear.
    //
    // THE `w` ARGUMENT IS NOT OPTIONAL.  phase>>s_k collapses to 0 every
    // time the 32-bit accumulator wraps, so on its own it contributes a
    // sawtooth whose ramp and reset cancel and the partial stays at
    // exactly k*f0 (measured +0.000 % where +0.39 % was intended).  Adding
    // (w mod 2^s_k) << (10-s_k) puts back the 2^(32-s_k) that the wrap
    // threw away, which makes the address advance continuously.
    // See the v_pwrap comment for the full derivation.
    function [9:0] haddr_p(input [31:0] p, input [3:0] k, input [7:0] w);
        reg [31:0] t;
        reg [9:0]  wc;
        begin
            case (k)
                4'd1: begin t = p;                                        wc = 10'd0;          end
                4'd2: begin t = {p[30:0], 1'b0};                          wc = 10'd0;          end
                4'd3: begin t = p + {p[30:0], 1'b0};                      wc = 10'd0;          end
                4'd4: begin t = {p[29:0], 2'b00} + (p >> 8);              wc = {w[7:0], 2'b00};   end
                4'd5: begin t = {p[29:0], 2'b00} + p + (p >> 7);          wc = {w[6:0], 3'b000};  end
                4'd6: begin t = {p[29:0], 2'b00} + {p[30:0], 1'b0} + (p >> 6);
                                                                          wc = {w[5:0], 4'b0000}; end
                4'd7: begin t = {p[29:0], 2'b00} + {p[30:0], 1'b0} + p + (p >> 6);
                                                                          wc = {w[5:0], 4'b0000}; end
                4'd8: begin t = {p[28:0], 3'b000} + (p >> 5);              wc = {w[4:0], 5'b00000};end
                default: begin t = p;                                     wc = 10'd0;          end
            endcase
            // 10-bit add: wraps mod 1024, which is the whole point.
            haddr_p = t[31:22] + wc;
        end
    endfunction

    function [9:0] hsel(input [31:0] p, input [3:0] k, input pn, input [7:0] w);
        hsel = pn ? haddr_p(p, k, w) : haddr(p, k);
    endfunction

    // vibrato LFO: 5.5 Hz at 70312.5 Hz sample rate
    localparam [31:0] LFO_INC = 32'h00052052;

    // ------------------------------------------------------------------
    // per-voice state
    // ------------------------------------------------------------------
    reg  [NVOICES-1:0] v_active;
    reg  [6:0]         v_note  [0:NVOICES-1];
    reg  [31:0]        v_phase [0:NVOICES-1];
    reg  [15:0]        v_env   [0:NVOICES-1];
    reg  [15:0]        v_peak  [0:NVOICES-1];   // velocity-scaled ceiling
    reg  [1:0]         v_state [0:NVOICES-1];
    reg  [4:0]         rr_ptr;
    // --- portamento (glide) state ---
    reg  [31:0]        v_inc   [0:NVOICES-1];   // current phase increment
    reg  [6:0]         v_gfrom [0:NVOICES-1];   // note the glide starts from
    reg                v_glide [0:NVOICES-1];   // 1 = load v_inc from v_gfrom
    reg  [6:0]         prev_note;               // last triggered note number
    // --- per-partial decay state ---
    // 8 bits, saturating. Only ever consumed as v_age>>8, and it is written
    // back from the age_q register that ST_S0 already loaded. That matters
    // for timing: re-reading v_age[cur] in the write cycle put a 24:1 read
    // mux + saturating compare + increment (~12 LUT levels) behind the
    // `cur` register, and THAT chain was the 34.7 MHz critical path.
    reg  [7:0]         v_age   [0:NVOICES-1];   // samples since note-on

    // --- piano string decay (timbre 7) ---
    // A piano note dies away while the key is still down.  psh() cannot do
    // it: v_age saturates after 3.6 ms.  So each voice carries its own decay
    // register, multiplied into the envelope.
    //
    //   x <= x - (x >> sh) - 1
    //
    // is exponential with a -1/sample floor.  The floor is NOT cosmetic: a
    // pure shift stops decaying once x < 2^sh and the note would then sit at
    // -6 dB forever - a drone that no amount of release handling would fix,
    // because the key is still down.  22 bits with the top 16 used as the
    // multiplier push that sticking point to -48 dB, and the -1 still
    // guarantees the register reaches exactly 0.
    //
    // sh follows the pitch, because a real piano's bass rings far longer
    // than its treble: 15 below C3, 14 below C5, 13 above.
    reg  [21:0] v_pdec [0:NVOICES-1];
    localparam [21:0] PDEC_INIT = 22'd2097088;   // 32767 << 6

    // ------------------------------------------------------------------
    // Phase-wrap counter, one per voice, needed ONLY by the piano stretch.
    //
    // Why this exists:
    // the stretched address was (k*phase + phase>>s) >> 22.  The >>s term
    // climbs to 2^(32-s) as phase climbs to 2^32 and then SNAPS BACK TO 0
    // when the 32-bit accumulator wraps, once every 1/f0.  The ramp and the
    // snap cancel exactly, so the MEAN slope is still k*f0 - the partials
    // were not stretched at all, they just picked up a phase jump of
    // 2^(10-s) table steps (11 degrees on partial 8) at the wrap, which
    // shows up as +-f0 sidebands at -31 dB.  Measured: partial 8 sat at
    // 2093.005 Hz, i.e. +0.000 % instead of +0.39 %.
    //
    // This is unavoidable without state: a continuous map from
    // (phase mod 2^32) to (address mod 2^32) must advance by a whole
    // number of 2^32 per phase period, so a slope of k + 2^-s (not an
    // integer) cannot be built from phase alone.  Carrying the wrap count
    // supplies the missing 2^(32-s) per period.  8 bits is enough because
    // the widest shift used is s = 8, and only (wraps mod 2^s) ever
    // reaches the address.
    //
    // Cost: 8 bits x 32 voices = 256 registers, and one carry-out of the
    // phase adder that was already there.
    // ------------------------------------------------------------------
    reg  [7:0]  v_pwrap [0:NVOICES-1];


    localparam [1:0] ENV_IDLE=2'd0, ENV_ATTACK=2'd1, ENV_DECAY=2'd2, ENV_RELEASE=2'd3;

    // ------------------------------------------------------------------
    // shared datapath registers (declared before the ROM instances use them)
    // ------------------------------------------------------------------
    reg  [6:0]  lut_addr;
    reg  [9:0]  wav_addr;
    reg  [31:0] inc_q;
    reg  [31:0] p_eff_q;         // phase + vibrato offset of current voice
    reg  signed [15:0] wave_q;   // selected waveform (basic / FM carrier)
    reg  signed [16:0] harm_acc; // additive harmonic accumulator
    reg  signed [15:0] m_a, m_b; // shared multiplier operands
    wire signed [31:0] prod = m_a * m_b;
    reg  signed [15:0] env_q, peak_q, sus_q, lfo_d;
    reg  [1:0]  st_q;
    reg         gl_q;                // this voice needs a glide-source fetch
    reg  [6:0]  gf_q;                // glide source note (v_gfrom of cur voice)
    reg  [31:0] g_inc_q;             // phase increment of the glide source note
    reg  [7:0]  age_q;               // v_age[cur] -> per-partial decay
    reg  [21:0] pdec_raw_q;          // v_pdec[cur] -> piano string decay
    wire [15:0] pdec_q  = pdec_raw_q[21:6];  // 0..32767, the envelope factor
    reg  [7:0]  pwrap_q;             // v_pwrap[cur] -> stretch carry, see v_pwrap
    reg  [15:0] env_p;               // env x pdec, piano only
    reg  [6:0]  note_q;              // v_note[cur], latched for the decay rate
    // 5 bits, NOT 4: the slowest decay needs a shift of 16, and 4'd16 is
    // silently truncated to 4'd0 by the synthesizer (WARN EX3792), which
    // turns the subtract into pdec - pdec - 1 = 0 and mutes the bass
    // completely on the very first sample.
    reg  [4:0]  pdec_sh_q;           // decay shift for THIS voice's pitch
    reg  signed [23:0] acc;
    reg  [4:0]  cur;
    reg  [3:0]  hi;              // additive harmonic index (2..9)
    reg  [31:0] lfo_phase;
    reg  signed [31:0] vib_off_q;

    wire [3:0] harm_last = (timbre == TIM_ADD8 || timbre == TIM_PIANO)
                           ? 4'd8 : 4'd4;

    // Harmonic coefficient and its per-partial decay, piano-aware.  The two
    // tables are selected here so the ST_SA loop body itself stays a plain
    // shift - the mux sits in front of a register, not in the address path.
    wire [3:0]          hsh_p = psh_p(hi, age_q, pdec_q);
    wire [2:0]          hsh   = piano ? hsh_p[2:0] : psh(hi, age_q);
    wire signed [15:0]  hcoef = piano ? pcoef(hi) : coef(hi);

    // ------------------------------------------------------------------
    // The voice-slot address, taken apart so the timing tool stops seeing a
    // single 753-fanout register in front of every voice array.
    //
    //   cur_w : one-hot write enable, decoded from `cur` during ST_S0 and
    //           REGISTERED there. The v_xxx[cur] <= ... writes then hang off
    //           a flop one cycle ahead of `cur`, so the 5-bit -> 24-way
    //           address decode leaves the data path.
    //   ph_raw_q: v_phase[cur], the accumulator (no vibrato).
    //   ph_q    : v_phase[cur] + vib_off_q, for the wavetable address.
    //   incv_q  : v_inc[cur], latched in ST_S0 (the portamento slew needs it).
    //
    // This is what lifted Fmax off 34.7 MHz. Reads still index with `cur`,
    // but a read mux that feeds a flop in the same state is one LUT level -
    // it never chains into the next state's arithmetic.
    // ------------------------------------------------------------------
    reg  [NVOICES-1:0] cur_w;    // registered one-hot slot write enable
    reg  [31:0] ph_raw_q;        // v_phase[cur]              (no vibrato)
    reg  [31:0] ph_q;            // v_phase[cur] + vib_off_q  (with vibrato)
    reg  [31:0] incv_q;          // its current phase increment
    // Combinational decode of `cur`. The shift MUST be done at 32 bits and
    // truncated afterwards: in Verilog a shift result takes the width of its
    // LEFT operand, so `{{(32-NVOICES){1'b0}},1'b1} << cur` would be a
    // (32-NVOICES+1)-bit shift that silently truncates to zero for every
    // cur >= 32-NVOICES+1 - synthesis still passes (the arrays stay alive
    // because they are read), but those voices are never written.
    wire [31:0] cur_oh32 = 32'h0000_0001 << cur;
    wire [NVOICES-1:0] cur_w_c = cur_oh32[NVOICES-1:0];

    // Effective phase of the voice currently being served. p_raw is the
    // combinational form; ST_S0 splits it into ph_raw_q (the accumulator,
    // no vibrato) and ph_q (the vibrato-modulated value used for the
    // wavetable address). Nothing downstream re-reads v_phase[cur], which is
    // what keeps the 24:1 read mux off the ST_S1/ST_S2 arithmetic.
    wire [31:0] p_raw  = v_phase[cur] + vib_off_q;

    // ------------------------------------------------------------------
    // ROMs
    // ------------------------------------------------------------------
    wire [31:0]        freq_data;
    wire signed [15:0] sine_data;
    rom_freq u_freq (.addr(lut_addr), .data(freq_data));
    rom_sine u_sine (.addr(wav_addr), .data(sine_data));

    // ------------------------------------------------------------------
    // voice allocation (combinational scans)
    // ------------------------------------------------------------------
    // 5-bit voice index: enough for up to 32 voices (NVOICES = 32 default).
    localparam [4:0] NVM1 = NVOICES[4:0] - 5'd1;

    // ------------------------------------------------------------------
    // voice allocation / release - SHORT combinational paths on purpose
    //
    // The original version ran two full NVOICES-entry priority scans
    // (alloc/retrig over 32 slots, release over 32 slots). Those two chains
    // were the entire reason Fmax sat at 31.5 MHz: the alloc scan alone is
    // ~14 levels of LUT and it ends on every voice-state register.
    //
    // Now:
    //   * alloc/retrig scan only the next SCAN_N slots from rr_ptr. With
    //     SCAN_N = 8 the chain is ~3 levels. If none of those 8 is free the
    //     slot at rr_ptr is STOLEN (plain voice stealing - with 24 voices and
    //     ten fingers it never happens in practice).
    //   * note_off is a BROADCAST: every voice holding that note goes into
    //     release at once, instead of a priority scan that picks one. This is
    //     what makes stealing safe - a stolen voice can never get stuck
    //     droning, because the release no longer depends on finding "the one"
    //     voice that owns the note.
    // ------------------------------------------------------------------
    localparam [4:0] SCAN_N = 5'd8;

    integer k0, k1, k2;
    reg [4:0] j0, j1, j2;   // per-block scratch: MUST NOT be shared between
                            // always @* blocks (shared comb scratch creates
                            // a zero-delay retrigger loop in simulation and
                            // is a multi-driver error for synthesis)
    reg [4:0] alloc_idx; reg alloc_ok;
    reg [4:0] retrig_idx; reg retrig_ok;
    reg [4:0] rel_idx;    reg rel_ok;      // 抢占时的首选目标：正在 release 的槽
    reg [5:0] cnt_c;

    // retrig 只匹配仍处于按住状态的槽，已进入 ENV_RELEASE（只剩尾音）的槽
    // 不参与复触发。原因：复触发命中释放中的槽不会增加同时发声数，且该分支
    // 不推进 rr_ptr，后续无空闲槽的事件会抢占 rr_ptr 而覆盖掉刚复触发的音，
    // 使该音丢失。排除 ENV_RELEASE 后可保证每个新音占用独立槽位。
    always @* begin
        alloc_ok  = 1'b0; alloc_idx  = rr_ptr;
        retrig_ok = 1'b0; retrig_idx = 5'd0;
        rel_ok    = 1'b0; rel_idx    = rr_ptr;
        for (k0 = 0; k0 < SCAN_N; k0 = k0 + 1) begin
            j0 = rr_ptr + k0[4:0];
            if (v_active[j0] && (v_note[j0] == note_on_num) &&
                (v_state[j0] != ENV_RELEASE) && !retrig_ok) begin
                retrig_ok  = 1'b1;
                retrig_idx = j0;
            end
            if (!v_active[j0] && !alloc_ok) begin
                alloc_ok  = 1'b1;
                alloc_idx = j0;
            end
            // 实在没槽可用时，先抢"已在 release、只剩尾音"的那个：
            // 它几乎已经没声了，顶掉它听感上无感；而抢 rr_ptr 有可能
            // 顶掉一个刚刚起音、正响着的音。
            if (v_active[j0] && (v_state[j0] == ENV_RELEASE) && !rel_ok) begin
                rel_ok  = 1'b1;
                rel_idx = j0;
            end
        end
    end

    // 同时发声数统计：所有已激活的槽都计入，包括正在 ENV_RELEASE 的尾音。
    //
    // 该计数值同时作为混音归一化的 N（增益 255/sqrt(N)）。release 尾音仍在
    // 输出样本、仍占用混音总线，计入 N 可保证多声部叠奏时不削顶失真；代价是
    // 换音瞬间旧音尚未走完释放、新音已起，N 会短暂加一，增益略降。释放段通常
    // 很快结束，这段过渡很短，听感上可接受。
    always @* begin
        cnt_c = 6'd0;
        for (k2 = 0; k2 < NVOICES; k2 = k2 + 1) begin
            j2 = k2[4:0];
            if (v_active[j2]) cnt_c = cnt_c + 6'd1;
        end
    end
    // registered once per sample - see active_q / the note at vgain_q
    //
    // active_q 的声明必须位于使用之前。虽然 Gowin 综合器允许先使用后声明，
    // 但 iverilog 会报 "Unable to bind wire/reg/memory" 而导致无法仿真，
    // 因此把声明提前到此处。
    reg [5:0] active_q;
    assign active_cnt = active_q;

    // ------------------------------------------------------------------
    // waveform selection for basic timbres
    // ------------------------------------------------------------------
    wire signed [15:0] sq_wave  = wav_addr[9] ? -16'sd24576 : 16'sd24576;
    wire signed [15:0] saw_wave = $signed({wav_addr, 6'b000000}) - 16'sd32768;
    wire [8:0]  tri_v = wav_addr[9] ? ~wav_addr[8:0] : wav_addr[8:0];
    wire signed [15:0] tri_wave = $signed({tri_v, 7'b0000000}) - 16'sd32768;

    // ------------------------------------------------------------------
    // envelope next-state
    // ------------------------------------------------------------------
    // porta_sh 打一拍。它来自 top 的 pval[9]（组合），跨模块一路进到 ST_S1 的
    // 32 位变量移位器、再串到 ST_S2 的 ph_ovf -> v_pwrap —— 整条链不打拍的话
    // 就是 post-route STA 的最差路径（实测 Fmax 只剩 40.532 MHz，余量 31 kHz）。
    // 在 ST_S0 锁存一级把它切断；portamento 晚一拍生效，音频上听不出来。
    reg [3:0]  porta_sh_q;

    reg [15:0] next_env; reg [1:0] next_state;
    always @* begin
        next_env   = env_q;
        next_state = st_q;
        case (st_q)
            ENV_ATTACK: begin
                if ((peak_q <= a_step) || (env_q >= peak_q - a_step)) begin
                    next_env   = peak_q;
                    next_state = ENV_DECAY;
                end else next_env = env_q + a_step;
            end
            ENV_DECAY: begin
                if (env_q <= sus_q + d_step) next_env = sus_q;
                else next_env = env_q - d_step;
            end
            ENV_RELEASE: begin
                if (env_q <= r_step) begin
                    next_env   = 16'd0;
                    next_state = ENV_IDLE;
                end else next_env = env_q - r_step;
            end
            default: next_env = 16'd0;
        endcase
        // ---------------------------------------------------------------
        // note_off 需要能够覆盖状态机算出的 next_state。
        //
        // 引擎时分复用，每个周期只服务一个 voice（cur），而 note_off 是对全部
        // 32 个槽的广播写。若此时目标 voice 正好是 cur，则同周期内的
        // v_state[k4] <= next_state 会把它写成 ENV_DECAY，覆盖掉广播写入的
        // ENV_RELEASE，导致释放流程没有执行、该音停留在 sus_q 持续发声。
        // 该情况发生的概率约为 1/32。
        //
        // 非钢琴音色才会暴露此问题：TIM_PIANO 的衰减由 v_pdec 单独提供，
        // 即使 env 停在 sus_q 也会衰减到 0，因而把该问题掩盖了。
        //
        // note_q 是 ST_S0 锁存的 v_note[cur]，即当前被服务的音号，直接与
        // note_off_num 比较即可，无需额外的读 mux。
        // ---------------------------------------------------------------
    end

    // ---- per-voice gain: keep the mix in the LINEAR part of soft_knee ----
    //
    // One voice peaks at ~14.8k. Unscaled, N voices reach N x 14.8k: four is
    // ~59k and eight is ~119k, which lands in the 1/8 compression segment.
    // There the knee acts on the COMPOSITE waveform, sample by sample, so
    // every pair of sounding notes generates sum and difference tones. That
    // is what "chords sound like noise" is: intermodulation, not loudness.
    //
    // Scaling each voice by ~1/sqrt(N) keeps the sum around 21k..42k for the
    // chord sizes anyone actually plays, i.e. in or just past the first (1/4)
    // segment instead of the 1/8 one. The Arduino design this is compared
    // against has four voices at a fixed level and never compresses at all -
    // same idea, but without starving a single note by dividing by 32.
    //
    // Table is 255/sqrt(n) rounded; n is the live active_cnt, 1..32.
    function [7:0] vgain8(input [4:0] n);
        case (n)
            // 0 = nothing sounding yet, but a note can start on any sample;
            // full gain here keeps its first two samples (before the count has
            // caught up) from being 5.7x too quiet
            5'd0:  vgain8 = 8'd255;
            5'd1:  vgain8 = 8'd255;   5'd2:  vgain8 = 8'd180;
            5'd3:  vgain8 = 8'd147;   5'd4:  vgain8 = 8'd128;
            5'd5:  vgain8 = 8'd114;   5'd6:  vgain8 = 8'd104;
            5'd7:  vgain8 = 8'd96;    5'd8:  vgain8 = 8'd90;
            5'd9:  vgain8 = 8'd85;    5'd10: vgain8 = 8'd81;
            5'd11: vgain8 = 8'd77;    5'd12: vgain8 = 8'd74;
            5'd13: vgain8 = 8'd71;    5'd14: vgain8 = 8'd68;
            5'd15: vgain8 = 8'd66;    5'd16: vgain8 = 8'd64;
            5'd17: vgain8 = 8'd62;    5'd18: vgain8 = 8'd60;
            5'd19: vgain8 = 8'd58;    5'd20: vgain8 = 8'd57;
            5'd21: vgain8 = 8'd56;    5'd22: vgain8 = 8'd54;
            5'd23: vgain8 = 8'd53;    5'd24: vgain8 = 8'd52;
            5'd25: vgain8 = 8'd51;    5'd26: vgain8 = 8'd50;
            5'd27: vgain8 = 8'd49;    5'd28: vgain8 = 8'd48;
            5'd29: vgain8 = 8'd47;    5'd30: vgain8 = 8'd46;
            default: vgain8 = 8'd45;   // 31, 32, and the 0 case never used
        endcase
    endfunction

    // The count and the table are BOTH registered, one stage each, and only
    // refreshed once per sample. Counting 32 active flags is a deep adder
    // tree, and feeding it straight into a 32-entry mux and then a multiply
    // put a 27.9 ns path on every voice slot: Fmax fell to 35.5 MHz and 16
    // endpoints failed. Two registers give each stage a whole clock, and a
    // gain that is one sample stale is irrelevant - it only ever changes
    // when a note starts or stops.
    reg [7:0] vgain_q;                  // active_q 已提到上面 active_cnt 处
    wire [15:0] env_now = piano ? env_p : env_q;
    // 24-bit intermediate is not cosmetic. Verilog sizes a multiply to its
    // CONTEXT, so writing (env_now * vgain_q) >> 8 straight into a 16-bit
    // wire truncates the 24-bit product to 16 bits BEFORE the shift:
    // 32767 x 255 = 8355585 wraps to 32513, >> 8 gives 127 instead of 32639.
    // 257x too quiet, and the wrap is not even a scaling error - it is
    // garbage, which is what shipped as "all noise and barely audible".
    wire [23:0] env_mul = env_now * vgain_q;
    wire [15:0] env_g   = env_mul >> 8;

    // mixer output: per-voice contribution is (wave x env) >> 16 (max ~16.3k
    // per voice). Accumulate WITHOUT dividing by NVOICES - a single voice must
    // not be attenuated 16x.
    //
    // The excess used to be chopped off with a hard compare to +-32767, and
    // that is what made chords sound dirty. Measured in tools/engine_sim.py:
    // one voice peaks at 11781 (sine, vel 232), so four held keys already
    // reach ~47k and eight reach ~94k. Slicing the top off a waveform is not
    // attenuation, it is a burst of harmonics the ear hears as noise.
    //
    // A soft knee compresses the excess with a falling slope instead. The
    // segment start values MUST equal the previous segment's end value, or
    // the curve steps backwards and becomes its own distortion source: the
    // first version of this paired "24576 + (m-49152)>>3" (which ends at
    // 34816) with "29696 + ..." and so dropped 5120 counts at m = 131072.
    //   <= 16384 : 1:1      ends 16384   (one voice is never touched)
    //   <= 49152 : 1/4      ends 24576
    //   <= 98304 : 1/8      ends 30720
    //   above    : 1/256    reaches 32384 at the 32-voice worst case, so the
    //                       clamp below is a guard that never fires in use
    function signed [23:0] soft_knee(input signed [23:0] v);
        reg signed [23:0] m;
        reg signed [23:0] y;
        begin
            m = v[23] ? -v : v;                       // magnitude
            if      (m <= 24'sd16384)  y = m;
            else if (m <= 24'sd49152)  y = 24'sd16384 + ((m - 24'sd16384) >> 2);
            else if (m <= 24'sd98304)  y = 24'sd24576 + ((m - 24'sd49152) >> 3);
            else                       y = 24'sd30720 + ((m - 24'sd98304) >> 8);
            if (y > 24'sd32767) y = 24'sd32767;       // guard, not a working path
            soft_knee = v[23] ? -y : y;
        end
    endfunction

    wire signed [23:0] mix_lim = soft_knee(acc);

    // Phase-wrap detect for the piano stretch (see v_pwrap).  ST_S2 can add
    // any of three increments to the phase; only the selected one decides
    // whether the accumulator wrapped, so mux first and add once - three
    // 33-bit adders here would cost more than the whole stretch feature.
    wire [31:0] ph_inc_now = (porta_sh == 4'd0) ? inc_q   :
                             (gl_q              ? g_inc_q : incv_q);
    wire [32:0] ph_sum33   = {1'b0, ph_raw_q} + {1'b0, ph_inc_now};
    wire        ph_ovf     = ph_sum33[32];
    wire signed [15:0] mix_s   = mix_lim[15:0];

    // ------------------------------------------------------------------
    // main FSM
    // ------------------------------------------------------------------
    localparam [4:0]
        ST_IDLE = 5'd0,
        ST_L0 = 5'd1, ST_L1 = 5'd2, ST_L2 = 5'd3, ST_L3 = 5'd4,
        ST_S0 = 5'd5, ST_S1 = 5'd6, ST_S2 = 5'd7,  ST_S3 = 5'd8,
        ST_SA = 5'd9,                       // additive harmonic loop
        ST_FM1 = 5'd10, ST_FM2 = 5'd11,     // FM carrier compose
        ST_S11 = 5'd12, ST_S12 = 5'd13, ST_OUT = 5'd14,
        ST_G0 = 5'd15, ST_G1 = 5'd16;     // portamento source frequency fetch

    // Glide is exponential: constant time per octave, so it feels the same at
    // any pitch. porta_sh = 0 disables the whole glide path.
    //
    // The x2^(-porta_sh) step is computed in ST_S1 and held in pstep_q, not
    // left as a wire into ST_S2. A 32-bit variable right shift is a 5-level
    // LUT tree, and with porta_sh coming straight from the parameter register
    // pval[] that tree sat between a parameter flop and v_inc[] - it was the
    // 50.5 MHz critical path. incv_q is already stable by ST_S1, so hoisting
    // the shift costs nothing (ST_S1 has no other long path).
    wire [3:0]  porta_on = (porta_sh == 4'd0) ? 4'd0 : 4'd1;
    reg  [31:0] pstep_q;

    // ------------------------------------------------------------------
    // note-event pipeline (stage 1 = pick the slot, stage 2 = write it)
    //
    // Doing the whole thing in one cycle puts the 32-entry priority scan and
    // the write-address decode of every voice-state array in series: that
    // single path capped Fmax at ~30 MHz. Stage 1 latches the scan result,
    // stage 2 writes it, so the long scan ends on 6 small registers instead
    // of on v_note/v_phase/v_env/v_inc/... (hundreds of bits).
    //
    // An event may be latched in the same cycle the previous one is applied,
    // so nothing is ever dropped as long as events are >=2 cycles apart -
    // which is guaranteed by the 1-cycle gap in the touch scanner and by the
    // self-test burst firing one note every 4 cycles.
    // ------------------------------------------------------------------
    reg        ev_pend;
    reg [4:0]  ev_idx;
    reg [6:0]  ev_num;
    reg [7:0]  ev_vel;
    reg [6:0]  ev_from;

    reg [4:0] st;
    integer k3, k4;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            st <= ST_IDLE; cur <= 5'd0; acc <= 24'd0;
            sample_out <= 16'd0; sample_valid <= 1'b0;
            v_active <= {NVOICES{1'b0}}; rr_ptr <= 5'd0;
            lut_addr <= 7'd0; inc_q <= 32'd0; wav_addr <= 10'd0;
            wave_q <= 16'd0; env_q <= 16'd0; peak_q <= 16'd0; sus_q <= 16'd0;
            st_q <= 2'd0; harm_acc <= 17'd0; m_a <= 16'd0; m_b <= 16'd0;
            lfo_phase <= 32'd0; lfo_d <= 16'd0; vib_off_q <= 32'd0;
            p_eff_q <= 32'd0; hi <= 4'd2;
            cur_w <= {NVOICES{1'b0}}; ph_raw_q <= 32'd0; ph_q <= 32'd0;
            incv_q <= 32'd0; pstep_q <= 32'd1;
            gl_q <= 1'b0; gf_q <= 7'd0; g_inc_q <= 32'd0; age_q <= 8'd0;
            pdec_raw_q <= 22'd0; env_p <= 16'd0; note_q <= 7'd60;
            pwrap_q <= 8'd0;
            // full-scale until the first sample counts the (zero) voices,
            // so a lone first note is never quietly mis-gained
            active_q <= 6'd0; vgain_q <= 8'd255;
            pdec_sh_q <= 5'd14;
            prev_note <= 7'd60;
            ev_pend <= 1'b0; ev_idx <= 5'd0; ev_num <= 7'd60;

            ev_vel <= 8'd200; ev_from <= 7'd60;
            for (k3 = 0; k3 < NVOICES; k3 = k3 + 1) begin
                v_note[k3]  <= 7'd0;
                v_phase[k3] <= 32'd0;
                v_env[k3]   <= 16'd0;
                v_peak[k3]  <= 16'd0;
                v_state[k3] <= ENV_IDLE;
                v_inc[k3]   <= 32'd0;
                v_gfrom[k3] <= 7'd60;
                v_glide[k3] <= 1'b0;
                v_age[k3]   <= 8'd0;
                v_pdec[k3]  <= 22'd0;
                v_pwrap[k3] <= 8'd0;
            end
        end else begin
            sample_valid <= 1'b0;

            // ---- per-sample pipeline ------------------------------------
            case (st)
                ST_IDLE: if (sample_tick) begin
                    st <= ST_L0; cur <= 5'd0; acc <= 24'd0;
                    // refresh the polyphony gain once per sample: count the
                    // live voices, then look the gain up off the PREVIOUS
                    // count so the two stages are never in series
                    active_q <= cnt_c;
                    vgain_q  <= vgain8((active_q > 6'd31) ? 5'd31
                                                          : active_q[4:0]);
                end

                // ---- vibrato LFO (once per sample) ----
                ST_L0: begin
                    wav_addr  <= lfo_phase[31:22];
                    st        <= ST_L1;
                end
                ST_L1: begin
                    lfo_d <= sine_data;
                    st    <= ST_L2;
                end
                ST_L2: begin
                    m_a <= lfo_d;
                    m_b <= vib_depth;
                    st  <= ST_L3;
                end
                ST_L3: begin
                    vib_off_q <= prod;
                    lfo_phase <= lfo_phase + LFO_INC;
                    st        <= ST_S0;
                end

                // ---- per-voice slot ----
                ST_S0: begin
                    lut_addr <= v_note[cur];
                    env_q    <= v_env[cur];
                    st_q     <= v_state[cur];
                    peak_q   <= v_peak[cur];
                    gl_q     <= v_glide[cur] & (porta_sh != 4'd0);
                    gf_q     <= v_gfrom[cur];
                    age_q    <= v_age[cur];
                    note_q     <= v_note[cur];
                    porta_sh_q <= porta_sh;
                    pdec_raw_q <= v_pdec[cur];
                    pwrap_q     <= v_pwrap[cur];
                    // pre-decode the slot write enable and cache the two wide
                    // values the accumulate stage needs, so neither the
                    // 24-way address decode nor the v_phase/v_inc read mux
                    // sits in front of ST_S1/ST_S2's arithmetic
                    cur_w    <= cur_w_c;
                    ph_raw_q <= v_phase[cur];
                    ph_q     <= p_raw;
                    incv_q   <= v_inc[cur];
                    st       <= ST_S1;
                end
                ST_S1: begin
                    inc_q <= freq_data;
                    m_a   <= peak_q;                 // sustain target = peak x sus_lvl
                    m_b   <= sus_lvl;
                    pstep_q <= (incv_q >> porta_sh_q) |
                               {31'd0, (porta_sh_q != 4'd0)};
                    // piano decay rate follows the pitch: bass rings longest.
                    // Registered here so the ST_S2 subtract is fed by a flop
                    // rather than by the 7-bit compare.
                    // +1 shift on every range (was 15/14/13).  Measured on a
                    // held MIDI 60, the old rate was -16.6 dB at 0.5 s and
                    // -37.7 dB at 1 s, and the note was DIGITALLY SILENT at
                    // 1.27 s - so it read as "quiet" even though its peak is
                    // actually 4 % above the sawtooth recipe's.  A real piano
                    // C4 is still going seconds later.  +1 gives -8.3 dB at
                    // 0.5 s and -18 dB at 1 s (see _piano_tone.py); +2 would
                    // be more realistic still (-9 dB at 1 s) but holds voices
                    // twice as long, and running out of voices is exactly the
                    // "tail cut off" complaint - so +1 is the safe side.
                    // Raise these three numbers by one more for a longer ring.
                    pdec_sh_q <= (note_q < 7'd48) ? 5'd16 :
                                 (note_q < 7'd72) ? 5'd15 : 5'd14;
                    // glide needs a second ROM read (the source note's
                    // increment); skipped when portamento is off, so the
                    // normal path loses no cycles.
                    st    <= gl_q ? ST_G0 : ST_S2;
                end
                ST_G0: begin
                    lut_addr <= gf_q;
                    st       <= ST_G1;
                end
                ST_G1: begin
                    g_inc_q <= freq_data;
                    st      <= ST_S2;
                end
                ST_S2: begin
                    // sus_q = peak x sus_lvl >> 15 (sus_lvl is 0..32767,
                    // i.e. the fraction times 32768)
                    sus_q <= prod[30:15];
                    // The shared multiplier is IDLE in the ST_S2 -> ST_S3 gap
                    // (the sus product is consumed here, and ST_S3 reloads
                    // it), so the piano can spend that gap on env x pdec for
                    // free - no extra state, no extra cycle. sus_q above
                    // reads prod BEFORE this edge, so reloading the operands
                    // here is race-free.
                    if (piano) begin
                        m_a <= env_q;
                        m_b <= pdec_q;
                    end
                    // ---- portamento / phase accumulate ----
                    // All three writes go through cur_w (the registered
                    // one-hot) and read only ph_raw_q / incv_q / inc_q, so
                    // the data path here is just adders and a comparator.
                    // The accumulator must use ph_raw_q (phase WITHOUT the
                    // vibrato offset): writing ph_q back would make the LFO a
                    // random walk instead of a modulation.
                    for (k4 = 0; k4 < NVOICES; k4 = k4 + 1) begin
                        if (cur_w[k4]) begin
                            if (porta_sh_q == 4'd0) begin
                                // glide disabled: snap to the target note
                                v_inc[k4]   <= inc_q;
                                v_phase[k4] <= ph_raw_q + inc_q;
                            end else if (gl_q) begin
                                // first sample of the glide: start from the
                                // old note
                                v_inc[k4]   <= g_inc_q;
                                v_glide[k4] <= 1'b0;
                                v_phase[k4] <= ph_raw_q + g_inc_q;
                            end else begin
                                // slew v_inc toward the target by
                                // porta_step/sample (constant time/octave)
                                if (incv_q < inc_q)
                                    v_inc[k4] <= ((inc_q - incv_q) <= pstep_q)
                                              ? inc_q : (incv_q + pstep_q);
                                else if (incv_q > inc_q)
                                    v_inc[k4] <= ((incv_q - inc_q) <= pstep_q)
                                              ? inc_q : (incv_q - pstep_q);
                                v_phase[k4] <= ph_raw_q + incv_q;
                            end
                            // note age drives the per-partial amplitude
                            // decay; written from age_q (latched in ST_S0)
                            v_age[k4] <= (age_q == 8'hFF) ? 8'hFF
                                                          : (age_q + 8'd1);
                            // piano string decay: exponential with a -1
                            // floor, so it always reaches exactly 0 instead
                            // of parking at -6 dB (see v_pdec above).  THIS
                            // IS THE ONLY PER-SAMPLE WRITE - without it the
                            // register is reset/Note-on constant and the
                            // synthesiser is entitled to collapse all 22 bits
                            // into one, which silently removes the decay.
                            v_pdec[k4] <= (pdec_raw_q == 22'd0) ? 22'd0
                                        : (pdec_raw_q
                                           - (pdec_raw_q >> pdec_sh_q)
                                           - 22'd1);
                            // piano stretch carry: one count per phase wrap.
                            // Read from the pwrap_q flop (same discipline as
                            // the v_pdec line above) and advanced by the
                            // carry-out of the very add that writes v_phase,
                            // so the two can never drift apart.
                            v_pwrap[k4] <= pwrap_q + {7'd0, ph_ovf};
                        end
                    end
                    p_eff_q      <= ph_q;
                    // first wavetable address: modulator @2f for FM,
                    // fundamental for everything else.
                    // IMPORTANT: must use ph_q (this voice's phase on THIS
                    // cycle) - using p_eff_q here reads the PREVIOUS voice's
                    // phase (nonblocking), turning the output into
                    // sine(vibrato-offset) x envelope ~= DC!
                    wav_addr     <= (timbre == TIM_FM) ? haddr(ph_q, 4'd2)
                                                       : hsel(ph_q, 4'd1, piano, pwrap_q);
                    st <= ST_S3;
                end
                ST_S3: begin
                    // piano: the env x pdec product loaded in the ST_S2 gap
                    // lands here. Latched unconditionally - it is one 16-bit
                    // register and the other timbres simply never read it.
                    env_p <= prod[30:15];
                    case (timbre)
                    TIM_ADD, TIM_ADD8, TIM_PIANO: begin
                        m_a      <= sine_data;             // harmonic 1
                        m_b      <= piano ? pcoef(4'd1) : coef(4'd1);
                        wav_addr <= hsel(p_eff_q, 4'd2, piano, pwrap_q); // fetch h2
                        hi       <= 4'd2;
                        st       <= ST_SA;
                    end
                    TIM_FM: begin
                        m_a <= sine_data;                 // modulator sample
                        m_b <= fm_index;
                        st  <= ST_FM1;
                    end
                    default: begin
                        wave_q <= (timbre == TIM_SINE) ? sine_data  :
                                  (timbre == TIM_SQ)   ? sq_wave    :
                                  (timbre == TIM_SAW)  ? saw_wave   : tri_wave;
                        st <= ST_S11;
                    end
                    endcase
                end

                // ---- additive harmonic loop ----
                // entered with: mult(h1) set up, wav_addr = h2, hi = 2.
                // per visit: accumulate previous product, multiply current
                // harmonic, schedule next fetch. Exit when hi == last+1.
                ST_SA: begin
                    if (hi == 4'd2) harm_acc <= prod >>> 16;
                    else            harm_acc <= harm_acc + (prod >>> 16);
                    if (hi == harm_last + 4'd1) begin
                        st <= ST_S11;
                    end else begin
                        m_a <= sine_data;               // harmonic hi
                        m_b <= hcoef >>> hsh;           // piano: own recipe
                        if (hi < harm_last)
                            wav_addr <= hsel(p_eff_q, hi + 4'd1, piano, pwrap_q);
                        hi <= hi + 4'd1;
                        st  <= ST_SA;
                    end
                end

                // ---- FM carrier compose ----
                ST_FM1: begin
                    // carrier phase = p_eff + modulator x index
                    wav_addr <= (p_eff_q + prod) >> 22;
                    st <= ST_FM2;
                end
                ST_FM2: begin
                    wave_q <= sine_data;                // carrier sample
                    st     <= ST_S11;
                end

                // ---- envelope multiply + accumulate ----
                ST_S11: begin
                    m_a <= is_add ? harm_acc[15:0] : wave_q;
                    // piano plays through env x string-decay, everything
                    // else through the plain ADSR; both then get the
                    // polyphony-dependent gain from vgain8() above
                    m_b <= env_g;
                    st  <= ST_S12;
                end
                ST_S12: begin
                    acc          <= acc + (prod >>> 16);
                    // same cur_w one-hot write as ST_S2 - see the cur_w notes
                    for (k4 = 0; k4 < NVOICES; k4 = k4 + 1) begin
                        if (cur_w[k4]) begin
                            // --------------------------------------------------
                            // 用"快照与实时状态是否一致"判断是否写回 next_state。
                            //
                            // 引擎时分复用：ST_S0 把 v_state[cur] 锁存进 st_q，十余个
                            // 周期后 ST_S12 才写回。在这段窗口内随时可能有事件到达
                            // （起音广播、松手广播），而 st_q 仍是旧值。若直接用过期
                            // 快照算出的 next_state 写回，会覆盖事件刚写入的状态，
                            // 典型表现为：松手落在窗口内时，快照为 DECAY 会把刚写入
                            // 的 ENV_RELEASE 覆盖回去，造成松手后仍然发声；同音连续
                            // 弹奏时，快照仍为 RELEASE 会算出 IDLE，把刚起的新音清除。
                            //
                            // 因此判据取 st_q 与 v_state[k4] 是否一致：
                            //   一致   -> 期间无事件干扰，正常写入 next_state
                            //   不一致 -> 期间有事件，本周期不更新，待 ST_S0 刷新
                            //             快照后自动恢复
                            //
                            // 相位/增量/衰减这些每采样必推进的不受影响，
                            // 只有包络和状态被保护。
                            // --------------------------------------------------
                            if (st_q == v_state[k4]) begin
                                v_env[k4]   <= next_env;
                                v_state[k4] <= next_state;
                                if ((st_q == ENV_RELEASE) &&
                                    (next_state == ENV_IDLE))
                                    v_active[k4] <= 1'b0;
                                if (piano && (pdec_q == 16'd0))
                                    v_active[k4] <= 1'b0;
                            end
                        end
                    end
                    if (cur == NVM1) st <= ST_OUT;
                    else begin cur <= cur + 4'd1; st <= ST_S0; end
                end

                ST_OUT: begin
                    sample_out   <= mix_s;
                    sample_valid <= 1'b1;
                    st           <= ST_IDLE;
                end
                default: st <= ST_IDLE;
            endcase

    // ------------------------------------------------------------------
    // ---- voice allocation / release ----------------------------
    // MUST be evaluated AFTER the case: if a note event lands on
    // the same clock as the voice's ST_S12 slot update
    // (v_state[cur] <= next_state), the event assignment has to
    // win - the later nonblocking write in source order wins, so
    // keeping this block last makes events race-free.
    if (ev_pend) begin
        ev_pend          <= 1'b0;
        v_note[ev_idx]   <= ev_num;
        v_active[ev_idx] <= 1'b1;
        v_state[ev_idx]  <= ENV_ATTACK;
        v_env[ev_idx]    <= 16'd0;
        v_peak[ev_idx]   <= {ev_vel, 7'h00};
        v_phase[ev_idx]  <= 32'd0;
        v_gfrom[ev_idx]  <= ev_from;
        v_glide[ev_idx]  <= 1'b1;
        v_age[ev_idx]    <= 8'd0;
        v_pdec[ev_idx]   <= PDEC_INIT;
        // phase restarts at 0 here, so the stretch carry must too - a stale
        // count would offset every partial by up to 1023 table steps.
        v_pwrap[ev_idx]  <= 8'd0;
    end
    if (note_on) begin
        ev_pend <= 1'b1;
        // retrig（同音还按着）> 空闲槽 > 抢 release 尾音槽 > 抢 rr_ptr
        ev_idx  <= retrig_ok ? retrig_idx :
                   alloc_ok  ? alloc_idx  :
                   rel_ok    ? rel_idx    : rr_ptr;
        ev_num  <= note_on_num;
        ev_vel  <= note_on_vel;
        ev_from <= prev_note;
        if (!retrig_ok) rr_ptr <= rr_ptr + 5'd1;
    end
    if (note_on) prev_note <= note_on_num;
    // note_off: broadcast. Every voice holding that note goes to release, so
    // a stolen voice can never be left droning.
    if (rel_all) begin
        for (k1 = 0; k1 < NVOICES; k1 = k1 + 1)
            if (v_active[k1]) v_state[k1] <= ENV_RELEASE;
    end else if (note_off) begin
        for (k1 = 0; k1 < NVOICES; k1 = k1 + 1)
            if (v_active[k1] && (v_note[k1] == note_off_num) &&
                (v_state[k1] != ENV_RELEASE))
                v_state[k1] <= ENV_RELEASE;
    end
        end
    end

endmodule
`default_nettype wire
