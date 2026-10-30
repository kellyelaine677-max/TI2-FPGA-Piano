`timescale 1ns/1ps
// ============================================================================
// song_player.v - offline LED song guide (competition "拓展 4")
//
// Steps through a melody (built-in Twinkle / 小星星, or a downloaded score)
// and emits the usual note_on / note_off event pulses, so the melody is
// actually synthesised by the engine (never a recording). At the same time it
// publishes
//   * cur_slot : one-hot chromatic slot of the sounding note -> WS2812
//                guidance strip lights that slot, i.e. "play this key next"
//   * progress : index of the current note -> shown on the LCD
//
// The note_off of the old note and the note_on of the new one are deliberately
// issued in TWO SEPARATE clock cycles: the engine processes note_on before
// note_off inside one cycle, so pulsing both at once with the same note number
// would retrigger and immediately release the voice.
//
// 引导模式（guide_only）采用事件驱动推进，不依赖拍计时：
//   灯带点亮当前待弹的键 -> 用户弹奏 -> 与谱面一致才推进到下一个音。
// 引导模式与自动播放各自独立推进，引导模式下拍计时器不参与推进，
// 因此灯带不会自行按拍前进，弹奏者可按自己的节奏进行。
//
// 时值单位是**半拍**（BEAT = 0.175 s @ 40.5 MHz）：
//   1 = 八分音符, 2 = 四分音符, 4 = 二分音符, 8 = 全音符（最长 7.5 拍）
// 晴天/退后/花海全是八分音符密集的流行歌，按整拍弹节奏完全是错的 ——
// 这是"下载的谱子弹出来不对"的主因，所以粒度从整拍改到半拍。
//
// 音级 7 = **休止符**：不出声、LED 灭，但时间轴照常推进。
// ============================================================================
`default_nettype none

module song_player (
    input  wire        clk,
    input  wire        rstn,
    input  wire        enable,          // 1 = play, 0 = stop and reset to top
    input  wire        guide_only,      // 1 = 只推进 LED 指引，不自动发声
    // 引导模式的推进来源：用户在琴上真的按了一个键。
    // key_step = 1 个时钟的脉冲，key_semi = 那个键的半音槽 0..11。
    // 只有 key_semi 等于当前该弹的音，指针才前进 —— 弹错了灯就停在那儿等你。
    input  wire        key_step,
    input  wire [3:0]  key_semi,
    input  wire [6:0]  note_base,       // transposition base (octave follows)
    output reg         note_on,
    output reg         note_off,
    output reg  [6:0]  note_num,
    output reg  [11:0] cur_slot,        // one-hot 0..11 for the LED strip
    output reg  [7:0]  progress,        // current note index
    // 拓展 4: 在线曲谱下载. Bytes arrive from the Bluetooth SysEx decoder:
    //   sc_start -> sc_data is the number of notes that follow
    //   sc_byte  -> sc_data is one note, {3-bit degree, 4-bit units}
    //   sc_end   -> commit
    // Until the first download completes the built-in melody plays unchanged,
    // so a board that never receives a score behaves exactly as it did.
    input  wire       sc_start,
    input  wire       sc_byte,
    input  wire       sc_end,
    input  wire [6:0] sc_data
);

    localparam [7:0] N_NOTES = 8'd42;
    // 0.175 s per half beat (八分音符) @ 40.5 MHz system clock
    localparam [24:0] BEAT = 25'd7_087_500;

    // scale degrees: 0=C 1=D 2=E 3=F 4=G 5=A 6=B (7 = rest, only in RAM)
    function [2:0] song_deg(input [7:0] i);
        case (i)
            8'd0, 8'd1: song_deg = 3'd0;   // C C
            8'd2, 8'd3: song_deg = 3'd4;   // G G
            8'd4, 8'd5: song_deg = 3'd5;   // A A
            8'd6:       song_deg = 3'd4;   // G-
            8'd7, 8'd8: song_deg = 3'd3;   // F F
            8'd9, 8'd10: song_deg = 3'd2;  // E E
            8'd11, 8'd12: song_deg = 3'd1; // D D
            8'd13:      song_deg = 3'd0;   // C-
            8'd14, 8'd15: song_deg = 3'd4; // G G
            8'd16, 8'd17: song_deg = 3'd3; // F F
            8'd18, 8'd19: song_deg = 3'd2; // E E
            8'd20:      song_deg = 3'd1;   // D-
            8'd21, 8'd22: song_deg = 3'd4; // G G
            8'd23, 8'd24: song_deg = 3'd3; // F F
            8'd25, 8'd26: song_deg = 3'd2; // E E
            8'd27:      song_deg = 3'd1;   // D-
            8'd28, 8'd29: song_deg = 3'd0; // C C
            8'd30, 8'd31: song_deg = 3'd4; // G G
            8'd32, 8'd33: song_deg = 3'd5; // A A
            8'd34:      song_deg = 3'd4;   // G-
            8'd35, 8'd36: song_deg = 3'd3; // F F
            8'd37, 8'd38: song_deg = 3'd2; // E E
            8'd39, 8'd40: song_deg = 3'd1; // D D
            default:     song_deg = 3'd0;  // C-
        endcase
    endfunction

    // scale degree -> chromatic semitone. note_num is fed to rom_freq which
    // expects a real MIDI number, so a diatonic degree MUST be expanded here.
    // The previous code added the degree itself (0..6), which played the guide
    // melody as C C# D D# E F F# - wrong intervals, and it also lit the wrong
    // LED slot (G = key 7, not key 4). Downloaded scores use the same 3-bit
    // degree encoding, so this had to be fixed before 在线曲谱下载 could be
    // demonstrated correctly.
    function [3:0] deg2semi(input [2:0] d);
        case (d)
            3'd0: deg2semi = 4'd0;     // C
            3'd1: deg2semi = 4'd2;     // D
            3'd2: deg2semi = 4'd4;     // E
            3'd3: deg2semi = 4'd5;     // F
            3'd4: deg2semi = 4'd7;     // G
            3'd5: deg2semi = 4'd9;     // A
            default: deg2semi = 4'd11; // B (3'd7 = rest, gated upstream)
        endcase
    endfunction

    // built-in Twinkle: last note of each 7-note phrase is held for 1 beat
    // (= 2 half-beat units now that the unit is a half beat)
    function [3:0] song_beats(input [7:0] i);
        case (i)
            8'd6, 8'd13, 8'd20, 8'd27, 8'd34: song_beats = 4'd4;
            default: song_beats = 4'd2;
        endcase
    endfunction

    reg [7:0]  idx;
    reg [3:0]  dur_left;
    reg [24:0] beat_cnt;
    reg        sounding;
    reg        pend;
    reg [6:0]  cur_note;
    reg [2:0]  deg_q;
    reg        guide_q;    // guide_only 打一拍，用来抓"切进引导"的边沿
    reg        en_q;       // enable 打一拍，用来抓"重新开始"的边沿
    //
    //  需要同时检测 guide 与 enable 两个边沿，仅检测 guide_q 不够：
    // 在引导模式下重复下载曲谱时（sc_start 将 enable 拉低、sc_end 再拉高），
    // guide_only 与 guide_q 持续为 1，若只依赖 guide_q 边沿，则"指回第一个音"
    // 的逻辑不会执行，cur_slot 保持在 enable=0 时清零的状态，灯带整条熄灭。
    // 所以 enable 的上升沿也要重新指向第一个音。
    // 连续休止符的自动跳读额度。休止符不出声也不亮灯，不跳过去的话用户会
    // 停在一片灭掉的灯带前不知道该干嘛。给个上限防止谱子异常时绕圈。
    reg [2:0]  rest_skip;

    // downloaded score: 64 slots of {degree[2:0], units[3:0]}
    reg [6:0]  score [0:63];
    reg [6:0]  sc_len;
    reg [6:0]  sc_wptr;
    reg        sc_loaded;

    // one slot is read per note change, so a plain register array is fine -
    // at 0.175 s per unit there is no need for a synchronous read.
    wire [7:0] n_notes = sc_loaded ? {1'b0, sc_len} : N_NOTES;
    wire       use_ram  = sc_loaded;
    wire [2:0] deg_now  = use_ram ? score[idx[5:0]][6:4] : song_deg(idx);
    wire [3:0] bts_now  = use_ram ? score[idx[5:0]][3:0] : song_beats(idx);
    wire [3:0] semi_now = deg2semi(deg_now);   // chromatic key slot 0..11

    // ---- 下一个音（引导模式推进时用）----
    // 推进那一拍就要把 cur_slot 指到"下一个音"，所以 idx+1 的组合值要现算。
    wire [7:0] idx_next = (idx == n_notes - 8'd1) ? 8'd0 : (idx + 8'd1);
    wire [2:0] deg_next = use_ram ? score[idx_next[5:0]][6:4] : song_deg(idx_next);
    wire [3:0] semi_next = deg2semi(deg_next);
    // 复位/刚进引导时指向第 0 个音
    wire [2:0] deg_zero = use_ram ? score[6'd0][6:4] : song_deg(8'd0);
    wire [3:0] semi_zero = deg2semi(deg_zero);
    wire [11:0] slot_of_zero = (deg_zero == 3'd7) ? 12'd0 : (12'd1 << semi_zero);
    wire [11:0] slot_of_next = (deg_next == 3'd7) ? 12'd0 : (12'd1 << semi_next);

    // ---- 下一个音的槽位信息 ----
    //
    //  slot_of_next / cur_slot 用于引导模式下点亮下一个待弹的键。
    //
    // 曲谱中同音连续出现时（如《小星星》的 do do / sol sol），每个音仍独立
    // 发送 note_on / note_off，不合并时值：这类曲目的成对同音本应为两个独立
    // 短音，合并后节奏会被改变。换音时亚毫秒级的释放与起振重叠属于正常乐音，
    // 无需消除。

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            note_on <= 1'b0; note_off <= 1'b0; note_num <= 7'd60;
            cur_slot <= 12'd0; progress <= 8'd0;
            idx <= 8'd0; dur_left <= 4'd0; beat_cnt <= 25'd0;
            sounding <= 1'b0; pend <= 1'b0; cur_note <= 7'd60; deg_q <= 3'd0;
            sc_len <= 7'd0; sc_wptr <= 7'd0; sc_loaded <= 1'b0;
            guide_q <= 1'b0; en_q <= 1'b0;
        end else begin
            note_on  <= 1'b0;
            note_off <= 1'b0;

            // score download, independent of whether the guide is playing
            if (sc_start) begin
                sc_len    <= sc_data;
                sc_wptr   <= 7'd0;
                sc_loaded <= 1'b0;        // do not play a half-written score
            end else if (sc_byte) begin
                score[sc_wptr[5:0]] <= sc_data;
                if (sc_wptr != 7'd63) sc_wptr <= sc_wptr + 7'd1;
            end else if (sc_end) begin
                sc_loaded <= 1'b1;
            end

            if (!enable) begin
                //  停止时必须发出 note_off 以关闭正在发声的音：引擎的释放
                // 流程由 note_off 启动，若仅拉低 sounding 而不发送 note_off，
                // 该音会一直持续。钢琴音色衰减快不易察觉，方波/正弦的释放时间
                // 超过 100 ms，会表现为停止播放后仍有长音残留。
                if (sounding) begin
                    note_off <= 1'b1;
                    note_num <= cur_note;
                end
            idx <= 8'd0; dur_left <= 4'd0; beat_cnt <= 25'd0;
            sounding <= 1'b0; progress <= 8'd0;
            cur_slot <= 12'd0;
            rest_skip <= 3'd0;
                // enable 拉高后要靠 pend 打出第一个音，而 pend 只在这里和
                // "上一个音时值用完" 那一条路径上被置 1。写成 0 的话
                // sounding=0 / pend=0 / dur_left=0 三者互锁：beat 到点时
                // 两个分支都不成立，pend 再也回不来，曲子**一个音都不会响**
                // （LED 灯带也全灭）—— 表现就是"下载完曲谱自动播放没反应"。
                pend <= 1'b1;
            end else if (guide_only) begin
                // ==========================================================
                // 引导模式：事件驱动，等你弹对了才走（不跑计时器）
                // ==========================================================
                if (!guide_q || (enable && !en_q)) begin
                    // 刚切进引导 / 重新开始：把自动播放的残音收掉，
                    // 指针回到第一个音。
                    // 用户的按键由触摸/键盘路径自己发声，本模块在引导模式下
                    // 一个音都不发。
                    if (sounding) begin
                        note_off <= 1'b1;
                        note_num <= cur_note;
                    end
                    sounding  <= 1'b0;
                    pend      <= 1'b0;
                    idx       <= 8'd0;
                    dur_left  <= 4'd0;
                    beat_cnt  <= 25'd0;
                    progress  <= 8'd0;
                    cur_slot  <= slot_of_zero;
                    rest_skip <= 3'd4;      // 开头的休止符自动跳过去
                end else if (key_step && (key_semi == semi_now)) begin
                    // 弹对了 → 灯带指向下一个音
                    idx       <= idx_next;
                    progress  <= idx_next;
                    cur_slot  <= slot_of_next;
                    rest_skip <= 3'd4;
                end else if (rest_skip != 3'd0 && deg_now == 3'd7) begin
                    // 停在休止符上：自动跳读
                    idx       <= idx_next;
                    progress  <= idx_next;
                    cur_slot  <= slot_of_next;
                    rest_skip <= rest_skip - 3'd1;
                end else begin
                    rest_skip <= 3'd0;
                end
            end else begin
                if (beat_cnt != BEAT) beat_cnt <= beat_cnt + 25'd1;
                else begin
                    beat_cnt <= 25'd0;
                    if (dur_left != 4'd0) dur_left <= dur_left - 4'd1;
                    else begin
                        // sounding=1 才说明引擎里真的有本模块的音在响，
                        // 这里的 note_off **不需要**再按 guide_only gate：
                        // 引导模式下 sounding 恒为 0（见 pend 分支），
                        // 所以永远不会误关用户正在弹的音。
                        if (sounding) begin
                            note_off <= 1'b1;
                            note_num <= cur_note;
                        end
                        sounding <= 1'b0;
                        pend     <= 1'b1;
                    end
                end
                if (pend) begin
                    deg_q <= deg_now;
                    // 休止符：LED 灭、不出声，但时间轴照常推进
                    cur_slot <= (deg_now == 3'd7) ? 12'd0 : (12'd1 << semi_now);
                    if (!guide_only && deg_now != 3'd7) begin
                        note_num <= note_base + {3'b000, semi_now};
                        cur_note <= note_base + {3'b000, semi_now};
                        note_on  <= 1'b1;
                    end
                    // sounding 只在"真的往引擎塞了一个音"时才置 1。
                    // 引导模式下也置 1 的话它就没有对应的 note_on，
                    // 拍点到了还会去发一个没人听过的 note_off。
                    sounding <= !guide_only && (deg_now != 3'd7);
                    pend     <= 1'b0;
                    progress <= idx;
                    // 时值：dur_left 是"还要再等几拍才松手"。
                    // 拍点到来时先减 dur_left，减到 0 后的**下一个**拍点才
                    // note_off，所以这里必须预减 1，否则 1 个单位的音会被
                    // 拖成 2 个单位、整首曲子慢一倍。
                    dur_left <= (bts_now == 4'd0) ? 4'd0 : (bts_now - 4'd1);
                    idx      <= (idx == n_notes - 8'd1) ? 8'd0 : (idx + 8'd1);
                end
            end
            guide_q <= guide_only;
            en_q    <= enable;
        end
    end

endmodule
`default_nettype wire
