`timescale 1ns/1ps
// ============================================================================
// midi_rx.v - note on / note off out of a MIDI byte stream
//
// Standard MIDI over the wire: 0x9n = note on, 0x8n = note off, running
// status allowed. A 0x9n with velocity 0 IS a note off - the spec says so
// and several apps never send 0x8n at all, so missing that leaves notes
// stuck on forever.
//
// Everything else (CC, pitch bend, aftertouch, sysex, clock) is consumed and
// dropped. That is why the data-byte count per status matters: 0xCn and 0xDn
// carry ONE data byte, everything else two. Counting wrong desynchronises the
// stream for good, and the result looks exactly like a dead link.
// ============================================================================
`default_nettype none

module midi_rx (
    input  wire       clk,
    input  wire       rstn,
    input  wire [7:0] din,
    input  wire       din_valid,
    output reg        note_on,     // 1-clk pulse, note_num valid
    output reg        note_off,    // 1-clk pulse, note_num valid
    output reg  [6:0] note_num,
    // CC 20..31 remap the 12 slots (拓展 4: 演奏键位自定义). CC is used
    // instead of SysEx because this already has a two-byte decoder - SysEx
    // would need a variable-length buffer, and there is no BSRAM left for one.
    output reg        cc_wr,       // 1-clk pulse
    output reg  [3:0] cc_idx,      // 0..11 = slot
    output reg  [3:0] cc_val,      // 0..11 = note offset
    // Score download (拓展 4: 在线曲谱下载), carried in SysEx so it cannot
    // collide with note traffic:
    //   F0 7D 01 <count> <b0> <b1> ... <bN-1> F7
    // Each byte is {3-bit scale degree, 4-bit beats}. SysEx is used because
    // it is the standard escape hatch; the alternative, more CC numbers,
    // would take one message per byte and a second decoder.
    output reg        sc_start,    // sc_data = number of notes following
    output reg        sc_byte,     // sc_data = one note byte
    output reg        sc_end,      // commit what was received
    output reg  [6:0] sc_data
);

    reg [7:0] status;       // running status; 0 = none yet
    reg [6:0] d0;
    reg       have_d0;
    reg       sysex;        // inside an F0 ... F7 block
    reg [1:0] sc_st;        // 0 vendor, 1 command, 2 count, 3 payload

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            status <= 8'd0; d0 <= 7'd0; have_d0 <= 1'b0;
            note_on <= 1'b0; note_off <= 1'b0; note_num <= 7'd0;
            cc_wr <= 1'b0; cc_idx <= 4'd0; cc_val <= 4'd0;
            sysex <= 1'b0; sc_st <= 2'd0;
            sc_start <= 1'b0; sc_byte <= 1'b0; sc_end <= 1'b0;
            sc_data <= 7'd0;
        end else begin
            note_on  <= 1'b0;
            note_off <= 1'b0;
            cc_wr    <= 1'b0;
            sc_start <= 1'b0;
            sc_byte  <= 1'b0;
            sc_end   <= 1'b0;
            if (din_valid) begin
                if (din[7]) begin                     // status byte
                    if (din == 8'hF0) begin
                        sysex   <= 1'b1;
                        sc_st   <= 2'd0;
                        status  <= 8'd0;
                        have_d0 <= 1'b0;
                    end else begin
                        if (din == 8'hF7 && sysex) sc_end <= 1'b1;
                        sysex   <= 1'b0;                 // F7 or any system
                        status  <= (din >= 8'hF0) ? 8'd0 : din;
                        have_d0 <= 1'b0;                 // clears running status
                    end
                end else if (sysex) begin
                    // payload of our own SysEx only; anything else is skipped
                    case (sc_st)
                    2'd0: if (din == 8'h7D) sc_st <= 2'd1;   // vendor id
                    2'd1: if (din == 8'h01) sc_st <= 2'd2;   // cmd: load score
                    2'd2: begin
                        sc_data  <= din[6:0];                // note count
                        sc_start <= 1'b1;
                        sc_st    <= 2'd3;
                    end
                    default: begin
                        sc_data <= din[6:0];                 // one note byte
                        sc_byte <= 1'b1;
                    end
                    endcase
                end else if (status != 8'd0) begin
                    if (!have_d0) begin
                        d0 <= din[6:0];
                        // one-byte messages: keep waiting for the next status
                        have_d0 <= !(status[7:4] == 4'hC || status[7:4] == 4'hD);
                    end else begin
                        have_d0 <= 1'b0;
                        case (status[7:4])
                        4'h9: begin                   // note on
                            note_num <= d0;
                            if (din[6:0] == 7'd0) note_off <= 1'b1;   // vel 0
                            else                 note_on  <= 1'b1;
                        end
                        4'h8: begin                   // note off
                            note_num <= d0;
                            note_off <= 1'b1;
                        end
                        4'hB: begin                   // control change
                            // only 20..31 mean anything to us; the rest
                            // (mod wheel, expression, ...) are dropped
                            if (d0 >= 7'd20 && d0 <= 7'd31) begin
                                // CC 20..31 -> slot 0..11. Written as 4-bit
                                // arithmetic on purpose: d0 - 7'd20 is a
                                // 7-bit expression and the tool warned about
                                // truncating it into the 4-bit cc_idx
                                // (EX3791). The enclosing if already proves
                                // d0 is 20..31, so d0[3:0] is 4..15 and this
                                // gives 0..11 with nothing thrown away.
                                cc_idx <= d0[3:0] - 4'd4;
                                cc_val <= (din[6:0] > 7'd11) ? 4'd11
                                                             : din[3:0];
                                cc_wr  <= 1'b1;
                            end
                            // CC 32 -> slot 12: 曲谱播放方式
                            //   0 = 引导（只点亮 LED 等你弹）
                            //   1 = 直接播放（板子自己把曲子弹出来）
                            else if (d0 == 7'd32) begin
                                cc_idx <= 4'd12;
                                cc_val <= (din[6:0] == 7'd0) ? 4'd0 : 4'd1;
                                cc_wr  <= 1'b1;
                            end
                            // CC 33 -> slot 13: 音色选择（手机上自由切换）
                            //   0 正弦 1 方波 2 锯齿 3 三角 4 FM 5 加法
                            //   6 八谐波加法 7 钢琴
                            // 值只有 0..7，取低 3 位即可，cc_val[3] 恒 0。
                            else if (d0 == 7'd33) begin
                                cc_idx <= 4'd13;
                                cc_val <= {1'b0, din[2:0]};
                                cc_wr  <= 1'b1;
                            end
                            // CC 34 -> slot 14: 播放开关
                            //   0 = 停止（并从头复位）  1 = 播放
                            // 手机上的"停止播放"按钮用，否则下载完的曲子
                            // 会一直循环停不下来。
                            else if (d0 == 7'd34) begin
                                cc_idx <= 4'd14;
                                cc_val <= (din[6:0] == 7'd0) ? 4'd0 : 4'd1;
                                cc_wr  <= 1'b1;
                            end
                        end
                        default: ;                    // bend / ... ignore
                        endcase
                    end
                end
            end
        end
    end

endmodule
`default_nettype wire
