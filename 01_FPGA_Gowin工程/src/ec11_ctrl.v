`timescale 1ns/1ps
// ============================================================================
// ec11_ctrl.v - EC11 rotary encoder driver (quadrature + push button)
//
// ONE mechanical click must move the selection by exactly ONE step.
//
// The old version emitted a pulse on every valid Gray transition. That is
// only right for a 20-pulse / 20-detent EC11, where one click is a full
// four-step cycle. This board's knob is the equally common 15-pulse /
// 30-detent variant - one click is HALF a cycle, i.e. two transitions - so
// one click moved the selection two steps. With eight timbres the selection
// then only ever landed on 0 / 2 / 4 / 6 and every odd entry (7 = piano)
// was unreachable, which is exactly the "I can never turn it to piano"
// report.
//
// STEPS_PER_DETENT below is the knob for that. If a knob ever needs two
// clicks to move one step, it is the 4-transition type - change it to 4.
//
// The old version also had no synchroniser and no debounce. A and B are
// asynchronous inputs: contact bounce that fell back into the previous state
// and returned looked like another perfectly legal Gray transition, so the
// step count varied from click to click ("often two, sometimes not").
// ============================================================================
`default_nettype none

module ec11_ctrl (
    input  wire clk,
    input  wire rstn,
    input  wire a,
    input  wire b,
    input  wire btn,
    output reg  inc,
    output reg  dec,
    output reg  btn_press
);

    // ---- how many valid Gray transitions make one click -----------------
    //   2 : 15 pulses / 30 detents  (this board)
    //   4 : 20 pulses / 20 detents
    localparam [1:0] STEPS_PER_DETENT = 2'd2;

    // 8 kHz sample tick @ 40.5 MHz (40500000 / 8000 - 1 = 5061).
    // The old 1 kHz tick was slower than the knob: a flicked EC11 makes
    // several hundred transitions per second and the sampler walked straight
    // past them, losing clicks.
    localparam [12:0] SAM_END = 13'd5061;
    reg [12:0] sam_cnt;
    wire tick = (sam_cnt == SAM_END);
    always @(posedge clk or negedge rstn) begin
        if (!rstn) sam_cnt <= 13'd0;
        else sam_cnt <= tick ? 13'd0 : sam_cnt + 13'd1;
    end

    // ---- synchronisers (a / b / btn are asynchronous to clk) ------------
    reg [1:0] a_s, b_s, btn_s;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            a_s <= 2'b11; b_s <= 2'b11; btn_s <= 2'b11;
        end else begin
            a_s   <= {a_s[0],   a};
            b_s   <= {b_s[0],   b};
            btn_s <= {btn_s[0], btn};
        end
    end

    // ---- debounce: 4 stable samples at 8 kHz = 0.5 ms -------------------
    reg [3:0] a_db, b_db;
    reg       a_lvl, b_lvl;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            a_db <= 4'hF; b_db <= 4'hF;
            a_lvl <= 1'b1; b_lvl <= 1'b1;
        end else if (tick) begin
            a_db <= {a_db[2:0], a_s[1]};
            b_db <= {b_db[2:0], b_s[1]};
            // only accept a level that has held for all four samples
            if      (a_db == 4'b1111) a_lvl <= 1'b1;
            else if (a_db == 4'b0000) a_lvl <= 1'b0;
            if      (b_db == 4'b1111) b_lvl <= 1'b1;
            else if (b_db == 4'b0000) b_lvl <= 1'b0;
        end
    end

    // ---- quadrature decode, one pulse per click -------------------------
    reg [1:0] ab_q;
    reg [1:0] steps;    // transitions counted inside the current click
    reg       dir_q;    // 1 = CW
    wire [1:0] ab = {a_lvl, b_lvl};

    reg cw, ccw;
    always @* begin
        cw  = 1'b0;
        ccw = 1'b0;
        case ({ab_q, ab})
            // CW: 00->01->11->10->00
            4'b0001, 4'b0111, 4'b1110, 4'b1000: cw  = 1'b1;
            // CCW: 00->10->11->01->00
            4'b0010, 4'b1011, 4'b1101, 4'b0100: ccw = 1'b1;
            default: ;     // no move, or both bits moved (missed a transition)
        endcase
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            ab_q <= 2'b11;
            steps <= 2'd0;
            dir_q <= 1'b0;
            inc <= 1'b0; dec <= 1'b0;
        end else begin
            inc <= 1'b0;
            dec <= 1'b0;
            if (tick) begin
                ab_q <= ab;
                if (cw || ccw) begin
                    if ((cw && dir_q) || (ccw && !dir_q)) begin
                        // same direction as the click in progress
                        if (steps == STEPS_PER_DETENT - 2'd1) begin
                            steps <= 2'd0;
                            if (cw) inc <= 1'b1; else dec <= 1'b1;
                        end else steps <= steps + 2'd1;
                    end else begin
                        // direction reversed: restart the click
                        dir_q <= cw;
                        steps <= 2'd1;
                    end
                end
            end
        end
    end

    // ---- button: 20 ms debounce, pulse on the falling edge --------------
    // 160 samples at 8 kHz = 20 ms. Input idles high (pull-up).
    reg [7:0] btn_cnt;
    reg       btn_lvl, btn_prev;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            btn_cnt <= 8'd0; btn_lvl <= 1'b1; btn_prev <= 1'b1;
            btn_press <= 1'b0;
        end else begin
            btn_press <= 1'b0;
            if (tick) begin
                if (btn_s[1] != btn_lvl) begin
                    if (btn_cnt >= 8'd160) begin
                        btn_lvl  <= btn_s[1];
                        btn_cnt  <= 8'd0;
                    end else btn_cnt <= btn_cnt + 8'd1;
                end else btn_cnt <= 8'd0;

                btn_prev <= btn_lvl;
                if (btn_prev && !btn_lvl) btn_press <= 1'b1;
            end
        end
    end

endmodule
`default_nettype wire
