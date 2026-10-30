`timescale 1ns/1ps
// ============================================================================
// i2s_tx.v - PT8211 DAC driver (LSBJ / "Japanese" format)
//
// PT8211 is NOT standard I2S! Per the PT8211 datasheet (and matching the
// Sipeed official demo): DIN is shifted into the input register on the
// RISING edge of BCK; the 16-bit word is MSB first and its LSB must be the
// last bit before the slot ends; WS low = right slot, WS high = left slot.
// The DAC latches both channel registers at each WS edge.
//
//   (The old version sent standard-I2S: data delayed one BCK after the WS
//   edge plus 16 zero-padding bits per slot - the padding pushed the data
//   out of the DAC's latch window -> garbage audio.)
//
// Clocking (core clk = 40.5 MHz, from the 27 MHz oscillator through the rPLL
// in clk_gen.v):
//   BCK   = clk / 18 = 2.25 MHz  (datasheet limit 18.4 MHz)
//   frame = 32 BCK = 576 clk ->  fs = 70.3125 kHz (WS limit 384 kHz OK)
//
// The frame length is deliberately 576 core clocks, not 768: the synthesis
// engine emits one sample every 576 clocks (top_synth's tick_cnt == 575), and
// the I2S frame MUST match it exactly or the DAC plays a pitched-up/down
// version of the stream. 40.5 MHz / 576 = 70.3125 kHz, the same audio rate as
// the earlier 27 MHz / 384 build.
//
// Timing margins: WS/DIN change on BCK falling edges, DAC samples on
// rising edges -> 9 clk (222 ns) setup/hold, spec requires 25 ns.
// ============================================================================
`default_nettype none

module i2s_tx (
    input  wire               clk,
    input  wire               rstn,
    input  wire               sample_valid,
    input  wire signed [15:0] sample_data,
    output reg                bck,
    output reg                lrck,     // WS: 1 = left slot, 0 = right slot
    output reg                sdout
);

    // ---- BCK = clk/18, 9 clk high / 9 clk low -------------------------
    // div is 5 bits because it must reach 17: an 18-cycle period is what makes
    // one frame (32 BCK) exactly 576 core clocks, i.e. one engine sample.
    // (A 4-bit div caps at 15 -> the period silently collapses to 9 clk, the
    // frame to 288 clk and the DAC runs at 140.625 kHz, double the engine.)
    reg [4:0] div;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) div <= 5'd0;
        else div <= (div == 5'd17) ? 5'd0 : div + 5'd1;
    end
    wire bck_rise = (div == 5'd0);
    wire bck_fall = (div == 5'd9);

    // BCK toggle (NOTE: without this block BCK is stuck at 0 and the DAC
    // never clocks in a single bit - total silence!)
    always @(posedge clk or negedge rstn) begin
        if (!rstn)     bck <= 1'b0;
        else if (bck_rise) bck <= 1'b1;
        else if (bck_fall) bck <= 1'b0;
    end

    // ---- bit index within the frame: 0..31 (16 per slot) ---------------
    reg [4:0] bidx;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) bidx <= 5'd0;
        else if (bck_fall) bidx <= (bidx == 5'd31) ? 5'd0 : bidx + 5'd1;
    end

    // ---- sample capture (engine ticks once per frame) ------------------
    reg [15:0] cur;    // sample driving the current frame (both slots, mono)
    reg [15:0] nxt;    // newest captured sample
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            cur <= 16'd0; nxt <= 16'd0;
        end else begin
            if (sample_valid) nxt <= sample_data;
            if (bck_fall && bidx == 5'd31) cur <= nxt;   // frame boundary
        end
    end

    // ---- WS: slot 0..15 = left (1), 16..31 = right (0) -----------------
    always @(posedge clk or negedge rstn) begin
        if (!rstn) lrck <= 1'b1;
        else if (bck_fall) begin
            if (bidx == 5'd15)     lrck <= 1'b0;
            else if (bidx == 5'd31) lrck <= 1'b1;
        end
    end

    // ---- DIN: bit for BCK cycle n is cur[15 - n[3:0]] ------------------
    // DIN changes on BCK falling edges (stable at the DAC's rising-edge
    // sampling point). MSB sits on the first BCK of each slot, LSB on the
    // last one - true LSBJ. At a slot boundary the index wraps 15 -> 0,
    // which selects cur[15]: the MSB of the next slot.
    always @(posedge clk or negedge rstn) begin
        if (!rstn) sdout <= 1'b0;
        else if (bck_fall) begin
            if (bidx == 5'd31) sdout <= nxt[15];        // new frame MSB
            else               sdout <= cur[4'd14 - bidx[3:0]];
        end
    end

endmodule
`default_nettype wire
