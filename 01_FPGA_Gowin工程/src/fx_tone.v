`timescale 1ns/1ps
// ============================================================================
// fx_tone.v - one-pole IIR low-pass "tone" filter on the mix bus
//
//   y[n] = y[n-1] + a * (x[n] - y[n-1]),   a = (cutoff + 1) / 256
//
// cutoff = 255 -> a = 1    -> y = x (fully open, no filtering)
// cutoff = 0   -> a = 1/256 -> very dark (fc ~ 44 Hz at 70.3 kHz)
//
// One 17x9 multiply plus one add per sample, one clock of latency.
// This is the "滤波" playing instruction from the competition brief and adds
// another continuous expression dimension on top of velocity / vibrato / FX.
// ============================================================================
`default_nettype none

module fx_tone (
    input  wire               clk,
    input  wire               rstn,
    input  wire               in_valid,
    input  wire signed [15:0] in_data,
    input  wire        [7:0]  cutoff,     // 0..255, 255 = bypass
    output reg                out_valid,
    output reg  signed [15:0] out_data
);

    reg signed [15:0] y;

    wire signed [16:0] diff = in_data - y;
    wire        [8:0]  a    = {1'b0, cutoff} + 9'd1;      // 1..256
    wire signed [24:0] mul  = diff * $signed({7'b0000000, a});
    wire signed [15:0] ny   = y + (mul >>> 8);

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            y         <= 16'sd0;
            out_valid <= 1'b0;
            out_data  <= 16'sd0;
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                y        <= ny;
                out_data <= ny;
            end
        end
    end

endmodule
`default_nettype wire
