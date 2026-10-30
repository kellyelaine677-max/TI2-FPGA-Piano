`timescale 1ns/1ps
// ============================================================================
// fx_echo.v - Multi-tap feedback delay (echo / simple reverb)
//
// Circular buffer of DEPTH samples (4096 @ 70.3 kHz = 58 ms) with four
// unequally spaced taps (Schroeder-style comb set) read each sample:
//
//     avg  = (buf[w-T1] + buf[w-T2] + buf[w-T3] + buf[w-T4]) / 4
//     write = dry + avg * feedback        (loop gain < 1 -> stable)
//     out   = dry + avg * wet
//
// Taps T1..T4 are near-coprime so their repeats interleave -> dense tail.
// Feedback 0.4, wet 0.3 defaults give a warm slap-back; cranking feedback
// approaches a reverb-like bloom.
//
// Runs in ~9 clk per audio sample, BRAM single port (4 reads + 1 write).
// Latency: one sample period (out_valid follows in_valid).
// ============================================================================
`default_nettype none

module fx_echo #(
    // 4096 x 16 = 4 BSRAM blocks. (8192-deep needed 8 blocks and pushed the
    // total past the GW2A-18's 46 - the overflow cascaded into DFFs.)
    parameter AW     = 12,               // address bits, DEPTH = 2^AW
    parameter [11:0] T1 = 12'd4095,      // tap delays (samples)
    parameter [11:0] T2 = 12'd3071,
    parameter [11:0] T3 = 12'd2296,
    parameter [11:0] T4 = 12'd1724
)(
    input  wire               clk,
    input  wire               rstn,
    input  wire               in_valid,
    input  wire signed [15:0] in_data,
    // runtime parameters, 0..255 fixed-point fractions
    input  wire [7:0]         fb_level,   // feedback  (0..255)/256
    input  wire [7:0]         wet_level,  // wet mix   (0..255)/256
    output reg                out_valid,
    output reg  signed [15:0] out_data
);

    // ------------------------------------------------------------------
    // delay line (single-port sync BRAM)
    // ------------------------------------------------------------------
    reg signed [15:0] mem [0:(1<<AW)-1];
    // Startup zero-fill: hardware B-SRAM does not reliably power up as zeros,
    // and any random content is audible through the wet path (and feeds the
    // feedback loop). One cell per clock -> 4096 clk = 152 us, synthesizable
    // (the old `initial` loop could not be unrolled by GowinSynthesis: EX3935).
    reg [AW-1:0] clr_cnt;
    reg          clr_done;
    reg  [AW-1:0] raddr;
    reg  [AW-1:0] waddr;
    reg           we;
    reg  signed [15:0] wdata;
    reg  signed [15:0] rdata;

    always @(posedge clk) begin
        if (!clr_done) mem[clr_cnt] <= 16'sd0;   // startup zero-fill
        else if (we)   mem[waddr]   <= wdata;
        rdata <= mem[raddr];
    end

    // write pointer
    reg [AW-1:0] wptr;

    // tap address helpers (mod 2^AW)
    wire [AW-1:0] a1 = wptr - T1[AW-1:0];
    wire [AW-1:0] a2 = wptr - T2[AW-1:0];
    wire [AW-1:0] a3 = wptr - T3[AW-1:0];
    wire [AW-1:0] a4 = wptr - T4[AW-1:0];

    // shared multiplier
    reg  signed [15:0] m_a, m_b;
    wire signed [31:0] prod = m_a * m_b;

    // tap registers + combinational sum
    reg signed [15:0] d1, d2, d3, d4;
    wire signed [17:0] sumw = $signed({d1[15], d1}) + $signed({d2[15], d2})
                            + $signed({d3[15], d3}) + $signed({d4[15], d4});
    reg signed [15:0] avg_q;
    reg signed [15:0] dry_q;

    // FSM
    localparam [3:0]
        E0 = 4'd0, E1 = 4'd1, E2 = 4'd2, E3 = 4'd3, E4 = 4'd4,
        E5 = 4'd5, E6 = 4'd6, E7 = 4'd7, E8 = 4'd8;
    reg [3:0] est;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            est <= E0; out_valid <= 1'b0; out_data <= 16'd0;
            wptr <= 0; raddr <= 0; waddr <= 0; we <= 1'b0;
            wdata <= 16'd0;
            clr_cnt <= 0; clr_done <= 1'b0;
            m_a <= 16'd0; m_b <= 16'd0;
            d1 <= 16'd0; d2 <= 16'd0; d3 <= 16'd0; d4 <= 16'd0;
            avg_q <= 16'd0; dry_q <= 16'd0;
        end else begin
            out_valid <= 1'b0;
            we <= 1'b0;

            // one memory cell cleared per clock until the buffer is all zero
            if (!clr_done) begin
                clr_cnt <= clr_cnt + 1'b1;
                if (&clr_cnt) clr_done <= 1'b1;
            end

            case (est)
                E0: if (in_valid && clr_done) begin
                    dry_q <= in_data;
                    raddr <= a1;
                    est   <= E1;
                end
                E1: begin d1 <= rdata; raddr <= a2; est <= E2; end
                E2: begin d2 <= rdata; raddr <= a3; est <= E3; end
                E3: begin d3 <= rdata; raddr <= a4; est <= E4; end
                E4: begin
                    d4  <= rdata;                    // settle tap 4
                    est <= E5;
                end
                E5: begin
                    avg_q <= sumw[17:2];             // /4 average
                    m_a   <= sumw[17:2];
                    m_b   <= $signed({1'b0, fb_level});
                    est   <= E6;
                end
                E6: begin
                    // prod = avg * feedback
                    wdata <= dry_q + (prod >>> 16);
                    waddr <= wptr;
                    we    <= 1'b1;
                    wptr  <= wptr + 1'b1;
                    m_a   <= avg_q;
                    m_b   <= $signed({1'b0, wet_level});
                    est   <= E7;
                end
                E7: begin
                    // prod = avg * wet
                    out_data  <= dry_q + (prod >>> 16);
                    est       <= E8;
                end
                E8: begin
                    out_valid <= 1'b1;
                    est       <= E0;
                end
                default: est <= E0;
            endcase
        end
    end

endmodule
`default_nettype wire
