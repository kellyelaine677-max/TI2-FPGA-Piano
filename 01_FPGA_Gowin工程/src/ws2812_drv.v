`timescale 1ns/1ps
// ============================================================================
// ws2812_drv.v - WS2812B RGB strip driver with pixel RAM and auto-refresh
//
// 灯带为 12 颗，与 12 个半音槽一一对应：
// 引导模式下"第 i 颗灯亮 = 该弹第 i 个键"。
//
// Internal NPIX-entry GRB RAM, external write port, self-timed frame sender:
//   frame = NPIX pixels x 24 bits x 51 clk (794 kHz bit rate) + reset gap
//   + idle wait -> ~60 fps refresh. frame_start pulses at the reset gap.
//
// Bit timing @ 40.5 MHz system clock (WS2812B datasheet window):
//   bit period 51 clk (1.26 us), T0H = 17 clk (0.42 us), T1H = 34 clk (0.84 us)
//   reset = 2673 clk low (66 us > 50 us min)
// Every number is the 27 MHz value x 1.5, so the wire timing is unchanged.
//
// 换灯带只改 NPIX 一个参数（顶层实例化时传），像素 RAM 深度和帧长都跟着变。
// ============================================================================
`default_nettype none

module ws2812_drv #(
    parameter NPIX   = 12,
    parameter FRAME_WAIT = 24'd600000   // idle wait after reset -> ~60 fps
)(
    input  wire        clk,
    input  wire        rstn,
    // pixel write port (address held stable, one write per wr_en pulse)
    input  wire        wr_en,
    input  wire [5:0]  wr_addr,
    input  wire [23:0] wr_data,     // {G[23:16], R[15:8], B[7:0]} native GRB
    output reg         frame_start, // 1-clk pulse at start of reset gap
    output wire        dout
);

    // ------------------------------------------------------------------
    // pixel RAM (GRB order, MSB first on the wire)
    // ------------------------------------------------------------------
    reg [23:0] ram [0:NPIX-1];

    // ------------------------------------------------------------------
    // frame sender FSM
    // ------------------------------------------------------------------
    localparam [2:0] S_IDLE = 3'd0, S_BIT = 3'd1, S_RST = 3'd2, S_WAIT = 3'd3;

    reg [2:0]  st;
    reg [5:0]  pix;
    reg [4:0]  bitn;      // 23..0 within a pixel
    reg [6:0]  bcnt;      // 0..67 within a bit (7 bits for the 54 MHz rate)
    reg [23:0] sh;
    reg [23:0] wait_cnt;

    // T0H = 17 clk (0.42 us), T1H = 33 clk (0.81 us), bit period 51 clk.
    assign dout = (st == S_BIT) ? ((bcnt < (sh[bitn] ? 6'd33 : 6'd17)) ? 1'b1 : 1'b0)
                                : 1'b0;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            st <= S_IDLE; pix <= 6'd0; bitn <= 5'd23; bcnt <= 6'd0;
            sh <= 24'd0; wait_cnt <= 24'd0;
            frame_start <= 1'b0;
        end else begin
            frame_start <= 1'b0;
            // pixel RAM write port
            if (wr_en) ram[wr_addr] <= wr_data;

            case (st)
                S_IDLE: begin
                    pix <= 6'd0;
                    sh  <= ram[0];
                    bitn <= 5'd23;
                    st  <= S_BIT;
                end
                S_BIT: begin
                    if (bcnt == 6'd50) begin
                        bcnt <= 6'd0;
                        if (bitn == 5'd0) begin
                            if (pix == NPIX-1) begin
                                st <= S_RST;
                                frame_start <= 1'b1;
                            end else begin
                                pix  <= pix + 6'd1;
                                sh   <= ram[pix + 6'd1];
                                bitn <= 5'd23;
                            end
                        end else bitn <= bitn - 5'd1;
                    end else bcnt <= bcnt + 6'd1;
                end
                S_RST: begin
                    // reset gap: dout held low (st != S_BIT), ~66 us
                    if (wait_cnt >= 24'd2700) begin
                        wait_cnt <= 24'd0;
                        st       <= S_WAIT;
                    end else wait_cnt <= wait_cnt + 24'd1;
                end
                S_WAIT: begin
                    if (wait_cnt >= FRAME_WAIT) begin
                        wait_cnt <= 24'd0;
                        st <= S_IDLE;
                    end else wait_cnt <= wait_cnt + 24'd1;
                end
                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
`default_nettype wire
