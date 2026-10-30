`timescale 1ns/1ps
// ============================================================================
// uart_rx.v - minimal 8N1 UART receiver (for the Bluetooth link)
//
// 40.5 MHz / 115200 = 351.56, so 352 clocks per bit -> 115057 baud, -0.12 %
// off, far inside the ~2 % a UART tolerates. Each bit is sampled at its
// centre (count 176). No parity, no flow control, no FIFO: one byte at a
// time, so the consumer has to be able to take one every ~87 us.
//
// rxd is asynchronous, hence the two-flop synchroniser - sampling an
// unsynchronised input directly is metastable, and a UART that runs for
// hours will eventually hit it.
//
// A start bit is only accepted if the line is still low at the half-bit
// point, and a byte is only handed out if the stop bit reads high. Both
// checks are what keep line noise from turning into phantom notes.
// ============================================================================
`default_nettype none

module uart_rx #(
    parameter CLKS_PER_BIT = 352
)(
    input  wire       clk,
    input  wire       rstn,
    input  wire       rxd,
    output reg  [7:0] dout,
    output reg        dout_valid
);
    // CPB 需取 16 位，不能取 9 位。
    // 9600 波特时 CLKS_PER_BIT = 4219 = 13'b1_0000_0111_1011，截成 9 位只剩
    // 123，接收器实际工作在约 329 kbaud（比 9600 快约 34 倍）：stop bit 偶尔
    // 仍能读到高电平而给出 dout_valid，解出的数据全是乱码。115200 时 CPB=352
    // 可装进 9 位，因此该问题在改用 9600 后才显现。取 16 位可容纳
    // 9600~300 波特对应的 CPB。
    localparam [15:0] CPB  = CLKS_PER_BIT[15:0];
    localparam [15:0] HALF = CLKS_PER_BIT[15:1];    // 半位，起始位中心点

    reg [1:0] rxd_sync;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) rxd_sync <= 2'b11;
        else       rxd_sync <= {rxd_sync[0], rxd};
    end

    localparam [1:0] S_IDLE = 2'd0, S_START = 2'd1, S_BIT = 2'd2, S_STOP = 2'd3;

    reg [15:0] cnt;
    reg [2:0] bitn;          // 0..7
    reg [7:0] sh;
    reg       frame_ok;
    reg [1:0] st;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            st <= S_IDLE; cnt <= 16'd0; bitn <= 3'd0; sh <= 8'd0;
            frame_ok <= 1'b0; dout <= 8'd0; dout_valid <= 1'b0;
        end else begin
            dout_valid <= 1'b0;
            case (st)
            S_IDLE: begin
                cnt <= 16'd0; bitn <= 3'd0;
                if (!rxd_sync[1]) st <= S_START;      // falling edge
            end
            S_START: begin
                if (cnt == HALF) begin
                    // still low at the centre of the start bit = real start,
                    // otherwise it was a glitch and we go back to waiting
                    if (rxd_sync[1]) st <= S_IDLE;
                    else begin cnt <= 16'd0; st <= S_BIT; end
                end else cnt <= cnt + 16'd1;
            end
            S_BIT: begin
                if (cnt == CPB - 16'd1) begin
                    cnt <= 16'd0;
                    sh <= {rxd_sync[1], sh[7:1]};     // LSB first
                    if (bitn == 3'd7) begin st <= S_STOP; frame_ok <= 1'b0; end
                    else bitn <= bitn + 3'd1;
                end else cnt <= cnt + 16'd1;
            end
            S_STOP: begin
                if (cnt == HALF) frame_ok <= rxd_sync[1];
                if (cnt == CPB - 16'd1) begin
                    cnt <= 16'd0;
                    st  <= S_IDLE;
                    if (frame_ok) begin
                        dout       <= sh;
                        dout_valid <= 1'b1;
                    end
                end else cnt <= cnt + 16'd1;
            end
            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
`default_nettype wire
