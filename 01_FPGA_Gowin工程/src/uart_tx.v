`timescale 1ns/1ps
// ============================================================================
// uart_tx.v - minimal 8N1 UART transmitter (Bluetooth link, note events out)
//
// Same bit time as uart_rx: 40.5 MHz / 115200 = 351.56 -> 352 clocks per bit,
// 115057 baud, -0.12 % off. LSB first, one stop bit, no parity.
//
// start is a 1-cycle strobe and is only sampled while idle - events that
// arrive mid-byte are simply dropped by the caller, which is fine here: a
// note event is 3 bytes = ~260 us at this rate, and nobody plays 3800 notes
// a second.
// ============================================================================
`default_nettype none

module uart_tx #(
    parameter CLKS_PER_BIT = 352
)(
    input  wire       clk,
    input  wire       rstn,
    input  wire       start,
    input  wire [7:0] data,
    output reg        txd,
    output reg        busy
);
    // 16 位而不是 9 位：9600 波特时 CLKS_PER_BIT = 4219（13 位），截到 9 位
    // 只剩 123，发送器会跑在 329 kbaud。和 uart_rx 同一个坑，两边一起改。
    localparam [15:0] CPB = CLKS_PER_BIT[15:0];

    reg [15:0] cnt;
    reg [3:0] bitn;          // 0 = start, 1..8 = data, 9 = stop
    reg [7:0] sh;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            txd <= 1'b1; busy <= 1'b0; cnt <= 16'd0; bitn <= 4'd0; sh <= 8'd0;
        end else if (!busy) begin
            txd <= 1'b1;
            if (start) begin
                busy <= 1'b1; cnt <= 16'd0; bitn <= 4'd0;
                sh  <= data;
                txd <= 1'b0;                     // start bit goes out now
            end
        end else begin
            if (cnt != CPB - 16'd1) begin
                cnt <= cnt + 16'd1;
            end else begin
                cnt  <= 16'd0;
                bitn <= bitn + 4'd1;
                // the bit that is starting on this edge
                case (bitn + 4'd1)
                    4'd1: txd <= sh[0];
                    4'd2: txd <= sh[1];
                    4'd3: txd <= sh[2];
                    4'd4: txd <= sh[3];
                    4'd5: txd <= sh[4];
                    4'd6: txd <= sh[5];
                    4'd7: txd <= sh[6];
                    4'd8: txd <= sh[7];
                    default: txd <= 1'b1;        // 9 = stop bit
                endcase
                if (bitn == 4'd9) begin
                    busy <= 1'b0;
                    txd  <= 1'b1;
                end
            end
        end
    end

endmodule
`default_nettype wire
