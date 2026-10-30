`timescale 1ns/1ps
// ============================================================================
// bt_tx.v - stream note events out over the Bluetooth UART as MIDI bytes
//
// 拓展 4: 演奏录音回传. The recording itself lives in the phone app - the
// FPGA just streams every note on/off the moment it happens, as ordinary
// MIDI bytes. That is what keeps this cheap: no event buffer, no timestamps,
// no RAM (BSRAM is completely full anyway, so anything that needed a buffer
// was never going to fit).
//
// Messages are always 3 bytes with the status byte repeated - running status
// would save a byte but adds a state, and at 260 us per message the link is
// idle ~99.7 % of the time at human playing speed, so there is nothing to
// save it for.
//
// Events that arrive while a message is still going out are dropped. At 3800
// messages/s of headroom versus maybe 20 notes/s of playing, losing one
// needs a deliberately hostile app.
// ============================================================================
`default_nettype none

module bt_tx (
    input  wire       clk,
    input  wire       rstn,
    // note events (1-clk pulses), already gated to exclude the self-test
    input  wire       ev_on,
    input  wire       ev_off,
    input  wire [6:0] ev_num,
    output wire       txd
);
    localparam [7:0] NOTE_ON  = 8'h90;
    localparam [7:0] NOTE_OFF = 8'h80;
    localparam [7:0] VEL      = 8'h40;   // fixed: the engine does not scale it

    wire       tx_start;
    wire [7:0] tx_data;
    wire       tx_busy;

    // 352 = 115200, 4219 = 9600. Kept as a parameter on the module so
    // top_synth can set it from one place; see BT_CPB there.
    uart_tx #(.CLKS_PER_BIT(4219)) u_utx (
        .clk (clk), .rstn (rstn),
        .start (tx_start), .data (tx_data),
        .txd (txd), .busy (tx_busy)
    );

    localparam [1:0] T_IDLE = 2'd0, T_B0 = 2'd1, T_B1 = 2'd2, T_B2 = 2'd3;

    reg [1:0] st;
    reg [7:0] b0, b1, b2;

    assign tx_start = (st != T_IDLE) && !tx_busy;
    assign tx_data  = (st == T_B0) ? b0 : (st == T_B1) ? b1 : b2;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            st <= T_IDLE; b0 <= 8'd0; b1 <= 8'd0; b2 <= 8'd0;
        end else begin
            case (st)
            T_IDLE: begin
                if (ev_on) begin
                    b0 <= NOTE_ON;  b1 <= {1'b0, ev_num}; b2 <= VEL;
                    st  <= T_B0;
                end else if (ev_off) begin
                    b0 <= NOTE_OFF; b1 <= {1'b0, ev_num}; b2 <= 8'd0;
                    st  <= T_B0;
                end
            end
            T_B0: if (!tx_busy && tx_start) st <= T_B1;
            T_B1: if (!tx_busy && tx_start) st <= T_B2;
            T_B2: if (!tx_busy && tx_start) st <= T_IDLE;
            endcase
        end
    end

endmodule
`default_nettype wire
