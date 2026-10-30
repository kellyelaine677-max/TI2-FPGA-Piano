`timescale 1ns/1ps
// ============================================================================
// i2c_bit.v - Transaction-level open-drain I2C master bit engine
//
// One command per byte/START/STOP. The controller FSM issues a 1-cycle cmd
// strobe when busy==0; engine executes and pulses done (busy drops the same
// cycle the strobe of the NEXT command is accepted).
//
// SCL = 100 kHz class (bit = 4 quarters x 2.5 us @ 27 MHz, QUART = 68).
// Open-drain: oe=1 drives low, oe=0 releases (external pull-ups give high).
//
// Command protocol (each assumes entry with SCL low, SDA released,
// except START which may come from bus-idle or repeated-start):
//   START : SCL high, SDA high->low, then SCL low          (ready for WR)
//   WR    : 8 data bits (MSB first) + slave ACK sample     (ends SCL low)
//   RD    : 8 data bits sampled + master ACK/NACK (ack_n)  (ends SCL low)
//   STOP  : SDA low while SCL high, then SDA high          (bus idle)
// ============================================================================
`default_nettype none

module i2c_bit #(
    parameter QUART = 68         // clk cycles per I2C quarter-bit
                                  // 136 @ 54 MHz -> 2.5 us -> SCL = 100 kHz
)(
    input  wire        clk,
    input  wire        rstn,
    input  wire        cmd_start,
    input  wire        cmd_stop,
    input  wire        cmd_wr,
    input  wire        cmd_rd,
    input  wire        ack_n,      // 1 = NACK on last read byte
    input  wire [7:0]  wr_data,
    output reg  [7:0]  rd_data,
    output reg         ack_err,    // valid at done of WR
    output reg         busy,
    output reg         done,
    inout  wire        sda,
    inout  wire        scl
);

    localparam [2:0] OP_START = 3'd0, OP_WR = 3'd1, OP_RD = 3'd2, OP_STOP = 3'd3;

    reg  [2:0] op;
    reg  [1:0] phase;              // quarter within bit (0..3)
    reg  [7:0] cnt;                // clk divider inside quarter (needs 8 bits
                                   // for QUART = 136 @ 54 MHz)
    reg  [3:0] bitn;               // 0..7 data bits, 8 = ack slot
    reg         sda_oe, scl_oe;

    assign sda = sda_oe ? 1'b0 : 1'bz;
    assign scl = scl_oe ? 1'b0 : 1'bz;

    wire qlast = (cnt == QUART-1);

    // execute one quarter of the given op
    // ph = quarter index, bn = bit index (8 = ack slot)
    task exec_quarter(input [2:0] opcode, input [1:0] ph, input [3:0] bn);
        begin
            case (opcode)
            OP_WR: begin
                if (bn <= 4'd7) begin
                    case (ph)
                        2'd0: begin sda_oe <= ~wr_data[3'd7 - bn[2:0]]; scl_oe <= 1'b1; end
                        2'd1: scl_oe <= 1'b0;
                        2'd2: ;
                        2'd3: scl_oe <= 1'b1;
                    endcase
                end else begin      // ack slot
                    case (ph)
                        2'd0: sda_oe <= 1'b0;         // release, slave drives
                        2'd1: scl_oe <= 1'b0;
                        2'd2: if (sda) ack_err <= 1'b1;
                        2'd3: begin scl_oe <= 1'b1; end
                    endcase
                end
            end
            OP_RD: begin
                if (bn <= 4'd7) begin
                    case (ph)
                        2'd0: sda_oe <= 1'b0;         // release, slave drives
                        2'd1: scl_oe <= 1'b0;
                        2'd2: rd_data <= {rd_data[6:0], sda};
                        2'd3: scl_oe <= 1'b1;
                    endcase
                end else begin      // master ack/nack slot
                    case (ph)
                        2'd0: sda_oe <= ack_n ? 1'b0 : 1'b1;
                        2'd1: scl_oe <= 1'b0;
                        2'd2: ;
                        2'd3: begin scl_oe <= 1'b1; sda_oe <= 1'b0; end
                    endcase
                end
            end
            OP_START: begin
                case (ph)
                    2'd0: begin scl_oe <= 1'b0; sda_oe <= 1'b0; end
                    2'd1: sda_oe <= 1'b1;             // SDA falls, SCL high
                    2'd2: scl_oe <= 1'b1;             // SCL low, ready for data
                    2'd3: ;
                endcase
            end
            OP_STOP: begin
                case (ph)
                    2'd0: begin sda_oe <= 1'b1; scl_oe <= 1'b1; end
                    2'd1: scl_oe <= 1'b0;             // SCL high, SDA low
                    2'd2: sda_oe <= 1'b0;             // SDA rises -> STOP
                    2'd3: ;
                endcase
            end
            default: ;
            endcase
        end
    endtask

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            op <= OP_START; phase <= 2'd0; cnt <= 0; bitn <= 4'd0;
            sda_oe <= 1'b0; scl_oe <= 1'b0;
            busy <= 1'b0; done <= 1'b0; ack_err <= 1'b0; rd_data <= 8'd0;
        end else begin
            done <= 1'b0;
            if (!busy) begin
                if (cmd_start) begin
                    op <= OP_START; busy <= 1'b1; phase <= 2'd0; cnt <= 0; bitn <= 0;
                    exec_quarter(OP_START, 2'd0, 4'd0);
                end else if (cmd_wr) begin
                    op <= OP_WR; busy <= 1'b1; phase <= 2'd0; cnt <= 0; bitn <= 0;
                    ack_err <= 1'b0;
                    exec_quarter(OP_WR, 2'd0, 4'd0);
                end else if (cmd_rd) begin
                    op <= OP_RD; busy <= 1'b1; phase <= 2'd0; cnt <= 0; bitn <= 0;
                    exec_quarter(OP_RD, 2'd0, 4'd0);
                end else if (cmd_stop) begin
                    op <= OP_STOP; busy <= 1'b1; phase <= 2'd0; cnt <= 0; bitn <= 0;
                    exec_quarter(OP_STOP, 2'd0, 4'd0);
                end
            end else if (qlast) begin
                cnt <= 0;
                if (phase != 2'd3) begin
                    phase <= phase + 2'd1;
                    exec_quarter(op, phase + 2'd1, bitn);
                end else begin
                    // end of quarter 3
                    if ((op == OP_START) || (op == OP_STOP) || (bitn == 4'd8)) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                    end else begin
                        bitn <= bitn + 4'd1;
                        phase <= 2'd0;
                        exec_quarter(op, 2'd0, bitn + 4'd1);
                    end
                end
            end else begin
                cnt <= cnt + 1;
            end
        end
    end

endmodule
`default_nettype wire
