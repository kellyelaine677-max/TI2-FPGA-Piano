`timescale 1ns/1ps
// ============================================================================
// st7789_drv.v - ST7789V 240x320 SPI LCD driver with RGB332 framebuffer
//
// Owns the LCD serial bus. Power-up sequence:
//   RST low 15 ms -> RST high 10 ms -> init command script (~350 ms)
//   -> backlight on -> endless refresh:
//        RAMWR(2C) + 76800 pixels (dc=1): each fb byte (RGB332) expands to
//        RGB565 and streams as 2 SPI bytes, MSB first.
//
// SPI: one bit per 2 clk -> SCL = 13.5 MHz; data changes while SCL low.
// CS tied low, DC switches command/data.
//
// Handshake (level, loss-free): issuer sets tx_req=1 with tx_byte/tx_dc when
// the engine is idle; engine latches on the next clk and runs 16 SCL edges;
// issuer clears tx_req once spi_busy is seen.
//
// Framebuffer: 240x320 x 8 bit = 76.8 KB (inferred simple-dual-port BSRAM).
//   Port A: pixel write (wr_en + 17-bit address + 8-bit RGB332 data)
//   Port B: sequential synchronous read for refresh scanout
// ============================================================================
`default_nettype none

module st7789_drv (
    input  wire        clk,
    input  wire        rstn,
    input  wire        chip_sel,   // 1'b1 = ST7789, 1'b0 = ILI9341
    input  wire        inv_sel,    // 1'b1 = INVON, 1'b0 = INVOFF
    input  wire        reinit,     // 1-clk pulse: restart the init sequencer
    // pixel write port
    input  wire        wr_en,
    input  wire [16:0] wr_addr,      // y*240 + x
    input  wire [7:0]  wr_data,      // RGB332
    // LCD pins
    output wire        lcd_scl,
    output wire        lcd_sda,
    output wire        lcd_dc,       // 0 = command, 1 = data
    output wire        lcd_cs,       // tied low
    output reg         lcd_rst,
    output reg         lcd_bl,       // backlight, high = on
    output wire        disp_ready,   // high once init finished
    input  wire        self_test,    // diagnostic: stream cycling solid colour
    input  wire        lcd_sdo,      // panel MISO - read-back for ID probe
    output wire        id_st7789,    // panel answered RDDID 04h = 85 85 52
    output wire        id_ili9341,   // panel answered D3h     = 00 93 41
    output wire        id_any        // SDO carried any non-zero bit at all
);

    // CS framing: assert (low) for exactly one byte, release (high) between
    // bytes. It used to be tied permanently low, and several ST7789/ILI9341
    // controllers latch a command on the CS RISING edge - with CS held low the
    // bytes never get committed, so the panel accepts nothing and stays white.
    // (This is also how an STM32 drives it, which is why the same panel works
    // there. sst is declared below; referencing it here is fine in Verilog.)
    assign lcd_cs = (sst == S_IDLE);

    // ------------------------------------------------------------------
    // framebuffer
    // ------------------------------------------------------------------
    reg [7:0] fb [0:76799];
    reg [7:0] rd_pipe;        // scanout pixel byte
    reg [16:0] rd_addr;       // sequential scanout address

    // Startup zero-fill: hardware B-SRAM does not reliably power up as zeros,
    // and THIS is what made the panel pure white - RGB332 0xFF is white, so an
    // uncleared framebuffer streams a white screen. 76800 clk = 2.8 ms, fully
    // synthesizable (an `initial` loop of this size is rejected: EX3935).
    reg [16:0] clr_addr;
    reg        clr_done;

    always @(posedge clk) begin
        if (!clr_done) fb[clr_addr] <= 8'd0;            // startup clear
        else if (wr_en) fb[wr_addr] <= wr_data;
        rd_pipe <= fb[rd_addr];
    end

    // ------------------------------------------------------------------
    // millisecond tick
    // ------------------------------------------------------------------
    reg [15:0] ms_cnt;
`ifdef SIM_FAST
    // simulation only: ~1 us tick so the full init+first frame runs in ms
    wire ms_tick = (ms_cnt == 16'd26);
`else
    wire ms_tick = (ms_cnt == 16'd40499);   // 1 ms @ 40.5 MHz
`endif
    always @(posedge clk or negedge rstn) begin
        if (!rstn) ms_cnt <= 16'd0;
        else ms_cnt <= ms_tick ? 16'd0 : ms_cnt + 16'd1;
    end
    // NOTE: do NOT keep a free-running ms counter here - the sequencer below
    // needs its own millisecond counter (seq_ms), or the register ends up
    // driven from two always blocks (Gowin EX3999 multi-driver).

    // ------------------------------------------------------------------
    // SPI byte engine
    // ------------------------------------------------------------------
    reg  [7:0] spi_sh;
    reg        rx_en;    // top-owned: capture SDO while clocking a read byte
    reg        rx_q;     // engine-owned per-byte latch of rx_en
    reg [31:0] rx_sh;    // 32 sampled SDO bits (bit31 = first sampled)
    // SPI mode 0: the panel samples DIN on the SCL RISING edge, so DIN must
    // change on the FALLING edge and be stable before the next rise. The old
    // 2-state FSM toggled DIN and SCL on the SAME clock edge, so the panel
    // captured the previous bit -> every byte arrived shifted right by one
    // (0x11 -> 0x08) -> garbage commands -> permanently white screen.
    reg  [2:0] sst;           // 0 idle, 1 prep, 2 SCL high, 3 SCL low, 4 done
    localparam [2:0]
        S_IDLE = 3'd0, S_PREP = 3'd1, S_HIGH = 3'd2, S_LOW = 3'd3, S_DONE = 3'd4;
    reg  [3:0] bitc;          // bits clocked out in the current byte (0..7)
    reg        dc_q;
    reg        tx_req;         // level request (top-owned)
    reg  [7:0] tx_byte;        // top-owned
    reg        tx_dc;          // top-owned

    // ------------------------------------------------------------------
    // SPI bit-rate divider
    // ------------------------------------------------------------------
    // Each SCL half-period lasts DIV_MAX+1 core clocks, so
    //   SCL = clk / (2*(DIV_MAX+1)) = 40.5 MHz / 6 = 6.75 MHz.
    // The rate is deliberately unchanged from the 27 MHz build (where
    // DIV_MAX = 1 gave the same 6.75 MHz), so the ~5.5 fps full-frame
    // refresh the spectrum/Lissajous display relies on is identical.
    // Drop back to DIV_MAX = 5 (3.375 MHz) if long jumper wires glitch.
    reg [3:0] div_cnt;
    localparam [3:0] DIV_MAX = 4'd2;   // SCL = 40.5 MHz / 6 = 6.75 MHz
    wire div_done = (div_cnt == DIV_MAX);

    assign lcd_scl   = (sst == S_HIGH);
    assign lcd_sda   = spi_sh[7];
    assign lcd_dc    = dc_q;
    wire spi_busy    = (sst != S_IDLE);
    wire tx_free     = !tx_req && !spi_busy;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            sst <= S_IDLE; spi_sh <= 8'd0; dc_q <= 1'b0; bitc <= 4'd0;
            rx_q <= 1'b0; rx_sh <= 32'd0; div_cnt <= 4'd0;
        end else begin
            case (sst)
                // load the byte, then give it one clk to settle while SCL is low
                S_IDLE: if (tx_req) begin
                    spi_sh <= tx_byte;
                    dc_q   <= tx_dc;
                    rx_q   <= rx_en;
                    div_cnt<= 4'd0;
                    sst    <= S_PREP;
                end
                S_PREP: begin
                    bitc    <= 4'd0;
                    div_cnt <= 4'd0;
                    sst     <= S_HIGH;              // SCL still low
                end
                // SCL high phase: the panel samples DIN on the RISING edge, so
                // DIN must already be stable - it was set up during S_PREP /
                // the previous S_LOW. Wait out the divider, then drop SCL and
                // change DIN on the FALLING edge (strict SPI mode 0). Exactly
                // 8 bits are always clocked.
                S_HIGH: begin
                    if (!div_done) begin
                        div_cnt <= div_cnt + 4'd1;
                    end else begin
                        div_cnt <= 4'd0;
                        // read-back: panel shifts SDO on the SCL falling edge,
                        // so the bit is stable during S_HIGH - sample it here
                        if (rx_q) rx_sh <= {rx_sh[30:0], lcd_sdo};
                        spi_sh <= {spi_sh[6:0], 1'b0};
                        if (bitc == 4'd7) begin
                            bitc <= 4'd0;
                            sst  <= S_DONE;
                        end else begin
                            bitc <= bitc + 4'd1;
                            sst  <= S_LOW;
                        end
                    end
                end
                S_LOW: begin
                    if (!div_done) begin
                        div_cnt <= div_cnt + 4'd1;
                    end else begin
                        div_cnt <= 4'd0;
                        sst <= S_HIGH;              // SCL rise: DIN stable
                    end
                end
                S_DONE: sst <= S_IDLE;                  // byte complete
                default: sst <= S_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // init script  (UNION: brings up BOTH ST7789 and ILI9341)
    // ------------------------------------------------------------------
    // The two controllers share almost all commands, so one "fat" init drives
    // either panel. ST7789 ignores the ILI9341-only registers (0xCF/0xED/0xE8/
    // 0xCB/0xF7/0xEA) and runs the rest; ILI9341 NEEDS the Power/VCOM/Gamma
    // block or it powers up WHITE even though the bus is perfectly happy.
    // This is the scope-free fix for "link fine, screen white, not sure which
    // chip" - one bitstream covers both. MADCTL stays 0x00 (no flip, RGB
    // order) to match our 240x320 framebuffer mapping; inversion stays OFF.
    wire [7:0] INV_CMD = inv_sel ? 8'h21 : 8'h20;   // 0x20 INVOFF / 0x21 INVON
    // ST7789 and ILI9341 need different init lengths; select by chip_sel.
    // NOTE: must be a wire, not a localparam - chip_sel is a runtime input.
    wire [6:0] N_INIT = (chip_sel == 1'b1) ? 7'd73 : 7'd96;

    // Init script as combinational ROM (case-based functions).
    // The previous version used reg arrays filled by an `initial` block, which
    // GowinSynthesis turned INTO RAM ("Extracting RAM for identifier ..."), and
    // Gowin RAM cannot be reliably initialised -> the panel received a random
    // command stream and stayed a blank white screen. Functions always map to
    // LUTs, so the bytes are part of the logic and cannot be lost.
    function        ini_dc_f;
        input [6:0] i;
        begin
            if (chip_sel == 1'b1) begin
                // ST7789: data (parameter) bytes are the indexed param values
                case (i)
                    7'd2,7'd4, 7'd6,7'd7,7'd8,7'd9,7'd10,
                    7'd12,7'd14,7'd16,7'd18,7'd19,7'd21,7'd22,7'd24,7'd25,
                    7'd27,7'd29,7'd30,
                    7'd32,7'd33,7'd34,7'd35,7'd36,7'd37,7'd38,7'd39,7'd40,
                    7'd41,7'd42,7'd43,7'd44,7'd45,
                    7'd47,7'd48,7'd49,7'd50,7'd51,7'd52,7'd53,7'd54,7'd55,
                    7'd56,7'd57,7'd58,7'd59,7'd60,
                    7'd64,7'd65,7'd66,7'd67, 7'd69,7'd70,7'd71,7'd72:
                        ini_dc_f = 1'b1;
                    default: ini_dc_f = 1'b0;   // command bytes
                endcase
            end else begin
                // ILI9341 (union) - existing table
                case (i)
                    // 0xCF 3 params, 0xED 4, 0xE8 3, 0xCB 5, 0xF7 1, 0xEA 2
                    7'd2,7'd3,7'd4, 7'd6,7'd7,7'd8,7'd9, 7'd11,7'd12,7'd13,
                    7'd15,7'd16,7'd17,7'd18,7'd19, 7'd21, 7'd23,7'd24,
                    // 0xC0 1, 0xC1 1, 0xC5 2, 0xC7 1, 0x3A 1, 0x36 1
                    7'd26, 7'd28, 7'd30,7'd31, 7'd33, 7'd35, 7'd37,
                    // 0x2A 4, 0x2B 4, 0xB1 2, 0xB6 3, 0xF2 1, 0x26 1
                    7'd39,7'd40,7'd41,7'd42, 7'd44,7'd45,7'd46,7'd47,
                    7'd49,7'd50, 7'd52,7'd53,7'd54, 7'd56, 7'd58,
                    // 0xE0 15, 0xE1 15
                    7'd60,7'd61,7'd62,7'd63,7'd64,7'd65,7'd66,7'd67,7'd68,7'd69,
                    7'd70,7'd71,7'd72,7'd73,7'd74,
                    7'd76,7'd77,7'd78,7'd79,7'd80,7'd81,7'd82,7'd83,7'd84,7'd85,
                    7'd86,7'd87,7'd88,7'd89,7'd90:
                        ini_dc_f = 1'b1;    // parameter (data) bytes
                    default: ini_dc_f = 1'b0;   // command bytes
                endcase
            end
        end
    endfunction

    function [7:0] ini_byte_f;
        input [6:0] i;
        begin
            if (chip_sel == 1'b1) begin
                // ============================================================
                // ST7789 240x320 init (standard porch/VCOM + INVON)
                // ============================================================
                case (i)
                    7'd0:  ini_byte_f = 8'h11;   // SLPOUT
                    7'd1:  ini_byte_f = 8'h36;   // MADCTL (no flip, RGB)
                    7'd2:  ini_byte_f = 8'h00;
                    7'd3:  ini_byte_f = 8'h3A;   // COLMOD
                    7'd4:  ini_byte_f = 8'h55;   // 16-bit RGB565
                    7'd5:  ini_byte_f = 8'hB2;   // porch
                    7'd6:  ini_byte_f = 8'h0C;
                    7'd7:  ini_byte_f = 8'h0C;
                    7'd8:  ini_byte_f = 8'h00;
                    7'd9:  ini_byte_f = 8'h33;
                    7'd10: ini_byte_f = 8'h33;
                    7'd11: ini_byte_f = 8'hB7;   // gate control
                    7'd12: ini_byte_f = 8'h35;
                    7'd13: ini_byte_f = 8'hBB;   // VCOMS
                    7'd14: ini_byte_f = 8'h19;
                    7'd15: ini_byte_f = 8'hC0;   // LCM
                    7'd16: ini_byte_f = 8'h2C;
                    7'd17: ini_byte_f = 8'hC2;   // VDV
                    7'd18: ini_byte_f = 8'h01;
                    7'd19: ini_byte_f = 8'h01;
                    7'd20: ini_byte_f = 8'hC3;   // VRH
                    7'd21: ini_byte_f = 8'h12;
                    7'd22: ini_byte_f = 8'h3C;
                    7'd23: ini_byte_f = 8'hC4;   // VDV set
                    7'd24: ini_byte_f = 8'h20;
                    7'd25: ini_byte_f = 8'h00;
                    7'd26: ini_byte_f = 8'hC6;   // FRCTRL2
                    7'd27: ini_byte_f = 8'h0F;
                    7'd28: ini_byte_f = 8'hD0;   // power control
                    7'd29: ini_byte_f = 8'hA4;
                    7'd30: ini_byte_f = 8'hA1;
                    7'd31: ini_byte_f = 8'hE0;   // gamma +
                    7'd32: ini_byte_f = 8'hD0; 7'd33: ini_byte_f = 8'h04;
                    7'd34: ini_byte_f = 8'h0D; 7'd35: ini_byte_f = 8'h11;
                    7'd36: ini_byte_f = 8'h13; 7'd37: ini_byte_f = 8'h2B;
                    7'd38: ini_byte_f = 8'h3F; 7'd39: ini_byte_f = 8'h54;
                    7'd40: ini_byte_f = 8'h4C; 7'd41: ini_byte_f = 8'h18;
                    7'd42: ini_byte_f = 8'h0D; 7'd43: ini_byte_f = 8'h0B;
                    7'd44: ini_byte_f = 8'h1F; 7'd45: ini_byte_f = 8'h23;
                    7'd46: ini_byte_f = 8'hE1;   // gamma -
                    7'd47: ini_byte_f = 8'hD0; 7'd48: ini_byte_f = 8'h04;
                    7'd49: ini_byte_f = 8'h0C; 7'd50: ini_byte_f = 8'h11;
                    7'd51: ini_byte_f = 8'h13; 7'd52: ini_byte_f = 8'h2C;
                    7'd53: ini_byte_f = 8'h3F; 7'd54: ini_byte_f = 8'h44;
                    7'd55: ini_byte_f = 8'h51; 7'd56: ini_byte_f = 8'h2F;
                    7'd57: ini_byte_f = 8'h1F; 7'd58: ini_byte_f = 8'h1F;
                    7'd59: ini_byte_f = 8'h20; 7'd60: ini_byte_f = 8'h23;
                    7'd61: ini_byte_f = INV_CMD; // 0x21 INVON / 0x20 INVOFF
                                                 // (panel polarity: this module
                                                 // renders INVOFF correctly)
                    7'd62: ini_byte_f = 8'h29;   // DISPON
                    7'd63: ini_byte_f = 8'h2A;   // CASET 0..239
                    7'd64: ini_byte_f = 8'h00; 7'd65: ini_byte_f = 8'h00;
                    7'd66: ini_byte_f = 8'h00; 7'd67: ini_byte_f = 8'hEF;
                    7'd68: ini_byte_f = 8'h2B;   // RASET 0..319
                    7'd69: ini_byte_f = 8'h00; 7'd70: ini_byte_f = 8'h00;
                    7'd71: ini_byte_f = 8'h01; 7'd72: ini_byte_f = 8'h3F;
                    7'd73: ini_byte_f = 8'h2C;   // RAMWR
                    default: ini_byte_f = 8'h00;
                endcase
            end else begin
                // ============================================================
                // ILI9341 (union) init - original table, unchanged
                // ============================================================
                case (i)
                    7'd0:  ini_byte_f = 8'h01;   // SWRESET
                    // --- ILI9341 power-on block (ignored by ST7789) ---
                    7'd1:  ini_byte_f = 8'hCF;   // Power Control B
                    7'd2:  ini_byte_f = 8'h00;
                    7'd3:  ini_byte_f = 8'hC1;
                    7'd4:  ini_byte_f = 8'h30;
                    7'd5:  ini_byte_f = 8'hED;   // Power on sequence
                    7'd6:  ini_byte_f = 8'h64;
                    7'd7:  ini_byte_f = 8'h03;
                    7'd8:  ini_byte_f = 8'h12;
                    7'd9:  ini_byte_f = 8'h81;
                    7'd10: ini_byte_f = 8'hE8;   // Driver timing
                    7'd11: ini_byte_f = 8'h85;
                    7'd12: ini_byte_f = 8'h00;
                    7'd13: ini_byte_f = 8'h78;
                    7'd14: ini_byte_f = 8'hCB;   // Power Control A
                    7'd15: ini_byte_f = 8'h39;
                    7'd16: ini_byte_f = 8'h2C;
                    7'd17: ini_byte_f = 8'h00;
                    7'd18: ini_byte_f = 8'h34;
                    7'd19: ini_byte_f = 8'h02;
                    7'd20: ini_byte_f = 8'hF7;   // Pump ratio
                    7'd21: ini_byte_f = 8'h20;
                    7'd22: ini_byte_f = 8'hEA;   // Driver timing
                    7'd23: ini_byte_f = 8'h00;
                    7'd24: ini_byte_f = 8'h00;
                    // --- shared power / interface config ---
                    7'd25: ini_byte_f = 8'hC0;   // Power Control 1 (Vreg1)
                    7'd26: ini_byte_f = 8'h23;
                    7'd27: ini_byte_f = 8'hC1;   // Power Control 2 (Vreg2)
                    7'd28: ini_byte_f = 8'h10;
                    7'd29: ini_byte_f = 8'hC5;   // VCOM Control 1
                    7'd30: ini_byte_f = 8'h3E;
                    7'd31: ini_byte_f = 8'h28;
                    7'd32: ini_byte_f = 8'hC7;   // VCOM Control 2
                    7'd33: ini_byte_f = 8'h86;
                    7'd34: ini_byte_f = 8'h3A;   // COLMOD
                    7'd35: ini_byte_f = 8'h55;   // 16-bit/pixel RGB565
                    7'd36: ini_byte_f = 8'h36;   // MADCTL
                    7'd37: ini_byte_f = 8'h00;   // no flip, RGB order
                    7'd38: ini_byte_f = 8'h2A;   // CASET 0..239
                    7'd39: ini_byte_f = 8'h00;
                    7'd40: ini_byte_f = 8'h00;
                    7'd41: ini_byte_f = 8'h00;
                    7'd42: ini_byte_f = 8'hEF;
                    7'd43: ini_byte_f = 8'h2B;   // RASET 0..319
                    7'd44: ini_byte_f = 8'h00;
                    7'd45: ini_byte_f = 8'h00;
                    7'd46: ini_byte_f = 8'h01;
                    7'd47: ini_byte_f = 8'h3F;
                    7'd48: ini_byte_f = 8'hB1;   // Frame rate
                    7'd49: ini_byte_f = 8'h00;
                    7'd50: ini_byte_f = 8'h18;
                    7'd51: ini_byte_f = 8'hB6;   // Display Function
                    7'd52: ini_byte_f = 8'h08;
                    7'd53: ini_byte_f = 8'h82;
                    7'd54: ini_byte_f = 8'h27;
                    7'd55: ini_byte_f = 8'hF2;   // Gamma enable
                    7'd56: ini_byte_f = 8'h00;
                    7'd57: ini_byte_f = 8'h26;   // Gamma curve
                    7'd58: ini_byte_f = 8'h01;
                    7'd59: ini_byte_f = 8'hE0;   // Gamma +
                    7'd60: ini_byte_f = 8'h0F;
                    7'd61: ini_byte_f = 8'h31;
                    7'd62: ini_byte_f = 8'h2B;
                    7'd63: ini_byte_f = 8'h0C;
                    7'd64: ini_byte_f = 8'h0E;
                    7'd65: ini_byte_f = 8'h08;
                    7'd66: ini_byte_f = 8'h4E;
                    7'd67: ini_byte_f = 8'hF1;
                    7'd68: ini_byte_f = 8'h37;
                    7'd69: ini_byte_f = 8'h07;
                    7'd70: ini_byte_f = 8'h10;
                    7'd71: ini_byte_f = 8'h03;
                    7'd72: ini_byte_f = 8'h0E;
                    7'd73: ini_byte_f = 8'h09;
                    7'd74: ini_byte_f = 8'h00;
                    7'd75: ini_byte_f = 8'hE1;   // Gamma -
                    7'd76: ini_byte_f = 8'h00;
                    7'd77: ini_byte_f = 8'h0E;
                    7'd78: ini_byte_f = 8'h14;
                    7'd79: ini_byte_f = 8'h03;
                    7'd80: ini_byte_f = 8'h11;
                    7'd81: ini_byte_f = 8'h07;
                    7'd82: ini_byte_f = 8'h31;
                    7'd83: ini_byte_f = 8'hC1;
                    7'd84: ini_byte_f = 8'h48;
                    7'd85: ini_byte_f = 8'h08;
                    7'd86: ini_byte_f = 8'h0F;
                    7'd87: ini_byte_f = 8'h0C;
                    7'd88: ini_byte_f = 8'h31;
                    7'd89: ini_byte_f = 8'h36;
                    7'd90: ini_byte_f = 8'h0F;
                    // --- wake + display on ---
                    7'd91: ini_byte_f = 8'h11;   // SLPOUT
                    7'd92: ini_byte_f = INV_CMD;  // INVOFF (keep black black)
                    7'd93: ini_byte_f = 8'h13;   // NORON
                    7'd94: ini_byte_f = 8'h29;   // DISPON
                    7'd95: ini_byte_f = 8'h2C;   // RAMWR (start pixel stream)
                    default: ini_byte_f = 8'h00;
                endcase
            end
        end
    endfunction

    function [7:0] ini_dly_f;
        input [6:0] i;
        begin
            if (chip_sel == 1'b1) begin
                // ST7789: SLPOUT needs 120 ms, DISPON 50 ms
                case (i)
                    7'd0:  ini_dly_f = 8'd120;   // after SLPOUT
                    7'd62: ini_dly_f = 8'd50;    // after DISPON
                    default: ini_dly_f = 8'd0;
                endcase
            end else begin
                // ILI9341
                case (i)
                    7'd0:  ini_dly_f = 8'd150;   // after SWRESET
                    7'd91: ini_dly_f = 8'd120;   // after SLPOUT
                    7'd94: ini_dly_f = 8'd50;    // after DISPON
                    default: ini_dly_f = 8'd0;
                endcase
            end
        end
    endfunction

    // RGB332 -> RGB565 expansion of the scanout byte
    wire [2:0] cr = rd_pipe[7:5];
    wire [2:0] cg = rd_pipe[4:2];
    wire [1:0] cb = rd_pipe[1:0];
    wire [15:0] px = {cr, cr[2:1], cg, cg, cb, cb, cb[1]};

    // ------------------------------------------------------------------
    // self-test: stream a cycling solid colour (red -> green -> blue ->
    // white -> black, 1 s per step) whenever self_test is asserted. No
    // framebuffer, no pattern - it only proves the panel is actually
    // executing our RAMWR writes. Tie self_test to the dip_test jumper.
    //   sees red/green/blue cycling -> bus + panel OK, fix the pattern;
    //   still pure white            -> panel ignores our writes (wrong
    //                                  chip / interface mode / wiring).
    // ------------------------------------------------------------------
    reg [9:0] st_ms;     // 0..999 ms -> 1 s step
    reg [2:0] st_idx;    // 0 red, 1 green, 2 blue, 3 white, 4 black
    wire [15:0] st_color =
        (st_idx == 3'd0) ? 16'hF800 :   // red
        (st_idx == 3'd1) ? 16'h07E0 :   // green
        (st_idx == 3'd2) ? 16'h001F :   // blue
        (st_idx == 3'd3) ? 16'hFFFF :   // white
                            16'h0000;   // black
    wire [15:0] px_out = self_test ? st_color : px;

    // ------------------------------------------------------------------
    // ID read-back (diagnostic): right after init, clock RDDID (04h) and
    // Read ID4 (D3h) and capture 32 SDO bits each. Known answers:
    //   ST7789V  04h -> 85 85 52     ILI9341  D3h -> 00 93 41
    // (each preceded by one dummy clock, so the 24 ID bits sit at
    //  rx_sh[30:7]; the [23:0] check also covers a full dummy byte.)
    // The top maps the three flags to the onboard LEDs - the panel
    // identifies itself even when the glass shows nothing.
    // ------------------------------------------------------------------
    reg [31:0] id04_r, idd3_r;
    reg [1:0]  rd_cnt;
    // The 24 ID bits can land at any of 9 positions in the 32 sampled SDO bits,
    // depending on how many dummy clocks the controller inserts before its
    // data. OR together all 9 sliding 24-bit windows so the decode is
    // alignment-safe for both ST7789 (RDDID 04h -> 85 85 52) and ILI9341
    // (RDID4 D3h -> 00 93 41). Written as explicit ORs (no function/loop) so it
    // maps cleanly to both iverilog and the Gowin synthesizer.
    wire [23:0] ST7789_ID = 24'h858552;
    wire [23:0] ILI9341_ID = 24'h009341;
    assign id_st7789 =
           (id04_r[31:8] == ST7789_ID) || (id04_r[30:7] == ST7789_ID) ||
           (id04_r[29:6] == ST7789_ID) || (id04_r[28:5] == ST7789_ID) ||
           (id04_r[27:4] == ST7789_ID) || (id04_r[26:3] == ST7789_ID) ||
           (id04_r[25:2] == ST7789_ID) || (id04_r[24:1] == ST7789_ID) ||
           (id04_r[23:0] == ST7789_ID);
    assign id_ili9341 =
           (idd3_r[31:8] == ILI9341_ID) || (idd3_r[30:7] == ILI9341_ID) ||
           (idd3_r[29:6] == ILI9341_ID) || (idd3_r[28:5] == ILI9341_ID) ||
           (idd3_r[27:4] == ILI9341_ID) || (idd3_r[26:3] == ILI9341_ID) ||
           (idd3_r[25:2] == ILI9341_ID) || (idd3_r[24:1] == ILI9341_ID) ||
           (idd3_r[23:0] == ILI9341_ID);
    // id_any = the panel actually DROVE the SDO line with real data, not just
    // floating high (all-1) or stuck low (all-0). A floating SDO would
    // otherwise fake a "responds" reading and send us down the wrong path.
    assign id_any = (((|id04_r) && (~&id04_r)) || ((|idd3_r) && (~&idd3_r)));

    // ------------------------------------------------------------------
    // top-level sequencer
    // ------------------------------------------------------------------
    localparam [3:0] T_RST0 = 4'd0, T_RST1 = 4'd1, T_INIT = 4'd2,
                     T_FETCH = 4'd3, T_HI = 4'd4, T_LO = 4'd5, T_WRAP = 4'd6,
                     T_IDC = 4'd7, T_IDR = 4'd8, T_IDSNAP = 4'd9,
                     T_ID2C = 4'd10, T_ID2R = 4'd11, T_IDSNAP2 = 4'd12;
    reg [3:0]  tst;
    reg [7:0]  seq_ms;    // sequencer-owned ms counter (single driver!)
    reg [6:0]  ini_idx;
    reg [7:0]  dly_left;
    reg        dly_run;
    reg        ini_done;
    // (rx_en is declared with the SPI engine above and driven here)

    assign disp_ready = ini_done;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            tst <= T_RST0; lcd_rst <= 1'b0; lcd_bl <= 1'b0;
            seq_ms <= 8'd0;
            clr_addr <= 17'd0; clr_done <= 1'b0;
            ini_idx <= 5'd0; dly_left <= 8'd0; dly_run <= 1'b0;
            rd_addr <= 17'd0;   // rd_pipe: driven by the scanout block only
            ini_done <= 1'b0;
            tx_req <= 1'b0; tx_byte <= 8'd0; tx_dc <= 1'b0;
            rx_en  <= 1'b0;
            st_ms  <= 10'd0; st_idx <= 3'd0;
            rd_cnt <= 2'd0; id04_r <= 32'd0; idd3_r <= 32'd0;
        end else if (reinit) begin
            // soft re-init: re-run the whole power-up + init script with the
            // current chip_sel/inv_sel. Framebuffer clear is kept (already
            // black) and the BACKLIGHT IS LEFT ON on purpose - blinking it
            // here made the panel "flash" every cycle and hid what the glass
            // was really doing. Driven by a 1-clk pulse on variant change.
            tst <= T_RST0; lcd_rst <= 1'b0;
            seq_ms <= 8'd0;
            ini_idx <= 7'd0; dly_left <= 8'd0; dly_run <= 1'b0;
            ini_done <= 1'b0;
            tx_req <= 1'b0; tx_byte <= 8'd0; tx_dc <= 1'b0;
            rx_en  <= 1'b0;
            st_ms  <= 10'd0; st_idx <= 3'd0;
            rd_cnt <= 2'd0; id04_r <= 32'd0; idd3_r <= 32'd0;
        end else begin
            // request release once the engine has latched it
            if (tx_req && spi_busy) tx_req <= 1'b0;

            // sequencer ms counter (cleared by the states below on
            // transition cycles - case wins over this default assign)
            if (ms_tick) seq_ms <= seq_ms + 8'd1;

            // framebuffer zero-fill: one byte per clock until the whole
            // 76800-byte buffer is black - nothing is shown before this
            // finishes, so no garbage frame ever reaches the panel.
            if (!clr_done) begin
                clr_addr <= clr_addr + 17'd1;
                if (clr_addr == 17'd76799) clr_done <= 1'b1;
            end

            // delay engine
            if (dly_run && ms_tick) begin
                if (dly_left == 8'd1) begin dly_run <= 1'b0; dly_left <= 8'd0; end
                else dly_left <= dly_left - 8'd1;
            end

            // self-test: advance colour every 1 s while self_test is set
            if (self_test && ms_tick) begin
                if (st_ms == 10'd999) begin
                    st_ms  <= 10'd0;
                    st_idx <= (st_idx == 3'd4) ? 3'd0 : st_idx + 3'd1;
                end else begin
                    st_ms <= st_ms + 10'd1;
                end
            end

            case (tst)
                T_RST0: begin
                    lcd_rst <= 1'b0;
                    // wait for the framebuffer clear before driving the panel
                    if (clr_done && seq_ms == 8'd15) begin
                        tst <= T_RST1; seq_ms <= 8'd0;
                    end
                end
                T_RST1: begin
                    lcd_rst <= 1'b1;
                    if (seq_ms == 8'd10) begin tst <= T_INIT; seq_ms <= 8'd0; end
                end
                T_INIT: begin
                    if (!dly_run && tx_free) begin
                        if (ini_idx == N_INIT) begin
                            ini_done <= 1'b1;
                            lcd_bl   <= 1'b1;
                            tst      <= T_IDC;   // probe panel ID before pixels
                        end else begin
                            tx_byte <= ini_byte_f(ini_idx);
                            tx_dc   <= ini_dc_f(ini_idx);
                            tx_req  <= 1'b1;
                            if (ini_dly_f(ini_idx) != 8'd0) begin
                                dly_left <= ini_dly_f(ini_idx);
                                dly_run  <= 1'b1;
                            end
                            ini_idx <= ini_idx + 7'd1;
                        end
                    end
                end
                // fetch pixel byte: rd_pipe is loaded every clock by the
                // scanout block above (rd_pipe <= fb[rd_addr]), so just wait
                // one cycle for it to settle before clocking it out
                T_FETCH: begin
                    tst <= T_HI;
                end
                // send RGB565 high byte
                T_HI: begin
                    if (tx_free) begin
                        tx_byte <= px_out[15:8];
                        tx_dc   <= 1'b1;
                        tx_req  <= 1'b1;
                        tst     <= T_LO;
                    end
                end
                // send RGB565 low byte, advance
                T_LO: begin
                    if (tx_free) begin
                        tx_byte <= px_out[7:0];
                        tx_dc   <= 1'b1;
                        tx_req  <= 1'b1;
                        if (rd_addr == 17'd76799) begin
                            rd_addr <= 17'd0;
                            tst     <= T_WRAP;   // re-issue RAMWR, next frame
                        end else begin
                            rd_addr <= rd_addr + 17'd1;
                            tst     <= T_FETCH;
                        end
                    end
                end
                T_WRAP: begin
                    if (tx_free) begin
                        tx_byte <= 8'h2C;        // RAMWR
                        tx_dc   <= 1'b0;
                        tx_req  <= 1'b1;
                        tst     <= T_FETCH;
                    end
                end
                // ---- ID probe: 04h then D3h, 4 read bytes each ----
                T_IDC: if (tx_free) begin
                    tx_byte <= 8'h04;  tx_dc <= 1'b0;  tx_req <= 1'b1;
                    rd_cnt  <= 2'd0;
                    tst     <= T_IDR;
                end
                T_IDR: if (tx_free) begin
                    tx_byte <= 8'h00;  tx_dc <= 1'b1;  tx_req <= 1'b1;
                    rx_en   <= 1'b1;
                    rd_cnt  <= rd_cnt + 2'd1;
                    if (rd_cnt == 2'd3) tst <= T_IDSNAP;
                end
                T_IDSNAP: begin
                    rx_en <= 1'b0;
                    if (!spi_busy) begin
                        id04_r <= rx_sh;     // byte stream done, 32 bits in
                        tst    <= T_ID2C;
                    end
                end
                T_ID2C: if (tx_free) begin
                    tx_byte <= 8'hD3;  tx_dc <= 1'b0;  tx_req <= 1'b1;
                    rd_cnt  <= 2'd0;
                    tst     <= T_ID2R;
                end
                T_ID2R: if (tx_free) begin
                    tx_byte <= 8'h00;  tx_dc <= 1'b1;  tx_req <= 1'b1;
                    rx_en   <= 1'b1;
                    rd_cnt  <= rd_cnt + 2'd1;
                    if (rd_cnt == 2'd3) tst <= T_IDSNAP2;
                end
                T_IDSNAP2: begin
                    rx_en <= 1'b0;
                    if (!spi_busy) begin
                        idd3_r <= rx_sh;
                        tst    <= T_FETCH;   // then start the colour wheel
                    end
                end
                default: tst <= T_RST0;
            endcase
        end
    end

endmodule
`default_nettype wire
