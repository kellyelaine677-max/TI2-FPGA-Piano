`timescale 1ns/1ps
// ============================================================================
// sensor_hub.v - MPR121 capacitive touch + PCF8591 ADC (pressure/velocity)
//
// Owns the I2C bus exclusively (single master). After power-up:
//   1. wait 100 ms (sensor supply settle)
//   2. probe the four possible MPR121 addresses (0x5A..0x5D) and use the one
//      that answers - the ADDR pin is board-dependent and a wrong address
//      means a permanently silent keyboard
//   3. MPR121 soft reset (0x80=0x63), wait 10 ms
//   4. MPR121 init: ECR 0x5E=0x00 (stop), touch/release thresholds for
//      ELE0..11 (0x08/0x04), filter config 0x5C=0x10, 0x5D=0x20,
//      debounce 0x5B=0x00, then ECR 0x5E=0x8C (run, 12 electrodes)
//   5. wait 50 ms (baseline settling)
//   6. forever, every ~2 ms:
//        - poll MPR121 status regs 0x00/0x01 -> touch12
//        - poll PCF8591 ADC ch0 (2-byte read, 2nd byte valid) -> press8
//
// ECR IS AT 0x5E. It was written to 0x72 once: 0x6C..0x72 are the per-
// electrode charge-time registers (CDTx), so that write only set ELE11's
// charge time and never started the chip. The MPR121 comes out of reset in
// Stop Mode and measures nothing until ECR is written, so the electrode-
// enable write is the one register that decides whether touch works at all.
//
// ECR value 0x8C, not 0x0C: the low nibble (12) enables ELE0..11 and the
// 0x80 sets CL = 2b10, which loads the baseline from the first measurement
// instead of ramping it up from zero. With CL = 00 the 3rd-level filter's
// long time constant leaves the part deaf for tens of seconds after power-up
// - it looks exactly like "touch does not work" while slowly recovering.
//
// I2C addresses: MPR121 = 0x5A..0x5D (probed), PCF8591 = 0x48.
//
// Outputs are registered and glitch-free; touch_stb pulses on any change.
// press8: 0..255, roughly proportional to pressure (FSR divider on ch0).
// ============================================================================
`default_nettype none

module sensor_hub (
    input  wire        clk,
    input  wire        rstn,
    inout  wire        sda,
    inout  wire        scl,
    output reg  [11:0] touch12,     // electrode 0..11 touched flags
    output reg         touch_stb,   // 1-clk pulse when touch12 changes
    output reg  [7:0]  press8,      // PCF8591 ch0, 0..255
    // Did a pressure sensor ever answer? The board ships without one, and
    // press8 then reads 0 - which is "no pressure", not "no sensor". Feeding
    // that into the touch velocity clamped every touched note to the floor,
    // so a touch came out about seven times quieter than a keypad press.
    // This bit is what tells the two apart.
    output reg         press_ok,
    output reg         hub_ready,   // high after init sequence completed
    // Bus diagnostics. "No touch" and "no chip" look identical from the
    // outside, and nothing on the LCD or the audio path can tell them apart,
    // so the hub reports which address answered and whether one answered at
    // all. These drive LEDs - see the assign at the end of top_synth.v.
    output reg  [1:0]  hub_addr,    // probed address: 0=0x5A .. 3=0x5D
    output reg         hub_found,   // an MPR121 answered the address probe
    // Raw electrode 0 filtered data (10 bit), read every poll round.
    // Kept on purpose: when touch "does not work" the question is always
    // "is the electrode data moving at all, or is it moving but staying
    // under the threshold?" - those have completely different fixes, and
    // only the raw number can tell them apart.
    output reg  [9:0]  elec0,
    // baseline - filtered for electrode 0. This is the number the part
    // actually compares against the touch threshold, so it is the one to
    // look at when the raw data moves but nothing ever triggers: if it
    // stays near zero the baseline is chasing the finger, if it is large
    // and still no touch then the threshold or the data path is wrong.
    output reg  [7:0]  elec_diff,
    // Baseline byte as read, and the sign of the difference. elec_diff is an
    // unsigned 8-bit subtraction, so a NEGATIVE difference (filtered data
    // sitting ABOVE the baseline - the direction the part reads as "released")
    // wraps to something like 250 and looks exactly like a huge touch. One
    // bit says which way round it is; without it "big number but no touch"
    // is unreadable.
    output reg  [7:0]  bl0,
    output reg         elec_neg,
    // Out-of-range flags (0x02/0x03). This is the missing third case: a
    // large baseline-minus-filtered number with NO touch flag is almost
    // always OOR, because the part suppresses touch detection on any
    // electrode whose filtered data sits outside [LSL, USL]. Without this
    // bit "big number + no touch" is unexplainable.
    output reg  [11:0] oor12
);

    // ------------------------------------------------------------------
    // I2C engine instance
    // ------------------------------------------------------------------
    reg        cmd_start, cmd_stop, cmd_wr, cmd_rd;
    reg        ack_n;
    reg  [7:0] wr_data;
    wire [7:0] rd_data;
    wire       ack_err, busy, done;

    // QUART x 4 = 404 core clocks per I2C bit @ 40.5 MHz = 9.98 us
    // -> SCL = 100.2 kHz, inside the MPR121 / PCF8591 400 kHz limit.
    // QUART sets the SCL rate: one I2C bit is 4 quarters, so
    // SCL = clk / (4 * QUART). This hub runs on the 40.5 MHz system clock,
    // and QUART = 101 landed at 100.25 kHz - 0.25 % OVER the PCF8591's
    // 100 kHz maximum. The MPR121 allows 400 kHz, which is why touch worked
    // and the ADC silently never answered: every one of its polls ran out of
    // spec. 118 -> 85.8 kHz, inside spec for both parts with margin.
    // (The comment inside i2c_bit.v still says "@ 27 MHz, QUART = 68" - that
    // was written for the pre-PLL 27 MHz system clock and is now stale.)
    i2c_bit #(.QUART(118)) u_i2c (
        .clk (clk), .rstn (rstn),
        .cmd_start (cmd_start), .cmd_stop (cmd_stop),
        .cmd_wr (cmd_wr), .cmd_rd (cmd_rd),
        .ack_n (ack_n), .wr_data (wr_data),
        .rd_data (rd_data), .ack_err (ack_err),
        .busy (busy), .done (done),
        .sda (sda), .scl (scl)
    );

    // ------------------------------------------------------------------
    // millisecond tick (40500 cycles @ 40.5 MHz)
    // ------------------------------------------------------------------
    reg [15:0] ms_cnt;
    wire ms_tick = (ms_cnt == 16'd40499);
    always @(posedge clk or negedge rstn) begin
        if (!rstn) ms_cnt <= 16'd0;
        else ms_cnt <= ms_tick ? 16'd0 : ms_cnt + 16'd1;
    end

    reg [16:0] ms_wait;   // elapsed ms in delay states

    // ------------------------------------------------------------------
    // MPR121 init table:
    //   1 stop + 11 baseline filter + 4 auto-config + 24 threshold
    //   + 3 config + 1 run  = 44 writes
    // ------------------------------------------------------------------
    localparam N_INIT = 44;
    reg [7:0] init_r [0:N_INIT-1];
    reg [7:0] init_v [0:N_INIT-1];
    integer ii;
    initial begin
        // Registers may only be changed while the part is in Stop Mode. A
        // soft reset already leaves it there, but be explicit: replaying
        // this table onto a running part would otherwise fail silently.
        init_r[0]  = 8'h5E; init_v[0]  = 8'h00;   // ECR: stop, 0 electrodes

        // Baseline filter controls. Left at the power-on defaults these are
        // the one group nobody sets, and they decide how fast the baseline
        // chases the electrode - too fast and it swallows the touch.
        init_r[1]  = 8'h2B; init_v[1]  = 8'h01;   // MHD_R
        init_r[2]  = 8'h2C; init_v[2]  = 8'h01;   // NHD_R
        init_r[3]  = 8'h2D; init_v[3]  = 8'h00;   // NCL_R  (was 0x0E: rose 14
                                                  // counts/sample, at ESI=1 ms
                                                  // the baseline ran away)
        init_r[4]  = 8'h2E; init_v[4]  = 8'h00;   // FDL_R
        init_r[5]  = 8'h2F; init_v[5]  = 8'h01;   // MHD_F
        init_r[6]  = 8'h30; init_v[6]  = 8'h01;   // NHD_F
        init_r[7]  = 8'h31; init_v[7]  = 8'hFF;   // NCL_F: hold the baseline
        init_r[8]  = 8'h32; init_v[8]  = 8'h02;   // FDL_F
        init_r[9]  = 8'h33; init_v[9]  = 8'h00;   // NHD_T
        init_r[10] = 8'h34; init_v[10] = 8'h00;   // NCL_T
        init_r[11] = 8'h35; init_v[11] = 8'h00;   // FDL_T

        // Auto-configuration (AN3889). The global CDC/CDT written below are
        // a guess at the electrode capacitance; if a pad's capacitance is
        // outside what that guess can measure, the electrode data rails at
        // an extreme and no threshold will ever fire. Auto-config runs once
        // when the part enters Run Mode and picks a charge current and time
        // per electrode from the actual measurement.
        init_r[12] = 8'h7B; init_v[12] = 8'h0B;   // ACE=1, AREN=1
        // Limits are in the >>2 domain (0..255), and the electrode data is
        // range-checked against them on EVERY measurement, not just during
        // auto-configuration: a pad whose data leaves [LSL, USL] is flagged
        // out-of-range and its touch detection is suppressed. That is the
        // trap - a finger pushes the data DOWN by 100..300 raw counts, so a
        // window with too little room under the target turns every real
        // touch into an OOR instead of a touch.
        //   USL = (VDD-0.7)/VDD * 256          = 0xC9 at VDD = 3.3 V
        //   TL  = 0.9 * USL is the usual advice, but a reachable target
        //        matters more than a high one - 0x8C is what this board's
        //        auto-configuration actually lands on.
        //   LSL is set well below the 0.65*USL guideline on purpose: the
        //        guideline sizes the window for the untouched level, and
        //        leaves only ~39 counts under the target - a finger blows
        //        straight through it.
        init_r[13] = 8'h7D; init_v[13] = 8'hC9;   // USL  (3.3 V)
        init_r[14] = 8'h7E; init_v[14] = 8'h50;   // LSL  (touch headroom)
        init_r[15] = 8'h7F; init_v[15] = 8'h8C;   // target level

        // Thresholds. 2/1 was a diagnostic setting - it fires on roughly
        // 8 raw counts, which is inside the noise floor, and the board then
        // reported electrodes "touched" with nothing near the pads. 10/5 is
        // 40/20 raw counts: ~5x the observed noise, far below a real touch
        // (a finger moves the electrode data by 100+ counts).
        for (ii = 0; ii < 12; ii = ii + 1) begin
            init_r[16+2*ii] = 8'h41 + 2*ii; init_v[16+2*ii] = 8'h0A;  // touch thr
            init_r[17+2*ii] = 8'h42 + 2*ii; init_v[17+2*ii] = 8'h05;  // release thr
        end
        init_r[40] = 8'h5C; init_v[40] = 8'h10;   // CONFIG1: FFI=00, CDC=16uA
        init_r[41] = 8'h5D; init_v[41] = 8'h24;   // CONFIG2: CDT=1us, ESI=16ms
                                                  // (power-on default; ESI=1ms
                                                  // made every baseline filter
                                                  // constant act 16x faster)
        init_r[42] = 8'h5B; init_v[42] = 8'h00;   // debounce (power-on default)
        init_r[43] = 8'h5E; init_v[43] = 8'h8C;   // ECR: RUN, ELE0..11, CL=2b10
    end

    // ------------------------------------------------------------------
    // hub sequencer
    // ------------------------------------------------------------------
    // 7 bits, not 6: adding the out-of-range poll took the last state to
    // 6'd64, which does not fit in 6 bits. It did not fail to compile and it
    // did not warn - 6'd64 silently truncated to 6'd0, which is S_PWR, so
    // every poll round ended by soft-resetting the part and replaying the
    // whole 100 ms power-up sequence. The chip was in Stop Mode most of the
    // time and could never hold a touch. A width bug that looks like a
    // dead sensor is the worst kind, so tb_sensor now counts soft resets.
    localparam [6:0]
        S_PWR     = 7'd0,                            // 100 ms power settle
        S_MR0     = 7'd1,  S_MR1 = 7'd2,  S_MR2 = 7'd3,   // START, WR addr, WR 0x80
        S_MR3     = 7'd4,  S_MR4 = 7'd5,  S_MR5 = 7'd6,   // WR 0x63, STOP, 10ms
        S_II0     = 7'd7,  S_II1 = 7'd8,  S_II2 = 7'd9,   // START, WR addr, WR reg
        S_II3     = 7'd10, S_II4 = 7'd11,                  // WR val, STOP
        S_SETTLE  = 7'd12,                           // 50 ms baseline settle
        S_TMR     = 7'd13,                           // 2 ms poll period
        S_MP0     = 7'd14, S_MP1 = 7'd15, S_MP2 = 7'd16, S_MP3 = 7'd17,
        S_MP4     = 7'd18, S_MP5 = 7'd19, S_MP6 = 7'd20, S_MP7 = 7'd21,
        S_MP8     = 7'd22, S_MP9 = 7'd23,
        S_PC0     = 7'd24, S_PC1 = 7'd25, S_PC2 = 7'd26, S_PC3 = 7'd27,
        S_PC4     = 7'd28, S_PC5 = 7'd29, S_PC6 = 7'd30, S_PC7 = 7'd31,
        S_PC8     = 7'd32, S_PC9 = 7'd33,
        S_AD0     = 7'd34, S_AD1 = 7'd35, S_AD2 = 7'd36, S_AD3 = 7'd37,
        // Generic multi-byte register read. One transaction per electrode,
        // with the register pointer rewritten every time, so this does not
        // rely on the part auto-incrementing - a 24-byte burst read would be
        // three times faster but would silently return the same electrode
        // twelve times on a part that does not increment.
        S_MB0     = 7'd38, S_MB1 = 7'd39, S_MB2 = 7'd40, S_MB3 = 7'd41,
        S_MB4     = 7'd42, S_MB5 = 7'd43, S_MB6 = 7'd44, S_MB7 = 7'd45,
        S_MB8     = 7'd46,
        S_ED0     = 7'd47,                           // arm the next read
        S_FIN     = 7'd48,                           // commit this round
        S_OO0     = 7'd56, S_OO1 = 7'd57, S_OO2 = 7'd58, S_OO3 = 7'd59,
        S_OO4     = 7'd60, S_OO5 = 7'd61, S_OO6 = 7'd62, S_OO7 = 7'd63,
        S_OO8     = 7'd64;

    reg [6:0]  st;
    // 6 bits, not 5: N_INIT is 44 and a 5-bit index tops out at 31, which
    // silently wraps and replays the table forever. mpr121_check.py now
    // asserts the declared width can actually hold N_INIT.
    reg [5:0]  init_idx;
    reg [7:0]  t_b0;
    reg [1:0]  addr_idx; // which of 0x5A..0x5D is being probed
    // ---- per-electrode copies the fallback decision runs on ----
    reg [9:0]  elec  [0:11];   // filtered data, all twelve
    reg [7:0]  blv   [0:11];   // the part's own baseline, >>2
    reg [11:0] touch_fb;       // touch decided here, not by the part
    reg [11:0] elec_neg_q;     // per electrode: filtered data above baseline
    reg [7:0]  mbuf  [0:1];    // bytes of the current read
    reg [5:0]  bcnt;           // byte index inside the current read
    reg [5:0]  mb_n;           // bytes to read (1 or 2)
    reg [7:0]  mb_reg;         // register pointer for the current read
    reg        mb_sel;         // 0 = electrode data, 1 = baseline
    reg [3:0]  ei;             // electrode index 0..11
    reg [11:0] mpr_touch;      // what the part itself reported
    reg [7:0]  mpr_wr;   // MPR121 write address in use (probed at start-up)
    reg [7:0]  mpr_rd;   // MPR121 read address (mpr_wr + 1)
    // PCF8591 write address in use. A0/A1/A2 select one of eight addresses
    // (0x48..0x4F) and plenty of boards ship with those pins uncommitted -
    // no jumpers fitted and no pull resistors - so the pins float and the
    // part can answer anywhere in that range, or nowhere. Probing the way
    // the MPR121 already does means the ADC is found whatever the board
    // does with A0..A2; the read address is always pcf_wr + 1.
    reg [7:0]  pcf_wr;
    reg        pcf_lock;  // 1 = an address answered, stop rotating
    reg        err_q;    // any NACK seen during the current poll cycle
    reg        run;      // handshake: set when a command is issued, cleared
                         // one cycle after engine done - prevents the hub
                         // from firing the next command on the same cycle
                         // the engine is still latching the previous one

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            st <= S_PWR; ms_wait <= 17'd0; init_idx <= 6'd0; run <= 1'b0;
            err_q <= 1'b0; addr_idx <= 2'd0;
            mpr_wr <= 8'hB4; mpr_rd <= 8'hB5;   // 0x5A until a probe succeeds
            pcf_wr <= 8'h90; pcf_lock <= 1'b0;  // 0x48 until a probe succeeds
            cmd_start <= 1'b0; cmd_stop <= 1'b0; cmd_wr <= 1'b0; cmd_rd <= 1'b0;
            ack_n <= 1'b0; wr_data <= 8'd0; t_b0 <= 8'd0;
            touch12 <= 12'd0; touch_stb <= 1'b0; press8 <= 8'd0;
            press_ok <= 1'b0;
            hub_ready <= 1'b0; hub_found <= 1'b0; hub_addr <= 2'd0;
            elec0 <= 10'd0; elec_diff <= 8'd0; oor12 <= 12'd0;
            bl0 <= 8'd0; elec_neg <= 1'b0;
            touch_fb <= 12'd0; mpr_touch <= 12'd0; ei <= 4'd0;
            elec_neg_q <= 12'd0;
            bcnt <= 6'd0; mb_n <= 6'd2; mb_reg <= 8'h04; mb_sel <= 1'b0;
            for (ii = 0; ii < 12; ii = ii + 1) begin
                elec[ii] <= 10'd0;
                blv[ii]  <= 8'd0;
            end
            mbuf[0] <= 8'd0; mbuf[1] <= 8'd0;
        end else begin
            cmd_start <= 1'b0;
            cmd_stop  <= 1'b0;
            cmd_wr    <= 1'b0;
            cmd_rd    <= 1'b0;
            touch_stb <= 1'b0;

            if (done) begin
                run <= 1'b0;
                if (ack_err) err_q <= 1'b1;   // device did not answer
            end

            case (st)
            // ---------- power-up ----------
            S_PWR: if (ms_tick) begin
                ms_wait <= ms_wait + 17'd1;
                if (ms_wait == 17'd99) begin ms_wait <= 17'd0; st <= S_AD0; end
            end
            // ---------- MPR121 address probe ----------
            // Which of 0x5A..0x5D a board answers on depends on how its ADDR
            // pin is tied - SparkFun/Adafruit ship 0x5A, Grove V3 ships 0x5B,
            // and a bare module can be strapped to any of the four. On the
            // wrong address every transfer NACKs, err_q stays set and touch12
            // is frozen at 0: from the outside that is indistinguishable from
            // a dead sensor. Probe once at start-up instead of assuming.
            S_AD0: if (!busy && !run) begin
                cmd_start <= 1'b1; run <= 1'b1; st <= S_AD1; err_q <= 1'b0;
            end
            S_AD1: if (!busy && !run) begin
                cmd_wr   <= 1'b1;
                wr_data  <= 8'hB4 + {5'd0, addr_idx, 1'b0};
                run      <= 1'b1;
                st       <= S_AD2;
            end
            S_AD2: if (!busy && !run) begin cmd_stop <= 1'b1; run <= 1'b1; st <= S_AD3; end
            S_AD3: begin
                if (!err_q) begin                       // it answered
                    mpr_wr <= 8'hB4 + {5'd0, addr_idx, 1'b0};
                    mpr_rd <= 8'hB4 + {5'd0, addr_idx, 1'b0} + 8'd1;
                    hub_found <= 1'b1;
                    hub_addr  <= addr_idx;
                    st     <= S_MR0;
                end else if (addr_idx == 2'd3) begin
                    // nothing on the bus: keep 0x5A and run the sequence
                    // anyway, so the polls report NACK and touch12 stays 0
                    mpr_wr <= 8'hB4; mpr_rd <= 8'hB5;
                    hub_found <= 1'b0;
                    st <= S_MR0;
                end else begin
                    addr_idx <= addr_idx + 2'd1; st <= S_AD0;
                end
            end
            // ---------- MPR121 soft reset ----------
            S_MR0: if (!busy && !run) begin cmd_start <= 1'b1; run <= 1'b1; st <= S_MR1; end
            S_MR1: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= mpr_wr; run <= 1'b1; st <= S_MR2; end
            S_MR2: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= 8'h80; run <= 1'b1; st <= S_MR3; end
            S_MR3: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= 8'h63; run <= 1'b1; st <= S_MR4; end
            S_MR4: if (!busy && !run) begin cmd_stop <= 1'b1; run <= 1'b1; st <= S_MR5; ms_wait <= 17'd0; end
            S_MR5: if (ms_tick) begin
                ms_wait <= ms_wait + 17'd1;
                if (ms_wait == 17'd9) begin ms_wait <= 17'd0; init_idx <= 6'd0; st <= S_II0; end
            end
            // ---------- MPR121 register init loop ----------
            S_II0: if (!busy && !run) begin cmd_start <= 1'b1; run <= 1'b1; st <= S_II1; end
            S_II1: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= mpr_wr; run <= 1'b1; st <= S_II2; end
            S_II2: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= init_r[init_idx]; run <= 1'b1; st <= S_II3; end
            S_II3: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= init_v[init_idx]; run <= 1'b1; st <= S_II4; end
            S_II4: if (!busy && !run) begin
                cmd_stop <= 1'b1;
                run <= 1'b1;
                if (init_idx == N_INIT-1) begin st <= S_SETTLE; ms_wait <= 17'd0; end
                else begin init_idx <= init_idx + 6'd1; st <= S_II0; end
            end
            // ---------- baseline settle ----------
            S_SETTLE: if (ms_tick) begin
                ms_wait <= ms_wait + 17'd1;
                if (ms_wait == 17'd49) begin ms_wait <= 17'd0; st <= S_TMR; hub_ready <= 1'b1; end
            end
            // ---------- poll timer ----------
            // 2 ms period: touch confirmed within 2 ms (poll) + ~1 ms (two
            // I2C transfers @ 100 kHz). A 10 ms period here used to be the
            // latency-budget killer - topic stretch goal is <= 5 ms.
            S_TMR: if (ms_tick) begin
                ms_wait <= ms_wait + 17'd1;
                if (ms_wait == 17'd1) begin ms_wait <= 17'd0; st <= S_MP0; end
            end
            // ---------- MPR121 status poll ----------
            // START, WR 0xB4, WR 0x00, START, WR 0xB5, RD(ack), RD(nack), STOP
            S_MP0: if (!busy && !run) begin
                cmd_start <= 1'b1; run <= 1'b1; st <= S_MP1;
                err_q <= 1'b0;                 // start a clean poll
            end
            S_MP1: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= mpr_wr; run <= 1'b1; st <= S_MP2; end
            S_MP2: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= 8'h00; run <= 1'b1; st <= S_MP3; end
            S_MP3: if (!busy && !run) begin cmd_start <= 1'b1; run <= 1'b1; st <= S_MP4; end
            S_MP4: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= mpr_rd; run <= 1'b1; st <= S_MP5; end
            S_MP5: if (!busy && !run) begin cmd_rd <= 1'b1; ack_n <= 1'b0; run <= 1'b1; st <= S_MP6; end
            S_MP6: if (!busy && !run) begin
                // rd_data still holds byte 0 (status reg 0x00) - latch it
                t_b0    <= rd_data;
                cmd_rd  <= 1'b1;
                ack_n   <= 1'b1;
                run     <= 1'b1;
                st      <= S_MP7;
            end
            S_MP7: if (!busy && !run) begin
                // rd_data holds byte 1 (status reg 0x01, low nibble = ELE8..11)
                // IMPORTANT: commit only if every transfer in this poll was
                // acknowledged. With the MPR121 absent the bus idles high and
                // every read byte is 0xFF - that used to latch touch12 = 12'hFFF
                // (all 12 electrodes "touched") -> 12 phantom notes held
                // forever -> a loud, permanent background cluster.
                // Kept as "what the part says", not as the final answer.
                // Reading all twelve electrodes below lets this hub decide
                // for itself, and the two are ORed at the end of the round.
                if (!err_q) mpr_touch <= {rd_data[3:0], t_b0};
                st <= S_MP8;
            end
            S_MP8: if (!busy && !run) begin cmd_stop <= 1'b1; run <= 1'b1; st <= S_MP9; end
            S_MP9: begin ei <= 4'd0; mb_sel <= 1'b0; st <= S_ED0; end
            // ---------- all twelve electrodes: data then baseline ----------
            // Electrode n filtered data = 0x04 + 2n (two bytes, low first,
            // the second byte's bits 1:0 are the two MSBs).
            // Electrode n baseline      = 0x1E + n (one byte, baseline[9:2]).
            // The part's own status register is not trusted on its own: a
            // board was measured at baseline 137 / filtered 26 while touched
            // - an 111-count difference against a threshold of 10 - and it
            // still reported no touch, with no out-of-range flag either. The
            // raw numbers are unambiguous, so the comparison is done here.
            S_ED0: begin
                mb_reg <= mb_sel ? (8'h1E + {4'd0, ei})
                                 : (8'h04 + {3'd0, ei, 1'b0});
                mb_n   <= mb_sel ? 6'd1 : 6'd2;
                st     <= S_MB0;
            end
            S_MB0: if (!busy && !run) begin
                cmd_start <= 1'b1; run <= 1'b1; st <= S_MB1; err_q <= 1'b0;
            end
            S_MB1: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= mpr_wr; run <= 1'b1; st <= S_MB2; end
            S_MB2: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= mb_reg;  run <= 1'b1; st <= S_MB3; end
            S_MB3: if (!busy && !run) begin cmd_start <= 1'b1; run <= 1'b1; st <= S_MB4; end
            S_MB4: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= mpr_rd;  run <= 1'b1; st <= S_MB5; end
            S_MB5: if (!busy && !run) begin
                bcnt <= 6'd0; cmd_rd <= 1'b1; ack_n <= (mb_n == 6'd1);
                run <= 1'b1; st <= S_MB6;
            end
            S_MB6: if (!busy && !run) begin
                if (!err_q) mbuf[bcnt[0]] <= rd_data;
                if (bcnt + 6'd1 == mb_n) st <= S_MB7;
                else begin
                    bcnt   <= bcnt + 6'd1;
                    ack_n  <= (bcnt + 6'd2 == mb_n);   // NACK the last byte
                    cmd_rd <= 1'b1; run <= 1'b1;
                end
            end
            S_MB7: if (!busy && !run) begin cmd_stop <= 1'b1; run <= 1'b1; st <= S_MB8; end
            S_MB8: begin
                if (mb_sel == 1'b0) begin
                    elec[ei] <= {mbuf[1][1:0], mbuf[0]};
                    if (ei == 4'd11) begin ei <= 4'd0; mb_sel <= 1'b1; st <= S_ED0; end
                    else begin ei <= ei + 4'd1; st <= S_ED0; end
                end else begin
                    blv[ei] <= mbuf[0];
                    if (ei == 4'd11) st <= S_FIN;
                    else begin ei <= ei + 4'd1; st <= S_ED0; end
                end
            end
            // ---------- decide, locally, what a touch is ----------
            // Same comparison the part documents - baseline minus filtered,
            // in the >>2 domain the baseline register lives in - with real
            // hysteresis. Measured: 000 untouched, 111 while a finger is on
            // the pad, so 25 / 12 sits in the middle of a very wide gap.
            S_FIN: begin
                elec0     <= elec[0];
                bl0       <= blv[0];
                elec_diff <= blv[0] - elec[0][9:2];
                elec_neg  <= (elec[0][9:2] > blv[0]);
                // elec_neg_q is the per-electrode sign: an unsigned 8-bit
                // subtraction turns "filtered above baseline" - the direction
                // the part reads as released - into 250-ish, which would sail
                // straight over the threshold. Guard on it explicitly.
                for (ii = 0; ii < 12; ii = ii + 1) begin
                    elec_neg_q[ii] <= (elec[ii][9:2] > blv[ii]);
                    if ((elec[ii][9:2] <= blv[ii]) &&
                        (blv[ii] - elec[ii][9:2]) >= 8'd25)
                        touch_fb[ii] <= 1'b1;
                    else if ((elec[ii][9:2] > blv[ii]) ||
                             ((blv[ii] - elec[ii][9:2]) <= 8'd12))
                        touch_fb[ii] <= 1'b0;
                end
                if ((mpr_touch | touch_fb) != touch12) begin
                    touch12   <= mpr_touch | touch_fb;
                    touch_stb <= 1'b1;
                end
                st <= S_OO0;
            end
            // ---------- MPR121 out-of-range status (0x02/0x03) ----------
            // Two bytes: 0x02 = ELE0..7, 0x03 = ELE8..11 (low nibble).
            // Same "commit only if the whole poll was ACKed" guard as the
            // touch status: a missing part makes every read byte 0xFF.
            S_OO0: if (!busy && !run) begin
                cmd_start <= 1'b1; run <= 1'b1; st <= S_OO1; err_q <= 1'b0;
            end
            S_OO1: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= mpr_wr; run <= 1'b1; st <= S_OO2; end
            S_OO2: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= 8'h02; run <= 1'b1; st <= S_OO3; end
            S_OO3: if (!busy && !run) begin cmd_start <= 1'b1; run <= 1'b1; st <= S_OO4; end
            S_OO4: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= mpr_rd; run <= 1'b1; st <= S_OO5; end
            S_OO5: if (!busy && !run) begin cmd_rd <= 1'b1; ack_n <= 1'b0; run <= 1'b1; st <= S_OO6; end
            S_OO6: if (!busy && !run) begin
                t_b0    <= rd_data;             // 0x02: ELE0..7
                cmd_rd  <= 1'b1;
                ack_n   <= 1'b1;
                run     <= 1'b1;
                st      <= S_OO7;
            end
            S_OO7: if (!busy && !run) begin
                if (!err_q) oor12 <= {rd_data[3:0], t_b0};
                st <= S_OO8;
            end
            S_OO8: if (!busy && !run) begin cmd_stop <= 1'b1; run <= 1'b1; st <= S_PC0; end
            // ---------- PCF8591 ADC poll ----------
            // START, WR 0x90, WR 0x00(ch0), START, WR 0x91, RD(ack), RD(nack), STOP
            S_PC0: if (!busy && !run) begin
                cmd_start <= 1'b1; run <= 1'b1; st <= S_PC1;
                err_q <= 1'b0;                 // start a clean poll
            end
            S_PC1: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= pcf_wr; run <= 1'b1; st <= S_PC2; end
            S_PC2: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= 8'h00; run <= 1'b1; st <= S_PC3; end
            S_PC3: if (!busy && !run) begin cmd_start <= 1'b1; run <= 1'b1; st <= S_PC4; end
            S_PC4: if (!busy && !run) begin cmd_wr <= 1'b1; wr_data <= pcf_wr + 8'd1; run <= 1'b1; st <= S_PC5; end
            S_PC5: if (!busy && !run) begin cmd_rd <= 1'b1; ack_n <= 1'b0; run <= 1'b1; st <= S_PC6; end
            S_PC6: if (!busy && !run) begin cmd_rd <= 1'b1; ack_n <= 1'b1; run <= 1'b1; st <= S_PC7; end
            S_PC7: if (!busy && !run) begin
                // rd_data holds the 2nd (valid) conversion byte
                if (!err_q) begin
                    press8    <= rd_data;   // ignore NACKed (absent) ADC
                    press_ok  <= 1'b1;      // it answered: there is a sensor
                    pcf_lock  <= 1'b1;      // stop rotating: this is the one
                end else if (!pcf_lock) begin
                    // nobody at this address - step to the next one in
                    // 0x48..0x4F. The write address advances by 2 because
                    // the LSB of an I2C address byte is the R/W bit.
                    // A full sweep is 16 ms; once one answers it is locked.
                    pcf_wr <= (pcf_wr == 8'h9E) ? 8'h90 : (pcf_wr + 8'd2);
                end
                st <= S_PC8;
            end
            S_PC8: if (!busy && !run) begin cmd_stop <= 1'b1; run <= 1'b1; st <= S_PC9; end
            S_PC9: st <= S_TMR;
            default: st <= S_PWR;
            endcase
        end
    end

endmodule
`default_nettype wire
