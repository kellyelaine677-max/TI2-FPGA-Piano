`timescale 1ns/1ps
// ============================================================================
// key_scan.v - 4x4 matrix keypad scanner with debounce
//
// Scans one column per 250 us (active-low drive, rows need pull-ups - set
// PULL_MODE=UP in the .cst). Each key keeps a 4-sample history (1 ms apart);
// pressed  = four consecutive 1s, released = four consecutive 0s.
// Full matrix scan = 1 ms, so a press is recognised after 3-4 ms - this IS
// the dominant term of the control-to-audio latency budget (topic: <= 5 ms),
// synthesis adds < 0.1 ms on top.
// Outputs level states plus 1-clk on/off edge pulses with the key index.
//
// Key layout: key index = {column, row} = c*4 + r, i.e.:
//   col drive 0 -> keys 0..3 (rows 0..3), col 1 -> keys 4..7, ...
// ============================================================================
`default_nettype none

module key_scan (
    input  wire        clk,        // 27 MHz
    input  wire        rstn,
    output reg  [3:0]  col,        // active-low column drive
    input  wire [3:0]  row,        // rows, external pull-ups, low = pressed
    output reg  [15:0] key_state,
    output reg         key_on,     // 1-clk pulse
    output reg         key_off,    // 1-clk pulse
    output reg  [3:0]  key_idx
);

    // 250 us per column @ 40.5 MHz system clock (40.5e6 / 10125 = 4 kHz)
    localparam [15:0] DIV_END = 16'd10124;

    reg [15:0] div;
    wire slot_end = (div == DIV_END);
    always @(posedge clk or negedge rstn) begin
        if (!rstn) div <= 16'd0;
        else div <= slot_end ? 16'd0 : div + 16'd1;
    end

    reg [1:0] csel;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) csel <= 2'd0;
        else if (slot_end) csel <= csel + 2'd1;
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn) col <= 4'b1110;
        else col <= ~(4'b0001 << csel);
    end

    // 4-sample history per key (memory: 16 keys x 4 bits)
    reg [3:0] shist [0:15];
    wire [3:0] pressed = ~row;
    integer r;
    reg [4:0] hidx;
    reg [3:0] htmp;
    always @(posedge clk) begin
        if (slot_end) begin
            for (r = 0; r < 4; r = r + 1) begin
                hidx = {1'b0, csel, 2'd0} + r;
                htmp = shist[hidx];
                shist[hidx] <= {htmp[2:0], pressed[r]};
            end
        end
    end

    // debounced next state
    reg [15:0] ks_next;
    always @* begin
        ks_next = key_state;
        for (r = 0; r < 16; r = r + 1) begin
            if (shist[r] == 4'b1111) ks_next[r] = 1'b1;
            else if (shist[r] == 4'b0000) ks_next[r] = 1'b0;
        end
    end

    // ---- event queue --------------------------------------------------
    // The old code ran one loop over all 16 keys and drove key_on / key_off
    // / key_idx with non-blocking assignments. Inside a clocked block the
    // LAST non-blocking write wins, so when two keys changed in the same
    // 250 us slot one event overwrote the other and it was gone for good:
    // key_state was updated wholesale right after, so the dropped edge was
    // never seen again. Worse, key_idx was shared by both branches, so a
    // PRESS could be reported with a RELEASE's index - and when that index
    // landed on one of the four control keys (12..15) top_synth's
    // key_is_note gate threw the event away entirely.
    //
    // That is the "note that never stops" bug: note_off never reaches the
    // engine, the voice sits in ENV_DECAY with v_active held, and the only
    // way to free it is to press the same key again. Those leaked voices
    // also fill the 16-slot pool, after which every new note STEALS a
    // still-ringing voice - which is the "tail cut off" symptom.
    //
    // Every transition now goes into a pending mask and is drained one
    // event per slot, release before press. Nothing can be dropped.
    reg [15:0] pend_on, pend_off;

    wire [15:0] new_on  = ks_next  & ~key_state;
    wire [15:0] new_off = ~ks_next &  key_state;
    wire [15:0] set_off = pend_off | new_off;
    wire [15:0] set_on  = pend_on  | new_on;

    // lowest pending index for each direction (scratch qp is private to
    // this block - shared comb scratch is a multi-driver error)
    reg [3:0] p_off, p_on;
    reg       v_off, v_on;
    integer qp;
    always @* begin
        p_off = 4'd0; v_off = 1'b0;
        p_on  = 4'd0; v_on  = 1'b0;
        for (qp = 0; qp < 16; qp = qp + 1) begin
            if (set_off[qp] && !v_off) begin p_off = qp[3:0]; v_off = 1'b1; end
            if (set_on[qp]  && !v_on ) begin p_on  = qp[3:0]; v_on  = 1'b1; end
        end
    end

    // release wins over press: it frees a slot before the next one is taken
    wire        use_off = v_off;
    wire        use_on  = v_on & ~v_off;
    wire [15:0] off_nxt = set_off & ~(use_off ? (16'd1 << p_off) : 16'd0);
    wire [15:0] on_nxt  = set_on  & ~(use_on  ? (16'd1 << p_on ) : 16'd0);

    // state update + edge pulses
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            key_state <= 16'd0;
            key_on <= 1'b0; key_off <= 1'b0; key_idx <= 4'd0;
            pend_on <= 16'd0; pend_off <= 16'd0;
        end else begin
            key_on  <= 1'b0;
            key_off <= 1'b0;
            if (slot_end) begin
                key_state <= ks_next;
                pend_off  <= off_nxt;
                pend_on   <= on_nxt;
                if (use_off) begin
                    key_off <= 1'b1;
                    key_idx <= p_off;
                end else if (use_on) begin
                    key_on  <= 1'b1;
                    key_idx <= p_on;
                end
            end
        end
    end

endmodule
`default_nettype wire
