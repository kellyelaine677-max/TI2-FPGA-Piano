`timescale 1ns/1ps
// ============================================================================
// goertzel_bank.v - 12-band spectrum analyzer on the mixed audio output
//
// One shared multiplier serves 12 Goertzel bins, time-multiplexed inside the
// 384-cycle sample period. Block length N = 2048 samples (~29 ms refresh).
// Bin frequencies: musical series 110..5280 Hz at fs = 70312.5 Hz.
//
// Recursion (plain-value states, k in Q16):
//     s0 = x + ((k_q * s1) >>> 16) - s2
// Block-end power (18-bit magnitude windows v1 = s1>>16, v2 = s2>>16):
//     power ~ v1*v1 + v2*v2 - ((k_q*v1)>>>16)*v2
// Bar display: log2(power) MSB position -> 0..255 scaled.
// ============================================================================
`default_nettype none

module goertzel_bank (
    input  wire               clk,
    input  wire               rstn,
    input  wire               sample_valid,
    input  wire signed [15:0] sample_in,
    output reg  [95:0]        mag_bar,  // 12 x 8-bit bars, FLAT: bin i = mag_bar[i*8 +: 8]
                                        // (unpacked array ports are not synthesizable in Gowin)
    output reg                frame_stb
);

    // Goertzel coefficients round(2*cos(2*pi*f/fs) * 2^16), fs = 70312.5 Hz
    localparam signed [17:0] K00 = 18'sd131066;  // 110 Hz
    localparam signed [17:0] K01 = 18'sd131058;  // 165 Hz
    localparam signed [17:0] K02 = 18'sd131047;  // 220 Hz
    localparam signed [17:0] K03 = 18'sd131015;  // 330 Hz
    localparam signed [17:0] K04 = 18'sd130971;  // 440 Hz
    localparam signed [17:0] K05 = 18'sd130844;  // 660 Hz
    localparam signed [17:0] K06 = 18'sd130667;  // 880 Hz
    localparam signed [17:0] K07 = 18'sd130161;  // 1320 Hz
    localparam signed [17:0] K08 = 18'sd129454;  // 1760 Hz
    localparam signed [17:0] K09 = 18'sd127441;  // 2640 Hz
    localparam signed [17:0] K10 = 18'sd124641;  // 3520 Hz
    localparam signed [17:0] K11 = 18'sd116751;  // 5280 Hz

    // 功率计算用的 cos(w)/sin(w)（各 Q16）。
    //
    // 功率按恒等变形
    //     |X|^2 = (s1 - cos(w)*s2)^2 + (sin(w)*s2)^2
    // 计算。若直接用 v1^2 + v2^2 - (k*v1>>16)*v2（v1=s1>>16，v2=s2>>16），
    // 则属于"大数相减得小数"：s1、s2 为 34 位（幅度可达 2^28），而 110 Hz
    // 处的 |X|^2 仅占低 10 位左右，三个 24 位数相减抵消掉 13 位后，剩余的
    // 主要是右移 16 位引入的量化噪声，低频柱高会出现随机跳变。
    //
    // 恒等变形的两项均为平方项、恒非负，直接相加无抵消。
    // s1、s2 仍保持 34 位全精度，减法只缩小幅度而不损失有效位，
    // 因此取 18 位送入乘法器（>>16）时误差约 3%，
    // 且 power 的绝对尺度与直接计算式一致（均为 |X|^2 / 2^32），
    // 故 log2 -> 柱高的映射常数无需改动。
    localparam signed [17:0] C00 = 18'sd65533;  // cos, 110 Hz
    localparam signed [17:0] C01 = 18'sd65529;  // cos, 165 Hz
    localparam signed [17:0] C02 = 18'sd65523;  // cos, 220 Hz
    localparam signed [17:0] C03 = 18'sd65508;  // cos, 330 Hz
    localparam signed [17:0] C04 = 18'sd65485;  // cos, 440 Hz
    localparam signed [17:0] C05 = 18'sd65422;  // cos, 660 Hz
    localparam signed [17:0] C06 = 18'sd65333;  // cos, 880 Hz
    localparam signed [17:0] C07 = 18'sd65081;  // cos, 1320 Hz
    localparam signed [17:0] C08 = 18'sd64727;  // cos, 1760 Hz
    localparam signed [17:0] C09 = 18'sd63721;  // cos, 2640 Hz
    localparam signed [17:0] C10 = 18'sd62321;  // cos, 3520 Hz
    localparam signed [17:0] C11 = 18'sd58376;  // cos, 5280 Hz

    localparam signed [17:0] S00 = 18'sd644;    // sin, 110 Hz
    localparam signed [17:0] S01 = 18'sd966;    // sin, 165 Hz
    localparam signed [17:0] S02 = 18'sd1288;   // sin, 220 Hz
    localparam signed [17:0] S03 = 18'sd1932;   // sin, 330 Hz
    localparam signed [17:0] S04 = 18'sd2576;   // sin, 440 Hz
    localparam signed [17:0] S05 = 18'sd3863;   // sin, 660 Hz
    localparam signed [17:0] S06 = 18'sd5148;   // sin, 880 Hz
    localparam signed [17:0] S07 = 18'sd7712;   // sin, 1320 Hz
    localparam signed [17:0] S08 = 18'sd10265;  // sin, 1760 Hz
    localparam signed [17:0] S09 = 18'sd15318;  // sin, 2640 Hz
    localparam signed [17:0] S10 = 18'sd20276;  // sin, 3520 Hz
    localparam signed [17:0] S11 = 18'sd29787;  // sin, 5280 Hz

    function signed [17:0] ksel(input [3:0] b);
        case (b)
            4'd0:  ksel = K00;  4'd1:  ksel = K01;  4'd2:  ksel = K02;
            4'd3:  ksel = K03;  4'd4:  ksel = K04;  4'd5:  ksel = K05;
            4'd6:  ksel = K06;  4'd7:  ksel = K07;  4'd8:  ksel = K08;
            4'd9:  ksel = K09;  4'd10: ksel = K10;  default: ksel = K11;
        endcase
    endfunction

    function signed [17:0] kcsel(input [3:0] b);
        case (b)
            4'd0:  kcsel = C00;  4'd1:  kcsel = C01;  4'd2:  kcsel = C02;
            4'd3:  kcsel = C03;  4'd4:  kcsel = C04;  4'd5:  kcsel = C05;
            4'd6:  kcsel = C06;  4'd7:  kcsel = C07;  4'd8:  kcsel = C08;
            4'd9:  kcsel = C09;  4'd10: kcsel = C10;  default: kcsel = C11;
        endcase
    endfunction

    function signed [17:0] kssel(input [3:0] b);
        case (b)
            4'd0:  kssel = S00;  4'd1:  kssel = S01;  4'd2:  kssel = S02;
            4'd3:  kssel = S03;  4'd4:  kssel = S04;  4'd5:  kssel = S05;
            4'd6:  kssel = S06;  4'd7:  kssel = S07;  4'd8:  kssel = S08;
            4'd9:  kssel = S09;  4'd10: kssel = S10;  default: kssel = S11;
        endcase
    endfunction

    // per-bin Goertzel states (plain-value, |s| bounded by ~N*x < 2^30)
    reg signed [33:0] s1 [0:11];
    reg signed [33:0] s2 [0:11];
    reg signed [15:0] x_q;

    reg [10:0] blk;
    reg        last_sample;

    // shared multiplier: 18 x 34 -> 52-bit
    reg  signed [17:0] m_a;
    reg  signed [33:0] m_b;
    wire signed [51:0] prod = m_a * m_b;

    // ------------------------------------------------------------------
    // 4*log2(p) + 2 位小数（返回 8 位）
    //
    // 不采用仅取最高位位置的方式：那样只有"整数个 2 的幂"一种粒度，一根
    // 柱子只能取 20 来个离散高度，容易跳变或整体低于门槛。取最高位之后的
    // 两位作为小数，分辨率提高到 4 倍，柱高连续。
    //
    // 返回值 lg2 = 4*floor(log2 p) + frac2，frac2 = MSB 之后的两位。
    // 例：p = 10 = 0b1010 -> MSB 在 bit3，后面两位 01 -> lg2 = 13。
    // ------------------------------------------------------------------
    function [7:0] log2_q(input [47:0] p);
        integer b;
        reg found;
        begin
            log2_q = 8'd0;
            found  = 1'b0;
            for (b = 47; b >= 2; b = b - 1) begin
                if (!found && p[b]) begin
                    log2_q = {b[5:0], p[b-1], p[b-2]};
                    found  = 1'b1;
                end
            end
            // p 只有最低两位时循环覆盖不到，单独特判
            if (!found) begin
                if (p[1]) log2_q = {6'd1, p[0], 1'b0};
                else      log2_q = 8'd0;      // p == 0 或 1
            end
        end
    endfunction

    // FSM states
    localparam [3:0]
        P_IDLE = 4'd0,
        P_L0 = 4'd1, P_L1 = 4'd2, P_L2 = 4'd3, P_L3 = 4'd4,
        P_E0 = 4'd5, P_E1 = 4'd6, P_E2 = 4'd7, P_E3 = 4'd8,
        P_E4 = 4'd9, P_E5 = 4'd10, P_E6 = 4'd11, P_E7 = 4'd12;

    reg [3:0]  pst;
    reg [3:0]  bin;
    // 无抵消功率路径的中间量（34 位全精度）
    reg signed [33:0] d_q;      // s1 - cos(w)*s2
    reg signed [33:0] e_q;      // sin(w)*s2
    reg signed [33:0] pC;       // cos(w)*s2
    reg signed [47:0] pA;
    reg signed [47:0] power;
    reg [7:0]  lg2;       // 4*log2(power) + 2 位小数
    reg [7:0]  bar_new;   // 本帧算出的柱子高度（峰值保持之前）

    // 柱子每帧最多下降多少。一帧 2048 点 = 29 ms，掉到底约 0.5 s。
    localparam [7:0] BAR_FALL = 8'd12;

    integer i0;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            pst <= P_IDLE; blk <= 11'd0; bin <= 4'd0;
            x_q <= 16'd0;
            m_a <= 18'd0; m_b <= 34'd0;
            d_q <= 34'd0; e_q <= 34'd0; pC <= 34'd0;
            pA <= 48'd0; power <= 48'd0;
            lg2 <= 8'd0;
            frame_stb <= 1'b0;
            for (i0 = 0; i0 < 12; i0 = i0 + 1) begin
                s1[i0] <= 34'd0;
                s2[i0] <= 34'd0;
                mag_bar[i0*8 +: 8] <= 8'd0;
            end
        end else begin
            frame_stb <= 1'b0;

            case (pst)
                // ---- latch new sample, run recursion over 12 bins ----
                P_IDLE: if (sample_valid) begin
                    x_q         <= sample_in;
                    bin         <= 4'd0;
                    last_sample <= (blk == 11'd2047);
                    pst         <= P_L0;
                end
                // s0 = x + ((k*s1)>>>16) - s2
                P_L0: begin
                    m_a <= ksel(bin);
                    m_b <= s1[bin];
                    pst <= P_L1;
                end
                P_L1: begin                     // 等待 prod 稳定
                    pst  <= P_L2;
                end
                P_L2: begin
                    // (prod >>> 16) truncated to 34 bits = prod[49:16]:
                    // |k*s1| < 2^47 so bits [51:50] are pure sign copies and
                    // mod-2^34 arithmetic equals the old 52-bit-then-truncate
                    s2[bin] <= s1[bin];
                    s1[bin] <= {{18{x_q[15]}}, x_q} + $signed(prod[49:16]) - s2[bin];
                    pst <= P_L3;
                end
                P_L3: begin
                    if (bin == 4'd11) begin
                        if (last_sample) begin
                            bin <= 4'd0;
                            pst <= P_E0;      // block complete -> magnitudes
                        end else begin
                            blk <= blk + 11'd1;
                            pst <= P_IDLE;
                        end
                    end else begin
                        bin <= bin + 4'd1;
                        pst <= P_L0;
                    end
                end

                // ---- block end: |X|^2 = (s1 - cos(w)*s2)^2 + (sin(w)*s2)^2 ----
                // 无抵消形式：两个平方项恒非负，直接相加。
                // s1/s2 全程保持 34 位，只有最后进乘法器时才取高 18 位。
                P_E0: begin
                    m_a <= kcsel(bin);                        // cos(w) Q16
                    m_b <= s2[bin];
                    pst <= P_E1;
                end
                P_E1: begin
                    pC    <= prod[49:16];                     // cos(w)*s2
                    m_a   <= kssel(bin);                      // sin(w) Q16
                    m_b   <= s2[bin];
                    pst   <= P_E2;
                end
                P_E2: begin
                    e_q  <= prod[49:16];                      // sin(w)*s2
                    d_q  <= s1[bin] - pC;                     // s1 - cos(w)*s2
                    pst  <= P_E3;
                end
                P_E3: begin
                    m_a <= d_q[33:16];                        // (d>>16) * (d>>16)
                    m_b <= {{16{d_q[33]}}, d_q[33:16]};
                    pst <= P_E4;
                end
                P_E4: begin
                    pA  <= prod[47:0];                        // (d>>16)^2
                    m_a <= e_q[33:16];                        // (e>>16) * (e>>16)
                    m_b <= {{16{e_q[33]}}, e_q[33:16]};
                    pst <= P_E5;
                end
                P_E5: begin
                    power <= pA + prod[47:0];                 // 相加，无相减
                    pst   <= P_E6;
                end
                P_E6: begin
                    lg2 <= log2_q(power[47] ? 48'd0 : power);
                    pst <= P_E7;
                end
                P_E7: begin
                    // ------------------------------------------------------
                    // power -> 柱高映射。
                    //
                    // 若门槛设在 power>=64、之后每级 12，显示窗口只能覆盖较高
                    // 电平。实测 12 音 ADD8 齐奏时 12 个
                    // 频点的 power 只有 9~83：
                    //     110:40  165:22  220:69  330:83  440:70  660:68
                    //     880:45 1320:20 1760:14 2640:9  3520:19 5280:43
                    // 其中近一半低于 64 会被抹成 0，频谱上可见的峰数不足。
                    //
                    // 根子在于混音本身离满刻度很远：ADD8 的谐波系数 coef(1)
                    // 只给 1/4 满量程，再乘 255/sqrt(N) 的复音归一化，12 音
                    // 齐奏混音峰值才 3484/32767 ≈ -19 dBFS，单频点只有 -38
                    // dBFS 上下。显示窗口必须覆盖到那里。
                    //
                    // 新映射：power>=2 起步，每 1/4 个 log2 单位 6 个计数，
                    // 约 32 dB 幅度动态范围铺满 255 级。
                    //   lg2<=4  (power<2,  单频点幅度 < 90)     -> 0
                    //   lg2>=47 (power>3444, 单频点幅度 > 3756) -> 255
                    // 16 位量化噪声本身只到 power≈0.04，远在门槛之下。
                    // ------------------------------------------------------
                    if (lg2 <= 8'd4)        bar_new = 8'd0;
                    else if (lg2 >= 8'd47)  bar_new = 8'd255;
                    else                    bar_new = (lg2 - 8'd4) * 8'd6;

                    // ------------------------------------------------------
                    // 峰值保持 + 缓降
                    //
                    // 频点间隔只有 1.6 个 bin（110 与 165 相差 55 Hz，而
                    // 2048 点的分辨率是 34.3 Hz）——比任何窗函数的主瓣都窄，
                    // 矩形窗的泄漏必然让相邻频点互相"打架"。实测同一个音在
                    // 连续 3 帧里读出 96 / 84 / 60，柱子会明显抖。
                    // 上升沿立即跟上、下降沿每帧最多掉 BAR_FALL，屏幕上就是
                    // 常见的"柱子慢慢落下"的观感，顺带把瞬时峰保留住。
                    // ------------------------------------------------------
                    if (bar_new >= mag_bar[bin*8 +: 8])
                        mag_bar[bin*8 +: 8] <= bar_new;
                    else if (mag_bar[bin*8 +: 8] > BAR_FALL)
                        mag_bar[bin*8 +: 8] <= mag_bar[bin*8 +: 8] - BAR_FALL;
                    else
                        mag_bar[bin*8 +: 8] <= 8'd0;
                    // reset bin for next block
                    s1[bin] <= 34'd0;
                    s2[bin] <= 34'd0;
                    if (bin == 4'd11) begin
                        blk <= 11'd0;
                        pst <= P_IDLE;
                        frame_stb <= 1'b1;
                    end else begin
                        bin <= bin + 4'd1;
                        pst <= P_E0;
                    end
                end
                default: pst <= P_IDLE;
            endcase
        end
    end

endmodule
`default_nettype wire
