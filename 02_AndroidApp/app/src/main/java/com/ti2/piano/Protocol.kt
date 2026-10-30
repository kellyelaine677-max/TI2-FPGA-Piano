package com.ti2.piano

import org.json.JSONArray
import org.json.JSONObject

/**
 * 与 FPGA 侧 `midi_rx.v` / `song_player.v` 完全对应的协议层。
 *
 * 链路：串口 9600 8N1，上面跑标准 MIDI 字节流。
 *   note on   : 90 nn 40        （力度板子不用，统一给 0x40）
 *   note off  : 80 nn 00
 *   键位映射  : B0 (14+i) v     i=0..11 槽号，v=0..11 半音偏移
 *   播放方式  : B0 20 v         v=1 引导 / v=0 直接播放
 *   音色      : B0 21 v         v=0..7，对应 voice_engine 的 TIM_*
 *   播放开关  : B0 22 v         v=1 播 / v=0 停止并复位
 *   曲谱下载  : F0 7D 01 <N> <b0>..<b(N-1)> F7
 *
 * 音符字节 7 位：[6:4] 音级 0..6 = C D E F G A B，**7 = 休止符**；
 *              [3:0] 时值，**单位是半拍**（1 = 八分音符，2 = 四分音符）。
 * 注意：硬件编码没有八度、没有升降号（#4 一律就近取自然音级），
 * 时值最小半拍 —— 十六分音符会被并到八分。
 */
object Proto {

    const val VEL = 0x40

    /** 板上一个时值单位（半拍）多少毫秒。BEAT = 7_087_500 / 40.5 MHz = 175 ms */
    const val HALF_BEAT_MS = 175L

    fun noteOn(note: Int): ByteArray =
        byteArrayOf(0x90.toByte(), (note and 0x7F).toByte(), VEL.toByte())

    fun noteOff(note: Int): ByteArray =
        byteArrayOf(0x80.toByte(), (note and 0x7F).toByte(), 0x00)

    /** 槽 slot(0..11) 映射到半音偏移 semi(0..11)，对应 CC 20..31 */
    fun keymapCc(slot: Int, semi: Int): ByteArray =
        byteArrayOf(0xB0.toByte(), (0x14 + slot).toByte(), (semi and 0x0F).toByte())

    /**
     * CC 32：曲谱播放方式。guide=true 只点亮 LED 等用户弹，false 板子自己弹。
     * 必须在 SysEx 之前发，因为板子收到 F7 就立刻开始播。
     */
    fun playModeCc(guide: Boolean): ByteArray =
        byteArrayOf(0xB0.toByte(), 0x20.toByte(), (if (guide) 1 else 0).toByte())

    /** CC 33：音色 0..7。和琴上的编码器旋钮改的是同一个寄存器。 */
    fun timbreCc(t: Int): ByteArray =
        byteArrayOf(0xB0.toByte(), 0x21.toByte(), (t and 0x07).toByte())

    /** 与 voice_engine.v 的 TIM_SINE..TIM_PIANO 顺序严格一一对应，**别调顺序** */
    val timbreNames = arrayOf("正弦", "方波", "锯齿", "三角", "FM", "加法", "八谐波", "钢琴")

    /** CC 34：播放开关。false 停止并从头复位，true 开始播放。 */
    fun playCtlCc(on: Boolean): ByteArray =
        byteArrayOf(0xB0.toByte(), 0x22.toByte(), (if (on) 1 else 0).toByte())

    /** 一条 SysEx 曲谱。notes 为空时返回空数组（不下发）。 */
    fun scoreSysex(notes: List<Score.Note>): ByteArray {
        if (notes.isEmpty()) return ByteArray(0)
        val n = notes.size.coerceAtMost(64)
        val out = ByteArray(4 + n + 1)
        out[0] = 0xF0.toByte()
        out[1] = 0x7D.toByte()
        out[2] = 0x01.toByte()
        out[3] = n.toByte()
        for (i in 0 until n) {
            val d = notes[i].deg and 0x07          // 7 = 休止符，正好用 3 位装下
            val b = notes[i].beats.coerceIn(1, 15) and 0x0F
            out[4 + i] = ((d shl 4) or b).toByte()
        }
        out[4 + n] = 0xF7.toByte()
        return out
    }

    /**
     * 按倍率缩放时值：mul=1 时值减半（快一倍），2 = 不变，3 = 加半（慢一倍）。
     * 用来在 App 侧调曲速，不用重新烧板子。
     * 时值最小 1 个半拍单位，缩放后向下取整并夹紧到 1..15。
     */
    fun scale(notes: List<Score.Note>, mul: Int): List<Score.Note> {
        if (mul == 2) return notes
        return notes.map { n ->
            val b = if (mul == 1) (n.beats + 1) / 2
                    else (n.beats * 3 + 1) / 2
            Score.Note(n.deg, b.coerceIn(1, 15))
        }
    }

    /** 一首曲子播完要多少毫秒（用于自动发"停止"，不让板子无限循环） */
    fun durationMs(notes: List<Score.Note>): Long =
        notes.take(64).sumOf { it.beats.coerceIn(1, 15) } * HALF_BEAT_MS

    fun noteName(semi: Int): String {
        val names = arrayOf("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")
        return names[semi % 12]
    }

    /** 音级 0..7 的名字，用于曲谱显示 */
    fun degreeName(deg: Int): String =
        if (deg == 7) "休" else arrayOf("C", "D", "E", "F", "G", "A", "B")[deg % 7]
}

/**
 * 曲谱模型。受 7 位编码限制：**没有八度、没有升降号**，音级 7 = 休止符。
 * [Note.beats] 的单位是**半拍**（1 = 八分音符，2 = 四分音符，4 = 二分音符）。
 */
data class Score(val name: String, val notes: List<Note>) {
    data class Note(val deg: Int, val beats: Int)

    /** 给人看的预览：音名序列，长音后面拖一横，每 8 个音一行 */
    fun describe(): String = buildString {
        append("共 ").append(notes.size).append(" 个音，约 ")
            .append("%.1f".format(Proto.durationMs(notes) / 1000.0)).append(" 秒\n")
        notes.forEachIndexed { i, n ->
            append(Proto.degreeName(n.deg))
            when {
                n.beats >= 4 -> append("——")
                n.beats >= 2 -> append("—")
            }
            append(if ((i + 1) % 8 == 0) "\n" else " ")
        }
    }.trimEnd()

    /** 转成 "d:b" 紧凑串，便于手改和存文件（b 为半拍数） */
    fun toCompact(): String = notes.joinToString(",") { "${it.deg}:${it.beats}" }

    companion object {
        fun parse(name: String, json: String): Score {
            val obj = JSONObject(json)
            val arr: JSONArray = when {
                obj.has("notes") -> obj.getJSONArray("notes")
                obj.has("data") -> obj.getJSONArray("data")
                else -> JSONArray()
            }
            val notes = ArrayList<Note>(arr.length())
            for (i in 0 until arr.length()) {
                val e = arr.get(i)
                when (e) {
                    is JSONObject -> {
                        val d = when {
                            e.has("d") -> e.getInt("d")
                            e.has("deg") -> e.getInt("deg")
                            e.has("degree") -> e.getInt("degree")
                            else -> 0
                        }
                        // 时值兼容两种写法：b/beats 直接给半拍数（1=八分），
                        // beat 给"拍"（可以带小数，1 = 四分音符），内部统一 ×2
                        val b: Int = when {
                            e.has("b") -> e.optDouble("b", 1.0).toInt()
                            e.has("beats") -> e.optDouble("beats", 1.0).toInt()
                            e.has("beat") -> (e.optDouble("beat", 1.0) * 2).toInt()
                            else -> 2
                        }
                        notes.add(Note(d.coerceIn(0, 7), b.coerceIn(1, 15)))
                    }
                    is String -> {
                        // 紧凑串 "0:1,4:1"（d:b，b 为半拍数）
                        val p = e.split(":")
                        if (p.size == 2) notes.add(
                            Note(p[0].trim().toInt().coerceIn(0, 7),
                                p[1].trim().toInt().coerceIn(1, 15))
                        )
                    }
                }
            }
            val nm = if (obj.has("name")) obj.getString("name") else name
            return Score(nm, notes)
        }

        /** 从 "0:1,4:1,5:2" 这种手工串解析（d:b，b 为半拍数） */
        fun fromCompact(name: String, s: String): Score {
            val notes = s.split(",", "，", " ", "\n")
                .map { it.trim() }.filter { it.isNotEmpty() }
                .mapNotNull {
                    val p = it.split(":")
                    if (p.size != 2) null
                    else try {
                        Note(p[0].toInt().coerceIn(0, 7), p[1].toInt().coerceIn(1, 15))
                    } catch (_: Exception) { null }
                }
            return Score(name, notes)
        }
    }
}

/**
 * 内置曲库。
 *
 *  时值单位 = **半拍**：1 = 八分音符，2 = 四分音符，4 = 二分音符；音级 7 = 休止。
 *  硬件编码没有八度和升降号：原谱的高音点/低音点全部折进同一个八度，
 *    #4 之类的变化音就近取自然音级 —— 这是 7 位编码的硬限制。
 * 晴天/退后/花海都是按用户提供的简谱图转录的**主旋律单声部**，
 * 取最有辨识度的一段（板上最多 64 个音符，放不下整首）。
 */
object SongBook {
    private fun s(vararg pairs: Pair<Int, Int>) = pairs.map { Score.Note(it.first, it.second) }
    private const val R = 7   // 休止符

    /** 与 FPGA song_player.v 内置旋律逐音一致（半拍粒度） */
    val twinkle = Score("小星星（板内同款）", s(
        0 to 2, 0 to 2, 4 to 2, 4 to 2, 5 to 2, 5 to 2, 4 to 4,
        3 to 2, 3 to 2, 2 to 2, 2 to 2, 1 to 2, 1 to 2, 0 to 4,
        4 to 2, 4 to 2, 3 to 2, 3 to 2, 2 to 2, 2 to 2, 1 to 4,
        4 to 2, 4 to 2, 3 to 2, 3 to 2, 2 to 2, 2 to 2, 1 to 4,
        0 to 2, 0 to 2, 4 to 2, 4 to 2, 5 to 2, 5 to 2, 4 to 4,
        3 to 2, 3 to 2, 2 to 2, 2 to 2, 1 to 2, 1 to 2, 0 to 4
    ))

    // -----------------------------------------------------------------------
    // 简谱数字 -> 音级的换算：本工程是固定 do = C，简谱是首调，按 1=C 记谱时
    //   n -> n-1     （1=C=0, 2=D=1, 3=E=2, 5=G=4, 6=A=5, 7=B=6）
    // 原谱里的高音点/低音点一律折进同一个八度（编码没有八度位）。
    //
    // 时值要照原曲的速度给，不能一律塞最短值（1 = 175 ms）：
    //   慢歌（晴天 ♩=70、退后 66）八分音符约 0.35~0.45 s -> 用 2
    //   中速（小星星）四分音符约 0.35 s -> 用 2，句尾 4
    //   快歌（花海 BPM 170）八分音符约 0.18 s -> 用 1
    // 时值若全部取 1，整首速度会达到 5.7 个音/秒，节奏过于急促。
    // -----------------------------------------------------------------------

    // 晴天 —— 来源：网络简谱（1=C 记谱）
    //   主歌「故事的小黄花」5 5 6 5 3 2 / 「从出生那年就飘着」6 1 2 3 2 1
    //   副歌「刮风这天我试过握着你手」5 6 1 2 3 5 3
    //        「但偏偏雨渐渐大到我看你不见」2 3 5 6 5 3 2
    //  该曲不使用休止符（d=7）：播放器时间轴为均匀分布，插入休止符
    //    会产生无预期的停顿。需要留白时改为延长前一个音的时值。
    // -----------------------------------------------------------------------
    val qingtian = Score("晴天", s(
        4 to 2, 4 to 2, 5 to 2, 4 to 2, 2 to 2, 1 to 4,          // 故事的小黄花
        5 to 2, 0 to 2, 1 to 2, 2 to 2, 1 to 2, 0 to 4,          // 从出生那年就飘着
        4 to 2, 4 to 2, 5 to 2, 6 to 4,                          // 童年的荡秋千
        1 to 2, 6 to 2, 4 to 6,                                  // 随记忆一直晃到现在
        4 to 2, 5 to 2, 0 to 2, 1 to 2, 2 to 2, 4 to 2, 2 to 4,  // 刮风这天我试过握着你手
        1 to 2, 2 to 2, 4 to 2, 5 to 2, 4 to 2, 2 to 2, 1 to 4,  // 但偏偏雨渐渐大到我看你不见
        4 to 2, 4 to 2, 5 to 2, 6 to 8                           // 收句
    ))

    // -----------------------------------------------------------------------
    // 花海 —— 来源：网络简谱（1=C 记谱），副歌段
    //   12355 不要你离开 / 32123 距离隔不开
    //   123+1+1 思念变成海 / +176565 在窗外进不来
    //   12355 原谅说太快 / 32121 爱成了阻碍
    //   12355443+132 手中的风筝放太快回不来
    //   （+1 = 高音 do，折回后就是音级 0）
    // -----------------------------------------------------------------------
    val huahai = Score("花海", s(
        0 to 1, 1 to 1, 2 to 1, 4 to 1, 4 to 2,                  // 不要你离开
        2 to 1, 1 to 1, 0 to 1, 1 to 1, 2 to 2,                  // 距离隔不开
        0 to 1, 1 to 1, 2 to 1, 0 to 1, 0 to 2,                  // 思念变成海
        0 to 1, 6 to 1, 5 to 1, 4 to 1, 5 to 1, 4 to 2,          // 在窗外进不来
        0 to 1, 1 to 1, 2 to 1, 4 to 1, 4 to 2,                  // 原谅说太快
        2 to 1, 1 to 1, 0 to 1, 1 to 1, 0 to 2,                  // 爱成了阻碍
        0 to 1, 1 to 1, 2 to 1, 4 to 1, 4 to 1, 3 to 1, 3 to 1, 2 to 1, 0 to 1, 2 to 1, 1 to 2  // 手中的风筝放太快回不来
    ))

    // -----------------------------------------------------------------------
    // 退后 ——  该曲仅有和弦谱（C Am / Dm G）可供参考，
    // 此处按和弦骨架编配，非逐音转录，与原曲旋律可能存在出入。
    // -----------------------------------------------------------------------
    val tuihou = Score("退后（按和弦编）", s(
        2 to 2, 4 to 2, 5 to 2, 4 to 2, 2 to 2, 1 to 4,           // 我知道你我都没有错  (C)
        1 to 2, 2 to 2, 1 to 2, 0 to 2, 5 to 4,                   // 只是忘了怎么退后    (Am)
        3 to 2, 5 to 2, 4 to 2, 3 to 2, 2 to 4,                   // 信誓旦旦给了承诺    (Dm)
        1 to 2, 2 to 2, 4 to 2, 3 to 2, 1 to 4,                   // 却被时间扑了空      (G)
        2 to 2, 4 to 2, 5 to 2, 4 to 2, 2 to 2, 1 to 4,           // 我知道我们都没有错  (C)
        0 to 2, 1 to 2, 2 to 2, 4 to 2, 2 to 4,                   // 只是放手会比较好过  (Am/D)
        3 to 2, 2 to 2, 1 to 2, 0 to 2, 0 to 8                    // 最美的爱情回忆里待续 (F G C)
    ))

    val joy = Score("欢乐颂", s(
        2 to 2, 2 to 2, 3 to 2, 4 to 2,
        4 to 2, 3 to 2, 2 to 2, 1 to 2,
        0 to 2, 0 to 2, 1 to 2, 2 to 2,
        2 to 4, 1 to 2, 1 to 4
    ))

    val tiger = Score("两只老虎", s(
        0 to 2, 1 to 2, 2 to 2, 0 to 2,
        0 to 2, 1 to 2, 2 to 2, 0 to 2,
        2 to 2, 3 to 2, 4 to 4,
        2 to 2, 3 to 2, 4 to 4
    ))

    val boat = Score("划船歌", s(
        0 to 2, 0 to 2, 0 to 2, 1 to 2, 2 to 2,
        2 to 2, 1 to 2, 2 to 2, 3 to 2, 4 to 4
    ))

    val scale = Score("音阶往返", s(
        0 to 2, 1 to 2, 2 to 2, 3 to 2, 4 to 2, 5 to 2, 6 to 2,
        6 to 2, 5 to 2, 4 to 2, 3 to 2, 2 to 2, 1 to 2, 0 to 4
    ))

    val all: List<Score> = listOf(qingtian, tuihou, huahai, twinkle, joy, tiger, boat, scale)
}

/**
 * 极简 MIDI 输入解析（处理 running status）。
 * 琴 → 手机 方向只有 90/80 两种事件，但写通用一点更省心。
 */
class MidiStreamParser {
    private var status = 0
    private var need = 0
    private val buf = IntArray(2)
    private var got = 0

    /** 返回解析出的事件，或 null。事件 = (status, d1, d2) */
    fun push(b: Int): Triple<Int, Int, Int>? {
        val v = b and 0xFF
        if (v >= 0x80) {
            if (v >= 0xF0) {            // 系统消息：本题不涉及，直接复位
                status = 0; need = 0; got = 0
                return null
            }
            status = v
            need = when (v and 0xF0) {
                0xC0, 0xD0 -> 1
                else -> 2
            }
            got = 0
            return null
        }
        if (status == 0) return null
        buf[got] = v                      // buf 是 IntArray，别 toByte()
        got += 1
        if (got >= need) {
            got = 0
            val d2 = if (need == 2) buf[1] and 0x7F else 0
            return Triple(status, buf[0] and 0x7F, d2)
        }
        return null
    }
}
