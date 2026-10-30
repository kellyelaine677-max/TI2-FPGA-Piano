package com.ti2.piano

import kotlin.math.max

/**
 * 演奏录音 → 标准 MIDI 文件。
 *
 * 板子把每个音符事件编码为 3 字节 MIDI 流式送出（`bt_tx.v`），
 * 没有时间戳。这里用「手机收到的时刻」补时间戳：UART 是实时的，
 * 3 字节 @9600 只有 3.1 ms 延迟，相对一首曲子完全可以忽略。
 *
 * 存成标准 .mid 后既能在 App 里回放（按 delta 把事件发回琴，琴自己重演），
 * 也能导出到电脑用任何 DAW 打开 —— 评委拿到的是一份能验证的文件。
 */
object MidiFile {

    private const val TPQ = 96            // ticks per quarter note
    private const val TEMPO_US = 500_000  // 120 BPM -> 一拍 500 ms
    private val MS_PER_TICK = TEMPO_US / 1000.0 / TPQ   // 5.208 ms

    data class Ev(val tick: Long, val status: Int, val d1: Int, val d2: Int) {
        val ms: Long get() = (tick * MS_PER_TICK).toLong()
    }

    /** 可变长数量（VLQ），MIDI 标准里所有 delta time 都用它 */
    private fun vlq(v: Long, out: MutableList<Byte>) {
        var value = max(0L, v)
        val buf = ByteArray(4)
        var n = 0
        buf[n] = (value and 0x7FL).toByte()   // 注意：value 是 Long，掩码必须写成 0x7FL
        n += 1
        value = value shr 7
        while (value > 0) {
            buf[n] = ((value and 0x7FL) or 0x80L).toByte()
            n += 1
            value = value shr 7
        }
        for (i in n - 1 downTo 0) out.add(buf[i])
    }

    private fun be32(v: Int, out: MutableList<Byte>) {
        out.add(((v shr 24) and 0xFF).toByte())
        out.add(((v shr 16) and 0xFF).toByte())
        out.add(((v shr 8) and 0xFF).toByte())
        out.add((v and 0xFF).toByte())
    }

    private fun be16(v: Int, out: MutableList<Byte>) {
        out.add(((v shr 8) and 0xFF).toByte())
        out.add((v and 0xFF).toByte())
    }

    private fun be24(v: Int, out: MutableList<Byte>) {
        out.add(((v shr 16) and 0xFF).toByte())
        out.add(((v shr 8) and 0xFF).toByte())
        out.add((v and 0xFF).toByte())
    }

    fun msToTick(ms: Long): Long = (ms / MS_PER_TICK).toLong()

    /** events 必须已按 tick 升序 */
    fun build(events: List<Ev>): ByteArray {
        val body = ArrayList<Byte>(256)

        // tempo meta: FF 51 03 tttttt  (500000 us/quarter = 120 BPM)
        vlq(0, body)
        body.add(0xFF.toByte()); body.add(0x51.toByte()); body.add(0x03.toByte())
        be24(TEMPO_US, body)

        var prev = 0L
        for (e in events) {
            vlq(e.tick - prev, body)
            prev = e.tick
            body.add((e.status and 0xFF).toByte())
            body.add((e.d1 and 0x7F).toByte())
            body.add((e.d2 and 0x7F).toByte())
        }

        // end of track
        vlq(0, body)
        body.add(0xFF.toByte()); body.add(0x2F.toByte()); body.add(0x00.toByte())

        val out = ArrayList<Byte>(body.size + 22)
        out.addAll("MThd".toByteArray(Charsets.US_ASCII).toList())
        be32(6, out)
        be16(0, out)      // format 0
        be16(1, out)      // 1 track
        be16(TPQ, out)    // division
        out.addAll("MTrk".toByteArray(Charsets.US_ASCII).toList())
        be32(body.size, out)
        out.addAll(body)
        return out.toByteArray()
    }
}

/**
 * 录音机。线程安全：BLE 回调在 binder 线程，SPP 在读线程。
 */
object Rec {
    private val parser = MidiStreamParser()
    private val raw = ArrayList<Pair<Long, IntArray>>()   // (ms, [status,d1,d2])  回传来的
    private val local = ArrayList<Pair<Long, IntArray>>()  // 手机自己弹的（回传断了时的兜底）
    private var t0 = 0L
    @Volatile var running = false

    /** 本次录音里"回传"和"本地"各录到几个音，用于给用户明确的提示 */
    @Volatile var fromBoard = 0
    @Volatile var fromPhone = 0

    @Synchronized fun reset() { raw.clear(); local.clear(); fromBoard = 0; fromPhone = 0 }

    @Synchronized fun start() {
        raw.clear(); local.clear()
        fromBoard = 0; fromPhone = 0
        t0 = System.currentTimeMillis()
        running = true
    }

    @Synchronized fun stop() { running = false }

    @Synchronized fun feed(bytes: ByteArray) {
        if (!running) return
        val now = System.currentTimeMillis() - t0
        for (b in bytes) {
            val ev = parser.push(b.toInt()) ?: continue
            val st = ev.first
            if (st and 0xF0 != 0x90 && st and 0xF0 != 0x80) continue   // 只留音符
            var status = st
            val d2 = ev.third
            // 力度 0 的 note-on 等价于 note-off（板子只发 90/80，这里按标准兜一下）
            if (st and 0xF0 == 0x90 && d2 == 0) status = 0x80 or (st and 0x0F)
            raw.add(now to intArrayOf(status, ev.second, d2))
            fromBoard = raw.size
        }
    }

    /**
     * 手机自己弹的音。
     *
     * 录音本来只录"琴回传"的事件，可一旦回传方向不通（模块 RXD→T8 没接、
     * notify 没使能）就一个音都录不到，用户看到的就是"录音功能没用"。
     * 所以 App 自己发出去的音符也记一份：回传有数据时优先用回传（它才包含
     * 在琴上弹的音），回传没数据时用本地这份兜底，至少"手机弹"能录下来。
     */
    @Synchronized fun feedLocal(status: Int, d1: Int, d2: Int) {
        if (!running) return
        val now = System.currentTimeMillis() - t0
        local.add(now to intArrayOf(status, d1, d2))
        fromPhone = local.size
    }

    @Synchronized fun events(): List<MidiFile.Ev> {
        val src = if (raw.isNotEmpty()) raw else local
        val ticks = src.map { (ms, e) ->
            MidiFile.Ev(MidiFile.msToTick(ms), e[0], e[1], e[2])
        }
        // 保险：保证单调递增
        var prev = -1L
        return ticks.map {
            val t = max(prev + 1, it.tick)
            prev = t
            MidiFile.Ev(t, it.status, it.d1, it.d2)
        }
    }

    @Synchronized fun count(): Int = if (raw.isNotEmpty()) raw.size else local.size

    /** 这次录音的数据来自哪里：给 UI 显示，别让用户以为录到了琴上的音 */
    @Synchronized fun source(): String = when {
        raw.isNotEmpty() -> "琴回传"
        local.isNotEmpty() -> "手机本地（回传不通）"
        else -> "无"
    }

    /** 回放：按记录的真实间隔把事件发回琴，琴重新合成一遍 */
    fun playback(sink: (ByteArray) -> Unit, onDone: () -> Unit) {
        val list = events()
        if (list.isEmpty()) { onDone(); return }
        Thread {
            var prev = 0L
            for (e in list) {
                val wait = max(0L, e.ms - prev)
                prev = e.ms
                if (wait > 0) try { Thread.sleep(wait) } catch (_: InterruptedException) { break }
                sink(byteArrayOf(e.status.toByte(), e.d1.toByte(), e.d2.toByte()))
            }
            onDone()
        }.also { it.isDaemon = true; it.start() }
    }
}
