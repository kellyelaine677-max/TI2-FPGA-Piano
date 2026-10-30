package com.ti2.piano

import android.content.Context
import android.os.Handler
import android.os.Looper
import java.util.concurrent.Executors

/**
 * 全 App 唯一的链路持有者。Fragments 通过它收发，
 * 这样切换 Tab 时连接不会断、录音也能跨页面继续。
 */
object Link {

    @Volatile var transport: Transport? = null
    @Volatile var state: LinkState = LinkState.IDLE
    @Volatile var statusText: String = "未连接"
    @Volatile var devName: String = ""

    /** 12 个槽各自映射到的半音 0..11，默认是恒等映射（与板上复位值一致） */
    val keymap = IntArray(12) { it }

    /** 当前音色 0..7，与 voice_engine 的 TIM_* 一一对应 */
    @Volatile var timbre: Int = 0

    /** 当前八度，App 侧自己算绝对音高（板上的八度键不影响 MIDI 通道） */
    @Volatile var octave: Int = 4          // 4 => C4 = 60
    val baseNote get() = 60 + (octave - 4) * 12

    private val main = Handler(Looper.getMainLooper())
    private val stateListeners = ArrayList<(LinkState, String) -> Unit>()
    private val rxListeners = ArrayList<(ByteArray) -> Unit>()

    fun addStateListener(l: (LinkState, String) -> Unit) {
        synchronized(stateListeners) { stateListeners.add(l) }
        l(state, statusText)
    }

    fun removeStateListener(l: (LinkState, String) -> Unit) {
        synchronized(stateListeners) { stateListeners.remove(l) }
    }

    fun addRxListener(l: (ByteArray) -> Unit) {
        synchronized(rxListeners) { rxListeners.add(l) }
    }

    fun removeRxListener(l: (ByteArray) -> Unit) {
        synchronized(rxListeners) { rxListeners.remove(l) }
    }

    private fun fireState() {
        val snap = ArrayList(stateListeners)
        main.post { snap.forEach { it(state, statusText) } }
    }

    private fun fireRx(b: ByteArray) {
        val snap: List<(ByteArray) -> Unit> = synchronized(rxListeners) { ArrayList(rxListeners) }
        main.post { snap.forEach { it(b) } }
    }

    fun connect(ctx: Context, dev: DevInfo) {
        disconnect()
        val t: Transport = if (dev.kind == Kind.SPP) SppTransport() else BleTransport(ctx)
        transport = t
        devName = dev.name
        t.onState = { s, m ->
            state = s
            statusText = when (s) {
                LinkState.IDLE -> "还没连上琴"
                LinkState.CONNECTING -> "正在连接…"
                LinkState.CONNECTED -> "已连上 $m"
                LinkState.FAILED -> "没连上（$m）"
            }
            if (s == LinkState.FAILED || s == LinkState.IDLE) transport = null
            fireState()
        }
        t.onRx = { onRxBridge(it) }
        state = LinkState.CONNECTING
        statusText = "连接中：${dev.name}"
        fireState()
        t.open(dev)
    }

    fun disconnect() {
        try { transport?.close() } catch (_: Exception) {}
        transport = null
        state = LinkState.IDLE
        statusText = "未连接"
        fireState()
    }

    val connected: Boolean get() = transport != null && state == LinkState.CONNECTED

    /**
     * 发送一律丢到单线程 executor：
     * BLE 的分包写要等 write 回调，最长可能阻塞几秒，绝不能跑在 UI 线程上。
     * 单线程同时保证了 SysEx 分包和后续事件不会交错。
     */
    private val io = Executors.newSingleThreadExecutor { r ->
        Thread(r, "ti2-tx").also { it.isDaemon = true }
    }

    /** 诊断用：累计已发出的字节数。一直涨说明 App 这边确实在发。 */
    @Volatile var txBytes = 0L

    /**
     * 诊断用：累计收到的字节数（琴 → 手机方向）。
     * 一直是 0 = 回传方向不通（模块 RXD 那根线没接 / notify 没使能成功），
     * 录音功能靠的就是这个方向 —— 收不到就永远录不到东西。
     */
    @Volatile var rxBytes = 0L

    fun send(b: ByteArray) {
        if (b.isEmpty()) return
        txBytes += b.size
        // 录音兜底：App 自己发出去的音符也记一份。回传方向通的时候 Rec 优先
        // 用回传（那才包含在琴上弹的音），不通时才退回本地这份。
        if (b.size == 3 && Rec.running) {
            val st = b[0].toInt() and 0xFF
            if (st and 0xF0 == 0x90 || st and 0xF0 == 0x80)
                Rec.feedLocal(st, b[1].toInt() and 0x7F, b[2].toInt() and 0x7F)
        }
        io.execute { try { transport?.write(b) } catch (_: Exception) {} }
    }

    /** Transport 收到数据先过这里计数再分发 */
    internal fun onRxBridge(b: ByteArray) {
        rxBytes += b.size
        fireRx(b)
    }

    /** 连上后把键位表整体推一遍，弥补板上没有 EEPROM 的问题 */
    fun pushKeymap() {
        for (i in 0 until 12) send(Proto.keymapCc(i, keymap[i]))
    }

    /**
     * 连上后把手机里的设置整体推一遍（键位 + 音色）。
     * 板上没有 EEPROM，音色掉电也回到 0，所以每次连上都要重发。
     */
    fun pushSettings() {
        pushKeymap()
        send(Proto.timbreCc(timbre))
    }
}
