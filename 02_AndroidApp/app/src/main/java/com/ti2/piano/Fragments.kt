package com.ti2.piano

import android.content.res.ColorStateList
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.AdapterView
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.EditText
import android.widget.Spinner
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.fragment.app.Fragment
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import java.net.HttpURLConnection
import java.net.URL

// ---------------------------------------------------------------------------
// 1. 演奏：一个八度的 12 键，支持多指和弦
// ---------------------------------------------------------------------------
class KeysFragment : Fragment(R.layout.frag_keys) {

    /** 音色按钮，onViewCreated 里填充 */
    private val timBtns = arrayOfNulls<Button>(8)

    override fun onViewCreated(v: View, s: Bundle?) {
        val piano = v.findViewById<PianoView>(R.id.piano)
        val tvOct = v.findViewById<TextView>(R.id.tvOct)
        val tvBase = v.findViewById<TextView>(R.id.tvBase)

        fun refresh() {
            tvOct.text = Link.octave.toString()
            tvBase.text = "C${Link.octave}"
        }
        refresh()

        v.findViewById<Button>(R.id.btnOctDown).setOnClickListener {
            if (Link.octave > 1) { Link.octave--; refresh() }
        }
        v.findViewById<Button>(R.id.btnOctUp).setOnClickListener {
            if (Link.octave < 7) { Link.octave++; refresh() }
        }

        // ---- 音色：CC 33，8 个按钮一排两个 ----
        val ids = intArrayOf(
            R.id.tim0, R.id.tim1, R.id.tim2, R.id.tim3,
            R.id.tim4, R.id.tim5, R.id.tim6, R.id.tim7
        )
        Link.timbre = prefs().getInt("timbre", 0).coerceIn(0, 7)
        for (i in 0 until 8) {
            val b = v.findViewById<Button>(ids[i])
            timBtns[i] = b
            b.text = Proto.timbreNames[i]
            b.setOnClickListener { pickTimbre(i) }
        }
        paintTimbre()

        // 音高 = 当前八度基准 + 键位页里该槽的映射（默认恒等，即 base+i 半音）
        piano.onKey = { slot, down ->
            val note = Link.baseNote + Link.keymap[slot]
            Link.send(if (down) Proto.noteOn(note) else Proto.noteOff(note))
        }
    }

    private fun prefs() = requireContext().getSharedPreferences("ti2", 0)

    private fun pickTimbre(i: Int) {
        Link.timbre = i
        prefs().edit().putInt("timbre", i).apply()
        paintTimbre()
        Link.send(Proto.timbreCc(i))
    }

    /** 选中的那个用品牌色底，其余保持暗色底 */
    private fun paintTimbre() {
        val on = ColorStateList.valueOf(0xFF0288D1.toInt())
        val off = ColorStateList.valueOf(0xFF272E38.toInt())
        for (i in 0 until 8) {
            val b = timBtns[i] ?: continue
            val sel = i == Link.timbre
            b.backgroundTintList = if (sel) on else off
            b.setTextColor(if (sel) 0xFFFFFFFF.toInt() else 0xFF9E9E9E.toInt())
        }
    }

    /**
     * 从别的 Tab 切回来时同步一下：琴上的旋钮可能刚被转过，
     * 同时保证 App 里存的音色和琴上的一致（掉电丢失靠这里补回来）。
     */
    override fun onResume() {
        super.onResume()
        Link.timbre = prefs().getInt("timbre", 0).coerceIn(0, 7)
        paintTimbre()
        if (Link.connected) Link.send(Proto.timbreCc(Link.timbre))
    }

    override fun onPause() {
        view?.findViewById<PianoView>(R.id.piano)?.releaseAll()
        super.onPause()
    }
}

// ---------------------------------------------------------------------------
// 2. 键位自定义：CC 20~31。板上没有 EEPROM，配置存手机，连上自动重发
// ---------------------------------------------------------------------------
class KeymapFragment : Fragment(R.layout.frag_keymap) {

    private lateinit var rv: RecyclerView

    override fun onViewCreated(v: View, s: Bundle?) {
        rv = v.findViewById(R.id.rvKeymap)
        load()
        rv.layoutManager = LinearLayoutManager(requireContext())
        rv.adapter = KMAdapter { slot, semi ->
            Link.keymap[slot] = semi
            save()
            Link.send(Proto.keymapCc(slot, semi))
        }

        v.findViewById<Button>(R.id.btnReset).setOnClickListener {
            for (i in 0 until 12) Link.keymap[i] = i
            save()
            rv.adapter?.notifyDataSetChanged()
            Link.pushKeymap()
            Toast.makeText(requireContext(), "已恢复默认并下发", Toast.LENGTH_SHORT).show()
        }
        v.findViewById<Button>(R.id.btnPush).setOnClickListener {
            Link.pushKeymap()
            Toast.makeText(requireContext(), "已重新下发 12 条 CC", Toast.LENGTH_SHORT).show()
        }
    }

    override fun onResume() {
        super.onResume()
        rv.adapter?.notifyDataSetChanged()
    }

    private fun prefs() = requireContext().getSharedPreferences("ti2", 0)

    private fun load() {
        val p = prefs()
        for (i in 0 until 12) Link.keymap[i] = p.getInt("km$i", i).coerceIn(0, 11)
    }

    private fun save() {
        val e = prefs().edit()
        for (i in 0 until 12) e.putInt("km$i", Link.keymap[i])
        e.apply()
    }

    private class KMAdapter(private val onChange: (Int, Int) -> Unit) :
        RecyclerView.Adapter<KMAdapter.H>() {

        class H(v: View) : RecyclerView.ViewHolder(v) {
            val tv: TextView = v.findViewById(R.id.tvSlot)
            val sp: Spinner = v.findViewById(R.id.spSemi)
        }

        override fun onCreateViewHolder(p: ViewGroup, t: Int): H =
            H(LayoutInflater.from(p.context).inflate(R.layout.item_keymap, p, false))

        override fun getItemCount() = 12

        override fun onBindViewHolder(h: H, pos: Int) {
            h.tv.text = "键 ${pos + 1}"
            val names = (0..11).map { "${Proto.noteName(it)}   ($it)" }
            h.sp.adapter = ArrayAdapter(h.itemView.context, R.layout.spinner_item, names).also {
                it.setDropDownViewResource(R.layout.spinner_item)
            }
            h.sp.setSelection(Link.keymap[pos])
            h.sp.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
                override fun onItemSelected(a: AdapterView<*>, vv: View?, i: Int, id: Long) {
                    if (i != Link.keymap[pos]) onChange(pos, i)
                }
                override fun onNothingSelected(a: AdapterView<*>) {}
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 3. 在线曲谱下载：SysEx F0 7D 01 <N> … F7
// ---------------------------------------------------------------------------
class ScoreFragment : Fragment(R.layout.frag_score) {

    private val main = Handler(Looper.getMainLooper())
    private val songs = SongBook.all.toMutableList()
    private var current: Score = SongBook.qingtian

    private lateinit var spSong: Spinner
    private lateinit var tvPreview: TextView
    private lateinit var tvLog: TextView
    private lateinit var btnGuide: Button
    private lateinit var btnPlayMode: Button

    /** true = 引导（只亮灯），false = 直接播放 */
    private var guideMode = false

    /**
     * 曲速倍率。板子一个时值单位是固定 175 ms（半拍），而不同版本 bitstream
     * 的这个常数可能不一样（旧版是 350 ms），听起来就会整首慢一倍。
     * 这里让用户在 App 侧直接缩放时值，不用重新烧板子就能调准。
     */
    private var speed = 2      // 1 = 快一倍, 2 = 正常, 3 = 慢一倍

    /** 曲子放完自动停止用的定时器 */
    private val autoStop = Runnable {
        Link.send(Proto.playCtlCc(false))
        tvLog.text = getString(R.string.score_autostop)
    }

    private fun paintMode() {
        btnGuide.backgroundTintList = ColorStateList.valueOf(
            resources.getColor(if (guideMode) R.color.brand else R.color.surface2, null))
        btnPlayMode.backgroundTintList = ColorStateList.valueOf(
            resources.getColor(if (!guideMode) R.color.brand else R.color.surface2, null))
    }

    override fun onViewCreated(v: View, s: Bundle?) {
        spSong = v.findViewById(R.id.spSong)
        tvPreview = v.findViewById(R.id.tvPreview)
        tvLog = v.findViewById(R.id.tvScoreLog)
        val etUrl = v.findViewById<EditText>(R.id.etUrl)

        render()

        spSong.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(a: AdapterView<*>, vv: View?, i: Int, id: Long) {
                if (i in songs.indices) { current = songs[i]; tvPreview.text = current.describe() }
            }
            override fun onNothingSelected(a: AdapterView<*>) {}
        }

        // 播放方式：CC 32 必须在 SysEx 之前发（板子收到 F7 就立刻开播）
        btnGuide = v.findViewById(R.id.btnModeGuide)
        btnPlayMode = v.findViewById(R.id.btnModePlay)
        btnGuide.setOnClickListener { guideMode = true; paintMode() }
        btnPlayMode.setOnClickListener { guideMode = false; paintMode() }
        paintMode()

        v.findViewById<Button>(R.id.btnDownload).setOnClickListener {
            if (!Link.connected) { toast("还没连上琴"); return@setOnClickListener }
            Link.send(Proto.playModeCc(guideMode))
            val sy = Proto.scoreSysex(Proto.scale(current.notes, speed))
            Link.send(sy)
            tvLog.text = if (guideMode)
                "《${current.name}》已传到琴上，LED 会逐个亮，跟着弹就行"
            else
                "《${current.name}》已传到琴上，正在自动播放"
            // 板子播完会从头循环，不会自己停 —— 这里按总时长掐个表，
            // 放完自动发 CC 34=0 停掉。引导模式不用掐（本来就不发声）。
            main.removeCallbacks(autoStop)
            if (!guideMode && sy.isNotEmpty())
                main.postDelayed(autoStop,
                    Proto.durationMs(Proto.scale(current.notes, speed)) + 400)
        }

        // 停止播放：CC 34 = 0，板子停下来并把残音收掉（song_player 收尾）
        v.findViewById<Button>(R.id.btnStopPlay).setOnClickListener {
            main.removeCallbacks(autoStop)
            Link.send(Proto.playCtlCc(false))
            tvLog.text = "已停止"
        }

        // 曲速：调整后需重新点击「传到琴上」才会生效
        val btnSp = arrayOf(
            v.findViewById<Button>(R.id.btnSpFast),
            v.findViewById<Button>(R.id.btnSpNorm),
            v.findViewById<Button>(R.id.btnSpSlow)
        )
        val spVal = intArrayOf(1, 2, 3)
        fun paintSpeed() {
            for (i in 0 until 3) btnSp[i].backgroundTintList = ColorStateList.valueOf(
                resources.getColor(if (spVal[i] == speed) R.color.brand else R.color.surface2, null))
        }
        for (i in 0 until 3) btnSp[i].setOnClickListener {
            speed = spVal[i]; paintSpeed()
            tvPreview.text = current.describe()
        }
        paintSpeed()

        // 从手机里选一个曲谱文件（演示「在线导入」最省事的办法：不用架服务器）
        v.findViewById<Button>(R.id.btnImportFile).setOnClickListener {
            tvLog.text = ""
            importFile.launch(arrayOf("application/json", "text/plain", "*/*"))
        }

        v.findViewById<Button>(R.id.btnImportUrl).setOnClickListener {
            val u = etUrl.text.toString().trim()
            if (u.isEmpty()) { toast("先粘贴一个曲谱网址"); return@setOnClickListener }
            tvLog.text = "正在下载…"
            fetch(u) { txt ->
                if (txt == null) { tvLog.text = "没下载成功，检查网址或网络"; return@fetch }
                addScore(txt, "网络")
            }
        }
    }

    private val importFile = registerForActivityResult(
        ActivityResultContracts.OpenDocument()
    ) { uri ->
        if (uri == null) return@registerForActivityResult
        try {
            val txt = requireContext().contentResolver
                .openInputStream(uri)?.bufferedReader()?.readText()
            if (txt.isNullOrBlank()) { tvLog.text = "这文件是空的"; return@registerForActivityResult }
            addScore(txt, "文件")
        } catch (e: Exception) {
            tvLog.text = "读文件失败：${e.message}"
        }
    }

    /** 重建曲目下拉并刷新预览。selectLast 用于导入后直接选到新曲子 */
    private fun render(selectLast: Boolean = false) {
        spSong.adapter = ArrayAdapter(requireContext(), R.layout.spinner_item,
            songs.map { it.name }).also {
            it.setDropDownViewResource(R.layout.spinner_item)
        }
        // 同名曲子用 indexOf 会选错，导入后直接选最后一条最稳
        spSong.setSelection(if (selectLast) songs.size - 1
        else songs.indexOf(current).coerceAtLeast(0))
        tvPreview.text = current.describe()
    }

    /** 解析一段曲谱文本并加到列表里，成功就自动选中 */
    private fun addScore(txt: String, from: String) {
        try {
            val sc = Score.parse("导入的曲谱", txt)
            if (sc.notes.isEmpty()) { tvLog.text = "没解析出音符，检查文件格式"; return }
            songs.add(sc)
            current = sc
            render(selectLast = true)
            tvLog.text = "从${from}导入《${sc.name}》 ${sc.notes.size} 个音，点上面的按钮传到琴上"
        } catch (e: Exception) {
            tvLog.text = "解析失败：${e.message}"
        }
    }

    private fun fetch(url: String, done: (String?) -> Unit) {
        Thread {
            var txt: String? = null
            try {
                val c = URL(url).openConnection() as HttpURLConnection
                c.connectTimeout = 8000
                c.readTimeout = 8000
                c.instanceFollowRedirects = true
                txt = c.inputStream.bufferedReader().readText()
                c.disconnect()
            } catch (e: Exception) { txt = null }
            val r = txt
            main.post { done(r) }
        }.also { it.isDaemon = true; it.start() }
    }

    private fun toast(m: String) = Toast.makeText(requireContext(), m, Toast.LENGTH_SHORT).show()

    override fun onDestroyView() {
        main.removeCallbacks(autoStop)
        super.onDestroyView()
    }
}

// ---------------------------------------------------------------------------
// 4. 演奏录音回传：收事件 → 打时间戳 → 存 .mid → 可回放 / 可导出
// ---------------------------------------------------------------------------
class RecordFragment : Fragment(R.layout.frag_record) {

    private lateinit var tvInfo: TextView
    private lateinit var tvList: TextView
    private lateinit var btnRec: Button

    private val export = registerForActivityResult(
        ActivityResultContracts.CreateDocument("audio/midi")
    ) { uri ->
        if (uri == null) return@registerForActivityResult
        val data = MidiFile.build(Rec.events())
        try {
            requireContext().contentResolver.openOutputStream(uri)?.use { it.write(data) }
            Toast.makeText(requireContext(), "已导出，可以用电脑上的播放器打开", Toast.LENGTH_SHORT).show()
        } catch (e: Exception) {
            Toast.makeText(requireContext(), "导出失败：${e.message}", Toast.LENGTH_SHORT).show()
        }
    }

    override fun onViewCreated(v: View, s: Bundle?) {
        tvInfo = v.findViewById(R.id.tvRecInfo)
        tvList = v.findViewById(R.id.tvRecList)
        btnRec = v.findViewById(R.id.btnRec)

        btnRec.setOnClickListener {
            if (Rec.running) {
                Rec.stop()
                btnRec.text = getString(R.string.btn_record)
                val n = Rec.count()
                refresh()
                if (n == 0) {
                    // 空录音几乎都是回传方向不通：录音优先录琴回传的事件。
                    // 诊断页看"已收到"是不是 0 就能确认。
                    Toast.makeText(requireContext(),
                        "一个音都没录到。\n点顶部状态文字看诊断：如果「已收到 0 字节」，" +
                        "说明琴→手机的回传不通（模块 RXD→T8 没接，或 notify 没使能）。",
                        Toast.LENGTH_LONG).show()
                } else {
                    val tip = if (Rec.source() == "琴回传") "（来自琴的回传）"
                    else "（回传不通，用的是手机本地记录 —— 在琴上弹的音录不到）"
                    Toast.makeText(requireContext(),
                        "录了 $n 个音符 $tip", Toast.LENGTH_LONG).show()
                }
            } else {
                if (!Link.connected) {
                    Toast.makeText(requireContext(), "还没连上琴", Toast.LENGTH_SHORT).show()
                    return@setOnClickListener
                }
                Rec.start()
                btnRec.text = getString(R.string.btn_stop)
                tvInfo.text = "录音中…弹几个键试试"
            }
        }

        v.findViewById<Button>(R.id.btnPlay).setOnClickListener {
            if (!Link.connected) { Toast.makeText(requireContext(), "还没连上琴", Toast.LENGTH_SHORT).show(); return@setOnClickListener }
            if (Rec.count() == 0) { Toast.makeText(requireContext(), "还没有录音", Toast.LENGTH_SHORT).show(); return@setOnClickListener }
            tvInfo.text = "回放中…"
            Rec.playback({ Link.send(it) }) {
                activity?.runOnUiThread {
                    tvInfo.text = "回放结束"
                    refresh()
                }
            }
        }

        v.findViewById<Button>(R.id.btnExport).setOnClickListener {
            if (Rec.count() == 0) { Toast.makeText(requireContext(), "还没有录音", Toast.LENGTH_SHORT).show(); return@setOnClickListener }
            export.launch("ti2_performance.mid")
        }

        v.findViewById<Button>(R.id.btnClear).setOnClickListener {
            Rec.reset()
            refresh()
        }

        refresh()
    }

    private fun refresh() {
        val ev = Rec.events()
        tvInfo.text = if (ev.isEmpty()) getString(R.string.rec_idle)
        else "录了 ${ev.size} 个音，时长 ${"%.1f".format(ev.last().ms / 1000.0)} 秒\n来源：${Rec.source()}"
        tvList.text = ev.take(60).joinToString("\n") { e ->
            val t = "${e.ms} ms".padStart(9)
            val n = Proto.noteName(e.d1 % 12)
            val kind = if (e.status and 0xF0 == 0x90) "ON " else "OFF"
            "$t  $kind  ${e.d1}($n)  v=${e.d2}"
        }
    }
}
