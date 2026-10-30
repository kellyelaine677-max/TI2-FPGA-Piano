package com.ti2.piano

import android.annotation.SuppressLint
import android.app.AlertDialog
import android.bluetooth.BluetoothAdapter
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.View
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.viewpager2.adapter.FragmentStateAdapter
import androidx.viewpager2.widget.ViewPager2
import com.google.android.material.bottomnavigation.BottomNavigationView

class MainActivity : AppCompatActivity() {

    private lateinit var pager: ViewPager2
    private lateinit var nav: BottomNavigationView
    private lateinit var tvStatus: TextView
    private lateinit var btnConnect: Button
    private lateinit var vDot: View
    private val main = Handler(Looper.getMainLooper())
    private val found = ArrayList<DevInfo>()

    override fun onCreate(b: Bundle?) {
        super.onCreate(b)
        setContentView(R.layout.activity_main)

        tvStatus = findViewById(R.id.tvStatus)
        btnConnect = findViewById(R.id.btnConnect)
        vDot = findViewById(R.id.vDot)
        pager = findViewById(R.id.pager)
        nav = findViewById(R.id.nav)

        pager.adapter = object : FragmentStateAdapter(this) {
            override fun getItemCount() = 4
            override fun createFragment(pos: Int) = when (pos) {
                0 -> KeysFragment()
                1 -> KeymapFragment()
                2 -> ScoreFragment()
                else -> RecordFragment()
            }
        }
        pager.isUserInputEnabled = true

        nav.inflateMenu(R.menu.bottom_menu)
        nav.setOnItemSelectedListener {
            pager.currentItem = when (it.itemId) {
                R.id.nav_keymap -> 1
                R.id.nav_score -> 2
                R.id.nav_record -> 3
                else -> 0
            }
            true
        }
        pager.registerOnPageChangeCallback(object : ViewPager2.OnPageChangeCallback() {
            override fun onPageSelected(position: Int) {
                nav.menu.getItem(position).isChecked = true
            }
        })

        btnConnect.setOnClickListener {
            if (Link.connected) Link.disconnect() else pickDevice()
        }
        // 点状态文字弹出诊断，用来二分定位"连上了但没反应"到底断在哪
        tvStatus.setOnClickListener { showDiag() }

        // 录音要跨 Tab 工作，所以在 Activity 层统一喂数据
        Link.addRxListener(rxListener)
        Link.addStateListener(stateListener)

        ensurePermissions()
    }

    private val rxListener: (ByteArray) -> Unit = { Rec.feed(it) }

    private val stateListener: (LinkState, String) -> Unit = { s, text ->
        tvStatus.text = text
        btnConnect.text = if (s == LinkState.CONNECTED) getString(R.string.disconnect)
        else getString(R.string.connect)
        // 圆点颜色就是状态：灰=没连，蓝=连着呢，绿=已连上，红=失败
        val c = when (s) {
            LinkState.CONNECTED -> R.color.ok
            LinkState.CONNECTING -> R.color.brand
            LinkState.FAILED -> R.color.rec
            LinkState.IDLE -> R.color.off
        }
        vDot.backgroundTintList =
            android.content.res.ColorStateList.valueOf(resources.getColor(c, theme))
        if (s == LinkState.CONNECTED) {
            // 板上没有 EEPROM，连上就把手机里存的键位 + 音色推一遍
            main.postDelayed({ Link.pushSettings() }, 200)
        }
    }

    /**
     * 二分定位：字节数一直在涨 = App 确实在发，问题在模块→FPGA 那一段
     * （接线 / 波特率 / bitstream）；字节数不动 = App 这边根本没发出去。
     */
    private fun showDiag() {
        val msg = buildString {
            append("已发出：").append(Link.txBytes).append(" 字节")
            if (Link.txBytes == 0L) append("\n（还是 0，说明什么都没发出去）")
            append("\n\n已收到：").append(Link.rxBytes).append(" 字节")
            if (Link.rxBytes == 0L)
                append("\n（0 = 琴→手机方向不通。录音功能靠这个方向，收不到就录不了）")
            append("\n\n———— 链路明细 ————\n\n")
            append(Link.transport?.diag() ?: "未连接")
        }
        AlertDialog.Builder(this)
            .setTitle(R.string.diag_title)
            .setMessage(msg)
            .setPositiveButton(R.string.diag_test) { _, _ ->
                Link.send(Proto.noteOn(60))
                main.postDelayed({ Link.send(Proto.noteOff(60)) }, 700)
                Toast.makeText(this, "发了一个 C4，听一下有没有声", Toast.LENGTH_SHORT).show()
            }
            // 回传自检：清零计数 -> 请用户在琴上弹 -> 看计数涨不涨。
            // 自检时由 App 自身发音会同时被 Rec.feedLocal 记为本地记录，
            // 与回传数据混在一起无法区分，因此改为提示在琴上弹奏，
            // 接收到的即为琴端回传数据。
            .setNeutralButton("回传自检") { _, _ ->
                Link.rxBytes = 0
                AlertDialog.Builder(this)
                    .setTitle("回传自检")
                    .setMessage("现在请在【琴的触摸键】上弹 3~4 个音（别用手机键盘），\n5 秒后看结果。")
                    .setPositiveButton(R.string.ok, null)
                    .show()
                main.postDelayed({
                    val n = Link.rxBytes
                    val r = if (n > 0)
                        "收到 $n 字节 —— 琴→手机方向是通的！\n\n" +
                        "（那录音若还提示「回传不通」，就是录音页的判定问题，再录一次即可）\n\n" +
                        (Link.transport?.diag() ?: "")
                    else
                        "收到 0 字节 —— 琴→手机方向不通。\n\n" +
                        (Link.transport?.diag() ?: "") +
                        "\n\n对着上面这三项排查：\n" +
                        "· 【收】显示「无」或「不含 NOTIFY」→ 模块没暴露通知特征，换模块\n" +
                        "· 【CCCD】status 不是 成功(0) → 订阅没写进去，重连一次\n" +
                        "· 上面都对但还是 0 → 就是硬件：模块 RXD 有没有接到板上的 T8、" +
                        "波特率是不是 9600、PWRC 别接地"
                    AlertDialog.Builder(this)
                        .setTitle("回传自检结果")
                        .setMessage(r)
                        .setPositiveButton(R.string.ok, null)
                        .show()
                }, 5000)
            }
            .setNegativeButton(R.string.cancel, null)
            .show()
    }

    private fun ensurePermissions() {
        val missing = Bt.neededPermissions().filter {
            ActivityCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }
        if (missing.isNotEmpty()) requestPermissions(missing.toTypedArray(), 1)
    }

    override fun onRequestPermissionsResult(
        requestCode: Int, permissions: Array<out String>, grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (grantResults.any { it != PackageManager.PERMISSION_GRANTED })
            Toast.makeText(this, "没有蓝牙权限就无法扫描和连接", Toast.LENGTH_LONG).show()
    }

    // -----------------------------------------------------------------------
    // 设备选择：经典蓝牙（HC-06）+ 低功耗蓝牙（JDY-16）混在一个列表里
    // -----------------------------------------------------------------------
    private fun pickDevice() {
        if (!Bt.enabled(this)) {
            Toast.makeText(this, "请先在系统设置里打开蓝牙", Toast.LENGTH_LONG).show()
            try {
                @Suppress("DEPRECATION")
                startActivity(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE))
            } catch (_: Exception) {}
            return
        }
        found.clear()
        // SPP：必须先在系统设置里配对过；BLE：已配对过的。按地址去重避免同一个模块出现两次
        //  BLE 排在前面：JDY-16 配对后 type 常仍为 UNKNOWN，若先取
        // `paired()`，该设备会被判定为经典蓝牙并建立 RFCOMM socket，
        // 对仅支持 BLE 的模块表现为可连接但无法收发。
        val seen = HashSet<String>()
        for (d in Bt.bondedLe(this) + Bt.paired(this)) if (seen.add(d.address)) found.add(d)
        if (found.isEmpty()) scanLe() else showPicker()
    }

    @SuppressLint("MissingPermission")
    private fun scanLe() {
        if (!Bt.hasPermissions(this)) { ensurePermissions(); return }
        val scanner = Bt.adapter(this)?.bluetoothLeScanner
        if (scanner == null) { showPicker(); return }
        Toast.makeText(this, "正在扫描…（8 秒）", Toast.LENGTH_SHORT).show()
        val cb = object : ScanCallback() {
            override fun onScanResult(callbackType: Int, result: ScanResult) {
                val d = result.device ?: return
                val name = d.name ?: return
                val info = DevInfo(Kind.BLE, name, d.address, d)
                if (found.none { it.address == info.address }) {
                    found.add(info)
                }
            }
            override fun onScanFailed(errorCode: Int) {
                main.post { Toast.makeText(this@MainActivity, "扫描失败 $errorCode", Toast.LENGTH_SHORT).show() }
            }
        }
        scanner.startScan(cb)
        main.postDelayed({
            try { scanner.stopScan(cb) } catch (_: Exception) {}
            showPicker()
        }, 8000)
    }

    private fun showPicker() {
        if (found.isEmpty()) {
            Toast.makeText(
                this,
                "没找到设备。\n经典蓝牙要先在系统设置里配对一次；\n低功耗蓝牙打开蓝牙就能扫到。",
                Toast.LENGTH_LONG
            ).show()
            return
        }
        val names = found.map { it.toString() }.toTypedArray()
        AlertDialog.Builder(this)
            .setTitle(R.string.pick_device)
            .setItems(names) { _, which -> Link.connect(this, found[which]) }
            .setNegativeButton(R.string.cancel, null)
            .setNeutralButton("重新扫描") { _, _ ->
                found.clear()
                found.addAll(Bt.paired(this))
                scanLe()
            }
            .show()
    }

    override fun onDestroy() {
        Link.removeRxListener(rxListener)
        Link.removeStateListener(stateListener)
        Link.disconnect()
        super.onDestroy()
    }
}
