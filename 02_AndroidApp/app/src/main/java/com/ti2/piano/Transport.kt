package com.ti2.piano

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.BluetoothSocket
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.os.Handler
import android.os.Looper
import androidx.core.app.ActivityCompat
import java.io.IOException
import java.util.UUID
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

enum class Kind { SPP, BLE }

enum class LinkState { IDLE, CONNECTING, CONNECTED, FAILED }

data class DevInfo(
    val kind: Kind,
    val name: String,
    val address: String,
    val device: BluetoothDevice
) {
    val kindLabel: String get() = if (kind == Kind.SPP) "经典蓝牙" else "低功耗蓝牙"
    // 列表里不显示 MAC 地址 —— 对用户没意义，只会显得像调试界面
    override fun toString(): String = "$kindLabel  $name"
}

interface Transport {
    val kind: Kind
    var onRx: ((ByteArray) -> Unit)?
    var onState: ((LinkState, String) -> Unit)?
    fun open(dev: DevInfo)
    fun write(data: ByteArray)
    fun close()
    /** 诊断用的一行描述，用来确认"连上"到底连到了什么 */
    fun info(): String

    /**
     * 诊断用的多行明细。
     *
     * 回传一直为 0 的时候，光看"连上了"没有意义 —— 必须能看到
     * notify 特征找没找到、CCCD 写没写成功、回调到底触发过几次。
     * 这三项有任何一项不对，收方向就一定是死的。
     */
    fun diag(): String = info()
}

/** 权限与设备枚举的小工具 */
@SuppressLint("MissingPermission")
object Bt {
    fun adapter(ctx: Context): BluetoothAdapter? =
        (ctx.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager)?.adapter

    fun enabled(ctx: Context): Boolean = adapter(ctx)?.isEnabled == true

    fun neededPermissions(): Array<String> = if (Build.VERSION.SDK_INT >= 31)
        arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
    else
        arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)

    fun hasPermissions(ctx: Context): Boolean =
        neededPermissions().all {
            ActivityCompat.checkSelfPermission(ctx, it) == PackageManager.PERMISSION_GRANTED
        }

    /**
     * 经典蓝牙：已配对设备（HC-06 必须先在系统设置里配对一次）
     *
      *  若判据取 `type != DEVICE_TYPE_LE`，则 DEVICE_TYPE_UNKNOWN(0) 也会被
      * 归为经典蓝牙。JDY-16 刚配对完成、或系统尚未缓存设备类型时 type 为
      * UNKNOWN，会被标为"经典蓝牙"并走 SPP（RFCOMM）连接，
      * 对仅支持 BLE 的模块将无法正常收发。
      * 因此此处仅认明确的 CLASSIC，UNKNOWN 一律归入 BLE。
     */
    fun paired(ctx: Context): List<DevInfo> {
        val a = adapter(ctx) ?: return emptyList()
        return a.bondedDevices.orEmpty()
            .filter { it.type == BluetoothDevice.DEVICE_TYPE_CLASSIC }
            .map { DevInfo(Kind.SPP, it.name ?: "未知设备", it.address, it) }
            .sortedBy { it.name }
    }

    /** BLE：已配对过的、类型不是纯经典蓝牙的设备（含 UNKNOWN / DUAL） */
    fun bondedLe(ctx: Context): List<DevInfo> {
        val a = adapter(ctx) ?: return emptyList()
        return a.bondedDevices.orEmpty()
            .filter { it.type != BluetoothDevice.DEVICE_TYPE_CLASSIC }
            .map { DevInfo(Kind.BLE, it.name ?: "未知设备", it.address, it) }
            .sortedBy { it.name }
    }
}

// ---------------------------------------------------------------------------
// 经典蓝牙 SPP（HC-06 / HC-05）
// ---------------------------------------------------------------------------
@SuppressLint("MissingPermission")
class SppTransport : Transport {
    override val kind = Kind.SPP
    override var onRx: ((ByteArray) -> Unit)? = null
    override var onState: ((LinkState, String) -> Unit)? = null

    private val uuid = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
    @Volatile private var socket: BluetoothSocket? = null
    @Volatile private var worker: Thread? = null

    override fun open(dev: DevInfo) {
        close()
        onState?.invoke(LinkState.CONNECTING, dev.name)
        worker = Thread {
            // socket 用非空 val 持有，后面就不用到处加 ?. / !!
            try {
            val s: BluetoothSocket = try {
                dev.device.createRfcommSocketToServiceRecord(uuid)
            } catch (e: Exception) {
                // 部分 HC-06 底板不认 SDP，直接走通道 1
                @Suppress("DEPRECATION")
                val m = dev.device.javaClass.getMethod("createRfcommSocket", Int::class.javaPrimitiveType)
                m.invoke(dev.device, 1) as BluetoothSocket
            }
            try {
                s.connect()
                socket = s
                onState?.invoke(LinkState.CONNECTED, dev.name)
                val input = s.inputStream
                val buf = ByteArray(256)
                while (!Thread.currentThread().isInterrupted) {
                    val n = input.read(buf)
                    if (n <= 0) break
                    readCount++
                    lastRxHex = hex(buf, n)
                    onRx?.invoke(buf.copyOf(n))
                }
                onState?.invoke(LinkState.FAILED, "连接已断开")
            } catch (e: Exception) {
                onState?.invoke(LinkState.FAILED, e.message ?: "SPP 连接失败")
            } finally {
                try { s.close() } catch (_: IOException) {}
            }
            } catch (e: Exception) {
                onState?.invoke(LinkState.FAILED, "无法创建 socket：${e.message}")
            }
        }.also { it.isDaemon = true; it.start() }
    }

    override fun write(data: ByteArray) {
        try {
            socket?.outputStream?.let { it.write(data); it.flush() }
        } catch (_: Exception) {
            onState?.invoke(LinkState.FAILED, "发送失败")
        }
    }

    override fun close() {
        worker?.interrupt()
        worker = null
        try { socket?.close() } catch (_: Exception) {}
        socket = null
    }

    override fun info(): String =
        "经典蓝牙 SPP" + if (socket?.isConnected == true) "（socket 已通）" else "（socket 未通）"

    @Volatile private var lastRxHex = ""
    @Volatile private var readCount = 0L

    private fun hex(b: ByteArray, n: Int): String =
        (0 until minOf(12, n)).joinToString(" ") {
            (b[it].toInt() and 0xFF).toString(16).uppercase().padStart(2, '0')
        } + if (n > 12) " …($n)" else ""

    override fun diag(): String = buildString {
        append("【传输】").append(info()).append("\n")
        append("【读取】socket 读到 ").append(readCount).append(" 次\n")
        append("【末包】").append(if (lastRxHex.isEmpty()) "（没收到过任何字节）" else lastRxHex)
        append("\n\n如果这里显示的是经典蓝牙，而你的模块是 JDY-16/HM-10 这类")
        append("BLE 模块，那就是选错通道了 —— 重新连接，列表里选带")
        append("「低功耗蓝牙」字样的那一行。")
    }
}

// ---------------------------------------------------------------------------
// 低功耗蓝牙透传（JDY-16 / HM-10 / MLT-BT05，以及 Nordic UART 兼容模块）
//
// BLE 与 SPP 最大的区别：一包只有 MTU-3 字节有效负载（默认 20）。
// 曲谱 SysEx 最长 68 字节，必须分包，且必须等上一个 write 的回调再发下一包，
// 否则会静默丢包。这些都封在 write() 里，上层无感。
// ---------------------------------------------------------------------------
@SuppressLint("MissingPermission")
class BleTransport(private val ctx: Context) : Transport {
    override val kind = Kind.BLE
    override var onRx: ((ByteArray) -> Unit)? = null
    override var onState: ((LinkState, String) -> Unit)? = null

    private val SRV = listOf(
        UUID.fromString("0000ffe0-0000-1000-8000-00805f9b34fb"), // JDY-16 / HM-10
        UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e")  // Nordic UART
    )
    private val CCCD = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

    private var gatt: BluetoothGatt? = null
    private var writeChar: BluetoothGattCharacteristic? = null
    private var notifyChar: BluetoothGattCharacteristic? = null
    // 模块可能暴露不止一个 notify 特征。实测某批 JDY-16 固件是
    // FFE1（写+通知）+ FFE2（通知）双特征结构，而串口数据只从 FFE1 推：
    // 只订阅其中一个（原代码取"最后一个带 NOTIFY 的"= FFE2）就会
    // CCCD 写成功、回调却永远不来。所以这里存整个列表，全部订阅。
    private var notifyChars: List<BluetoothGattCharacteristic> = emptyList()
    @Volatile private var mtu = 20
    @Volatile private var pending: CountDownLatch? = null
    @Volatile private var connState = LinkState.IDLE
    private var devName = "BLE"
    private val main = Handler(Looper.getMainLooper())

    // ---- 诊断采集：回传为 0 时靠这几项一次定位是断在哪 --------------------
    @Volatile private var svcSeen = ""          // 扫到的全部服务 UUID
    @Volatile private var notifyListDesc = ""   // 订阅了哪些 notify 特征
    @Volatile private var cccdResults = ""      // 每个 CCCD 的写入结果
    @Volatile private var writeProp = 0         // 写特征的 properties
    @Volatile var notifyCount = 0L              // onCharacteristicChanged 触发次数
    @Volatile var lastRxHex = ""                // 最后收到的原始字节
    @Volatile private var mtuStatus = -999

    // CCCD 要一个一个串行写（GATT 不允许并发操作），队列在 onDescriptorWrite 里推进
    private var cccdQueue: ArrayDeque<BluetoothGattCharacteristic>? = null
    @Volatile private var notifyStarted = false

    private fun hex(b: ByteArray): String = b.take(12).joinToString(" ") {
        (it.toInt() and 0xFF).toString(16).uppercase().padStart(2, '0')
    } + if (b.size > 12) " …(${b.size})" else ""

    override fun diag(): String {
        val w = writeChar
        return buildString {
            append("【传输】低功耗蓝牙  MTU=$mtu（requestMtu 状态 $mtuStatus）\n")
            append("【服务】").append(if (svcSeen.isEmpty()) "（空）" else svcSeen).append("\n")
            append("【写  】").append(w?.uuid?.toString()?.take(8) ?: "无")
                .append("  属性 0x").append(writeProp.toString(16)).append("\n")
            append("【收  】").append(if (notifyListDesc.isEmpty()) "无 ← 没有 notify 特征！" else notifyListDesc)
                .append("\n")
            append("【CCCD】").append(if (cccdResults.isEmpty()) "（还没写）" else cccdResults)
                .append("\n")
            append("【回调】onCharacteristicChanged 触发 ").append(notifyCount).append(" 次\n")
            append("【末包】").append(if (lastRxHex.isEmpty()) "（没收到过任何字节）" else lastRxHex)
            append("\n\n判定：")
            append(when {
                notifyChars.isEmpty() -> "\n· 模块根本没暴露 notify 特征 → 收方向必然为 0"
                !cccdResults.contains("成功") -> "\n· CCCD 一个都没写成功 → 模块不会推数据，重连一次"
                notifyCount == 0L -> "\n· 订阅全部成功但一次回调都没有 → 模块没往手机发（硬件/波特率）"
                else -> "\n· 收方向正常"
            })
        }
    }

    override fun open(dev: DevInfo) {
        close()
        devName = dev.name
        onState?.invoke(LinkState.CONNECTING, dev.name)
        gatt = dev.device.connectGatt(ctx, false, cb, BluetoothDevice.TRANSPORT_LE)
    }

    private val cb = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                onState?.invoke(LinkState.CONNECTING, "$devName：发现服务…")
                g.discoverServices()
            } else {
                connState = LinkState.FAILED
                onState?.invoke(LinkState.FAILED, "BLE 断开 ($status)")
                close()
            }
        }

        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
            var wc: BluetoothGattCharacteristic? = null
            val ncs = ArrayList<BluetoothGattCharacteristic>()
            val svcs = ArrayList<String>()
            for (s in g.services) {
                svcs.add(s.uuid.toString().substring(4, 8))
                if (s.uuid !in SRV) continue
                for (c in s.characteristics) {
                    val props = c.properties
                    if (props and (BluetoothGattCharacteristic.PROPERTY_WRITE or
                                BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE) != 0) {
                        if (wc == null) wc = c
                    }
                    //  收集**全部**带 NOTIFY/INDICATE 的特征，不要只留一个。
                    // 实测 JDY-16 某固件：FFE1 和 FFE2 都标 NOTIFY，但串口数据
                    // 只从 FFE1 推。原代码 `nc = c` 循环完只剩最后一个（FFE2），
                    // 订了个永远不出声的特征 —— CCCD 写成功、回调 0 次。
                    if (props and (BluetoothGattCharacteristic.PROPERTY_NOTIFY or
                                BluetoothGattCharacteristic.PROPERTY_INDICATE) != 0) ncs.add(c)
                }
            }
            svcSeen = svcs.joinToString(",")
            // 兜底：有些模块把 notify 特征放在 FFE0/NUS 之外的自定义服务里，
            // 退一步全表扫一遍。
            if (ncs.isEmpty()) {
                for (s in g.services) for (c in s.characteristics) {
                    if (c.properties and (BluetoothGattCharacteristic.PROPERTY_NOTIFY or
                                BluetoothGattCharacteristic.PROPERTY_INDICATE) != 0) {
                        ncs.add(c)
                        svcSeen += "（notify 取自 ${s.uuid.toString().substring(4, 8)}）"
                        break
                    }
                }
            }
            // JDY-16 的 FFE1 通常同时具备 write 和 notify，缺哪个就用另一个顶上
            if (wc == null) wc = ncs.firstOrNull()
            writeChar = wc
            notifyChars = ncs
            notifyChar = ncs.firstOrNull()
            writeProp = wc?.properties ?: 0
            notifyListDesc = ncs.joinToString(", ") {
                "${it.uuid.toString().substring(4, 8)}(0x${it.properties.toString(16)})"
            }
            if (wc == null) {
                onState?.invoke(LinkState.FAILED, "找不到可写特征（FFE1 / NUS RX）")
                return
            }
            //  GATT 操作必须串行：requestMtu 还没回调就 writeDescriptor 的话，
            // 后者会被底层直接丢掉（返回 false 或 GATT_BUSY），CCCD 写不进去，
            // notify 就永远使能不了 —— 表现就是"能发不能收"，录音永远录不到。
            // 所以：先 requestMtu，使能 notify 放到 onMtuChanged 回调里。
            try { g.requestMtu(64) } catch (_: Exception) {}
            // 个别手机不发 onMtuChanged 回调，兜底 600 ms 后自己继续
            main.postDelayed({ if (!notifyStarted) enableNotify(g) }, 600)
        }

        override fun onMtuChanged(g: BluetoothGatt, m: Int, status: Int) {
            mtuStatus = status
            if (status == BluetoothGatt.GATT_SUCCESS) mtu = m
            enableNotify(g)
        }

        override fun onDescriptorWrite(g: BluetoothGatt, d: BluetoothGattDescriptor, status: Int) {
            val cu = d.characteristic?.uuid?.toString()?.substring(4, 8) ?: "?"
            cccdResults = (if (cccdResults.isEmpty()) "" else "$cccdResults  ") +
                    "$cu:" + when (status) {
                0 -> "成功(0)" ; else -> "失败($status)"
            }
            // 一个写完再写下一个（GATT 不允许并发），队列空了才算全部订阅完
            writeNextCccd(g)
        }

        private fun enableNotify(g: BluetoothGatt) {
            if (notifyStarted) return
            notifyStarted = true
            if (notifyChars.isEmpty()) {
                // 连 notify 特征都没有也照常报已连上：发方向不受影响，
                // 诊断页的"收：无"会把问题暴露出来
                onState?.invoke(LinkState.CONNECTED, devName)
                return
            }
            cccdQueue = ArrayDeque(notifyChars)
            writeNextCccd(g)
        }

        private fun writeNextCccd(g: BluetoothGatt) {
            val q = cccdQueue ?: return
            val nc = q.removeFirstOrNull()
            if (nc == null) {
                // 全部 CCCD 写完才算真正连上
                if (connState != LinkState.CONNECTED) {
                    connState = LinkState.CONNECTED
                    onState?.invoke(LinkState.CONNECTED, devName)
                }
                return
            }
            notifyChar = nc
            g.setCharacteristicNotification(nc, true)
            val d = nc.getDescriptor(CCCD)
            if (d == null) { writeNextCccd(g); return }
            //  必须先给 descriptor 赋使能值再 write：value 为空的 CCCD 写出去
            // 等于"别发通知"，模块于是一个字节都不推。
            //
            // INDICATE 特征使能值是 0x0002，NOTIFY 是 0x0001，写错了一样收不到。
            d.value = if (nc.properties and BluetoothGattCharacteristic.PROPERTY_INDICATE != 0)
                BluetoothGattDescriptor.ENABLE_INDICATION_VALUE
            else
                BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            if (!g.writeDescriptor(d)) writeNextCccd(g)
        }

        override fun onCharacteristicWrite(g: BluetoothGatt, c: BluetoothGattCharacteristic, status: Int) {
            pending?.countDown()
        }

        private fun deliver(b: ByteArray) {
            if (b.isEmpty()) return
            notifyCount++
            lastRxHex = hex(b)
            onRx?.invoke(b)
        }

        @Deprecated("旧版回调，Android 12 及以下使用")
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic) {
            if (Build.VERSION.SDK_INT < 33) deliver(c.value ?: ByteArray(0))
        }

        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic, value: ByteArray) {
            deliver(value)
        }
    }

    override fun write(data: ByteArray) {
        val g = gatt ?: return
        val c = writeChar ?: return
        val step = (mtu - 3).coerceIn(20, 512)
        var off = 0
        while (off < data.size) {
            val len = minOf(step, data.size - off)
            c.value = data.copyOfRange(off, off + len)
            c.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
            val latch = CountDownLatch(1)
            pending = latch
            if (!g.writeCharacteristic(c)) { pending = null; break }
            latch.await(2, TimeUnit.SECONDS)
            off += len
        }
    }

    override fun close() {
        connState = LinkState.IDLE
        notifyStarted = false
        cccdQueue = null
        main.removeCallbacksAndMessages(null)
        try { gatt?.disconnect() } catch (_: Exception) {}
        try { gatt?.close() } catch (_: Exception) {}
        gatt = null
        writeChar = null
        notifyChar = null
        notifyChars = emptyList()
    }

    override fun info(): String {
        val w = writeChar
        return if (w == null) "低功耗蓝牙（还没找到可写特征！）"
        else "低功耗蓝牙  写特征 ${w.uuid.toString().take(8)}…  MTU=$mtu"
    }
}
