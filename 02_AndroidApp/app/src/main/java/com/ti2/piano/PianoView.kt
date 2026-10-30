package com.ti2.piano

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import kotlin.math.max

/**
 * 一个八度的 12 键键盘（7 白 5 黑），支持多指同时按。
 *
 * 键序与板上触摸键一致：槽 i 的默认音就是 base + i 半音，
 * 白键是 0,2,4,5,7,9,11（C D E F G A B）。
 */
class PianoView @JvmOverloads constructor(
    ctx: Context, attrs: AttributeSet? = null
) : View(ctx, attrs) {

    /** onKey(slot, down) */
    var onKey: ((Int, Boolean) -> Unit)? = null

    private val white = intArrayOf(0, 2, 4, 5, 7, 9, 11)
    private val black = intArrayOf(1, 3, 6, 8, 10)
    /** 黑键落在第几个白键边界之后 */
    private val blackAfter = intArrayOf(1, 2, 4, 5, 6)

    private val pressed = BooleanArray(12)
    private val pointerKey = HashMap<Int, Int>()

    private val pWhite = Paint(Paint.ANTI_ALIAS_FLAG)
    private val pBlack = Paint(Paint.ANTI_ALIAS_FLAG)
    private val pLine = Paint(Paint.ANTI_ALIAS_FLAG)
    private val pText = Paint(Paint.ANTI_ALIAS_FLAG)
    private val r = RectF()

    private var ww = 0f      // 白键宽
    private var bw = 0f      // 黑键宽
    private var bh = 0f      // 黑键高

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        ww = w / 7f
        bw = ww * 0.62f
        bh = h * 0.62f
        pText.textSize = ww * 0.30f
        pText.textAlign = Paint.Align.CENTER
        pLine.style = Paint.Style.STROKE
        pLine.strokeWidth = 1.5f
        pLine.color = 0xFF616161.toInt()
    }

    override fun onDraw(c: Canvas) {
        val w = width.toFloat()
        val h = height.toFloat()

        // 白键。注意：分隔线只能用 drawLine 画。
        // 分隔线需使用 drawLine 绘制：若用 drawRect(r, pLine)，pLine 默认为
        // FILL 风格，会把已绘制的白键（含按下高亮）整块覆盖为灰色。
        for (i in 0..6) {
            val semi = white[i]
            pWhite.color = if (pressed[semi]) 0xFF4FC3F7.toInt() else 0xFFF5F5F5.toInt()
            r.set(i * ww, 0f, (i + 1) * ww, h)
            c.drawRect(r, pWhite)
            if (i > 0) c.drawLine(i * ww, 0f, i * ww, h, pLine)
            pText.color = if (pressed[semi]) 0xFF00344C.toInt() else 0xFF616161.toInt()
            c.drawText(Proto.noteName(semi), i * ww + ww / 2, h - 8f, pText)
        }

        // 黑键压在上面
        for (i in black.indices) {
            val semi = black[i]
            val x = blackAfter[i] * ww - bw / 2
            pBlack.color = if (pressed[semi]) 0xFF01579B.toInt() else 0xFF212121.toInt()
            r.set(x, 0f, x + bw, bh)
            c.drawRect(r, pBlack)
        }
    }

    private fun hit(x: Float, y: Float): Int {
        // 先判黑键（在顶层）
        if (y <= bh) {
            for (i in black.indices) {
                val x0 = blackAfter[i] * ww - bw / 2
                if (x >= x0 && x <= x0 + bw) return black[i]
            }
        }
        var i = (x / ww).toInt()
        i = max(0, i.coerceAtMost(6))
        return white[i]
    }

    private fun setKey(slot: Int, down: Boolean) {
        if (pressed[slot] == down) return
        pressed[slot] = down
        invalidate()
        onKey?.invoke(slot, down)
    }

    override fun onTouchEvent(e: MotionEvent): Boolean {
        when (e.actionMasked) {
            MotionEvent.ACTION_DOWN, MotionEvent.ACTION_POINTER_DOWN -> {
                val idx = e.actionIndex
                val id = e.getPointerId(idx)
                val k = hit(e.getX(idx), e.getY(idx))
                pointerKey[id] = k
                setKey(k, true)
            }
            MotionEvent.ACTION_MOVE -> {
                for (i in 0 until e.pointerCount) {
                    val id = e.getPointerId(i)
                    val old = pointerKey[id] ?: continue
                    val k = hit(e.getX(i), e.getY(i))
                    if (k != old) {
                        setKey(old, false)
                        pointerKey[id] = k
                        setKey(k, true)
                    }
                }
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_POINTER_UP -> {
                val idx = e.actionIndex
                val id = e.getPointerId(idx)
                pointerKey.remove(id)?.let { setKey(it, false) }
            }
            MotionEvent.ACTION_CANCEL -> {
                for (i in 0 until 12) setKey(i, false)
                pointerKey.clear()
            }
        }
        return true
    }

    fun releaseAll() {
        for (i in 0 until 12) setKey(i, false)
        pointerKey.clear()
    }
}
