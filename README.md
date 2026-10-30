# TI2 电子琴 — 参赛提交代码

以 FPGA 为唯一计算平台的实时音频合成电子琴（电子设计竞赛 选题二）。
全部声音由 HDL 逻辑逐样本实时合成，不含任何预录音频或 PCM 播放通路。

---

## 一、目录结构

```
TI2_参赛提交/
├── 01_FPGA_Gowin工程/          Gowin FPGA 工程（硬件逻辑，全部为可综合 Verilog）
│   ├── gowin_ti2.gprj          Gowin 工程文件（器件 GW2A-LV18PG256C8/I7）
│   ├── syn_ti2.tcl             综合脚本
│   ├── pnr_ti2.tcl             布局布线 + 位流生成脚本
│   ├── src/                    22 个 Verilog 源文件
│   └── constraints/            引脚约束(.cst) 与时序约束(.sdc)
│
├── 02_AndroidApp/              Android 上位机（Kotlin）
│   ├── app/src/main/           源码：java/com/ti2/piano/ 与 res/
│   ├── app/build.gradle.kts    模块构建脚本
│   ├── gradle/wrapper/         Gradle Wrapper
│   ├── scores/                 曲谱 JSON 与局域网曲谱服务器启动脚本
│   └── README.md               App 自身说明
│
├── 03_硬件接线/                蓝牙模块与板卡的接线说明（BT_WIRING.md）
├── 04_演示材料/                演示视频口播稿（3 分钟内，可直接照念）
└── 05_作品海报/                A3 横版作品海报（PDF 可直接送印 + PPTX 可编辑）
```

## 二、FPGA 部分主要模块

| 文件 | 功能 |
|---|---|
| `top_synth.v` | 顶层：时钟/复位、事件汇总、自测音序、模块互连 |
| `voice_engine.v` | 32 复音时分复用合成引擎：相位累加、波表/谐波寻址、ADSR、混音归一化 |
| `goertzel_bank.v` | 12 路 Goertzel 滤波器，做频谱分析（12 个频点） |
| `i2s_tx.v` | I2S 发送，音频输出至板载 DAC |
| `sensor_hub.v` | 电容触摸条与压力感应采样 |
| `key_scan.v` | 4×4 矩阵键盘扫描 |
| `ec11_ctrl.v` | 双 EC11 旋转编码器 |
| `display_ctrl.v` | LCD 画面：李萨如 X-Y、频谱条、波形、参数面板 |
| `st7789_drv.v` / `font_cn.v` | ST7789V 驱动与中文字模 |
| `ws2812_drv.v` | WS2812B 灯带驱动 |
| `song_player.v` | 曲谱播放与 LED 演奏指引 |
| `fx_echo.v` / `fx_tone.v` | 延迟回声与音色滤波效果器 |
| `rom_sine.v` / `rom_freq.v` | 1024 点正弦函数表、频率字表 |
| `clk_gen.v` | rPLL 时钟产生（27 MHz → 40.5 MHz） |
| `midi_rx.v` / `uart_rx.v` / `uart_tx.v` / `bt_tx.v` / `i2c_bit.v` | MIDI / 串口 / 蓝牙透传 / I2C 接口 |

## 三、编译方式

### FPGA
环境：Gowin EDA V1.9.12.04（器件 GW2A-LV18PG256C8/I7，Tang Primer 20K）

```
gw_sh syn_ti2.tcl      # 综合
gw_sh pnr_ti2.tcl      # 布局布线并生成位流
```
也可直接用 Gowin IDE 打开 `gowin_ti2.gprj`。

### Android App
环境：Android Studio（Gradle Wrapper 已随包提供）

```
gradlew assembleDebug
```
或直接用 Android Studio 打开 `02_AndroidApp` 目录。
注：`local.properties`（本机 SDK 路径）需按本机环境自行生成，未随包提供。

## 四、主要技术指标

| 指标 | 数值 |
|---|---|
| 复音数 | 32（独立相位累加器口径） |
| 采样率 | 70 312.5 Hz（40.5 MHz ÷ 576） |
| 吞吐量（同时活跃振荡器数 × 采样率） | 32 × 70 312.5 ≈ 2.25 × 10⁶ |
| 端到端延迟 | < 0.3 ms |
| 系统时钟 / Fmax | 40.501 MHz 约束 / 47.075 MHz 实际，TNS = 0 |
| 资源占用 | CLS 10050/10368（97%），LUT/ALU 18865/20736（91%），寄存器 9041/16173（56%） |
| 块存储器 | BSRAM 46/46（100%，仅存放 1024×16 位正弦函数表） |

块存储器已 100% 占满，其中存放的是一张 1024 点、16 位的正弦函数表，
物理上无法容纳任何音频采样数据，可据此核对"无预录 PCM"这一合规性要求。

## 五、上板运行与自测

1. 烧录 `impl/pnr/gowin_ti2.fs`（由 `pnr_ti2.tcl` 生成）
2. 上电时压力感应区不要放置物体，首帧 ADC 读数会被采为静止零点
3. **内置自测**：同时按住键盘 key12 + key13 并保持 0.5 s 以上，
   自动播放 12 个音的扫频音序，LCD 频谱条上应同时出现 12 个峰，
   随后 32 个音齐发，复音数指示达到 32
4. **蓝牙自测**：详见 `03_硬件接线/BT_WIRING.md` 第 8 节，
   用串口工具发 `90 3C 40` 起音、`80 3C 00` 停音
5. 引脚分配核对：编译后查看 `impl/pnr/project.pin.html`，
   `bt_rxd` 应为 `T6/3`、`bt_txd` 应为 `T8/3`，Bank 电压列为 3.3

## 六、未包含的内容

以下内容为构建产物或本机环境相关，未纳入提交：

- `impl/` 综合布线中间文件与位流
- 综合与布线的 `syn_*.log` / `pnr_*.log` 构建日志
- `.bak` 备份文件、`local.properties`、`.gradle` 缓存、`apk` 安装包

如需位流，按第三节的编译方式在本地重新生成即可。

