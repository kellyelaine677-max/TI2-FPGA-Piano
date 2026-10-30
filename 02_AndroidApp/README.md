# TI2 电子琴 — 手机端 App

对应赛题**拓展要求 4**：蓝牙模块与手机 App 联动、在线曲谱下载、演奏键位自定义、
演奏录音回传。FPGA 工程在 `D:\fpga_fight\gowin_ti2`，本文档只讲手机端。

## 1. 工具链（已装好，在 `D:\Android`）

| 东西 | 路径 |
|---|---|
| Android Studio 2023.1.1.28 (Iguana) | `D:\Android\android-studio\bin\studio64.exe` |
| JDK 17（IDE 自带 jbr，Gradle 8.0 需要） | `D:\Android\android-studio\jbr` |
| Android SDK | `D:\Android\Sdk`（platform 34 / build-tools 34.0.0 / platform-tools） |
| Gradle 8.0 | `D:\Android\gradle-8.0`（工程里也有 wrapper） |
| Gradle 缓存 | `D:\Android\.gradle`（已写入环境变量 `GRADLE_USER_HOME`，IDE 打开不用重新下载） |

已设置的用户环境变量：`ANDROID_HOME`、`ANDROID_SDK_ROOT`、`GRADLE_USER_HOME`。

### 装 APK 到手机

编译产物 **`Ti2Piano-debug.apk`** 就在工程根目录（5.5 MB，debug 签名）。
直接微信/QQ 传到手机点开就能装（需要允许"未知来源"）。

或者：手机开 USB 调试、连电脑，跑

```
D:\Android\Sdk\platform-tools\adb.exe install -r D:\fpga_fight\Ti2Piano\Ti2Piano-debug.apk
```

### 改完代码重新编译

双击 **`build.bat`**（或用 Android Studio 打开工程）。它会自动配好
`JAVA_HOME`/`ANDROID_HOME` 并把产物拷到根目录。

### 用 Android Studio 打开

`D:\Android\android-studio\bin\studio64.exe` → 打开 `D:\fpga_fight\Ti2Piano`
（选这个目录，不是 `app` 子目录）。首次打开直接 Sync 即可，依赖已在缓存里。

> 编译已实际跑通：`BUILD SUCCESSFUL`，33 个 task。
> 踩到的坑记一下：material 1.11.0 **没有** `Theme.MaterialComponents.Dark.NoActionBar`
> （只有 Light / DayNight 变体），Dark 变体只有 Material3 有，所以主题必须用
> `Theme.Material3.Dark.NoActionBar`。

## 2. 连接

App 同时支持两种链路，点右上角「连接」后会弹出设备列表：

| 模块 | 准备工作 |
|---|---|
| **HC-06**（经典蓝牙 SPP） | 先在**系统设置 → 蓝牙**里配对一次（密码一般 1234 或 0000），App 才能列出来 |
| **JDY-16**（BLE） | 不用预配对，点「重新扫描」直接扫到 |

连上后 App 会自动把手机里存的键位表重发一遍（`B0 14~1F`），
弥补板子上没有 EEPROM 的问题。

## 3. 四个页面

### 弹琴
一个八度的 12 键（7 白 5 黑），**支持多指同时按 = 和弦**。
音高 = 当前八度基准 + 键位页里该键的映射值（默认就是 `base + i` 半音）。
八度按钮只影响 App 算出来的绝对音高，不受板上八度键影响。

### 键位
12 个键各自选一个音（C C# D … B），改动立刻生效。
设置存在手机里，**每次连上自动重发** —— 板子没有 EEPROM，掉电会忘。

### 曲谱
内置 **7 首**：晴天、退后、小星星、欢乐颂、两只老虎、划船歌、音阶往返。

两种导入方式：

- **从文件导入**（演示最省事）—— 把 `scores/` 里的 json 传到手机，点按钮选它
- **从网址导入** —— 填一个 json 的网址

`scores/` 目录里已经放了 `花海.json`（**专门用于演示导入**）、`晴天.json`、`退后.json`、
`sample.json`。谱不对可以直接改这些文件再重新导入，不用改代码重新编译。

JSON 格式：

```json
{"name":"曲名","notes":[{"d":0,"b":1},{"d":4,"b":2}]}
```

`d` = 音级 0~6（C D E F G A B），`b` = 时值 1~15（拍，板上一拍 0.35 s）。
也接受 `"deg"/"beats"` 别名，以及 `"0:1,4:2"` 这种紧凑串。

> **八度限制**：每个音符只有 7 位（3 位音级 + 4 位时值），**没有八度位**。
> 所以原曲里带高低音点的音（简谱的 `1̇`、`.6`）都折回了同一个八度 ——
> 旋律走向是对的，音区被压平了。《晴天》《退后》就是这么处理的。
> 要完整还原的话得改 FPGA 编码（每音符两字节），需要的话跟我说。

> 编码里每音符只有 7 位（3 位音级 + 4 位时值），**没有八度位**，
> 所以只能下发单个八度内的旋律。

### 录音
板子把每个音符事件当场流式送出（`90 nn 40` / `80 nn 00`），没有时间戳。
App 用"收到的时刻"补时间戳（UART 实时，3 字节 @9600 只有 3.1 ms 延迟，可忽略），
存成标准 `.mid`：

- **回放**：按记录的真实间隔把事件发回琴，琴自己重新合成一遍
- **导出**：存成 `.mid`，电脑上任何 DAW 都能打开 —— 评委拿到的是一份能验证的文件

音频本身传不动：70312.5 Hz × 16 bit = 1.4 Mbps，9600 波特物理上不可能。
这句话答辩时可以直接讲。

## 4. 文件结构

```
app/src/main/java/com/ti2/piano/
  Protocol.kt     MIDI / CC / SysEx 组包，曲谱模型与内置曲库，MIDI 输入解析
  Transport.kt    双模传输：SppTransport（HC-06）+ BleTransport（JDY-16 / HM-10）
  Link.kt         全 App 唯一的链路对象（发送走单线程 executor）
  MidiFile.kt     录音 → 标准 .mid 导出 + 回放
  PianoView.kt    12 键键盘 View，多指和弦
  Fragments.kt    四个页面
  MainActivity.kt 设备选择（SPP + BLE 混列）、权限、Tab 容器
```

## 5. 手工自测（不装 App 也行）

用任意蓝牙串口 App 切到十六进制发送：

| 目的 | 发送 | 预期 |
|---|---|---|
| 响一个音 | `90 3C 40` | 出声（C4） |
| 停 | `80 3C 00` | 停 |
| 键位改 G | `B0 14 07` | 按板上第 1 个触摸键音变了 |
| 下载 2 音曲谱 | `F0 7D 01 02 01 22 F7` | LED 曲目指引改播新谱 |

两条说明：

- CC 的值 0~11 是**半音偏移**不是音级：0=C 1=C# 2=D 3=D# 4=E … **7=G** … 11=B。
  想让 0 号槽弹 G 就发 `B0 14 07`。
- 这条 SysEx 是「音级 0（C）持续 1 拍 + 音级 2（E）持续 2 拍」：
  音符字节 `01` = `(0<<4)|1`，`22` = `(2<<4)|2`。
