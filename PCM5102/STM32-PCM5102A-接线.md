# STM32F407VET6 ↔ PCM5102A 接线说明

## I2S 信号线

| STM32 引脚 | 功能 | → | PCM5102A 引脚 | 功能 |
|------------|------|---|---------------|------|
| **PB10** | I2S2_CK (BCLK) | → | **BCK** (Pin 13) | Bit Clock |
| **PB12** | I2S2_WS (LRCK) | → | **LRCK** (Pin 15) | Word Select |
| **PC3** | I2S2_SD (DIN) | → | **DIN** (Pin 14) | Serial Data |

## 未使用 MCLK

代码中 `MCLKOutput = I2S_MCLKOUTPUT_DISABLE`。PCM5102A 自带 PLL，无需外部系统时钟，使用 BCK 即可恢复时钟。

**PCM5102A 的 SCK 引脚 (Pin 10)** 应接 **GND**（PLL 模式）。

## PCM5102A 控制引脚

| PCM5102A 引脚 | 接法 | 说明 |
|---------------|------|------|
| **XSMT** (Pin 12) | **3.3V** | 软静音禁用，正常输出 |
| **DEMP** (Pin 18) | **GND** | 不使用去加重 |
| **FMT** (Pin 16) | **GND** | I2S 格式 |
| **FLT** (Pin 17) | **GND** 或 **3.3V** | 滤波器选择：L=Normal latency，H=Low latency |

## 电源

| PCM5102A 引脚 | 接法 |
|---------------|------|
| **AVDD** (Pin 2,8) | 3.3V |
| **DVDD** (Pin 20) | 3.3V |
| **CPVDD** (Pin 5) | 3.3V |
| **AGND** (Pin 1,4,7,9) | GND |
| **DGND** (Pin 19) | GND |

## PCM5102A 输出

| 引脚 | 功能 |
|------|------|
| **LOL** (Pin 5) | 左声道 线输出 |
| **LOR** (Pin 6) | 右声道 线输出 |

直接接功放或耳机放大器即可，PCM5102A 内置电荷泵可输出 2.1Vrms 接地中心信号，无需隔直电容。

## 备注

- 所有 GND 共地
- 3.3V 电源加 10μF + 0.1μF 去耦电容
- PCM5102A 上电后 XSMT 拉高才会开始输出，上电默认静音
