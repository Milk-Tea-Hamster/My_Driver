# STM32H743 电赛快速开发模板

本工程用于电子设计竞赛现场快速组合常用功能。CubeMX 生成的 `Core`
目录负责时钟、GPIO、DMA 和外设初始化；`App` 目录负责可复用业务模块。

## 默认启用功能

功能开关统一位于 `App/app_config.h`：

- ADC1 + TIM6 TRGO + DMA Normal 块采样
- ADC 峰峰值、平均值和抗毛刺分析
- 2048 点 FFT、频谱、谱峰插值和波形初步识别
- TIM2 高低频双模式测频
- USART1/USART3 ReceiveToIdle DMA
- 裸机周期调度器

AD9910、AD9959 默认保留但不初始化，按项目需要打开对应开关。

## CubeMX 是硬件参数来源

修改 `.ioc` 并重新生成代码后，重点检查：

1. ADC1 仍由 TIM6 TRGO 上升沿触发。
2. ADC DMA 使用 Half Word、Memory Increment 和 DMA Normal。
3. TIM6 的 Prescaler、Period 和 TRGO 为 Update Event。
4. TIM2 CH1 保持输入捕获配置。
5. USART1/USART3 RX DMA 保持 DMA Normal，并启用对应 DMA 中断。
6. `.AXI_SRAM` 所在内存仍允许 DMA1 访问，并与 MPU Cache 属性一致。

应用层通过 TIM6 句柄和实际 APB1 定时器内核时钟计算采样率，不再复制
CubeMX 中的固定采样率。当前 CubeMX 配置对应约 1 MHz。

DMA 缓冲区显式放在非缓存 AXI SRAM；FFT 工作区显式放在 CPU 可直接访问的
DTCM。重新生成或修改 Keil scatter 文件时，需要保留 `.AXI_SRAM` 和
`.DTCM_RAM` 两个段的映射。

## ADC 使用方式

`main.c` 默认完成初始化：

```c
ADC_App_Init(&hadc1, &htim6);
```

DMA 采满一块数据后立即停止 TIM6 触发，主循环调用：

```c
ADC_App_Process(&g_wave_info);
```

处理完成后自动重新启动 DMA 和 TIM6。这样 CPU 分析期间缓冲区不会被 DMA
覆盖，适合比赛模板中的单缓冲稳定采样。

可通过以下接口读取结果：

```c
const adc_waveform_info_t *adc = ADC_App_GetWaveformInfo();
const uint16_t *samples = ADC_App_GetBuffer();
float sample_rate = ADC_App_GetSampleRateHz();
```

## FFT 使用方式

快速分析：

```c
FFT_Result result;
FFT_Analyze(adc_samples, &result);
```

也保留分步公开接口，便于现场自定义流程：

```c
fft_prepare(adc_samples, &input);
float vpp = find_vpp(&input);       /* 必须在原地 FFT 前调用 */
fft_window(&input, window);
fft_process(&input, &output);       /* 会覆盖 input 中的时域数据 */
fft_find_peaks(&output, &peaks);
```

波形识别阈值位于 `fft_analyzer.h`。默认值只作为理论起点，正式参赛项目应
使用实际模拟前端和信号源采集数据后标定。

## 串口接收

USART ReceiveToIdle DMA 使用 Normal 模式。`Serial_Process()` 在处理完一帧
后自动重新启动 DMA。用户只需在业务代码中覆盖弱函数：

```c
void Serial_OnFrame(UART_HandleTypeDef *huart,
                    const uint8_t *data,
                    uint16_t length)
{
    /* 解析完整 IDLE 帧；不要长期保存 data 指针。 */
}
```

## 注意事项

- `g_wave_info.FreqValid` 表示 TIM2 频率是否有效。
- `g_wave_info.AdcValid` 表示是否完成过至少一块 ADC 分析。
- FFT 负责频谱和波形特征；精确频率优先使用 TIM2 测频结果。
- 修改 FFT 点数时只修改 `APP_FFT_SIZE`，并确认 RAM 和处理时间满足要求。
- CubeMX 重新生成代码后，检查 `main.c` 的 USER CODE 区是否仍保留模块初始化。
