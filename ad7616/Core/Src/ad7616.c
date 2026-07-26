#include "ad7616.h"
#include "tim.h"

/* ================================================================
 * AD7616 驱动 — GPIO + TIM3 + DMA 方式
 *
 * 架构:
 *   TIM3 CH2 (PA7) = CONVST 1MHz PWM 连续输出
 *   TIM3 CH3 = DMA触发 (错相), 保留用于突发模式
 *   PE0~PE15 = 16位并行数据总线
 *   PC4=CS, PC5=RD, PB0=WR, PB1=BUSY, PB2=RESET
 *
 * 数据读取: BUSY下降沿 → 软件位带操作 RD + 直接读 GPIOE->IDR
 * 寄存器写入: 切换 GPIOE 为输出 → WR脉冲锁存 → 恢复 GPIOE 为输入
 * ================================================================ */

/* ---- 快速 GPIO 操作 (绕过 HAL, 用 BSRR 原子操作) ---- */
#define CS_LOW_FAST()   (AD7616_CS_PORT->BSRR = (uint32_t)AD7616_CS_PIN << 16U)
#define CS_HIGH_FAST()  (AD7616_CS_PORT->BSRR = AD7616_CS_PIN)
#define RD_LOW_FAST()   (AD7616_RD_PORT->BSRR = (uint32_t)AD7616_RD_PIN << 16U)
#define RD_HIGH_FAST()  (AD7616_RD_PORT->BSRR = AD7616_RD_PIN)
#define WR_LOW_FAST()   (AD7616_WR_PORT->BSRR = (uint32_t)AD7616_WR_PIN << 16U)
#define WR_HIGH_FAST()  (AD7616_WR_PORT->BSRR = AD7616_WR_PIN)

/* ---- 内部变量 ---- */
static AD7616_Range  g_range = AD7616_RANGE_10V;
static AD7616_Data   g_data;
static volatile uint32_t g_sample_count;

/* DMA 缓冲区: 放 AXI SRAM (0x24000000, MPU 已设 Non-Cacheable) */
static int16_t g_dma_buf[AD7616_BURST_WORDS];

/* ---- 内部函数声明 ---- */
static void AD7616_WriteReg(uint8_t addr, uint16_t data);
static void AD7616_SetDataGPIO_Output(void);
static void AD7616_SetDataGPIO_Input(void);
static void AD7616_DelayCycles(uint32_t cycles);

/* ---- 对外引用 ---- */
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim3_ch3;

/* ================================================================
 * 控制 GPIO 初始化 — AD7616 专用引脚, 不依赖 CubeMX gpio.c
 * ================================================================ */
void AD7616_InitControlGPIO(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PC4=CS, PC5=RD: 推挽输出, 初始高电平 */
    AD7616_CS_HIGH();
    AD7616_RD_HIGH();
    gpio.Pin   = AD7616_CS_PIN | AD7616_RD_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(AD7616_CS_PORT, &gpio);

    /* PB0=WR: 推挽输出, 初始高电平 */
    AD7616_WR_HIGH();
    gpio.Pin   = AD7616_WR_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(AD7616_WR_PORT, &gpio);

    /* PB2=RESET: 推挽输出, 初始高电平 */
    AD7616_RST_HIGH();
    gpio.Pin   = AD7616_RST_PIN;
    HAL_GPIO_Init(AD7616_RST_PORT, &gpio);

    /* PB1=BUSY: 浮空输入, EXTI1 下降沿 */
    gpio.Pin  = AD7616_BUSY_PIN;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(AD7616_BUSY_PORT, &gpio);

    /* EXTI1 中断优先级 */
    HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    /* PE0~PE15: 浮空输入 (数据总线默认输入) */
    AD7616_SetDataGPIO_Input();
}

/* ================================================================
 * AD7616 完整初始化
 * ================================================================ */
void AD7616_Init(void)
{
    /* 1. 控制引脚初始化 */
    AD7616_InitControlGPIO();

    /* 2. 硬件复位 */
    AD7616_RST_LOW();
    HAL_Delay(1);                       /* ≥1.2µs, 工程取 1ms */
    AD7616_RST_HIGH();
    HAL_Delay(20);                      /* 等待内部基准稳定 ≥15ms */

    /* 3. 软件模式: 配置全局量程寄存器 */
    AD7616_SetGlobalRange(g_range);

    /* 4. 配置寄存器: 无过采样, 无序列, 无突发, 无 CRC, 无状态字 */
    AD7616_WriteReg(AD7616_REG_CONFIG, 0x0000);
}

/* ================================================================
 * 设置全局量程 (所有 16 通道同一量程)
 * 软件模式量程编码: 00=±10V, 01=±2.5V, 10=±5V
 * ================================================================ */
void AD7616_SetGlobalRange(AD7616_Range range)
{
    uint16_t data = 0;
    g_range = range;

    /* 每通道 2 bit, 4 通道拼成 1 字节: 00=pair3, 01=pair2, 10=pair1, 11=pair0 */
    for (int i = 0; i < 4; i++) {
        data |= (range & 0x03) << (i * 2);
    }

    AD7616_WriteReg(AD7616_REG_RANGE_A1, data);  /* V0A~V3A */
    AD7616_WriteReg(AD7616_REG_RANGE_A2, data);  /* V4A~V7A */
    AD7616_WriteReg(AD7616_REG_RANGE_B1, data);  /* V0B~V3B */
    AD7616_WriteReg(AD7616_REG_RANGE_B2, data);  /* V4B~V7B */
}

/* ================================================================
 * 选择单通道对 (手动模式, 非序列)
 * pair: 0~7 对应 V0~V7
 * ================================================================ */
void AD7616_SelectChannelPair(uint8_t pair)
{
    uint16_t ch = (pair & 0x07);
    AD7616_WriteReg(AD7616_REG_CHANNEL, (ch << 4) | ch);
}

/* ================================================================
 * 配置 8 层序列器 + 突发模式
 * 一个 CONVST 完成 8 对通道, BUSY 下降后连续读取 16 字
 * ================================================================ */
void AD7616_ConfigBurstSequence(void)
{
    /* 序列器堆栈: 地址 0x20~0x27, 最后一层设 SSREN=1 */
    for (int i = 0; i < 8; i++) {
        uint16_t data = (i << 4) | i;   /* CHB=i, CHA=i */
        if (i == 7) {
            data |= (1 << 8);            /* SSREN: 序列结束 */
        }
        AD7616_WriteReg(AD7616_REG_SEQ_START + i, data);
    }

    /* 使能序列器和突发模式 */
    AD7616_WriteReg(AD7616_REG_CONFIG,
                    AD7616_CFG_SEQEN | AD7616_CFG_BURSTEN);
}

/* ================================================================
 * 启动/停止 CONVST (TIM3 CH2 PWM → PA7)
 * ================================================================ */
void AD7616_StartConversion(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void AD7616_StopConversion(void)
{
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
}

/* ================================================================
 * 单对通道读取 (A+B 并行, 快速寄存器操作)
 *
 * 时序 (AD7616 并行模式):
 *   CS↓
 *   RD↓ → 等 ≥30ns → 读 GPIOE->IDR → RD↑ (A 通道)
 *   等 ≥10ns
 *   RD↓ → 等 ≥30ns → 读 GPIOE->IDR → RD↑ (B 通道)
 *   CS↑
 *
 * 在 480MHz Cortex-M7 上每个 NOP ≈ 2ns, 使用 ~18 个 NOP ≈ 36ns
 * ================================================================ */
void AD7616_ReadPair(int16_t *data_a, int16_t *data_b)
{
    uint16_t raw_a, raw_b;
    uint32_t port_val __attribute__((unused));

    /* 等待 BUSY 下降 */
    while (AD7616_BUSY_READ() != 0) {
        ;
    }

    CS_LOW_FAST();

    /* ---- 读 A 通道 ---- */
    RD_LOW_FAST();
    /* 等待数据有效 ≥30ns, ~18 NOPs ≈ 36ns */
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    raw_a = AD7616_DATA_PORT->IDR & 0xFFFF;
    RD_HIGH_FAST();

    /* RD 高电平 ≥10ns, ~6 NOPs ≈ 12ns */
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();

    /* ---- 读 B 通道 ---- */
    RD_LOW_FAST();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    raw_b = AD7616_DATA_PORT->IDR & 0xFFFF;
    RD_HIGH_FAST();

    CS_HIGH_FAST();

    *data_a = (int16_t)raw_a;
    *data_b = (int16_t)raw_b;

    (void)port_val;
}

/* ================================================================
 * 突发读取: DMA 方式读取 len 个字
 * 使用 TIM3 CH3 DMA (DMA1_Stream4), 源 = GPIOE->IDR
 * ================================================================ */
void AD7616_ReadBurst(int16_t *buf, uint16_t len)
{
    if (len > AD7616_BURST_WORDS) len = AD7616_BURST_WORDS;

    /* 等待 BUSY 下降 */
    while (AD7616_BUSY_READ() != 0) {
        ;
    }

    CS_LOW_FAST();

    /* 重新配置 DMA: 源=GPIOE->IDR (固定), 目标=buf (递增) */
    DMA_Stream_TypeDef *dma = (DMA_Stream_TypeDef *)hdma_tim3_ch3.Instance;
    __HAL_DMA_DISABLE(&hdma_tim3_ch3);
    dma->PAR  = (uint32_t)&(AD7616_DATA_PORT->IDR);
    dma->M0AR = (uint32_t)buf;
    dma->NDTR = len;
    dma->CR  |= DMA_SxCR_EN;
    /* DMA 由 TIM3 CH3 比较事件触发 (已在 CubeMX 配置 DMAMUX) */

    /* 等待 DMA 完成 (轮询或中断) */
    uint32_t timeout = 100000;
    while ((dma->NDTR > 0) && (--timeout > 0)) {
        ;
    }

    __HAL_DMA_DISABLE(&hdma_tim3_ch3);
    CS_HIGH_FAST();
}

/* ================================================================
 * 通信自检
 * 设置 CHA=CHB=0xB → 转换后应读到 A=0xAAAA, B=0x5555
 * 返回: 1=通过, 0=失败
 * ================================================================ */
uint8_t AD7616_SelfTest(void)
{
    int16_t a, b;
    uint8_t pass = 1;

    /* 配置自检通道 */
    uint16_t ch = ((uint16_t)AD7616_CH_SELF_TEST << 4) | AD7616_CH_SELF_TEST;
    AD7616_WriteReg(AD7616_REG_CHANNEL, ch);

    /* 发送一次 CONVST 脉冲 */
    AD7616_StopConversion();
    HAL_Delay(1);

    /* 手动触发一次转换: 软件翻转 CONVST 对应的 GPIO */
    /* PA7 连到 CONVST, 临时切为 GPIO 翻转 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_OUTPUT_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);   /* CONVST ↑ */

    /* 读取 */
    AD7616_ReadPair(&a, &b);

    /* 恢复 TIM3 CH2 复用功能 */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* 校验 */
    if (a != (int16_t)0xAAAA || b != (int16_t)0x5555) {
        pass = 0;
    }

    /* 恢复正常通道 */
    AD7616_SelectChannelPair(0);
    AD7616_StartConversion();

    return pass;
}

/* ================================================================
 * BUSY 下降沿回调 — 在 HAL_GPIO_EXTI_Callback() 中调用
 *
 * 此处实现双缓冲采样: 每次 BUSY 下降读一对 A/B, 交替存入
 * ping/pong 缓冲区, 供上层 FFT/RMS 处理
 * ================================================================ */
void AD7616_BUSY_Callback(void)
{
    int16_t a, b;
    /* 快速读取 (不等待 BUSY, 因为已在 ISR 上下文中确认 BUSY 已下降) */

    CS_LOW_FAST();

    /* A 通道 */
    RD_LOW_FAST();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    a = AD7616_DATA_PORT->IDR & 0xFFFF;
    RD_HIGH_FAST();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();

    /* B 通道 */
    RD_LOW_FAST();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    b = AD7616_DATA_PORT->IDR & 0xFFFF;
    RD_HIGH_FAST();

    CS_HIGH_FAST();

    /* 存入 ping-pong 缓冲区 (以通道 0 为例) */
    g_data.chan_a[0] = (int16_t)a;
    g_data.chan_b[0] = (int16_t)b;
    g_data.fresh = 1;

    g_sample_count++;
}

/* ================================================================
 * 获取最新采样数据
 * ================================================================ */
AD7616_Data *AD7616_GetData(void)
{
    if (g_data.fresh) {
        g_data.fresh = 0;
        return &g_data;
    }
    return NULL;
}

/* ================================================================
 * 原始补码 → 电压 (mV)
 * AD7616 16位补码: 0x7FFF ≈ +FS, 0x8000 ≈ -FS
 * ================================================================ */
float AD7616_CodeToVoltage(int16_t code, AD7616_Range range)
{
    float fs;  /* 满量程电压 */
    switch (range) {
        case AD7616_RANGE_2_5V: fs = 2.5f;  break;
        case AD7616_RANGE_5V:   fs = 5.0f;  break;
        case AD7616_RANGE_10V:
        default:                fs = 10.0f; break;
    }
    return (float)code * fs / 32768.0f * 1000.0f;  /* mV */
}

/* ================================================================
 * 内部: 写寄存器 (软件并行模式)
 *
 * 命令格式: D15=1(写), D14~D9=地址, D8~D0=数据
 * 时序: GPIOE 切输出 → CS↓ → WR↓ → WR↑(锁存) → CS↑ → GPIOE 切输入
 * ================================================================ */
static void AD7616_WriteReg(uint8_t addr, uint16_t data)
{
    uint16_t cmd = 0x8000 | ((uint16_t)(addr & 0x3F) << 9) | (data & 0x01FF);

    AD7616_SetDataGPIO_Output();

    AD7616_CS_LOW();
    AD7616_DATA_PORT->ODR = cmd;

    /* WR 脉冲: 低→高 锁存数据 */
    WR_LOW_FAST();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();   /* ≥10ns */
    WR_HIGH_FAST();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();   /* ≥10ns */

    AD7616_CS_HIGH();

    AD7616_SetDataGPIO_Input();
}

/* ================================================================
 * 内部: 切换 PE0~PE15 为推挽输出
 * ================================================================ */
static void AD7616_SetDataGPIO_Output(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_All;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(AD7616_DATA_PORT, &gpio);
}

/* ================================================================
 * 内部: 切换 PE0~PE15 为浮空输入
 * ================================================================ */
static void AD7616_SetDataGPIO_Input(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_All;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(AD7616_DATA_PORT, &gpio);
}

/* ================================================================
 * 内部: 粗略周期延时 (用于非关键路径)
 * ================================================================ */
static void AD7616_DelayCycles(uint32_t cycles)
{
    for (volatile uint32_t i = 0; i < cycles; i++) {
        __NOP();
    }
}
