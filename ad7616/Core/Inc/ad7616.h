#ifndef __AD7616_H__
#define __AD7616_H__

#include "stm32h7xx_hal.h"

/* ========== AD7616 数据总线 — PE0~PE15 ========== */
#define AD7616_DATA_PORT            GPIOE
#define AD7616_DATA_PORT_CLK()      __HAL_RCC_GPIOE_CLK_ENABLE()

/* ========== AD7616 控制引脚 ========== */
/* CS   — PC4 */
#define AD7616_CS_PIN       GPIO_PIN_4
#define AD7616_CS_PORT      GPIOC
/* RD   — PC5 */
#define AD7616_RD_PIN       GPIO_PIN_5
#define AD7616_RD_PORT      GPIOC
/* WR   — PB0 */
#define AD7616_WR_PIN       GPIO_PIN_0
#define AD7616_WR_PORT      GPIOB
/* BUSY — PB1 (EXTI1) */
#define AD7616_BUSY_PIN     GPIO_PIN_1
#define AD7616_BUSY_PORT    GPIOB
/* RESET — PB2 */
#define AD7616_RST_PIN      GPIO_PIN_2
#define AD7616_RST_PORT     GPIOB

/* ========== 快捷宏 ========== */
#define AD7616_CS_LOW()     HAL_GPIO_WritePin(AD7616_CS_PORT, AD7616_CS_PIN, GPIO_PIN_RESET)
#define AD7616_CS_HIGH()    HAL_GPIO_WritePin(AD7616_CS_PORT, AD7616_CS_PIN, GPIO_PIN_SET)
#define AD7616_RD_LOW()     HAL_GPIO_WritePin(AD7616_RD_PORT, AD7616_RD_PIN, GPIO_PIN_RESET)
#define AD7616_RD_HIGH()    HAL_GPIO_WritePin(AD7616_RD_PORT, AD7616_RD_PIN, GPIO_PIN_SET)
#define AD7616_WR_LOW()     HAL_GPIO_WritePin(AD7616_WR_PORT, AD7616_WR_PIN, GPIO_PIN_RESET)
#define AD7616_WR_HIGH()    HAL_GPIO_WritePin(AD7616_WR_PORT, AD7616_WR_PIN, GPIO_PIN_SET)
#define AD7616_RST_LOW()    HAL_GPIO_WritePin(AD7616_RST_PORT, AD7616_RST_PIN, GPIO_PIN_RESET)
#define AD7616_RST_HIGH()   HAL_GPIO_WritePin(AD7616_RST_PORT, AD7616_RST_PIN, GPIO_PIN_SET)
#define AD7616_BUSY_READ()  HAL_GPIO_ReadPin(AD7616_BUSY_PORT, AD7616_BUSY_PIN)

/* ========== 量程 ========== */
typedef enum {
    AD7616_RANGE_10V  = 0x00,   /* 硬件模式编码: RNG1=1, RNG0=1 */
    AD7616_RANGE_2_5V = 0x01,   /* RNG1=0, RNG0=1 */
    AD7616_RANGE_5V   = 0x02,   /* RNG1=1, RNG0=0 */
} AD7616_Range;

/* ========== 寄存器地址 ========== */
#define AD7616_REG_CONFIG      0x02
#define AD7616_REG_CHANNEL     0x03
#define AD7616_REG_RANGE_A1    0x04
#define AD7616_REG_RANGE_A2    0x05
#define AD7616_REG_RANGE_B1    0x06
#define AD7616_REG_RANGE_B2    0x07
#define AD7616_REG_SEQ_START   0x20

/* 配置寄存器位 */
#define AD7616_CFG_BURSTEN     (1 << 6)
#define AD7616_CFG_SEQEN       (1 << 5)
#define AD7616_CFG_OS(x)       (((x) & 0x07) << 2)
#define AD7616_CFG_STATUSEN    (1 << 1)
#define AD7616_CFG_CRCEN       (1 << 0)

/* ========== 通道选择 ========== */
/* 手动通道选择寄存器 (地址 0x03) 的 CHA/CHB 编码: 0~7 对应 V0~V7, 0xB=自检 */
#define AD7616_CH_SELF_TEST    0x0B

/* ========== 采样缓冲区 ========== */
#define AD7616_CH_PAIRS         8
#define AD7616_BURST_WORDS      16

typedef struct {
    int16_t chan_a[AD7616_CH_PAIRS];
    int16_t chan_b[AD7616_CH_PAIRS];
    uint8_t fresh;
} AD7616_Data;

/* ========== API ========== */

void AD7616_Init(void);
void AD7616_InitControlGPIO(void);
void AD7616_SetGlobalRange(AD7616_Range range);
void AD7616_SelectChannelPair(uint8_t pair);
void AD7616_ConfigBurstSequence(void);
void AD7616_StartConversion(void);
void AD7616_StopConversion(void);

/* 单对通道读取（A+B, 阻塞等待 BUSY 下降后读取）*/
void AD7616_ReadPair(int16_t *data_a, int16_t *data_b);

/* 突发模式读取 16 字 */
void AD7616_ReadBurst(int16_t *buf, uint16_t len);

/* 通信自检：返回 1=通过, 0=失败 */
uint8_t AD7616_SelfTest(void);

/* BUSY 下降沿回调 — 放入 HAL_GPIO_EXTI_Callback() */
void AD7616_BUSY_Callback(void);

/* 获取最新采样数据 */
AD7616_Data *AD7616_GetData(void);

/* 原始码 → 电压 (mV) */
float AD7616_CodeToVoltage(int16_t code, AD7616_Range range);

#endif /* __AD7616_H__ */
