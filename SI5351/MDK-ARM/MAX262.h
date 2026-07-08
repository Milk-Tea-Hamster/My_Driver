#ifndef __MAX262_H
#define __MAX262_H

#include "stm32h7xx_hal.h"

/*
 *  【硬件连接】
 *    PC0-PC3 → MAX262 A0-A3 (地址线)
 *    PC4-PC5 → MAX262 D0-D1 (数据线)
 *    PC6     → MAX262 /WR   (写选通, 低有效)
 *    PA0     → MAX262 CLK   (TIM2_CH1 PWM 输出作为 f_CLK)
 *
 *  【滤波器输出引脚 (参考 MAX262 数据手册)】
 *    Channel A: 低通 LP=19脚 | 带通 BP=20脚 | 高通 HP=3脚 (仅 Mode 3)
 *    Channel B: 低通 LP=18脚 | 带通 BP=17脚 | 高通 HP=14脚(仅 Mode 3)
 *
 *  【关键约束】
 *    f_CLK = K × f0, 其中 K 与模式及 F 字有关
 *    模式 1/3/4: K = (π/2)×(26+F)   → F=0 时 Kmin ≈ 40.84
 *    模式 2:     K = (π/2)×(26+F)/√2 → F=0 时 Kmin ≈ 28.88
 *    MAX262 最大 f_CLK ≈ 4MHz (@±5V), 因此 f0 上限约 100kHz (Mode1) / 140kHz (Mode2)
 *
 *  【使用示例】
 *    // 3kHz 巴特沃斯低通, 通道 A
 *    MAX262_LowPass(3000, 0.707, MAX262_CHANNEL_A);
 *
 *    // 10kHz 带通, Q=5, 通道 B
 *    MAX262_BandPass(10000, 5, MAX262_CHANNEL_B);
 *
 *    // 1kHz 高通, Q=1
 *    MAX262_HighPass(1000, 1, MAX262_CHANNEL_A);
 * ===========================================================================
 */

/* ===========================================================================
 *  第一部分: 宏定义 — 滤波器参数常量
 * =========================================================================== */

/* ---- 工作模式 (M1M0 寄存器值, 与数据手册一致) ---- */
#define MAX262_MODE_1     0   /* M1M0=00: LP/BP/Notch 输出, K 较大 */
#define MAX262_MODE_2     1   /* M1M0=01: LP/BP/Notch 输出, K 较小(K/√2), Q 公式不同 */
#define MAX262_MODE_3     2   /* M1M0=10: LP/BP/HP 输出 (唯一支持高通的模式) */
#define MAX262_MODE_4     3   /* M1M0=11: LP/BP/AP 输出 */

/* ---- 滤波器类型 ---- */
#define MAX262_LOWPASS    0
#define MAX262_HIGHPASS   1

/* ---- 通道选择 ---- */
#define MAX262_CHANNEL_A  0
#define MAX262_CHANNEL_B  1

/* ---- 硬件参数限制 ---- */
#define MAX262_F_WORD_MAX 63   /* F 字上限 (6 位) */
#define MAX262_F_WORD_MIN 0    /* F 字下限 */
#define MAX262_Q_MIN      0.6f /* Q 值安全下限, 防止寄存器值无符号下溢 */

/* ---- WR 引脚控制宏 — PC6, 低电平有效 ---- */
#define MAX262_WR_SET()     (GPIOC->BSRR = GPIO_PIN_6)            /* PC6 = 1, 不选中 */
#define MAX262_WR_RESET()   (GPIOC->BSRR = (uint32_t)GPIO_PIN_6 << 16U)  /* PC6 = 0, 写使能 */

/* ===========================================================================
 *  第二部分: 简易 API — 一键配置, 推荐日常使用
 * =========================================================================== */

/**
 * @brief 配置低通滤波器 (Mode 1, 自动设置 f_CLK)
 * @param fc       -3dB 截止频率 (Hz)
 *                 Q=0.707 时 fc=f0; Q 越大 fc 越偏离 f0
 * @param Q        品质因数
 *                 0.707 = 巴特沃斯响应 (通带最平坦, 推荐)
 *                 0.5~1 = 平缓过渡带
 *                 1~10  = 通带有 peaking, 过渡带变陡
 *                 >10   = 窄带/谐振, 趋近带通特性
 * @param channel  通道: MAX262_CHANNEL_A 或 MAX262_CHANNEL_B
 * @note   信号从对应通道的 LP 引脚输出 (A=19脚, B=18脚)
 * @note   内部使用 Mode 1, F 字自动选取最优值
 */
void MAX262_LowPass(float fc, float Q, uint8_t channel);

/**
 * @brief 配置高通滤波器 (Mode 3 — 唯一支持 HP 输出的模式)
 * @param fc       -3dB 截止频率 (Hz)
 * @param Q        品质因数 (含义同低通)
 * @param channel  通道
 * @note   信号从对应通道的 HP 引脚输出 (A=3脚, B=14脚)
 */
void MAX262_HighPass(float fc, float Q, uint8_t channel);

/**
 * @brief 配置带通滤波器 — 按中心频率 (Mode 1)
 * @param f0       中心频率 (Hz), 即增益峰值所在频率
 * @param Q        品质因数 = f0 / BW
 *                 带宽 BW = f0 / Q, 即 -3dB 通带宽度
 *                 Q=1  → BW = f0   (宽带宽)
 *                 Q=10 → BW = f0/10
 *                 Q=100→ BW = f0/100 (窄带高选择性)
 * @param channel  通道
 * @note   信号从对应通道的 BP 引脚输出 (A=20脚, B=17脚)
 */
void MAX262_BandPass(float f0, float Q, uint8_t channel);

/**
 * @brief 配置带通滤波器 — 按通带边缘 (Mode 1)
 * @param fl       下截止频率 (Hz), -3dB 点
 * @param fh       上截止频率 (Hz), -3dB 点
 *                 f0 = sqrt(fl × fh), Q = f0 / (fh - fl)
 * @param channel  通道
 * @note   信号从对应通道的 BP 引脚输出 (A=20脚, B=17脚)
 */
void MAX262_BandPassEdge(float fl, float fh, uint8_t channel);

/**
 * @brief 获取当前 f_CLK 设置值
 * @return f_CLK 频率 (Hz), 即 TIM2 CH1 当前 PWM 输出频率
 */
uint32_t MAX262_GetClock(void);

/* ===========================================================================
 *  第三部分: 进阶 API — 手动控制, 用于非标准场景
 * =========================================================================== */

/**
 * @brief MAX262 软件初始化 (GPIO 由 MX_GPIO_Init 配置, 此处确保 WR 闲置为高)
 */
void MAX262_Init(void);

/**
 * @brief 直接设置 TIM2 PWM 频率作为 f_CLK
 * @param f_clk 目标频率 (Hz), 有效范围 ≈ 1Hz ~ 120MHz
 */
void MAX262_PWM_Set(uint32_t f_clk);

/**
 * @brief 低通/高通手动配置 — 可指定工作模式, 返回所需 f_CLK (不自动设置 PWM)
 * @param fc       截止频率 (Hz)
 * @param Q        品质因数
 * @param passMode MAX262_LOWPASS 或 MAX262_HIGHPASS
 * @param workMode 工作模式 MODE_1~4 (高通时自动覆写为 MODE_3)
 * @param channel  通道
 * @return         所需的 f_CLK (Hz), 调用者自行调用 MAX262_PWM_Set
 */
float MAX262_LHP_WorkFclk(float fc, float Q, uint8_t passMode, uint8_t workMode, uint8_t channel);

/**
 * @brief 带通手动配置 — 可指定工作模式, 返回所需 f_CLK (不自动设置 PWM)
 * @param fh       上截止频率 (Hz)
 * @param fl       下截止频率 (Hz)
 * @param workMode 工作模式 (推荐 MODE_1)
 * @param channel  通道
 * @return         所需的 f_CLK (Hz)
 */
float MAX262_BP_WorkFclk(float fh, float fl, uint8_t workMode, uint8_t channel);

/* ===========================================================================
 *  第四部分: 寄存器级 API — 直接读写 MAX262 内部寄存器
 * =========================================================================== */

/**
 * @brief 向 MAX262 指定地址写入 2 位数据
 * @param add     4 位寄存器地址 (0~15, 见数据手册 Table 2)
 * @param dat2bit 2 位数据 (写入 D0,D1)
 */
void MAX262_Write(uint8_t add, uint8_t dat2bit);

/**
 * @brief 设置通道 A 的 F 字 (6 位频率控制字, 0~63)
 */
void MAX262_SetAf(uint8_t datF);

/**
 * @brief 设置通道 A 的 Q 值 (7 位品质因数控制字, 0~127)
 */
void MAX262_SetAQ(uint8_t datQ);

/**
 * @brief 设置通道 B 的 F 字
 */
void MAX262_SetBf(uint8_t datF);

/**
 * @brief 设置通道 B 的 Q 值
 */
void MAX262_SetBQ(uint8_t datQ);

#endif
