#include "MAX262.h"
#include <math.h>

extern TIM_HandleTypeDef htim2;

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ===========================================================================
 *  内部常量
 * =========================================================================== */

/* MAX262 最大 f_CLK (4MHz @ ±5V); MAX260 可达 7.5MHz */
#define MAX262_FCLK_MAX  4000000UL

/* 延时校准系数 — H7 @480MHz, 每条 NOP ≈ 2.08ns */
#define DELAY_CAL_OFFSET  2

/* ===========================================================================
 *  内部状态
 * =========================================================================== */

static uint32_t g_current_fclk = 0; /* 当前 f_CLK 设定值 */

/* ===========================================================================
 *  底层延时
 * =========================================================================== */

static void MAX262_Delay_NS(uint32_t ns_count) {
    uint32_t ticks = ns_count * DELAY_CAL_OFFSET;
    while (ticks--) {
        __NOP();
    }
}

/* ===========================================================================
 *  初始化
 * =========================================================================== */

void MAX262_Init(void) {
    MAX262_WR_SET(); /* WR 高电平 = 不选中芯片 */
}

/* ===========================================================================
 *  寄存器级写操作
 * =========================================================================== */

void MAX262_Write(uint8_t add, uint8_t dat2bit) {
    uint32_t odr_val;

    /* 1. 设置地址 A0-A3 和数据 D0-D1 */
    odr_val = GPIOC->ODR;
    odr_val &= ~0x003F;
    odr_val |= (add & 0x0F);
    odr_val |= ((dat2bit << 4) & 0x30);
    GPIOC->ODR = odr_val;

    /* 2. 数据建立时间 — MAX262 要求 tDS ≥ 50ns */
    MAX262_Delay_NS(100);

    /* 3. 写脉冲 — 数据在 WR 上升沿锁存 */
    MAX262_WR_RESET();
    MAX262_Delay_NS(300);   /* tWR ≥ 200ns */
    MAX262_WR_SET();
    MAX262_Delay_NS(50);    /* 数据保持 */
}

/* ===========================================================================
 *  F 字 / Q 字 写入辅助
 * =========================================================================== */

static void write_F_word(uint8_t base_addr, uint8_t f_word) {
    f_word &= 0x3F;
    MAX262_Write(base_addr + 0, f_word & 0x03);
    MAX262_Write(base_addr + 1, (f_word >> 2) & 0x03);
    MAX262_Write(base_addr + 2, (f_word >> 4) & 0x03);
}

void MAX262_SetAf(uint8_t datF) { write_F_word(1, datF); }
void MAX262_SetBf(uint8_t datF) { write_F_word(9, datF); }

static void write_Q_word(uint8_t base_addr, uint8_t q_word) {
    q_word &= 0x7F;
    MAX262_Write(base_addr + 0, q_word & 0x03);
    MAX262_Write(base_addr + 1, (q_word >> 2) & 0x03);
    MAX262_Write(base_addr + 2, (q_word >> 4) & 0x03);
    MAX262_Write(base_addr + 3, (q_word >> 6) & 0x01);
}

void MAX262_SetAQ(uint8_t datQ) { write_Q_word(4, datQ); }
void MAX262_SetBQ(uint8_t datQ) { write_Q_word(12, datQ); }

/* ===========================================================================
 *  频率计算引擎
 * =========================================================================== */

/**
 * @brief fc(截止频率) → f0(中心频率) 转换
 *
 * 二阶滤波器 -3dB 截止频率 fc 与中心频率 f0 的关系:
 *   x = 1 − 1/(2Q²),  ratio = √(x + √(x²+1))
 *   低通: f0 = fc / ratio  (fc < f0, 当 Q > 0.707)
 *   高通: f0 = fc × ratio  (fc > f0, 当 Q > 0.707)
 */
static float fc_to_f0(float fc, float Q, uint8_t passMode) {
    float x = 1.0f - 0.5f / (Q * Q);
    float ratio = sqrtf(x + sqrtf(x * x + 1.0f));
    return (passMode == MAX262_LOWPASS) ? (fc / ratio) : (fc * ratio);
}

/**
 * @brief 计算 f_CLK / f0 比值 K
 *
 * 模式 1/3/4: K = (π/2) × (26 + F)         → F=0 时 K = 13π ≈ 40.84
 * 模式 2:     K = (π/2) × (26 + F) / √2    → F=0 时 K = 13π/√2 ≈ 28.88
 */
static float calc_K(uint8_t workMode, uint8_t f_word) {
    float k = (M_PI / 2.0f) * (26.0f + (float)(f_word & 0x3F));
    if (workMode == MAX262_MODE_2) {
        k /= 1.41421356f; /* ÷ √2 */
    }
    return k;
}

/**
 * @brief 自动选择最优 F 字
 * 策略: 在不超过 max_clk 的前提下选最大 F (更高的 f_CLK → 更好的抗混叠)
 * @return F 字 (0~63), 若连 F=0 都超标则返回 0
 */
static uint8_t auto_select_F(float f0, uint8_t workMode, uint32_t max_clk) {
    uint8_t best_f = 0;
    for (uint8_t f = 0; f <= 63; f++) {
        if (calc_K(workMode, f) * f0 <= (float)max_clk) {
            best_f = f;
        } else {
            break; /* f 增大只会让 K 更大 */
        }
    }
    return best_f;
}

/**
 * @brief Q → 寄存器值 Qn (带安全限幅)
 *
 * 模式 2:   Q = 90.51 / (128 − Qn)  →  Qn = 128 − 90.51/Q
 * 其他模式: Q = 64    / (128 − Qn)  →  Qn = 128 − 64/Q
 */
static uint8_t Q_to_register(float Q, uint8_t workMode) {
    if (Q < MAX262_Q_MIN) Q = MAX262_Q_MIN;

    float qn_float = (workMode == MAX262_MODE_2)
        ? (128.0f - 90.51f / Q)
        : (128.0f - 64.0f  / Q);

    if (qn_float < 0.0f)   qn_float = 0.0f;
    if (qn_float > 127.0f) qn_float = 127.0f;

    return (uint8_t)(qn_float + 0.5f);
}

/* ===========================================================================
 *  核心: 写入 MAX262 寄存器 (模式 + F 字 + Q 值)
 * =========================================================================== */

/**
 * @brief 将模式和 f0/Q 写入 MAX262 指定通道, 返回所需 f_CLK
 */
static float config_channel(float f0, float Q, uint8_t workMode, uint8_t channel) {
    uint8_t f_word = auto_select_F(f0, workMode, MAX262_FCLK_MAX);
    uint8_t Qn     = Q_to_register(Q, workMode);

    if (channel == MAX262_CHANNEL_A) {
        MAX262_Write(0, workMode);
        MAX262_SetAf(f_word);
        MAX262_SetAQ(Qn);
    } else {
        MAX262_Write(8, workMode);
        MAX262_SetBf(f_word);
        MAX262_SetBQ(Qn);
    }

    return calc_K(workMode, f_word) * f0;
}

/* ===========================================================================
 *  进阶 API: 手动控制 (不自动设置 PWM)
 * =========================================================================== */

float MAX262_LHP_WorkFclk(float fc, float Q, uint8_t passMode, uint8_t workMode, uint8_t channel) {
    float f0 = fc_to_f0(fc, Q, passMode);

    /* 高通必须用 Mode 3 — 这是唯一有 HP 输出的模式 */
    if (passMode == MAX262_HIGHPASS) {
        workMode = MAX262_MODE_3;
    }

    return config_channel(f0, Q, workMode, channel);
}

float MAX262_BP_WorkFclk(float fh, float fl, uint8_t workMode, uint8_t channel) {
    float f0 = sqrtf(fh * fl);
    float Q  = f0 / (fh - fl);
    return config_channel(f0, Q, workMode, channel);
}

/* ===========================================================================
 *  PWM 时钟输出
 * =========================================================================== */

void MAX262_PWM_Set(uint32_t f_clk) {
    if (f_clk == 0) return;

    uint32_t timer_clk = 240000000UL;
    uint32_t psc, arr;

    if (f_clk < 4000) {
        psc = 240 - 1;              /* 预分频 → 1MHz */
        arr = 1000000UL / f_clk;
    } else if (f_clk > (timer_clk / 2)) {
        psc = 0;
        arr = 1;                    /* 上限 120MHz */
    } else {
        psc = 0;
        arr = timer_clk / f_clk;
    }

    if (arr > 0) arr -= 1;

    __HAL_TIM_SET_PRESCALER(&htim2, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (arr + 1) / 2);

    HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);

    g_current_fclk = f_clk;
}

uint32_t MAX262_GetClock(void) {
    return g_current_fclk;
}

/* ===========================================================================
 *  简易 API: 一键配置滤波器 + 自动设置时钟
 * =========================================================================== */

/**
 * @brief 低通滤波器 — Mode 1, LP 输出
 *
 * 内部流程:
 *   fc, Q → 计算 f0 (fc_to_f0, 低通转换)
 *   f0, Mode1 → 自动选 F 字 (auto_select_F)
 *   F 字, Mode1, Q → 写入 MAX262 寄存器
 *   f0, F 字 → 计算 f_CLK → 设置 TIM2 PWM
 */
void MAX262_LowPass(float fc, float Q, uint8_t channel) {
    float f0 = fc_to_f0(fc, Q, MAX262_LOWPASS);
    uint32_t f_clk = (uint32_t)config_channel(f0, Q, MAX262_MODE_1, channel);
    MAX262_PWM_Set(f_clk);
}

/**
 * @brief 高通滤波器 — Mode 3 (唯一支持 HP 输出)
 */
void MAX262_HighPass(float fc, float Q, uint8_t channel) {
    float f0 = fc_to_f0(fc, Q, MAX262_HIGHPASS);
    uint32_t f_clk = (uint32_t)config_channel(f0, Q, MAX262_MODE_3, channel);
    MAX262_PWM_Set(f_clk);
}

/**
 * @brief 带通滤波器 (按中心频率) — Mode 1, BP 输出
 *
 * f0 即增益峰值所在频率; 带宽 BW = f0/Q
 */
void MAX262_BandPass(float f0, float Q, uint8_t channel) {
    uint32_t f_clk = (uint32_t)config_channel(f0, Q, MAX262_MODE_1, channel);
    MAX262_PWM_Set(f_clk);
}

/**
 * @brief 带通滤波器 (按通带边缘) — Mode 1, BP 输出
 *
 * 由上下截止频率反推 f0 和 Q
 */
void MAX262_BandPassEdge(float fl, float fh, uint8_t channel) {
    float f0 = sqrtf(fh * fl);
    float Q  = f0 / (fh - fl);
    uint32_t f_clk = (uint32_t)config_channel(f0, Q, MAX262_MODE_1, channel);
    MAX262_PWM_Set(f_clk);
}
