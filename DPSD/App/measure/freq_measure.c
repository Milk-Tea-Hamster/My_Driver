/**
 * @file    freq_measure.c
 * @brief   频率测量实现 —— 基于 STM32H743 TIM2 纯硬件双模式
 * @note    引脚: PA5 (TIM2_CH1)
 *          低频: 测周法 (TIM2 输入捕获 + 从模式复位)
 *          高频: 测频法 (TIM2 外部时钟模式1 ETR计数，零中断)
 */

#include "freq_measure.h"
#include "tim.h"
#include "gpio.h"

#define SWITCH_HIGH_HZ      50000.0f
#define SWITCH_LOW_HZ       40000.0f
#define GATE_TIME_MS        100U
#define SIGNAL_TIMEOUT_MS   500U

static uint32_t freq_get_apb1_timer_clock_hz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t apb1_div = READ_BIT(RCC->D2CFGR, RCC_D2CFGR_D2PPRE1);

    if (READ_BIT(RCC->CFGR, RCC_CFGR_TIMPRE) == 0U) {
        return (apb1_div == RCC_D2CFGR_D2PPRE1_DIV1)
                   ? pclk1
                   : (pclk1 * 2U);
    }

    if ((apb1_div == RCC_D2CFGR_D2PPRE1_DIV1) ||
        (apb1_div == RCC_D2CFGR_D2PPRE1_DIV2) ||
        (apb1_div == RCC_D2CFGR_D2PPRE1_DIV4)) {
        return HAL_RCC_GetHCLKFreq();
    }

    return pclk1 * 4U;
}

static void freq_hw_switch(FreqMeasure *self, uint8_t target_mode) {
    GPIO_InitTypeDef gpio = {0};
    self->first_frame = 1;

    HAL_TIM_Base_Stop(self->htim);
    HAL_TIM_IC_Stop(self->htim, TIM_CHANNEL_1);
    HAL_TIM_Base_DeInit(self->htim);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin       = GPIO_PIN_5;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLDOWN;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);

    if (target_mode == FMODE_PERIOD) {
        MX_TIM2_Init();
        __HAL_TIM_CLEAR_FLAG(self->htim, TIM_FLAG_CC1 | TIM_FLAG_UPDATE);
        HAL_TIM_Base_Start(self->htim);
        HAL_TIM_IC_Start(self->htim, TIM_CHANNEL_1);
    } else {
        self->htim->Instance = TIM2;
        self->htim->Init.Prescaler = 0;
        self->htim->Init.CounterMode = TIM_COUNTERMODE_UP;
        self->htim->Init.Period = 0xFFFFFFFF;
        self->htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        HAL_TIM_Base_Init(self->htim);

        TIM_SlaveConfigTypeDef sSlaveConfig = {0};
        sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
        sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
        sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
        sSlaveConfig.TriggerFilter = 0;
        HAL_TIM_SlaveConfigSynchro(self->htim, &sSlaveConfig);

        __HAL_TIM_SET_COUNTER(self->htim, 0);
        HAL_TIM_Base_Start(self->htim);
    }
}

static float freq_read_period(FreqMeasure *self) {
    if (__HAL_TIM_GET_FLAG(self->htim, TIM_FLAG_CC1) != RESET) {
        uint32_t cap = HAL_TIM_ReadCapturedValue(self->htim, TIM_CHANNEL_1);
        __HAL_TIM_CLEAR_FLAG(self->htim, TIM_FLAG_CC1);
        __HAL_TIM_CLEAR_FLAG(self->htim, TIM_FLAG_UPDATE);

        if (self->first_frame) {
            self->first_frame = 0;
            return -1.0f;
        }

        if (cap == 0) return 0.0f;
        return (float)((double)self->tick_freq / (double)(cap + 1));
    }
    return -1.0f;
}

/* ================= 公共 API ================= */

void FreqMeasure_Init(FreqMeasure *self, TIM_HandleTypeDef *htim) {
    self->htim            = htim;
    self->tick_freq       = (float)freq_get_apb1_timer_clock_hz();
    self->switch_high_thr = SWITCH_HIGH_HZ;
    self->switch_low_thr  = SWITCH_LOW_HZ;
    self->gate_time_ms    = GATE_TIME_MS;
    self->signal_timeout_ms = SIGNAL_TIMEOUT_MS;
    self->mode            = FMODE_PERIOD;
    self->gate_start_ms   = 0U;
    self->last_signal_ms  = HAL_GetTick();
    self->measuring       = 0U;
    self->first_frame     = 1U;
    freq_hw_switch(self, FMODE_PERIOD);
}

void FreqMeasure_Process(FreqMeasure *self, Wave_Struct *wave) {
    if (self->mode == FMODE_PERIOD) {
        float f = freq_read_period(self);
        if (f >= 0.0f) {
            wave->Freq = f;
            wave->FreqValid = (f > 0.0f) ? 1U : 0U;
            self->last_signal_ms = HAL_GetTick();

            if (f > self->switch_high_thr) {
                self->mode = FMODE_COUNT;
                freq_hw_switch(self, FMODE_COUNT);
                self->measuring = 0;
            }
        } else if ((HAL_GetTick() - self->last_signal_ms) >=
                   self->signal_timeout_ms) {
            wave->Freq = 0.0f;
            wave->FreqValid = 0U;
        }
    } else {
        if (!self->measuring) {
            __HAL_TIM_SET_COUNTER(self->htim, 0);
            self->gate_start_ms = HAL_GetTick();
            self->measuring     = 1;
        } else {
            uint32_t current_ms = HAL_GetTick();
            uint32_t delta_ms   = current_ms - self->gate_start_ms;

            if (delta_ms >= self->gate_time_ms) {
                uint32_t total_pulses = __HAL_TIM_GET_COUNTER(self->htim);

                if (delta_ms > 0) {
                    wave->Freq = (float)(((double)total_pulses * 1000.0) / (double)delta_ms);
                    wave->FreqValid = (total_pulses > 0U) ? 1U : 0U;
                    if (total_pulses > 0U) {
                        self->last_signal_ms = current_ms;
                    }
                }

                self->measuring = 0;

                if (wave->Freq < self->switch_low_thr) {
                    self->mode = FMODE_PERIOD;
                    freq_hw_switch(self, FMODE_PERIOD);
                }
            }
        }
    }
}
