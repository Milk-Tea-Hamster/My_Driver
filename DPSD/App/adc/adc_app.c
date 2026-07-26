#include "adc_app.h"

#include "adc.h"
#include "tim.h"
#include "dpsd.h"
#include <string.h>

#define HIST_BINS    512U
#define BIN_WIDTH    (ADC_CODE_NUM / HIST_BINS)

__attribute__((section(".bss.AXI_SRAM"), aligned(32)))
uint16_t adc_buffer[ADC_LEN];

adc_waveform_info_t ADC_info;
volatile uint8_t adc_con_flag = 0U;

dpsd_result_t g_dpsd_result;

static uint32_t s_histogram[HIST_BINS];

static float adc_to_v(uint16_t value)
{
    return (float)value * ADC_VREF / (float)ADC_MAX_CODE;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc->Instance == ADC1) && (adc_con_flag == 0U)) {
        (void)HAL_TIM_Base_Stop(&htim6);
        adc_con_flag = 1U;
    }
}//ADC中断回调

void adc_get_vpp(adc_waveform_info_t *info,
                 const uint16_t *data,
                 uint32_t len)
{
    uint16_t vmax = 0U;
    uint16_t vmin = (uint16_t)ADC_MAX_CODE;
    uint64_t sum = 0U;

    if ((info == NULL) || (data == NULL) || (len == 0U)) {
        return;
    }

    for (uint32_t i = 0U; i < len; ++i) {
        const uint16_t value = data[i];

        if (value > vmax) {
            vmax = value;
        }
        if (value < vmin) {
            vmin = value;
        }
        sum += value;
    }

    info->max_adc = vmax;
    info->min_adc = vmin;
    info->vmax = adc_to_v(vmax);
    info->vmin = adc_to_v(vmin);
    info->vpp = info->vmax - info->vmin;
    info->vavg = ((float)sum / (float)len) *
                 ADC_VREF / (float)ADC_MAX_CODE;
}

void adc_get_vpp_robust(adc_waveform_info_t *info,
                        const uint16_t *data,
                        uint32_t len,
                        float discard_pct)
{
    uint64_t sum = 0U;
    uint32_t discard_num;
    uint32_t cumulative;
    uint16_t low_value = 0U;
    uint16_t high_value = (uint16_t)ADC_MAX_CODE;

    if ((info == NULL) || (data == NULL) || (len == 0U)) {
        return;
    }
    if (discard_pct <= 0.0f) {
        adc_get_vpp(info, data, len);
        return;
    }
    if (discard_pct >= 50.0f) {
        discard_pct = 49.0f;
    }

    memset(s_histogram, 0, sizeof(s_histogram));

    for (uint32_t i = 0U; i < len; ++i) {
        uint32_t bin = data[i] / BIN_WIDTH;

        if (bin >= HIST_BINS) {
            bin = HIST_BINS - 1U;
        }
        ++s_histogram[bin];
        sum += data[i];
    }

    discard_num = (uint32_t)((float)len * discard_pct / 100.0f);
    if (discard_num == 0U) {
        discard_num = 1U;
    }

    cumulative = 0U;
    for (uint32_t i = 0U; i < HIST_BINS; ++i) {
        const uint32_t previous = cumulative;
        cumulative += s_histogram[i];

        if (cumulative >= discard_num) {
            const uint32_t count =
                (s_histogram[i] == 0U) ? 1U : s_histogram[i];
            const uint32_t offset = discard_num - previous;

            low_value = (uint16_t)(
                i * BIN_WIDTH + offset * BIN_WIDTH / count);
            break;
        }
    }

    cumulative = 0U;
    for (uint32_t i = HIST_BINS; i-- > 0U;) {
        const uint32_t previous = cumulative;
        cumulative += s_histogram[i];

        if (cumulative >= discard_num) {
            const uint32_t count =
                (s_histogram[i] == 0U) ? 1U : s_histogram[i];
            const uint32_t offset = discard_num - previous;
            uint32_t value =
                (i + 1U) * BIN_WIDTH - offset * BIN_WIDTH / count;

            if (value > ADC_MAX_CODE) {
                value = ADC_MAX_CODE;
            }
            high_value = (uint16_t)value;
            break;
        }
    }

    info->max_adc = high_value;
    info->min_adc = low_value;
    info->vmax = adc_to_v(high_value);
    info->vmin = adc_to_v(low_value);
    info->vpp = info->vmax - info->vmin;
    info->vavg = ((float)sum / (float)len) *
                 ADC_VREF / (float)ADC_MAX_CODE;
}

void adc_get_vpp_auto(adc_waveform_info_t *info,
                      const uint16_t *data,
                      uint32_t len)
{
    adc_waveform_info_t basic;
    adc_waveform_info_t robust;

    if ((info == NULL) || (data == NULL) || (len == 0U)) {
        return;
    }

    adc_get_vpp(&basic, data, len);
    adc_get_vpp_robust(&robust, data, len, 3.0f);

    if (basic.vpp > robust.vpp * 1.08f) {
        *info = robust;
    } else {
        *info = basic;
    }
}

void adc_proc(void)
{
    if (adc_con_flag != 1U) {
        return;
    }
    adc_con_flag = 0U;
    HAL_ADC_Stop_DMA(&hadc1);
    SCB_InvalidateDCache_by_Addr( (uint32_t *)adc_buffer,sizeof(adc_buffer));
    adc_get_vpp_auto(&ADC_info, adc_buffer, ADC_LEN);//自动适应普通峰峰值与毛刺峰峰值算法
    dpsd_process(adc_buffer, &g_dpsd_result);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)adc_buffer,sizeof(adc_buffer));
    HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_buffer,ADC_LEN);
    HAL_TIM_Base_Start(&htim6);
}
