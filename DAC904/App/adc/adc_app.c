#include "adc_app.h"

#include "adc.h"
#include "tim.h"
#include <string.h>

#define HIST_BINS    512U
#define BIN_WIDTH    (ADC_CODE_NUM / HIST_BINS)
#define BUFFER_HALF0 0x01U
#define BUFFER_HALF1 0x02U

__attribute__((section(".bss.AXI_SRAM"), aligned(32)))
uint16_t adc_buffer[ADC_LEN];

__attribute__((section(".bss.AXI_SRAM"), aligned(32)))
uint16_t dac_buffer[ADC_LEN];

adc_waveform_info_t ADC_info;
volatile uint8_t adc_con_flag = 0U;

static uint32_t s_histogram[HIST_BINS];
static volatile uint8_t s_adc_ready_mask;
static volatile uint8_t s_dac_free_mask;
static volatile uint32_t s_adc_overrun_count;
static volatile uint32_t s_dac_underrun_count;
static volatile uint32_t s_dma_error_count;

static float adc_to_v(uint16_t value)
{
    return (float)value * ADC_VREF / (float)ADC_MAX_CODE;
}

static void mark_adc_ready(uint8_t mask)
{
    if ((s_adc_ready_mask & mask) != 0U) {
        ++s_adc_overrun_count;
    }
    s_adc_ready_mask |= mask;
    adc_con_flag = 1U;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        mark_adc_ready(BUFFER_HALF0);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        mark_adc_ready(BUFFER_HALF1);
    }
}

static void dac_dma_half_callback(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    if ((s_dac_free_mask & BUFFER_HALF0) != 0U) {
        ++s_dac_underrun_count;
    }
    s_dac_free_mask |= BUFFER_HALF0;
}

static void dac_dma_full_callback(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    if ((s_dac_free_mask & BUFFER_HALF1) != 0U) {
        ++s_dac_underrun_count;
    }
    s_dac_free_mask |= BUFFER_HALF1;
}

static void dac_dma_error_callback(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    ++s_dma_error_count;
}

static void process_half(uint32_t offset)
{
    uint16_t vmax = 0U;
    uint16_t vmin = (uint16_t)ADC_MAX_CODE;
    uint64_t sum = 0U;

    for (uint32_t i = 0U; i < ADC_HALF_LEN; ++i) {
        const uint16_t adc_code =
            (uint16_t)(adc_buffer[offset + i] & ADC_MAX_CODE);

        if (adc_code > vmax) {
            vmax = adc_code;
        }
        if (adc_code < vmin) {
            vmin = adc_code;
        }
        sum += adc_code;

        /*
         * Default processing is a 12-bit to 14-bit full-scale mapping.
         * A filter or gain/offset stage can replace this expression;
         * its output must remain in the range 0..DAC_MAX_CODE.
         */
        dac_buffer[offset + i] =
            (uint16_t)(((uint32_t)adc_code << 2U) & DAC_MAX_CODE);
    }

    ADC_info.max_adc = vmax;
    ADC_info.min_adc = vmin;
    ADC_info.vmax = adc_to_v(vmax);
    ADC_info.vmin = adc_to_v(vmin);
    ADC_info.vpp = ADC_info.vmax - ADC_info.vmin;
    ADC_info.vavg = ((float)sum / (float)ADC_HALF_LEN) *
                    ADC_VREF / (float)ADC_MAX_CODE;
}

HAL_StatusTypeDef ADC_DAC_Stream_Start(void)
{
    HAL_StatusTypeDef status;

    s_adc_ready_mask = 0U;
    s_dac_free_mask = 0U;
    s_adc_overrun_count = 0U;
    s_dac_underrun_count = 0U;
    s_dma_error_count = 0U;
    adc_con_flag = 0U;

    for (uint32_t i = 0U; i < ADC_LEN; ++i) {
        dac_buffer[i] = DAC_MID_CODE;
    }
    GPIOC->ODR = DAC_MID_CODE;

    status = HAL_ADCEx_Calibration_Start(&hadc1,
                                         ADC_CALIB_OFFSET,
                                         ADC_SINGLE_ENDED);
    if (status != HAL_OK) {
        return status;
    }

    hdma_tim4_ch1.XferHalfCpltCallback = dac_dma_half_callback;
    hdma_tim4_ch1.XferCpltCallback = dac_dma_full_callback;
    hdma_tim4_ch1.XferErrorCallback = dac_dma_error_callback;

    status = HAL_DMA_Start_IT(&hdma_tim4_ch1,
                              (uint32_t)dac_buffer,
                              (uint32_t)&GPIOC->ODR,
                              ADC_LEN);
    if (status != HAL_OK) {
        return status;
    }
    __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC1);

    status = HAL_ADC_Start_DMA(&hadc1,
                               (uint32_t *)adc_buffer,
                               ADC_LEN);
    if (status != HAL_OK) {
        __HAL_TIM_DISABLE_DMA(&htim4, TIM_DMA_CC1);
        (void)HAL_DMA_Abort(&hdma_tim4_ch1);
        return status;
    }

    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE | TIM_FLAG_CC1);

    status = HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    if (status != HAL_OK) {
        (void)HAL_ADC_Stop_DMA(&hadc1);
        __HAL_TIM_DISABLE_DMA(&htim4, TIM_DMA_CC1);
        (void)HAL_DMA_Abort(&hdma_tim4_ch1);
    }
    return status;
}

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
    for (;;) {
        uint8_t ready;
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        ready = (uint8_t)(s_adc_ready_mask & s_dac_free_mask);
        if ((ready & BUFFER_HALF0) != 0U) {
            s_adc_ready_mask &= (uint8_t)~BUFFER_HALF0;
            s_dac_free_mask &= (uint8_t)~BUFFER_HALF0;
            ready = BUFFER_HALF0;
        } else if ((ready & BUFFER_HALF1) != 0U) {
            s_adc_ready_mask &= (uint8_t)~BUFFER_HALF1;
            s_dac_free_mask &= (uint8_t)~BUFFER_HALF1;
            ready = BUFFER_HALF1;
        } else {
            ready = 0U;
            adc_con_flag = (s_adc_ready_mask != 0U) ? 1U : 0U;
        }
        if (primask == 0U) {
            __enable_irq();
        }

        if (ready == BUFFER_HALF0) {
            process_half(0U);
        } else if (ready == BUFFER_HALF1) {
            process_half(ADC_HALF_LEN);
        } else {
            break;
        }
    }
}

uint32_t ADC_DAC_GetAdcOverrunCount(void)
{
    return s_adc_overrun_count;
}

uint32_t ADC_DAC_GetDacUnderrunCount(void)
{
    return s_dac_underrun_count;
}

uint32_t ADC_DAC_GetDmaErrorCount(void)
{
    return s_dma_error_count;
}
