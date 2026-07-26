#ifndef __ADC_APP_H
#define __ADC_APP_H

#include "stm32h7xx_hal.h"


#define ADC_VREF      3.3f
#define ADC_BITS      12U
#define ADC_CODE_NUM  (1UL << ADC_BITS)
#define ADC_MAX_CODE  (ADC_CODE_NUM - 1UL)
#define ADC_LEN       1024U
#define ADC_HALF_LEN  (ADC_LEN / 2U)
#define DAC_MAX_CODE  0x3FFFU
#define DAC_MID_CODE  0x2000U

typedef struct {
    float vpp;          /* 峰峰值电压，单位：V */
    float vmax;         /* 最大电压，单位：V */
    float vmin;         /* 最小电压，单位：V */
    float vavg;         /* 平均电压，单位：V */
    uint16_t max_adc;   /* 最大 ADC 原始值 */
    uint16_t min_adc;   /* 最小 ADC 原始值 */
} adc_waveform_info_t;


extern uint16_t adc_buffer[ADC_LEN];
extern uint16_t dac_buffer[ADC_LEN];
extern adc_waveform_info_t ADC_info;
extern volatile uint8_t adc_con_flag;

HAL_StatusTypeDef ADC_DAC_Stream_Start(void);
void adc_proc(void);
uint32_t ADC_DAC_GetAdcOverrunCount(void);
uint32_t ADC_DAC_GetDacUnderrunCount(void);
uint32_t ADC_DAC_GetDmaErrorCount(void);

/* 直接扫描全部采样点，计算峰峰值。 */
void adc_get_vpp(adc_waveform_info_t *info,
                 const uint16_t *data,
                 uint32_t len);


void adc_get_vpp_robust(adc_waveform_info_t *info,
                        const uint16_t *data,
                        uint32_t len,
                        float discard_pct);

/* 自动在普通算法和抗毛刺算法之间选择。 */
void adc_get_vpp_auto(adc_waveform_info_t *info,
                      const uint16_t *data,
                      uint32_t len);

#endif /* __ADC_APP_H */
