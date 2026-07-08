#ifndef __WAVEFORM_H__
#define __WAVEFORM_H__

#include "main.h"

/* 采样率固定 2MHz */
#define WAVE_SAMPLE_RATE    2000000U
/* 频率范围 500Hz ~ 2kHz，超范围自动钳位 */
#define WAVE_FREQ_MIN       500U
#define WAVE_FREQ_MAX       2000U
/* 最低频率对应的最大采样点数 */
#define WAVE_MAX_SAMPLES    4000U

/* VREF = 3.3V，DAC 满幅输出峰峰值 */
#define VREF_VOLTAGE    3.3f
#define VPP_MAX         3.3f
#define VPP_MIN         0.0f
#define VPP_STEP        0.1f

typedef enum {
    WAVE_SINE = 0,
    WAVE_TRIANGLE,
    WAVE_SAWTOOTH,
    WAVE_SQUARE
} WaveType;

void WaveGen_Init(void);
void WaveGen_Start(WaveType type, uint16_t freq_hz);
void WaveGen_Stop(void);
void WaveGen_SetFrequency(uint16_t freq_hz);
void WaveGen_SetAmplitude(uint8_t pct);
void WaveGen_SetDuty(uint8_t pct);
void WaveGen_AdjustVpp(float delta);   /* 双击按键调节峰峰值，自动钳位 */
float     WaveGen_GetVpp(void);
WaveType  WaveGen_GetType(void);
uint16_t  WaveGen_GetFrequency(void);
uint8_t   WaveGen_GetAmplitude(void);
uint8_t   WaveGen_GetDuty(void);

#endif /* __WAVEFORM_H__ */
