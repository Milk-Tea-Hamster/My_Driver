#include "waveform.h"
#include "dac.h"
#include "tim.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 波形数据缓冲区，4 字节对齐 */
static uint16_t wave_table[WAVE_MAX_SAMPLES] __attribute__((aligned(4)));
static uint16_t wave_samples;
static WaveType  current_type  = WAVE_SINE;
static uint16_t  current_freq  = 1000;
static uint8_t   current_amp   = 100;   /* 幅度百分比 0~100 */
static uint8_t   current_duty  = 50;    /* 方波占空比 0~100 */

/* 夹紧频率到有效范围 [500Hz, 2kHz] */
static uint16_t clamp_freq(uint16_t freq)
{
    if (freq < WAVE_FREQ_MIN) return WAVE_FREQ_MIN;
    if (freq > WAVE_FREQ_MAX) return WAVE_FREQ_MAX;
    return freq;
}

/*
 * 禁用 DAC 输出缓冲。
 * 缓冲开启时 DAC 无法输出接近 0V 的电压（最低约 0.2V），导致削底失真。
 * 禁用后输出可接近轨到轨，但输出阻抗升高至约 15kΩ，需高阻负载。
 * BOFF1 只能在 EN1=0 时写入，调用前确保 DAC 通道已停止。
 */
static void dac_disable_buffer(void)
{
    DAC->CR |= DAC_CR_BOFF1;
}

/* 幅度缩放因子: 0.0 ~ 1.0 */
static float amp_factor(void)
{
    return current_amp / 100.0f;
}

/* 生成正弦波表，中心值 2048，峰值幅度受 current_amp 控制 */
static void gen_sine(uint16_t samples)
{
    float amp = amp_factor() * 2047.0f;
    for (uint16_t i = 0; i < samples; i++) {
        float phase = 2.0f * (float)M_PI * i / samples;
        wave_table[i] = (uint16_t)(2048.0f + amp * sinf(phase) + 0.5f);
    }
}

/* 生成三角波表，以 2047.5 为中心按幅度缩放 */
static void gen_triangle(uint16_t samples)
{
    float mid = 2047.5f;
    float full = 4095.0f;
    float scale = amp_factor();
    for (uint16_t i = 0; i < samples; i++) {
        float phase = (float)i / samples;
        float raw = (phase < 0.5f) ? (2.0f * phase) : (2.0f - 2.0f * phase);
        float val = mid + (raw * full - mid) * scale;
        wave_table[i] = (uint16_t)(val + 0.5f);
    }
}

/* 生成锯齿波表，以 2047.5 为中心按幅度缩放 */
static void gen_sawtooth(uint16_t samples)
{
    float mid = 2047.5f;
    float full = 4095.0f;
    float scale = amp_factor();
    for (uint16_t i = 0; i < samples; i++) {
        float phase = (float)i / samples;
        float val = mid + (phase * full - mid) * scale;
        wave_table[i] = (uint16_t)(val + 0.5f);
    }
}

/* 生成方波表，前 duty% 的采样点为高电平，其余为低电平 */
static void gen_square(uint16_t samples)
{
    float mid = 2047.5f;
    float half_swing = 2047.5f * amp_factor();  /* 峰值偏离中点的幅度 */
    uint16_t high = (uint16_t)(mid + half_swing + 0.5f);
    uint16_t low  = (uint16_t)(mid - half_swing + 0.5f);
    uint16_t high_cnt = (uint16_t)((uint32_t)samples * current_duty / 100);

    for (uint16_t i = 0; i < high_cnt; i++) {
        wave_table[i] = high;
    }
    for (uint16_t i = high_cnt; i < samples; i++) {
        wave_table[i] = low;
    }
}

/* 初始化：默认生成 100% 幅度 1kHz 正弦波表 */
void WaveGen_Init(void)
{
    wave_samples = WAVE_SAMPLE_RATE / 1000; /* 2000 点 */
    gen_sine(wave_samples);
}

/* 启动指定波形和频率的输出 */
void WaveGen_Start(WaveType type, uint16_t freq_hz)
{
    freq_hz = clamp_freq(freq_hz);
    wave_samples = WAVE_SAMPLE_RATE / freq_hz;

    if (wave_samples > WAVE_MAX_SAMPLES) {
        wave_samples = WAVE_MAX_SAMPLES;
    }

    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    HAL_TIM_Base_Stop(&htim2);
    htim2.Instance->CNT = 0;

    /* 禁用输出缓冲，消除削底失真 */
    dac_disable_buffer();

    current_type = type;
    current_freq = freq_hz;

    switch (type) {
    case WAVE_SINE:
        gen_sine(wave_samples);
        break;
    case WAVE_TRIANGLE:
        gen_triangle(wave_samples);
        break;
    case WAVE_SAWTOOTH:
        gen_sawtooth(wave_samples);
        break;
    case WAVE_SQUARE:
        gen_square(wave_samples);
        break;
    }

    HAL_TIM_Base_Start(&htim2);
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1,
                      (uint32_t *)wave_table, wave_samples,
                      DAC_ALIGN_12B_R);
}

/* 停止波形输出 */
void WaveGen_Stop(void)
{
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    HAL_TIM_Base_Stop(&htim2);
}

/* 切换频率，保持当前波形类型和幅度不变 */
void WaveGen_SetFrequency(uint16_t freq_hz)
{
    WaveGen_Start(current_type, freq_hz);
}

/* 设置幅度 0~100%，超范围钳位，重新生成波表并重启动输出 */
void WaveGen_SetAmplitude(uint8_t pct)
{
    if (pct > 100) pct = 100;
    current_amp = pct;
    WaveGen_Start(current_type, current_freq);
}

/* 获取当前峰峰值 (V) */
float WaveGen_GetVpp(void)
{
    return current_amp * VREF_VOLTAGE / 100.0f;
}

/* 设置峰峰值，自动钳位到有效范围，重新生成波表 */
static void WaveGen_SetVpp(float vpp)
{
    if (vpp < VPP_MIN) vpp = VPP_MIN;
    if (vpp > VPP_MAX) vpp = VPP_MAX;
    current_amp = (uint8_t)(vpp * 100.0f / VREF_VOLTAGE + 0.5f);
    WaveGen_Start(current_type, current_freq);
}

/* 以 VPP_STEP 步进调节峰峰值，正值增加负值减小，自动钳位 */
void WaveGen_AdjustVpp(float delta)
{
    float vpp = WaveGen_GetVpp() + delta;
    WaveGen_SetVpp(vpp);
}

/* 设置方波占空比 0~100%，仅对方波有效，超范围钳位 */
void WaveGen_SetDuty(uint8_t pct)
{
    if (pct > 100) pct = 100;
    current_duty = pct;
    if (current_type == WAVE_SQUARE) {
        WaveGen_Start(current_type, current_freq);
    }
}

WaveType WaveGen_GetType(void)
{
    return current_type;
}

uint16_t WaveGen_GetFrequency(void)
{
    return current_freq;
}

uint8_t WaveGen_GetAmplitude(void)
{
    return current_amp;
}

uint8_t WaveGen_GetDuty(void)
{
    return current_duty;
}
