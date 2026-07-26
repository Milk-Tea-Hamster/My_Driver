#include "dpsd.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

#define M_2PI  (2.0f * M_PI)

static struct {
    /* 用户配置 */
    float     sample_rate;              /* 采样率 Hz */
    float     target_freq;              /* 目标频率 Hz */
    uint32_t  block_size;               /* 每块采样点数 */
    bool      initialized;

    /* ---- 异常检测历史 ---- */
    float     last_amplitude;           /* 上一次的有效幅度，用于跳变检测 */
    bool      last_amplitude_valid;

    /* ---- 中值滤波环形缓冲 ---- */
    float     median_buf[DPSD_MEDIAN_WINDOW];
    uint32_t  median_idx;
    bool      median_seeded;              /* 中值滤波是否已用首个有效值初始化 */

    /* ---- SNR 计算中间量 ---- */
    float     signal_power_sum;         /* 信号功率累加（IIR） */
    float     noise_power_sum;          /* 残差功率累加（IIR） */

    /* ---- 统计 ---- */
    dpsd_stats_t stats;
} g_dpsd;

/* ---- 参考信号查找表 ---- */
#if DPSD_USE_LUT
static float g_lut_cos[DPSD_MAX_BLOCK_SIZE];  /* cos 参考表（静态分配，避免 malloc）*/
static float g_lut_sin[DPSD_MAX_BLOCK_SIZE];  /* sin 参考表 */
static uint32_t g_lut_len = 0;                /* 实际使用的查找表长度 */
#endif

/* ========================================================================
 * 内部辅助函数
 * ======================================================================== */

/* ADC 码值 → 电压 */
static inline float adc_to_voltage(uint16_t code, float dc_code)
{
    return ((float)code - dc_code) * DPSD_ADC_VREF / (float)DPSD_ADC_MAX_CODE;
}

/* 电压 → ADC 码值 */
static inline float voltage_to_adc(float v)
{
    return v * (float)DPSD_ADC_MAX_CODE / DPSD_ADC_VREF;
}

/* 相位归一化到 (-pi, pi] */
static inline float wrap_phase(float p)
{
    while (p > M_PI)  p -= M_2PI;
    while (p <= -M_PI) p += M_2PI;
    return p;
}

/* 快速中值 —— 插入排序 O(k²)，k = DPSD_MEDIAN_WINDOW 很小，够用 */
static float sliding_median_insert(float new_val)
{
    float buf[DPSD_MEDIAN_WINDOW];
    uint32_t i, j;

    /* 环形写入 */
    g_dpsd.median_buf[g_dpsd.median_idx] = new_val;
    g_dpsd.median_idx = (g_dpsd.median_idx + 1u) % DPSD_MEDIAN_WINDOW;

    /* 拷贝 + 排序 */
    memcpy(buf, g_dpsd.median_buf, sizeof(buf));

    for (i = 1; i < DPSD_MEDIAN_WINDOW; i++) {
        float key = buf[i];
        j = i;
        while (j > 0 && buf[j - 1] > key) {
            buf[j] = buf[j - 1];
            j--;
        }
        buf[j] = key;
    }

    return buf[DPSD_MEDIAN_WINDOW / 2]; /* 中值 */
}

/* ========================================================================
 * 参考信号查找表生成
 * ======================================================================== */

#if DPSD_USE_LUT

static int generate_lut(void)
{
    uint32_t i;

    /* 查找表长度 = block_size */
    g_lut_len = g_dpsd.block_size;

    if (g_lut_len > DPSD_MAX_BLOCK_SIZE) {
        return -1;
    }

    float dt = 1.0f / g_dpsd.sample_rate;

    for (i = 0; i < g_lut_len; i++) {
        float phase = M_2PI * g_dpsd.target_freq * (float)i * dt;
        g_lut_cos[i] = cosf(phase);
        g_lut_sin[i] = sinf(phase);
    }

    return 0;
}

static void free_lut(void)
{
    /* 静态分配，无需释放，仅清零长度 */
    g_lut_len = 0;
}

#endif /* DPSD_USE_LUT */

/* ========================================================================
 * API 实现
 * ======================================================================== */

int dpsd_init(float sample_rate, float target_freq, uint32_t block_size)
{
    /* 参数校验 */
    if (sample_rate <= 0.0f || target_freq <= 0.0f || block_size == 0) {
        return -1;
    }

    if (target_freq >= sample_rate / 2.0f) {
        return -1; /* 超过奈奎斯特频率 */
    }

    /* 清零全局状态 */
    memset(&g_dpsd, 0, sizeof(g_dpsd));

    g_dpsd.sample_rate  = sample_rate;
    g_dpsd.target_freq  = target_freq;
    g_dpsd.block_size   = block_size;
    g_dpsd.initialized  = true;
    g_dpsd.median_seeded = false;

#if DPSD_USE_LUT
    if (generate_lut() != 0) {
        g_dpsd.initialized = false;
        return -1;
    }
#endif

    return 0;
}

int dpsd_set_target_freq(float freq_hz)
{
    if (!g_dpsd.initialized) return -1;
    if (freq_hz <= 0.0f || freq_hz >= g_dpsd.sample_rate / 2.0f) return -1;

    g_dpsd.target_freq = freq_hz;

#if DPSD_USE_LUT
    free_lut();
    if (generate_lut() != 0) return -1;
#endif

    return 0;
}

int dpsd_process(const uint16_t *adc_data, dpsd_result_t *result)
{
    uint32_t i;
    double I_sum, Q_sum;
    float dc_code_f, dc_offset_v;
    float I, Q, amplitude_raw, amplitude_filtered;
    float phase_rad, phase_deg;
    float snr_db;

    if (!g_dpsd.initialized || adc_data == NULL) {
        if (result) {
            memset(result, 0, sizeof(*result));
            result->error_mask = DPSD_ERR_NOT_INIT;
        }
        return -1;
    }

    g_dpsd.stats.total_blocks++;

    /* ---- 第 1 步：计算直流偏置 ---- */
    {
        double sum = 0.0;
        for (i = 0; i < g_dpsd.block_size; i++) {
            sum += (double)adc_data[i];
        }
        dc_code_f = (float)(sum / (double)g_dpsd.block_size);
    }

    /*
     * 直流饱和检测：
     * 如果直流偏置太接近 ADC 量程边界，说明信号可能超出输入范围
     */
    bool dc_saturated = false;
    if (dc_code_f < (float)DPSD_CLIP_MARGIN ||
        dc_code_f > (float)(DPSD_ADC_MAX_CODE - DPSD_CLIP_MARGIN)) {
        dc_saturated = true;
    }

    dc_offset_v = adc_to_voltage((uint16_t)dc_code_f, 0.0f);

    /* ---- 第 2 步：正交混频 + 削顶检测 ---- */
    I_sum = 0.0;
    Q_sum = 0.0;
    bool clipped = false;

#if DPSD_USE_LUT
    for (i = 0; i < g_dpsd.block_size; i++) {
#else
    {
        float dt = 1.0f / g_dpsd.sample_rate;
    }
    for (i = 0; i < g_dpsd.block_size; i++) {
#endif
        uint16_t code = adc_data[i];

        /* 削顶/削底检测 */
        if (code < DPSD_CLIP_MARGIN || code > (DPSD_ADC_MAX_CODE - DPSD_CLIP_MARGIN)) {
            clipped = true;
        }

        float voltage = adc_to_voltage(code, dc_code_f);

#if DPSD_USE_LUT
        I_sum += (double)(voltage * g_lut_cos[i]);
        Q_sum += (double)(voltage * g_lut_sin[i]);
#else
        float phase = M_2PI * g_dpsd.target_freq * (float)i * dt;
        I_sum += (double)(voltage * cosf(phase));
        Q_sum += (double)(voltage * sinf(phase));
#endif
    }

    /* 归一化 */
    float norm = 2.0f / (float)g_dpsd.block_size;
    I = (float)( I_sum * norm);
    Q = (float)(-Q_sum * norm);   /* 取负以保持相位惯例 */

    /* ---- 第 3 步：幅度和相位 ---- */
    amplitude_raw = sqrtf(I * I + Q * Q);
    phase_rad     = atan2f(Q, I);
    phase_deg     = phase_rad * 180.0f / M_PI;
    if (phase_deg < 0.0f) phase_deg += 360.0f;

    /* ---- 第 4 步：SNR 估计 ---- */
    /*
     * 用原始信号减去重建的正弦波计算残差：
     * 重建信号 = I·cos(phase) - Q·sin(phase) = amplitude·cos(phase_ref_i - phase_sig)
     * 简化做法：信号功率 = amplitude²/2，总功率 = Var(voltage)，噪声 = 总 - 信号
     */
    {
        /* 计算输入信号的总功率（AC 分量）*/
        double total_ac_power = 0.0;
        for (i = 0; i < g_dpsd.block_size; i++) {
            float v = adc_to_voltage(adc_data[i], dc_code_f);
            total_ac_power += (double)(v * v);
        }
        total_ac_power /= (double)g_dpsd.block_size;

        float signal_power = amplitude_raw * amplitude_raw / 2.0f;
        float noise_power  = (float)total_ac_power - signal_power;

        if (noise_power < 0.0f) noise_power = 1e-12f; /* 避免负数和对数爆炸 */

        snr_db = 10.0f * log10f(signal_power / noise_power);

        /* IIR 平滑，用于统计 */
        const float alpha = 0.1f;
        g_dpsd.signal_power_sum += alpha * (signal_power - g_dpsd.signal_power_sum);
        g_dpsd.noise_power_sum  += alpha * (noise_power  - g_dpsd.noise_power_sum);
    }

    /* ---- 第 5 步：异常检测与剔除 ---- */

    uint32_t err = DPSD_ERR_NONE;
    bool low_snr       = false;
    bool amplitude_jump = false;

    if (clipped || dc_saturated) {
        err |= DPSD_ERR_CLIPPED;
        g_dpsd.stats.clip_events++;
    }

    if (dc_saturated) {
        err |= DPSD_ERR_DC_SATURATION;
    }

    if (snr_db < DPSD_SNR_MIN_DB) {
        low_snr = true;
        err |= DPSD_ERR_LOW_SNR;
        g_dpsd.stats.low_snr_events++;
    }

    /* 幅度跳变检测 */
    if (g_dpsd.last_amplitude_valid) {
        float rel_change = fabsf(amplitude_raw - g_dpsd.last_amplitude)
                         / (g_dpsd.last_amplitude + 1e-9f);

        if (rel_change > DPSD_AMP_JUMP_MAX) {
            amplitude_jump = true;
            err |= DPSD_ERR_AMP_JUMP;
            g_dpsd.stats.jump_events++;
        }
    }

    /* ---- 第 6 步：中值滤波幅度（剔除孤立野点）---- */
    if (!g_dpsd.median_seeded) {
        /* 首个测量值：填满整个中值缓冲区，避免冷启动时输出 0 */
        uint32_t k;
        for (k = 0; k < DPSD_MEDIAN_WINDOW; k++) {
            g_dpsd.median_buf[k] = amplitude_raw;
        }
        g_dpsd.median_seeded = true;
    }
    amplitude_filtered = sliding_median_insert(amplitude_raw);

    /*
     * 有效性判断：
     *  - 必须没有削顶（硬件层面不可恢复）
     *  - SNR 和跳变可以分别设置容忍策略
     *
     *  默认策略：削顶 → 不可信；低 SNR 或跳变 → 依旧可参考但标记
     */
    bool valid = (!clipped && !dc_saturated);

    /* 更新历史 */
    if (valid) {
        g_dpsd.last_amplitude = amplitude_filtered;
        g_dpsd.last_amplitude_valid = true;
    }

    /* ---- 第 7 步：填充结果 ---- */
    if (result != NULL) {
        result->amplitude_peak = amplitude_filtered;
        result->amplitude_pp   = amplitude_filtered * 2.0f;
        result->amplitude_rms  = amplitude_filtered / 1.41421356237f; /* sqrt(2) */
        result->phase_deg      = phase_deg;
        result->phase_rad      = phase_rad;
        result->dc_offset_v    = dc_offset_v;
        result->frequency_hz   = g_dpsd.target_freq;
        result->I_raw          = I;
        result->Q_raw          = Q;
        result->snr_db         = snr_db;
        result->valid          = valid;
        result->clipped        = clipped;
        result->low_snr        = low_snr;
        result->amplitude_jump = amplitude_jump;
        result->error_mask     = err;
    }

    /* 统计更新 */
    if (valid) {
        g_dpsd.stats.valid_blocks++;
        g_dpsd.stats.avg_snr_db += 0.01f * (snr_db - g_dpsd.stats.avg_snr_db);
    } else {
        g_dpsd.stats.rejected_blocks++;
    }

    return 0;
}

void dpsd_reset(void)
{
    g_dpsd.last_amplitude_valid = false;
    g_dpsd.last_amplitude = 0.0f;
    g_dpsd.median_idx = 0;
    g_dpsd.median_seeded = false;
    g_dpsd.signal_power_sum = 0.0f;
    g_dpsd.noise_power_sum  = 0.0f;
    memset(g_dpsd.median_buf, 0, sizeof(g_dpsd.median_buf));
    memset(&g_dpsd.stats, 0, sizeof(g_dpsd.stats));
}

void dpsd_get_stats(dpsd_stats_t *stats)
{
    if (stats != NULL) {
        memcpy(stats, &g_dpsd.stats, sizeof(dpsd_stats_t));
    }
}
