#ifndef DPSD_PORTABLE_H
#define DPSD_PORTABLE_H

#include <stdint.h>
#include <stdbool.h>

/* ADC 量程 */
#define DPSD_ADC_MAX_CODE   4095        /* 12 位 ADC = 4095，16 位 = 65535 */
#define DPSD_ADC_VREF       3.3f        /* ADC 参考电压（V）*/

/* 异常检测阈值 */
#define DPSD_CLIP_MARGIN     200        /* ADC 值距离 0 或 MAX 小于此值 → 判定削顶 */
#define DPSD_AMP_JUMP_MAX    0.30f      /* 相邻两次幅度相对跳变超过 30% → 可疑 */
#define DPSD_SNR_MIN_DB      6.0f       /* SNR 低于 6dB → 结果不可信 */
#define DPSD_MEDIAN_WINDOW   5          /* 滑动中值滤波窗口大小（奇数） */

/* 参考信号生成方式 */
#define DPSD_USE_LUT         1          /* 1 = 查表（推荐，可移植），0 = 实时算 sin/cos */
#define DPSD_MAX_BLOCK_SIZE  8192       /* 查找表最大支持的采样点数 */

/* 单次检测结果 */
typedef struct {
    /* ---- 测量值 ---- */
    float amplitude_peak;               /* 峰值幅度（V） */
    float amplitude_pp;                 /* 峰峰值（V） */
    float amplitude_rms;                /* 有效值（V） */
    float phase_deg;                    /* 相位（0 ~ 360°） */
    float phase_rad;                    /* 相位（-pi ~ pi） */
    float dc_offset_v;                  /* 直流偏置（V） */
    float frequency_hz;                 /* 跟踪后的实际频率（Hz） */

    /* ---- 中间量（调试用）---- */
    float I_raw;                        /* 同相分量原始值 */
    float Q_raw;                        /* 正交分量原始值 */

    /* ---- 质量标志 ---- */
    float    snr_db;                    /* 信噪比估计（dB） */
    bool     valid;                     /* 本次结果总体是否有效 */
    bool     clipped;                   /* ADC 是否发生了削顶/削底 */
    bool     low_snr;                   /* SNR 是否低于阈值 */
    bool     amplitude_jump;            /* 幅度是否发生了突变 */
    uint32_t error_mask;                /* 原始错误位掩码，调试用 */
} dpsd_result_t;

/* 错误位定义 */
#define DPSD_ERR_NONE           0x00
#define DPSD_ERR_CLIPPED        (1u << 0)
#define DPSD_ERR_LOW_SNR        (1u << 1)
#define DPSD_ERR_AMP_JUMP       (1u << 2)
#define DPSD_ERR_NULL_PTR       (1u << 3)
#define DPSD_ERR_NOT_INIT       (1u << 4)
#define DPSD_ERR_BAD_CONFIG     (1u << 5)
#define DPSD_ERR_DC_SATURATION  (1u << 6)

/* 性能统计（可选） */
typedef struct {
    uint32_t total_blocks;              /* 总处理块数 */
    uint32_t valid_blocks;              /* 有效块数 */
    uint32_t rejected_blocks;           /* 被剔除块数 */
    uint32_t clip_events;               /* 削顶事件数 */
    uint32_t jump_events;               /* 幅度跳变事件数 */
    uint32_t low_snr_events;            /* 低 SNR 事件数 */
    float    avg_snr_db;                /* 平均 SNR */
} dpsd_stats_t;


/**
 * @brief 初始化 DPSD 模块
 *
 * @param sample_rate    ADC 采样率（Hz），例如 2500000.0f
 * @param target_freq    目标信号频率（Hz），例如 10000.0f
 * @param block_size     每次处理的采样点数，例如 8192
 *                       影响等效噪声带宽：ENBW = sample_rate / block_size
 * @return 0=成功，-1=参数非法
 *
 * @note  采样率应为信号频率的整数倍或远大于信号频率；
 *        block_size 越大幅频选择性越好但延迟越大
 */
int dpsd_init(float sample_rate, float target_freq, uint32_t block_size);

/**
 * @brief 处理一块 ADC 数据
 *
 * @param adc_data    ADC 原始采样数组，长度 = dpsd_init 时设定的 block_size
 * @param result      输出结果指针（可为 NULL，仅更新内部统计）
 * @return 0=成功，-1=参数错误
 *
 * @note  本函数在 result->valid == false 时仍然填充幅度和相位值，
 *        用户可根据 valid 标志自行决定是否采用
 */
int dpsd_process(const uint16_t *adc_data, dpsd_result_t *result);

/**
 * @brief 修改目标频率（用于扫频测量）
 *
 * @param freq_hz 新的目标频率（Hz），必须 > 0 且 < sample_rate/2
 * @return 0=成功，-1=越界
 */
int dpsd_set_target_freq(float freq_hz);

/**
 * @brief 重置内部状态（频率跟踪 + 中值滤波历史 + 统计）
 */
void dpsd_reset(void);

/**
 * @brief 获取运行统计
 */
void dpsd_get_stats(dpsd_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* DPSD_PORTABLE_H */
