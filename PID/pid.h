/**
 * @file pid.h
 * @brief PID 控制器 C 语言库头文件
 *
 * 本文件定义了一个功能完整的 PID 控制器，支持多种 PID 算法变体，
 * 包括标准 PID、基于测量的 PID、加权 PID 等高级模式。
 * 适用于 STM32、AVR 等单片机及嵌入式系统。
 * 纯 C 实现，零外部依赖。
 *
 */

/**
 * PID 控制器公式与方程：
 *
 * 标准形式: u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
 *                = P + I + D
 *
 * 其中:
 * u(t) 为输出
 * e(t) = 目标值 - 测量值
 * de(t)/dt = (e(t) - e(t-1))/dt
 *          = d(目标值)/dt - d(测量值)/dt
 * ∫e(t)dt = (e(t-1) + e(t-2) + ... + e(0)) * dt
 * dt 为采样时间
 * Kp, Ki, Kd 为增益系数
 *
 *
 * 其他公式变体：
 *
 * - 基于测量的比例项:
 *      P = - Kp * 测量值
 *
 * - 加权比例项:
 *      P = Kp*(b * 目标值 - 测量值)
 *      其中 b 为 0~1 的权重因子
 *
 * - 积分限幅:
 *      I = Ki * min(∫e(t)dt, 上限)
 *
 * - 条件积分:
 *      I = Ki*∫e(t)dt, 仅当 |e(t)| < 误差阈值时
 *
 * - 基于测量的微分项:
 *      D = - Kd * d(测量值)/dt
 *
 * - 加权微分项:
 *      D = Kd*(c * d(目标值)/dt - d(测量值)/dt)
 *      其中 c 为 0~1 的权重因子
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 默认参数宏定义
 * ================================================================ */

/** @brief 默认采样时间（1.0 秒） */
#define PID_DEFAULT_SAMPLE_TIME 1.0f

/** @brief 加权比例模式的默认权重 */
#define PID_DEFAULT_PROP_WEIGHT 1.0f

/** @brief 积分限幅模式的默认限幅值 */
#define PID_DEFAULT_CLAMP_INT_LIMIT 200.0f

/** @brief 条件积分模式的默认误差阈值 */
#define PID_DEFAULT_COND_INT_ERR_THRESHOLD 20.0f

/** @brief 加权微分模式的默认权重 */
#define PID_DEFAULT_DERIVATIVE_WEIGHT 1.0f

/** @brief 默认死区值（无死区） */
#define PID_DEFAULT_DEADZONE 0.0f

/* ================================================================
 * 枚举类型定义
 * ================================================================ */

/** @brief 比例项计算模式 */
typedef enum {
    PID_PROPORTIONAL_STANDARD,      /**< 标准比例: P = Kp * 误差 */
    PID_PROPORTIONAL_ON_MEASUREMENT,/**< 基于测量的比例: P = -Kp * 测量值 */
    PID_PROPORTIONAL_WEIGHTED       /**< 加权比例: P = Kp*(b*目标值 - 测量值) */
} PIDProportionalMode;

/** @brief 积分项计算模式 */
typedef enum {
    PID_INTEGRAL_STANDARD,   /**< 标准积分: I = Ki * ∫误差 dt */
    PID_INTEGRAL_CLAMPED,    /**< 限幅积分: I = Ki * min(∫误差 dt, 限幅值) */
    PID_INTEGRAL_CONDITIONAL /**< 条件积分: 仅当 |误差| < 阈值时累积 */
} PIDIntegralMode;

/** @brief 微分项计算模式 */
typedef enum {
    PID_DERIVATIVE_STANDARD,       /**< 标准微分: D = Kd * d(误差)/dt */
    PID_DERIVATIVE_ON_MEASUREMENT, /**< 基于测量的微分: D = -Kd * d(测量值)/dt */
    PID_DERIVATIVE_WEIGHTED        /**< 加权微分: D = Kd*(c*d(目标值)/dt - d(测量值)/dt) */
} PIDDerivativeMode;

/* ================================================================
 * PID 控制器结构体
 * ================================================================ */

/**
 * @brief PID 控制器结构体
 *
 * 包含 PID 控制器的所有状态变量和配置参数。
 * 使用 PID_Init() 初始化，通过 PID_Calc() 执行控制计算。
 *
 * 使用示例:
 * @code
 * PIDController pid;
 * PID_Init(&pid, 2.0f, 0.5f, 0.1f);
 * PID_SetTarget(&pid, 100.0f);
 *
 * while (1) {
 *     float measurement = read_sensor();
 *     float output = PID_Calc(&pid, measurement, 255.0f, 0.0f);
 *     set_actuator(output);
 *     delay_ms(10);
 * }
 * @endcode
 */
typedef struct {
    /* 增益系数 */
    float kp, ki, kd;

    /* 采样时间（秒） */
    float sample_time;

    /* 上次计算的输出值 */
    float output;

    /* 目标值 */
    float target;

    /* 当前误差、累积积分、上次误差 */
    float error, error_integral, prev_error;

    /* 上次测量值和上次目标值（用于微分计算） */
    float prev_measurement, prev_target;

    /* 加权比例模式的权重参数 b */
    float proportional_weight;

    /* 积分限幅值 */
    float clamped_integral_limit;

    /* 条件积分的误差阈值 */
    float cond_integral_error_threshold;

    /* 加权微分模式的权重参数 c */
    float derivative_weight;

    /* 比例项计算模式 */
    PIDProportionalMode proportional_mode;

    /* 积分项计算模式 */
    PIDIntegralMode integral_mode;

    /* 微分项计算模式 */
    PIDDerivativeMode derivative_mode;

    /* 死区值 */
    float deadzone;
} PIDController;

/* ================================================================
 * API 函数声明
 * ================================================================ */

/**
 * @brief 初始化 PID 控制器
 * @param pid PID 控制器指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
void PID_Init(PIDController *pid, float kp, float ki, float kd);

/**
 * @brief 设置 PID 增益参数
 * @param pid PID 控制器指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
void PID_SetParams(PIDController *pid, float kp, float ki, float kd);

/**
 * @brief 设置目标值
 * @param pid PID 控制器指针
 * @param target 目标值
 */
void PID_SetTarget(PIDController *pid, float target);

/**
 * @brief 设置采样时间
 * @param pid PID 控制器指针
 * @param sample_time 采样时间（秒），必须为正数
 * @note 若 sample_time <= 0，使用默认值 1.0 秒
 */
void PID_SetSampleTime(PIDController *pid, float sample_time);

/**
 * @brief 设置比例模式（不带权重参数）
 * @param pid PID 控制器指针
 * @param mode 比例模式
 */
void PID_SetProportionalMode(PIDController *pid, PIDProportionalMode mode);

/**
 * @brief 设置比例模式（带权重参数）
 * @param pid PID 控制器指针
 * @param mode 比例模式
 * @param weight 加权比例模式的权重参数 b（0~1）
 */
void PID_SetProportionalModeWeighted(PIDController *pid, PIDProportionalMode mode, float weight);

/**
 * @brief 设置积分模式（不带附加参数）
 * @param pid PID 控制器指针
 * @param mode 积分模式
 */
void PID_SetIntegralMode(PIDController *pid, PIDIntegralMode mode);

/**
 * @brief 设置积分模式（带附加参数）
 * @param pid PID 控制器指针
 * @param mode 积分模式
 * @param value 限幅模式的限幅值 或 条件积分的误差阈值（取绝对值）
 */
void PID_SetIntegralModeWithValue(PIDController *pid, PIDIntegralMode mode, float value);

/**
 * @brief 设置微分模式（不带权重参数）
 * @param pid PID 控制器指针
 * @param mode 微分模式
 */
void PID_SetDerivativeMode(PIDController *pid, PIDDerivativeMode mode);

/**
 * @brief 设置微分模式（带权重参数）
 * @param pid PID 控制器指针
 * @param mode 微分模式
 * @param weight 加权微分模式的权重参数 c（0~1）
 */
void PID_SetDerivativeModeWeighted(PIDController *pid, PIDDerivativeMode mode, float weight);

/**
 * @brief 设置死区
 * @param pid PID 控制器指针
 * @param deadzone 死区值（取绝对值）
 * @note 当 |误差| < 死区时，误差视为零，防止微小误差导致的控制器抖动
 */
void PID_SetDeadzone(PIDController *pid, float deadzone);

/**
 * @brief PID 核心计算
 * @param pid PID 控制器指针
 * @param curr_measurement 当前测量值
 * @param upper_limit 输出上限
 * @param lower_limit 输出下限
 * @return 限幅后的 PID 输出值
 * @note 若 lower_limit > upper_limit，两者自动交换
 */
float PID_Calc(PIDController *pid, float curr_measurement, float upper_limit, float lower_limit);

/**
 * @brief 重置 PID 控制器状态
 * @param pid PID 控制器指针
 * @note 清除积分、误差历史、输出值，但保留目标值和所有配置参数
 */
void PID_Reset(PIDController *pid);

/* ================================================================
 * 内联 getter 函数（获取控制器内部状态）
 * ================================================================ */

/**
 * @brief 获取上次计算的输出值
 * @param pid PID 控制器指针
 * @return 上次输出值
 */
static inline float PID_GetLastOutput(const PIDController *pid) {
    return pid->output;
}

/**
 * @brief 获取当前误差值
 * @param pid PID 控制器指针
 * @return 当前误差（目标值 - 测量值）
 */
static inline float PID_GetError(const PIDController *pid) {
    return pid->error;
}

/**
 * @brief 获取当前积分累积值
 * @param pid PID 控制器指针
 * @return 当前积分累积值
 */
static inline float PID_GetIntegral(const PIDController *pid) {
    return pid->error_integral;
}

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
