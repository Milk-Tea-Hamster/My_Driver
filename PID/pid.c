/**
 * @file pid.c
 * @brief PID 控制器 C 语言实现文件
 * 纯 C 实现，零外部依赖（无 <math.h>），适用于单片机及嵌入式系统。
 */

#include "pid.h"



/**
 * @brief 计算浮点数的绝对值
 * @param x 输入值
 * @return |x|
 */
static inline float pid_fabsf(float x) {
    return (x < 0.0f) ? -x : x;
}

/* ================================================================
 * API 函数实现
 * ================================================================ */

/**
 * @brief 初始化 PID 控制器
 */
void PID_Init(PIDController *pid, float kp, float ki, float kd) {
    /* 增益系数 */
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    /* 采样时间 */
    pid->sample_time = PID_DEFAULT_SAMPLE_TIME;

    /* 运行时状态 */
    pid->output = 0.0f;
    pid->target = 0.0f;
    pid->error = 0.0f;
    pid->error_integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_target = 0.0f;

    /* 模式参数默认值 */
    pid->proportional_weight = PID_DEFAULT_PROP_WEIGHT;
    pid->clamped_integral_limit = PID_DEFAULT_CLAMP_INT_LIMIT;
    pid->cond_integral_error_threshold = PID_DEFAULT_COND_INT_ERR_THRESHOLD;
    pid->derivative_weight = PID_DEFAULT_DERIVATIVE_WEIGHT;

    /* 默认为标准模式 */
    pid->proportional_mode = PID_PROPORTIONAL_STANDARD;
    pid->integral_mode = PID_INTEGRAL_STANDARD;
    pid->derivative_mode = PID_DERIVATIVE_STANDARD;

    /* 死区 */
    pid->deadzone = PID_DEFAULT_DEADZONE;

    /* 采样时间合法性检查 */
    if (pid->sample_time <= 0.0f) {
        pid->sample_time = PID_DEFAULT_SAMPLE_TIME;
    }
}

/**
 * @brief 设置 PID 增益参数
 */
void PID_SetParams(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief 设置目标值
 */
void PID_SetTarget(PIDController *pid, float target) {
    pid->prev_target = pid->target;
    pid->target = target;
}

/**
 * @brief 设置采样时间
 */
void PID_SetSampleTime(PIDController *pid, float sample_time) {
    if (sample_time <= 0.0f) {
        sample_time = PID_DEFAULT_SAMPLE_TIME;
    }
    pid->sample_time = sample_time;
}

/**
 * @brief 设置比例模式（不带权重）
 */
void PID_SetProportionalMode(PIDController *pid, PIDProportionalMode mode) {
    pid->proportional_mode = mode;
}

/**
 * @brief 设置比例模式（带权重）
 */
void PID_SetProportionalModeWeighted(PIDController *pid, PIDProportionalMode mode, float weight) {
    pid->proportional_mode = mode;
    pid->proportional_weight = weight;
}

/**
 * @brief 设置积分模式（不带参数）
 */
void PID_SetIntegralMode(PIDController *pid, PIDIntegralMode mode) {
    pid->integral_mode = mode;
}

/**
 * @brief 设置积分模式（带参数）
 */
void PID_SetIntegralModeWithValue(PIDController *pid, PIDIntegralMode mode, float value) {
    pid->integral_mode = mode;

    if (mode == PID_INTEGRAL_CLAMPED) {
        pid->clamped_integral_limit = pid_fabsf(value);
    } else if (mode == PID_INTEGRAL_CONDITIONAL) {
        pid->cond_integral_error_threshold = pid_fabsf(value);
    }
}

/**
 * @brief 设置微分模式（不带权重）
 */
void PID_SetDerivativeMode(PIDController *pid, PIDDerivativeMode mode) {
    pid->derivative_mode = mode;
}

/**
 * @brief 设置微分模式（带权重）
 */
void PID_SetDerivativeModeWeighted(PIDController *pid, PIDDerivativeMode mode, float weight) {
    pid->derivative_mode = mode;
    pid->derivative_weight = weight;
}

/**
 * @brief 设置死区
 */
void PID_SetDeadzone(PIDController *pid, float deadzone) {
    /* 强制取正数 */
    if (deadzone < 0.0f) {
        deadzone = -deadzone;
    }
    pid->deadzone = deadzone;
}

/**
 * @brief 重置控制器状态
 */
void PID_Reset(PIDController *pid) {
    pid->output = 0.0f;
    pid->error = 0.0f;
    pid->error_integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_target = pid->target;
}

/**
 * @brief PID 核心计算
 *
 * 执行一次 PID 控制迭代，包含：
 * 1. 输出限幅自动交换检查
 * 2. 误差计算与死区处理
 * 3. 比例项计算（标准/基于测量/加权）
 * 4. 积分项计算（标准/限幅/条件）
 * 5. 微分项计算（标准/基于测量/加权）
 * 6. 历史状态保存
 * 7. 输出合成与限幅
 */
float PID_Calc(PIDController *pid, float curr_measurement, float upper_limit, float lower_limit) {
    float p, i, d;
    float delta_e, delta_m;

    /* 输出限幅自动交换 */
    if (lower_limit > upper_limit) {
        float temp = lower_limit;
        lower_limit = upper_limit;
        upper_limit = temp;
    }

    /* 计算误差 */
    pid->error = pid->target - curr_measurement;

    /* 死区处理：误差在死区内时强制归零，消除微小噪声引起的抖动 */
    if (pid_fabsf(pid->error) < pid->deadzone) {
        pid->error = 0.0f;
        pid->prev_error = 0.0f;
    }

    /* 计算增量 */
    delta_e = pid->error - pid->prev_error;
    delta_m = curr_measurement - pid->prev_measurement;

    /* ---- 比例项 P ---- */
    p = 0.0f;
    switch (pid->proportional_mode) {
    case PID_PROPORTIONAL_STANDARD:
        p = pid->kp * pid->error;
        break;
    case PID_PROPORTIONAL_ON_MEASUREMENT:
        p = -(pid->kp) * curr_measurement;
        break;
    case PID_PROPORTIONAL_WEIGHTED:
        p = pid->kp * (pid->proportional_weight * pid->target - curr_measurement);
        break;
    }

    /* ---- 积分项 I ---- */
    switch (pid->integral_mode) {
    case PID_INTEGRAL_STANDARD:
        pid->error_integral += pid->error * pid->sample_time;
        break;
    case PID_INTEGRAL_CLAMPED:
        pid->error_integral += pid->error * pid->sample_time;
        if (pid_fabsf(pid->error_integral) > pid->clamped_integral_limit) {
            pid->error_integral = (pid->error_integral > 0.0f)
                ? pid->clamped_integral_limit
                : -(pid->clamped_integral_limit);
        }
        break;
    case PID_INTEGRAL_CONDITIONAL:
        /* 仅当误差在阈值内时累积积分，防止大误差导致积分饱和 */
        if (pid_fabsf(pid->error) < pid->cond_integral_error_threshold) {
            pid->error_integral += pid->error * pid->sample_time;
        }
        break;
    }
    i = pid->ki * pid->error_integral;

    /* ---- 微分项 D ---- */
    d = 0.0f;
    switch (pid->derivative_mode) {
    case PID_DERIVATIVE_STANDARD:
        d = pid->kd * delta_e / pid->sample_time;
        break;
    case PID_DERIVATIVE_ON_MEASUREMENT:
        d = -(pid->kd) * delta_m / pid->sample_time;
        break;
    case PID_DERIVATIVE_WEIGHTED:
        d = pid->kd * (pid->derivative_weight * (pid->target - pid->prev_target) - delta_m)
            / pid->sample_time;
        break;
    }

    /* 保存历史值，供下次计算使用 */
    pid->prev_measurement = curr_measurement;
    pid->prev_error = pid->error;

    /* ---- 输出合成与限幅 ---- */
    pid->output = p + i + d;

    if (pid->output > upper_limit) {
        pid->output = upper_limit;
    }
    if (pid->output < lower_limit) {
        pid->output = lower_limit;
    }

    return pid->output;
}
