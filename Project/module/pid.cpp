#include "pid.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// 计算滤波器系数
void PID_UpdateDifferentialFilterCoefficients(PidController_t *pid) {
    if (pid == NULL) return;
    
    float samplingFreq = pid->config.diffFilterSamplingFreq;
    float cutoffFreq = pid->config.diffFilterCutoffFreq;
    float q = pid->config.diffFilterQ;
    
    // 检查是否低于奈奎斯特频率
    if (cutoffFreq < (samplingFreq / 2.0f)) {
        // 设置变量
        const float omega = 2.0f * PI * cutoffFreq / samplingFreq;
        const float sn = sinf(omega);
        const float cs = cosf(omega);
        const float alpha = sn / (2.0f * q);

        // 低通滤波器系数
        float b0 = (1.0f - cs) / 2.0f;
        float b1 = 1.0f - cs;
        float b2 = (1.0f - cs) / 2.0f;
        
        const float a0 = 1.0f + alpha;
        const float a1 = -2.0f * cs;
        const float a2 = 1.0f - alpha;

        // 预计算系数
        pid->status.diffFilterB0 = b0 / a0;
        pid->status.diffFilterB1 = b1 / a0;
        pid->status.diffFilterB2 = b2 / a0;
        pid->status.diffFilterA1 = a1 / a0;
        pid->status.diffFilterA2 = a2 / a0;
    } else {
        // 截止频率高于奈奎斯特频率 - 无滤波直接通过
        pid->status.diffFilterB0 = 1.0f;
        pid->status.diffFilterB1 = 0.0f;
        pid->status.diffFilterB2 = 0.0f;
        pid->status.diffFilterA1 = 0.0f;
        pid->status.diffFilterA2 = 0.0f;
    }
}

// C接口实现 - 初始化函数现在只接受预先配置好的控制器
void PID_Init(PidController_t *pid) {
    if (pid == NULL) return;
    
    // 初始化状态变量
    pid->status.target = 0.0f;
    pid->status.actual = 0.0f;
    pid->status.error = 0.0f;
    pid->status.lastError = 0.0f;
    pid->status.prevError = 0.0f;
    pid->status.integral = 0.0f;
    pid->status.differential = 0.0f;
    pid->status.output = 0.0f;
    
    // 初始化滤波器状态
    pid->status.diffFilterD1 = 0.0f;
    pid->status.diffFilterD2 = 0.0f;
    
    // 如果启用了微分滤波，计算滤波器系数
    if (pid->config.diffFilterEnabled) {
        PID_UpdateDifferentialFilterCoefficients(pid);
    } else {
        // 未启用时设置为直通
        pid->status.diffFilterB0 = 1.0f;
        pid->status.diffFilterB1 = 0.0f;
        pid->status.diffFilterB2 = 0.0f;
        pid->status.diffFilterA1 = 0.0f;
        pid->status.diffFilterA2 = 0.0f;
    }
}

void PID_SetTarget(PidController_t *pid, float target) {
    if (pid == NULL) return;
    pid->status.target = target;
}

// 二阶低通滤波器实现
float PID_BiquadLPFilter(PidController_t *pid, float input) {
    if (pid == NULL) return input;
    
    const float result = pid->status.diffFilterB0 * input + pid->status.diffFilterD1;
    pid->status.diffFilterD1 = pid->status.diffFilterB1 * input - pid->status.diffFilterA1 * result + pid->status.diffFilterD2;
    pid->status.diffFilterD2 = pid->status.diffFilterB2 * input - pid->status.diffFilterA2 * result;
    
    return result;
}

// 核心更新函数，提供全部参数
float PID_Update(PidController_t *pid, float actual, PidMode_e mode, PidType_e type) {
    if (pid == NULL) return 0.0f;
    
    pid->status.actual = actual;
    pid->status.error = pid->status.target - actual;
    
    // 计算用于积分项的误差（应用死区处理）
    float integralError = pid->status.error;
    if (fabsf(integralError) <= pid->config.errorDeadband) {
        integralError = 0.0f;
    }
    
    // 积分项计算 - 只有在使用PI或PID时才计算，使用经过死区处理的误差
    if ((type == PID_TYPE_PI || type == PID_TYPE_PID) && pid->config.ki != 0.0f) {
        // 积分分离
        if (pid->config.integralSeparationThreshold <= 0.0f || 
            fabsf(pid->status.error) < pid->config.integralSeparationThreshold) {
            
            // 积分抗饱和处理
            if (!pid->config.antiSaturationEnabled || 
                (pid->status.output < pid->config.maxOutput && pid->status.output > -pid->config.maxOutput)) {
                pid->status.integral += integralError;  // 使用经过死区处理的误差
            }
            
            // 积分限幅
            if ((pid->status.integral * pid->config.ki) > pid->config.maxIntegral) {
                pid->status.integral = pid->config.maxIntegral / pid->config.ki;
            } else if ((pid->status.integral * pid->config.ki) < -pid->config.maxIntegral) {
                pid->status.integral = (-pid->config.maxIntegral) / pid->config.ki;
            }
        }
    }
    
    // 微分项计算 - 只有在使用PD或PID时才计算，使用原始误差
    if (type == PID_TYPE_PD || type == PID_TYPE_PID) {
        pid->status.differential = pid->status.error - pid->status.lastError;  // 使用原始误差
        
        // 微分滤波 - 使用二阶低通滤波器
        if (pid->config.diffFilterEnabled && pid->config.kd != 0.0f) {
            pid->status.differential = PID_BiquadLPFilter(pid, pid->status.differential);
        }
    } else {
        pid->status.differential = 0.0f;
    }
    
    // 根据传入的模式参数计算输出，比例项和微分项都使用原始误差
    if (mode == PID_POSITION) {
        // 位置式PID - 比例项使用原始误差
        pid->status.output = pid->config.kp * pid->status.error;
        
        if (type == PID_TYPE_PI || type == PID_TYPE_PID) {
            pid->status.output += pid->config.ki * pid->status.integral;
        }
        
        if (type == PID_TYPE_PD || type == PID_TYPE_PID) {
            pid->status.output += pid->config.kd * pid->status.differential;
        }
    } else { // PID_INCREMENTAL
        // 增量式PID - 比例项使用原始误差
        float increment = pid->config.kp * (pid->status.error - pid->status.lastError);
        
        if (type == PID_TYPE_PI || type == PID_TYPE_PID) {
            increment += pid->config.ki * integralError;  // 积分项使用经过死区处理的误差
        }
        
        if (type == PID_TYPE_PD || type == PID_TYPE_PID) {
            increment += pid->config.kd * (pid->status.error - 2 * pid->status.lastError + pid->status.prevError);
        }
                     
        
        pid->status.output += increment;
    }
    
    // 输出限幅
    if (pid->status.output > pid->config.maxOutput) {
        pid->status.output = pid->config.maxOutput;
    } else if (pid->status.output < -pid->config.maxOutput) {
        pid->status.output = -pid->config.maxOutput;
    }
    
    // 保存误差
    pid->status.prevError = pid->status.lastError;
    pid->status.lastError = pid->status.error;
    
    return pid->status.output;
}

// 位置式PID更新函数
float PID_UpdatePosition(PidController_t *pid, float actual, PidType_e type) {
    return PID_Update(pid, actual, PID_POSITION, type);
}

// 增量式PID更新函数
float PID_UpdateIncremental(PidController_t *pid, float actual, PidType_e type) {
    return PID_Update(pid, actual, PID_INCREMENTAL, type);
}

void PID_Reset(PidController_t *pid) {
    if (pid == NULL) return;
    
    // 重置状态变量
    pid->status.error = 0.0f;
    pid->status.lastError = 0.0f;
    pid->status.prevError = 0.0f;
    pid->status.integral = 0.0f;
    pid->status.differential = 0.0f;
    pid->status.output = 0.0f;
    pid->status.diffFilterD1 = 0.0f;
    pid->status.diffFilterD2 = 0.0f;
}

void PID_ClearIntegral(PidController_t *pid) {
    if (pid == NULL) return;
    pid->status.integral = 0.0f;
}

void PID_SetLimits(PidController_t *pid, float maxOutput, float maxIntegral) {
    if (pid == NULL) return;
    
    pid->config.maxOutput = maxOutput > 0.0f ? maxOutput : pid->config.maxOutput;
    pid->config.maxIntegral = maxIntegral > 0.0f ? maxIntegral : pid->config.maxIntegral;
}

void PID_SetErrorDeadband(PidController_t *pid, float deadband) {
    if (pid == NULL) return;
    pid->config.errorDeadband = deadband >= 0.0f ? deadband : 0.0f;
}

void PID_SetIntegralSeparation(PidController_t *pid, float threshold) {
    if (pid == NULL) return;
    pid->config.integralSeparationThreshold = threshold >= 0.0f ? threshold : 0.0f;
}

void PID_SetAntiSaturation(PidController_t *pid, uint8_t enabled) {
    if (pid == NULL) return;
    pid->config.antiSaturationEnabled = enabled;
}

// 初始化二阶低通滤波器
void PID_InitDifferentialFilter(PidController_t *pid, float samplingFreq, float cutoffFreq, float q) {
    if (pid == NULL) return;
    
    // 更新配置
    pid->config.diffFilterSamplingFreq = samplingFreq;
    pid->config.diffFilterCutoffFreq = cutoffFreq;
    pid->config.diffFilterQ = q;
    
    // 重新计算系数
    PID_UpdateDifferentialFilterCoefficients(pid);
    
    // 重置滤波器状态
    pid->status.diffFilterD1 = 0.0f;
    pid->status.diffFilterD2 = 0.0f;
    
    // 启用滤波器
    pid->config.diffFilterEnabled = 1;
}

// 设置微分滤波器
void PID_SetDifferentialFilter(PidController_t *pid, uint8_t enabled, float cutoffFreq) {
    if (pid == NULL) return;
    
    pid->config.diffFilterEnabled = enabled;
    
    if (enabled && cutoffFreq > 0.0f) {
        pid->config.diffFilterCutoffFreq = cutoffFreq;
        PID_UpdateDifferentialFilterCoefficients(pid);
    }
}

void PID_SetParams(PidController_t *pid, float kp, float ki, float kd) {
    if (pid == NULL) return;
    pid->config.kp = kp;
    pid->config.ki = ki;
    pid->config.kd = kd;
}

// C++接口实现
#ifdef __cplusplus

// 构造函数 - 接受配置结构体的引用
PidController::PidController(const PidConfig_t& config) {
    // 拷贝配置参数
    m_pid.config = config;
    
    // 初始化状态
    PID_Init(&m_pid);
}

void PidController::setTarget(float target) {
    PID_SetTarget(&m_pid, target);
}

float PidController::update(float actual, PidMode_e mode, PidType_e type) {
    return PID_Update(&m_pid, actual, mode, type);
}

float PidController::update(float target, float actual, PidMode_e mode, PidType_e type) {
    setTarget(target);
    return PID_Update(&m_pid, actual, mode, type);
}

float PidController::updatePosition(float actual, PidType_e type) {
    return PID_UpdatePosition(&m_pid, actual, type);
}

float PidController::updatePosition(float target, float actual, PidType_e type) {
    setTarget(target);
    return PID_UpdatePosition(&m_pid, actual, type);
}

float PidController::updateIncremental(float actual, PidType_e type) {
    return PID_UpdateIncremental(&m_pid, actual, type);
}

float PidController::updateIncremental(float target, float actual, PidType_e type) {
    setTarget(target);
    return PID_UpdateIncremental(&m_pid, actual, type);
}

void PidController::reset() {
    PID_Reset(&m_pid);
}

void PidController::clearIntegral() {
    PID_ClearIntegral(&m_pid);
}

void PidController::setLimits(float maxOutput, float maxIntegral) {
    PID_SetLimits(&m_pid, maxOutput, maxIntegral);
}

void PidController::setErrorDeadband(float deadband) {
    PID_SetErrorDeadband(&m_pid, deadband);
}

void PidController::setIntegralSeparation(float threshold) {
    PID_SetIntegralSeparation(&m_pid, threshold);
}

void PidController::setAntiSaturation(bool enabled) {
    PID_SetAntiSaturation(&m_pid, enabled ? 1 : 0);
}

void PidController::setDifferentialFilter(bool enabled, float cutoffFreq) {
    PID_SetDifferentialFilter(&m_pid, enabled ? 1 : 0, cutoffFreq);
}

void PidController::initDifferentialFilter(float samplingFreq, float cutoffFreq, float q) {
    PID_InitDifferentialFilter(&m_pid, samplingFreq, cutoffFreq, q);
}

void PidController::setParams(float kp, float ki, float kd) {
    PID_SetParams(&m_pid, kp, ki, kd);
}

#endif // __cplusplus
