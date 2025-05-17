#ifndef PID_H
#define PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// PID模式枚举
typedef enum {
    PID_POSITION = 0,  // 位置式PID
    PID_INCREMENTAL    // 增量式PID
} PidMode_e;

// PID控制器类型枚举
typedef enum {
    PID_TYPE_P = 0,    // 仅使用比例项
    PID_TYPE_PI,       // 使用比例和积分项
    PID_TYPE_PD,       // 使用比例和微分项
    PID_TYPE_PID       // 使用所有项
} PidType_e;

// 滤波器类型枚举
typedef enum {
    FILTER_LPF = 0,    // 低通滤波
    FILTER_NOTCH       // 陷波滤波
} FilterType_e;

// PID配置参数结构体
typedef struct {
    float kp;                          // 比例系数
    float ki;                          // 积分系数
    float kd;                          // 微分系数
    
    float maxOutput;                   // 输出限幅
    float maxIntegral;                 // 积分限幅
    
    float integralSeparationThreshold; // 积分分离阈值
    float errorDeadband;               // 误差死区
    
    uint8_t antiSaturationEnabled;     // 是否开启积分抗饱和
    uint8_t diffFilterEnabled;         // 是否开启微分滤波
    
    // 滤波器配置参数
    float diffFilterSamplingFreq;      // 采样频率(Hz)
    float diffFilterCutoffFreq;        // 截止频率(Hz)
    float diffFilterQ;                 // 品质因子
} PidConfig_t;

// PID状态变量结构体
typedef struct {
    float target;                      // 目标值
    float actual;                      // 实际值
    
    float error;                       // 当前误差
    float lastError;                   // 上一次误差
    float prevError;                   // 前一次误差
    float integral;                    // 积分项
    float differential;                // 微分项
    float output;                      // 输出值
    
    // 二阶低通滤波器参数 - 计算得到的系数
    float diffFilterB0;                // 滤波器系数b0
    float diffFilterB1;                // 滤波器系数b1 
    float diffFilterB2;                // 滤波器系数b2
    float diffFilterA1;                // 滤波器系数a1
    float diffFilterA2;                // 滤波器系数a2
    
    // 滤波器状态变量
    float diffFilterD1;                // 滤波器状态变量d1
    float diffFilterD2;                // 滤波器状态变量d2
} PidStatus_t;

// PID控制器结构体
typedef struct {
    PidConfig_t config;                // 配置参数
    PidStatus_t status;                // 状态变量
} PidController_t;


// C接口函数
void PID_Init(PidController_t *pid);
void PID_SetTarget(PidController_t *pid, float target);
float PID_Update(PidController_t *pid, float actual, PidMode_e mode, PidType_e type);
float PID_UpdatePosition(PidController_t *pid, float actual, PidType_e type);
float PID_UpdateIncremental(PidController_t *pid, float actual, PidType_e type);
void PID_Reset(PidController_t *pid);
void PID_ClearIntegral(PidController_t *pid);
void PID_SetLimits(PidController_t *pid, float maxOutput, float maxIntegral);
void PID_SetErrorDeadband(PidController_t *pid, float deadband);
void PID_SetIntegralSeparation(PidController_t *pid, float threshold);
void PID_SetAntiSaturation(PidController_t *pid, uint8_t enabled);
void PID_SetParams(PidController_t *pid, float kp, float ki, float kd);

// 滤波器接口
void PID_SetDifferentialFilter(PidController_t *pid, uint8_t enabled, float cutoffFreq);
void PID_InitDifferentialFilter(PidController_t *pid, float samplingFreq, float cutoffFreq, float q);
void PID_UpdateDifferentialFilterCoefficients(PidController_t *pid);

#ifdef __cplusplus
}

// C++接口类
class PidController {
private:
    PidController_t m_pid;

public:
    // 构造函数只接受配置结构体
    PidController(const PidConfig_t& config);
    
    void setTarget(float target);
    
    // 综合更新方法（指定模式和类型）
    float update(float actual, PidMode_e mode = PID_POSITION, PidType_e type = PID_TYPE_PID);
    float update(float target, float actual, PidMode_e mode = PID_POSITION, PidType_e type = PID_TYPE_PID);
    
    // 专用位置式/增量式更新方法
    float updatePosition(float actual, PidType_e type = PID_TYPE_PID);
    float updatePosition(float target, float actual, PidType_e type = PID_TYPE_PID);
    float updateIncremental(float actual, PidType_e type = PID_TYPE_PID);
    float updateIncremental(float target, float actual, PidType_e type = PID_TYPE_PID);
    
    void reset();
    void clearIntegral();
    
    void setLimits(float maxOutput, float maxIntegral);
    void setErrorDeadband(float deadband);
    void setIntegralSeparation(float threshold);
    void setAntiSaturation(bool enabled);
    void setDifferentialFilter(bool enabled, float cutoffFreq = 20.0f);
    void initDifferentialFilter(float samplingFreq, float cutoffFreq, float q = 0.707f);
    
    // 获取参数
    float getKp() const { return m_pid.config.kp; }
    float getKi() const { return m_pid.config.ki; }
    float getKd() const { return m_pid.config.kd; }
    float getTarget() const { return m_pid.status.target; }
    float getOutput() const { return m_pid.status.output; }
    float getError() const { return m_pid.status.error; }
    float getIntegral() const { return m_pid.status.integral; }
    float getDifferential() const { return m_pid.status.differential; }
    
    // 设置参数
    void setKp(float kp) { m_pid.config.kp = kp; }
    void setKi(float ki) { m_pid.config.ki = ki; }
    void setKd(float kd) { m_pid.config.kd = kd; }
    void setParams(float kp, float ki, float kd);
    
    // 访问内部结构
    const PidController_t* getController() const { return &m_pid; }
    PidController_t* getController() { return &m_pid; }
};

#endif // __cplusplus

#endif // PID_H