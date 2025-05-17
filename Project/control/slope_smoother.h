#ifndef __SLOPE_SMOOTHER_H__
#define __SLOPE_SMOOTHER_H__

#include "time_timestamp.h"

class SlopeSmoother {
public:
    // 构造函数，允许指定初始值（默认为0）、每毫秒最大变化率和超时时间
    SlopeSmoother(float maxChangePerMillisecond, float timeoutMilliseconds, float initialValue = 0.0f);
    
    // 更新函数 - 根据目标值和时间差计算平滑后的值
    float update(float targetValue);
    
    // 获取当前值
    float getCurrentValue() const;
    
    // 直接设置当前值（忽略斜率限制）
    void setCurrentValue(float value);
    
    // 设置/获取每毫秒最大变化量
    void setMaxChangePerMillisecond(float maxChangePerMillisecond);
    float getMaxChangePerMillisecond() const;
    
    // 设置/获取超时时间
    void setTimeoutMilliseconds(float timeoutMilliseconds);
    float getTimeoutMilliseconds() const;
    
private:
    float currentValue_;           // 当前值
    float maxChangePerMillisecond_;     // 每毫秒最大变化量
    float timeoutMilliseconds_;         // 超时时间（毫秒）
    timestamp_t lastUpdateTime_;        // 上一次更新的时间戳
};

#endif // __SLOPE_SMOOTHER_H__
