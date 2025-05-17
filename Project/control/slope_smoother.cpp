#include "slope_smoother.h"
#include <cmath>

SlopeSmoother::SlopeSmoother(float maxChangePerMillisecond, float timeoutMilliseconds, float initialValue)
    : currentValue_(initialValue)
    , maxChangePerMillisecond_(maxChangePerMillisecond)
    , timeoutMilliseconds_(timeoutMilliseconds)
    , lastUpdateTime_(utils::time::TimeStamp::now())
{
}

float SlopeSmoother::update(float targetValue)
{
    // 获取当前时间戳
    timestamp_t currentTime = utils::time::TimeStamp::now();
    
    // 计算自上次更新以来的时间（毫秒）
    // 先转换为微秒，再转换为毫秒浮点数
    uint64_t deltaTimeMicroseconds = utils::time::TimeStamp::toMicroseconds(
        utils::time::TimeStamp::diff(currentTime, lastUpdateTime_));
    double deltaTimeMilliseconds = deltaTimeMicroseconds / 1000.0f;
    
    // 更新上次更新时间戳
    lastUpdateTime_ = currentTime;
    
    // 如果超时，仅更新时间戳，不调整参数
    if (deltaTimeMilliseconds > timeoutMilliseconds_ || deltaTimeMilliseconds < 0.0) {
        return currentValue_;
    }
    
    // 计算基于最大变化速率和时间差的最大可能变化量
    float maxChange = (float)(maxChangePerMillisecond_ * deltaTimeMilliseconds);
    
    // 计算目标值与当前值的差异
    float diff = targetValue - currentValue_;
    
    // 如果差异在最大可能变化范围内，则直接设置为目标值
    if (std::abs(diff) <= maxChange) {
        currentValue_ = targetValue;
    } 
    // 否则按最大变化量向目标值方向移动
    else {
        if (diff > 0) {
            currentValue_ += maxChange;
        } else {
            currentValue_ -= maxChange;
        }
    }
    
    return currentValue_;
}

float SlopeSmoother::getCurrentValue() const
{
    return currentValue_;
}

void SlopeSmoother::setCurrentValue(float value)
{
    currentValue_ = value;
    // 重置时间戳以确保下一次更新的时间差计算正确
    lastUpdateTime_ = utils::time::TimeStamp::now();
}

void SlopeSmoother::setMaxChangePerMillisecond(float maxChangePerMillisecond)
{
    maxChangePerMillisecond_ = maxChangePerMillisecond;
}

float SlopeSmoother::getMaxChangePerMillisecond() const
{
    return maxChangePerMillisecond_;
}

void SlopeSmoother::setTimeoutMilliseconds(float timeoutMilliseconds)
{
    timeoutMilliseconds_ = timeoutMilliseconds;
}

float SlopeSmoother::getTimeoutMilliseconds() const
{
    return timeoutMilliseconds_;
}
