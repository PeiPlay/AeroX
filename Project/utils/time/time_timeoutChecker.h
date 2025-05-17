#ifndef __TIME_TIMEOUT_CHECKER_H__
#define __TIME_TIMEOUT_CHECKER_H__

#include "time_timestamp.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// C风格的超时检查器结构体
typedef struct {
    timestamp_t start_time;    // 开始时间点
    timestamp_t timeout_value; // 超时时间长度
} TimeoutChecker_t;

// 初始化超时检查器
void TimeoutChecker_Init(TimeoutChecker_t* checker, timestamp_t timeout);

// 设置超时时间
void TimeoutChecker_SetTimeout(TimeoutChecker_t* checker, timestamp_t timeout);
void TimeoutChecker_SetTimeoutMicroseconds(TimeoutChecker_t* checker, uint64_t us);
void TimeoutChecker_SetTimeoutMilliseconds(TimeoutChecker_t* checker, uint64_t ms);
void TimeoutChecker_SetTimeoutSeconds(TimeoutChecker_t* checker, uint64_t s);
void TimeoutChecker_SetTimeoutSecondsFloat(TimeoutChecker_t* checker, float s);

// 重置检查器（重新开始计时）
void TimeoutChecker_Reset(TimeoutChecker_t* checker);

// 检查是否已超时
int TimeoutChecker_IsTimeout(TimeoutChecker_t* checker);

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
namespace utils {
namespace time {

// C++风格的超时检查器类
class TimeoutChecker {
private:
    timestamp_t start_time_;    // 开始时间点
    timestamp_t timeout_value_; // 超时时间长度

public:
    // 构造函数
    TimeoutChecker(timestamp_t timeout = 0);
    
    // 设置超时时间
    void setTimeout(timestamp_t timeout);
    void setTimeoutMicroseconds(uint64_t us);
    void setTimeoutMilliseconds(uint64_t ms);
    void setTimeoutSeconds(uint64_t s);
    void setTimeoutSecondsFloat(float s);
    
    // 重置检查器（重新开始计时）
    void reset();
    
    // 检查是否已超时
    bool isTimeout() const;
};
    
} // namespace time
} // namespace utils
#endif // __cplusplus

#endif // __TIME_TIMEOUT_CHECKER_H__
