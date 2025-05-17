#ifndef __TIME_TIMESTAMP_H__
#define __TIME_TIMESTAMP_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 定义时间戳类型为64位整数
typedef uint64_t timestamp_t;

// 时间单位转换宏定义
#define TICKS_PER_MICROSECOND(core_clock)  ((core_clock) / 1000000ULL)
#define TICKS_PER_MILLISECOND(core_clock)  ((core_clock) / 1000ULL)
#define TICKS_PER_SECOND(core_clock)       (core_clock)

// 仅保留时间戳到秒的转换函数(float版本)
float TimeStamp_ToSecondsFloat(timestamp_t timestamp);
uint64_t TimeStamp_ToMicroseconds(timestamp_t timestamp);
uint64_t TimeStamp_ToMilliseconds(timestamp_t timestamp);

// 从各时间单位到时间戳的转换函数
timestamp_t TimeStamp_FromMicroseconds(uint64_t microseconds);
timestamp_t TimeStamp_FromMilliseconds(uint64_t milliseconds);
timestamp_t TimeStamp_FromSeconds(uint64_t seconds);
timestamp_t TimeStamp_FromSecondsFloat(float seconds);

// 时间戳差值计算函数
timestamp_t TimeStamp_Diff(timestamp_t end, timestamp_t start);

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
namespace utils {
namespace time {

// C++封装的时间戳操作类
class TimeStamp {
public:
    // 获取当前时间戳
    static timestamp_t now();
    
    // 从不同时间单位创建时间戳
    static timestamp_t fromMicroseconds(uint64_t microseconds);
    static timestamp_t fromMilliseconds(uint64_t milliseconds);
    static timestamp_t fromSeconds(uint64_t seconds);
    static timestamp_t fromSecondsFloat(float seconds);
    
    // 时间戳转换为不同时间单位
    static float toSecondsFloat(timestamp_t timestamp);
    static uint64_t toMicroseconds(timestamp_t timestamp);
    static uint64_t toMilliseconds(timestamp_t timestamp);
    
    // 计算两个时间戳之间的差值
    static timestamp_t diff(timestamp_t end, timestamp_t start);
};

} // namespace time
} // namespace utils
#endif // __cplusplus

#endif // __TIME_TIMESTAMP_H__