#ifndef __TIME_DELAY_H__
#define __TIME_DELAY_H__

#include "time_timestamp.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 基本延时函数
void TimeDelay_Microseconds(uint64_t us);
void TimeDelay_Milliseconds(uint64_t ms);
void TimeDelay_Seconds(uint64_t s);

// 浮点数延时函数
void TimeDelay_SecondsFloat(float s);

// 基于时间戳的延时函数
void TimeDelay_UntilTimestamp(timestamp_t target_timestamp);

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
namespace utils {
namespace time {

class Delay {
public:
    // 基本延时函数
    static void microseconds(uint64_t us);
    static void milliseconds(uint64_t ms);
    static void seconds(uint64_t s);
    
    // 浮点数延时函数
    static void secondsFloat(float s);
    
    // 基于时间戳的延时函数
    static void untilTimestamp(timestamp_t target_timestamp);
};

} // namespace time
} // namespace utils
#endif // __cplusplus

#endif // __TIME_DELAY_H__