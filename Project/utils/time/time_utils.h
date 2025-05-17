#ifndef __TIME_TIME_UTILS_H__
#define __TIME_TIME_UTILS_H__

#include <stdint.h> 
#include "time_delay.h"
#include "time_timeoutChecker.h"
#include "time_timestamp.h"
#include "time_watch.h"

#ifdef __cplusplus
extern "C" {
#endif

uint64_t TimeUtils_GetGlobalTick(void);

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
namespace utils {
namespace time {

// 获取全局时钟函数
uint64_t getGlobalTick();

} // namespace time
} // namespace utils
#endif // __cplusplus


#endif // __TIME_TIME_UTILS_H__