#include "time_timestamp.h"
#include "time_utils.h"

extern uint32_t SystemCoreClock;

#ifdef __cplusplus
extern "C" {
#endif

float TimeStamp_ToSecondsFloat(timestamp_t timestamp) {
    return (float)timestamp / (float)SystemCoreClock;
}

uint64_t TimeStamp_ToMicroseconds(timestamp_t timestamp) {
    return (timestamp * 1000000ULL) / SystemCoreClock;
}

uint64_t TimeStamp_ToMilliseconds(timestamp_t timestamp) {
    return (timestamp * 1000ULL) / SystemCoreClock;
}

timestamp_t TimeStamp_FromMicroseconds(uint64_t microseconds) {
    return microseconds * SystemCoreClock / 1000000ULL;
}

timestamp_t TimeStamp_FromMilliseconds(uint64_t milliseconds) {
    return milliseconds * SystemCoreClock / 1000ULL;
}

timestamp_t TimeStamp_FromSeconds(uint64_t seconds) {
    return seconds * SystemCoreClock;
}

timestamp_t TimeStamp_FromSecondsFloat(float seconds) {
    return (timestamp_t)((float)seconds * (float)SystemCoreClock);
}

timestamp_t TimeStamp_Diff(timestamp_t end, timestamp_t start) {
    return end >= start ? end - start : 0;
}

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
namespace utils {
namespace time {

timestamp_t TimeStamp::now() {
    return TimeUtils_GetGlobalTick();
}

timestamp_t TimeStamp::fromMicroseconds(uint64_t microseconds) {
    return TimeStamp_FromMicroseconds(microseconds);
}

timestamp_t TimeStamp::fromMilliseconds(uint64_t milliseconds) {
    return TimeStamp_FromMilliseconds(milliseconds);
}

timestamp_t TimeStamp::fromSeconds(uint64_t seconds) {
    return TimeStamp_FromSeconds(seconds);
}

timestamp_t TimeStamp::fromSecondsFloat(float seconds) {
    return TimeStamp_FromSecondsFloat(seconds);
}

float TimeStamp::toSecondsFloat(timestamp_t timestamp) {
    return TimeStamp_ToSecondsFloat(timestamp);
}

uint64_t TimeStamp::toMicroseconds(timestamp_t timestamp) {
    return TimeStamp_ToMicroseconds(timestamp);
}

uint64_t TimeStamp::toMilliseconds(timestamp_t timestamp) {
    return TimeStamp_ToMilliseconds(timestamp);
}

timestamp_t TimeStamp::diff(timestamp_t end, timestamp_t start) {
    return TimeStamp_Diff(end, start);
}

} // namespace time
} // namespace utils
#endif // __cplusplus