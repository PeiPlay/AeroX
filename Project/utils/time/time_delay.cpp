#include "time_delay.h"
#include "time_utils.h"
#include "time_timestamp.h"

#ifdef __cplusplus
extern "C" {
#endif

void TimeDelay_UntilTimestamp(timestamp_t target_timestamp) {
    timestamp_t current_timestamp;
    
    do {
        current_timestamp = TimeUtils_GetGlobalTick();
    } while (current_timestamp < target_timestamp);
}

void TimeDelay_Microseconds(uint64_t us) {
    timestamp_t start_timestamp = TimeUtils_GetGlobalTick();
    timestamp_t ticks_to_wait = TimeStamp_FromMicroseconds(us);
    timestamp_t target_timestamp = start_timestamp + ticks_to_wait;
    
    TimeDelay_UntilTimestamp(target_timestamp);
}

void TimeDelay_Milliseconds(uint64_t ms) {
    timestamp_t start_timestamp = TimeUtils_GetGlobalTick();
    timestamp_t ticks_to_wait = TimeStamp_FromMilliseconds(ms);
    timestamp_t target_timestamp = start_timestamp + ticks_to_wait;
    
    TimeDelay_UntilTimestamp(target_timestamp);
}

void TimeDelay_Seconds(uint64_t s) {
    timestamp_t start_timestamp = TimeUtils_GetGlobalTick();
    timestamp_t ticks_to_wait = TimeStamp_FromSeconds(s);
    timestamp_t target_timestamp = start_timestamp + ticks_to_wait;
    
    TimeDelay_UntilTimestamp(target_timestamp);
}

void TimeDelay_SecondsFloat(float s) {
    timestamp_t start_timestamp = TimeUtils_GetGlobalTick();
    timestamp_t ticks_to_wait = TimeStamp_FromSecondsFloat(s);
    timestamp_t target_timestamp = start_timestamp + ticks_to_wait;
    
    TimeDelay_UntilTimestamp(target_timestamp);
}

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
namespace utils {
namespace time {

void Delay::untilTimestamp(timestamp_t target_timestamp) {
    TimeDelay_UntilTimestamp(target_timestamp);
}

void Delay::microseconds(uint64_t us) {
    TimeDelay_Microseconds(us);
}

void Delay::milliseconds(uint64_t ms) {
    TimeDelay_Milliseconds(ms);
}

void Delay::seconds(uint64_t s) {
    TimeDelay_Seconds(s);
}

void Delay::secondsFloat(float s) {
    TimeDelay_SecondsFloat(s);
}

} // namespace time
} // namespace utils
#endif // __cplusplus