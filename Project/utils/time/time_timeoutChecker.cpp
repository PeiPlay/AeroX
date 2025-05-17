#include "time_timeoutChecker.h"
#include "time_utils.h"
#include "time_timestamp.h"

#ifdef __cplusplus
extern "C" {
#endif

void TimeoutChecker_Init(TimeoutChecker_t* checker, timestamp_t timeout) {
    if (checker) {
        checker->start_time = TimeUtils_GetGlobalTick();
        checker->timeout_value = timeout;
    }
}

void TimeoutChecker_SetTimeout(TimeoutChecker_t* checker, timestamp_t timeout) {
    if (checker) {
        checker->timeout_value = timeout;
    }
}

void TimeoutChecker_SetTimeoutMicroseconds(TimeoutChecker_t* checker, uint64_t us) {
    if (checker) {
        checker->timeout_value = TimeStamp_FromMicroseconds(us);
    }
}

void TimeoutChecker_SetTimeoutMilliseconds(TimeoutChecker_t* checker, uint64_t ms) {
    if (checker) {
        checker->timeout_value = TimeStamp_FromMilliseconds(ms);
    }
}

void TimeoutChecker_SetTimeoutSeconds(TimeoutChecker_t* checker, uint64_t s) {
    if (checker) {
        checker->timeout_value = TimeStamp_FromSeconds(s);
    }
}

void TimeoutChecker_SetTimeoutSecondsFloat(TimeoutChecker_t* checker, float s) {
    if (checker) {
        checker->timeout_value = TimeStamp_FromSecondsFloat(s);
    }
}

void TimeoutChecker_Reset(TimeoutChecker_t* checker) {
    if (checker) {
        checker->start_time = TimeUtils_GetGlobalTick();
    }
}

int TimeoutChecker_IsTimeout(TimeoutChecker_t* checker) {
    if (!checker) {
        return 0;
    }
    
    timestamp_t current_time = TimeUtils_GetGlobalTick();
    timestamp_t elapsed = current_time - checker->start_time;
    
    return elapsed >= checker->timeout_value;
}

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
namespace utils {
namespace time {

TimeoutChecker::TimeoutChecker(timestamp_t timeout)
    : start_time_(utils::time::getGlobalTick())
    , timeout_value_(timeout) {
}

void TimeoutChecker::setTimeout(timestamp_t timeout) {
    timeout_value_ = timeout;
}

void TimeoutChecker::setTimeoutMicroseconds(uint64_t us) {
    timeout_value_ = utils::time::TimeStamp::fromMicroseconds(us);
}

void TimeoutChecker::setTimeoutMilliseconds(uint64_t ms) {
    timeout_value_ = utils::time::TimeStamp::fromMilliseconds(ms);
}

void TimeoutChecker::setTimeoutSeconds(uint64_t s) {
    timeout_value_ = utils::time::TimeStamp::fromSeconds(s);
}

void TimeoutChecker::setTimeoutSecondsFloat(float s) {
    timeout_value_ = utils::time::TimeStamp::fromSecondsFloat(s);
}

void TimeoutChecker::reset() {
    start_time_ = utils::time::getGlobalTick();
}

bool TimeoutChecker::isTimeout() const {
    timestamp_t current_time = utils::time::getGlobalTick();
    timestamp_t elapsed = current_time - start_time_;
    
    return elapsed >= timeout_value_;
}

} // namespace time
} // namespace utils
#endif // __cplusplus
