#include "time_watch.h"
#include "time_timestamp.h"
#include "time_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

void TimeWatch_Init(TimeWatch_t* watch) {
    if (watch) {
        watch->start_time = 0;
        watch->elapsed_time = 0;
        watch->is_running = 0;
    }
}

void TimeWatch_Start(TimeWatch_t* watch) {
    if (watch && !watch->is_running) {
        watch->start_time = TimeUtils_GetGlobalTick();
        watch->is_running = 1;
    }
}

void TimeWatch_Stop(TimeWatch_t* watch) {
    if (watch && watch->is_running) {
        timestamp_t current_time = TimeUtils_GetGlobalTick();
        watch->elapsed_time += (current_time - watch->start_time);
        watch->is_running = 0;
    }
}

void TimeWatch_Reset(TimeWatch_t* watch) {
    if (watch) {
        watch->elapsed_time = 0;
        if (watch->is_running) {
            watch->start_time = TimeUtils_GetGlobalTick();
        }
    }
}

void TimeWatch_Restart(TimeWatch_t* watch) {
    if (watch) {
        watch->elapsed_time = 0;
        watch->start_time = TimeUtils_GetGlobalTick();
        watch->is_running = 1;
    }
}

int TimeWatch_IsRunning(const TimeWatch_t* watch) {
    return watch ? watch->is_running : 0;
}

timestamp_t TimeWatch_GetElapsedTicks(const TimeWatch_t* watch) {
    if (!watch) {
        return 0;
    }

    timestamp_t elapsed = watch->elapsed_time;
    if (watch->is_running) {
        timestamp_t current_time = TimeUtils_GetGlobalTick();
        elapsed += (current_time - watch->start_time);
    }
    
    return elapsed;
}

float TimeWatch_GetElapsedSecondsFloat(const TimeWatch_t* watch) {
    return TimeStamp_ToSecondsFloat(TimeWatch_GetElapsedTicks(watch));
}

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
namespace utils {
namespace time {

StopWatch::StopWatch(bool auto_start) 
    : start_time_(0), elapsed_time_(0), is_running_(false) {
    if (auto_start) {
        start();
    }
}

StopWatch::~StopWatch() {
    // 析构函数不需要特殊处理
}

void StopWatch::start() {
    if (!is_running_) {
        start_time_ = TimeUtils_GetGlobalTick();
        is_running_ = true;
    }
}

void StopWatch::stop() {
    if (is_running_) {
        timestamp_t current_time = TimeUtils_GetGlobalTick();
        elapsed_time_ += (current_time - start_time_);
        is_running_ = false;
    }
}

void StopWatch::reset() {
    elapsed_time_ = 0;
    if (is_running_) {
        start_time_ = TimeUtils_GetGlobalTick();
    }
}

void StopWatch::restart() {
    elapsed_time_ = 0;
    start_time_ = TimeUtils_GetGlobalTick();
    is_running_ = true;
}

bool StopWatch::isRunning() const {
    return is_running_;
}

timestamp_t StopWatch::getElapsedTicks() const {
    timestamp_t elapsed = elapsed_time_;
    if (is_running_) {
        timestamp_t current_time = TimeUtils_GetGlobalTick();
        elapsed += (current_time - start_time_);
    }
    
    return elapsed;
}

float StopWatch::getElapsedSecondsFloat() const {
    return TimeStamp::toSecondsFloat(getElapsedTicks());
}

} // namespace time
} // namespace utils 
#endif // __cplusplus