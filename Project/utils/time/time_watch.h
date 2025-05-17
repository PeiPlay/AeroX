#ifndef __TIME_WATCH_H__
#define __TIME_WATCH_H__

#include "time_timestamp.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// C风格的计时器结构体
typedef struct {
    timestamp_t start_time;
    timestamp_t elapsed_time;
    int is_running;
} TimeWatch_t;

// 计时器初始化
void TimeWatch_Init(TimeWatch_t* watch);

// 计时器控制函数
void TimeWatch_Start(TimeWatch_t* watch);
void TimeWatch_Stop(TimeWatch_t* watch);
void TimeWatch_Reset(TimeWatch_t* watch);
void TimeWatch_Restart(TimeWatch_t* watch);

// 获取计时器状态
int TimeWatch_IsRunning(const TimeWatch_t* watch);

// 获取计时结果函数
timestamp_t TimeWatch_GetElapsedTicks(const TimeWatch_t* watch);
float TimeWatch_GetElapsedSecondsFloat(const TimeWatch_t* watch);

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
namespace utils {
namespace time {

// C++风格的计时器类
class StopWatch {
private:
    timestamp_t start_time_;
    timestamp_t elapsed_time_;
    bool is_running_;

public:
    // 构造与析构
    StopWatch(bool auto_start = false);
    ~StopWatch();

    // 计时器控制
    void start();
    void stop();
    void reset();
    void restart();

    // 状态查询
    bool isRunning() const;

    // 获取计时结果
    timestamp_t getElapsedTicks() const;
    float getElapsedSecondsFloat() const;
};

} // namespace time
} // namespace utils
#endif // __cplusplus

#endif // __TIME_WATCH_H__