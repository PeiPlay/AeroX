#include "motor.h"
#include <algorithm> // 用于 std::clamp (C++17) 或手动实现

Motor::Motor(bool reversed)
    : reversed_(reversed), currentThrottle_(0.0f) {}

float Motor::getThrottle() const {
    return currentThrottle_;
}

float Motor::adjustThrottleForDirection(float throttle) {
    // 限制油门值在 -100.0 到 100.0 之间
    // C++17: throttle = std::clamp(throttle, -100.0f, 100.0f);
    // C++11 或更早:
    throttle = std::max(-100.0f, std::min(100.0f, throttle));

    // 如果电机需要反向，则将油门值取反
    if (reversed_) {
        throttle = -throttle;
    }
    return throttle;
}

// 注意：纯虚函数 init() 和 setThrottle() 没有在这里实现，
// 它们需要在派生类中具体实现。
