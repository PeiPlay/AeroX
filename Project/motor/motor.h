#ifndef MOTOR_H
#define MOTOR_H

class Motor {
public:
    /**
     * @brief Motor 构造函数
     * @param reversed 电机是否需要反向。默认为 false。
     */
    explicit Motor(bool reversed = false);

    /**
     * @brief Motor 虚析构函数
     */
    virtual ~Motor() = default;

    /**
     * @brief 初始化电机控制器（例如 PWM）
     */
    virtual void init() = 0;

    /**
     * @brief 设置电机油门
     * @param throttle 油门值，范围从 -100.0 (反向最大) 到 100.0 (正向最大)
     */
    virtual void setThrottle(float throttle) = 0;

    /**
     * @brief 获取当前设置的油门值（考虑反向）
     * @return 当前油门值
     */
    float getThrottle() const;

protected:
    bool reversed_; // 电机是否反向安装
    float currentThrottle_; // 当前设定的油门值 (-100.0 to 100.0)

    /**
     * @brief 内部函数，根据 reversed_ 标志调整实际油门值
     * @param throttle 输入油门值 (-100.0 to 100.0)
     * @return 调整后的油门值 (-100.0 to 100.0)
     */
    float adjustThrottleForDirection(float throttle);
};

#endif // MOTOR_H
