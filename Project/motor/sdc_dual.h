#ifndef __SDC_DUAL_H__
#define __SDC_DUAL_H__

#include "main.h"
#include "tim.h"
#include "motor.h"

class SdcDualMotor : public Motor {
public:
    /**
     * @brief SdcDualMotor 构造函数
     * @param htim 定时器句柄指针
     * @param tim_channel 定时器通道 (例如 TIM_CHANNEL_1)
     * @param reversed 电机是否需要反向。默认为 false。
     */
    SdcDualMotor(TIM_HandleTypeDef* htim, uint32_t tim_channel, bool reversed = false);

    /**
     * @brief 初始化电机控制器
     *        设置PWM到油门中点 (1500us)，启动PWM并等待0.5秒。
     */
    void init() override;

    /**
     * @brief 设置电机油门
     * @param throttle 油门值，范围从 -100.0 (反向最大) 到 100.0 (正向最大)
     */
    void setThrottle(float throttle) override;

    /**
     * @brief 设置油门限制因子
     * @param factor 限制因子，范围 [0.0, 1.0]。0.0 表示完全关闭，1.0 表示允许达到最大油门。
     */
    void setThrottleLimitFactor(float factor);

    /**
     * @brief 获取当前油门限制因子
     * @return 当前油门限制因子
     */
    float getThrottleLimitFactor() const;

    /**
     * @brief 获取当前PWM比较值
     * @return 当前PWM比较值
     */
    uint32_t getCurrentCompareValue() const;

private:
    TIM_HandleTypeDef* htim_;
    uint32_t tim_channel_;
    uint32_t timer_arr_val_; // 存储 htim_->Instance->ARR 的值
    float throttle_limit_factor_ = 0.5f; // 油门限制因子，范围 [0.0, 1.0]
    uint32_t current_compare_value_; // 当前设置的PWM比较值
};

#endif // __SDC_DUAL_H__