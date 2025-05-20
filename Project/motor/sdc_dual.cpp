#include "sdc_dual.h"
#include <algorithm> // 用于 std::max 和 std::min

// PWM 参数常量
static constexpr float BASE_PULSE_US = 1500.0f;
static constexpr float CONTROL_RANGE_US = 500.0f;
static constexpr float MIN_PULSE_US = BASE_PULSE_US - CONTROL_RANGE_US; // 1000.0f
static constexpr float MAX_PULSE_US = BASE_PULSE_US + CONTROL_RANGE_US; // 2000.0f
static constexpr float PWM_PERIOD_US = 2000.0f;                         // PWM 周期为 2ms = 2000us


SdcDualMotor::SdcDualMotor(TIM_HandleTypeDef *htim, uint32_t tim_channel, bool reversed, float throttle_limit)
    : Motor(reversed), htim_(htim), tim_channel_(tim_channel), timer_arr_val_(0), throttle_limit_factor_(throttle_limit), current_compare_value_(0)
{
    // timer_arr_val_ 将在 init() 中获取
}

void SdcDualMotor::init()
{
    if (htim_ == nullptr)
    {
        // 处理错误：定时器句柄未初始化
        return;
    }
    timer_arr_val_ = htim_->Instance->ARR;
    if (timer_arr_val_ == 0)
    {
        // 处理错误：ARR值为0，可能导致除零错误或PWM不工作
        return;
    }

    // 设置PWM到50%占空比
    // 脉冲宽度 = PWM周期 / 2
    float initial_pulse_us = PWM_PERIOD_US / 2.0f;
    current_compare_value_ = static_cast<uint32_t>((initial_pulse_us / PWM_PERIOD_US) * (static_cast<float>(timer_arr_val_) + 1.0f));

    __HAL_TIM_SET_COMPARE(htim_, tim_channel_, current_compare_value_);
    HAL_TIM_PWM_Start(htim_, tim_channel_);

}

void SdcDualMotor::setThrottle(float throttle)
{
    if (htim_ == nullptr || timer_arr_val_ == 0)
    {
        // 处理错误：定时器句柄未初始化或ARR无效
        return;
    }

    this->currentThrottle_ = throttle; // 存储用户请求的原始油门值

    // 根据电机方向调整油门值，并限制在 [-100.0, 100.0]
    float effective_throttle = adjustThrottleForDirection(throttle);

    // 应用油门限制因子，仅限制最大输出，不进行整体缩放
    // effective_throttle 此时在 [-100.0, 100.0] 范围内
    float max_limited_throttle = 100.0f * throttle_limit_factor_;
    effective_throttle = std::max(-max_limited_throttle, std::min(max_limited_throttle, effective_throttle));
    // effective_throttle 现在被钳位在 [-100.0 * throttle_limit_factor_, 100.0 * throttle_limit_factor_]

    // 将调整并限制后的油门值映射到控制脉冲宽度
    // effective_throttle 的范围是 [-100.0 * throttle_limit_factor_, 100.0 * throttle_limit_factor_]
    // control_us 的范围将是 [-CONTROL_RANGE_US * throttle_limit_factor_, CONTROL_RANGE_US * throttle_limit_factor_]
    float control_us = effective_throttle * (CONTROL_RANGE_US / 100.0f);

    // 计算目标脉冲宽度
    float target_pulse_us = BASE_PULSE_US + control_us;

    // 钳位目标脉冲宽度到硬件支持的绝对 [MIN_PULSE_US, MAX_PULSE_US] 范围
    // 这一步确保即使计算结果因某种原因超出理论范围，也不会损坏设备
    target_pulse_us = std::max(MIN_PULSE_US, std::min(MAX_PULSE_US, target_pulse_us));

    // 计算PWM比较值
    // 比较值 = (脉冲宽度 / PWM周期) * (ARR + 1)
    current_compare_value_ = static_cast<uint32_t>((target_pulse_us / PWM_PERIOD_US) * (static_cast<float>(timer_arr_val_) + 1.0f));

    __HAL_TIM_SET_COMPARE(htim_, tim_channel_, current_compare_value_);
}

void SdcDualMotor::setThrottleLimitFactor(float factor)
{
    // 钳位因子到 [0.0, 1.0]
    throttle_limit_factor_ = std::max(0.0f, std::min(1.0f, factor));
}

float SdcDualMotor::getThrottleLimitFactor() const
{
    return throttle_limit_factor_;
}

uint32_t SdcDualMotor::getCurrentCompareValue() const
{
    return current_compare_value_;
}
