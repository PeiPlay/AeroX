#ifndef CHASSIS_H
#define CHASSIS_H

#include "motor.h"
#include "pid.h"
#include "Attitude.h"
#include <stdint.h>

// 定义电机索引，便于理解
#define MOTOR_FRONT_RIGHT 0 // 电机1 (右上)
#define MOTOR_FRONT_LEFT  1 // 电机2 (左上)
#define MOTOR_REAR_LEFT   2 // 电机3 (左下)
#define MOTOR_REAR_RIGHT  3 // 电机4 (右下)

// 定义PID索引
#define PID_ROLL_ANGLE  0
#define PID_PITCH_ANGLE 1
#define PID_YAW_ANGLE   2 // 通常控制角速度，但保留角度PID可能性
#define PID_ROLL_RATE   0
#define PID_PITCH_RATE  1
#define PID_YAW_RATE    2

/**
 * @brief 油门控制模式
 */
enum class ThrottleMode {
    DIRECT,   // 直接控制油门值
    ADDITIVE  // 叠加控制油门值 (hoverThrottle + additive)
};

/**
 * @brief Chassis 外部依赖项结构体
 * @details 包含 Chassis 运行所需的外部对象指针
 */
struct ChassisDependencies {
    AttitudeManager* attitudeMgr = nullptr;
    Motor* motors[4] = {nullptr, nullptr, nullptr, nullptr};
    PidController* anglePIDs[3] = {nullptr, nullptr, nullptr}; // Roll, Pitch, Yaw Angle
    PidController* ratePIDs[3] = {nullptr, nullptr, nullptr};  // Roll, Pitch, Yaw Rate
    // 可以添加其他配置参数，例如控制循环频率等
};

/**
 * @brief 四旋翼底盘控制类
 * @details 整合姿态、电机和PID控制器，实现串级PID姿态控制
 */
class Chassis {
public:
    /**
     * @brief 内部配置结构体
     */
    struct Config {
        AttitudeManager* attitudeMgr = nullptr;
        Motor* motors[4] = {nullptr, nullptr, nullptr, nullptr};
        PidController* anglePIDs[3] = {nullptr, nullptr, nullptr}; // 外环: Roll, Pitch, Yaw Angle
        PidController* ratePIDs[3] = {nullptr, nullptr, nullptr};  // 内环: Roll, Pitch, Yaw Rate
        // 可以添加其他从外部传入的配置参数
    };

    /**
     * @brief 内部目标值结构体
     * @details 仅包含外部设定的目标值
     */
    struct Target {
        float roll = 0.0f;     // 目标横滚角 (rad)
        float pitch = 0.0f;    // 目标俯仰角 (rad)
        float yaw = 0.0f;      // 目标偏航角 (rad) - 注意：通常控制偏航角速度，但这里允许设置目标角度，由角度PID转换为目标角速度
        float throttleOverride = 0.0f; // 直接油门值 (0.0 to 100.0) - 用于直接控制模式
        float throttleAdditive = 0.0f; // 叠加油门值 (e.g., -50.0 to 50.0) - 用于叠加控制模式, 最终油门 = hoverThrottle_ + throttleAdditive
        ThrottleMode currentThrottleMode = ThrottleMode::ADDITIVE; // 当前油门控制模式, 默认为叠加模式
    };

    /**
     * @brief 内部状态结构体
     */
    struct Status {
        // 当前传感器读数 (原始IMU值)
        float imuRoll = 0.0f;    // rad, 来自AttitudeManager
        float imuPitch = 0.0f;   // rad, 来自AttitudeManager
        float imuYaw = 0.0f;     // rad, 来自AttitudeManager
        float imuGyro[3] = {0.0f, 0.0f, 0.0f}; // rad/s, [roll, pitch, yaw], 来自AttitudeManager

        // 转换到机体坐标系后的姿态角和角速度
        float chassisRoll = 0.0f;    // rad
        float chassisPitch = 0.0f;   // rad
        float chassisYaw = 0.0f;     // rad
        float chassisGyro[3] = {0.0f, 0.0f, 0.0f}; // rad/s, [roll, pitch, yaw]

        // PID 中间计算结果
        float targetRollRateCmd = 0.0f;  // 角度环输出的目标横滚角速度 (rad/s)
        float targetPitchRateCmd = 0.0f; // 角度环输出的目标俯仰角速度 (rad/s)
        float targetYawRateCmd = 0.0f;   // 角度环输出的目标偏航角速度 (rad/s)
        float rollCmd = 0.0f;            // 角速度环输出的横滚控制量 (混合前)
        float pitchCmd = 0.0f;           // 角速度环输出的俯仰控制量 (混合前)
        float yawCmd = 0.0f;             // 角速度环输出的偏航控制量 (混合前)

        // 最终输出
        float motorOutputs[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 存储计算出的电机输出值 (-100.0 to 100.0)

        // 其他状态
        bool isInitialized = false;   // 初始化标志
    };

    /**
     * @brief 构造函数
     * @param deps 外部依赖项
     */
    explicit Chassis(const ChassisDependencies& deps);

    /**
     * @brief 虚析构函数
     */
    virtual ~Chassis() = default;

    /**
     * @brief 初始化底盘
     * @details 初始化所有电机和PID控制器
     * @return 初始化是否成功
     */
    virtual bool init();

    /**
     * @brief 设置目标姿态（欧拉角）
     * @param roll 目标横滚角 (rad)
     * @param pitch 目标俯仰角 (rad)
     * @param yaw 目标偏航角 (rad)
     */
    virtual void setTargetAttitude(float roll, float pitch, float yaw);

    virtual float getTargetRoll(void);
    virtual float getTargetPitch(void);
    virtual float getTargetYaw(void);

    /**
     * @brief 设置直接控制油门值
     * @param throttle 油门值 (0.0 to 100.0)
     */
    virtual void setThrottleOverride(float throttle);

    /**
     * @brief 获取当前油门值
     * @return 油门值 (0.0 to 100.0)
     */
    virtual float getThrottleOverride(void);

    /**
     * @brief 设置叠加油门值
     * @param additiveThrottle 叠加油门值 (例如 -50.0 to 50.0)
     */
    virtual void setThrottleAdditive(float additiveThrottle);

    /**
     * @brief 设置油门控制模式
     * @param mode 油门控制模式 (DIRECT 或 ADDITIVE)
     */
    virtual void setThrottleMode(ThrottleMode mode);

    /**
     * @brief 更新底盘控制状态（应在固定频率下调用）
     * @param dt 时间增量 (s)，用于PID计算
     * @details 执行姿态读取、串级PID计算和电机输出更新
     */
    virtual void update();

    /**
     * @brief 使所有电机停转
     */
    virtual void disarm();

protected:
    Config config_; // 存储配置参数
    Target target_; // 存储目标值
    Status status_; // 存储当前状态和临时变量
    float hoverThrottle_ = 50.0f; // 叠加模式下的基础悬停油门 (0.0 to 100.0)

    /**
     * @brief 电机混合算法
     * @details 根据PID输出计算每个电机的油门值 (输出范围 -100.0 to 100.0)
     * @param throttle 基础油门 (0.0 to 100.0)
     * @param rollCmd 角速度环输出的横滚控制量
     * @param pitchCmd 角速度环输出的俯仰控制量
     * @param yawCmd 角速度环输出的偏航控制量
     */
    virtual void mixer(float throttle, float rollCmd, float pitchCmd, float yawCmd);

    /**
     * @brief 限制浮点数值在指定范围内
     */
    virtual float constrain(float value, float minVal, float maxVal);
};

#endif // CHASSIS_H
