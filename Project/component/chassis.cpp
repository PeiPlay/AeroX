#include "chassis.h"
#include <math.h>
#include <algorithm> // for std::max, std::min
#include <cstring>   // for memcpy

// Chassis 类的构造函数实现
Chassis::Chassis(const ChassisDependencies& deps) :
    config_(), // 使用默认构造函数初始化 config_
    target_(), // 使用默认构造函数初始化 target_
    status_()  // 使用默认构造函数初始化 status_
{
    // 从外部依赖项拷贝指针到内部 config_ 结构体
    config_.attitudeMgr = deps.attitudeMgr;
    memcpy(config_.motors, deps.motors, sizeof(config_.motors));
    memcpy(config_.anglePIDs, deps.anglePIDs, sizeof(config_.anglePIDs));
    memcpy(config_.ratePIDs, deps.ratePIDs, sizeof(config_.ratePIDs));
    // config_.altitudePID 和 config_.hoverThrottle 已移除

    // target_ 和 status_ 的成员已通过默认初始化设置为0或false
    // target_.currentThrottleMode 默认为 ThrottleMode::ADDITIVE
}

bool Chassis::init() {
    status_.isInitialized = false; // Default to not initialized

    if (!config_.attitudeMgr) {
        return false; // Attitude manager is essential
    }

    for (int i = 0; i < 4; ++i) {
        if (!config_.motors[i]) {
            return false; // All motors must be valid
        }
    }

    for (int i = 0; i < 3; ++i) {
        if (!config_.anglePIDs[i]) {
            // Depending on control strategy, some PIDs might be optional.
            // For a standard setup, all angle PIDs (Roll, Pitch, Yaw) are usually required.
            // If Yaw angle PID is optional (e.g., direct yaw rate control), this check might be adjusted.
            // Assuming all three are required for now.
            return false; 
        }
        if (!config_.ratePIDs[i]) {
            return false; // All rate PIDs are essential
        }
    }

    // All checks passed
    status_.isInitialized = true;
    return true;
}

void Chassis::setTargetAttitude(float roll, float pitch, float yaw) {
    target_.roll = roll;
    target_.pitch = pitch;
    target_.yaw = yaw;
    // 注意：这里仅设置目标角度，具体的角速度目标将在 update() 中由角度PID计算得出
}

void Chassis::setThrottleOverride(float throttle) {
    target_.throttleOverride = constrain(throttle, 0.0f, 100.0f);
}

void Chassis::setThrottleAdditive(float additiveThrottle) {
    // 叠加油门的范围可以根据需要调整，这里假设一个范围，例如 -hoverThrottle_ 到 (100.0f - hoverThrottle_)
    // 或者更简单地，不在此处限制，而是在最终计算时限制总油门
    target_.throttleAdditive = additiveThrottle;
}

void Chassis::setThrottleMode(ThrottleMode mode) {
    target_.currentThrottleMode = mode;
}

// setBaseThrottle, setTargetAltitude, updateCurrentAltitude, setAltitudeControlActive 方法已移除

void Chassis::update() {
    if (!status_.isInitialized || !config_.attitudeMgr) {
        // 必要组件未初始化，则不执行更新
        return;
    }

    // 1. 读取当前姿态和角速度
    // 从AttitudeManager获取的是IMU直接输出的角度和角速度
    config_.attitudeMgr->getAttitude(status_.imuRoll, status_.imuPitch, status_.imuYaw);
    config_.attitudeMgr->getGyro(status_.imuGyro);

    // 根据chassis.md的定义，将IMU角度转换为机体坐标系角度
    // phi_body (roll) = +IMU.roll
    // theta_body (pitch) = -IMU.pitch
    // psi_body (yaw) = -IMU.yaw
    status_.chassisRoll = status_.imuRoll;
    status_.chassisPitch = -status_.imuPitch;
    status_.chassisYaw = -status_.imuYaw;

    // 同样地转换角速度
    // chassisRollRate = +imuRollRate
    // chassisPitchRate = -imuPitchRate
    // chassisYawRate = -imuYawRate
    status_.chassisGyro[0] = status_.imuGyro[0]; // Roll rate
    status_.chassisGyro[1] = -status_.imuGyro[1]; // Pitch rate
    status_.chassisGyro[2] = -status_.imuGyro[2]; // Yaw rate
    // 高度读取和状态更新已移除

    // 2. 外环PID (角度环) -> 目标角速度
    // 使用转换后的机体坐标系角度作为PID的当前值
    if (config_.anglePIDs[PID_ROLL_ANGLE]) {
        status_.targetRollRateCmd = config_.anglePIDs[PID_ROLL_ANGLE]->update(target_.roll, status_.chassisRoll);
    }
    if (config_.anglePIDs[PID_PITCH_ANGLE]) {
        status_.targetPitchRateCmd = config_.anglePIDs[PID_PITCH_ANGLE]->update(target_.pitch, status_.chassisPitch);
    }
    if (config_.anglePIDs[PID_YAW_ANGLE]) { 
        status_.targetYawRateCmd = config_.anglePIDs[PID_YAW_ANGLE]->update(target_.yaw, status_.chassisYaw);
    }


    // 3. 内环PID (角速度环) -> 控制指令
    // 使用转换后的机体坐标系角速度作为PID的当前值
    if (config_.ratePIDs[PID_ROLL_RATE]) {
        status_.rollCmd = config_.ratePIDs[PID_ROLL_RATE]->update(status_.targetRollRateCmd, status_.chassisGyro[0]);
    }
    if (config_.ratePIDs[PID_PITCH_RATE]) {
        status_.pitchCmd = config_.ratePIDs[PID_PITCH_RATE]->update(status_.targetPitchRateCmd, status_.chassisGyro[1]);
    }
    if (config_.ratePIDs[PID_YAW_RATE]) {
        status_.yawCmd = config_.ratePIDs[PID_YAW_RATE]->update(status_.targetYawRateCmd, status_.chassisGyro[2]);
    }

    // 4. 计算油门输入
    float throttleInputForMixer = 0.0f;
    if (target_.currentThrottleMode == ThrottleMode::DIRECT) {
        throttleInputForMixer = target_.throttleOverride;
    } else { // ThrottleMode::ADDITIVE
        throttleInputForMixer = hoverThrottle_ + target_.throttleAdditive;
    }

    // 限制总油门在0.0到100.0之间
    throttleInputForMixer = constrain(throttleInputForMixer, 0.0f, 100.0f);

    // 5. 电机混合
    mixer(throttleInputForMixer, status_.rollCmd, status_.pitchCmd, status_.yawCmd);

    // 6. 应用电机输出
    for (int i = 0; i < 4; ++i) {
        if (config_.motors[i]) {
            config_.motors[i]->setThrottle(status_.motorOutputs[i]);
        }
    }
}

void Chassis::disarm() {
    for (int i = 0; i < 4; ++i) {
        if (config_.motors[i]) {
            config_.motors[i]->setThrottle(0.0f); // Set throttle to 0 for disarming
        }
        status_.motorOutputs[i] = 0.0f; // Also clear internal status
    }
    // Optionally, reset PID controllers if needed
    // for (int i = 0; i < 3; ++i) {
    //     if (config_.anglePIDs[i]) config_.anglePIDs[i]->reset();
    //     if (config_.ratePIDs[i]) config_.ratePIDs[i]->reset();
    // }
}

void Chassis::mixer(float throttle, float rollCmd, float pitchCmd, float yawCmd) {
    // 确保基础油门在有效范围内 (0.0 to 100.0)
    throttle = constrain(throttle, 0.0f, 100.0f);

    // PID的输出 rollCmd, pitchCmd, yawCmd 也需要被缩放到合适的范围与油门叠加。
    // 假设 PID 输出已经是合适的比例，可以直接加减。
    // throttle 参数现在直接是 0-100 的尺度。

    // 新的电机索引和混合逻辑 (参考 chassis.md 和标准 X 型四旋翼)
    // MOTOR_FRONT_RIGHT (0) -> 电机1 (右上, CCW)
    // MOTOR_FRONT_LEFT  (1) -> 电机2 (左上, CW)
    // MOTOR_REAR_LEFT   (2) -> 电机3 (左下, CCW)
    // MOTOR_REAR_RIGHT  (3) -> 电机4 (右下, CW)

    // 正 rollCmd: 右滚 (右侧下沉, 左侧上升)
    // 正 pitchCmd: 机头向上 (前侧上升, 后侧下沉)
    // 正 yawCmd: 顺时针偏航 (机头向右)

    float motor_fr, motor_fl, motor_rl, motor_rr;

    motor_fr = throttle - rollCmd + pitchCmd + yawCmd; // M0 (Motor 1)
    motor_fl = throttle + rollCmd + pitchCmd - yawCmd; // M1 (Motor 2)
    motor_rl = throttle + rollCmd - pitchCmd + yawCmd; // M2 (Motor 3)
    motor_rr = throttle - rollCmd - pitchCmd - yawCmd; // M3 (Motor 4)

    // 限制每个电机的输出在 20.0 到 100.0 之间，以匹配 motor.h 的 setThrottle 范围
    status_.motorOutputs[MOTOR_FRONT_RIGHT] = constrain(motor_fr, 0.0f, 100.0f);
    status_.motorOutputs[MOTOR_FRONT_LEFT]  = constrain(motor_fl, 0.0f, 100.0f);
    status_.motorOutputs[MOTOR_REAR_LEFT]   = constrain(motor_rl, 0.0f, 100.0f);
    status_.motorOutputs[MOTOR_REAR_RIGHT]  = constrain(motor_rr, 0.0f, 100.0f);
}


// 限制浮点数值在指定范围内
float Chassis::constrain(float value, float minVal, float maxVal) {
    return std::max(minVal, std::min(value, maxVal));
}

// ... 其他方法的具体实现将在此处添加 ...
