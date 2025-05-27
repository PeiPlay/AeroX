#include "move.h"
#include <math.h>
#include <algorithm>
#include <cstring>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Move 类的构造函数实现
Move::Move(const MoveDependencies& deps) :
    config_(),
    target_(),
    status_()
{
    // 从外部依赖项拷贝指针到内部 config_ 结构体
    config_.lidar = deps.lidar;
    memcpy(config_.positionPIDs, deps.positionPIDs, sizeof(config_.positionPIDs));
    memcpy(config_.velocityPIDs, deps.velocityPIDs, sizeof(config_.velocityPIDs));
}

bool Move::init() {
    status_.isInitialized = false;

    if (!config_.lidar) {
        return false; // 雷达是必需的
    }

    for (int i = 0; i < 3; ++i) {
        if (!config_.positionPIDs[i]) {
            return false; // 所有位置PID都是必需的
        }
        if (!config_.velocityPIDs[i]) {
            return false; // 所有速度PID都是必需的
        }
    }

    // // 初始化雷达
    // if (!config_.lidar->init()) {
    //     return false;
    // }

    // 重置目标值
    target_.x_ground = 0.0f;
    target_.y_ground = 0.0f;
    target_.z_ground = 0.0f;
    target_.vx = 0.0f;
    target_.vy = 0.0f;
    target_.vz = 0.0f;
    target_.use_position_control = true;

    // 重置状态
    memset(&status_, 0, sizeof(Status));
    status_.isInitialized = true;
    
    return true;
}

void Move::setTargetPosition(float x, float y, float z) {
    target_.x_ground = x;
    target_.y_ground = y;
    target_.z_ground = z;
    target_.use_position_control = true;
}

void Move::setTargetVelocity(float vx, float vy, float vz) {
    target_.vx = vx;
    target_.vy = vy;
    target_.vz = vz;
    target_.use_position_control = false;
}

void Move::setPositionControlMode(bool use_position_control) {
    target_.use_position_control = use_position_control;
}

void Move::getCurrentPosition(float& x, float& y, float& z) const {
    x = status_.currentX_ground;
    y = status_.currentY_ground;
    z = status_.currentZ_ground;
}

void Move::getCurrentVelocity(float& vx, float& vy, float& vz) const {
    vx = status_.currentVx_ground;
    vy = status_.currentVy_ground;
    vz = status_.currentVz_ground;
}


void Move::getPositionErrorBody(float& ex, float& ey, float& ez) const {
    ex = status_.errorX_body;
    ey = status_.errorY_body;
    ez = status_.errorZ_body;
}

void Move::getCurrentVelocityBody(float& vx, float& vy, float& vz) const {
    vx = status_.currentVx_body;
    vy = status_.currentVy_body;
    vz = status_.currentVz_body;
}

void Move::getAttitudeCommand(float& roll, float& pitch, float& throttle) const {
    roll = status_.rollCmd;
    pitch = status_.pitchCmd;
    throttle = status_.throttleCmd;
}

void Move::update() {
    if (!status_.isInitialized || !config_.lidar) {
        return;
    }

    // 1. 更新传感器数据
    updateSensorData();

    // 2. 更新位置误差的坐标转换
    updatePositionErrorTransform();

    // 3. 外环PID (位置环) -> 目标速度 (使用机身坐标系下的位置误差)
    if (target_.use_position_control) {
        // 位置控制模式 - 使用机身坐标系下的位置误差
        if (config_.positionPIDs[PID_X_POSITION]) {
            status_.targetVxCmd = config_.positionPIDs[PID_X_POSITION]->update(
                status_.errorX_body, 0.0f);
        }
        if (config_.positionPIDs[PID_Y_POSITION]) {
            status_.targetVyCmd = config_.positionPIDs[PID_Y_POSITION]->update(
                status_.errorY_body, 0.0f);
        }
        if (config_.positionPIDs[PID_Z_POSITION]) {
            status_.targetVzCmd = config_.positionPIDs[PID_Z_POSITION]->update(
                status_.errorZ_body, 0.0f);
        }
    } else {
        // 速度控制模式，直接使用目标速度 (机身坐标系)
        status_.targetVxCmd = target_.vx;
        status_.targetVyCmd = target_.vy;
        status_.targetVzCmd = target_.vz;
    }

    // 4. 内环PID (速度环) -> 姿态指令 (使用机身坐标系速度)
    // X速度控制 -> 俯仰角指令 (机体坐标系中，X正方向对应机头前进)
    if (config_.velocityPIDs[PID_X_VELOCITY]) {
        status_.pitchCmd = config_.velocityPIDs[PID_X_VELOCITY]->update(
            status_.targetVxCmd, status_.currentVx_body);
    }

    // Y速度控制 -> 横滚角指令 (机体坐标系中，Y正方向对应机体右侧)
    if (config_.velocityPIDs[PID_Y_VELOCITY]) {
        status_.rollCmd = config_.velocityPIDs[PID_Y_VELOCITY]->update(
            status_.targetVyCmd, status_.currentVy_body);
    }

    // Z速度控制 -> 油门指令
    if (config_.velocityPIDs[PID_Z_VELOCITY]) {
        status_.throttleCmd = config_.velocityPIDs[PID_Z_VELOCITY]->update(
            status_.targetVzCmd, status_.currentVz_body);
    }

    // 5. 限制输出范围
    status_.rollCmd = constrain(status_.rollCmd, -M_PI/6, M_PI/6);   // ±30度
    status_.pitchCmd = constrain(status_.pitchCmd, -M_PI/6, M_PI/6); // ±30度
    status_.throttleCmd = constrain(status_.throttleCmd, -50.0f, 50.0f); // 相对油门调整
}

void Move::reset() {
    // 重置PID控制器
    for (int i = 0; i < 3; ++i) {
        if (config_.positionPIDs[i]) {
            config_.positionPIDs[i]->reset();
        }
        if (config_.velocityPIDs[i]) {
            config_.velocityPIDs[i]->reset();
        }
    }

    // 清零输出指令
    status_.rollCmd = 0.0f;
    status_.pitchCmd = 0.0f;
    status_.throttleCmd = 0.0f;
    status_.targetVxCmd = 0.0f;
    status_.targetVyCmd = 0.0f;
    status_.targetVzCmd = 0.0f;
}

void Move::setOffset(float offset_x, float offset_y, float offset_z, float offset_yaw) {
    status_.offsetX = offset_x;
    status_.offsetY = offset_y;
    status_.offsetZ = offset_z;
    status_.offsetYaw = normalizeAngle(offset_yaw);
}

void Move::getOffset(float& offset_x, float& offset_y, float& offset_z, float& offset_yaw) const {
    offset_x = status_.offsetX;
    offset_y = status_.offsetY;
    offset_z = status_.offsetZ;
    offset_yaw = status_.offsetYaw;
}

void Move::setCurrentAsOrigin() {
    if (!config_.lidar) {
        return;
    }

    // 获取当前传感器原始数据
    LidarPoseData pose_data = config_.lidar->getPoseData();
    LidarImuData imu_data = config_.lidar->getImuData();
    
    if (pose_data.valid && imu_data.valid) {
        // 将当前传感器读数设为偏移量
        status_.offsetX = pose_data.x;
        status_.offsetY = pose_data.y;
        status_.offsetZ = pose_data.z;
        status_.offsetYaw = normalizeAngle(imu_data.yaw);
    }
}

void Move::updateSensorData() {
    if (!config_.lidar) {
        return;
    }

    // 获取位置数据 (地面坐标系) - 原始传感器数据
    LidarPoseData pose_data = config_.lidar->getPoseData();
    if (pose_data.valid) {
        // 应用偏移量：计算坐标 = 原始坐标 - 偏移量
        status_.currentX_ground = pose_data.x - status_.offsetX;
        status_.currentY_ground = pose_data.y - status_.offsetY;
        status_.currentZ_ground = pose_data.z - status_.offsetZ;
    }

    // 获取速度数据 (地面坐标系)
    LidarVelocityData velocity_data = config_.lidar->getVelocityData();
    if (velocity_data.valid) {
        status_.currentVx_ground = velocity_data.vx_filtered;
        status_.currentVy_ground = velocity_data.vy_filtered;
        status_.currentVz_ground = velocity_data.vz_filtered;
    }

    // 获取IMU数据 (偏航角) - 应用偏移量
    LidarImuData imu_data = config_.lidar->getImuData();
    if (imu_data.valid) {
        // 应用偏移量并归一化：计算角度 = 原始角度 - 偏移角度
        status_.currentYaw = normalizeAngle(imu_data.yaw - status_.offsetYaw);
    }

    // 转换当前速度到机身坐标系
    transformGroundToBody(status_.currentVx_ground, status_.currentVy_ground, status_.currentVz_ground,
                        status_.currentYaw,
                        status_.currentVx_body, status_.currentVy_body, status_.currentVz_body);
}

void Move::transformGroundToBody(float x_ground, float y_ground, float z_ground, 
                               float yaw, 
                               float& x_body, float& y_body, float& z_body) {
    // 坐标系转换：从地面坐标系到机身坐标系
    // 地面坐标系：X-北，Y-东，Z-上
    // 机身坐标系：X-前，Y-右，Z-下
    
    float cos_yaw = cosf(yaw);
    float sin_yaw = sinf(yaw);
    
    // 旋转变换矩阵 (绕Z轴旋转-yaw角)
    // [x_body]   [cos_yaw  sin_yaw  0] [x_ground]
    // [y_body] = [-sin_yaw cos_yaw  0] [y_ground]
    // [z_body]   [0        0        1] [z_ground]
    
    x_body = cos_yaw * x_ground + sin_yaw * y_ground;
    y_body = -sin_yaw * x_ground + cos_yaw * y_ground;
    z_body = z_ground; // Z轴方向保持不变 (都是向上)
}

void Move::updatePositionErrorTransform() {
    // 计算地面坐标系下的位置误差
    float errorX_ground = target_.x_ground - status_.currentX_ground;
    float errorY_ground = target_.y_ground - status_.currentY_ground;
    float errorZ_ground = target_.z_ground - status_.currentZ_ground;
    
    // 将位置误差转换为机身坐标系
    transformGroundToBody(errorX_ground, errorY_ground, errorZ_ground,
                        status_.currentYaw,
                        status_.errorX_body, status_.errorY_body, status_.errorZ_body);
}

float Move::constrain(float value, float minVal, float maxVal) {
    return std::max(minVal, std::min(value, maxVal));
}

float Move::normalizeAngle(float angle) {
    while (angle > M_PI) {
        angle -= 2.0f * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
    return angle;
}
