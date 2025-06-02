#include "point.h"
#include "utils.h"
#include <cmath>

// 默认构造函数
Point::Point() 
    : target_pose_{0.0f, 0.0f, 0.0f, 0.0f}, 
      tolerance_{0.1f, 0.1f, 10}, 
      counter_(0) {
}

// 参数构造函数
Point::Point(const Pose& target, const ToleranceParams& tolerance)
    : target_pose_(target), tolerance_(tolerance), counter_(0) {
}

// 设置目标位姿
void Point::setTargetPose(const Pose& target) {
    target_pose_ = target;
    counter_ = 0; // 重置计数器
}

// 设置误差容限
void Point::setErrorTolerance(float err_r, float err_yaw) {
    tolerance_.err_r = err_r;
    tolerance_.err_yaw = err_yaw;
}

// 设置容差参数
void Point::setToleranceParams(const ToleranceParams& tolerance) {
    tolerance_ = tolerance;
}

// 设置稳定性阈值
void Point::setThreshold(uint32_t threshold) {
    tolerance_.threshold = threshold;
}

// 获取目标位姿
const Pose& Point::getTargetPose() const {
    return target_pose_;
}

// 获取容差参数
const ToleranceParams& Point::getToleranceParams() const {
    return tolerance_;
}

// 获取当前计数器值
uint32_t Point::getCounter() const {
    return counter_;
}

// 重置计数器
void Point::resetCounter() {
    counter_ = 0;
}

// 核心接口：检查是否到达目标点
bool Point::isReached(const Pose& current_pose, PoseDiff* pose_diff) {
    PoseDiff diff;
    calculatePoseDiff(current_pose, target_pose_, diff);
    
    // 如果提供了指针，则输出位姿差值
    if (pose_diff != nullptr) {
        *pose_diff = diff;
    }
    
    // 检查是否在误差范围内
    if (isWithinTolerance(diff)) {
        counter_++;
        // 仅当计数器大于阈值时返回true
        return counter_ > tolerance_.threshold;
    } else {
        counter_ = 0; // 清空计数器
        return false;
    }
}

// 计算位姿差值
void Point::calculatePoseDiff(const Pose& current, const Pose& target, PoseDiff& diff) {
    diff.dx = target.x - current.x;
    diff.dy = target.y - current.y;
    diff.dz = target.z - current.z;
    diff.dyaw = math_normalize_radian_pi(target.yaw - current.yaw);

    // 计算位置距离
    diff.distance = sqrtf(diff.dx * diff.dx + diff.dy * diff.dy + diff.dz * diff.dz);
}

// 检查是否在误差范围内
bool Point::isWithinTolerance(const PoseDiff& diff) {
    return (diff.distance <= tolerance_.err_r) && (fabsf(diff.dyaw) <= tolerance_.err_yaw);
}
