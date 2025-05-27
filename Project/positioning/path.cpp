// 由于Path是模板类，主要实现在path_impl.h中
// 这里可以放置一些非模板的辅助函数

#include "path.h"
#include "utils.h"
#include <cmath>

// 默认构造函数
Path::Path() 
    : target_count_(0), guide_point_(), guide_tolerance_(0.02f, 0.05f, 5),
      current_target_index_(0), current_step_(0), total_steps_(0), step_distance_(0.0f),
      state_(PathState::IDLE), start_pose_(), end_pose_() {
    // 初始化目标点指针数组为nullptr
    for (uint32_t i = 0; i < MAX_TARGET_POINTS; ++i) {
        target_points_[i] = nullptr;
    }
}

// 参数构造函数
Path::Path(const ToleranceParams& guide_tolerance)
    : target_count_(0), guide_point_(), guide_tolerance_(guide_tolerance),
      current_target_index_(0), current_step_(0), total_steps_(0), step_distance_(0.0f),
      state_(PathState::IDLE), start_pose_(), end_pose_() {
    // 初始化目标点指针数组为nullptr
    for (uint32_t i = 0; i < MAX_TARGET_POINTS; ++i) {
        target_points_[i] = nullptr;
    }
}

// 添加目标点
bool Path::addTargetPoint(Point* point) {
    if (point == nullptr || target_count_ >= MAX_TARGET_POINTS) {
        return false;
    }
    
    target_points_[target_count_] = point;
    target_count_++;
    return true;
}

// 清空所有目标点
void Path::clearTargetPoints() {
    for (uint32_t i = 0; i < MAX_TARGET_POINTS; ++i) {
        target_points_[i] = nullptr;
    }
    target_count_ = 0;
    resetPath();
}

// 获取目标点数量
uint32_t Path::getTargetCount() const {
    return target_count_;
}

// 设置引导点容差参数
void Path::setGuideParameters(const ToleranceParams& tolerance) {
    guide_tolerance_ = tolerance;
}

// 开始路径跟踪
void Path::startPath() {
    if (!isValidTargetPoints() || target_count_ == 0) {
        return;
    }
    
    current_target_index_ = 0;
    current_step_ = 0;
    state_ = PathState::MOVING_TO_TARGET;
    
    // 设置引导点为第一个目标点
    guide_point_.setTargetPose(target_points_[0]->getTargetPose());
    guide_point_.setToleranceParams(target_points_[0]->getToleranceParams());
    guide_point_.resetCounter();
}

// 重置路径
void Path::resetPath() {
    current_target_index_ = 0;
    current_step_ = 0;
    total_steps_ = 0;
    step_distance_ = 0.0f;
    state_ = PathState::IDLE;
    guide_point_.resetCounter();
}

// 获取当前路径状态
PathState Path::getState() const {
    return state_;
}

// 获取当前目标点索引
uint32_t Path::getCurrentTargetIndex() const {
    return current_target_index_;
}

// 获取路径进度
float Path::getProgress() const {
    if (target_count_ == 0 || state_ == PathState::IDLE) {
        return 0.0f;
    }
    if (state_ == PathState::COMPLETED) {
        return 1.0f;
    }
    
    float segment_progress = (total_steps_ > 0) ? (float)current_step_ / (float)total_steps_ : 0.0f;
    float total_progress = (float)current_target_index_ / (float)target_count_ + segment_progress / (float)target_count_;
    return total_progress;
}

// 核心接口：检查是否到达路径终点
bool Path::isReached(const Pose& current_pose, PoseDiff* pose_diff) {
    // 在所有状态下都计算位姿差值
    if (pose_diff != nullptr) {
        if (state_ == PathState::IDLE && target_count_ > 0) {
            // IDLE状态：计算与第一个目标点的差值
            calculatePoseDiff(current_pose, target_points_[0]->getTargetPose(), *pose_diff);
        } else if (state_ == PathState::COMPLETED && target_count_ > 0) {
            // COMPLETED状态：计算与最后一个目标点的差值
            calculatePoseDiff(current_pose, target_points_[target_count_ - 1]->getTargetPose(), *pose_diff);
        }
    }
    
    if (state_ == PathState::IDLE || !isValidTargetPoints()) {
        return false;
    }
    
    if (state_ == PathState::COMPLETED) {
        return true;
    }
    
    // 检查是否到达当前引导点
    bool guide_reached = guide_point_.isReached(current_pose, pose_diff);
    
    if (guide_reached) {
        if (state_ == PathState::MOVING_TO_TARGET) {
            // 到达目标点，准备移动到下一段
            moveToNextSegment();
        } else if (state_ == PathState::MOVING_BETWEEN_TARGETS) {
            // 在目标点之间移动，更新引导点
            updateGuidePoint();
        }
    }
    
    return state_ == PathState::COMPLETED;
}

// 计算位姿差值（添加为public方法的私有实现）
void Path::calculatePoseDiff(const Pose& current, const Pose& target, PoseDiff& diff) {
    diff.dx = target.x - current.x;
    diff.dy = target.y - current.y;
    diff.dz = target.z - current.z;
    diff.dyaw = math_normalize_radian_pi(target.yaw - current.yaw);
    
    // 计算位置距离
    diff.distance = sqrtf(diff.dx * diff.dx + diff.dy * diff.dy + diff.dz * diff.dz);
}

// 计算当前位置与指定目标点的位姿差值
void Path::calculatePoseDiffToTarget(const Pose& current_pose, uint32_t target_index, PoseDiff& diff) const {
    if (target_index >= target_count_ || target_points_[target_index] == nullptr) {
        // 无效索引，返回零差值
        diff = PoseDiff();
        return;
    }
    
    const Pose& target = target_points_[target_index]->getTargetPose();
    diff.dx = target.x - current_pose.x;
    diff.dy = target.y - current_pose.y;
    diff.dz = target.z - current_pose.z;
    diff.dyaw = math_normalize_radian_pi(target.yaw - current_pose.yaw);
    
    // 计算位置距离
    diff.distance = sqrtf(diff.dx * diff.dx + diff.dy * diff.dy + diff.dz * diff.dz);
}

// 获取当前引导点位姿
const Pose& Path::getCurrentGuidePose() const {
    return guide_point_.getTargetPose();
}

// 计算两点间的插值参数
void Path::calculateInterpolationParams(const Pose& start, const Pose& end) {
    float dx = end.x - start.x;
    float dy = end.y - start.y;
    float dz = end.z - start.z;
    float total_distance = sqrtf(dx * dx + dy * dy + dz * dz);
    
    if (total_distance < GUIDE_POINT_INTERVAL) {
        total_steps_ = 1;
        step_distance_ = total_distance;
    } else {
        total_steps_ = (uint32_t)(total_distance / GUIDE_POINT_INTERVAL + 0.5f);
        step_distance_ = total_distance / (float)total_steps_;
    }
    
    current_step_ = 0;
}

// 更新引导点到下一个位置
void Path::updateGuidePoint() {
    current_step_++;
    
    if (current_step_ >= total_steps_) {
        // 到达下一个目标点
        current_target_index_++;
        if (current_target_index_ >= target_count_) {
            // 路径完成
            state_ = PathState::COMPLETED;
            return;
        }
        
        // 设置引导点为目标点，等待稳定
        guide_point_.setTargetPose(target_points_[current_target_index_]->getTargetPose());
        guide_point_.setToleranceParams(target_points_[current_target_index_]->getToleranceParams());
        state_ = PathState::MOVING_TO_TARGET;
        guide_point_.resetCounter();
    } else {
        // 插值到下一个引导点位置
        float ratio = (float)current_step_ / (float)total_steps_;
        Pose next_guide_pose = interpolatePose(start_pose_, end_pose_, ratio);
        guide_point_.setTargetPose(next_guide_pose);
        guide_point_.resetCounter();
    }
}

// 线性插值计算位姿
Pose Path::interpolatePose(const Pose& start, const Pose& end, float ratio) {
    Pose result;
    result.x = start.x + (end.x - start.x) * ratio;
    result.y = start.y + (end.y - start.y) * ratio;
    result.z = start.z + (end.z - start.z) * ratio;
    
    // 角度插值需要考虑角度环绕
    float yaw_diff = math_normalize_radian_pi(end.yaw - start.yaw);
    result.yaw = math_normalize_radian_pi(start.yaw + yaw_diff * ratio);
    
    return result;
}

// 进入下一个路径段
void Path::moveToNextSegment() {
    if (current_target_index_ + 1 >= target_count_) {
        // 已经是最后一个目标点
        state_ = PathState::COMPLETED;
        return;
    }
    
    // 准备移动到下一个目标点
    start_pose_ = target_points_[current_target_index_]->getTargetPose();
    end_pose_ = target_points_[current_target_index_ + 1]->getTargetPose();
    
    calculateInterpolationParams(start_pose_, end_pose_);
    
    // 设置引导点容差参数
    guide_point_.setToleranceParams(guide_tolerance_);
    
    state_ = PathState::MOVING_BETWEEN_TARGETS;
    updateGuidePoint();
}

// 检查目标点数组有效性
bool Path::isValidTargetPoints() const {
    if (target_count_ == 0) {
        return false;
    }
    
    for (uint32_t i = 0; i < target_count_; ++i) {
        if (target_points_[i] == nullptr) {
            return false;
        }
    }
    return true;
}
