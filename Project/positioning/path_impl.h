#ifndef PATH_IMPL_H
#define PATH_IMPL_H

// 默认构造函数
template<uint32_t N>
Path<N>::Path() 
    : guide_point_(), guide_err_r_(0.02f), guide_err_yaw_(0.05f), guide_threshold_(5),
      current_target_index_(0), current_step_(0), total_steps_(0), step_distance_(0.0f),
      state_(PathState::IDLE), start_pose_(), end_pose_() {
    // 初始化目标点指针数组为nullptr
    for (uint32_t i = 0; i < N; ++i) {
        target_points_[i] = nullptr;
    }
}

// 参数构造函数
template<uint32_t N>
Path<N>::Path(Point* points[N], float guide_err_r, float guide_err_yaw, uint32_t guide_threshold)
    : guide_point_(), guide_err_r_(guide_err_r), guide_err_yaw_(guide_err_yaw), guide_threshold_(guide_threshold),
      current_target_index_(0), current_step_(0), total_steps_(0), step_distance_(0.0f),
      state_(PathState::IDLE), start_pose_(), end_pose_() {
    initializePath(points);
}

// 初始化路径
template<uint32_t N>
void Path<N>::initializePath(Point* points[N]) {
    for (uint32_t i = 0; i < N; ++i) {
        target_points_[i] = points[i];
    }
    resetPath();
}

// 设置引导点容差参数
template<uint32_t N>
void Path<N>::setGuideParameters(float err_r, float err_yaw, uint32_t threshold) {
    guide_err_r_ = err_r;
    guide_err_yaw_ = err_yaw;
    guide_threshold_ = threshold;
}

// 开始路径跟踪
template<uint32_t N>
void Path<N>::startPath() {
    if (!isValidTargetPoints() || N == 0) {
        return;
    }
    
    current_target_index_ = 0;
    current_step_ = 0;
    state_ = PathState::MOVING_TO_TARGET;
    
    // 设置引导点为第一个目标点
    guide_point_.setTargetPose(target_points_[0]->getTargetPose());
    guide_point_.setErrorTolerance(target_points_[0]->getTargetPose().x, target_points_[0]->getTargetPose().y); // 需要获取目标点的容差
    guide_point_.resetCounter();
}

// 重置路径
template<uint32_t N>
void Path<N>::resetPath() {
    current_target_index_ = 0;
    current_step_ = 0;
    total_steps_ = 0;
    step_distance_ = 0.0f;
    state_ = PathState::IDLE;
    guide_point_.resetCounter();
}

// 获取当前路径状态
template<uint32_t N>
PathState Path<N>::getState() const {
    return state_;
}

// 获取当前目标点索引
template<uint32_t N>
uint32_t Path<N>::getCurrentTargetIndex() const {
    return current_target_index_;
}

// 获取路径进度
template<uint32_t N>
float Path<N>::getProgress() const {
    if (N == 0 || state_ == PathState::IDLE) {
        return 0.0f;
    }
    if (state_ == PathState::COMPLETED) {
        return 1.0f;
    }
    
    float segment_progress = (total_steps_ > 0) ? (float)current_step_ / (float)total_steps_ : 0.0f;
    float total_progress = (float)current_target_index_ / (float)N + segment_progress / (float)N;
    return total_progress;
}

// 核心接口：检查是否到达路径终点
template<uint32_t N>
bool Path<N>::isReached(const Pose& current_pose, PoseDiff* pose_diff) {
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

// 获取当前引导点位姿
template<uint32_t N>
const Pose& Path<N>::getCurrentGuidePose() const {
    return guide_point_.getTargetPose();
}

// 计算两点间的插值参数
template<uint32_t N>
void Path<N>::calculateInterpolationParams(const Pose& start, const Pose& end) {
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
template<uint32_t N>
void Path<N>::updateGuidePoint() {
    current_step_++;
    
    if (current_step_ >= total_steps_) {
        // 到达下一个目标点
        current_target_index_++;
        if (current_target_index_ >= N) {
            // 路径完成
            state_ = PathState::COMPLETED;
            return;
        }
        
        // 设置引导点为目标点，等待稳定
        guide_point_.setTargetPose(target_points_[current_target_index_]->getTargetPose());
        // 使用目标点的容差参数
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
template<uint32_t N>
Pose Path<N>::interpolatePose(const Pose& start, const Pose& end, float ratio) {
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
template<uint32_t N>
void Path<N>::moveToNextSegment() {
    if (current_target_index_ + 1 >= N) {
        // 已经是最后一个目标点
        state_ = PathState::COMPLETED;
        return;
    }
    
    // 准备移动到下一个目标点
    start_pose_ = target_points_[current_target_index_]->getTargetPose();
    end_pose_ = target_points_[current_target_index_ + 1]->getTargetPose();
    
    calculateInterpolationParams(start_pose_, end_pose_);
    
    // 设置引导点容差参数
    guide_point_.setErrorTolerance(guide_err_r_, guide_err_yaw_);
    guide_point_.setThreshold(guide_threshold_);
    
    state_ = PathState::MOVING_BETWEEN_TARGETS;
    updateGuidePoint();
}

// 检查目标点数组有效性
template<uint32_t N>
bool Path<N>::isValidTargetPoints() const {
    for (uint32_t i = 0; i < N; ++i) {
        if (target_points_[i] == nullptr) {
            return false;
        }
    }
    return true;
}

#endif // PATH_IMPL_H
