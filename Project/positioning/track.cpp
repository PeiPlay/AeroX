#include "track.h"

// 构造函数
Track::Track() 
    : stage_count_(0), current_stage_index_(0), state_(TrackState::IDLE) {
    // 初始化阶段数组
    for (uint32_t i = 0; i < MAX_PATHS; ++i) {
        stages_[i] = PathStage();
    }
}

// 添加路径阶段
bool Track::addStage(Path* path, PathCompletedCallback callback, void* user_data) {
    if (path == nullptr || stage_count_ >= MAX_PATHS) {
        return false;
    }
    
    stages_[stage_count_] = PathStage(path, callback, user_data);
    stage_count_++;
    return true;
}

// 清空所有阶段
void Track::clearStages() {
    for (uint32_t i = 0; i < MAX_PATHS; ++i) {
        stages_[i] = PathStage();
    }
    stage_count_ = 0;
    resetTrack();
}

// 获取阶段数量
uint32_t Track::getStageCount() const {
    return stage_count_;
}

// 切换到指定路径
bool Track::switchToStage(uint32_t stage_index) {
    if (stage_index >= stage_count_ || stages_[stage_index].path == nullptr) {
        return false;
    }
    
    // 重置目标路径
    stages_[stage_index].path->resetPath();
    
    // 切换到指定阶段
    current_stage_index_ = stage_index;
    
    // 如果轨迹正在执行，则启动新路径
    if (state_ == TrackState::EXECUTING) {
        stages_[current_stage_index_].path->startPath();
    }
    
    return true;
}

// 开始轨迹执行
void Track::startTrack() {
    if (!isValidStages() || stage_count_ == 0) {
        return;
    }
    
    current_stage_index_ = 0;
    state_ = TrackState::EXECUTING;
    
    // 开始第一个路径
    stages_[0].path->startPath();
}

// 暂停轨迹执行
void Track::pauseTrack() {
    if (state_ == TrackState::EXECUTING) {
        state_ = TrackState::PAUSED;
    }
}

// 恢复轨迹执行
void Track::resumeTrack() {
    if (state_ == TrackState::PAUSED) {
        state_ = TrackState::EXECUTING;
    }
}

// 重置轨迹
void Track::resetTrack() {
    current_stage_index_ = 0;
    state_ = TrackState::IDLE;
    
    // 重置所有路径
    for (uint32_t i = 0; i < stage_count_; ++i) {
        if (stages_[i].path != nullptr) {
            stages_[i].path->resetPath();
        }
    }
}

// 获取当前轨迹状态
TrackState Track::getState() const {
    return state_;
}

// 获取当前阶段索引
uint32_t Track::getCurrentStageIndex() const {
    return current_stage_index_;
}

// 获取整体进度
float Track::getTotalProgress() const {
    if (stage_count_ == 0 || state_ == TrackState::IDLE) {
        return 0.0f;
    }
    if (state_ == TrackState::COMPLETED) {
        return 1.0f;
    }
    
    float stage_progress = 0.0f;
    if (current_stage_index_ < stage_count_ && stages_[current_stage_index_].path != nullptr) {
        stage_progress = stages_[current_stage_index_].path->getProgress();
    }
    
    float total_progress = (float)current_stage_index_ / (float)stage_count_ + stage_progress / (float)stage_count_;
    return total_progress;
}

// 核心接口：处理轨迹执行
bool Track::process(const Pose& current_pose, PoseDiff* pose_diff) {
    if (state_ == TrackState::IDLE || state_ == TrackState::PAUSED || !isValidStages()) {
        // 在IDLE和PAUSED状态下也要更新pose_diff
        if (pose_diff != nullptr && stage_count_ > 0) {
            if (state_ == TrackState::IDLE && stages_[0].path != nullptr) {
                // IDLE状态：使用第一个路径计算差值
                stages_[0].path->isReached(current_pose, pose_diff);
            } else if (current_stage_index_ < stage_count_ && stages_[current_stage_index_].path != nullptr) {
                // PAUSED状态：使用当前路径计算差值
                stages_[current_stage_index_].path->isReached(current_pose, pose_diff);
            }
        }
        return false;
    }
    
    if (state_ == TrackState::COMPLETED) {
        // COMPLETED状态：使用最后一个路径计算差值
        if (pose_diff != nullptr && stage_count_ > 0 && stages_[stage_count_ - 1].path != nullptr) {
            stages_[stage_count_ - 1].path->isReached(current_pose, pose_diff);
        }
        return true;
    }
    
    // 检查当前路径是否完成
    if (current_stage_index_ < stage_count_ && stages_[current_stage_index_].path != nullptr) {
        bool path_completed = stages_[current_stage_index_].path->isReached(current_pose, pose_diff);
        
        if (path_completed) {
            // 调用完成回调
            if (stages_[current_stage_index_].callback != nullptr) {
                stages_[current_stage_index_].callback(current_stage_index_, stages_[current_stage_index_].user_data);
            }
            
            moveToNextStage();
        }
    }
    
    return state_ == TrackState::COMPLETED;
}

// 获取当前引导点位姿
const Pose& Track::getCurrentGuidePose() const {
    static Pose default_pose;
    
    if (current_stage_index_ < stage_count_ && stages_[current_stage_index_].path != nullptr) {
        return stages_[current_stage_index_].path->getCurrentGuidePose();
    }
    
    return default_pose;
}

// 获取当前路径指针
Path* Track::getCurrentPath() const {
    if (current_stage_index_ < stage_count_) {
        return stages_[current_stage_index_].path;
    }
    return nullptr;
}

// 移动到下一个阶段
void Track::moveToNextStage() {
    current_stage_index_++;
    
    if (current_stage_index_ >= stage_count_) {
        // 所有阶段完成
        state_ = TrackState::COMPLETED;
        return;
    }
    
    // 开始下一个路径
    if (stages_[current_stage_index_].path != nullptr) {
        stages_[current_stage_index_].path->startPath();
    }
}

// 检查阶段数组有效性
bool Track::isValidStages() const {
    if (stage_count_ == 0) {
        return false;
    }
    
    for (uint32_t i = 0; i < stage_count_; ++i) {
        if (stages_[i].path == nullptr) {
            return false;
        }
    }
    return true;
}
