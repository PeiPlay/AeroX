#ifndef TRACK_H
#define TRACK_H

#include "path.h"
#include <cstdint>

// 最大路径数量
#define MAX_PATHS 8

// 轨迹状态枚举
enum class TrackState {
    IDLE,           // 空闲状态
    EXECUTING,      // 执行中
    COMPLETED,      // 完成
    PAUSED         // 暂停
};

// 路径完成回调函数类型
typedef void (*PathCompletedCallback)(uint32_t path_index, void* user_data);

// 路径阶段结构体
struct PathStage {
    Path* path;                           // 路径指针
    PathCompletedCallback callback;       // 完成时的回调函数
    void* user_data;                     // 用户数据
    
    PathStage() : path(nullptr), callback(nullptr), user_data(nullptr) {}
    PathStage(Path* p, PathCompletedCallback cb = nullptr, void* data = nullptr) 
        : path(p), callback(cb), user_data(data) {}
};

// Track轨迹类
class Track {
private:
    PathStage stages_[MAX_PATHS];         // 路径阶段数组
    uint32_t stage_count_;                // 实际阶段数量
    uint32_t current_stage_index_;        // 当前阶段索引
    TrackState state_;                    // 轨迹状态
    
public:
    // 构造函数
    Track();
    
    // 添加路径阶段
    bool addStage(Path* path, PathCompletedCallback callback = nullptr, void* user_data = nullptr);
    
    // 清空所有阶段
    void clearStages();
    
    // 获取阶段数量
    uint32_t getStageCount() const;
    
    // 切换到指定路径（重置该路径）
    bool switchToStage(uint32_t stage_index);
    
    // 开始轨迹执行
    void startTrack();
    
    // 暂停轨迹执行
    void pauseTrack();
    
    // 恢复轨迹执行
    void resumeTrack();
    
    // 重置轨迹
    void resetTrack();
    
    // 获取当前轨迹状态
    TrackState getState() const;
    
    // 获取当前阶段索引
    uint32_t getCurrentStageIndex() const;
    
    // 获取整体进度（0.0-1.0）
    float getTotalProgress() const;
    
    // 核心接口：处理轨迹执行
    bool process(const Pose& current_pose, PoseDiff* pose_diff = nullptr);
    
    // 获取当前引导点位姿
    const Pose& getCurrentGuidePose() const;
    
    // 获取当前路径指针
    Path* getCurrentPath() const;
    
private:
    // 移动到下一个阶段
    void moveToNextStage();
    
    // 检查阶段数组有效性
    bool isValidStages() const;
};

#endif // TRACK_H
