#ifndef PATH_H
#define PATH_H

#include "point.h"
#include <cstdint>

// 引导点间距宏定义（单位：米）
#define GUIDE_POINT_INTERVAL 0.05f

// 路径状态枚举
enum class PathState {
    IDLE,                    // 空闲状态
    MOVING_TO_TARGET,        // 移动到目标点
    MOVING_BETWEEN_TARGETS,  // 在目标点之间移动
    COMPLETED               // 路径完成
};

// Path路径类（模板类，编译时确定目标点数量）
template<uint32_t N>
class Path {
private:
    Point* target_points_[N];       // 目标点指针数组
    Point guide_point_;             // 引导点（单一实例）
    ToleranceParams guide_tolerance_; // 引导点容差参数
    
    uint32_t current_target_index_; // 当前目标点索引
    uint32_t current_step_;         // 当前插值步数
    uint32_t total_steps_;          // 当前段总步数
    float step_distance_;           // 单步距离
    PathState state_;               // 路径状态
    
    Pose start_pose_;               // 当前段起始位姿
    Pose end_pose_;                 // 当前段结束位姿
    
public:
    // 构造函数
    Path();
    Path(Point* points[N], const ToleranceParams& guide_tolerance);
    
    // 初始化路径（设置目标点）
    void initializePath(Point* points[N]);
    
    // 设置引导点容差参数
    void setGuideParameters(const ToleranceParams& tolerance);
    
    // 开始路径跟踪
    void startPath();
    
    // 重置路径
    void resetPath();
    
    // 获取当前路径状态
    PathState getState() const;
    
    // 获取当前目标点索引
    uint32_t getCurrentTargetIndex() const;
    
    // 获取路径进度（0.0-1.0）
    float getProgress() const;
    
    // 核心接口：检查是否到达路径终点
    bool isReached(const Pose& current_pose, PoseDiff* pose_diff = nullptr);
    
    // 获取当前引导点位姿
    const Pose& getCurrentGuidePose() const;
    
private:
    // 计算两点间的插值步数和单步距离
    void calculateInterpolationParams(const Pose& start, const Pose& end);
    
    // 更新引导点到下一个位置
    void updateGuidePoint();
    
    // 线性插值计算位姿
    Pose interpolatePose(const Pose& start, const Pose& end, float ratio);
    
    // 进入下一个路径段
    void moveToNextSegment();
    
    // 检查是否有效的目标点数组
    bool isValidTargetPoints() const;
};

// 模板类实现包含在头文件中
#include "path_impl.h"

#endif // PATH_H
