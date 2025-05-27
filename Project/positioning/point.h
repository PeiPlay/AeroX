#ifndef POINT_H
#define POINT_H

#include <cstdint>

// 位姿结构体
struct Pose {
    float x;      // x坐标
    float y;      // y坐标  
    float z;      // z坐标
    float yaw;    // 偏航角（弧度）
    
    Pose() : x(0.0f), y(0.0f), z(0.0f), yaw(0.0f) {}
    Pose(float x, float y, float z, float yaw) : x(x), y(y), z(z), yaw(yaw) {}
};

// 位姿差值结构体
struct PoseDiff {
    float dx;     // x方向差值
    float dy;     // y方向差值
    float dz;     // z方向差值
    float dyaw;   // 偏航角差值
    float distance; // 位置距离
    
    PoseDiff() : dx(0.0f), dy(0.0f), dz(0.0f), dyaw(0.0f), distance(0.0f) {}
};

// 容差参数结构体
struct ToleranceParams {
    float err_r;        // 位置允许误差（米）
    float err_yaw;      // 偏航角允许误差（弧度）
    uint32_t threshold; // 稳定性检测阈值
    
    ToleranceParams() : err_r(0.1f), err_yaw(0.1f), threshold(10) {}
    ToleranceParams(float r, float yaw, uint32_t th) : err_r(r), err_yaw(yaw), threshold(th) {}
};

// 目标点类
class Point {
private:
    Pose target_pose_;          // 目标位姿
    ToleranceParams tolerance_; // 容差参数
    uint32_t counter_;          // 32位计数器
    
public:
    // 构造函数
    Point();
    Point(const Pose& target, const ToleranceParams& tolerance);
    
    // 设置目标位姿
    void setTargetPose(const Pose& target);
    
    // 设置误差容限
    void setErrorTolerance(float err_r, float err_yaw);
    
    // 设置容差参数
    void setToleranceParams(const ToleranceParams& tolerance);
    
    // 设置稳定性阈值
    void setThreshold(uint32_t threshold);
    
    // 获取目标位姿
    const Pose& getTargetPose() const;
    
    // 获取容差参数
    const ToleranceParams& getToleranceParams() const;
    
    // 获取当前计数器值
    uint32_t getCounter() const;
    
    // 重置计数器
    void resetCounter();
    
    // 核心接口：检查是否到达目标点
    bool isReached(const Pose& current_pose, PoseDiff* pose_diff = nullptr);
    
private:
    // 计算位姿差值
    void calculatePoseDiff(const Pose& current, const Pose& target, PoseDiff& diff);
    
    // 检查是否在误差范围内
    bool isWithinTolerance(const PoseDiff& diff);
};

#endif // POINT_H
