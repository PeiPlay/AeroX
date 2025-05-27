#ifndef MOVE_H
#define MOVE_H

#include "lidar.h"
#include "pid.h"
#include <stdint.h>

// 定义PID索引
#define PID_X_POSITION  0
#define PID_Y_POSITION  1
#define PID_Z_POSITION  2
#define PID_X_VELOCITY  0
#define PID_Y_VELOCITY  1
#define PID_Z_VELOCITY  2

/**
 * @brief Move 外部依赖项结构体
 * @details 包含 Move 运行所需的外部对象指针
 */
struct MoveDependencies {
    Lidar* lidar = nullptr;
    PidController* positionPIDs[3] = {nullptr, nullptr, nullptr}; // X, Y, Z Position (外环)
    PidController* velocityPIDs[3] = {nullptr, nullptr, nullptr}; // X, Y, Z Velocity (内环)
};

/**
 * @brief 无人机移动控制类
 * @details 整合雷达定位和PID控制器，实现串级PID位置控制
 */
class Move {
public:
    /**
     * @brief 内部配置结构体
     */
    struct Config {
        Lidar* lidar = nullptr;
        PidController* positionPIDs[3] = {nullptr, nullptr, nullptr}; // 外环: X, Y, Z Position
        PidController* velocityPIDs[3] = {nullptr, nullptr, nullptr}; // 内环: X, Y, Z Velocity
    };

    /**
     * @brief 内部目标值结构体
     */
    struct Target {
        // 地面坐标系目标值 (设定值)
        float x_ground = 0.0f;        // 目标X位置 (m) - 地面坐标系
        float y_ground = 0.0f;        // 目标Y位置 (m) - 地面坐标系
        float z_ground = 0.0f;        // 目标Z位置 (m) - 地面坐标系
        float vx = 0.0f;              // 目标X速度 (m/s) - 可选直接设置
        float vy = 0.0f;              // 目标Y速度 (m/s) - 可选直接设置
        float vz = 0.0f;              // 目标Z速度 (m/s) - 可选直接设置
        
        bool use_position_control = true; // true: 位置控制模式, false: 速度控制模式
    };

    /**
     * @brief 内部状态结构体
     */
    struct Status {
        // 当前传感器读数 - 地面坐标系
        float currentX_ground = 0.0f;           // 当前X位置 (m) - 地面坐标系
        float currentY_ground = 0.0f;           // 当前Y位置 (m) - 地面坐标系
        float currentZ_ground = 0.0f;           // 当前Z位置 (m) - 地面坐标系
        float currentVx_ground = 0.0f;          // 当前X速度 (m/s) - 地面坐标系
        float currentVy_ground = 0.0f;          // 当前Y速度 (m/s) - 地面坐标系
        float currentVz_ground = 0.0f;          // 当前Z速度 (m/s) - 地面坐标系
        
        // 偏移量 - 原点设定
        float offsetX = 0.0f;                   // X方向偏移量 (m)
        float offsetY = 0.0f;                   // Y方向偏移量 (m)
        float offsetZ = 0.0f;                   // Z方向偏移量 (m)
        float offsetYaw = 0.0f;                 // 偏航角偏移量 (rad)
        
        // 机身坐标系下的相对位置误差和速度 (用于控制)
        float errorX_body = 0.0f;               // X方向位置误差 (m) - 机身坐标系
        float errorY_body = 0.0f;               // Y方向位置误差 (m) - 机身坐标系
        float errorZ_body = 0.0f;               // Z方向位置误差 (m) - 机身坐标系
        float currentVx_body = 0.0f;            // 当前X速度 (m/s) - 机身坐标系
        float currentVy_body = 0.0f;            // 当前Y速度 (m/s) - 机身坐标系
        float currentVz_body = 0.0f;            // 当前Z速度 (m/s) - 机身坐标系
        
        // 当前姿态信息
        float currentYaw = 0.0f;                // 当前偏航角 (rad)

        // PID 中间计算结果
        float targetVxCmd = 0.0f;        // 位置环输出的目标X速度 (m/s)
        float targetVyCmd = 0.0f;        // 位置环输出的目标Y速度 (m/s)
        float targetVzCmd = 0.0f;        // 位置环输出的目标Z速度 (m/s)
        float rollCmd = 0.0f;            // 速度环输出的横滚角度指令 (rad)
        float pitchCmd = 0.0f;           // 速度环输出的俯仰角度指令 (rad)
        float throttleCmd = 0.0f;        // 速度环输出的油门指令 (相对值)

        // 数据有效性
        bool isInitialized = false;      // 初始化标志
    };

    /**
     * @brief 构造函数
     * @param deps 外部依赖项
     */
    explicit Move(const MoveDependencies& deps);

    /**
     * @brief 虚析构函数
     */
    virtual ~Move() = default;

    /**
     * @brief 初始化移动控制器
     * @return 初始化是否成功
     */
    virtual bool init();

    /**
     * @brief 设置目标位置 (地面坐标系)
     * @param x 目标X位置 (m)
     * @param y 目标Y位置 (m)
     * @param z 目标Z位置 (m)
     */
    virtual void setTargetPosition(float x, float y, float z);

    /**
     * @brief 设置目标速度（直接速度控制模式）
     * @param vx 目标X速度 (m/s)
     * @param vy 目标Y速度 (m/s)
     * @param vz 目标Z速度 (m/s)
     */
    virtual void setTargetVelocity(float vx, float vy, float vz);

    /**
     * @brief 设置控制模式
     * @param use_position_control true: 位置控制, false: 速度控制
     */
    virtual void setPositionControlMode(bool use_position_control);

    /**
     * @brief 获取当前位置 (地面坐标系)
     */
    virtual void getCurrentPosition(float& x, float& y, float& z) const;

    /**
     * @brief 获取当前速度 (地面坐标系)
     */
    virtual void getCurrentVelocity(float& vx, float& vy, float& vz) const;

    /**
     * @brief 获取当前位置误差 (机身坐标系)
     */
    virtual void getPositionErrorBody(float& ex, float& ey, float& ez) const;

    /**
     * @brief 获取当前速度 (机身坐标系)
     */
    virtual void getCurrentVelocityBody(float& vx, float& vy, float& vz) const;

    /**
     * @brief 获取输出的姿态指令
     * @param roll 输出横滚角指令 (rad)
     * @param pitch 输出俯仰角指令 (rad)
     * @param throttle 输出油门指令 (相对值)
     */
    virtual void getAttitudeCommand(float& roll, float& pitch, float& throttle) const;

    /**
     * @brief 更新移动控制状态（应在固定频率下调用）
     * @details 执行传感器读取、串级PID计算和姿态指令输出
     */
    virtual void update();

    /**
     * @brief 重置控制器
     */
    virtual void reset();

    /**
     * @brief 获取状态信息
     */
    virtual const Status& getStatus() const { return status_; }

    /**
     * @brief 设置坐标偏移量
     * @param offset_x X方向偏移量 (m)
     * @param offset_y Y方向偏移量 (m)
     * @param offset_z Z方向偏移量 (m)
     * @param offset_yaw 偏航角偏移量 (rad)
     */
    virtual void setOffset(float offset_x, float offset_y, float offset_z, float offset_yaw);

    /**
     * @brief 获取当前偏移量
     * @param offset_x 输出X方向偏移量 (m)
     * @param offset_y 输出Y方向偏移量 (m)
     * @param offset_z 输出Z方向偏移量 (m)
     * @param offset_yaw 输出偏航角偏移量 (rad)
     */
    virtual void getOffset(float& offset_x, float& offset_y, float& offset_z, float& offset_yaw) const;

    /**
     * @brief 将当前位置设为原点（0,0,0,0）
     * @details 将当前传感器读数设为偏移量，使计算后的坐标为(0,0,0,0)
     */
    virtual void setCurrentAsOrigin();

protected:
    Config config_;   // 存储配置参数
    Target target_;   // 存储目标值
    Status status_;   // 存储当前状态

    /**
     * @brief 限制浮点数值在指定范围内
     */
    virtual float constrain(float value, float minVal, float maxVal);

    /**
     * @brief 更新传感器数据
     */
    virtual void updateSensorData();

    /**
     * @brief 坐标系转换：从地面坐标系到机身坐标系
     * @param x_ground 地面坐标系X
     * @param y_ground 地面坐标系Y  
     * @param z_ground 地面坐标系Z
     * @param yaw 偏航角 (rad)
     * @param x_body 输出机身坐标系X
     * @param y_body 输出机身坐标系Y
     * @param z_body 输出机身坐标系Z
     */
    virtual void transformGroundToBody(float x_ground, float y_ground, float z_ground, 
                                     float yaw, 
                                     float& x_body, float& y_body, float& z_body);

    /**
     * @brief 计算并更新位置误差的坐标系转换
     */
    virtual void updatePositionErrorTransform();

    /**
     * @brief 角度归一化到 [-π, π] 范围
     * @param angle 输入角度 (rad)
     * @return 归一化后的角度 (rad)
     */
    virtual float normalizeAngle(float angle);
};

#endif // MOVE_H
