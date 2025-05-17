#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__
#include <stdint.h>


/**
 * @brief IMU传感器抽象基类
 * @details 定义了所有IMU传感器必须实现的接口
 */
class IMU {
public:
    /**
     * @brief 初始化IMU传感器
     * @return 初始化是否成功
     */
    virtual bool init() = 0;
    
    /**
     * @brief 读取IMU传感器数据
     * @param gyro 陀螺仪数据（角速度），单位：rad/s
     * @param accel 加速度计数据，单位：m/s^2
     */
    virtual void read(float gyro[3], float accel[3]) = 0;
    
    /**
     * @brief 虚析构函数，确保派生类被正确析构
     */
    virtual ~IMU() = default;
};


/**
 * @brief 姿态估计器抽象基类
 */
class AttitudeEstimator {
public:
    /**
     * @brief 更新姿态估计
     * @param gyro 陀螺仪数据（角速度），单位：rad/s
     * @param accel 加速度计数据，单位：m/s^2
     * @param mag 磁力计数据（可选），单位：任意，仅用于磁北对准
     */
    virtual void update(float gyro[3], float accel[3], float mag[3] = nullptr) = 0;
    
    /**
     * @brief 获取欧拉角
     * @param roll 滚转角（绕X轴旋转），单位：rad
     * @param pitch 俯仰角（绕Y轴旋转），单位：rad
     * @param yaw 偏航角（绕Z轴旋转），单位：rad
     */
    virtual void getEulerRadians(float& roll, float& pitch, float& yaw) = 0;
    
    /**
     * @brief 获取四元数
     * @param q 四元数数组，q[0]为实部，q[1:3]为虚部
     */
    virtual void getQuaternion(float q[4]) = 0;
    
    /**
     * @brief 重置姿态估计器状态
     */
    virtual void reset() = 0;
    
    /**
     * @brief 设置采样周期
     * @param dt 采样周期，单位：秒
     */
    virtual void setSamplePeriod(float dt) = 0;
    
    /**
     * @brief 虚析构函数，确保派生类被正确析构
     */
    virtual ~AttitudeEstimator() = default;
};




/**
 * @brief 姿态管理器类
 * @details 将IMU数据采集、姿态解算和数据传输整合在一起
 */
class AttitudeManager {
public:
    /**
     * @brief 构造函数
     * @param imu IMU传感器对象
     * @param estimator 姿态估计器对象
     */
    AttitudeManager(IMU* imu, AttitudeEstimator* estimator);
    
    /**
     * @brief 析构函数
     */
    ~AttitudeManager();
    
    /**
     * @brief 初始化姿态管理器
     * @return 是否成功
     */
    bool init();
    
    /**
     * @brief 单步更新（适用于循环调用）
     */
    void update();
    
    /**
     * @brief 获取当前欧拉角姿态
     * @param roll 横滚角（rad）
     * @param pitch 俯仰角（rad）
     * @param yaw 偏航角（rad）
     */
    void getAttitude(float& roll, float& pitch, float& yaw);

    /**
     * @brief 获取当前陀螺仪数据
     * @param gyro 陀螺仪数据数组，单位：rad/s
     */
    void getGyro(float gyro[3]);

    /**
     * @brief 获取当前加速度计数据
     * @param accel 加速度计数据数组，单位：m/s^2
     */
    void getAccel(float accel[3]);
    
    /**
     * @brief 获取当前四元数姿态
     * @param q 四元数数组，q[0]为实部，q[1:3]为虚部
     */
    void getQuaternion(float q[4]);
    
    
private:
    // 依赖的组件
    IMU* _imu;
    AttitudeEstimator* _estimator;
    
    // 内部状态
    bool _isInitialized;
    
    // IMU数据缓冲
    float _gyro[3];
    float _accel[3];
};




#endif /* __ATTITUDE_H__ */