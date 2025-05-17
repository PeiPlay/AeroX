/**
 * @file MahonyAHRS.h
 * @brief Mahony姿态航向参考系统实现
 * @details 实现了基于Mahony算法的互补滤波姿态估计器
 */

#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

#include "Attitude.h"

/**
 * @brief Mahony姿态估计器类
 * @details 实现基于Mahony算法的互补滤波姿态估计
 */
class MahonyAHRS : public AttitudeEstimator
{
public:
    /**
     * @brief 构造函数
     * @param sampleFreq 采样频率，单位：Hz
     * @param Kp 比例增益
     * @param Ki 积分增益
     */
    MahonyAHRS(float sampleFreq = 500.0f, float Kp = 0.6f, float Ki = 0.02f);

    /**
     * @brief 析构函数
     */
    virtual ~MahonyAHRS() = default;

    /**
     * @brief 初始化姿态估计器
     * @param accel 加速度计数据，用于初始化姿态
     * @param mag 磁力计数据，用于初始化偏航角（可选）
     */
    virtual void init(float accel[3] = nullptr, float mag[3] = nullptr);

    /**
     * @brief 更新姿态估计
     * @param gyro 陀螺仪数据（角速度），单位：rad/s
     * @param accel 加速度计数据，单位：m/s^2
     * @param mag 磁力计数据（可选），单位：任意，仅用于磁北对准
     */
    virtual void update(float gyro[3], float accel[3], float mag[3] = nullptr) override;

    /**
     * @brief 获取欧拉角
     * @param roll 滚转角（绕X轴旋转），单位：rad
     * @param pitch 俯仰角（绕Y轴旋转），单位：rad
     * @param yaw 偏航角（绕Z轴旋转），单位：rad
     */
    virtual void getEulerRadians(float &roll, float &pitch, float &yaw) override;

    /**
     * @brief 获取四元数
     * @param q 四元数数组，q[0]为实部，q[1:3]为虚部
     */
    virtual void getQuaternion(float q[4]) override;

    /**
     * @brief 重置姿态估计器状态
     */
    virtual void reset() override;

    /**
     * @brief 设置采样周期
     * @param dt 采样周期，单位：秒
     */
    virtual void setSamplePeriod(float dt) override;

    /**
     * @brief 设置比例增益
     * @param Kp 比例增益
     */
    void setKp(float Kp);

    /**
     * @brief 设置积分增益
     * @param Ki 积分增益
     */
    void setKi(float Ki);

private:
    // 四元数表示的姿态
    float _q0, _q1, _q2, _q3;

    // 预计算的欧拉角，在update时更新
    float _roll, _pitch, _yaw;

    // 积分误差
    float _integralFBx, _integralFBy, _integralFBz;

    // 采样频率（Hz）
    float _sampleFreq;

    // 采样周期（秒）
    float _invSampleFreq;

    // 控制参数
    float _Kp;
    float _Ki;

    // 加速度重力向量和磁北向量的归一化
    void normalizeVectors(float ax, float ay, float az, float &nx, float &ny, float &nz,
                          float mx, float my, float mz, float &wx, float &wy, float &wz);

    // 计算欧拉角（内部方法，仅在update中调用）
    void computeEulerRadians();
};

#endif // MAHONY_AHRS_H