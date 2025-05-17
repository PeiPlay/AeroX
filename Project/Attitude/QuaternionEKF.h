/**
 * @file QuaternionEKF.h
 * @brief 基于四元数的扩展卡尔曼滤波器
 * @details 使用扩展卡尔曼滤波算法进行姿态估计
 */

#ifndef QUATERNION_EKF_H
#define QUATERNION_EKF_H

#include "Attitude.h"
#include "math_utils.h"

/**
 * @brief 四元数扩展卡尔曼滤波器类
 * @details 实现基于四元数的扩展卡尔曼滤波姿态估计
 */
class QuaternionEKF : public AttitudeEstimator
{
public:
    /**
     * @brief 构造函数
     * @param sampleFreq 采样频率，单位：Hz
     */
    QuaternionEKF(float sampleFreq = 500.0f);

    /**
     * @brief 析构函数
     */
    virtual ~QuaternionEKF() = default;

    /**
     * @brief 初始化姿态估计器
     */
    virtual void init();

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
     * @brief 设置过程噪声协方差
     * @param Q 过程噪声协方差矩阵，大小为7x7
     */
    void setProcessNoise(const float Q[7][7]);

    /**
     * @brief 设置测量噪声协方差
     * @param R 测量噪声协方差矩阵，大小为3x3
     */
    void setMeasurementNoise(const float R[3][3]);

private:
    // 状态变量
    utils::math::Quaternion _quat; // 四元数姿态
    float _gyro_bias[3];         // 陀螺仪零偏

    // 状态协方差矩阵 (7x7)
    utils::math::Matrix _P;

    // 过程噪声协方差矩阵 (7x7)
    utils::math::Matrix _Q;

    // 测量噪声协方差矩阵 (3x3)
    utils::math::Matrix _R;

    // 采样时间
    float _dt;
    
    // 预先分配的中间矩阵，避免频繁申请和释放内存
    utils::math::Matrix _F;            // 状态转移矩阵 (7x7)
    utils::math::Matrix _F_transpose;  // F的转置 (7x7)
    utils::math::Matrix _H;            // 测量雅可比矩阵 (3x7)
    utils::math::Matrix _H_transpose;  // H的转置 (7x3)
    utils::math::Matrix _S;            // 新息协方差矩阵 (3x3)
    utils::math::Matrix _S_inverse;    // S的逆矩阵 (3x3)
    utils::math::Matrix _K;            // 卡尔曼增益 (7x3)
    utils::math::Matrix _residual;     // 测量残差 (3x1)
    utils::math::Matrix _state_correction; // 状态校正 (7x1)
    utils::math::Matrix _I;            // 单位矩阵 (7x7)
    utils::math::Matrix _temp_7x7;     // 临时矩阵，用于存储计算中间结果 (7x7)
    utils::math::Matrix _temp_7x3;     // 临时矩阵，用于存储计算中间结果 (7x3)

    // 状态转移函数
    void stateTransition(const float gyro[3]);

    // 测量更新函数
    void measurementUpdate(const float accel[3]);

    // 状态矩阵和雅可比矩阵
    void calculateF(const float gyro[3], utils::math::Matrix &F);
    void calculateH(utils::math::Matrix &H);

    // 计算预测的重力方向
    void predictAccel(float accel_pred[3]);
};

#endif // QUATERNION_EKF_H