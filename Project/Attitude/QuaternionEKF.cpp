/**
 * @file QuaternionEKF.cpp
 * @brief 基于四元数的扩展卡尔曼滤波器实现
 */

#include "QuaternionEKF.h"
#include "math_utils.h"

/**
 * @brief 构造函数
 */
QuaternionEKF::QuaternionEKF(float sampleFreq)
    : _P(7, 7), // 状态协方差矩阵 - 四元数(4) + 陀螺仪零偏(3) = 7维
      _Q(7, 7), // 过程噪声协方差
      _R(3, 3), // 测量噪声协方差
      _dt(1.0f / sampleFreq),
      // 初始化所有预分配的中间矩阵
      _F(7, 7),
      _F_transpose(7, 7),
      _H(3, 7),
      _H_transpose(7, 3),
      _S(3, 3),
      _S_inverse(3, 3),
      _K(7, 3),
      _residual(3, 1),
      _state_correction(7, 1),
      _I(7, 7),
      _temp_7x7(7, 7),
      _temp_7x3(7, 3)
{
    // 初始化
    reset();

    // 设置默认过程噪声
    _Q.setIdentity();
    _Q.scale(0.001f, _Q); // 默认值

    // 设置默认测量噪声
    _R.setIdentity();
    _R.scale(0.1f, _R); // 默认值
    
    // 初始化单位矩阵
    _I.setIdentity();
}

/**
 * @brief 初始化姿态估计器
 */
void QuaternionEKF::init()
{
    reset();
}

/**
 * @brief 更新姿态估计
 */
void QuaternionEKF::update(float gyro[3], float accel[3], float mag[3])
{
    // 状态预测
    stateTransition(gyro);

    // 测量更新
    measurementUpdate(accel);

    // 注意：当前简化实现不使用磁力计数据
}

/**
 * @brief 获取欧拉角
 */
void QuaternionEKF::getEulerRadians(float &roll, float &pitch, float &yaw)
{
    _quat.toEulerRad(roll, pitch, yaw);
}

/**
 * @brief 获取四元数
 */
void QuaternionEKF::getQuaternion(float q[4])
{
    q[0] = _quat.w;
    q[1] = _quat.x;
    q[2] = _quat.y;
    q[3] = _quat.z;
}

/**
 * @brief 重置姿态估计器
 */
void QuaternionEKF::reset()
{
    // 重置四元数为单位四元数
    _quat = MathUtils::Quaternion(1.0f, 0.0f, 0.0f, 0.0f);

    // 重置陀螺仪零偏
    _gyro_bias[0] = 0.0f;
    _gyro_bias[1] = 0.0f;
    _gyro_bias[2] = 0.0f;

    // 初始化状态协方差矩阵
    _P.setIdentity();

    // 四元数部分的方差初始化较小，因为我们比较确定初始姿态
    for (int i = 0; i < 4; i++)
    {
        _P(i, i) = 0.01f;
    }

    // 陀螺仪零偏部分的方差初始化较大，因为我们不确定初始零偏
    for (int i = 4; i < 7; i++)
    {
        _P(i, i) = 0.1f;
    }
}

/**
 * @brief 设置采样周期
 */
void QuaternionEKF::setSamplePeriod(float dt)
{
    _dt = dt;
}

/**
 * @brief 设置过程噪声协方差
 */
void QuaternionEKF::setProcessNoise(const float Q[7][7])
{
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            _Q(i, j) = Q[i][j];
        }
    }
}

/**
 * @brief 设置测量噪声协方差
 */
void QuaternionEKF::setMeasurementNoise(const float R[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            _R(i, j) = R[i][j];
        }
    }
}

/**
 * @brief 状态转移函数
 * @details 使用陀螺仪数据进行状态预测
 */
void QuaternionEKF::stateTransition(const float gyro[3])
{
    // 对陀螺仪数据进行零偏校正
    float gyro_corrected[3];
    gyro_corrected[0] = gyro[0] - _gyro_bias[0];
    gyro_corrected[1] = gyro[1] - _gyro_bias[1];
    gyro_corrected[2] = gyro[2] - _gyro_bias[2];

    // 计算角度变化（假设在短时间内角速度恒定）
    float angle_delta[3];
    angle_delta[0] = gyro_corrected[0] * _dt;
    angle_delta[1] = gyro_corrected[1] * _dt;
    angle_delta[2] = gyro_corrected[2] * _dt;

    // 计算角度变化的大小
    float angle_norm = math_sqrtf(angle_delta[0] * angle_delta[0] +
                             angle_delta[1] * angle_delta[1] +
                             angle_delta[2] * angle_delta[2]);

    // 创建表示姿态增量的四元数
    MathUtils::Quaternion q_delta;


    if (angle_norm > 1e-6f)
    {
        float half_angle = angle_norm * 0.5f;
        float sin_half_angle_over_angle = arm_sin_f32(half_angle) / angle_norm;

        q_delta.w = arm_cos_f32(half_angle);
        q_delta.x = angle_delta[0] * sin_half_angle_over_angle;
        q_delta.y = angle_delta[1] * sin_half_angle_over_angle;
        q_delta.z = angle_delta[2] * sin_half_angle_over_angle;
    }
    else
    {
        // 角度变化很小时的近似计算
        q_delta.w = 1.0f;
        q_delta.x = angle_delta[0] * 0.5f;
        q_delta.y = angle_delta[1] * 0.5f;
        q_delta.z = angle_delta[2] * 0.5f;
    }

    // 更新四元数状态
    _quat = _quat * q_delta;

    // 归一化四元数
    _quat.normalize();

    // 计算状态转移矩阵
    calculateF(gyro_corrected, _F);

    // 预测状态协方差
    // P = F * P * F^T + Q
    // 使用预分配矩阵避免创建临时对象
    _F.transpose(_F_transpose);
    _F.multiply(_P, _temp_7x7); // temp_7x7 = F * P
    _temp_7x7.multiply(_F_transpose, _P); // P = temp_7x7 * F_transpose
    _P.add(_Q, _P); // P = P + Q
}

/**
 * @brief 测量更新函数
 * @details 使用加速度计数据更新状态估计
 */
void QuaternionEKF::measurementUpdate(const float accel[3])
{
    // 归一化加速度计读数
    float accel_norm = math_sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

    // 检查加速度计数据是否有效
    if (accel_norm < 1e-6f)
    {
        return; // 加速度计数据无效，跳过测量更新
    }

    float accel_normalized[3];
    accel_normalized[0] = accel[0] / accel_norm;
    accel_normalized[1] = accel[1] / accel_norm;
    accel_normalized[2] = accel[2] / accel_norm;

    // 预测的重力方向（在机体坐标系中）
    float accel_predicted[3];
    predictAccel(accel_predicted);

    // 计算测量残差
    _residual(0, 0) = accel_normalized[0] - accel_predicted[0];
    _residual(1, 0) = accel_normalized[1] - accel_predicted[1];
    _residual(2, 0) = accel_normalized[2] - accel_predicted[2];

    // 计算测量雅可比矩阵
    calculateH(_H);

    // 计算卡尔曼增益
    // K = P * H^T * (H * P * H^T + R)^(-1)
    // 使用预分配矩阵避免创建临时对象
    _H.transpose(_H_transpose);
    _H.multiply(_P, _temp_7x3); // temp_7x3 = H * P
    _temp_7x3.multiply(_H_transpose, _S); // S = temp_7x3 * H_transpose = H * P * H^T
    _S.add(_R, _S); // S = S + R
    
    // 计算S的逆矩阵
    _S.inverse(_S_inverse);
    
    // 计算卡尔曼增益
    _P.multiply(_H_transpose, _temp_7x3); // temp_7x3 = P * H^T
    _temp_7x3.multiply(_S_inverse, _K); // K = temp_7x3 * S_inverse = P * H^T * S^(-1)

    // 更新状态估计
    _K.multiply(_residual, _state_correction); // state_correction = K * residual

    // 应用四元数校正
    MathUtils::Quaternion q_correction(
        1.0f,
        _state_correction(0, 0),
        _state_correction(1, 0),
        _state_correction(2, 0));

    // 归一化校正四元数
    q_correction.normalize();

    // 应用四元数校正
    _quat = _quat * q_correction;
    _quat.normalize();

    // 更新陀螺仪零偏
    _gyro_bias[0] += _state_correction(4, 0);
    _gyro_bias[1] += _state_correction(5, 0);
    _gyro_bias[2] += _state_correction(6, 0);

    // 更新状态协方差
    // P = (I - K * H) * P
    _K.multiply(_H, _temp_7x7); // temp_7x7 = K * H
    for (uint32_t i = 0; i < 7; i++) {
        for (uint32_t j = 0; j < 7; j++) {
            _temp_7x7(i, j) = _I(i, j) - _temp_7x7(i, j); // temp_7x7 = I - K * H
        }
    }
    _temp_7x7.multiply(_P, _P); // P = temp_7x7 * P = (I - K * H) * P
}

/**
 * @brief 计算状态转移矩阵
 * @details 计算状态转移的雅可比矩阵F
 */
void QuaternionEKF::calculateF(const float gyro[3], MathUtils::Matrix &F)
{
    F.setIdentity();

    // 四元数对四元数的雅可比矩阵
    F(0, 0) = 1.0f;
    F(0, 1) = -0.5f * gyro[0] * _dt;
    F(0, 2) = -0.5f * gyro[1] * _dt;
    F(0, 3) = -0.5f * gyro[2] * _dt;

    F(1, 0) = 0.5f * gyro[0] * _dt;
    F(1, 1) = 1.0f;
    F(1, 2) = 0.5f * gyro[2] * _dt;
    F(1, 3) = -0.5f * gyro[1] * _dt;

    F(2, 0) = 0.5f * gyro[1] * _dt;
    F(2, 1) = -0.5f * gyro[2] * _dt;
    F(2, 2) = 1.0f;
    F(2, 3) = 0.5f * gyro[0] * _dt;

    F(3, 0) = 0.5f * gyro[2] * _dt;
    F(3, 1) = 0.5f * gyro[1] * _dt;
    F(3, 2) = -0.5f * gyro[0] * _dt;
    F(3, 3) = 1.0f;

    // 四元数对陀螺仪零偏的雅可比矩阵
    F(0, 4) = 0.5f * _quat.x * _dt;
    F(0, 5) = 0.5f * _quat.y * _dt;
    F(0, 6) = 0.5f * _quat.z * _dt;

    F(1, 4) = -0.5f * _quat.w * _dt;
    F(1, 5) = -0.5f * _quat.z * _dt;
    F(1, 6) = 0.5f * _quat.y * _dt;

    F(2, 4) = 0.5f * _quat.z * _dt;
    F(2, 5) = -0.5f * _quat.w * _dt;
    F(2, 6) = -0.5f * _quat.x * _dt;

    F(3, 4) = -0.5f * _quat.y * _dt;
    F(3, 5) = 0.5f * _quat.x * _dt;
    F(3, 6) = -0.5f * _quat.w * _dt;
}

/**
 * @brief 计算测量雅可比矩阵
 * @details 计算测量方程的雅可比矩阵H
 */
void QuaternionEKF::calculateH(MathUtils::Matrix &H)
{
    // 初始化H为零矩阵
    for (uint32_t i = 0; i < H.rows(); i++)
    {
        for (uint32_t j = 0; j < H.cols(); j++)
        {
            H(i, j) = 0.0f;
        }
    }

    // 重力向量对四元数的雅可比矩阵
    // 计算重力向量对四元数的偏导数
    float qw = _quat.w;
    float qx = _quat.x;
    float qy = _quat.y;
    float qz = _quat.z;

    // dR_dq[0] - 加速度计x轴对四元数各分量的偏导数
    H(0, 0) = 2 * (qy * qz - qw * qx);
    H(0, 1) = -2 * qw;
    H(0, 2) = 2 * qz;
    H(0, 3) = 2 * qy;

    // dR_dq[1] - 加速度计y轴对四元数各分量的偏导数
    H(1, 0) = 2 * (qw * qy + qx * qz);
    H(1, 1) = 2 * qz;
    H(1, 2) = 2 * qw;
    H(1, 3) = 2 * qx;

    // dR_dq[2] - 加速度计z轴对四元数各分量的偏导数
    H(2, 0) = 2 * (qw * qz - qx * qy);
    H(2, 1) = -2 * qy;
    H(2, 2) = -2 * qx;
    H(2, 3) = 2 * qw;
}

/**
 * @brief 计算预测的重力方向
 * @details 使用当前姿态四元数计算重力方向在机体坐标系中的表示
 */
void QuaternionEKF::predictAccel(float accel_pred[3])
{
    // 在世界坐标系中，重力方向为 [0, 0, -1]
    float gravity_world[3] = {0.0f, 0.0f, -1.0f};

    // 使用共轭四元数将世界坐标系中的重力旋转到机体坐标系
    MathUtils::Quaternion q_conj = _quat.conjugate();
    q_conj.rotateVector(gravity_world, accel_pred);
}