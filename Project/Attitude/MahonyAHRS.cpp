/**
 * @file MahonyAHRS.cpp
 * @brief Mahony姿态航向参考系统实现
 */

#include "MahonyAHRS.h"
#include "math_utils.h"

/**
 * @brief 构造函数
 */
MahonyAHRS::MahonyAHRS(float sampleFreq, float Kp, float Ki)
    : _sampleFreq(sampleFreq),
      _Kp(Kp),
      _Ki(Ki)
{
    _invSampleFreq = 1.0f / _sampleFreq;
    reset();
}

/**
 * @brief 初始化姿态估计器
 */
void MahonyAHRS::init()
{
    reset();
}

/**
 * @brief 更新姿态估计
 */
void MahonyAHRS::update(float gyro[3], float accel[3], float mag[3])
{
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];
    float ax = accel[0], ay = accel[1], az = accel[2];
    float mx = 0.0f, my = 0.0f, mz = 0.0f;

    // 用于磁力计的临时变量
    float nx, ny, nz;
    float wx, wy, wz;

    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 如果加速度计有效（非零），进行归一化
    recipNorm = 0.0f;
    if ((ax != 0.0f) || (ay != 0.0f) || (az != 0.0f))
    {
        recipNorm = 1.0f / math_sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
    }

    // 如果磁力计数据有效，进行归一化
    if (mag != nullptr && ((mag[0] != 0.0f) || (mag[1] != 0.0f) || (mag[2] != 0.0f)))
    {
        mx = mag[0];
        my = mag[1];
        mz = mag[2];

        recipNorm = 1.0f / math_sqrtf(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
    }

    // 计算四元数的辅助变量
    q0q0 = _q0 * _q0;
    q0q1 = _q0 * _q1;
    q0q2 = _q0 * _q2;
    q0q3 = _q0 * _q3;
    q1q1 = _q1 * _q1;
    q1q2 = _q1 * _q2;
    q1q3 = _q1 * _q3;
    q2q2 = _q2 * _q2;
    q2q3 = _q2 * _q3;
    q3q3 = _q3 * _q3;

    // 参考方向计算在机体坐标系中的地球重力方向
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;

    // 如果有磁力计数据，计算地球磁场方向
    if (mag != nullptr && ((mx != 0.0f) || (my != 0.0f) || (mz != 0.0f)))
    {
        // 参考方向计算在机体坐标系中的地球磁场方向
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = math_sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // 参考方向计算在机体坐标系中的地球磁场方向
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // 计算测量方向与参考方向的误差
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
    }
    else
    {
        // 只使用加速度计数据计算误差
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);
    }

    // 应用比例反馈
    if (_Kp > 0.0f)
    {
        gx += _Kp * halfex;
        gy += _Kp * halfey;
        gz += _Kp * halfez;
    }

    // 应用积分反馈
    if (_Ki > 0.0f)
    {
        _integralFBx += _Ki * halfex * _invSampleFreq;
        _integralFBy += _Ki * halfey * _invSampleFreq;
        _integralFBz += _Ki * halfez * _invSampleFreq;

        gx += _integralFBx;
        gy += _integralFBy;
        gz += _integralFBz;
    }

    // 积分四元数率并归一化
    gx *= (0.5f * _invSampleFreq);
    gy *= (0.5f * _invSampleFreq);
    gz *= (0.5f * _invSampleFreq);
    qa = _q0;
    qb = _q1;
    qc = _q2;
    _q0 += (-qb * gx - qc * gy - _q3 * gz);
    _q1 += (qa * gx + qc * gz - _q3 * gy);
    _q2 += (qa * gy - qb * gz + _q3 * gx);
    _q3 += (qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = 1.0f / math_sqrtf(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
    _q0 *= recipNorm;
    _q1 *= recipNorm;
    _q2 *= recipNorm;
    _q3 *= recipNorm;
}

/**
 * @brief 获取欧拉角
 */
void MahonyAHRS::getEulerRadians(float &roll, float &pitch, float &yaw)
{
    computeEulerAngles(roll, pitch, yaw);
}

/**
 * @brief 获取四元数
 */
void MahonyAHRS::getQuaternion(float q[4])
{
    q[0] = _q0;
    q[1] = _q1;
    q[2] = _q2;
    q[3] = _q3;
}

/**
 * @brief 重置姿态估计器
 */
void MahonyAHRS::reset()
{
    _q0 = 1.0f;
    _q1 = 0.0f;
    _q2 = 0.0f;
    _q3 = 0.0f;

    _integralFBx = 0.0f;
    _integralFBy = 0.0f;
    _integralFBz = 0.0f;
}

/**
 * @brief 设置采样周期
 */
void MahonyAHRS::setSamplePeriod(float dt)
{
    _sampleFreq = 1.0f / dt;
    _invSampleFreq = dt;
}

/**
 * @brief 设置比例增益
 */
void MahonyAHRS::setKp(float Kp)
{
    _Kp = Kp;
}

/**
 * @brief 设置积分增益
 */
void MahonyAHRS::setKi(float Ki)
{
    _Ki = Ki;
}

/**
 * @brief 归一化向量
 */
void MahonyAHRS::normalizeVectors(float ax, float ay, float az, float &nx, float &ny, float &nz,
                                  float mx, float my, float mz, float &wx, float &wy, float &wz)
{
    float recipNorm;

    // 归一化加速度
    recipNorm = 1.0f / math_sqrtf(ax * ax + ay * ay + az * az);
    nx = ax * recipNorm;
    ny = ay * recipNorm;
    nz = az * recipNorm;

    // 归一化磁力计数据
    recipNorm = 1.0f / math_sqrtf(mx * mx + my * my + mz * mz);
    wx = mx * recipNorm;
    wy = my * recipNorm;
    wz = mz * recipNorm;
}

/**
 * @brief 计算欧拉角
 */
void MahonyAHRS::computeEulerAngles(float &roll, float &pitch, float &yaw)
{
    // 从四元数计算欧拉角
    roll = atan2f(2.0f * (_q0 * _q1 + _q2 * _q3), 1.0f - 2.0f * (_q1 * _q1 + _q2 * _q2));
    pitch = asinf(2.0f * (_q0 * _q2 - _q3 * _q1));
    yaw = atan2f(2.0f * (_q0 * _q3 + _q1 * _q2), 1.0f - 2.0f * (_q2 * _q2 + _q3 * _q3));
}