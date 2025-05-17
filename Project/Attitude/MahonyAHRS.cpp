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
void MahonyAHRS::init(float accel[3], float mag[3])
{
    if (accel == nullptr)
    {
        reset();
        return;
    }

    float ax = accel[0], ay = accel[1], az = accel[2];

    float mx, my, mz;
    if (mag == nullptr)
    {
        mx = my = mz = 0.0f;
    }
    else
    {
        mx = mag[0];
        my = mag[1];
        mz = mag[2];
    }
    // 归一化加速度
    float recipNorm = 1.0f / math_sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    // 归一化磁力计（若有效）
    if (mx != 0.0f || my != 0.0f || mz != 0.0f)
    {
        recipNorm = 1.0f / math_sqrtf(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
    }

    // 计算欧拉角
    float init_pitch = atan2f(-ax, az);
    float init_roll = atan2f(ay, az);

    // 使用ARM数学库的sin_cos函数同时计算sin和cos
    float sin_r, cos_r, sin_p, cos_p;

    arm_sin_cos_f32(init_roll, &sin_r, &cos_r);  // 同时计算sin(roll)和cos(roll)
    arm_sin_cos_f32(init_pitch, &sin_p, &cos_p); // 同时计算sin(pitch)和cos(pitch)

    float init_yaw = 0.0f;
    if (mx != 0.0f || my != 0.0f || mz != 0.0f)
    {
        float magX = mx * cos_p + my * sin_p * sin_r + mz * sin_p * cos_r;
        float magY = my * cos_r - mz * sin_r;
        init_yaw = atan2f(-magY, magX);
    }

    // 使用ARM数学库的sin_cos函数同时计算sin和cos (用于四元数计算)
    float cr2, cp2, cy2, sr2, sp2, sy2;
    arm_sin_cos_f32(init_roll * 0.5f, &sr2, &cr2);  // 同时计算sin(roll/2)和cos(roll/2)
    arm_sin_cos_f32(init_pitch * 0.5f, &sp2, &cp2); // 同时计算sin(pitch/2)和cos(pitch/2)
    arm_sin_cos_f32(init_yaw * 0.5f, &sy2, &cy2);   // 同时计算sin(yaw/2)和cos(yaw/2)

    _q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
    _q1 = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
    _q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
    _q3 = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;
    // 归一化四元数
    recipNorm = 1.0f / math_sqrtf(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
    _q0 *= recipNorm;
    _q1 *= recipNorm;
    _q2 *= recipNorm;
    _q3 *= recipNorm;

    // 计算并更新欧拉角
    computeEulerRadians();
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

    // 使用本地变量进行计算，避免直接操作成员变量
    float q0 = _q0, q1 = _q1, q2 = _q2, q3 = _q3;

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
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

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

    // 应用比例反馈 (乘以 2.0f 以匹配旧版本的 twoKpDef)
    // 注意：旧版本无论是否有磁力计都使用 twoKpDef
    float twoKp = 2.0f * _Kp;
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;


    // 应用积分反馈 (乘以 2.0f 以匹配旧版本的 twoKi)
    // 注意：旧版本无论是否有磁力计都使用 twoKi
    if (_Ki > 0.0f)
    {
        float twoKi = 2.0f * _Ki; // 匹配旧版本的 twoKi
        _integralFBx += twoKi * halfex * _invSampleFreq;
        _integralFBy += twoKi * halfey * _invSampleFreq;
        _integralFBz += twoKi * halfez * _invSampleFreq;

        gx += _integralFBx;
        gy += _integralFBy;
        gz += _integralFBz;
    }
    else // 如果 Ki 为 0，确保积分项为 0 (匹配旧版本逻辑)
    {
        _integralFBx = 0.0f;
        _integralFBy = 0.0f;
        _integralFBz = 0.0f;
    }


    // 积分四元数率并归一化 - 使用局部变量避免中间状态被读取
    gx *= (0.5f * _invSampleFreq);
    gy *= (0.5f * _invSampleFreq);
    gz *= (0.5f * _invSampleFreq);

    float qa = q0;
    float qb = q1;
    float qc = q2;
    float qd = q3;

    q0 = qa + (-qb * gx - qc * gy - qd * gz);
    q1 = qb + (qa * gx + qc * gz - qd * gy);
    q2 = qc + (qa * gy - qb * gz + qd * gx);
    q3 = qd + (qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = 1.0f / math_sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    // 原子式地更新成员变量四元数
    _q0 = q0 * recipNorm;
    _q1 = q1 * recipNorm;
    _q2 = q2 * recipNorm;
    _q3 = q3 * recipNorm;

    // 更新时直接计算欧拉角
    computeEulerRadians();
}

/**
 * @brief 获取欧拉角
 */
void MahonyAHRS::getEulerRadians(float &roll, float &pitch, float &yaw)
{
    // 直接返回预计算的欧拉角，无需重新计算
    roll = _roll;
    pitch = _pitch;
    yaw = _yaw;
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

    _roll = 0.0f;
    _pitch = 0.0f;
    _yaw = 0.0f;

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
void MahonyAHRS::computeEulerRadians()
{
    // 从四元数计算欧拉角并直接更新成员变量
    _roll = atan2f(2.0f * (_q0 * _q1 + _q2 * _q3), 1.0f - 2.0f * (_q1 * _q1 + _q2 * _q2));
    _pitch = asinf(2.0f * (_q0 * _q2 - _q3 * _q1));
    _yaw = atan2f(2.0f * (_q0 * _q3 + _q1 * _q2), 1.0f - 2.0f * (_q2 * _q2 + _q3 * _q3));
}