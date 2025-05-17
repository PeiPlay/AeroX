#include "math_quaternion.h"
#include <math.h> // Include for standard math functions if needed, though CMSIS is preferred

namespace utils {
namespace math {

// 构造函数
Quaternion::Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}

Quaternion::Quaternion(float w, float x, float y, float z)
    : w(w), x(x), y(y), z(z) {}

// 从欧拉角创建四元数 (弧度单位)
Quaternion Quaternion::fromEulerRad(float roll, float pitch, float yaw) {
    // 计算各个角度的一半的正弦和余弦值
    float cr = arm_cos_f32(roll * 0.5f);
    float sr = arm_sin_f32(roll * 0.5f);
    float cp = arm_cos_f32(pitch * 0.5f);
    float sp = arm_sin_f32(pitch * 0.5f);
    float cy = arm_cos_f32(yaw * 0.5f);
    float sy = arm_sin_f32(yaw * 0.5f);
    
    // ZYX顺序的四元数构造公式 - 先yaw(Z)，再pitch(Y)，最后roll(X)
    // 参考: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    float qw = cr * cp * cy + sr * sp * sy; // Corrected sign
    float qx = sr * cp * cy - cr * sp * sy; // Corrected sign
    float qy = cr * sp * cy + sr * cp * sy; // Corrected sign
    float qz = cr * cp * sy - sr * sp * cy; // Corrected sign
    
    return Quaternion(qw, qx, qy, qz);
}

// 从欧拉角创建四元数 (角度单位)
Quaternion Quaternion::fromEulerDeg(float roll, float pitch, float yaw) {
    return fromEulerRad(roll * DEG_TO_RAD, pitch * DEG_TO_RAD, yaw * DEG_TO_RAD);
}

// 从旋转矩阵创建四元数
Quaternion Quaternion::fromRotationMatrix(const float matrix[9]) {
    Quaternion q;
    float trace = matrix[0] + matrix[4] + matrix[8];
    
    if (trace > 0.0f) {
        float s = 0.5f / math_sqrtf(trace + 1.0f);
        q.w = 0.25f / s;
        q.x = (matrix[7] - matrix[5]) * s;
        q.y = (matrix[2] - matrix[6]) * s;
        q.z = (matrix[3] - matrix[1]) * s;
    } else {
        if (matrix[0] > matrix[4] && matrix[0] > matrix[8]) {
            float s = 2.0f * math_sqrtf(1.0f + matrix[0] - matrix[4] - matrix[8]);
            q.w = (matrix[7] - matrix[5]) / s;
            q.x = 0.25f * s;
            q.y = (matrix[1] + matrix[3]) / s;
            q.z = (matrix[2] + matrix[6]) / s;
        } else if (matrix[4] > matrix[8]) {
            float s = 2.0f * math_sqrtf(1.0f + matrix[4] - matrix[0] - matrix[8]);
            q.w = (matrix[2] - matrix[6]) / s;
            q.x = (matrix[1] + matrix[3]) / s;
            q.y = 0.25f * s;
            q.z = (matrix[5] + matrix[7]) / s;
        } else {
            float s = 2.0f * math_sqrtf(1.0f + matrix[8] - matrix[0] - matrix[4]);
            q.w = (matrix[3] - matrix[1]) / s;
            q.x = (matrix[2] + matrix[6]) / s;
            q.y = (matrix[5] + matrix[7]) / s;
            q.z = 0.25f * s;
        }
    }
    q.normalize();
    return q;
}

// 从轴角表示法创建四元数
Quaternion Quaternion::fromAxisAngle(float x, float y, float z, float angle_rad) {
    float length = math_sqrtf(x*x + y*y + z*z);
    if (length < FLOAT_EPSILON) {
        return Quaternion(); // 返回单位四元数
    }
    
    // 归一化轴向量
    x /= length;
    y /= length;
    z /= length;
    
    float half_angle = angle_rad * 0.5f;
    float sin_ha = arm_sin_f32(half_angle);
    
    return Quaternion(arm_cos_f32(half_angle), x * sin_ha, y * sin_ha, z * sin_ha);
}

// 基本操作
float Quaternion::norm() const {
    return math_sqrtf(w*w + x*x + y*y + z*z);
}

void Quaternion::normalize() {
    float n = norm();
    if (n > FLOAT_EPSILON) {
        float inv_n = 1.0f / n;
        w *= inv_n;
        x *= inv_n;
        y *= inv_n;
        z *= inv_n;
    } else {
        setIdentity(); // 如果模长接近零，设为单位四元数
    }
}

Quaternion Quaternion::normalized() const {
    Quaternion result(*this);
    result.normalize();
    return result;
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

Quaternion Quaternion::inverse() const {
    float n_squared = w*w + x*x + y*y + z*z;
    if (n_squared < FLOAT_EPSILON) {
        return Quaternion(); // 返回单位四元数
    }
    
    float inv_n_squared = 1.0f / n_squared;
    return Quaternion(w * inv_n_squared, -x * inv_n_squared, -y * inv_n_squared, -z * inv_n_squared);
}

// 运算符重载
Quaternion Quaternion::operator+(const Quaternion& q) const {
    return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
}

Quaternion Quaternion::operator-(const Quaternion& q) const {
    return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z);
}

Quaternion Quaternion::operator*(const Quaternion& q) const {
    return Quaternion(
        w * q.w - x * q.x - y * q.y - z * q.z,
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w
    );
}

Quaternion Quaternion::operator*(float scalar) const {
    return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
}

Quaternion operator*(float scalar, const Quaternion& q) {
    return q * scalar;
}

bool Quaternion::operator==(const Quaternion& q) const {
    return fabsf(w - q.w) < FLOAT_EPSILON && 
           fabsf(x - q.x) < FLOAT_EPSILON && 
           fabsf(y - q.y) < FLOAT_EPSILON && 
           fabsf(z - q.z) < FLOAT_EPSILON;
}

bool Quaternion::operator!=(const Quaternion& q) const {
    return !(*this == q);
}

// 四元数到欧拉角的转换
void Quaternion::toEulerRad(float& roll, float& pitch, float& yaw) const {
    // 先确保四元数是单位四元数
    Quaternion q = normalized();
    
    // ZYX顺序（航空角）的欧拉角计算
    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    roll = atan2f(sinr_cosp, cosr_cosp);
    
    // pitch (y-axis rotation)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f)
        pitch = copysignf(PI / 2.0f, sinp); // 使用90度作为极限情况
    else
        pitch = asinf(sinp);
    
    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    yaw = atan2f(siny_cosp, cosy_cosp);
}

void Quaternion::toEulerDeg(float& roll, float& pitch, float& yaw) const {
    toEulerRad(roll, pitch, yaw);
    roll *= RAD_TO_DEG;
    pitch *= RAD_TO_DEG;
    yaw *= RAD_TO_DEG;
}

// 四元数到旋转矩阵的转换
void Quaternion::toRotationMatrix(float matrix[9]) const {
    float xx = x * x;
    float xy = x * y;
    float xz = x * z;
    float xw = x * w;
    float yy = y * y;
    float yz = y * z;
    float yw = y * w;
    float zz = z * z;
    float zw = z * w;

    matrix[0] = 1.0f - 2.0f * (yy + zz);
    matrix[1] = 2.0f * (xy - zw);
    matrix[2] = 2.0f * (xz + yw);
    
    matrix[3] = 2.0f * (xy + zw);
    matrix[4] = 1.0f - 2.0f * (xx + zz);
    matrix[5] = 2.0f * (yz - xw);
    
    matrix[6] = 2.0f * (xz - yw);
    matrix[7] = 2.0f * (yz + xw);
    matrix[8] = 1.0f - 2.0f * (xx + yy);
}

// 四元数到轴角的转换
void Quaternion::toAxisAngle(float& x, float& y, float& z, float& angle_rad) const {
    Quaternion q = *this;
    if (q.w > 1.0f) { // 如果w>1则四元数不是规范化的
        q.normalize();
    }

    // 确保w在[-1,1]范围内以避免数值误差
    float w_clamped = (q.w < -1.0f) ? -1.0f : ((q.w > 1.0f) ? 1.0f : q.w);
    
    angle_rad = 2.0f * acosf(w_clamped);
    float s = sqrtf(1.0f - w_clamped * w_clamped);
    
    // 当角度非常小时，使用极限值
    if (s < FLOAT_EPSILON) {
        // 对于非常小的角度，向量部分几乎为零，此时任何轴都可以
        // 但为了保持连续性，我们使用四元数的虚部方向
        float norm = sqrtf(q.x*q.x + q.y*q.y + q.z*q.z);
        if (norm < FLOAT_EPSILON) {
            // 如果虚部也接近零，则使用默认轴
            x = 1.0f;
            y = 0.0f;
            z = 0.0f;
        } else {
            // 使用归一化的虚部作为轴
            x = q.x / norm;
            y = q.y / norm;
            z = q.z / norm;
        }
        angle_rad = 0.0f; // 极小角度视为零
    } else {
        x = q.x / s;
        y = q.y / s;
        z = q.z / s;
        
        // 标准化轴向量
        float axis_norm = sqrtf(x*x + y*y + z*z);
        if (axis_norm > FLOAT_EPSILON) {
            x /= axis_norm;
            y /= axis_norm;
            z /= axis_norm;
        }
    }
}

// 四元数差值 (球面线性插值)
Quaternion Quaternion::slerp(const Quaternion& q1, const Quaternion& q2, float t) {
    if (t <= 0.0f) return q1;
    if (t >= 1.0f) return q2;

    Quaternion q2_tmp = q2;
    
    // 计算四元数之间的点积
    float cos_theta = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
    
    // 如果点积为负，反转其中一个四元数，确保最短路径插值
    if (cos_theta < 0.0f) {
        q2_tmp = q2 * -1.0f;
        cos_theta = -cos_theta;
    }
    
    // 如果两个四元数几乎平行，使用线性插值
    if (cos_theta > 0.9995f) {
        Quaternion result = q1 * (1.0f - t) + q2_tmp * t;
        result.normalize();
        return result;
    }
    
    // 计算球面线性插值的参数
    float theta = arm_cos_f32(cos_theta);
    float sin_theta = arm_sin_f32(theta);
    float s1 = arm_sin_f32((1.0f - t) * theta) / sin_theta;
    float s2 = arm_sin_f32(t * theta) / sin_theta;
    
    // 执行球面线性插值
    return Quaternion(
        s1 * q1.w + s2 * q2_tmp.w,
        s1 * q1.x + s2 * q2_tmp.x,
        s1 * q1.y + s2 * q2_tmp.y,
        s1 * q1.z + s2 * q2_tmp.z
    );
}

// 计算两个四元数之间的角度（弧度）
float Quaternion::angleBetween(const Quaternion& q1, const Quaternion& q2) {
    // 确保四元数是单位四元数
    Quaternion q1n = q1.normalized();
    Quaternion q2n = q2.normalized();
    
    // 计算两个四元数的点积
    float dot = q1n.w * q2n.w + q1n.x * q2n.x + q1n.y * q2n.y + q1n.z * q2n.z;
    
    // 处理数值误差，确保dot在[-1, 1]范围内
    dot = (dot < -1.0f) ? -1.0f : ((dot > 1.0f) ? 1.0f : dot);
    
    // 计算四元数之间的角度 (q1与q2表示的旋转之间的最短角度)
    return 2.0f * acosf(fabsf(dot));
}

// 设置为单位四元数
void Quaternion::setIdentity() {
    w = 1.0f;
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
}

// 检查是否为单位四元数
bool Quaternion::isIdentity() const {
    return fabsf(w - 1.0f) < FLOAT_EPSILON && 
           fabsf(x) < FLOAT_EPSILON && 
           fabsf(y) < FLOAT_EPSILON && 
           fabsf(z) < FLOAT_EPSILON;
}

// 应用四元数旋转到向量
void Quaternion::rotateVector(const float v_in[3], float v_out[3]) const {
    // 计算 v' = q * v * q^-1
    // 对于单位四元数，q^-1 = q* (共轭)
    // 使用等效的旋转矩阵 R 进行计算: v_out = R * v_in
    // R = [ 1 - 2(yy + zz)   2(xy - wz)     2(xz + wy)  ]
    //     [ 2(xy + wz)     1 - 2(xx + zz)   2(yz - wx)  ]
    //     [ 2(xz - wy)     2(yz + wx)     1 - 2(xx + yy) ]
    
    // 确保四元数是单位四元数，如果不是，旋转结果可能不正确
    // 在实际应用中，通常会保持四元数归一化
    // Quaternion qn = this->normalized(); // 如果不确定是否归一化，可以取消注释这行
    // float w = qn.w, x = qn.x, y = qn.y, z = qn.z;
    
    float xx = x * x;
    float yy = y * y;
    float zz = z * z;
    float xy = x * y;
    float xz = x * z;
    float yz = y * z;
    float wx = w * x;
    float wy = w * y;
    float wz = w * z;

    // 计算旋转矩阵 R 的元素
    float R00 = 1.0f - 2.0f * (yy + zz);
    float R01 = 2.0f * (xy - wz);
    float R02 = 2.0f * (xz + wy);
    
    float R10 = 2.0f * (xy + wz);
    float R11 = 1.0f - 2.0f * (xx + zz);
    float R12 = 2.0f * (yz - wx);
    
    float R20 = 2.0f * (xz - wy);
    float R21 = 2.0f * (yz + wx);
    float R22 = 1.0f - 2.0f * (xx + yy);

    // 执行矩阵向量乘法 v_out = R * v_in
    v_out[0] = R00 * v_in[0] + R01 * v_in[1] + R02 * v_in[2];
    v_out[1] = R10 * v_in[0] + R11 * v_in[1] + R12 * v_in[2];
    v_out[2] = R20 * v_in[0] + R21 * v_in[1] + R22 * v_in[2];
}

} // namespace math
} // namespace utils