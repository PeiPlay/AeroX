#ifndef MATH_QUATERNION_H
#define MATH_QUATERNION_H

#include "math_utils.h"
#include "math_const.h"
#ifdef __cplusplus

namespace MathUtils {

class Quaternion {
public:
    float w; // 实部
    float x; // i分量
    float y; // j分量
    float z; // k分量

    // 构造函数
    Quaternion();
    Quaternion(float w, float x, float y, float z);
    
    // 从欧拉角创建四元数 (弧度单位)
    static Quaternion fromEulerRad(float roll, float pitch, float yaw);
    
    // 从欧拉角创建四元数 (角度单位)
    static Quaternion fromEulerDeg(float roll, float pitch, float yaw);
    
    // 从旋转矩阵创建四元数
    static Quaternion fromRotationMatrix(const float matrix[9]);
    
    // 从轴角表示法创建四元数
    static Quaternion fromAxisAngle(float x, float y, float z, float angle_rad);
    
    // 基本操作
    float norm() const;     // 四元数模长
    void normalize();       // 归一化 
    Quaternion normalized() const; // 返回归一化后的四元数，但不修改原四元数
    Quaternion conjugate() const;  // 共轭四元数
    Quaternion inverse() const;    // 逆四元数
    
    // 运算符重载
    Quaternion operator+(const Quaternion& q) const;
    Quaternion operator-(const Quaternion& q) const;
    Quaternion operator*(const Quaternion& q) const;
    Quaternion operator*(float scalar) const;
    friend Quaternion operator*(float scalar, const Quaternion& q);
    bool operator==(const Quaternion& q) const;
    bool operator!=(const Quaternion& q) const;
    
    // 四元数到欧拉角的转换
    void toEulerRad(float& roll, float& pitch, float& yaw) const;
    void toEulerDeg(float& roll, float& pitch, float& yaw) const;
    
    // 四元数到旋转矩阵的转换
    void toRotationMatrix(float matrix[9]) const;
    
    // 四元数到轴角的转换
    void toAxisAngle(float& x, float& y, float& z, float& angle_rad) const;
    
    // 四元数差值
    static Quaternion slerp(const Quaternion& q1, const Quaternion& q2, float t);
    
    // 计算两个四元数之间的角度（弧度）
    static float angleBetween(const Quaternion& q1, const Quaternion& q2);
    
    // 设置为单位四元数
    void setIdentity();
    
    // 检查是否为单位四元数
    bool isIdentity() const;
    
    // 应用四元数旋转到向量
    void rotateVector(const float v_in[3], float v_out[3]) const;
};

} // namespace MathUtils

#endif // __cplusplus

#endif // MATH_QUATERNION_H