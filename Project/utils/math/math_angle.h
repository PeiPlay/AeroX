#ifndef __MATH_ANGLE_H__
#define __MATH_ANGLE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "math_const.h"
#include "arm_math.h"

// 角度归一化到 [0, 2π)
float math_normalize_radian_0_2pi(float angle);

// 角度归一化到 [-π, π)
float math_normalize_radian_pi(float angle);

// 角度归一化到 [0, 360)
float math_normalize_angle_0_360(float angle);

// 角度归一化到 [-180, 180)
float math_normalize_angle_180(float angle);

// 计算两个角度之间的最小差值 (弧度制)，返回值范围 [-π, π)
float math_angle_diff_rad(float a, float b);

// 计算两个角度之间的最小差值 (角度制)，返回值范围 [-180, 180)
float math_angle_diff_deg(float a, float b);

// 使用CORDIC算法计算正弦，利用arm_sin_f32
static inline float math_sin(float angle) {
    return arm_sin_f32(angle);
}

// 使用CORDIC算法计算余弦，利用arm_cos_f32
static inline float math_cos(float angle) {
    return arm_cos_f32(angle);
}

// 使用atan2计算角度
float math_atan2(float y, float x);

// 角度插值
float math_angle_lerp(float a, float b, float t);

#ifdef __cplusplus
}
#endif

#endif // __MATH_ANGLE_H__