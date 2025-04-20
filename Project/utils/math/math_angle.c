#include "math_angle.h"
#include <math.h>

// 角度归一化到 [0, 2π)
float math_normalize_radian_0_2pi(float angle) {
    angle = fmodf(angle, 2.0f * M_PI);
    if (angle < 0.0f) {
        angle += 2.0f * M_PI;
    }
    return angle;
}

// 角度归一化到 [-π, π)
float math_normalize_radian_pi(float angle) {
    angle = fmodf(angle + M_PI, 2.0f * M_PI);
    if (angle < 0.0f) {
        angle += 2.0f * M_PI;
    }
    return angle - M_PI;
}

// 角度归一化到 [0, 360)
float math_normalize_angle_0_360(float angle) {
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f) {
        angle += 360.0f;
    }
    return angle;
}

// 角度归一化到 [-180, 180)
float math_normalize_angle_180(float angle) {
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0.0f) {
        angle += 360.0f;
    }
    return angle - 180.0f;
}

// 计算两个角度之间的最小差值 (弧度制)，返回值范围 [-π, π)
float math_angle_diff_rad(float a, float b) {
    float diff = fmodf(b - a + M_PI, 2.0f * M_PI);
    if (diff < 0.0f) {
        diff += 2.0f * M_PI;
    }
    return diff - M_PI;
}

// 计算两个角度之间的最小差值 (角度制)，返回值范围 [-180, 180)
float math_angle_diff_deg(float a, float b) {
    float diff = fmodf(b - a + 180.0f, 360.0f);
    if (diff < 0.0f) {
        diff += 360.0f;
    }
    return diff - 180.0f;
}

// 使用atan2计算角度
float math_atan2(float y, float x) {
    if (fabsf(x) < FLOAT_EPSILON) {
        if (y > 0.0f) {
            return M_PI_2;
        } else if (y < 0.0f) {
            return -M_PI_2;
        } else {
            return 0.0f;  // 不确定的情况
        }
    }
    return atan2f(y, x);
}

// 角度插值 (在最短路径上)
float math_angle_lerp(float a, float b, float t) {
    if (t <= 0.0f) return a;
    if (t >= 1.0f) return b;
    
    // 特殊情况处理：π/2 到 -π/2 的插值，走经过π的路径
    if (fabsf(a) == M_PI_2 && fabsf(b) == M_PI_2 && a * b < 0.0f) {
        if (t < 0.5f) {
            // 从起点移动到π
            return a + (a > 0 ? 1.0f : -1.0f) * (M_PI - M_PI_2) * (2.0f * t);
        } else {
            // 从π移动到终点
            return M_PI + (b - M_PI) * (2.0f * (t - 0.5f));
        }
    }
    
    // 标准情况处理
    float diff = math_angle_diff_rad(a, b);
    
    // 特殊情况：0到π之间的插值，确保选择正向路径
    if ((a == 0.0f && b == M_PI) || (a == M_PI && b == 0.0f)) {
        diff = (a < b) ? M_PI : -M_PI;
    }
    
    // 使用线性插值
    return a + diff * t;
}