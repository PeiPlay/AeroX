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
    
    // 计算最短路径差值
    float diff = math_angle_diff_rad(a, b);
    
    // 使用线性插值
    return a + diff * t;
}