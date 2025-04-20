#ifndef __MATH_COMMON_H__
#define __MATH_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "math_const.h"
#include "arm_math.h"

// 基本数学函数
static inline float math_abs(float x) {
    float result;
    arm_abs_f32(&x, &result, 1);
    return result;
}

static inline float math_max(float a, float b) {
    return (a > b) ? a : b;
}

static inline float math_min(float a, float b) {
    return (a < b) ? a : b;
}

static inline float math_clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static inline int math_sign(float x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

static inline float math_square(float x) {
    return x * x;
}

// 平均值计算
float math_mean(const float *data, uint32_t length);

// 标准差计算
float math_std_dev(const float *data, uint32_t length);

// 向量点积
float math_dot_product(const float *a, const float *b, uint32_t length);

// 线性插值
float math_lerp(float a, float b, float t);

// 向量归一化
void math_normalize(float *vec, uint32_t length);

// 阶乘
uint32_t math_factorial(uint32_t n);

// 指数平滑滤波
typedef struct {
    float alpha;
    float last_value;
    uint8_t initialized;
} math_exp_filter_t;

void math_exp_filter_init(math_exp_filter_t *filter, float alpha, float initial_value);
float math_exp_filter_update(math_exp_filter_t *filter, float new_value);

#ifdef __cplusplus
}
#endif

#endif // __MATH_COMMON_H__