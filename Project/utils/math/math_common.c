#include "math_common.h"
#include <math.h>

// 平均值计算
float math_mean(const float *data, uint32_t length) {
    if (!data || length == 0) {
        return 0.0f;
    }

    float sum = 0.0f;
    for (uint32_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum / length;
}

// 标准差计算
float math_std_dev(const float *data, uint32_t length) {
    if (!data || length <= 1) {
        return 0.0f;
    }

    float mean = math_mean(data, length);
    float variance = 0.0f;
    
    for (uint32_t i = 0; i < length; i++) {
        float diff = data[i] - mean;
        variance += diff * diff;
    }
    
    variance /= (length - 1); // 使用无偏估计
    return sqrtf(variance);
}

// 向量点积
float math_dot_product(const float *a, const float *b, uint32_t length) {
    if (!a || !b || length == 0) {
        return 0.0f;
    }
    
    float result;
    arm_dot_prod_f32(a, b, length, &result);
    return result;
}

// 线性插值
float math_lerp(float a, float b, float t) {
    t = math_clamp(t, 0.0f, 1.0f);
    return a + t * (b - a);
}

// 向量归一化
void math_normalize(float *vec, uint32_t length) {
    if (!vec || length == 0) {
        return;
    }
    
    float magnitude = 0.0f;
    
    // 计算向量长度
    for (uint32_t i = 0; i < length; i++) {
        magnitude += vec[i] * vec[i];
    }
    
    magnitude = sqrtf(magnitude);
    
    // 避免除以零
    if (magnitude < FLOAT_EPSILON) {
        return;
    }
    
    // 归一化
    float scale = 1.0f / magnitude;
    for (uint32_t i = 0; i < length; i++) {
        vec[i] *= scale;
    }
}

// 阶乘
uint32_t math_factorial(uint32_t n) {
    if (n <= 1) {
        return 1;
    }
    
    uint32_t result = 1;
    for (uint32_t i = 2; i <= n; i++) {
        result *= i;
    }
    return result;
}

// 初始化指数平滑滤波器
void math_exp_filter_init(math_exp_filter_t *filter, float alpha, float initial_value) {
    if (!filter) {
        return;
    }
    
    filter->alpha = math_clamp(alpha, 0.0f, 1.0f);
    filter->last_value = initial_value;
    filter->initialized = 1;
}

// 更新指数平滑滤波器
float math_exp_filter_update(math_exp_filter_t *filter, float new_value) {
    if (!filter) {
        return new_value;
    }
    
    if (!filter->initialized) {
        filter->last_value = new_value;
        filter->initialized = 1;
        return new_value;
    }
    
    float result = filter->alpha * new_value + (1.0f - filter->alpha) * filter->last_value;
    filter->last_value = result;
    return result;
}