#ifndef __MATH_BITS_H__
#define __MATH_BITS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// 计算一个整数中二进制位1的个数
uint32_t math_bits_count(uint32_t n);

// 计算一个32位整数中前导零的个数
uint32_t math_clz(uint32_t n);

// 计算一个32位整数中尾部零的个数
uint32_t math_ctz(uint32_t n);

// 计算一个整数取log2的结果（向下取整）
uint32_t math_log2_floor(uint32_t n);

// 计算一个整数取log2的结果（向上取整）
uint32_t math_log2_ceil(uint32_t n);

// 判断一个整数是否为2的幂
static inline int math_is_power_of_2(uint32_t n) {
    return n > 0 && (n & (n - 1)) == 0;
}

// 向上取整到下一个2的幂
uint32_t math_next_pow2(uint32_t n);

// 计算两个整数的最大公约数
uint32_t math_gcd(uint32_t a, uint32_t b);

// 计算两个整数的最小公倍数
uint32_t math_lcm(uint32_t a, uint32_t b);

// 将浮点数转换为定点数表示（Q格式）
int32_t math_float_to_fixed(float value, uint8_t frac_bits);

// 将定点数表示转换为浮点数（Q格式）
float math_fixed_to_float(int32_t value, uint8_t frac_bits);

// 交换两个值
#define MATH_SWAP(type, a, b) do { type temp = a; a = b; b = temp; } while(0)

#ifdef __cplusplus
}
#endif

#endif // __MATH_BITS_H__