#include "math_bits.h"
#include <limits.h>

// 计算一个整数中二进制位1的个数
uint32_t math_bits_count(uint32_t n) {
    uint32_t count = 0;
    while (n) {
        count += n & 1;
        n >>= 1;
    }
    return count;
}

// 计算一个32位整数中前导零的个数
uint32_t math_clz(uint32_t n) {
    if (n == 0) {
        return 32;
    }
    
    uint32_t count = 0;
    uint32_t mask = 1U << 31;
    
    while ((n & mask) == 0) {
        count++;
        mask >>= 1;
    }
    
    return count;
}

// 计算一个32位整数中尾部零的个数
uint32_t math_ctz(uint32_t n) {
    if (n == 0) {
        return 32;
    }
    
    uint32_t count = 0;
    while ((n & 1) == 0) {
        count++;
        n >>= 1;
    }
    
    return count;
}

// 计算一个整数取log2的结果（向下取整）
uint32_t math_log2_floor(uint32_t n) {
    if (n == 0) {
        return 0;  // 在数学上log2(0)是未定义的，这里返回0以便处理
    }
    
    uint32_t log = 0;
    while (n > 1) {
        n >>= 1;
        log++;
    }
    
    return log;
}

// 计算一个整数取log2的结果（向上取整）
uint32_t math_log2_ceil(uint32_t n) {
    if (n <= 1) {
        return 0;
    }
    
    uint32_t log = math_log2_floor(n);
    if ((1U << log) < n) {
        log++;
    }
    
    return log;
}

// 向上取整到下一个2的幂
uint32_t math_next_pow2(uint32_t n) {
    if (n == 0) {
        return 1;
    }
    
    n--;
    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    n++;
    
    return n;
}

// 计算两个整数的最大公约数 (使用辗转相除法)
uint32_t math_gcd(uint32_t a, uint32_t b) {
    if (a == 0) {
        return b;
    }
    if (b == 0) {
        return a;
    }
    
    while (b != 0) {
        uint32_t temp = b;
        b = a % b;
        a = temp;
    }
    
    return a;
}

// 计算两个整数的最小公倍数
uint32_t math_lcm(uint32_t a, uint32_t b) {
    if (a == 0 || b == 0) {
        return 0;
    }
    
    return (a / math_gcd(a, b)) * b;
}

// 将浮点数转换为定点数表示（Q格式）
int32_t math_float_to_fixed(float value, uint8_t frac_bits) {
    return (int32_t)(value * (1 << frac_bits));
}

// 将定点数表示转换为浮点数（Q格式）
float math_fixed_to_float(int32_t value, uint8_t frac_bits) {
    return (float)value / (1 << frac_bits);
}