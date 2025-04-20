#ifndef __MATH_CONST_H__
#define __MATH_CONST_H__

#include <math.h>

// 数学常量定义
#ifndef M_E
#define M_E          2.7182818284590452354f  // e
#endif

#ifndef M_LOG2E
#define M_LOG2E      1.4426950408889634074f  // log_2(e)
#endif

#ifndef M_LOG10E
#define M_LOG10E     0.43429448190325182765f // log_10(e)
#endif

#ifndef M_LN2
#define M_LN2        0.69314718055994530942f // log_e(2)
#endif

#ifndef M_LN10
#define M_LN10       2.30258509299404568402f // log_e(10)
#endif

#ifndef M_PI
#define M_PI         3.14159265358979323846f // pi
#endif

#ifndef M_PI_2
#define M_PI_2       1.57079632679489661923f // pi/2
#endif

#ifndef M_PI_4
#define M_PI_4       0.78539816339744830962f // pi/4
#endif

#ifndef M_1_PI
#define M_1_PI       0.31830988618379067154f // 1/pi
#endif

#ifndef M_2_PI
#define M_2_PI       0.63661977236758134308f // 2/pi
#endif

#ifndef M_2_SQRTPI
#define M_2_SQRTPI   1.12837916709551257390f // 2/sqrt(pi)
#endif

#ifndef M_SQRT2
#define M_SQRT2      1.41421356237309504880f // sqrt(2)
#endif

#ifndef M_SQRT1_2
#define M_SQRT1_2    0.70710678118654752440f // 1/sqrt(2)
#endif

// 角度与弧度转换
#define DEG_TO_RAD   (M_PI / 180.0f)
#define RAD_TO_DEG   (180.0f / M_PI)

// 重力常数
#define GRAVITY_CONST 9.80665f  // 标准重力加速度 (m/s^2)

// 浮点精度
#define FLOAT_EPSILON 1.192092896e-07f  // FLT_EPSILON

// 浮点数近似相等比较
#define IS_FLOAT_EQUAL(a, b) (fabsf((a) - (b)) <= FLOAT_EPSILON)

#endif // __MATH_CONST_H__