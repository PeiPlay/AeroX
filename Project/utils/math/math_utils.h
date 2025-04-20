#ifndef __MATH_UTILS_H__
#define __MATH_UTILS_H__

#include <cmsis_os.h>
#include "arm_math.h"

// 内存分配宏定义
#define __math_utils_malloc(size) pvPortMalloc(size)
#define __math_utils_free(ptr) vPortFree(ptr)

// 引入所有子模块
#include "math_const.h"
#include "math_common.h"
#include "math_angle.h"
#include "math_bits.h"
#include "math_matrix.h"
#include "math_quaternion.h"
#include "math_sort.h"

#endif // __MATH_UTILS_H__