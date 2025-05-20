#ifndef __TEST_H__
#define __TEST_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "main.h"
// 将 vofa.h 移到 extern "C" 内部处理

#ifdef __cplusplus
extern "C" {
#endif

// void test_printf(const char *format, ...);
// void test_usb_cdc_sendmsg(const char *msg, uint32_t len); // 确保在 extern "C" 内声明
void test_cpp_task(void);
void test_cpp_debug(void);

#ifdef __cplusplus
}
#endif

#include "vofa.h" // 移动到后面，以避免循环依赖问题

#endif // __TEST_H__