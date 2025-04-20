#ifndef __TEST_H__
#define __TEST_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void test_usb_cdc_sendmsg(const char *msg, uint32_t len);
void test_printf(const char *format, ...);
void test_cpp(void);
	
#ifdef __cplusplus
}
#endif

#endif // __TEST_H__