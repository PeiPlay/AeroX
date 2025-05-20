#include "test.h"
#include "utils.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "main.h" 
#include <functional>
#include <new>      
#include "pid.h"
#include "Attitude.h" 
#include "IMU/BMI088.h"
#include "MahonyAHRS.h"
#include <cmath>
#include "vofa.h"
#include "up_t201.h" 
#include "spl06.h"   
#include "tim.h"   
#include "sdc_dual.h" 
#include "watchdog.h" // Include watchdog header
#include "slope_smoother.h" // 引入斜坡平滑器头文件





extern "C" {
    void test_cpp_task(void) {

    
    }
    void test_cpp_debug(void) {

    }
} // extern "C"