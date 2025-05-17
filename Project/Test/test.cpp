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

// 如果 M_PI 未定义
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 外部声明 SPI 句柄 (假设在 main.c 中定义)
extern SPI_HandleTypeDef hspi2;

// 全局变量存储欧拉角 (单位: 度)
float g_roll_deg = 0.0f;
float g_pitch_deg = 0.0f;
float g_yaw_deg = 0.0f;

// BMI088 配置
BMI088Config_t bmi088_config = {
    .hspi = &hspi2,
    .ce_acc = {.port = GPIOC, .pin = GPIO_PIN_0},
    .ce_gyro = {.port = GPIOC, .pin = GPIO_PIN_3},
    .gyroRange = BMI088_GYRO_2000,
    .accelRange = BMI088_ACC_RANGE_3G
};

// 实例化 IMU 和姿态估计器
BMI088 bmi088_sensor(bmi088_config);
MahonyAHRS mahony_estimator(500.0f, 0.55f, 0.002f); // 使用默认参数
// 实例化姿态管理器
AttitudeManager attitude_manager(&bmi088_sensor, &mahony_estimator);


void test_imu_task(void) {
    float gyroBuf[3] = {0};
    float accelBuf[3] = {0};

    bmi088_sensor.init(); // 初始化传感器 (如果需要)
    osDelay(2);
    bmi088_sensor.read(gyroBuf, accelBuf); // 读取传感器数据 (如果需要)
    mahony_estimator.init(accelBuf); // 初始化姿态估计器 (如果需要)
    osDelay(2);

    // 初始化姿态管理器 (包含 IMU 初始化和校准)
    if (!attitude_manager.init()) {
        // 初始化失败处理 (例如，进入错误状态或死循环)
        // test_printf("Attitude Manager Init Failed!\n"); // 避免在循环中使用打印
        while(1) {
            osDelay(1000);
        }
    }

    // 设置姿态估计器的采样周期 (假设系统运行在 500Hz)
    // 注意：MahonyAHRS 默认构造函数已设置 500Hz，这里可以省略
    // mahony_estimator.setSamplePeriod(1.0f / 500.0f);

    // 无限循环更新姿态
    for (;;) {
        // 更新姿态管理器 (读取 IMU 数据并进行姿态解算)
        attitude_manager.update();

        // 获取欧拉角 (弧度)
        float roll_rad, pitch_rad, yaw_rad;
        attitude_manager.getAttitude(roll_rad, pitch_rad, yaw_rad);

        // 将弧度转换为度数并存储到全局变量
        g_roll_deg = roll_rad * (180.0f / M_PI);
        g_pitch_deg = pitch_rad * (180.0f / M_PI);
        g_yaw_deg = yaw_rad * (180.0f / M_PI);

        // 限制执行频率 (例如，目标 500Hz，osDelay(2) 对应 2ms 延迟)
        // 实际频率取决于 osDelay 的精度和 update() 的执行时间
        osDelay(2);
    }
}



extern "C" {
    void test_cpp_task(void) {

        // 测试函数的实现
        test_imu_task(); // 调用测试函数
    
    }
    void test_cpp_debug(void) {

        // // 测试调试函数的实现
        // // 这里可以添加调试代码，例如打印变量值等
        // test_printf("Debugging...\n");

        // //持续打印欧拉角
        // while (1) {
        //     vofa_print(g_roll_deg, g_pitch_deg, g_yaw_deg);
        // }
    
    }
} // extern "C"