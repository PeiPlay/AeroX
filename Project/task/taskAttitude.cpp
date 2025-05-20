#include "taskManager.h"
#include "config.h"
#include "cmsis_os.h"
static float roll_deg, pitch_deg, yaw_deg;
static float gyroBuf[3] = {0};
static float accelBuf[3] = {0};

// 添加extern "C"声明
extern "C" void taskAttitudeIMU(void *argument);

void AttitudeIMU_Init(void)
{
   // osDelay(1000);

    bmi088.init();
    osDelay(2);
    bmi088.read(gyroBuf, accelBuf);
    mahony_estimator.init(accelBuf);

    if (!attitude_manager.init())
    {
        // 初始化失败处理 (例如，进入错误状态或死循环)
        while(1)
        {
            osDelay(1000);
        }
    }

}

void AttitudeIMU_Update(void)
{
    attitude_manager.update();
}

void AttitudeIMU_Debug(void)
{
    // 将信息更新到全局变量
    float roll, pitch, yaw;
    attitude_manager.getAttitude(roll, pitch, yaw);
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
    roll_deg = roll * 180.0f / M_PI;
    pitch_deg = pitch * 180.0f / M_PI;
    yaw_deg = yaw * 180.0f / M_PI;
    attitude_manager.getGyro(gyroBuf);
    attitude_manager.getAccel(accelBuf);
}

void taskAttitudeIMU(void *argument)
{
    // 初始化IMU
    AttitudeIMU_Init();

    while (1)
    {

        //记录当前os时间以精确按照2ms为周期触发
        //uint32_t current_time = xTaskGetTickCount();
        // 更新IMU数据
        AttitudeIMU_Update();
        // 调试输出
        AttitudeIMU_Debug();

        // 计算下次触发时间
        //uint32_t next_time = current_time + 2; // 2ms周期

        // 线程释放直到下次触发
        //osDelayUntil(next_time);
        osDelay(2); // 2ms周期
    }
}