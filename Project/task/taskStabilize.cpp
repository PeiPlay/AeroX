#include "taskStabilize.h"
#include "config.h"
#include "slope_smoother.h"

float motor_1_th = 0.0;
float motor_2_th = 0.0;
float motor_3_th = 0.0;
float motor_4_th = 0.0;
float motor_all_th = 0.0;
SlopeSmoother motor_1_smoother(0.04f, 100.0f, motor_all_th);
SlopeSmoother motor_2_smoother(0.04f, 100.0f, motor_all_th);
SlopeSmoother motor_3_smoother(0.04f, 100.0f, motor_all_th);
SlopeSmoother motor_4_smoother(0.04f, 100.0f, motor_all_th);

void taskStabilize_Init_Motor(void)
{
    motor_1.init();
    motor_2.init();
    motor_3.init();
    motor_4.init();
    osDelay(200);
}

void taskStabilize_Init(void)
{
    taskStabilize_Init_Motor();
    chassis.init();

}


void taskStabilize(void *argument)
{
    osDelay(1000);
    taskStabilize_Init();
    while (1)
    {
        motor_1.setThrottle(motor_1_smoother.update(motor_all_th));
        motor_2.setThrottle(motor_2_smoother.update(motor_all_th));
        motor_3.setThrottle(motor_3_smoother.update(motor_all_th));
        motor_4.setThrottle(motor_4_smoother.update(motor_all_th));
        // 读取遥控器数据
        //chassis.update();
        osDelay(2);
    }

}
