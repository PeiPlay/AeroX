#include "taskStabilize.h"
#include "config.h"
#include "slope_smoother.h"

float motor_all_th = 0.0;
SlopeSmoother motor_smoother(0.04f, 100.0f, motor_all_th);

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
    osDelay(1000);
    chassis.setThrottleMode(ThrottleMode::DIRECT);
    while (1)
    {
        if(motor_all_th > 100.0f) motor_all_th = 100.0f;
        if(motor_all_th < 0.0f) motor_all_th = 0.0f;
        chassis.setThrottleOverride(motor_smoother.update(motor_all_th));

        //motor_1.setThrottle(motor_1_smoother.update(motor_all_th));
        //motor_2.setThrottle(motor_2_smoother.update(motor_all_th));
        //motor_3.setThrottle(motor_3_smoother.update(motor_all_th));
        //motor_4.setThrottle(motor_4_smoother.update(motor_all_th));
        // 读取遥控器数据
        chassis.update();
        osDelay(2);
    }

}
