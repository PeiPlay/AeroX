#include "taskStabilize.h"
#include "config.h"
#include "slope_smoother.h"
#include "utils.h"

float motor_all_th = 0.0;
SlopeSmoother motor_smoother(0.08f, 100.0f, motor_all_th);

float gs_target_th = 0.0f;
float gs_target_yaw = 0.0f;

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
    static bool gs_is_connected_prev = false;
    osDelay(1000);
    taskStabilize_Init();
    osDelay(1000);
    chassis.setThrottleMode(ThrottleMode::DIRECT);
    while (1)
    {
        if(gs_is_connected != gs_is_connected_prev)
        {
            //motor_smoother.setCurrentValue(chassis.getThrottleOverride());
            gs_is_connected_prev = gs_is_connected;
        }

        if(gs_is_connected_prev)
        {
            float temp = ((gs_rockers.left_y > 0.0f) ? (gs_rockers.left_y / 3.0f) : 0.0f) + 50.0f;
            gs_target_th = temp > 100.0f ? 100.0f : temp;

            temp = math_normalize_radian_pi((-0.00002f * gs_rockers.left_x) + chassis.getTargetYaw());
            //
            if(gs_switch(0))
            {
                chassis.setTargetAttitude(
					0.01745f*0.4 + ((-gs_rockers.right_x) / 128.0f) * 0.01745f * 7, 
					0.01745f*1.94 + ((-gs_rockers.right_y) / 128.0f) * 0.01745f * 7, temp);
                chassis.setThrottleOverride(motor_smoother.update(gs_target_th));
            }
            else
            {
                chassis.setThrottleOverride(motor_smoother.update(0.0f));
            }
        }
        else
        {
            chassis.setThrottleOverride(motor_smoother.update(30.0f));
        }

        chassis.update();

        // if(motor_all_th > 100.0f) motor_all_th = 100.0f;
        // if(motor_all_th < 0.0f) motor_all_th = 0.0f;
        // chassis.setThrottleOverride(motor_smoother.update(motor_all_th));

        //motor_1.setThrottle(motor_1_smoother.update(motor_all_th));
        //motor_2.setThrottle(motor_2_smoother.update(motor_all_th));
        //motor_3.setThrottle(motor_3_smoother.update(motor_all_th));
        //motor_4.setThrottle(motor_4_smoother.update(motor_all_th));
        // 读取遥控器数据
        //chassis.update();
        osDelay(2);
    }

}
