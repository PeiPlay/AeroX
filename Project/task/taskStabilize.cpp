#include "taskStabilize.h"
#include "taskCommuCheck.h"  // 替换为新的通信检查头文件
#include "config.h"
#include "slope_smoother.h"
#include "utils.h"

float motor_all_th = 0.0;
SlopeSmoother motor_smoother(0.04f, 100.0f, motor_all_th);

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
        if(GS_IS_CONNECTED != gs_is_connected_prev)
        {
            // 连接状态发生变化
            gs_is_connected_prev = GS_IS_CONNECTED;
        }
        
        if(GS_IS_CONNECTED)
        {
            // 遥控器正常链接

            float temp = ((GS_ROCKERS.left_y > 0.0f) ? (GS_ROCKERS.left_y / 3.0f) : 0.0f) + 30.0f;
            gs_target_th = temp > 100.0f ? 100.0f : temp;

            temp = math_normalize_radian_pi((-0.00002f * GS_ROCKERS.left_x) + chassis.getTargetYaw());
            
            if(GS_SWITCH(0) && !GS_SWITCH(1))
            {
                // 操作员希望使用遥控器控制
                // 设置目标姿态和油门
                chassis.setTargetAttitude(
					0.01745f*0.4*0.0f + ((-GS_ROCKERS.right_x) / 128.0f) * 0.01745f * 7, 
					0.01745f*1.94*0.0f + ((-GS_ROCKERS.right_y) / 128.0f) * 0.01745f * 7, temp);
                chassis.setThrottleOverride(motor_smoother.update(gs_target_th));
            }
            else if(!GS_SWITCH(0))
            {
                // 操作员希望飞机停止
                // 设置油门为0
                chassis.setThrottleOverride(motor_smoother.update(0.0f));
                chassis.setTargetAttitude(0.0f, 0.0f, chassis.getCurrentYaw());
            }
            else if(GS_SWITCH(1) && !LIDAR_IS_CONNECTED)
            {
                // 操作员希望使用雷达控制，但雷达掉线
                // 设置为缓降油门
                chassis.setThrottleOverride(motor_smoother.update(35.0f));
                chassis.setTargetAttitude(0.0f, 0.0f, chassis.getCurrentYaw());
            }
            // 操作员希望使用自动控制，且雷达正常链接
            // 直接跳转到update函数即可
        }
        else
        {
            // 遥控器未连接或掉线
            // 设置为缓降油门
            chassis.setThrottleOverride(motor_smoother.update(35.0f));
            chassis.setTargetAttitude(0.0f, 0.0f, chassis.getCurrentYaw());
        }

        chassis.update();
		osDelay(2);
	}

//	while(1){
//         if(motor_all_th > 100.0f) motor_all_th = 100.0f;
//         if(motor_all_th < 0.0f) motor_all_th = 0.0f;
//         //chassis.setThrottleOverride(motor_smoother.update(motor_all_th));

//        motor_1.setThrottle(motor_smoother.update(motor_all_th));
//        motor_2.setThrottle(motor_smoother.update(motor_all_th));
//        motor_3.setThrottle(motor_smoother.update(motor_all_th));
//        motor_4.setThrottle(motor_smoother.update(motor_all_th));
//        // 读取遥控器数据
//        //chassis.update();
//        osDelay(2);
//    }

}
