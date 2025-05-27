#include "taskMovement.h"
#include "taskCommuCheck.h"
#include "config.h"
#include "cmsis_os.h"
#include "slope_smoother.h"

float target_x = 0.0f; // 目标X位置 (m)
float target_y = 0.0f; // 目标Y位置 (m)
float target_z = 0.0f; // 目标Z位置 (m)

// ========== 任务初始化函数 ==========
extern "C" {
void taskMovement_Init(void)
{
    
    // 初始化Move控制器
    if (!move.init()) {
        // 初始化失败，进入错误处理
        return;
    }
    
    // 设置初始控制模式为位置控制
    move.setPositionControlMode(true);
    
    // 设置初始目标位置为当前位置
    float current_x, current_y, current_z;
    move.getCurrentPosition(current_x, current_y, current_z);
    move.setTargetPosition(current_x, current_y, current_z);
}

void taskMovement_Control(void)
{
    // 处理控制模式逻辑
    const float gain = 0.5f / (50.0f * 128.0f);
    
    
    float temp = ((GS_ROCKERS.left_y > 0.0f) ? GS_ROCKERS.left_y : 0.0f) * gain;
    temp += target_z;
    temp = std::clamp(temp, 0.0f, 1.5f); // 限制在0到1.5米之间
    target_z = temp;

    temp = GS_ROCKERS.right_x * gain;
    temp += target_x;
    temp = std::clamp(temp, -1.0f, 1.0f); // 限制在-1到1之间
    target_x = temp;

    temp = GS_ROCKERS.right_y * gain;
    temp += target_y;
    temp = std::clamp(temp, -1.0f, 1.0f); // 限制在-1到1之间
    target_y = temp;


    // 设置Move控制器的目标位置
    move.setTargetPosition(target_x, target_y, target_z);

}

void taskMovement(void *argument)
{
    // 等待系统稳定
    osDelay(1500);
    
    // 初始化运动控制
    taskMovement_Init();
    
    // 等待初始化完成
    osDelay(500);
    
    // 主循环 - 50Hz更新频率
    while (1)
    {
        if(GS_IS_CONNECTED && GS_SWITCH(0) && GS_SWITCH(1) && LIDAR_IS_CONNECTED) {
            taskMovement_Control();
            move.update();
        }
        else 
        {
            float current_x, current_y, current_z;
            move.getCurrentPosition(current_x, current_y, current_z);
            move.setTargetPosition(current_x, current_y, current_z);
        }

        // 20ms延迟，实现50Hz更新频率
        osDelay(20);
    }
}
}

