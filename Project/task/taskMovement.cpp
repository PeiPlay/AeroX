#include "taskMovement.h"
#include "taskCommuCheck.h"
#include "config.h"
#include "cmsis_os.h"
#include "slope_smoother.h"


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
        
        // 20ms延迟，实现50Hz更新频率
        osDelay(20);
    }
}
}

