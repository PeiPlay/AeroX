#include "taskMovement.h"
#include "taskCommuCheck.h"
#include "config.h"
#include "cmsis_os.h"
#include "slope_smoother.h"

PoseDiff pose_diff; // 用于存储位姿差值
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

/*
extern Point point_begin;
extern Point point_begin2;
extern Point point_stable1;
extern Point point_task3;
extern Point point_task4;
extern Point point_task5;
extern Point point_end1;
extern Point point_stable2;

extern Point point_task5_2;
extern Point point_stable3;

extern Point point_task5_3;
extern Point point_stable4;

extern Point point_task5_4;


extern Point point_end2;
extern Point point_stop;
*/

static bool path_begin = false; // 路径是否已开始

void taskMovement(void *argument)
{
    path1.addTargetPoint(&point_begin);
    path1.addTargetPoint(&point_begin2);
    path1.addTargetPoint(&point_stable1);
    path1.addTargetPoint(&point_task3);
    path1.addTargetPoint(&point_task4);
    path1.addTargetPoint(&point_task5);
    path1.addTargetPoint(&point_end1);
    path1.addTargetPoint(&point_stable2);
    path1.addTargetPoint(&point_task5_2);
    path1.addTargetPoint(&point_stable3);
    path1.addTargetPoint(&point_task5_3);
    path1.addTargetPoint(&point_stable4);
    path1.addTargetPoint(&point_task5_4);
    path1.addTargetPoint(&point_end2);    
    path1.addTargetPoint(&point_stop);

    path1.startPath();
    // 等待系统稳定
    osDelay(1500);
    
    // 初始化运动控制
    taskMovement_Init();

    // 等待初始化完成
    osDelay(500);
    
    // 主循环 - 50Hz更新频率
    while (1)
    {
        if(path_begin == false)
        {
            if(GS_SWITCH(0) && GS_SWITCH(1))
            {
                path_begin = true;
            }
            osDelay(20);
            continue;
        }

        Pose current_pose;
        move.getCurrentPosition(current_pose.x, current_pose.y, current_pose.z);
        current_pose.yaw = chassis.getCurrentYaw();
        // 更新路径状态
        path1.isReached(current_pose, &pose_diff);
        // 20ms延迟，实现50Hz更新频率
        osDelay(20);
    }
}
}

