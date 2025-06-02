#include "taskStabilize.h"
#include "taskCommuCheck.h"  // 替换为新的通信检查头文件
#include "config.h"
#include "slope_smoother.h"
#include "utils.h"
#include "cmsis_os.h"

float motor_all_th = 0.0;
SlopeSmoother motor_smoother(0.04f, 100.0f, motor_all_th);
extern PoseDiff pose_diff;


void taskStabilize_Init_Motor(void)
{
    motor_1.init();
    motor_2.init();
    motor_3.init();
    motor_4.init();
    osDelay(300);
    motor_1.setThrottle(0.0f);
    motor_2.setThrottle(0.0f);
    motor_3.setThrottle(0.0f);
    motor_4.setThrottle(0.0f);
    osDelay(100);
}

void taskStabilize_Init(void)
{
    taskStabilize_Init_Motor();
    chassis.init();
    chassis.setThrottleMode(ThrottleMode::DIRECT);
}

float override_throttle = 50.0f; // 油门覆盖值
SlopeSmoother override_smoother(0.04f, 100.0f, 0.0f);

void taskStabilize_Manual(void)
{
//手动控制模式
    float temp = ((GS_ROCKERS.left_y > 0.0f) ? (GS_ROCKERS.left_y / 3.0f) : 0.0f) + 30.0f;
    float gs_target_th = temp > 100.0f ? 100.0f : temp;

    // 读取手柄输入，设置目标姿态和油门
    float roll = GS_ROCKERS.right_x / 128.0f * 0.01745f * 7; // 转换为弧度
    float pitch = GS_ROCKERS.right_y / 128.0f * 0.01745f * 7; // 转换为弧度
    float yaw = math_normalize_radian_pi((-0.00002f * GS_ROCKERS.left_x) + chassis.getTargetYaw());

    chassis.setTargetAttitude(roll, pitch, yaw);
    chassis.setThrottleOverride(motor_smoother.update(gs_target_th));
}

void taskStabilize_Stop(void)
{
    // 停机模式
    // 设置油门为0，保持当前姿态
    chassis.setThrottleOverride(motor_smoother.update(0.0f));
    chassis.setTargetAttitude(0.0f, 0.0f, chassis.getCurrentYaw());
}

void taskStabilize_Auto(void)
{
    static uint64_t last_ctrl_ms = 0;
    static const uint64_t ctrl_interval_ms = 60; // 控制间隔60ms

    uint64_t current_ms = xTaskGetTickCount();

    if(current_ms - last_ctrl_ms < ctrl_interval_ms)
    {
        // 如果距离上次控制时间小于控制间隔，则直接返回
        return;
    }
    // 更新上次控制时间
    last_ctrl_ms = current_ms;

    Pose guide_pose = path1.getCurrentGuidePose();

    move.setTargetPosition(guide_pose.x, guide_pose.y, guide_pose.z); // 这里可以根据需要设置目标位置
    //move.setTargetPosition(0, 0, 1.0f); // 设置目标位置为(0, 0, 1.0)，即在Z轴上升1米
    move.update();

    float rollCmd, pitchCmd, throttleCmd;
    move.getAttitudeCommand(rollCmd, pitchCmd, throttleCmd);
    chassis.setTargetAttitude(
        rollCmd - 0.01745f * 0.2f,
        pitchCmd + 0.01745f * 0.2f,
        guide_pose.yaw
    );

    chassis.setThrottleOverride(
        throttleCmd + override_smoother.update(override_throttle)
    );
    motor_smoother.update(throttleCmd + override_smoother.update(override_throttle));
}

void taskStabilize_Emergency(void)
{
    // 紧急状态
    // 设置油门为缓降值，保持当前姿态
    chassis.setThrottleOverride(motor_smoother.update(35.0f));
    chassis.setTargetAttitude(0.0f, 0.0f, chassis.getCurrentYaw());
}

void taskStabilize_Update(void)
{
    chassis.update();
    osDelay(2);
}


void taskStabilize_Control(void)
{
    // 读取手柄是否链接
    bool gs_is_connected = GS_IS_CONNECTED;

    // 读取雷达是否链接
    bool lidar_is_connected = LIDAR_IS_CONNECTED;

    // 是否启动自稳
    bool activate_stabilize = GS_SWITCH(0); 

    // 是否启动自动控制
    bool activate_auto = activate_stabilize && (GS_SWITCH(1));

    // 强制紧急
    bool force_emergency = GS_SWITCH(2);

    // 若手柄未链接，则进入紧急状态
    if(!gs_is_connected)
    {
        // 进入紧急状态
        taskStabilize_Emergency();
        return;
    }
    if(force_emergency)
    {
        // 强制进入紧急状态
        taskStabilize_Emergency();
        return;
    }
    //手柄链接状态，但未启动自稳，说明是停机模式
    if(!activate_stabilize)
    {
        // 进入停机状态
        taskStabilize_Stop();
        return;
    }
    // 手柄链接状态，且启动了自稳，但未启动自动控制，说明是手动控制模式
    if(!activate_auto)
    {
        // 进入手动控制状态
        taskStabilize_Manual();
        return;
    }
    // 手柄链接状态，且启动了自稳，且启动了自动控制，说明是自动控制模式
    // 但雷达掉线，说明是紧急控制模式
    if(!lidar_is_connected)
    {
        // 进入紧急控制状态
        taskStabilize_Emergency();
        return;
    }
    // 手柄链接状态，且启动了自稳，且启动了自动控制，且雷达正常链接，说明是自动控制模式
    taskStabilize_Auto();
}

void taskStabilize(void *argument)
{
    static bool gs_is_connected_prev = false;
    osDelay(1000);
    taskStabilize_Init();
    osDelay(1000);
    
    while (1)
    {
        taskStabilize_Control();
        taskStabilize_Update();
	}
}
