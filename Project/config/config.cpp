#include "config.h"
#include "cmsis_os.h"
#include "main.h"
#include "usart.h"
#include "move.h"


// Attitude
BMI088 bmi088(CONFIG_BMI088_SET);
MahonyAHRS mahony_estimator(500.0f, 0.55f, 0.002f);
AttitudeManager attitude_manager(&bmi088, &mahony_estimator);


// NRF
Nrf nrf(CONFIG_NRF_SET);

// UPT20X
UPT20X upt201(&huart8);

// 四旋翼电机
SdcDualMotor motor_1(&htim1, TIM_CHANNEL_1, false);
SdcDualMotor motor_2(&htim1, TIM_CHANNEL_2, false);
SdcDualMotor motor_3(&htim1, TIM_CHANNEL_3, false);
SdcDualMotor motor_4(&htim1, TIM_CHANNEL_4, false);

PidController pid_roll_rad(CONFIG_PID_ROLL_RAD_SET);
PidController pid_pitch_rad(CONFIG_PID_PITCH_RAD_SET);
PidController pid_yaw_rad(CONFIG_PID_YAW_RAD_SET);

//三轴角速度pid(内环)

PidController pid_roll_spd(CONFIG_PID_ROLL_SPD_SET);
PidController pid_pitch_spd(CONFIG_PID_PITCH_SPD_SET);
PidController pid_yaw_spd(CONFIG_PID_YAW_SPD_SET);

// 飞控底盘
Chassis chassis(CONFIG_CHASSIS_SET);

Lidar lidar(&huart1);

PidController pid_x_vel(CONFIG_PID_X_VEL_SET);
PidController pid_y_vel(CONFIG_PID_Y_VEL_SET);
PidController pid_z_vel(CONFIG_PID_Z_VEL_SET);

PidController pid_x_pos(CONFIG_PID_X_POS_SET);
PidController pid_y_pos(CONFIG_PID_Y_POS_SET);
PidController pid_z_pos(CONFIG_PID_Z_POS_SET);

Move move(CONFIG_MOVE_SET);

Point point_begin(CONFIG_POINT_BEGIN_POSE_SET, CONFIG_POINT_PREPARE_TOLERANCE_SET);
Point point_begin2(CONFIG_POINT_BEGIN2_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_BEGIN_SET);
Point point_stable1(CONFIG_POINT_STABLE1_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_STABLE_SHORT_SET);
Point point_task3(CONFIG_POINT_TASK3_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_1_SET);
 
Point point_climb_prepare(CONFIG_POINT_CLIMB_PREPARE_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_STABLE_SET);
Point point_climb_point1(CONFIG_POINT_CLIMB_POINT1_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_CLIMB_SET);
Point point_climb_point2(CONFIG_POINT_CLIMB_POINT2_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_2_SET);
Point point_climb_finish(CONFIG_POINT_CLIMB_FINISH_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_CLIMB_SET);

Point point_task4(CONFIG_POINT_TASK4_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_2_SET);
Point point_task5(CONFIG_POINT_TASK5_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_2_SET);
Point point_end1(CONFIG_POINT_END_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_2_SET);
Point point_stable2(CONFIG_POINT_STABLE2_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_STABLE_SHORT_SET);
Point point_task5_2(CONFIG_POINT_TASK5_2_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_1_SET);
Point point_stable3(CONFIG_POINT_STABLE3_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_STABLE_SHORT_SET);
Point point_task5_3(CONFIG_POINT_TASK5_3_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_1_SET);
Point point_stable4(CONFIG_POINT_STABLE4_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_STABLE_SHORT_SET);
Point point_task5_4(CONFIG_POINT_TASK5_4_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_2_SET);
Point point_end2(CONFIG_POINT_END_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_2_SET);
Point point_stop(CONFIG_POINT_STOP_POSE_SET, CONFIG_POINT_GENERAL_TOLERANCE_STOP_SET);
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
Path path1;

HC12 hc12(&huart3, GPIOE, GPIO_PIN_15); // HC-12 SET引脚连接到PE15
