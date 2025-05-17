#include "config.h"
#include "cmsis_os.h"
#include "main.h"
#include "usart.h"


// // Attitude
// BMI088 bmi088(CONFIG_BMI088_SET);
// MahonyAHRS mahony_estimator(500.0f, 0.55f, 0.002f);
// AttitudeManager attitude_manager(&bmi088, &mahony_estimator);

// // UPT201
// UPT201 upt201(&huart7, 2048, osPriorityBelowNormal);

// // 四旋翼电机
// SdcDualMotor motor_1(&htim1, TIM_CHANNEL_1, false);
// SdcDualMotor motor_2(&htim1, TIM_CHANNEL_2, false);
// SdcDualMotor motor_3(&htim1, TIM_CHANNEL_3, false);
// SdcDualMotor motor_4(&htim1, TIM_CHANNEL_4, false);

// PidController pid_roll_rad(CONFIG_PID_ROLL_RAD_SET);
// PidController pid_pitch_rad(CONFIG_PID_PITCH_RAD_SET);
// PidController pid_yaw_rad(CONFIG_PID_YAW_RAD_SET);

// //三轴角速度pid(内环)

// PidController pid_roll_spd(CONFIG_PID_ROLL_SPD_SET);
// PidController pid_pitch_spd(CONFIG_PID_PITCH_SPD_SET);
// PidController pid_yaw_spd(CONFIG_PID_YAW_SPD_SET);

// // 飞控底盘
// Chassis chassis(CONFIG_CHASSIS_SET);
