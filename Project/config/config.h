#ifndef __CONFIG_H__
#define __CONFIG_H__

// C++部分
#ifdef __cplusplus
// Attitude
#include "Attitude.h"
#include "BMI088.h"
#include "MahonyAHRS.h"
// motor
#include "motor.h"
#include "sdc_dual.h"
// module
#include "scheduler.h"
#include "pid.h"
#include "serial_stream.h"
// device
#include "up_t201.h"
#include "spl06.h"
#include "nrf.h"
#include "ws2812.h"
// component
#include "chassis.h"
// debug
#include "vofa.h"

// // BMI088 配置
// #define CONFIG_BMI088_SET                              \
//     (BMI088Config_t)                                   \
//     {                                                  \
//         .hspi = &hspi2,                                \
//         .ce_acc = {.port = GPIOC, .pin = GPIO_PIN_0},  \
//         .ce_gyro = {.port = GPIOC, .pin = GPIO_PIN_3}, \
//         .gyroRange = BMI088_GYRO_2000,                 \
//         .accelRange = BMI088_ACC_RANGE_3G              \
//     }
// extern BMI088 bmi088;
// extern MahonyAHRS mahony_estimator;
// extern AttitudeManager attitude_manager;

// // UPT201 配置
// extern UPT201 upt201;

// // 四旋翼电机
// extern SdcDualMotor motor_1;
// extern SdcDualMotor motor_2;
// extern SdcDualMotor motor_3;
// extern SdcDualMotor motor_4;

// // 三轴角度pid(外环)

// #define CONFIG_PID_ROLL_RAD_SET                                     \
//     (PidConfig_t)                                                   \
//     {                                                               \
//         .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,                         \
//         .maxOutput = 0.0f, .maxIntegral = 0.0f,                     \
//         .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
//         .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
//     }

// #define CONFIG_PID_PITCH_RAD_SET                                    \
//     (PidConfig_t)                                                   \
//     {                                                               \
//         .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,                         \
//         .maxOutput = 0.0f, .maxIntegral = 0.0f,                     \
//         .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
//         .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
//     }

// #define CONFIG_PID_YAW_RAD_SET                                      \
//     (PidConfig_t)                                                   \
//     {                                                               \
//         .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,                         \
//         .maxOutput = 0.0f, .maxIntegral = 0.0f,                     \
//         .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
//         .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
//     }
// extern PidController pid_roll_rad;
// extern PidController pid_pitch_rad;
// extern PidController pid_yaw_rad;

// //三轴角速度pid(内环)

// #define CONFIG_PID_ROLL_SPD_SET                                     \
//     (PidConfig_t)                                                   \
//     {                                                               \
//         .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,                         \
//         .maxOutput = 0.0f, .maxIntegral = 0.0f,                     \
//         .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
//         .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
//     }

// #define CONFIG_PID_PITCH_SPD_SET                                    \
//     (PidConfig_t)                                                   \
//     {                                                               \
//         .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,                         \
//         .maxOutput = 0.0f, .maxIntegral = 0.0f,                     \
//         .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
//         .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
//     }

// #define CONFIG_PID_YAW_SPD_SET                                      \
//     (PidConfig_t)                                                   \
//     {                                                               \
//         .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,                         \
//         .maxOutput = 0.0f, .maxIntegral = 0.0f,                     \
//         .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
//         .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
//     }

// extern PidController pid_roll_spd;
// extern PidController pid_pitch_spd;
// extern PidController pid_yaw_spd;

// // 飞控底盘

// #define CONFIG_CHASSIS_SET                                          \
//     (ChassisDependencies)                                           \
//     {                                                               \
//         .attitudeMgr = &attitude_manager,                           \
//         .motors = {&motor_1, &motor_2, &motor_3, &motor_4},         \
//         .anglePIDs = {&pid_roll_rad, &pid_pitch_rad, &pid_yaw_rad}, \
//         .ratePIDs = { &pid_roll_spd,                                \
//                       &pid_pitch_spd,                               \
//                       &pid_yaw_spd }                                \
//     }
// extern Chassis chassis;

#endif

#endif // __CONFIG_H__