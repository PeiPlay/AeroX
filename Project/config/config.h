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
#include "upt20x.h"
#include "spl06.h"
#include "nrf.h"
#include "ws2812.h"
#include "lidar.h"

// component
#include "chassis.h"
#include "move.h"
// debug
#include "vofa.h"
//
#include "taskManager.h"

// BMI088 配置
#define CONFIG_BMI088_SET                              \
    (BMI088Config_t)                                   \
    {                                                  \
        .hspi = &hspi2,                                \
        .ce_acc = {.port = GPIOC, .pin = GPIO_PIN_0},  \
        .ce_gyro = {.port = GPIOC, .pin = GPIO_PIN_3}, \
        .gyroRange = BMI088_GYRO_2000,                 \
        .accelRange = BMI088_ACC_RANGE_3G              \
    }
extern BMI088 bmi088;
extern MahonyAHRS mahony_estimator;
extern AttitudeManager attitude_manager;

// NRF 配置

#define CONFIG_NRF_SET                                 \
    (Nrf_t)                                            \
    {                                                  \
        .hspi = &hspi4,                                \
        .ce = {.port = GPIOE, .pin = GPIO_PIN_3},      \
        .nss = {.port = GPIOE, .pin = GPIO_PIN_4},     \
        .irq = {.port = GPIOC, .pin = GPIO_PIN_13},    \
        .address_receive =                             \
            {                                          \
                .p1 = 'L',                             \
                .p2 = 0x01,                            \
                .p3 = 0x02,                            \
                .p4 = 0x03,                            \
                .p5 = 0x04,                            \
                .high_addr = {'I', 'M', 'I', 'T'}},    \
        .address_transmit = {'F', '2', '4', 'R', 'C'}, \
        .rf_channel = 6,                               \
        .nrf_rx_callback = ground_station_rx_callback  \
    }

extern Nrf nrf;
extern ground_station_rx_data_t ground_station_rx_data;

// UPT201 配置
extern UPT20X upt201;

// 四旋翼电机
extern SdcDualMotor motor_1;
extern SdcDualMotor motor_2;
extern SdcDualMotor motor_3;
extern SdcDualMotor motor_4;

// 三轴角度pid(外环)

#define CONFIG_PID_ROLL_RAD_SET                                     \
    (PidConfig_t)                                                   \
    {                                                               \
        .kp = 15.0f / 2.0f, .ki = 0.0f, .kd = 0.7f,                 \
        .maxOutput = 5.0f, .maxIntegral = 0.0f,                     \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }

#define CONFIG_PID_PITCH_RAD_SET                                    \
    (PidConfig_t)                                                   \
    {                                                               \
        .kp = 15.0f / 2.0f, .ki = 0.0f, .kd = 0.7f,                 \
        .maxOutput = 5.0f, .maxIntegral = 0.0f,                     \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }

#define CONFIG_PID_YAW_RAD_SET                                      \
    (PidConfig_t)                                                   \
    {                                                               \
        .kp = 7.0f / 2.0f, .ki = 0.0f, .kd = 0.9f,                  \
        .maxOutput = 3.0f, .maxIntegral = 0.0f,                     \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }
extern PidController pid_roll_rad;
extern PidController pid_pitch_rad;
extern PidController pid_yaw_rad;

// 三轴角速度pid(内环)

#define CONFIG_PID_ROLL_SPD_SET                                     \
    (PidConfig_t)                                                   \
    {                                                               \
        .kp = 6.5f, .ki = 0.0f, .kd = 0.9f,                         \
        .maxOutput = 95.0f, .maxIntegral = 0.0f,                    \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }

#define CONFIG_PID_PITCH_SPD_SET                                    \
    (PidConfig_t)                                                   \
    {                                                               \
        .kp = 6.5f, .ki = 0.0f, .kd = 0.9f,                         \
        .maxOutput = 95.0f, .maxIntegral = 0.0f,                    \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }

#define CONFIG_PID_YAW_SPD_SET                                      \
    (PidConfig_t)                                                   \
    {                                                               \
        .kp = 6.5f, .ki = 0.0f, .kd = 0.9f,                         \
        .maxOutput = 95.0f, .maxIntegral = 0.0f,                    \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }

extern PidController pid_roll_spd;
extern PidController pid_pitch_spd;
extern PidController pid_yaw_spd;

extern Lidar lidar;

// 飞控底盘

#define CONFIG_CHASSIS_SET                                          \
    (ChassisDependencies)                                           \
    {                                                               \
        .attitudeMgr = &attitude_manager,                           \
        .motors = {&motor_1, &motor_2, &motor_3, &motor_4},         \
        .anglePIDs = {&pid_roll_rad, &pid_pitch_rad, &pid_yaw_rad}, \
        .ratePIDs = { &pid_roll_spd,                                \
                      &pid_pitch_spd,                               \
                      &pid_yaw_spd }                                \
    }
extern Chassis chassis;

// 运动控制

#define CONFIG_PID_X_VEL_SET                                        \
    (PidConfig_t)                                                   \
    {                                                               \
        .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,                         \
        .maxOutput = 0.0f, .maxIntegral = 0.0f,                     \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }

#define CONFIG_PID_Y_VEL_SET                                        \
    (PidConfig_t)                                                   \
    {                                                               \
        .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,                         \
        .maxOutput = 0.0f, .maxIntegral = 0.0f,                     \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }

#define CONFIG_PID_Z_VEL_SET                                        \
    (PidConfig_t)                                                   \
    {                                                               \
        .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,                         \
        .maxOutput = 0.0f, .maxIntegral = 0.0f,                     \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }

extern PidController pid_x_vel;
extern PidController pid_y_vel;
extern PidController pid_z_vel;

#define CONFIG_PID_X_POS_SET                                        \
    (PidConfig_t)                                                   \
    {                                                               \
        .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,                         \
        .maxOutput = 0.0f, .maxIntegral = 0.0f,                     \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }

#define CONFIG_PID_Y_POS_SET                                        \
    (PidConfig_t)                                                   \
    {                                                               \
        .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,                         \
        .maxOutput = 0.0f, .maxIntegral = 0.0f,                     \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }

#define CONFIG_PID_Z_POS_SET                                        \
    (PidConfig_t)                                                   \
    {                                                               \
        .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,                         \
        .maxOutput = 0.0f, .maxIntegral = 0.0f,                     \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }

extern PidController pid_x_pos;
extern PidController pid_y_pos;
extern PidController pid_z_pos;

#define CONFIG_MOVE_SET                                       \
    (MoveDependencies)                                        \
    {                                                         \
        .lidar = &lidar,                                      \
        .positionPIDs = {&pid_x_pos, &pid_y_pos, &pid_z_pos}, \
        .velocityPIDs = { &pid_x_vel,                         \
                          &pid_y_vel,                         \
                          &pid_z_vel }                        \
    }

extern Move move;

#endif

#endif // __CONFIG_H__