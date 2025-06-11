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
#include "hc12.h"

#include "point.h"
#include "path.h"
#include "track.h"

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
        .kp = 5.2f, .ki = 0.0f, .kd = 0.9f,                         \
        .maxOutput = 3.5f, .maxIntegral = 0.0f,                     \
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
        .kp = 0.2f, .ki = 0.0f, .kd = 0.01f,                        \
        .maxOutput = 0.25f, .maxIntegral = 0.0f,                    \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }

#define CONFIG_PID_Y_VEL_SET                                        \
    (PidConfig_t)                                                   \
    {                                                               \
        .kp = 0.2f, .ki = 0.0f, .kd = 0.01f,                        \
        .maxOutput = 0.25f, .maxIntegral = 0.0f,                    \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }

#define CONFIG_PID_Z_VEL_SET                                        \
    (PidConfig_t)                                                   \
    {                                                               \
        .kp = 4.1f, .ki = 0.0f, .kd = 0.5f,                         \
        .maxOutput = 30.0f, .maxIntegral = 0.0f,                    \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }

extern PidController pid_x_vel;
extern PidController pid_y_vel;
extern PidController pid_z_vel;

#define CONFIG_PID_X_POS_SET                                        \
    (PidConfig_t)                                                   \
    {                                                               \
        .kp = 0.9f, .ki = 0.0f, .kd = 0.4f,                         \
        .maxOutput = 0.6f, .maxIntegral = 0.0f,                     \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }

#define CONFIG_PID_Y_POS_SET                                        \
    (PidConfig_t)                                                   \
    {                                                               \
        .kp = 0.9f, .ki = 0.0f, .kd = 0.4f,                         \
        .maxOutput = 0.6f, .maxIntegral = 0.0f,                     \
        .integralSeparationThreshold = 0.0f, .errorDeadband = 0.0f, \
        .antiSaturationEnabled = 0, .diffFilterEnabled = 0,         \
    }

#define CONFIG_PID_Z_POS_SET                                         \
    (PidConfig_t)                                                    \
    {                                                                \
        .kp = 1.2f, .ki = 0.05f, .kd = 0.2f,                         \
        .maxOutput = 1.5f, .maxIntegral = 0.8f,                      \
        .integralSeparationThreshold = 1.0f, .errorDeadband = 0.05f, \
        .antiSaturationEnabled = 1, .diffFilterEnabled = 0,          \
    }

extern PidController pid_x_pos;
extern PidController pid_y_pos;
extern PidController pid_z_pos;

#define CONFIG_MOVE_SET                                       \
    (MoveDependencies)                                        \
    {                                                         \
        .lidar = &lidar,                                      \
        .chassis = &chassis,                                  \
        .positionPIDs = {&pid_x_pos, &pid_y_pos, &pid_z_pos}, \
        .velocityPIDs = { &pid_x_vel,                         \
                          &pid_y_vel,                         \
                          &pid_z_vel }                        \
    }

extern Move move;

#define CONFIG_POINT_PREPARE_TOLERANCE_SET (ToleranceParams){ \
    .err_r = 5.0f,                                            \
    .err_yaw = 0.4,                                           \
    .threshold = 300}

#define CONFIG_POINT_GENERAL_TOLERANCE_1_SET (ToleranceParams){ \
    .err_r = 0.5f,                                              \
    .err_yaw = 0.4,                                             \
    .threshold = 1}
#define CONFIG_POINT_GENERAL_TOLERANCE_BEGIN_SET (ToleranceParams){ \
    .err_r = 5.0f,                                                  \
    .err_yaw = 0.4,                                                 \
    .threshold = 10}

#define CONFIG_POINT_GENERAL_TOLERANCE_STABLE_SET (ToleranceParams){ \
    .err_r = 5.0f,                                                   \
    .err_yaw = 0.4,                                                  \
    .threshold = 50}

#define CONFIG_POINT_GENERAL_TOLERANCE_STABLE_SHORT_SET (ToleranceParams){ \
    .err_r = 5.0f,                                                         \
    .err_yaw = 0.4,                                                        \
    .threshold = 150}

#define CONFIG_POINT_GENERAL_TOLERANCE_STOP_SET (ToleranceParams){ \
    .err_r = 5.0f,                                                 \
    .err_yaw = 0.4,                                                \
    .threshold = 10}

#define CONFIG_POINT_GENERAL_TOLERANCE_2_SET (ToleranceParams){ \
    .err_r = 0.5f,                                              \
    .err_yaw = 0.4,                                             \
    .threshold = 1}
#define CONFIG_POINT_GENERAL_TOLERANCE_CLIMB_SET (ToleranceParams){ \
    .err_r = 5.0f,                                                  \
    .err_yaw = 0.4,                                                 \
    .threshold = 10}

#define CONFIG_POINT_BEGIN_POSE_SET (Pose){ \
    .x = 0.0f,                              \
    .y = 0.0f,                              \
    .z = 0.4f,                              \
    .yaw = 0.0f}

#define CONFIG_POINT_BEGIN2_POSE_SET (Pose){ \
    .x = 0.0f,                               \
    .y = 0.0f,                               \
    .z = 1.0f,                               \
    .yaw = 0.0f}

#define CONFIG_POINT_STABLE1_POSE_SET (Pose){ \
    .x = 0.0f,                                \
    .y = 0.10f,                               \
    .z = 0.95f,                               \
    .yaw = 0.0f}

#define CONFIG_POINT_TASK3_POSE_SET (Pose){ \
    .x = 0.0f,                              \
    .y = 6.4f - 0.4f,                       \
    .z = 1.0f,                              \
    .yaw = 0.0f}

// T3点完成后悬停一会
#define CONFIG_POINT_CLIMB_PREPARE_POSE_SET (Pose){ \
    .x = 0.0f,                                      \
    .y = 6.4f - 0.4f,                               \
    .z = 1.0f,                                      \
    .yaw = -1.57f}
// 悬停后进行爬升
#define CONFIG_POINT_CLIMB_POINT1_POSE_SET (Pose){ \
    .x = 0.0f,                                     \
    .y = 6.4f - 0.4f,                              \
    .z = 1.0f + 0.8f+0.5f,                              \
    .yaw = -1.57f}
//
#define CONFIG_POINT_CLIMB_POINT2_POSE_SET (Pose){ \
    .x = -4.49f,                                   \
    .y = 6.4f - 0.4f,                              \
    .z = 1.0f + 0.8f+0.5f,                         \
    .yaw = -1.57f}

//
#define CONFIG_POINT_CLIMB_FINISH_POSE_SET (Pose){ \
    .x = -4.49f,                                   \
    .y = 6.4f - 0.4f,                              \
    .z = 1.0f,                                     \
    .yaw = -1.57f}

#define CONFIG_POINT_TASK4_POSE_SET (Pose){ \
    .x = -4.49f /*-2.5f*/,                  \
    .y = 6.4f - 0.4f /*5.6f*/,              \
    .z = 1.0f,                              \
    .yaw = -1.57f}

#define CONFIG_POINT_TASK5_POSE_SET (Pose){ \
    .x = -4.49f /*-2.5f*/,                  \
    .y = 6.4f - 1.1f /*5.6f - 1.6f*/,       \
    .z = 1.0f,                              \
    .yaw = -1.57f - 1.57f}

#define CONFIG_POINT_END_POSE_SET (Pose){            \
    .x = -4.49f /*-2.5f*/,                           \
    .y = 6.4f - 1.1f - 3.33f /*5.6f - 1.6f - 2.0f*/, \
    .z = 1.0f,                                       \
    .yaw = -1.57f - 1.57f}

#define CONFIG_POINT_STABLE2_POSE_SET (Pose){        \
    .x = -4.49f /*-2.45f*/,                          \
    .y = 6.4f - 1.1f - 3.33f /*5.6f - 1.6f - 2.0f*/, \
    .z = 0.95f,                                      \
    .yaw = -1.57f - 1.57f}

#define CONFIG_POINT_TASK5_2_POSE_SET (Pose){ \
    .x = -4.49f /*-2.5f*/,                    \
    .y = 6.4f - 1.1f /*5.6f - 1.6f*/,         \
    .z = 1.0f,                                \
    .yaw = 0.0f}

#define CONFIG_POINT_STABLE3_POSE_SET (Pose){ \
    .x = -4.49f /*-2.55f*/,                   \
    .y = 6.4f - 1.1f /*5.6f - 1.6f*/,         \
    .z = 0.95f,                               \
    .yaw = 0.0f}

#define CONFIG_POINT_TASK5_3_POSE_SET (Pose){ \
    .x = -4.49f /*-2.5f*/,                    \
    .y = 6.4f - 1.1f /*5.6f - 1.6f*/,         \
    .z = 0.5f,                                \
    .yaw = 0.0f}

#define CONFIG_POINT_STABLE4_POSE_SET (Pose){ \
    .x = -4.49f /*-2.55f*/,                   \
    .y = 6.4f - 1.1f /*5.6f - 1.6f*/,         \
    .z = 0.49f,                               \
    .yaw = 0.0f}

#define CONFIG_POINT_TASK5_4_POSE_SET (Pose){ \
    .x = -4.49f /*-2.5f*/,                    \
    .y = 6.4f - 1.1f /*5.6f - 1.6f*/,         \
    .z = 1.0f,                                \
    .yaw = -1.57f - 1.57f}

#define CONFIG_POINT_STOP_POSE_SET (Pose){           \
    .x = -4.49f /*-2.5f*/,                           \
    .y = 6.4f - 1.1f - 3.33f /*5.6f - 1.6f - 2.0f*/, \
    .z = -0.2f,                                      \
    .yaw = -1.57f - 1.57f}

#define CONFIG_POINT_GENERAL_TOLERANCE_SET (ToleranceParams){ \
    .err_r = 0.6f,                                            \
    .err_yaw = 0.4,                                           \
    .threshold = 1}

/*
y=560
x=-250
y - 160
y - 360
*/
/*
//483
//800
*/

extern Point point_begin;
extern Point point_begin2;
extern Point point_stable1;
extern Point point_task3;

extern Point point_climb_prepare;
extern Point point_climb_point1;
extern Point point_climb_point2;
extern Point point_climb_finish;

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

extern Path path1;

extern HC12 hc12;

#endif

#endif // __CONFIG_H__