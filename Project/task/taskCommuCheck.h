#ifndef TASK_COMMU_CHECK_H
#define TASK_COMMU_CHECK_H
#include "main.h"
#include "cmsis_os.h"

// 前向声明
struct LidarPoseData;
struct LidarImuData;

// ========== 连接恢复状态结构 ==========
typedef struct __attribute__((packed))
{
    uint64_t time_since_last_packet_ms;  // 自最后一包到现在的累计时间(毫秒)
    uint8_t consecutive_packets;         // 满足条件的连续数据包数量
} connection_recovery_state_t;

// ========== NRF地面站手柄数据结构 ==========
typedef struct __attribute__((packed))
{
    uint16_t ball_state:12;     // 球状态
    uint16_t start_dir:1;       // 启动方向
    uint16_t reserve:3;         // 保留位
} ground_station_field_info_t; // 2 bytes

typedef struct __attribute__((packed))
{
    uint32_t fkeys;
    uint32_t switchs;
    struct __attribute__((packed))
    {
        int8_t right_x; 
        int8_t right_y;
        int8_t left_x;
        int8_t left_y;
    } rockers;
} ground_station_keyboard_state_t; // 12 bytes

typedef struct __attribute__((packed))
{
    ground_station_field_info_t field_info;
    ground_station_keyboard_state_t keyboard;
} ground_station_rx_data_t; // 14 bytes

typedef struct __attribute__((packed))
{
    uint8_t is_connected;           // 连接状态: 0 = 未连接, 1 = 已连接
    uint8_t rx_flag;               // 接收标志位: 1 = 本轮询周期内收到数据, 0 = 未收到
    connection_recovery_state_t recovery; // 恢复状态
} ground_station_status_t;

// ========== Lidar连接状态结构 ==========
typedef struct __attribute__((packed))
{
    uint8_t is_connected;               // 连接状态: 0 = 未连接, 1 = 已连接
    uint8_t pose_rx_flag;              // 位姿数据接收标志位
    uint8_t imu_rx_flag;               // IMU数据接收标志位

    uint64_t pose_count;          // 位姿数据包计数
    uint64_t imu_count;           // IMU数据包计数

    float pose_rate;          // 位姿数据包接收率 (Hz)
    float imu_rate;           // IMU数据包接收率 (Hz)

    uint64_t pose_time_since_last_ms;   // 自最后一个位姿包到现在的累计时间(毫秒)
    uint64_t imu_time_since_last_ms;    // 自最后一个IMU包到现在的累计时间(毫秒)
    connection_recovery_state_t recovery; // 恢复状态
} lidar_status_t;

// ========== 全局变量声明 ==========
extern ground_station_rx_data_t ground_station_rx_data; // NRF接收数据结构体
extern ground_station_status_t ground_station_status; // NRF状态数据结构体
extern lidar_status_t lidar_status; // Lidar状态数据结构体

// ========== 函数声明 ==========
#ifdef __cplusplus
extern "C" {
#endif

// NRF通信回调函数
void ground_station_rx_callback(uint8_t channel, uint8_t* data, uint8_t len);

// Lidar通信回调函数
void lidar_pose_rx_callback(const struct LidarPoseData* pose_data);
void lidar_imu_rx_callback(const struct LidarImuData* imu_data);

// 通信检查任务主函数
void taskCommuCheck(void *argument);

#ifdef __cplusplus
}
#endif

// ========== 便捷宏定义 ==========
#define GS_IS_CONNECTED     ground_station_status.is_connected
#define GS_RX_DATA          ground_station_rx_data
#define GS_FKEY(n)          (bool)(ground_station_rx_data.keyboard.fkeys & ((uint32_t)1 << (uint32_t)n))
#define GS_SWITCH(n)        (bool)(ground_station_rx_data.keyboard.switchs & ((uint32_t)1 << (uint32_t)n))
#define GS_ROCKERS          ground_station_rx_data.keyboard.rockers

// Lidar便捷宏定义
#define LIDAR_IS_CONNECTED  lidar_status.is_connected

#endif // TASK_COMMU_CHECK_H