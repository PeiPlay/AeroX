#ifndef TASK_GROUND_STATION_H
#define TASK_GROUND_STATION_H
#include "main.h"
#include "time_utils.h"
#include "cmsis_os.h"


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
} ground_station_rx_t; // 14 bytes


typedef struct __attribute__((packed))
{
    uint8_t is_connected;      // 连接状态: 0 = 未连接, 1 = 已连接
    timestamp_t last_rx_timestamp; // 最新接收到数据包的时间戳

    // 用于恢复逻辑的字段
    timestamp_t recovery_first_packet_ts; // 当前恢复序列中第一个数据包的时间戳
    timestamp_t recovery_last_packet_ts;  // 当前恢复序列中最近一个数据包的时间戳
    uint8_t recovery_packet_streak;   // 满足短间隔条件的连续数据包数量
    // uint8_t reserve[3]; // 可选的填充字节，用于对齐或未来使用
} ground_station_status_t; // 大小: 1 + 8 + 8 + 8 + 1 = 26 bytes

extern ground_station_rx_t ground_station_rx_data; // NRF接收数据结构体
extern ground_station_status_t ground_station_status_data; // NRF状态数据结构体

void ground_station_rx_callback(uint8_t channel, uint8_t* data, uint8_t len);

void taskGroundStation_Init(void* argument);

#define gs_is_connected ground_station_status_data.is_connected
#define gs_data ground_station_rx_data
#define gs_fkey(n) (bool)(ground_station_rx_data.keyboard.fkeys & ((uint32_t)1 << (uint32_t)n))
#define gs_switch(n) (bool)(ground_station_rx_data.keyboard.switchs & ((uint32_t)1 << (uint32_t)n))
#define gs_rockers ground_station_rx_data.keyboard.rockers

#endif // TASK_GROUND_STATION_H