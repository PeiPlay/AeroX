#ifndef __NRF_COMMU_H
#define __NRF_COMMU_H
#include "main.h"


typedef struct __attribute__((packed))
{
    uint16_t ball_state:12;     // 球状态
    uint16_t start_dir:1;       // 启动方向
    uint16_t reserve:3;         // 保留位
} commu_nrf_field_info_t; // 2 bytes

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
} commu_nrf_keyboard_state_t; // 12 bytes

typedef struct __attribute__((packed))
{
    commu_nrf_field_info_t field_info;
    commu_nrf_keyboard_state_t keyboard;
} commu_nrf_rx_t; // 14 bytes

extern commu_nrf_rx_t commu_nrf_rx;

// NRF通信回调
void commu_nrf_rx_callback(uint8_t channel, uint8_t* data, uint8_t len);





#endif // __NRF_COMMU_H