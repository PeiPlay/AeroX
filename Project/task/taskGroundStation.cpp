#include "taskManager.h"
#include "config.h"

ground_station_rx_t ground_station_rx_data; // NRF接收数据结构体

// NRF通信回调
void ground_station_rx_callback(uint8_t channel, uint8_t* data, uint8_t len)
{
    if(len != sizeof(ground_station_rx_t)) return;
    ground_station_rx_data = *(ground_station_rx_t*)data;
}

void taskGroundStation_Init(void* argument)
{
    nrf.init();
}