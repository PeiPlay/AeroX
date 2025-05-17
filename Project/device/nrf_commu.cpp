#include "nrf_commu.h"

commu_nrf_rx_t commu_nrf_rx;



// NRF数据包接收回调函数
void commu_nrf_rx_callback(uint8_t channel, uint8_t* data, uint8_t len)
{
    if(len != sizeof(commu_nrf_rx_t)) return;
    commu_nrf_rx = *(commu_nrf_rx_t*)data;
}