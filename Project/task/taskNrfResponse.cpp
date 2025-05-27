#include "taskNrfResponse.h"
#include "config.h"
#include "cmsis_os.h"
#include <string.h>

// ========== 全局变量定义 ==========
NrfCommu_ReceivePackage_t nrf_response_package = {0}; // 全局响应数据包，供外界访问

// ========== NRF响应任务主函数 ==========
extern "C" {
void taskNrfResponse(void *argument)
{
    // 初始化响应数据包
    osDelay(1500);
    memset(&nrf_response_package, 0, sizeof(nrf_response_package));
    
    // static uint32_t last_transmit_time = 0;
    // bool test = false; // 测试标志位
    
    // 进入主循环
    while (1)
    {
        // 创建临时变量用于发送，确保线程安全
        NrfCommu_ReceivePackage_t temp_package;

        // if(HAL_GetTick() - last_transmit_time > 1000) // 如果距离上次发送大于1000ms
        // {
        //     last_transmit_time = HAL_GetTick(); // 获取当前时间戳
        //     test = !test; // 设置测试标志位
        // }

        // // 更新响应数据包内容
        // NRF_SET_CHASSIS_VESC_BIT(0, test); // 测试位0

        // 临界区：快速拷贝数据包到临时变量
        
        memcpy(&temp_package, &nrf_response_package, sizeof(NrfCommu_ReceivePackage_t));
        // 发送响应数据包到手柄（使用副本）
        taskENTER_CRITICAL();
        nrf.transmit((uint8_t*)&temp_package, sizeof(temp_package));
        taskEXIT_CRITICAL();
        // 50ms周期延迟
        osDelay(50);
    }
}
}
