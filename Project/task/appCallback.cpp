#include "appCallback.h"
#include "config.h"


#ifdef __cplusplus
extern "C" {
#endif


void APP_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    nrf.handleEXTI(GPIO_Pin); // 调用NRF的中断处理函数


}
void APP_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    
}
void APP_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    lidar.dmaRxCallback(huart); // 调用Lidar的串口接收回调函数
}


#ifdef __cplusplus
}
#endif