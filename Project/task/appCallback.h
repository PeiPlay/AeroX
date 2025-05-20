#ifndef APP_CALLBACK_H
#define APP_CALLBACK_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void APP_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void APP_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void APP_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void APP_UART_RxCpltCallback(UART_HandleTypeDef *huart);
#ifdef __cplusplus
}
#endif

#endif // APP_CALLBACK_H