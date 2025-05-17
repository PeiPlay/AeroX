#ifndef __CLOCK_DRV_H
#define __CLOCK_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h" // 包含 main.h 通常会引入 HAL 库定义


uint32_t TimDrv_GetPeriphCLKFreq(const TIM_HandleTypeDef *htim);
void TimDrv_Start(TIM_HandleTypeDef *htim);
void TimDrv_Shutdown(TIM_HandleTypeDef *htim);
uint8_t TimDrv_CalcPscAndAtr(TIM_HandleTypeDef *htim, double freq, uint32_t *psc, uint32_t *atr);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_DRV_H */

