#ifndef TASK_MOVEMENT_H
#define TASK_MOVEMENT_H

#include "main.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 运动控制任务函数
 * @param argument 任务参数
 */
void taskMovement(void *argument);

/**
 * @brief 运动控制初始化函数
 */
void taskMovement_Init(void);

#ifdef __cplusplus
}
#endif

#endif // TASK_MOVEMENT_H
