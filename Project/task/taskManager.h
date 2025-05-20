#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H
#include "main.h"
#include "cmsis_os.h"
#include "taskGroundStation.h"
#include "taskAttitude.h"
#include "taskStabilize.h"


#ifdef __cplusplus
extern "C" {
#endif
void taskAttitudeIMU(void *argument);
void taskStabilize(void *argument);
void taskManager_Init(void* argument);  // 添加函数声明

#ifdef __cplusplus
}
#endif

#endif // TASK_MANAGER_H