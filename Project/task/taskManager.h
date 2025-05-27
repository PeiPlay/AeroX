#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H
#include "main.h"
#include "cmsis_os.h"
#include "taskCommuCheck.h"
#include "taskAttitude.h"
#include "taskStabilize.h"
#include "taskMovement.h"  // 添加运动控制任务头文件

#ifdef __cplusplus
extern "C" {
#endif
void taskAttitudeIMU(void *argument);
void taskStabilize(void *argument);
void taskManager_Init(void* argument);
void taskCommuCheck_Init(void* argument);  // 通信检查初始化函数声明
void StartCommuCheckTask(void* argument);  // StartCommuCheckTask重定向函数声明
void taskNrfResponse(void *argument);      // NRF响应任务函数声明
void taskMovement(void *argument);         // 运动控制任务函数声明
#ifdef __cplusplus
}
#endif

#endif // TASK_MANAGER_H