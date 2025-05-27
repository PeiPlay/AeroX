#include "taskManager.h"
#include "taskCommuCheck.h"  // 添加新的头文件包含
#include "taskNrfResponse.h" // 添加NrfResponse头文件包含
#include "taskMovement.h"    // 添加Movement头文件包含
#include "cmsis_os.h"
#include "main.h"



void StartDefaultTask(void *argument)
{
  taskManager_Init(argument);  

  for (;;)
  {
    osDelay(100); 
  }
}


void StartAttitudeIMUTask(void *argument)
{
    /* USER CODE BEGIN StartAttitudeIMUTask */
    taskAttitudeIMU(argument); // 调用任务函数
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
    /* USER CODE END StartAttitudeIMUTask */
}

void StartStabilizeTask(void *argument)
{
    /* USER CODE BEGIN StartStabilizeTask */
    taskStabilize(argument); // 调用任务函数
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
    /* USER CODE END StartStabilizeTask */
}

void StartGroundStationTask(void *argument)
{
  /* USER CODE BEGIN StartGroundStationTask */
  taskCommuCheck(argument); // 调用新的通信检查任务函数

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGroundStationTask */
}

void StartCommuCheckTask(void *argument)
{
  /* USER CODE BEGIN StartCommuCheckTask */
  taskCommuCheck(argument); // 重定向到新的通信检查任务函数

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCommuCheckTask */
}

void StartNrfResponseTask(void *argument)
{
  /* USER CODE BEGIN StartNrfResponseTask */
  taskNrfResponse(argument); // 重定向到NRF响应任务函数

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartNrfResponseTask */
}

void StartMoveTask(void *argument)
{
  /* USER CODE BEGIN StartMoveTask */
  taskMovement(argument); // 重定向到运动控制任务函数

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMoveTask */
}