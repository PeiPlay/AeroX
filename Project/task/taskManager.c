#include "taskManager.h"
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