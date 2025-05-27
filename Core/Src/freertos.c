/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AttitudeIMUTask */
osThreadId_t AttitudeIMUTaskHandle;
const osThreadAttr_t AttitudeIMUTask_attributes = {
  .name = "AttitudeIMUTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for StabilizeTask */
osThreadId_t StabilizeTaskHandle;
const osThreadAttr_t StabilizeTask_attributes = {
  .name = "StabilizeTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GroundStationTa */
osThreadId_t GroundStationTaHandle;
const osThreadAttr_t GroundStationTa_attributes = {
  .name = "GroundStationTa",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CommuCheckTask */
osThreadId_t CommuCheckTaskHandle;
const osThreadAttr_t CommuCheckTask_attributes = {
  .name = "CommuCheckTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for NrfResponseTask */
osThreadId_t NrfResponseTaskHandle;
const osThreadAttr_t NrfResponseTask_attributes = {
  .name = "NrfResponseTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartAttitudeIMUTask(void *argument);
void StartStabilizeTask(void *argument);
void StartGroundStationTask(void *argument);
void StartCommuCheckTask(void *argument);
void StartNrfResponseTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of AttitudeIMUTask */
  AttitudeIMUTaskHandle = osThreadNew(StartAttitudeIMUTask, NULL, &AttitudeIMUTask_attributes);

  /* creation of StabilizeTask */
  StabilizeTaskHandle = osThreadNew(StartStabilizeTask, NULL, &StabilizeTask_attributes);

  /* creation of GroundStationTa */
  GroundStationTaHandle = osThreadNew(StartGroundStationTask, NULL, &GroundStationTa_attributes);

  /* creation of CommuCheckTask */
  CommuCheckTaskHandle = osThreadNew(StartCommuCheckTask, NULL, &CommuCheckTask_attributes);

  /* creation of NrfResponseTask */
  NrfResponseTaskHandle = osThreadNew(StartNrfResponseTask, NULL, &NrfResponseTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartAttitudeIMUTask */
/**
* @brief Function implementing the AttitudeIMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAttitudeIMUTask */
__weak void StartAttitudeIMUTask(void *argument)
{
  /* USER CODE BEGIN StartAttitudeIMUTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartAttitudeIMUTask */
}

/* USER CODE BEGIN Header_StartStabilizeTask */
/**
* @brief Function implementing the StabilizeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStabilizeTask */
__weak void StartStabilizeTask(void *argument)
{
  /* USER CODE BEGIN StartStabilizeTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartStabilizeTask */
}

/* USER CODE BEGIN Header_StartGroundStationTask */
/**
* @brief Function implementing the GroundStationTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGroundStationTask */
__weak void StartGroundStationTask(void *argument)
{
  /* USER CODE BEGIN StartGroundStationTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGroundStationTask */
}

/* USER CODE BEGIN Header_StartCommuCheckTask */
/**
* @brief Function implementing the CommuCheckTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommuCheckTask */
__weak void StartCommuCheckTask(void *argument)
{
  /* USER CODE BEGIN StartCommuCheckTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCommuCheckTask */
}

/* USER CODE BEGIN Header_StartNrfResponseTask */
/**
* @brief Function implementing the NrfResponseTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNrfResponseTask */
__weak void StartNrfResponseTask(void *argument)
{
  /* USER CODE BEGIN StartNrfResponseTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartNrfResponseTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

