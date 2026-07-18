/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
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
#include "CanBus_Task.h"
#include "CapInteract.h"
#include "ControlTask.h"
#include "fdcan.h"
#include "RcTask.h"
#include "UI.h"
#include "usb_device.h"
#include "Vision_interact.h"
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
/* Definitions for Judge_Task */
osThreadId_t Judge_TaskHandle;
const osThreadAttr_t Judge_Task_attributes = {
  .name = "Judge_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for Init_Task */
osThreadId_t Init_TaskHandle;
const osThreadAttr_t Init_Task_attributes = {
  .name = "Init_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for Ctrl_Task */
osThreadId_t Ctrl_TaskHandle;
const osThreadAttr_t Ctrl_Task_attributes = {
  .name = "Ctrl_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for Interact_Task */
osThreadId_t Interact_TaskHandle;
const osThreadAttr_t Interact_Task_attributes = {
  .name = "Interact_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for Reset_Task */
osThreadId_t Reset_TaskHandle;
const osThreadAttr_t Reset_Task_attributes = {
  .name = "Reset_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void JudgeTask(void *argument);
void InitTask(void *argument);
void CtrlTask(void *argument);
void InteractTask(void *argument);
void ResetTask(void *argument);

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
  /* creation of Judge_Task */
  Judge_TaskHandle = osThreadNew(JudgeTask, NULL, &Judge_Task_attributes);

  /* creation of Init_Task */
  Init_TaskHandle = osThreadNew(InitTask, NULL, &Init_Task_attributes);

  /* creation of Ctrl_Task */
  Ctrl_TaskHandle = osThreadNew(CtrlTask, NULL, &Ctrl_Task_attributes);

  /* creation of Interact_Task */
  Interact_TaskHandle = osThreadNew(InteractTask, NULL, &Interact_Task_attributes);

  /* creation of Reset_Task */
  Reset_TaskHandle = osThreadNew(ResetTask, NULL, &Reset_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_JudgeTask */
/**
  * @brief  Function implementing the Judge_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_JudgeTask */
void JudgeTask(void *argument)
{
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN JudgeTask */
  /* Infinite loop */
  for(;;)
  {
    // UI_ALL_Task();
    MiniPC_SendVisionFrameInfo(&MiniPC_SendPacket);
    osDelay(1);
  }
  /* USER CODE END JudgeTask */
}

/* USER CODE BEGIN Header_InitTask */
/**
* @brief Function implementing the Init_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InitTask */
void InitTask(void *argument)
{
  /* USER CODE BEGIN InitTask */
  /* Infinite loop */
  for(;;)
  {
    TotalInitTask();
    osDelay(1);
    osThreadTerminate(Init_TaskHandle);//结束初始化
  }
  /* USER CODE END InitTask */
}

/* USER CODE BEGIN Header_CtrlTask */
/**
* @brief Function implementing the Ctrl_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CtrlTask */
void CtrlTask(void *argument)
{
  /* USER CODE BEGIN CtrlTask */
  /* Infinite loop */
  for(;;)
  {
    TotalControl();

    osDelay(1);
  }
  /* USER CODE END CtrlTask */
}

/* USER CODE BEGIN Header_InteractTask */
/**
* @brief Function implementing the Interact_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InteractTask */
void InteractTask(void *argument)
{
  /* USER CODE BEGIN InteractTask */
  /* Infinite loop */
  for(;;)
  {
    //

    MotorCanSendCurrent();
    // static uint8_t _cnt=0;
    // if(_cnt++ > 10) {
      PowerSend();
    //   _cnt=0;
    // }
    // set_DM_J4310_motor_State(&hfdcan3, xMotorParam[YAW].ID, save_zero_point);
    // set_DM_J4310_motor_State(&hfdcan3, xMotorParam[PITCH].ID, save_zero_point);
    // set_DM_J4310_motor_State(&hfdcan3, xMotorParam[YAW].ID, enable);
    // set_DM_J4310_MIT_info(&hfdcan3, xMotorParam[YAW].ID, 0.0f, 0.0f, 0.0f, 0.0f, 5);
    //

    osDelay(1);
  }
  /* USER CODE END InteractTask */
}

/* USER CODE BEGIN Header_ResetTask */
/**
* @brief Function implementing the Reset_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ResetTask */
void ResetTask(void *argument)
{
  /* USER CODE BEGIN ResetTask */
  /* Infinite loop */
  for(;;)
  {
    MotorWatchdogMonitor();
    UI_ALL_Task();
    if (KEY_CTRL+KEY_R) {
      ALL_reset(0);
    }

    osDelay(1);
  }
  /* USER CODE END ResetTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

