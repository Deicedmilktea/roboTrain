/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

#include "can_receive.h"
#include "pid.h"

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
osThreadId Bullet_shootHandle;
osThreadId Bullet_rotateHandle;
osThreadId GimbalHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Bullet_shoot_Task(void const * argument);
void Bullet_rotate_Task(void const * argument);
void Gimbal_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of Bullet_shoot */
  osThreadDef(Bullet_shoot, Bullet_shoot_Task, osPriorityNormal, 0, 128);
  Bullet_shootHandle = osThreadCreate(osThread(Bullet_shoot), NULL);

  /* definition and creation of Bullet_rotate */
  osThreadDef(Bullet_rotate, Bullet_rotate_Task, osPriorityNormal, 0, 128);
  Bullet_rotateHandle = osThreadCreate(osThread(Bullet_rotate), NULL);

  /* definition and creation of Gimbal */
  osThreadDef(Gimbal, Gimbal_Task, osPriorityNormal, 0, 128);
  GimbalHandle = osThreadCreate(osThread(Gimbal), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Bullet_shoot_Task */
/**
  * @brief  Function implementing the Bullet_shoot thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Bullet_shoot_Task */
void Bullet_shoot_Task(void const * argument)
{
  /* USER CODE BEGIN Bullet_shoot_Task */

    PID friction_pid;
		//static float curr_left_speed = 500;
		//static float curr_right_speed = 500;
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
    pid_init(friction_pid, 2, 3, 1);
		//CAN_cmd_friction(500, 500);
    CAN_cmd_friction(cur_friction_speed, cur_friction_speed);
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
		cur_friction_speed = motor[1].speed_rpm;
    cur_friction_speed = pid_calc(friction_pid, cur_friction_speed, tar_friction_speed);
    //curr_right_speed = pid_calc(friction_pid, motor[2], tar_right_speed);
    osDelay(1);
  }
  /* USER CODE END Bullet_shoot_Task */
}

/* USER CODE BEGIN Header_Bullet_rotate_Task */
/**
* @brief Function implementing the Bullet_rotate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Bullet_rotate_Task */
void Bullet_rotate_Task(void const * argument)
{
  /* USER CODE BEGIN Bullet_rotate_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Bullet_rotate_Task */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the Gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
void Gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
