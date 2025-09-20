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
#include "USART.h"
#include "Motor.h"
#include "Encoder.h"
#include "string.h"
#include "Controll.h"
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
extern EncoderTypeDef MotorEncoder[2];
extern PIDTypeDef MotorPID[2];
extern uint8_t g_recv_buf[11];
extern Pitch_PIDTypeDef Pit;
extern Yaw_PIDTypeDef Yaw;
extern Distance_PIDTypeDef Distance;
float roll_angle;
float pitch_angle;
float yaw_angle;
extern float Distance_Measure;
float yaw_Init;
int Task_Flag = 0;
extern uint8_t Usart_RxData;
extern float Yaw_Expect1;
struct Places_Type places_1[8];
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
void StartTask36GR3626SPDPID(void const *argument);
void StartTaskUART1Send(void const *argument);
void StartTaskALL_Ctrl(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
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
void MX_FREERTOS_Init(void)
{
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask36GR3626SPDPID, osPriorityIdle, 0, 512);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTaskUART1Send, osPriorityIdle, 0, 512);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, StartTaskALL_Ctrl, osPriorityIdle, 0, 512);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask36GR3626SPDPID */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask36GR3626SPDPID */
void StartTask36GR3626SPDPID(void const *argument)
{
  /* USER CODE BEGIN StartTask36GR3626SPDPID */
  EncoderInit();
  MotorPIDInit();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  /* Infinite loop */
  for (;;)
  {
    if (MotorPID[0].RPMSetting > 0)
    {
      HAL_GPIO_WritePin(MotorDirCH1_GPIO_Port, MotorDirCH1_Pin, GPIO_PIN_SET);
      MotorEncoder[0].MotorDir = 1.0;
    }
    else
    {
      HAL_GPIO_WritePin(MotorDirCH1_GPIO_Port, MotorDirCH1_Pin, GPIO_PIN_RESET);
      MotorEncoder[0].MotorDir = -1.0;
    }
    MotorPID[0].FunctionCalPID(&MotorPID[0], MotorEncoder[0].RotationSpeed);
    MotorCtrl(0, MotorPID[0].PWMOutput);

    if (MotorPID[1].RPMSetting > 0)
    {
      HAL_GPIO_WritePin(MotorDirCH2_GPIO_Port, MotorDirCH2_Pin, GPIO_PIN_RESET);
      MotorEncoder[1].MotorDir = 1.0;
    }
    else
    {
      HAL_GPIO_WritePin(MotorDirCH2_GPIO_Port, MotorDirCH2_Pin, GPIO_PIN_SET);
      MotorEncoder[1].MotorDir = -1.0;
    }
    MotorPID[1].FunctionCalPID(&MotorPID[1], MotorEncoder[1].RotationSpeed);
    MotorCtrl(1, MotorPID[1].PWMOutput);

    osDelay(10);
  }
  /* USER CODE END StartTask36GR3626SPDPID */
}

/* USER CODE BEGIN Header_StartTaskUART1Send */
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskUART1Send */
void StartTaskUART1Send(void const *argument)
{
  /* USER CODE BEGIN StartTaskUART1Send */
  HAL_UART_Transmit_IT(&huart1, "Ready!", 6);
  HAL_UART_Receive(&huart6, g_recv_buf, sizeof(g_recv_buf), 200);
  HAL_UART_Receive_IT(&huart1, &Usart_RxData, 1);
  const uint8_t send_cmd[3] = {0xA5, 0x95, 0x3A};
  /* Infinite loop */
  for (;;)
  {
    HAL_UART_Transmit_IT(&huart6, send_cmd, sizeof(send_cmd));
    HAL_UART_Receive(&huart6, g_recv_buf, sizeof(g_recv_buf), 200);
    int16_t pitch_raw = (g_recv_buf[4] << 8) | g_recv_buf[5]; // 16位原始值（示例字节位，需按实际调整）
    int16_t roll_raw = (g_recv_buf[6] << 8) | g_recv_buf[7];
    int16_t yaw_raw = (g_recv_buf[8] << 8) | g_recv_buf[9];
    // 实际角度 = 原始数据 / 100（保�?2位小数）
    roll_angle = (float)roll_raw / 100.0f;
    pitch_angle = (float)pitch_raw / 100.0f;
    yaw_angle = (float)yaw_raw / 100.0f;
    // printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
    //        Pit.RPMOutput, pitch_angle, Yaw.YawSetting, yaw_angle, Pit.KP, Yaw.KI);
    printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
           Pit.PitchSetting, pitch_angle, Pit.RPMOutput, Pit.KP, Yaw.YawSetting, yaw_angle);

    osDelay(10);
  }
  /* USER CODE END StartTaskUART1Send */
}

/* USER CODE BEGIN Header_StartTaskALL_Ctrl */
/**
 * @brief Function implementing the myTask04 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskALL_Ctrl */
void StartTaskALL_Ctrl(void const *argument)
{
  /* USER CODE BEGIN StartTaskALL_Ctrl */
  Pitch_PID_Init();
  Yaw_PID_Init();
  Distance_PID_Init();
  places_1[0].x = 0.5f;
  places_1[0].y = 0.0f;
  places_1[1].x = 0.5f;
  places_1[1].y = 0.3f;
  places_1[2].x = 0.0f;
  places_1[2].y = 0.3f;
  places_1[3].x = 0.0f;
  places_1[3].y = 0.0f;
  places_1[4].x = 0.0f;
  places_1[4].y = 0.3f;
  places_1[5].x = 0.5f;
  places_1[5].y = 0.3f;
  places_1[6].x = 0.5f;
  places_1[6].y = 0.0f;
  places_1[7].x = 0.0f;
  places_1[7].y = 0.0f;
  /* Infinite loop */
  for (;;)
  {
    // Task0 按下按键，记录Yaw初始�?
    if (Task_Flag == 0)
    {
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
      {
        osDelay(100);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
        {
          Yaw.YawSetting = yaw_angle;
          yaw_Init = yaw_angle;
          Task_Flag = 1;
          Distance.DistanceSetting = 1.0f; // 设置目标距离，单位米
        }
      }
      LED_Flash(100);
    }
    if (Task_Flag == 1)
    {
      MotorPID[0].RPMSetting = Yaw.FunctionCalPID(&Yaw, yaw_angle);
      MotorPID[1].RPMSetting = -Yaw.FunctionCalPID(&Yaw, yaw_angle);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
      osDelay(10);
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
      {
        osDelay(400);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
        {
          Task_Flag = 2;
        }
      }
    }
    if (Task_Flag == 2)
    {
      // MotorPID[0].RPMSetting = Distance.FunctionCalPID(&Distance, Distance_Measure) + Yaw.FunctionCalPID(&Yaw, yaw_angle);
      // MotorPID[1].RPMSetting = Distance.FunctionCalPID(&Distance, Distance_Measure) - Yaw.FunctionCalPID(&Yaw, yaw_angle);
      // osDelay(100);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      Xunhang(places_1);
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
      {
        osDelay(400);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
        {
          Task_Flag = 3;
        }
      }
    }
    if (Task_Flag == 3)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
      // if (pitch_angle > 1.0f || pitch_angle < -1.0f)
      {
        MotorPID[0].RPMSetting = -Pit.FunctionCalPID(&Pit, pitch_angle);
        MotorPID[1].RPMSetting = -Pit.FunctionCalPID(&Pit, pitch_angle);
      }
    }
    osDelay(10);
  }
  /* USER CODE END StartTaskALL_Ctrl */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
