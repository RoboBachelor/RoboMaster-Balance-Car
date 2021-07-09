/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "bsp_imu.h"
#include "usbd_cdc_if.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//configGENERATE_RUN_TIME_STATS configUSE_STATS_FORMATTING_FUNCTIONS configUSE_TRACE_FACILITY
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
		uint8_t int_en = 0;
		uint8_t int_status = 0;
extern char buf[300]; // In main.c

extern SPI_HandleTypeDef hspi5;
extern uint8_t               ist_buff[6];                           /* buffer to save IST8310 raw data */
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId LED_Blink_TaskHandle;
osThreadId INS_Update_TaskHandle;
osThreadId Gimbal_ControlHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
extern void LED_Task(void const * argument);
extern void INS_Task(void const * argument);
extern void Gimbal_Task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return xTaskGetTickCount();//0;
}
/* USER CODE END 1 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LED_Blink_Task */
  osThreadDef(LED_Blink_Task, LED_Task, osPriorityBelowNormal, 0, 128);
  LED_Blink_TaskHandle = osThreadCreate(osThread(LED_Blink_Task), NULL);

  /* definition and creation of INS_Update_Task */
  osThreadDef(INS_Update_Task, INS_Task, osPriorityRealtime, 0, 1024);
  INS_Update_TaskHandle = osThreadCreate(osThread(INS_Update_Task), NULL);

  /* definition and creation of Gimbal_Control */
  osThreadDef(Gimbal_Control, Gimbal_Task, osPriorityHigh, 0, 512);
  Gimbal_ControlHandle = osThreadCreate(osThread(Gimbal_Control), NULL);

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
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
	
//	  UBaseType_t   ArraySize;
//    TaskStatus_t  *StatusArray;
//    uint8_t       x;
//		volatile int32_t FreeRTOSRunTimeTicks;
//	
//    ArraySize = uxTaskGetNumberOfTasks(); 
//    StatusArray = pvPortMalloc(ArraySize*sizeof(TaskStatus_t));
//    while(1)
//    {
//				FreeRTOSRunTimeTicks = xTaskGetTickCount();
//			
//        if(StatusArray != NULL){ 

//            ArraySize = uxTaskGetSystemState( (TaskStatus_t *) StatusArray,
//                                              (UBaseType_t   ) ArraySize,
//                                              (uint32_t *    ) &FreeRTOSRunTimeTicks );

//            sprintf(buf, "TaskName\t\tPriority\tTaskNumber\tMinStk\tRunTimeCounter\n");
//            for(x = 0;x<ArraySize;x++){
//                sprintf(buf + strlen(buf),"%s\t\t%d\t%d\t%d\t%d\r\n",
//                        StatusArray[x].pcTaskName,
//                        (int)StatusArray[x].uxCurrentPriority,
//                        (int)StatusArray[x].xTaskNumber,
//                        (int)StatusArray[x].usStackHighWaterMark,
//                        (int)StatusArray[x].ulRunTimeCounter);
//            }
//            //RTT_printf(0,"\n\n");
//        }
//				CDC_Transmit_FS((uint8_t*) buf, strlen(buf));
//        vTaskDelay(1000);
//    }
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
		//int_status = mpu_read_byte(0x3A);
		//int_en = mpu_read_byte(0x38);

		vTaskGetRunTimeStats(buf);
		sprintf(buf + strlen(buf), "xTaskGetTickCount()=%d\n", xTaskGetTickCount());
		CDC_Transmit_FS((uint8_t*) buf, strlen(buf));
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
