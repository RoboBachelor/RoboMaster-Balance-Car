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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LED_Blink_Task */
osThreadId_t LED_Blink_TaskHandle;
const osThreadAttr_t LED_Blink_Task_attributes = {
  .name = "LED_Blink_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for INS_Update_Task */
osThreadId_t INS_Update_TaskHandle;
const osThreadAttr_t INS_Update_Task_attributes = {
  .name = "INS_Update_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Gimbal_Control */
osThreadId_t Gimbal_ControlHandle;
const osThreadAttr_t Gimbal_Control_attributes = {
  .name = "Gimbal_Control",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LCD_Refresh */
osThreadId_t LCD_RefreshHandle;
const osThreadAttr_t LCD_Refresh_attributes = {
  .name = "LCD_Refresh",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void LED_Task(void *argument);
extern void INS_Task(void *argument);
extern void Gimbal_Task(void *argument);
extern void LCD_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* creation of LED_Blink_Task */
  LED_Blink_TaskHandle = osThreadNew(LED_Task, NULL, &LED_Blink_Task_attributes);

  /* creation of INS_Update_Task */
  INS_Update_TaskHandle = osThreadNew(INS_Task, NULL, &INS_Update_Task_attributes);

  /* creation of Gimbal_Control */
  Gimbal_ControlHandle = osThreadNew(Gimbal_Task, NULL, &Gimbal_Control_attributes);

  /* creation of LCD_Refresh */
  LCD_RefreshHandle = osThreadNew(LCD_Task, NULL, &LCD_Refresh_attributes);

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
void StartDefaultTask(void *argument)
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
    osDelay(10);
		//int_status = mpu_read_byte(0x3A);
		//int_en = mpu_read_byte(0x38);

//		vTaskGetRunTimeStats(buf);
//		sprintf(buf + strlen(buf), "xTaskGetTickCount()=%d\n", xTaskGetTickCount());

		uint8_t wza_buf[200];
		srand(xTaskGetTickCount());
		for(uint8_t i = 0; i < 49; ++i){
			wza_buf[i] = rand() % 256;
		}

		CDC_Transmit_FS((uint8_t*) wza_buf, 49);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
