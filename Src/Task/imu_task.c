#include "main.h"
#include "cmsis_os.h"
#include "bsp_imu.h"
#include "usbd_cdc_if.h"
#include "FusionAhrs.h"
#include "imu_task.h"

extern imu_t imu;	 // In bsp_imu.c
extern char buf[]; // In main.c

extern SPI_HandleTypeDef hspi5;

extern uint8_t mpu_ist_dma_tx_buff[];
extern uint8_t mpu_ist_dma_rx_buff[];

TaskHandle_t INSHandle;
int mpu6500_cnt = 0;

void INS_Task(void const * argument){
	
	FusionAhrs fusionAhrs;
	
	/* Initialise AHRS algorithm */
	FusionAhrsInitialise(&fusionAhrs, 0.5f);
	
	/* Get Handle for IMU Task */
	INSHandle = xTaskGetCurrentTaskHandle();
	
	while(1){
		
		/* Waiting notify from SPI DMA Complete Interrupt */
		while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS){};
		
		mpu6500_cnt++;
			
		/* Unpackage the buffer read form SPI DMA */
		imu_buff_handler(mpu_ist_dma_rx_buff + 1, mpu_ist_dma_rx_buff + 15);
		
		/* Update AHRS quaternions */
    FusionAhrsUpdateWithoutMagnetometer(&fusionAhrs, imu.gyro, imu.accel, 0.001f);

    /* Get eualr angles from quaternions */
		imu.eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));
		
		/* Get angle speed for gimbal from gyro_z and gyro_y */
		imu.gimbal_yaw_gyro = cos(imu.eulerAngles.angle.pitch / 57.29577958f) * imu.gyro.axis.z + sin(imu.eulerAngles.angle.pitch / 57.29577958f) * imu.gyro.axis.y;
		
		/* Send eular angles in Serial-Studio format */
		//sprintf(buf, "/*%.5f,%.5f,%.5f*/", imu.eulerAngles.angle.yaw, imu.eulerAngles.angle.roll, imu.eulerAngles.angle.pitch);
		//if(mpu6500_cnt % 20 == 0){
		//	CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
		//}
		
	}
	
}

void Wake_up_IMU_Task(){
	/* Wake up IMU_Task */
	if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
		return;
	if (INSHandle == NULL)
		return;
	
  static BaseType_t xHigherPriorityTaskWoken;
  vTaskNotifyGiveFromISR(INSHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
