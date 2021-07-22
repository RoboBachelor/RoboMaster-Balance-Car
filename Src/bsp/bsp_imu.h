/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_imu.h
 * @brief      this file contains the common defines and functions prototypes for 
 *             the bsp_imu.c driver
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __MPU_H__
#define __MPU_H__

#include "mytype.h"
#include "FusionTypes.h"

#define MPU_DELAY(x) HAL_Delay(x)

#define IST8310
#define MPU_HSPI hspi5
#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

#define MPU_IST_DMA_BUF_LEN 21

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	int16_t temp;

	int16_t gx;
	int16_t gy;
	int16_t gz;
	
	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
} mpu_data_t;

typedef struct
{
	
	FusionVector3 gyro;
	
	FusionVector3 accel;

	float gimbal_yaw_gyro;
	
	int16_t mx;
	int16_t my;
	int16_t mz;
	
	FusionEulerAngles eulerAngles;

	float vx;
	float vy;
	float vz;

	float temp;
} imu_t;

extern mpu_data_t mpu_data;
extern imu_t      imu;

uint8_t mpu_device_init(void);
void mpu_offset_call(void);

void imu_spi_get_data(void);
void imu_buff_handler(uint8_t* mpu_buff, uint8_t* ist_buf);

uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len);
uint8_t mpu_read_byte(uint8_t const reg);
void ist8310_get_data(uint8_t* buff);


/* --- DMA SPI5 --- */
void SPI5_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
void SPI5_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);

#endif


