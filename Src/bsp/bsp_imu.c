/**
 ***************************************(C) COPYRIGHT 2021 XJTLU Team GMaster************************
 * @file       bsp_imu.c
 * @brief      mpu6500 module driver, configurate MPU6500 and Read the Accelerator
 *             and Gyrometer data using SPI interface      
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018
 * @Version    V2.0.0
 * @Date       Jul-7-2021   
 ***************************************(C) COPYRIGHT 2021 XJTLU Team GMasterI***********************
 */

#include "stm32f4xx_hal.h"
#include "ist8310_reg.h" 
#include "mpu6500_reg.h"
#include "spi.h"
#include "bsp_imu.h"
#include "imu_task.h"
#include "cmsis_os.h"

#define Kp 2.0f                                              /* 
                                                              * proportional gain governs rate of 
                                                              * convergence to accelerometer/magnetometer 
																															*/
#define Ki 0.01f                                             /* 
                                                              * integral gain governs rate of 
                                                              * convergence of gyroscope biases 
																															*/

volatile float        exInt, eyInt, ezInt;                   /* error integral */
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;
static uint8_t        tx, rx;
static uint8_t        tx_buff[14] = { 0xff };
uint8_t               mpu_buff[14];                          /* buffer to save imu raw data */
uint8_t               ist_buff[6];                           /* buffer to save IST8310 raw data */
mpu_data_t            mpu_data;
imu_t                 imu={0};
uint8_t ist_error_code;

uint8_t mpu_ist_dma_tx_buff[MPU_IST_DMA_BUF_LEN];
uint8_t mpu_ist_dma_rx_buff[MPU_IST_DMA_BUF_LEN];

extern SPI_HandleTypeDef hspi5;

extern TaskHandle_t INSHandle;

/**
  * @brief  write a byte of data to specified register
  * @param  reg:  the address of register to be written
  *         data: data to be written
  * @retval 
  * @usage  call in ist_reg_write_by_mpu(),         
  *                 ist_reg_read_by_mpu(), 
  *                 mpu_master_i2c_auto_read_config(), 
  *                 ist8310_init(), 
  *                 mpu_set_gyro_fsr(),             
  *                 mpu_set_accel_fsr(), 
  *                 mpu_device_init() function
  */
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
    MPU_NSS_LOW;
    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    tx = data;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return 0;
}

/**
  * @brief  read a byte of data from specified register
  * @param  reg: the address of register to be read
  * @retval 
  * @usage  call in ist_reg_read_by_mpu(),         
  *                 mpu_device_init() function
  */
uint8_t mpu_read_byte(uint8_t const reg)
{
    MPU_NSS_LOW;
    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return rx;
}

/**
  * @brief  read bytes of data from specified register
  * @param  reg: address from where data is to be written
  * @retval 
  * @usage  call in ist8310_get_data(),         
  *                 mpu_get_data(), 
  *                 mpu_offset_call() function
  */
uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)
{
    MPU_NSS_LOW;
    tx         = regAddr | 0x80;
    tx_buff[0] = tx;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
    MPU_NSS_HIGH;
    return 0;
}

/**
	* @brief  write IST8310 register through MPU6500's I2C master
  * @param  addr: the address to be written of IST8310's register
  *         data: data to be written
	* @retval   
  * @usage  call in ist8310_init() function
	*/
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    /* turn off slave 1 at first */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
    MPU_DELAY(2);
    /* turn on slave 1 with one byte transmitting */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* wait longer to ensure the data is transmitted from slave 1 */
    MPU_DELAY(10);
}

/**
	* @brief  write IST8310 register through MPU6500's I2C Master
	* @param  addr: the address to be read of IST8310's register
	* @retval 
  * @usage  call in ist8310_init() function
	*/
static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval;
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    MPU_DELAY(10);
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
    /* turn off slave4 after read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);
    return retval;
}

/**
	* @brief    initialize the MPU6500 I2C Slave 0 for I2C reading.
	* @param    device_address: slave device address, Address[6:0]
	* @retval   void
	* @note     
	*/
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /* 
	   * configure the device address of the IST8310 
     * use slave1, auto transmit single measure mode 
	   */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    MPU_DELAY(2);

    /* use slave0,auto read data */
    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    MPU_DELAY(2);

    /* every eight mpu6500 internal samples one i2c master read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    MPU_DELAY(2);
    /* enable slave 0 and 1 access delay */
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    MPU_DELAY(2);
    /* enable slave 1 auto transmit */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
		/* Wait 6ms (minimum waiting time for 16 times internal average setup) */
    MPU_DELAY(6); 
    /* enable slave 0 with data_num bytes reading */
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    MPU_DELAY(2);
}

/**
	* @brief  Initializes the IST8310 device
	* @param  
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t ist8310_init()
{
	  /* enable iic master mode */
    mpu_write_byte(MPU6500_USER_CTRL, 0x30);
    MPU_DELAY(10);
	  /* enable iic 400khz */
    mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d); 
    MPU_DELAY(10);

    /* turn on slave 1 for ist write and slave 4 to ist read */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);  
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    MPU_DELAY(10);

    /* IST8310_R_CONFB 0x01 = device rst */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    MPU_DELAY(10);
    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
        return 1;

		/* soft reset */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01); 
    MPU_DELAY(10);

		/* config as ready mode to access register */
    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00); 
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
        return 2;
    MPU_DELAY(10);

		/* normal state, no int */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
        return 3;
    MPU_DELAY(10);
		
    /* config low noise mode, x,y,z axis 16 time 1 avg */
    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
        return 4;
    MPU_DELAY(10);

    /* Set/Reset pulse duration setup,normal mode */
    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
        return 5;
    MPU_DELAY(10);

    /* turn off slave1 & slave 4 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);

    /* configure and turn on slave 0 */
    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    MPU_DELAY(100);
    return 0;
}


/**
	* @brief  get the data of IST8310
  * @param  buff: the buffer to save the data of IST8310
	* @retval 
  * @usage  call in mpu_get_data() function
	*/
void ist8310_get_data(uint8_t* buff)
{
    mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6); 
}


/**
	* @brief  get the data of imu
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void imu_spi_get_data()
{
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
    ist8310_get_data(ist_buff);
}

void imu_buff_handler(uint8_t* mpu_buff, uint8_t* ist_buf){
    mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
    mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
    mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

    mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);

    memcpy(&mpu_data.mx, ist_buf, 6);
    memcpy(&imu.mx, &mpu_data.mx, 3 * sizeof(int16_t));
	
    imu.temp = 21 + mpu_data.temp / 333.87f;
	
	  /* 1000dps -> deg/s */
	imu.gyro.axis.x = - mpu_data.gz / 32.768f;
    imu.gyro.axis.y = - mpu_data.gx / 32.768f;
    imu.gyro.axis.z = mpu_data.gy / 32.768f;
		
	/* 8g -> m/s^2 */
	imu.accel.axis.x = - mpu_data.az / 4096.f * 9.80665f;
	imu.accel.axis.y = - mpu_data.ax / 4096.f * 9.80665f;
	imu.accel.axis.z = mpu_data.ay / 4096.f * 9.80665f;
}


/**
	* @brief  set imu 6500 gyroscope measure range
  * @param  fsr: range(0,±250dps;1,±500dps;2,±1000dps;3,±2000dps)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}


/**
	* @brief  set imu 6050/6500 accelerate measure range
  * @param  fsr: range(0,±2g;1,±4g;2,±8g;3,±16g)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3); 
}

uint8_t id;

/**
	* @brief  initialize imu mpu6500 and magnet meter ist3810
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
uint8_t mpu_device_init(void)
{
	MPU_DELAY(100);

	id                               = mpu_read_byte(MPU6500_WHO_AM_I);
	uint8_t MPU6500_Init_Data[10][2] = {{ MPU6500_PWR_MGMT_1, 0x80 },     /* Reset Device */ 
																			{ MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */ 
																			{ MPU6500_PWR_MGMT_2, 0x00 },     /* Enable Acc & Gyro */ 
																			{ MPU6500_CONFIG, 0x04 },         /* LPF 41Hz */ 
																			{ MPU6500_GYRO_CONFIG, 0x10 },    /* +-1000dps */ 
																			{ MPU6500_ACCEL_CONFIG, 0x10 },   /* +-8G */ 
																			{ MPU6500_ACCEL_CONFIG_2, 0x02 }, /* enable LowPassFilter  Set Acc LPF */ 
																			{ MPU6500_USER_CTRL, 0x20 },    	/* Enable AUX */ 
																			{ MPU6500_INT_ENABLE, 0x01},};		/* Enable INT */
	
	for (uint8_t i = 0; i < 10; i++)
	{
		mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		MPU_DELAY(2);
	}

	ist_error_code = ist8310_init();
	
	mpu_offset_call();
	
	/* Set the addresses of DMA Tx Buffer */
	mpu_ist_dma_tx_buff[0] = mpu_ist_dma_tx_buff[1] = MPU6500_ACCEL_XOUT_H | 0x80;
	
	for(uint8_t i = 0; i < MPU_IST_DMA_BUF_LEN; ++i){
		if(!mpu_ist_dma_tx_buff[i])
			mpu_ist_dma_tx_buff[i] = 0xff;
	}
	
	return 0;
}

/**
	* @brief  get the offset data of MPU6500
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_offset_call(void)
{
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	int i;
	for (i=0; i<300;i++)
	{
		mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

		mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
		mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
		mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];
	
		mpu_data.gx_offset += mpu_buff[8]  << 8 | mpu_buff[9];
		mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
		mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

		MPU_DELAY(5);
	}
	mpu_data.ax_offset = mpu_data.ax_offset / 300;
	mpu_data.ay_offset = mpu_data.ay_offset / 300;
	mpu_data.az_offset = mpu_data.az_offset / 300;
	mpu_data.gx_offset = mpu_data.gx_offset / 300;
	mpu_data.gy_offset = mpu_data.gy_offset / 300;
	mpu_data.gz_offset = mpu_data.gz_offset / 300;
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == MPU6500_INT_Pin){
		if(HAL_SPI_GetState(&hspi5) == HAL_SPI_STATE_READY){	
			if(INSHandle == NULL)
				return;
			MPU_NSS_LOW;
			HAL_SPI_TransmitReceive_DMA(&hspi5, mpu_ist_dma_tx_buff, mpu_ist_dma_rx_buff, MPU_IST_DMA_BUF_LEN);
		}
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi == &hspi5){
		MPU_NSS_HIGH;
		Wake_up_IMU_Task();
	}
}

