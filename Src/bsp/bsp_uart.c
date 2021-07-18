/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_uart.c
 * @brief      this file contains rc data receive and processing function
 * @note       
 * @Version    V1.0.0
 * @Date       Jan-30-2018
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */
                                                                                                              
#include "string.h"
#include "stdlib.h"
#include "bsp_uart.h"
#include "usart.h"
#include "main.h"
#include "crc8_crc16.h"

uint8_t dbus_buf[DBUS_BUFLEN];
uint8_t refree_buf[REFREE_MAX_LEN];
RC_ctrl_t rc;


/**
  * @brief      enable global uart it and do not use DMA transfer done it
  * @param[in]  huart: uart IRQHandler id
  * @param[in]  pData: receive buff 
  * @param[in]  Size:  buff size
  * @retval     set success or fail
  */
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
  uint32_t tmp1 = 0;

  tmp1 = huart->RxState;
	
	if (tmp1 == HAL_UART_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;

		/* Enable the DMA Stream */
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

		/* 
		 * Enable the DMA transfer for the receiver request by setting the DMAR bit
		 * in the UART CR3 register 
		 */
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

/**
  * @brief      returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param[in]  dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
  *             to 7 to select the DMA Stream.
  * @retval     The number of remaining data units in the current DMAy Streamx transfer.
  */
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  /* Return the number of remaining data units for DMAy Streamx */
  return ((uint16_t)(dma_stream->NDTR));
}



/**
  * @brief       handle received rc data
  * @param[out]  rc:   structure to save handled rc data
  * @param[in]   buff: the buff which saved raw rc data
  * @retval 
  */
void rc_callback_handler(RC_ctrl_t *rc_ctrl, uint8_t *sbus_buf)
{
	
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
		if(rc_ctrl->rc.ch[0] > 1024 + 660 || rc_ctrl->rc.ch[0] < 1024 - 660){
			rc_ctrl->rc.ch[0] = 1024;
			return;
		}
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1	
		if(rc_ctrl->rc.ch[1] > 1024 + 660 || rc_ctrl->rc.ch[1] < 1024 - 660){
			rc_ctrl->rc.ch[1] = 1024;
			return;
		}
		rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
		if(rc_ctrl->rc.ch[2] > 1024 + 660 || rc_ctrl->rc.ch[2] < 1024 - 660){
			rc_ctrl->rc.ch[2] = 1024;
			return;
		}
		rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

// 0x00__
ext_game_status_t game_status;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP;

// 0x01__
ext_event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action;
ext_referee_warning_t referee_warning;
ext_dart_remaining_time_t dart_remaining_time;

// 0x02__
ext_game_robot_status_t robot_state;
ext_power_heat_data_t power_heat_data;
ext_game_robot_pos_t game_robot_pos;
ext_buff_t robot_buff;
aerial_robot_energy_t robot_energy;
ext_robot_hurt_t robot_hurt;
ext_shoot_data_t shoot_data;
ext_bullet_remaining_t bullet_remaining;

// 0x03__
ext_student_interactive_data_t student_interactive_data;

uint16_t hit_count = 0;

/**
  * @brief       handle received rc data
  * @param[out]  rc:   structure to save handled rc data
  * @param[in]   buff: the buff which saved raw rc data
  * @retval 
  */
void refree_buffer_handler(uint8_t *buf, uint16_t len){
	
	for(uint8_t i = 0; i <= len - 5; ++i){
		
		// Find the header
		if(buf[i] == 0xA5){
			// Not correct header
			if(!verify_CRC8_check_sum(buf + i, 5)){
				continue;
			}
			
			// Get length
			uint16_t data_length = (uint16_t)buf[i + 2] << 8 | buf[i + 1];
			
			// Ensure all data is in buffer
			if(i + data_length > len){
				continue;
			}
			
			// Verify CRC16 of whole package
			if(!verify_CRC16_check_sum(buf + i, 9 + data_length)){
				continue;
			}
			
			// Get CMD_ID
			uint16_t cmd_id;
			memcpy((void*)&cmd_id, buf + i + 5, sizeof(uint16_t));
			
			switch(cmd_id){
				
				// 0x00__
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_status, buf + i + 7, sizeof(game_status));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, buf + i + 7, sizeof(game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP, buf + i + 7, sizeof(game_robot_HP));
        }
        break;

				// 0x01__
        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&field_event, buf + i + 7, sizeof(field_event));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action, buf + i + 7, sizeof(supply_projectile_action));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning, buf + i + 7, sizeof(referee_warning));
        }
        break;
				case DART_REMAINING_TIME_CMD_ID:{
						memcpy(&dart_remaining_time, buf + i + 7, sizeof(dart_remaining_time));
				}
				break;
				
				// 0x02__
        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, buf + i + 7, sizeof(robot_state));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data, buf + i + 7, sizeof(power_heat_data));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos, buf + i + 7, sizeof(game_robot_pos));
        }
        break;
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&robot_buff, buf + i + 7, sizeof(robot_buff));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&robot_energy, buf + i + 7, sizeof(robot_energy));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt, buf + i + 7, sizeof(robot_hurt));
						if(robot_hurt.hurt_type == 0){
							hit_count++;
						}
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data, buf + i + 7, sizeof(shoot_data));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining, buf + i + 7, sizeof(bullet_remaining));
        }
        break;
				
				// 0x03__
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&student_interactive_data, buf + i + 7, sizeof(student_interactive_data));
        }
        break;
        default:
        {
            break;
        }
			}
		}
	}
}


/**
  * @brief      clear idle it flag after uart receive a frame data
  * @param[in]  huart: uart IRQHandler id
  * @retval  
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	/* clear idle it flag avoid idle interrupt all the time */
	__HAL_UART_CLEAR_IDLEFLAG(huart);

	/* handle received data in idle interrupt */
	if (huart == &DBUS_HUART)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* handle dbus data dbus_buf from DMA */
		if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
		{
			rc_callback_handler(&rc, dbus_buf);	
		}
		
		/* restart dma transmission */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
	
	/* handle received data in idle interrupt */
	else if (huart == &huart6)
	{
		/* clear DMA transfer complete flag */
		// __HAL_DMA_DISABLE(huart->hdmarx);

		
		volatile uint32_t temp;
		temp = huart1.Instance->SR;
		temp = huart1.Instance->DR;
		
		HAL_UART_DMAStop(huart);
		
		/* handle refree data refree_buf from DMA */
		refree_buffer_handler(refree_buf, REFREE_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance));	
		
		/* restart dma transmission */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, REFREE_MAX_LEN);
		// __HAL_DMA_ENABLE(huart->hdmarx);
		
		HAL_UART_Receive_DMA(huart, refree_buf, REFREE_MAX_LEN);//Restart DMA
	}
}

/**
  * @brief      callback this function when uart interrupt 
  * @param[in]  huart: uart IRQHandler id
  * @retval  
  */
void uart_receive_handler(UART_HandleTypeDef *huart)
{  
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && 
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}

/**
  * @brief   initialize dbus uart device 
  * @param   
  * @retval  
  */
void dbus_uart_init(void)
{
	/* open uart idle it */
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);

	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}

void refree_uart_init(void){
	/* open uart idle it 
	__HAL_UART_CLEAR_IDLEFLAG(&REFREE_HUART);
	__HAL_UART_ENABLE_IT(&REFREE_HUART, UART_IT_IDLE);
	*/
	__HAL_UART_ENABLE_IT(&REFREE_HUART, UART_IT_IDLE);//??idle??
	HAL_UART_Receive_DMA(&REFREE_HUART, refree_buf, REFREE_MAX_LEN);//??DMA??,????rx_buffer????
	
	//uart_receive_dma_no_it(&REFREE_HUART, refree_buf, REFREE_MAX_LEN);	
}

