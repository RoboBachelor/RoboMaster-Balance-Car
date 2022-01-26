/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             只控制80w功率，主要通过控制电机电流设定值,如果限制功率是40w，减少
  *             JUDGE_TOTAL_CURRENT_LIMIT和POWER_CURRENT_LIMIT的值，还有底盘最大速度
  *             (包括max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "bsp_uart.h"

#define POWER_LIMIT         30.0f
#define WARNING_POWER       20.0f   
float WARNING_POWER_BUFF = 80.0f;

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    16384.f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      8000.0f
#define POWER_TOTAL_CURRENT_LIMIT       8000.0f

// Power limit
extern ext_power_heat_data_t power_heat_data;

/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */
float chassis_power_control(float current_set)
{
    float chassis_power = 0.0f;
    float chassis_power_buffer = 0.0f;
    float total_current_limit = 0.0f;
    float total_current = 0.0f;

		// Enable power limit?
		// TODO: Handle refree offline!
	
    if(1)
    {
				chassis_power = power_heat_data.chassis_power;
				chassis_power_buffer = power_heat_data.chassis_power_buffer;
			
			
        // power > 80w and buffer < 60j, because buffer < 60 means power has been more than 80w
        //功率超过80w 和缓冲能量小于60j,因为缓冲能量小于60意味着功率超过80w
        if(chassis_power_buffer < WARNING_POWER_BUFF)
        {
            float power_scale;
            if(chassis_power_buffer > 5.0f)
            {
                //scale down WARNING_POWER_BUFF
                //缩小WARNING_POWER_BUFF
                power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
            }
            else
            {
                //only left 10% of WARNING_POWER_BUFF
                power_scale = 8.0f / WARNING_POWER_BUFF;
            }
            //scale down
            //缩小
            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
        }
        else
        {
            //power > WARNING_POWER
            //功率大于WARNING_POWER
            if(chassis_power > WARNING_POWER)
            {
                float power_scale;
                //power < 80w
                //功率小于80w
                if(chassis_power < POWER_LIMIT)
                {
                    //scale down
                    //缩小
                    power_scale = (POWER_LIMIT - chassis_power) / (POWER_LIMIT - WARNING_POWER);
                    
                }
                //power > 80w
                //功率大于80w
                else
                {
                    power_scale = 0.0f;
                }
                
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
            }
            //power < WARNING_POWER
            //功率小于WARNING_POWER
            else
            {
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
            }
        }
    }

    //total_current_limit as max current
		if(current_set > total_current_limit)
			return total_current_limit;
		if(current_set < -total_current_limit)
			return -total_current_limit;
		return current_set;
		
}

void set_waring_buf_value(float value){
	WARNING_POWER_BUFF = value;
}
