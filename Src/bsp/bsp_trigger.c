#include "bsp_trigger.h"

void trigger_pwm_start(){
	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
}

void trigger_set_pulse(uint16_t pulse){
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, pulse);
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, pulse);
}

