#ifndef BSP_TRIGGER_H
#define BSP_TRIGGER_H

#include "main.h"
#include "tim.h"

#define trigger_set_left_pulse(pulse) \
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, (pulse))

#define trigger_set_right_pulse(pulse) \
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, (pulse))

void trigger_pwm_start(void);
void trigger_set_pulse(uint16_t pulse);

#endif
