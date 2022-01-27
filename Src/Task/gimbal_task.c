#include "gimbal_task.h"

extern RC_ctrl_t rc;
extern imu_t imu;
extern motor_measure_t motor_measure[];

Chassis_Motor_t motor_left, motor_right;

PidTypeDef balance_angle_pid;
PidTypeDef car_speed_pid;
PidTypeDef yaw_angle_pid;


float car_speed;
float car_speed_set;

float balance_angle_fdb;
float balance_angle_set;
float balance_angle_err_diff;

float current_set;
float current_diff_set;

float yaw_angle_fdb;
float yaw_angle_set = 0.f;
float yaw_angle_err_diff;

float gain_right_current = 1.6f;

balacne_car_mode_e car_mode = CAR_MODE_ZERO;

void Gimbal_Task(void const *argument) {
	// Initialise the xLastWakeTime variable with the current time.
	TickType_t xLastWakeTime = xTaskGetTickCount();

	
    motor_left.chassis_motor_measure = motor_measure + 0;
    motor_right.chassis_motor_measure = motor_measure + 1;


//    float balance_angle_pid_para[3] = {100 / 57 / 20 * 15000, 0,
//                                      3 / 57 / 20 * 15000};
	float balance_angle_pid_para[3] = {600, 0, 15};
    float car_speed_pid_para[3] = {3, 0.3, 8};
	float yaw_angle_pid_para[3] = {300, 0, 20};
	
    PID_Init(&balance_angle_pid, PID_POSITION, balance_angle_pid_para, 15000.0, 10000.0);	//, -1, 1e-3);
    PID_Init(&car_speed_pid, PID_POSITION, car_speed_pid_para, 15.0, 5.0);				//, 5, 1e-3);
    PID_Init(&yaw_angle_pid, PID_POSITION, yaw_angle_pid_para, 15000.0, 10000.0);	//, -1, 1e-3);

	while (1) {
		
		/* Get the feedback */
		motor_left.speed = motor_left.chassis_motor_measure->speed_rpm *
						   CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
		motor_right.speed = -motor_right.chassis_motor_measure->speed_rpm *
							CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
		
		car_speed = (motor_left.speed + motor_right.speed) / 2;
		
		balance_angle_fdb = imu.eulerAngles.angle.pitch;
		balance_angle_err_diff = -imu.gyro.axis.y;
		
		yaw_angle_fdb = imu.eulerAngles.angle.yaw;
		yaw_angle_err_diff = -imu.gyro.axis.z;
		yaw_angle_set -= rc.rc.ch[0] / 660.f / 5.f;
		
		/* Update the mode */
		if(switch_is_up(rc.rc.s[0]) || balance_angle_fdb > 30.f || balance_angle_fdb < -20.f){
			car_mode = CAR_MODE_ZERO;
		}
		else{
			car_mode = CAR_MODE_RUNNING;
		}
		
		
		
		/* Speed pid */
		static uint8_t cnt1 = 0;
		if (cnt1++ == 3) {
			car_speed_set = rc.rc.ch[1] / 200.f;
			balance_angle_set = 7.5 - PID_Calc(&car_speed_pid, car_speed, car_speed_set);
			cnt1 = 0;
		} 
		
		if (switch_is_mid(rc.rc.s[0])){
			balance_angle_set = 7.5f;
		}
		
		/* Balance Angle pid */
		current_set = gimbal_PID_calc(&balance_angle_pid, balance_angle_fdb, balance_angle_set, balance_angle_err_diff);
		
		/* Yaw Angle pid */
		current_diff_set = gimbal_PID_calc(&yaw_angle_pid, yaw_angle_fdb, yaw_angle_set, yaw_angle_err_diff);

		motor_left.give_current = current_set - current_diff_set;
		motor_right.give_current = -current_set - current_diff_set;
		motor_right.give_current *= gain_right_current;

		if(car_mode == CAR_MODE_ZERO){
			/* Send current via CAN bus */ 
			CAN_cmd_chassis(0, 0, 0, 0);
			
			/* Clear the PID */
			PID_clear(&balance_angle_pid);
			PID_clear(&car_speed_pid);
			
			yaw_angle_set = yaw_angle_fdb;
		}
		else{
			/* Send current via CAN bus */ 
			CAN_cmd_chassis(motor_left.give_current, motor_right.give_current, 0, 0);
		}
		
		
		/* Wait for the next cycle */
		vTaskDelayUntil(&xLastWakeTime, 1);
	}
}

void usb_cdc_unpackage(uint8_t *Buf, uint32_t *Len) {
	if (*Len < 15) {
		return;
	}
}


void LCD_Task(void const *argument) {

	/*
	 POINT_COLOR = WHITE;
	 LCD_Clear(BLACK);

	 LCD_ShowString(20, 20, "x=20,y=20");

	 LCD_ShowString(20, 160, "x=20,y=160");
	 LCD_ShowString(160, 20, "x=160,y=20");
	 LCD_ShowString(160, 160, "x=160,y=160");

	 vTaskDelay(10000);
	 */

	// Initialise the xLastWakeTime variable with the current time.
	TickType_t xLastWakeTime = xTaskGetTickCount();

	while (1) {

		/*
		 POINT_COLOR = BLACK;

		 for (uint8_t i = 0; i < 240; ++i) {
		 LCD_DrawPoint(i, 120 - plot_buf[i]);
		 LCD_DrawPoint(i, 120 - plot2_buf[i]);
		 }

		 uint8_t j = plot_start_index;
		 for (uint8_t i = 0; i < 240; ++i) {
		 plot_buf[i] = plot_data[j];
		 plot2_buf[i] = plot_data2[j];
		 POINT_COLOR = WHITE;
		 LCD_DrawPoint(i, 120 - plot_buf[i]);

		 POINT_COLOR = GREEN;
		 LCD_DrawPoint(i, 120 - plot2_buf[i]);
		 if (++j >= 240) j = 0;
		 }
		 */

		/* Wait for the next cycle */
		vTaskDelayUntil(&xLastWakeTime, 45);
	}
}
