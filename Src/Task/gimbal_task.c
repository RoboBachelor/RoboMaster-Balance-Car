#include "gimbal_task.h"

extern RC_ctrl_t rc;
extern imu_t imu;
extern motor_measure_t motor_measure[];

gimbal_motor_t 	yaw_motor, 
								pitch_motor;

uint16_t trigger_pulse = 1000;

uint8_t counter_div_4;

void Gimbal_Task(void const * argument){
	
  // Initialise the xLastWakeTime variable with the current time.
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	//Init Yaw and pitch motors PID parameters
	float yaw_gyro_pid[3] = {500.0f, 0.003f, 1000.0f};
	float pitch_gyro_pid[3] = {100.f, 2.2f, 400.f};
	
	float yaw_angle_pid[3] = {1, 0, 0};
	float pitch_angle_pid[3] = {4.5, 0.005, 0};
	
	//Set motor measure pointers point to motor_measure[] in bsp_can.c
	yaw_motor.motor_measure = motor_measure + 4;
	pitch_motor.motor_measure = motor_measure + 5;
	
	//Init gyro (angle speed) PIDs
	PID_Init(&yaw_motor.gyro_pid, PID_POSITION, yaw_gyro_pid, 20000, 10000);
	PID_Init(&pitch_motor.gyro_pid, PID_POSITION, pitch_gyro_pid, 20000, 10000);
	
	//Init angle (ecd or imu) PIDs
	PID_Init(&yaw_motor.angle_pid, PID_POSITION, yaw_angle_pid, 800, 400);
	PID_Init(&pitch_motor.angle_pid, PID_POSITION, pitch_angle_pid, 800, 400);
	
	//Init current out filter
	filter_init(&yaw_motor.current_filter, 30.f);
	filter_init(&pitch_motor.current_filter, 45.f);
	
	filter_init(&pitch_motor.angle_filter, 45.f);
	
	while(1){
		
		/* --- [1] Feedback from sensors --- */
		
		yaw_motor.motor_gyro		= imu.gimbal_yaw_gyro;
		pitch_motor.motor_gyro	= imu.gyro.axis.x;
		
		//pitch_motor.motor_gyro = filter_calc(&pitch_motor.angle_filter, pitch_motor.motor_gyro);
		
		/* --- [2] Set mode and values from RC --- */
		
		yaw_motor.motor_gyro_set = - rc.rc.ch[0] / 3.f;
		pitch_motor.motor_gyro_set = rc.rc.ch[1] / 3.f;
		
		yaw_motor.angle_set = -rc.rc.ch[0] / 10.f;
		pitch_motor.angle_set = rc.rc.ch[1] / 10.f - 10.f;
		
		/* --- [3] Calculate control variables --- */
		
		filter_calc(&pitch_motor.angle_filter, imu.eulerAngles.angle.pitch);
		
		
		// Angle loop PID
		if(++counter_div_4 >= 4){
			counter_div_4 = 0;
		}
			yaw_motor.motor_gyro_set = gimbal_PID_calc(&yaw_motor.angle_pid, imu.eulerAngles.angle.yaw, yaw_motor.angle_set, yaw_motor.motor_gyro);
			pitch_motor.motor_gyro_set = gimbal_PID_calc(&pitch_motor.angle_pid, pitch_motor.angle_filter.out, pitch_motor.angle_set, pitch_motor.motor_gyro);
			
		
		// Angle speed loop PID
		yaw_motor.current_set = PID_Calc(&yaw_motor.gyro_pid, yaw_motor.motor_gyro, yaw_motor.motor_gyro_set);
		pitch_motor.current_set = PID_Calc(&pitch_motor.gyro_pid, pitch_motor.motor_gyro, pitch_motor.motor_gyro_set);
		
		// Update filter
		
		yaw_motor.current_set = filter_calc(&yaw_motor.current_filter, yaw_motor.current_set);
		pitch_motor.current_set = filter_calc(&pitch_motor.current_filter, pitch_motor.current_set);
		
		/* --- [4] Send current values via CAN --- */
		
		yaw_motor.given_current = -(int16_t)yaw_motor.current_set;
		pitch_motor.given_current = (int16_t)pitch_motor.current_set;
		
		CAN_cmd_gimbal(yaw_motor.given_current, pitch_motor.given_current, 0, 0);
		
		trigger_pulse = 1250 + rc.rc.ch[3] / 660.0f * 250.0f;
		
		trigger_set_pulse(trigger_pulse);
		
		/* Wait for the next cycle */
    vTaskDelayUntil(&xLastWakeTime, 1);
	}
}


int16_t d1, d2, d3;

void usb_cdc_unpackage(uint8_t* Buf, uint32_t *Len){
	char command[10];
	sscanf((char*)Buf, "%s", command);
	if(strcmp(command, "DIST") == 0){
		sscanf((char*)Buf + 4, "%hd%hd%hd", &d1, &d2, &d3);
	}
}
