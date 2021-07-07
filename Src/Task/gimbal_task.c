#include "gimbal_task.h"

extern RC_ctrl_t rc;
extern imu_t imu;

extern motor_measure_t motor_measure[];

gimbal_motor_t 	yaw_motor, 
								pitch_motor;

void Gimbal_Task(void const * argument){
	
  // Initialise the xLastWakeTime variable with the current time.
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	//Init Yaw and pitch motors PID parameters
	float yaw_gyro_pid[3] = {500.0f, 0.003f, 1000.0f};
	float pitch_gyro_pid[3] = {40.f, 0.005f, 80.f};
	
	float yaw_angle_pid[3] = {10, 0, 0};
	float pitch_angle_pid[3] = {10, 0, 0};
	
	//Set motor measure pointers point to motor_measure[] in bsp_can.c
	yaw_motor.motor_measure = motor_measure + 4;
	pitch_motor.motor_measure = motor_measure + 5;
	
	//Init gyro (angle speed) PIDs
	PID_Init(&yaw_motor.gyro_pid, PID_POSITION, yaw_gyro_pid, 20000, 10000);
	PID_Init(&pitch_motor.gyro_pid, PID_POSITION, pitch_gyro_pid, 20000, 10000);
	
	//Init angle (ecd or imu) PIDs
	PID_Init(&yaw_motor.angle_pid, PID_POSITION, yaw_angle_pid, 10000, 2000);
	PID_Init(&pitch_motor.angle_pid, PID_POSITION, pitch_angle_pid, 10000, 2000);

	while(1){
		
		/* --- [1] Feedback from sensors --- */
		
		yaw_motor.motor_gyro		= imu.gimbal_yaw_gyro;
		pitch_motor.motor_gyro	= imu.gyro.axis.x;
		
		/* --- [2] Set mode and values from RC --- */
		
		yaw_motor.motor_gyro_set = - rc.rc.ch[0] / 3.f;
		pitch_motor.motor_gyro_set = rc.rc.ch[1] / 3.f;
		
		/* --- [3] Calculate control variables --- */
		
		yaw_motor.current_set = PID_Calc(&yaw_motor.gyro_pid, yaw_motor.motor_gyro, yaw_motor.motor_gyro_set);
		pitch_motor.current_set = PID_Calc(&pitch_motor.gyro_pid, pitch_motor.motor_gyro, pitch_motor.motor_gyro_set);
		
		/* --- [4] Send current values via CAN --- */
		
		yaw_motor.given_current = -(int16_t)yaw_motor.current_set;
		pitch_motor.given_current = (int16_t)pitch_motor.current_set;
		
		CAN_cmd_gimbal(yaw_motor.given_current, pitch_motor.given_current, 0, 0);
		
		/* Wait for the next cycle */
    vTaskDelayUntil(&xLastWakeTime, 1);
	}
}
