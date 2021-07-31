#include "gimbal_task.h"

extern RC_ctrl_t rc;
extern imu_t imu;
extern motor_measure_t motor_measure[];

gimbal_motor_t yaw_motor, pitch_motor;

float trigger_target_pulse = 1000;
float trigger_send_pulse = 1000;
uint8_t trigger_is_on = 0;
uint8_t last_trigger_sw_state = 0;

int16_t chassis_current_set;

uint8_t counter_div_4;

shoot_control_t shoot_control;

//int8_t plot_data[240];
//int8_t plot_data2[240];
//uint8_t plot_start_index = 0;

volatile uint32_t gimbal_cnt = 0;

/* Chassis Control Variables */
Chassis_Motor_t chassis_motor;

float accel_gimbal_y;

uint8_t chassis_auto_flag = 0;
uint8_t chassis_dir_is_left = 1;
uint16_t chassis_runtime_cur_dir = 0;




/* Vision Control Variables */
typedef struct {
	uint8_t header;
	uint8_t latency;
	uint8_t found_target;
	float yaw, pitch, dist;
} __packed vision_rx_t;

typedef struct{
	vision_rx_t vision_rx;
	float absolute_yaw;
	float absolute_pitch;
	float predicted_yaw;
	float predicted_pitch;
	uint8_t found_target_flag;
	uint8_t new_data_ready_flag;
} vision_control_t;

vision_control_t vision_control;

KalmanFilter_t yaw_kf, pitch_kf;

#define dt 1e-3

float F_Init[9] = {
    1, 0, 0, dt, 1, 0, 0.5 * dt *dt, dt, 1,
};

float H_Init[3] = {
    0,
    0,
    1,
};

float B_Init[3] = {
    0,
    0,
    0,
};

float Q_Init[9] = {0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1};
float R_Init[1] = {0.5};

float Q_Init_height[9] = {0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1};

float R_Init_height[1] = {4};

float state_min_variance[3] = {0.03, 0.005, 0.1};

float coord_trans_matrix_data[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float milli_trans_matrix_data[9] = {
    1, 0, 0, dt, 1, 0, 0.5 * dt *dt, dt, 1,
};
float P_Init[9] = {
    10, 0, 0, 0, 30, 0, 0, 0, 10,
};

float delay_trans_matrix_data[9] = {
    1, 0, 0, 0, 1, 0, 0, 0, 1,
};

arm_matrix_instance_f32 milli_trans_matrix, cur_yaw;
arm_matrix_instance_f32 delay_trans_matrix, predicted_yaw;
float cur_yaw_data[3];
float predicted_yaw_data[3];

void deg_limit(float *x){
	if(*x > 180.f) *x -= 360.f;
	if(*x < -180.f) *x += 360.f;
}

void chassis_change_dir(){
	chassis_dir_is_left = (chassis_dir_is_left + 1) & 0x1;
	chassis_runtime_cur_dir = 0;
}

void Gimbal_Task(void const *argument) {
	// Initialise the xLastWakeTime variable with the current time.
	TickType_t xLastWakeTime = xTaskGetTickCount();

	// Init Yaw and pitch motors PID parameters
	float yaw_gyro_pid[3] = { 400.0f, 0, 100000.0f };
	float yaw_angle_pid[3] = { 17, 0, 0.85 };

	float yaw_ecd_angle_pid[3] = { 12, 0, 0.35 };
	float pitch_gyro_pid[3] = { 50.f, 0.8f, 50.f };
	float pitch_angle_pid[3] = { 40, 0.4, 0.2 };

	// Set motor measure pointers point to motor_measure[] in bsp_can.c
	yaw_motor.motor_measure = motor_measure + 4;
	pitch_motor.motor_measure = motor_measure + 5;

	// Init gyro (angle speed) PIDs
	PID_Init(&yaw_motor.gyro_pid, PID_POSITION, yaw_gyro_pid, 30000, 10000);
	PID_Init(&pitch_motor.gyro_pid, PID_POSITION, pitch_gyro_pid, 24000, 5000);

	// Init angle (ecd or imu) PIDs
	PID_Init(&yaw_motor.angle_pid, PID_POSITION, yaw_angle_pid, 800, 80);
	PID_Init(&pitch_motor.angle_pid, PID_POSITION, pitch_angle_pid, 200, 40);

	// Init current out filter
	filter_init(&yaw_motor.current_filter, 60.f);
	filter_init(&pitch_motor.current_filter, 45.f);

	float Trigger_speed_pid[3] = { TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI,
			TRIGGER_ANGLE_PID_KD };
	shoot_control.shoot_motor_measure = motor_measure + 6;
	PID_Init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid,
			TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);

	yaw_motor.gyro_pid.proportion_output_filter_coefficient = 0.95;
	yaw_motor.gyro_pid.derivative_output_filter_coefficient = 0.99995;

	pitch_motor.gyro_pid.proportion_output_filter_coefficient = 0.8;
	pitch_motor.gyro_pid.derivative_output_filter_coefficient = 0.8;

	/* Chassis Motor M3508 */
	chassis_motor.chassis_motor_measure = motor_measure + 0;
	float m3508_speed_pid_para[] = M3508_MOTOR_SPEED_PID;
	PID_Init(&chassis_motor.pid_speed, PID_POSITION, m3508_speed_pid_para,
			M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT);

	set_waring_buf_value(100.f);

	chassis_motor.accel_filter.a = 0.99;

	/* Kalman_filter Init */
	
  Matrix_Init(&milli_trans_matrix, 3, 3, milli_trans_matrix_data);
  Matrix_Init(&delay_trans_matrix, 3, 3, delay_trans_matrix_data);
  Matrix_Init(&predicted_yaw, 3, 1, predicted_yaw_data);
	
	Kalman_Filter_Create(&yaw_kf, 3, 0, 1);
  memcpy(yaw_kf.F_data, F_Init, sizeof(F_Init));
  memcpy(yaw_kf.B_data, B_Init, sizeof(B_Init));
  memcpy(yaw_kf.H_data, H_Init, sizeof(H_Init));
  memcpy(yaw_kf.R_data, R_Init, sizeof(R_Init));
  memcpy(yaw_kf.Q_data, Q_Init, sizeof(Q_Init));
  memcpy(yaw_kf.P_data, P_Init, sizeof(P_Init));
  memcpy(yaw_kf.StateMinVariance, state_min_variance, sizeof(state_min_variance));
	yaw_kf.UseAutoAdjustment = 0;
			
  Matrix_Init(&cur_yaw, 3, 1, cur_yaw_data);
			
	vTaskDelay(1500);

	yaw_motor.angle_set = imu.eulerAngles.angle.yaw;
	pitch_motor.angle_set = imu.eulerAngles.angle.pitch;

	while (1) {
		gimbal_cnt++;
		/* --- [1] Feedback from sensors --- */

		yaw_motor.motor_gyro = imu.gimbal_yaw_gyro;
		pitch_motor.motor_gyro = imu.gyro.axis.x;

		yaw_motor.ecd_angle = (-yaw_motor.motor_measure->ecd + 4096) / 4096.f
				* 180.f;
		yaw_motor.imu_angle = imu.eulerAngles.angle.yaw;

		pitch_motor.imu_angle = imu.eulerAngles.angle.pitch;

		shoot_control.speed = shoot_control.shoot_motor_measure->speed_rpm
				* MOTOR_RPM_TO_SPEED;

		if (!last_trigger_sw_state && switch_is_up(rc.rc.s[1])) {
			trigger_is_on = ~trigger_is_on & 0x1;
		}
		last_trigger_sw_state = switch_is_up(rc.rc.s[1]);

		float tmp_speed = chassis_motor.chassis_motor_measure->speed_rpm
				* CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
		chassis_motor.accel = tmp_speed - chassis_motor.speed;
		chassis_motor.speed = tmp_speed;

		accel_gimbal_y = imu.accel.axis.y
				* arm_cos_f32(imu.eulerAngles.angle.pitch / 57.29577958f)
				- imu.accel.axis.z
						* arm_sin_f32(imu.eulerAngles.angle.pitch / 57.29577958f);

		chassis_motor.accel = accel_gimbal_y
				* arm_cos_f32((yaw_motor.ecd_angle - 60) / 57.29577958f)
				+ imu.accel.axis.x
						* arm_sin_f32((yaw_motor.ecd_angle - 60) / 57.29577958f);

		filter_calc(&chassis_motor.accel_filter, chassis_motor.accel);

		/* 250Hz Task, Record data for plotting
		if(++counter_div_4 >= 4){
			counter_div_4 = 0;
			plot_data[plot_start_index] = (int8_t) pitch_motor.gyro_filter.out;
			plot_data2[plot_start_index] = (int8_t) pitch_motor.motor_gyro_set;
			plot_start_index++;
			if(plot_start_index >= 240) plot_start_index = 0;
		}
		*/

		/* --- [2] Set mode and values from RC --- */

		/* [2.1] Vision Gimbal Control */
		/* Set gyro from RC
		 yaw_motor.motor_gyro_set = - rc.rc.ch[0] / 3.f;
		 pitch_motor.motor_gyro_set = rc.rc.ch[1] / 3.f;
		 */

		/* Set angle from RC
		 yaw_motor.angle_set = -rc.rc.ch[0] / 10.f;
		 pitch_motor.angle_set = rc.rc.ch[1] / 10.f - 10.f;
		 */

		if(vision_control.new_data_ready_flag){
			vision_control.new_data_ready_flag = 0;
			
			float Ts = vision_control.vision_rx.latency * 1e-3;
			yaw_kf.F_data[3] = Ts;
			yaw_kf.F_data[7] = Ts;
			yaw_kf.F_data[6] = 0.5 * Ts * Ts;
			yaw_kf.MeasuredVector[0] = vision_control.absolute_yaw;
			Kalman_Filter_Update(&yaw_kf);
			memcpy(cur_yaw_data, yaw_kf.xhat_data, sizeof(float) * 3);
			

			
		}
		
		if(vision_control.found_target_flag){
					
			float horizon = vision_control.vision_rx.latency * 1e-3 + 0.1;
			// Update prediction matrix
			delay_trans_matrix_data[3] = delay_trans_matrix_data[7] = horizon;
			delay_trans_matrix_data[6] = 0.5 * horizon * horizon;
			// Predict angle
			Matrix_Multiply(&delay_trans_matrix, &cur_yaw, &predicted_yaw);
			
			Matrix_Multiply(&milli_trans_matrix, &cur_yaw, &cur_yaw);
			
			yaw_motor.angle_set = predicted_yaw_data[2] + 57.29f * atanf(8 / vision_control.vision_rx.dist);
			pitch_motor.angle_set = vision_control.absolute_pitch + 3;

			
		} 
		else{
			/* Set angle increasement from RC */
			yaw_motor.angle_set -= rc.rc.ch[0] / 3300.f;
			pitch_motor.angle_set += rc.rc.ch[1] / 3300.f;

			/* Rewrite angle increasement in auto mode */
			if(chassis_auto_flag){
				yaw_motor.angle_set -= 0.09;
				yaw_motor.angle_set = PITCH_ANGLE_MIN;
			}
			
			deg_limit(&yaw_motor.angle_set);
			
			
      Kalman_Filter_Reset(&yaw_kf);
      memcpy(yaw_kf.P_data, P_Init, sizeof(P_Init));
			
		}
		if (pitch_motor.angle_set > PITCH_ANGLE_MAX)
			pitch_motor.angle_set = PITCH_ANGLE_MAX;
		if (pitch_motor.angle_set < PITCH_ANGLE_MIN)
			pitch_motor.angle_set = PITCH_ANGLE_MIN;

		
		/* [2.2] Triggr Speed Control */
		
		/* Set trigger speed from RC switcher */
		 if (switch_is_down(rc.rc.s[1]) && trigger_is_on) {
		 shoot_control.speed_set = 3.f;
		 } else {
		 shoot_control.speed_set = 0.f;
		 }
		 
		/* Set trigger speed from RC throttle 
		shoot_control.speed_set = rc.rc.ch[3] / 660.f * 5.f + 5.f;
		*/
		
		/* Rewrite the trigger speed if in auto mode */
		 if(chassis_auto_flag){
			 if(vision_control.vision_rx.found_target){
					shoot_control.speed_set = 3.f;
			 }
			 else{
					shoot_control.speed_set = 0.f;
			 }
		 }
		 
		/* [2.3] Chassis Control */

		if (switch_is_up(rc.rc.s[0])) {
			chassis_auto_flag = 1;
		} else {
			chassis_auto_flag = 0;
		}

		if (chassis_auto_flag) {
			// Dir: Left
			if (chassis_dir_is_left) {
				chassis_motor.speed_set = 1.7f;
				if (chassis_motor.accel_filter.out < -4.7f) {
					chassis_change_dir();
				}
			}
			// Dir: Right
			else {
				chassis_motor.speed_set = -1.7f;
				if (chassis_motor.accel_filter.out > 4.7f) {
					chassis_change_dir();
				}
			}
			
			
			if(++chassis_runtime_cur_dir > 4000){
				chassis_change_dir();
			}
		} else {
			chassis_motor.speed_set = rc.rc.ch[2] / 660.f * 4.f;
		}

		/* Set chassis current from RC */
		// chassis_current_set = rc.rc.ch[2] * -10;

		/* --- [3] Calculate control variables --- */

		/* Angle loop PID calc: 250Hz */
		yaw_motor.motor_gyro_set = gimbal_PID_calc(&yaw_motor.angle_pid,
				yaw_motor.imu_angle, yaw_motor.angle_set,
				-yaw_motor.motor_gyro);
		pitch_motor.motor_gyro_set = gimbal_PID_calc(&pitch_motor.angle_pid,
				pitch_motor.imu_angle, pitch_motor.angle_set,
				-pitch_motor.motor_gyro);

		/* Gyro loop PID */
		yaw_motor.current_set = PID_Calc(&yaw_motor.gyro_pid,
				yaw_motor.motor_gyro, yaw_motor.motor_gyro_set);
		pitch_motor.current_set = PID_Calc(&pitch_motor.gyro_pid,
				pitch_motor.motor_gyro, pitch_motor.motor_gyro_set);

		/* Trigger motor PID */
		PID_Calc(&shoot_control.trigger_motor_pid, shoot_control.speed,
				shoot_control.speed_set);
		shoot_control.given_current =
				(int16_t) (shoot_control.trigger_motor_pid.out);

		/* Chassis Motor PID */
		chassis_motor.current_set = (int16_t) PID_Calc(&chassis_motor.pid_speed,
				chassis_motor.speed, chassis_motor.speed_set);
		chassis_motor.give_current = (int16_t) chassis_power_control(
				chassis_motor.current_set);

		/* --- [4] Send current values via CAN --- */

		/* Enable current filter
		 yaw_motor.given_current = - (int16_t) filter_calc(&yaw_motor.current_filter, yaw_motor.current_set);
		 pitch_motor.given_current = (int16_t) (filter_calc(&pitch_motor.current_filter, pitch_motor.current_set) + 6000.f * cosf(imu.eulerAngles.angle.pitch / 57.29577958f));
		 */

		/* Disable current filter */
		yaw_motor.given_current = -(int16_t) yaw_motor.current_set;
		pitch_motor.given_current = (int16_t) (pitch_motor.current_set
				+ 6000.f * arm_cos_f32(imu.eulerAngles.angle.pitch / 57.29577958f));

		CAN_cmd_gimbal(yaw_motor.given_current, pitch_motor.given_current,
				shoot_control.given_current, 0);

		CAN_cmd_chassis(chassis_motor.give_current, 0, 0, 0);

		if (trigger_is_on) {
			trigger_target_pulse = 1200.f;
		} else {
			trigger_target_pulse = 1000.f;
		}

		if (trigger_target_pulse - trigger_send_pulse > 0.15) {
			trigger_send_pulse += 0.15;
		} else if (trigger_target_pulse - trigger_send_pulse < -0.15) {
			trigger_send_pulse -= 0.15;
		} else {
			trigger_send_pulse = trigger_target_pulse;
		}

		trigger_set_pulse((uint16_t) trigger_send_pulse);

		/* Wait for the next cycle */
		vTaskDelayUntil(&xLastWakeTime, 1);
	}
}

void usb_cdc_unpackage(uint8_t *Buf, uint32_t *Len) {
	if (*Len < 15) {
		return;
	}
	if (Buf[0] != 0xF6) {
		return;
	}
	memcpy(&vision_control.vision_rx, Buf, sizeof(vision_rx_t));
	if(vision_control.vision_rx.found_target){
		get_history_eular_angle(vision_control.vision_rx.latency,
				&vision_control.absolute_yaw, &vision_control.absolute_pitch);
		vision_control.absolute_yaw += vision_control.vision_rx.yaw;
		vision_control.absolute_pitch += vision_control.vision_rx.pitch;
		deg_limit(&vision_control.absolute_yaw);
		deg_limit(&vision_control.absolute_pitch);
	}
	vision_control.found_target_flag = vision_control.vision_rx.found_target;
	vision_control.new_data_ready_flag = 1;
}


//int8_t plot_buf[240];
//int8_t plot2_buf[240];

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
