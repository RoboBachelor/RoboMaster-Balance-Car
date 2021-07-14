#include "gimbal_task.h"

extern RC_ctrl_t rc;
extern imu_t imu;
extern motor_measure_t motor_measure[];

gimbal_motor_t yaw_motor, pitch_motor;

uint16_t trigger_pulse = 1000;

uint8_t counter_div_4;

shoot_control_t shoot_control;

int8_t plot_data[240];
int8_t plot_data2[240];
uint8_t plot_start_index = 0;

volatile uint32_t gimbal_cnt = 0;

void Gimbal_Task(void const* argument) {
    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Init Yaw and pitch motors PID parameters
    float yaw_gyro_pid[3] = {400.0f, 0, 100000.0f};
    float yaw_angle_pid[3] = {25, 0.2, 2.5};

    float yaw_ecd_angle_pid[3] = {12, 0, 0.35};
    float pitch_gyro_pid[3] = {50.f, 0.2f, 100.f};
    float pitch_angle_pid[3] = {50, 0.4, 0.12};

    // Set motor measure pointers point to motor_measure[] in bsp_can.c
    yaw_motor.motor_measure = motor_measure + 4;
    pitch_motor.motor_measure = motor_measure + 5;

    // Init gyro (angle speed) PIDs
    PID_Init(&yaw_motor.gyro_pid, PID_POSITION, yaw_gyro_pid, 30000, 10000);
    PID_Init(&pitch_motor.gyro_pid, PID_POSITION, pitch_gyro_pid, 30000, 15000);

    // Init angle (ecd or imu) PIDs
    PID_Init(&yaw_motor.angle_pid, PID_POSITION, yaw_ecd_angle_pid, 800, 80);
    PID_Init(&pitch_motor.angle_pid, PID_POSITION, pitch_angle_pid, 200, 40);

    // Init current out filter
    filter_init(&yaw_motor.current_filter, 60.f);
    filter_init(&pitch_motor.current_filter, 45.f);

    float Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    shoot_control.shoot_motor_measure = motor_measure + 6;
    PID_Init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
	yaw_motor.gyro_pid.proportion_output_filter_coefficient = 0.95;
	yaw_motor.gyro_pid.derivative_output_filter_coefficient = 0.99995;
		
		
		pitch_motor.gyro_pid.proportion_output_filter_coefficient = 0.8;
		pitch_motor.gyro_pid.derivative_output_filter_coefficient = 0.8;
    vTaskDelay(1000);

    while (1) {
        /* --- [1] Feedback from sensors --- */

        yaw_motor.motor_gyro = imu.gimbal_yaw_gyro;
        pitch_motor.motor_gyro = imu.gyro.axis.x;

        yaw_motor.ecd_angle = (-yaw_motor.motor_measure->ecd + 4096) / 4096.f * 180.f;

        shoot_control.speed = shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;

        /* --- [2] Set mode and values from RC --- */

        /* Set gyro from RC
        yaw_motor.motor_gyro_set = - rc.rc.ch[0] / 3.f;
        pitch_motor.motor_gyro_set = rc.rc.ch[1] / 3.f;
        */
        /* Set angle increasement from RC
        yaw_motor.angle_set -= rc.rc.ch[0] / 3300.f;
        pitch_motor.angle_set += rc.rc.ch[1] / 3300.f;

        if(pitch_motor.angle_set > 12.f) pitch_motor.angle_set = 12.f;
        if(pitch_motor.angle_set < -44.f) pitch_motor.angle_set = -44.f;
        */

        /* Set angle from RC */
        yaw_motor.angle_set = -rc.rc.ch[0] / 10.f;
        pitch_motor.angle_set = rc.rc.ch[1] / 10.f - 10.f;

        /* Set trigger speed from RC */
        if (switch_is_down(rc.rc.s[0])) {
            shoot_control.speed_set = 2.f;
        } else {
            shoot_control.speed_set = 0.f;
        }

        gimbal_cnt++;

        /* --- [3] Calculate control variables --- */

        /* 250Hz Task */
        // if(++counter_div_4 >= 4){
        // 	counter_div_4 = 0;

        // 	plot_data[plot_start_index] = (int8_t) pitch_motor.gyro_filter.out;
        // 	plot_data2[plot_start_index] = (int8_t) pitch_motor.motor_gyro_set;
        // 	plot_start_index++;
        // 	if(plot_start_index >= 240) plot_start_index = 0;

        // }

        /* Angle loop PID calc: 250Hz */
        yaw_motor.motor_gyro_set = gimbal_PID_calc(
            &yaw_motor.angle_pid, yaw_motor.ecd_angle, yaw_motor.angle_set,
            -yaw_motor.motor_gyro);
        pitch_motor.motor_gyro_set = gimbal_PID_calc(
            &pitch_motor.angle_pid, imu.eulerAngles.angle.pitch, pitch_motor.angle_set,
            -pitch_motor.motor_gyro);

        /* Gyro loop PID */
        yaw_motor.current_set = PID_Calc(&yaw_motor.gyro_pid, yaw_motor.motor_gyro,
                                         yaw_motor.motor_gyro_set);
        pitch_motor.current_set = PID_Calc(
            &pitch_motor.gyro_pid, pitch_motor.motor_gyro, pitch_motor.motor_gyro_set);

        /* Trigger motor PID */
        PID_Calc(&shoot_control.trigger_motor_pid, shoot_control.speed,
                 shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);

        /* --- [4] Send current values via CAN --- */

        /* Enable current filter
        yaw_motor.given_current = - (int16_t) filter_calc(&yaw_motor.current_filter, yaw_motor.current_set);
        pitch_motor.given_current = (int16_t) (filter_calc(&pitch_motor.current_filter, pitch_motor.current_set) + 6000.f * cosf(imu.eulerAngles.angle.pitch / 57.29577958f));
        */

        /* Disable current filter */
        yaw_motor.given_current = -(int16_t)yaw_motor.current_set;
        pitch_motor.given_current = (int16_t)(pitch_motor.current_set +
                                              6000.f *
                                                  cosf(imu.eulerAngles.angle.pitch / 57.29577958f));

        CAN_cmd_gimbal(yaw_motor.given_current, pitch_motor.given_current,
                       shoot_control.given_current, 0);

        trigger_pulse = 1250 + rc.rc.ch[3] / 660.0f * 250.0f;

        trigger_set_pulse(trigger_pulse);

        /* Wait for the next cycle */
        vTaskDelayUntil(&xLastWakeTime, 1);
    }
}

int16_t d1, d2, d3;

void usb_cdc_unpackage(uint8_t* Buf, uint32_t* Len) {
    char command[10];
    sscanf((char*)Buf, "%s", command);
    if (strcmp(command, "DIST") == 0) {
        sscanf((char*)Buf + 4, "%hd%hd%hd", &d1, &d2, &d3);
    }
}

int8_t plot_buf[240];
int8_t plot2_buf[240];

void LCD_Task(void const* argument) {
    POINT_COLOR = WHITE;
    LCD_Clear(BLACK);

    /*
    LCD_ShowString(20, 20, "x=20,y=20");

    LCD_ShowString(20, 160, "x=20,y=160");
    LCD_ShowString(160, 20, "x=160,y=20");
    LCD_ShowString(160, 160, "x=160,y=160");

    vTaskDelay(10000);
    */

    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
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

        /* Wait for the next cycle */
        vTaskDelayUntil(&xLastWakeTime, 45);
    }
}
