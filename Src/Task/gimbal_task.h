#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "main.h"

#include "bsp_uart.h"
#include "bsp_imu.h"
#include "bsp_can.h"
#include "bsp_trigger.h"
#include "pid.h"
#include "cmsis_os.h"
#include "filter.h"
#include "stdio.h"
#include "bsp_lcd.h"

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
    const motor_measure_t *motor_measure;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
	
    PidTypeDef angle_pid;
    PidTypeDef gyro_pid;
	
		filter_t gyro_filter;
		filter_t current_filter;
	
    uint16_t offset_ecd;
    float max_ecd_angle; //deg
    float min_ecd_angle; //deg
    float ecd_angle;     //deg
    float imu_angle;     //deg
    float angle_set; 		//deg
    float motor_gyro;    //deg/s
    float motor_gyro_set;
    float motor_speed;
    float current_set;
    int16_t given_current;

} gimbal_motor_t;

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,
    SHOOT_READY_BULLET,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,
} shoot_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
    //ramp_function_source_t fric1_ramp;
    uint16_t fric_pwm1;
    //ramp_function_source_t fric2_ramp;
    uint16_t fric_pwm2;
    PidTypeDef trigger_motor_pid;
    float trigger_speed_set;
    float speed;
    float speed_set;
    float angle;
    float set_angle;
    int16_t given_current;
    int8_t ecd_count;

    uint8_t press_l;
    uint8_t press_r;
    uint8_t last_press_l;
    uint8_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    uint8_t move_flag;

    uint8_t key;
    uint8_t key_time;

    uint16_t heat_limit;
    uint16_t heat;
} shoot_control_t;

//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP        800.0f
#define TRIGGER_ANGLE_PID_KI        0.5f
#define TRIGGER_ANGLE_PID_KD        0.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f

#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f

void usb_cdc_unpackage(uint8_t* Buf, uint32_t *Len);

#endif
