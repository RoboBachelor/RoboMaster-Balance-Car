#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "main.h"

#include "bsp_uart.h"
#include "bsp_imu.h"
#include "bsp_can.h"
#include "pid.h"
#include "cmsis_os.h"

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //���ԭʼֵ����
    GIMBAL_MOTOR_GYRO,    //��������ǽǶȿ���
    GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
} gimbal_motor_mode_e;

typedef struct
{
    const motor_measure_t *motor_measure;
    PidTypeDef angle_pid;
    PidTypeDef gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
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

#endif
