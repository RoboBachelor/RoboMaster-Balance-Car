#ifndef IMU_TASK_H
#define IMU_TASK_H

void Wake_up_IMU_Task(void);
int8_t get_history_eular_angle(uint8_t latency, float *yaw, float *pitch);

#endif
