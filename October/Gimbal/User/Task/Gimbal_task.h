#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "struct_typedef.h"

#define pi 3.1415926

void Gimbal_task(void const *pvParameters);

/*将目标角度从（-pi, pi）映射到（0, 8091）*/
float angle_map(float cur_angle);

/*角度过零处理*/
void angle_over_zero(float *tar, float *cur);

/*控制云台，发送数据驱动电机*/
void CAN_cmd_gimbal(int16_t gimbal_speed);

/*接收遥控器数据控制云台旋转*/
void remote_gimbal_control(void);

#endif