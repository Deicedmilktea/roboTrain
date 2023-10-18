#ifndef PID_H
#define PID_H

#include "can_receive.h"

typedef struct
{
	float target_val;   //目标值
	float err;          //偏差值
	float err_last;     //上一个偏差值
	float Kp,Ki,Kd;     //比例、积分、微分系数
	float integral;     //积分值
	float output_val;   //输出值
}PID;

void pid_init(PID pid, float kp, float ki, float kd);

float pid_calc(PID pid, float actual_val, float tar_val);


#endif