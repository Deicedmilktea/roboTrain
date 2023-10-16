#include "pid.h"

float pid_init(PID *pid, float kp, float ki, float kd)
{
    pid.Kp = kp;
    pid.Ki = ki;
    pid.Kd = kd;
}

float pid_calculate(PID *pid, float actual_val, float tar_val)
{
    /*计算目标值与实际值的误差*/
	pid.err = pid.target_val - actual_val;
	
	/*积分项*/
	pid.integral += pid.err;

	/*PID算法实现*/
	pid.output_val = pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * (pid.err - pid.err_last);

	/*误差传递*/
	pid.err_last = pid.err;

	/*返回当前实际值*/
	return pid.output_val;
}