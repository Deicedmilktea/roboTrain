#include "gimbal.h"
extern float pi;

/*将电机的角度（0-8091）映射到（-pi - pi）*/
float angle_map(float cur_angle)
{
	return (cur_angle + pi) * 8191/(2 * pi);
}

/* 角度过零处理*/
void angle_over_zero(float *tar, float *cur)
{
	if(*tar - *cur > 4096)    //4096 ：半圈机械角度
	{
		*tar -= 8191;        //8191,8192无所谓了，四舍五入
	}
	else if(*tar - *cur < -4096)
	{
		*tar += 8192;
	}
//	else
//	{
//		//*cur = *cur;
//		// do nothing
//	}
}
