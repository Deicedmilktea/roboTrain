#ifndef _GIMBAL_H_
#define _GIMBAL_H_

/*将电机的角度（0-8091）映射到（-pi - pi）*/
float angle_map(float cur_angle);

/* 角度过零处理*/
void angle_over_zero(float *tar, float *cur);

#endif