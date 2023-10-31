#ifndef _GIMBAL_H_
#define _GIMBAL_H_

/*将目标角度从（-pi, pi）映射到（0, 8091）*/
float angle_map(float cur_angle);

/* 角度过零处理*/
void angle_over_zero(float *tar, float *cur);

#endif