#include "gimbal.h"
extern float pi;

/*将电机的角度（0-8091）映射到（-pi - pi）*/
float angle_map(float cur_angle)
{
	return cur_angle * 8191/(2 * pi);
}