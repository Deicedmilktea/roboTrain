#ifndef __PID_H
#define __PID_H
#include "main.h"

typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  
  float ref;      // target value
  float fdb;      // feedback value  
  float err[2];   // error and last error

  float p_out;
  float i_out;
  float d_out;
  float output;
}pid_struct_t;

void pid_init(pid_struct_t *pid, float kp, float ki,float kd, float i_max, float out_max);
							
float pid_calc(pid_struct_t *pid, float ref, float fdb);	
							
//void set_motor_voltage_3508(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);	
							
//void set_motor_voltage_6020(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
					
#endif