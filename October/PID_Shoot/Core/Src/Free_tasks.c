#include "FreeRTOS.h"
#include "can_receive.h"
#include "pid.h"
#include "cmsis_os.h"

extern motor_measure_t motor[8];

//左摩擦轮
float cur_left_fric_current = 0;
float tar_left_fric_speed = 3000;
float cur_left_fric_speed = 0;

//右摩擦轮
float cur_right_fric_current = 0;
float tar_right_fric_speed = 3000;
float cur_right_fric_speed = 0;


void Bullet_shoot_Task(void const * argument)
{
    pid_struct_t left_friction_pid;
		pid_struct_t right_friction_pid;
		//static float curr_left_speed = 500;
		//static float curr_right_speed = 500;
		
    pid_init(&left_friction_pid, 10, 3, 2, 1000, 3000);
		pid_init(&right_friction_pid, 10, 3, 2, 1000, 3000);
	
  for(;;)
  {
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
		//cur_left_fric_current = motor[1].given_current;
		cur_left_fric_speed = motor[1].speed_rpm;
    cur_left_fric_current = pid_calc(&left_friction_pid, tar_left_fric_speed, cur_left_fric_speed);
		
		//cur_right_fric_current = motor[2].given_current;
		cur_right_fric_speed = motor[2].speed_rpm;
    cur_right_fric_current = pid_calc(&right_friction_pid, tar_right_fric_speed, -cur_right_fric_speed);//电机反转，为负
    //curr_right_speed = pid_calc(friction_pid, motor[2], tar_right_speed);
		
		//CAN_cmd_friction(500, 500);
    CAN_cmd_friction(cur_left_fric_current, -cur_right_fric_current);
		
    osDelay(1);
  }
}


//弹丸拨轮
float cur_trigger_current = 0;
float tar_trigger_speed = 500;
float cur_trigger_speed = 0;


void Bullet_rotate_Task(void const * argument)
{
	pid_struct_t trigger_pid;
	pid_init(&trigger_pid, 10, 1, 0, 500, 500);
	
  for(;;)
  {
		//CAN_cmd_trigger(500, 500);
		cur_trigger_speed = -motor[4].speed_rpm;
    cur_trigger_current = pid_calc(&trigger_pid, tar_trigger_speed, cur_trigger_speed);
    CAN_cmd_trigger(-cur_trigger_current);
    osDelay(1);
  }
}


//云台
float tar_gimbal_speed = 2000;
float cur_gimbal_speed = 0;
float cur_gimbal_current = 0;


void Gimbal_Task(void const * argument)
{
  pid_struct_t gimbal_pid;
  for(;;)
  {
		pid_init(&gimbal_pid, 2, 0, 0, 500, 2000);
		//CAN_cmd_gimbal(500, 500);
		cur_gimbal_speed = motor[6].speed_rpm;
    cur_gimbal_current = pid_calc(&gimbal_pid, tar_gimbal_speed, cur_gimbal_speed);
    CAN_cmd_gimbal(cur_gimbal_current);
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}
