#include "FreeRTOS.h"
#include "can_receive.h"
#include "pid.h"
#include "cmsis_os.h"

extern motor_measure_t motor[4];

//左摩擦轮
float tar_left_fric_speed = 500;
float cur_left_fric_speed = 0;

//右摩擦轮
float tar_right_fric_speed = 500;
float cur_right_fric_speed = 0;

//弹丸拨轮
float tar_trigger_speed = 500;
float cur_trigger_speed = 0;

//云台
float tar_gimbal_speed = 500;
float cur_gimbal_speed = 0;

void Bullet_shoot_Task(void const * argument)
{
    pid_struct_t friction_pid;
		//static float curr_left_speed = 500;
		//static float curr_right_speed = 500;
  for(;;)
  {
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
    pid_init(&friction_pid, 1, 2, 1, 1000, 500);
		//CAN_cmd_friction(500, 500);
    CAN_cmd_friction(cur_left_fric_speed, -cur_right_fric_speed);
		cur_left_fric_speed = motor[1].speed_rpm;
    cur_left_fric_speed = pid_calc(&friction_pid, tar_left_fric_speed, cur_left_fric_speed);
		cur_right_fric_speed = motor[2].speed_rpm;
    cur_right_fric_speed = pid_calc(&friction_pid, tar_right_fric_speed, cur_right_fric_speed);
    //curr_right_speed = pid_calc(friction_pid, motor[2], tar_right_speed);
    osDelay(1);
  }
}

void Bullet_rotate_Task(void const * argument)
{
	pid_struct_t trigger_pid;
  for(;;)
  {
		pid_init(&trigger_pid, 1, 2, 1, 500, 500);
		//CAN_cmd_trigger(500, 500);
    CAN_cmd_trigger(cur_trigger_speed);
		cur_trigger_speed = motor[0].speed_rpm;
    cur_trigger_speed = pid_calc(&trigger_pid, tar_trigger_speed, cur_trigger_speed);
    osDelay(1);
  }
}

void Gimbal_Task(void const * argument)
{
  pid_struct_t gimbal_pid;
  for(;;)
  {
		pid_init(&gimbal_pid, 1, 2, 1, 500, 500);
		//CAN_cmd_gimbal(500, 500);
    CAN_cmd_gimbal(cur_gimbal_speed);
		cur_gimbal_speed = motor[0].speed_rpm;
    cur_gimbal_speed = pid_calc(&gimbal_pid, tar_gimbal_speed, cur_gimbal_speed);
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}
