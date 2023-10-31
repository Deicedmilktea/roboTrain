#include "Gimbal_task.h"
#include "rc_potocal.h"

//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
		
extern RC_ctrl_t rc_ctrl;

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
extern motor_info_t  motor_info_chassis[8]; 

pid_struct_t gimbal_angle_pid;
pid_struct_t gimbal_speed_pid;

fp32 gimbal_angle_pid_value [3]={10,0.5,0};
fp32 gimbal_speed_pid_value [3]={10,0.5,0};

float tar_gimbal_speed = 0;
float gimbal_speed_out = 0;
float cur_gimbal_speed = 0;
//	float cur_gimbal_current = 0;

float tar_gimbal_angle = 0;

float tar_gimbal_angle1 = 0;

float gimbal_angle_out = 0;
float cur_gimbal_angle = 0;

/*云台运动task*/
void Gimbal_task(void const *pvParameters)
{
	pid_init(&gimbal_angle_pid, gimbal_angle_pid_value, 8191, 500);
	pid_init(&gimbal_speed_pid, gimbal_speed_pid_value, 3000, 100);
	tar_gimbal_angle = angle_map(tar_gimbal_angle);
	
	for(;;)
	{
		cur_gimbal_speed = motor_info_chassis[6].rotor_speed;
		cur_gimbal_angle = motor_info_chassis[6].rotor_angle;
		remote_gimbal_control();  //加上遥控器的控制
//		angle_over_zero(&tar_gimbal_angle, &cur_gimbal_angle);
		angle_over_zero(&tar_gimbal_angle1, &cur_gimbal_angle);
    gimbal_angle_out = pid_calc(&gimbal_angle_pid, tar_gimbal_angle1, cur_gimbal_angle);
    gimbal_speed_out = pid_calc(&gimbal_speed_pid, tar_gimbal_speed, gimbal_angle_out);
    CAN_cmd_gimbal(-1.4*gimbal_speed_out);
		HAL_Delay(1);
	}
}


/*将目标角度从（-pi, pi）映射到（0, 8091）*/
float angle_map(float cur_angle)
{
	return (cur_angle + pi) * 8191/(2 * pi);
}

/*角度过零处理*/
void angle_over_zero(float *tar, float *cur)
{
	if(*tar - *cur > 4096)    //4096 ：半圈机械角度
	{
		*tar -= 8191;        //8191,8192无所谓了，四舍五入
	}
	else if(*tar - *cur < -4096)
	{
		*tar += 8191;
	}
}

/*控制云台，发送数据驱动电机*/
void CAN_cmd_gimbal(int16_t gimbal_speed)
{
		uint32_t send_mail_box;
    gimbal_tx_message.StdId = 0x1FF;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[4] = (gimbal_speed >> 8);
    gimbal_can_send_data[5] = gimbal_speed;
    HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/*接收遥控器数据控制云台旋转*/
void remote_gimbal_control(void)
{
	tar_gimbal_angle += 0.02*rc_ctrl.rc.ch[2];
	if(tar_gimbal_angle > 8191)
		tar_gimbal_angle = tar_gimbal_angle - 8191;
	if(tar_gimbal_angle < 0)
		tar_gimbal_angle = tar_gimbal_angle + 8191;
	tar_gimbal_angle1 = tar_gimbal_angle;
}

