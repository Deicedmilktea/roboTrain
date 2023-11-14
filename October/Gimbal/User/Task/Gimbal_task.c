#include "Gimbal_task.h"
#include "rc_potocal.h"
#include "ins_task.h"
#include "cmsis_os.h"

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
extern INS_t INS;
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
extern motor_info_t  motor_info_chassis[8]; 

pid_struct_t gimbal_gyro_angle_pid;
pid_struct_t gimbal_encoder_angle_pid;
pid_struct_t gimbal_gyro_speed_pid;
pid_struct_t gimbal_encoder_speed_pid;

//Kp, Ki, Kd
fp32 gimbal_gyro_angle_pid_value [3]={10,0.5,0};
fp32 gimbal_encoder_angle_pid_value [3]={10,0.5,0};
fp32 gimbal_gyro_speed_pid_value [3]={1.5,0.1,0};
fp32 gimbal_encoder_speed_pid_value [3]={1.5,0.1,0};

float tar_gimbal_speed = 0;
float gimbal_speed_out = 0;
float cur_gimbal_speed = 0;
//	float cur_gimbal_current = 0;

float tar_gimbal_angle = 0;

float gimbal_angle_out = 0;
float cur_gimbal_angle = 0;

int error6 = 0;


/********************云台运动task*********************/
void Gimbal_task(void const *pvParameters)
{

	tar_gimbal_angle = angle_map(tar_gimbal_angle);
	
	for(;;)
	{
		error6++;
		remote_gimbal_control(1);  //加上遥控器的控制
		osDelay(1);
	}
}

void Gimbal_init()
{
	pid_init(&gimbal_gyro_angle_pid, gimbal_gyro_angle_pid_value, 8191, 500);
	pid_init(&gimbal_encoder_speed_pid, gimbal_gyro_speed_pid_value, 3000, 100);
	pid_init(&gimbal_gyro_angle_pid, gimbal_encoder_angle_pid_value, 180, 100);
	pid_init(&gimbal_encoder_speed_pid, gimbal_encoder_speed_pid_value, 30, 30);
}

/*将目标角度从（-pi, pi）映射到（0, 8091）*/
float angle_map(float cur_angle)
{
	return (cur_angle + pi) * 8191/(2 * pi);
}

/*角度过零处理*/
void angle_over_zero(float *tar, float *cur, int gimbal_mode)
{
	if(gimbal_mode == 0)
	{
			if(*tar - *cur > 4096)    //4096 ：半圈机械角度
			{
				*tar -= 8191;
			}
			else if(*tar - *cur < -4096)
			{
				*tar += 8191;
			}
	}
	
	if(gimbal_mode == 1)
	{
			if(*tar - *cur > 180)    //180 ：半圈机械角度
			{
				*tar -= 180;
			}
			else if(*tar - *cur < -180)
			{
				*tar += 180;
			}
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

/*处理接收遥控器数据控制云台旋转*/
void remote_gimbal_control(int gimbal_mode)
{
		if(gimbal_mode == 0)
		{
			//传入编码器数据
			cur_gimbal_speed = motor_info_chassis[6].rotor_speed;
			cur_gimbal_angle = motor_info_chassis[6].rotor_angle;
			
			tar_gimbal_angle += 0.02*rc_ctrl.rc.ch[2];
			if(tar_gimbal_angle > 8191)
				tar_gimbal_angle -= 8191;
			if(tar_gimbal_angle < 0)
				tar_gimbal_angle += 8191;
			
			angle_over_zero(&tar_gimbal_angle, &cur_gimbal_angle, 0);
			gimbal_angle_out = pid_calc(&gimbal_gyro_angle_pid, cur_gimbal_angle, tar_gimbal_angle);
			gimbal_speed_out = pid_calc(&gimbal_encoder_speed_pid, gimbal_angle_out, tar_gimbal_speed);

			
    		CAN_cmd_gimbal(-1.4*gimbal_speed_out);//给电流
		}
		
		if(gimbal_mode == 1)
		{
			//传入陀螺仪数据
			cur_gimbal_speed = INS.Gyro[3];
			
			//将角度转化到（0，360）
			if(INS.Yaw > 0 || INS.Yaw == 0)
				cur_gimbal_angle = INS.Yaw;
			else
				cur_gimbal_angle = INS.Yaw + 360;
			
			tar_gimbal_angle += rc_ctrl.rc.ch[2]/600;
			if(tar_gimbal_angle > 180)
				tar_gimbal_angle -= 360;
			if(tar_gimbal_angle < -180)
				tar_gimbal_angle += 360;
			
			angle_over_zero(&tar_gimbal_angle, &cur_gimbal_angle, 1);
			gimbal_angle_out = pid_calc(&gimbal_gyro_angle_pid, cur_gimbal_angle, tar_gimbal_angle);
			gimbal_speed_out = pid_calc(&gimbal_encoder_speed_pid, gimbal_angle_out, tar_gimbal_speed);

			CAN_cmd_gimbal(-100*gimbal_speed_out);//给电流
		}
}
