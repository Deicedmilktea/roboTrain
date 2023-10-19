#ifndef __CAN_RECEIVE_H__
#define __CAN_RECEIVE_H__

#include "main.h"
#include "can.h"


typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
}motor_measure_t;




void can1_filter_init(void);
void can2_filter_init(void);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

void CAN_cmd_friction(int16_t left_friction, int16_t right_friction);

void CAN_cmd_trigger(int16_t trigger_speed);

void CAN_cmd_gimbal(int16_t gimbal_speed);

#endif