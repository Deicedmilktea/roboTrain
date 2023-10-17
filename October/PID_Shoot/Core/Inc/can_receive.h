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


//1: trigger motor; 2: left friction; 3: right friction; 4: gimbal motor
static motor_measure_t motor[4];


void can_filter_init(void);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

void CAN_cmd_friction(int16_t left_friction, int16_t right_friction);

#endif