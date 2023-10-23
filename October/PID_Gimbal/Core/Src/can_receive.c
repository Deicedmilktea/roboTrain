#include "can_receive.h"
#include "can.h"

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
motor_measure_t motor[1];

//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

void can1_filter_init(void)
{
 	CAN_FilterTypeDef can_filter;
	can_filter.FilterBank = 0;
	can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter.FilterIdHigh = 0;
	can_filter.FilterIdLow = 0;
	can_filter.FilterMaskIdHigh = 0;
	can_filter.FilterMaskIdLow = 0;
	can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	can_filter.SlaveStartFilterBank = 14;
	can_filter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1,&can_filter);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan1);
}

void can2_filter_init(void)
{
 	CAN_FilterTypeDef can_filter;
	can_filter.FilterBank = 14;
	can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter.FilterIdHigh = 0;
	can_filter.FilterIdLow = 0;
	can_filter.FilterMaskIdHigh = 0;
	can_filter.FilterMaskIdLow = 0;
	can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	can_filter.SlaveStartFilterBank = 14;
	can_filter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan2,&can_filter);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan2);
}

int error = 0;

//receive motor can message
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		error++;
	
		if(rx_header.StdId == 0x207)
			get_motor_measure(&motor[0], rx_data);
			
		
//		switch (rx_header.StdId)
//    {
//        case 0x201:
//        case 0x202:
//        case 0x203:
//        case 0x204:
//				case 0x205:
//				case 0x206:
//				case 0x207:
//				case 0x208:
//        {
//            uint8_t i = 0;
//            //get motor id
//            i = rx_header.StdId - 0x207;
//            get_motor_measure(&motor[i], rx_data);
//            //detect_hook(CHASSIS_MOTOR1_TOE + i);
//            break;
//        }

//				default:
//				{
//						break;
//				}
//    }
}

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