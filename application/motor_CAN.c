#include "motor_CAN.h"
#include "cmsis_os.h"

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
#define get_LK9025_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[7] << 8 | (data)[6]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[5] << 8 | (data)[4]);      \
        (ptr)->given_current = (uint16_t)((data)[3] << 8 | (data)[2]);  \
        (ptr)->temperate = (data)[1];                                     \
    }
    

static CAN_TxHeaderTypeDef turret_tx_messege;
static CAN_TxHeaderTypeDef gimbal_tx_messege;
static CAN_TxHeaderTypeDef  chassis_tx_message;
    
static uint8_t turret_can_send_data[8];
static uint8_t gimbal_can_send_data[8];
static uint8_t chassis_can_send_data[8];
    
motor_measure_t motor_chassis[2];
motor_measure_t motor_gimbal[5];
/**
  * @brief          发送电机控制电流(0x205,0x206)
  * @param[in]      motor1: (0x201) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_TURRET(int16_t shoot1,int16_t shoot2,int16_t trigger)
{
  turret_tx_messege.StdId = CAN_TURRET_ID;
  turret_tx_messege.RTR   = CAN_RTR_DATA;
  turret_tx_messege.DLC   = 0x08;
  turret_tx_messege.IDE   = CAN_ID_STD;
  
  turret_can_send_data[0] = shoot1 >> 8;
  turret_can_send_data[1] = shoot1;
  turret_can_send_data[2] = shoot2 >> 8;
  turret_can_send_data[3] = shoot2;
  turret_can_send_data[4] = trigger>> 8;
  turret_can_send_data[5] = trigger;
  turret_can_send_data[6] = 0;
  turret_can_send_data[7] = 0;
  
  HAL_CAN_AddTxMessage(&hcan1,&turret_tx_messege,turret_can_send_data,0);
  
  
}
/**
  * @brief          发送电机控制电流(0x205,0x206)
  * @param[in]      motor1: (0x205) 6020电机控制电压, 范围 [-30000,30000]
  * @param[in]      motor2: (0x206) 6020电机控制电压, 范围 [-30000,30000]
  * @retval         none
  */
void CAN_cmd_GIMBAL(int16_t yaw,int16_t pitch)
{
  gimbal_tx_messege.StdId = CAN_GIMBAL_ID;
  gimbal_tx_messege.RTR   = CAN_RTR_DATA;
  gimbal_tx_messege.DLC   = 0x08;
  gimbal_tx_messege.IDE   = CAN_ID_STD;
  
  gimbal_can_send_data[0] = yaw >> 8;
  gimbal_can_send_data[1] = yaw;
  gimbal_can_send_data[2] = pitch >> 8;
  gimbal_can_send_data[3] = pitch;
  gimbal_can_send_data[4] = 0;
  gimbal_can_send_data[5] = 0;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  
  HAL_CAN_AddTxMessage(&hcan1,&gimbal_tx_messege,gimbal_can_send_data,0);
}
/**
  * @brief          发送电机控制电流(0x141,0x142,0x143,0x144)
  * @param[in]      9025电机控制电流, 范围 [-2000,2000]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x280;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 ;
    chassis_can_send_data[1] = motor1>> 8;
    chassis_can_send_data[2] = motor2 ;
    chassis_can_send_data[3] = motor2>> 8;
    chassis_can_send_data[4] = 0 ;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0 ;
    chassis_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    // ??循环把FIFO里所有帧都读完（关键）
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

        if (hcan == &hcan1)
        {
            if (rx_data[0] == 0x01) {
                Mit_receive_Velocity.can1_01 = uint_to_float((rx_data[3]<<4)|(rx_data[4]>>4), V_MIN, V_MAX, 12);
                Mit_receive_Position.can1_01 = uint_to_float((rx_data[1]<<8)|(rx_data[2]), P_MIN, P_MAX, 16);
                Mit_receive_Current.can1_01 = uint_to_float((((rx_data[4]&0x0F))<<8)|(rx_data[5]), -40.f, 40.f, 12);
            }
            if (rx_data[0] == 0x02) {
                Mit_receive_Velocity.can1_02 = uint_to_float((rx_data[3]<<4)|(rx_data[4]>>4), V_MIN, V_MAX, 12);
                Mit_receive_Position.can1_02 = uint_to_float((rx_data[1]<<8)|(rx_data[2]), P_MIN, P_MAX, 16);
                Mit_receive_Current.can1_02 = uint_to_float((((rx_data[4]&0x0F))<<8)|(rx_data[5]), -40.f, 40.f, 12);
            }

            if (rx_header.StdId == 0x141) get_LK9025_measure(&motor_chassis[1], rx_data);
            if (rx_header.StdId == 0x142) get_LK9025_measure(&motor_chassis[0], rx_data);
        }

        else if (hcan == &hcan2)
        {
            if (rx_data[0] == 0x01) {
                Mit_receive_Velocity.can2_01 = uint_to_float((rx_data[3]<<4)|(rx_data[4]>>4), V_MIN, V_MAX, 12);
                Mit_receive_Position.can2_01 = uint_to_float((rx_data[1]<<8)|(rx_data[2]), P_MIN, P_MAX, 16);
                Mit_receive_Current.can2_01 = uint_to_float((((rx_data[4]&0x0F))<<8)|(rx_data[5]), -40.f, 40.f, 12);
            }
            if (rx_data[0] == 0x02) {
                Mit_receive_Velocity.can2_02 = uint_to_float((rx_data[3]<<4)|(rx_data[4]>>4), V_MIN, V_MAX, 12);
                Mit_receive_Position.can2_02 = uint_to_float((rx_data[1]<<8)|(rx_data[2]), P_MIN, P_MAX, 16);
                Mit_receive_Current.can2_02 = uint_to_float((((rx_data[4]&0x0F))<<8)|(rx_data[5]), -40.f, 40.f, 12);
            }
        }
    }
}


/**
  * @brief          返回6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_gimbal[4];
}
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_gimbal[3];
}

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_gimbal[2];
}
/**
  * @brief          返回右摩擦轮电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_fric_motor_r_measure_point(void)
{
    return &motor_gimbal[1];
}
/**
  * @brief          返回左摩擦轮电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_fric_motor_l_measure_point(void)
{
    return &motor_gimbal[0];
}
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}


















