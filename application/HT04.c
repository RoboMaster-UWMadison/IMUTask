/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "HT04.h"
#include "main.h"

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))  
#define MABS(x) ((x > 0) ? x : -x)
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


Moto_Dreame_Float_t Dreame_moto_float[6] = {0};
Mit_Velocity_type Mit_receive_Velocity = {0};   //�ĸ�������ٶ�
Mit_Position_type Mit_receive_Position = {0};   //�ĸ������λ��
Mit_Current_type Mit_receive_Current = {0};     //�ĸ�����ĵ���
// MIT LQR ���������ӵ��ٶ�
volatile float CurVelocity1 = 0;   
volatile float CurVelocity2 = 0; 
m_rmd_t rmd_01,rmd_02;


/**
  * @brief  Converts a float to an unsigned int, given range and number of bits
  * @param
  * @retval 
  */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief  converts unsigned int to float, given range and number of bits
  * @param
  * @retval 
  */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//    CAN_RxHeaderTypeDef rx_header;
//    uint8_t rx_data[8];
//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

//	    if (hcan == &hcan1)
//    {
//		//ע�⣺���������Ӧ���ƣ�����һ�α���֮�󣬲Ż��յ�һ�α���
//		if( rx_data[0] == 0x01 ) // && rx_header.StdId != ��ǰʹ�õĵ��ID����ֹ��MIT���������ͻ
//		{			
//		Mit_receive_Velocity.can1_01 = uint_to_float((rx_data[3]<<4)|(rx_data[4]>>4), V_MIN, V_MAX, 12);

//		Mit_receive_Position.can1_01 = uint_to_float((rx_data[1]<<8)|(rx_data[2]), P_MIN, P_MAX, 16);

//		Mit_receive_Current.can1_01 = uint_to_float((((rx_data[4]&0x0F))<<8)|(rx_data[5]), -40.f, 40.f, 12);
//		} 
//		if( rx_data[0] == 0x02) 
//		{			
//		Mit_receive_Velocity.can1_02 = uint_to_float((rx_data[3]<<4)|(rx_data[4]>>4), V_MIN, V_MAX, 12);

//		Mit_receive_Position.can1_02 = uint_to_float((rx_data[1]<<8)|(rx_data[2]), P_MIN, P_MAX, 16);

//		Mit_receive_Current.can1_02 = uint_to_float((((rx_data[4]&0x0F))<<8)|(rx_data[5]), -40.f, 40.f, 12);
//		}
//  }
//    if (hcan == &hcan2)
//    {        
//			 if( rx_data[0] == 0x01 ) // && rx_header.StdId != ��ǰʹ�õĵ��ID����ֹ��MIT���������ͻ
//		{			
//		Mit_receive_Velocity.can2_01 = uint_to_float((rx_data[3]<<4)|(rx_data[4]>>4), V_MIN, V_MAX, 12);

//		Mit_receive_Position.can2_01 = uint_to_float((rx_data[1]<<8)|(rx_data[2]), P_MIN, P_MAX, 16);

//		Mit_receive_Current.can2_01 = uint_to_float((((rx_data[4]&0x0F))<<8)|(rx_data[5]), -40.f, 40.f, 12);
//		} 
//		if( rx_data[0] == 0x02) 
//		{			
//		Mit_receive_Velocity.can2_02 = uint_to_float((rx_data[3]<<4)|(rx_data[4]>>4), V_MIN, V_MAX, 12);

//		Mit_receive_Position.can2_02 = uint_to_float((rx_data[1]<<8)|(rx_data[2]), P_MIN, P_MAX, 16);

//		Mit_receive_Current.can2_02 = uint_to_float((((rx_data[4]&0x0F))<<8)|(rx_data[5]), -40.f, 40.f, 12);
//      }
//    }
//    

//  }
/* MIT��� ��buf�е�����ͨ��CAN�ӿڷ��ͳ�ȥ�ĺ��� */
static void CanTransmit_can1(uint8_t *buf, uint8_t len,uint16_t id)
{
    CAN_TxHeaderTypeDef TxHead;             /**!< canͨ�ŷ���Э��ͷ */
    uint32_t canTxMailbox;
    
    if((buf != NULL) && (len != 0))
    {
        TxHead.StdId    = id;     /* ָ����׼��ʶ������ֵ��0x00-0x7FF */ //0x01
        TxHead.IDE      = CAN_ID_STD;       /* ָ����Ҫ������Ϣ�ı�ʶ������ */
        TxHead.RTR      = CAN_RTR_DATA;     /* ָ����Ϣ����֡���� */
        TxHead.DLC      = len;              /* ָ����Ҫ�����֡���� */
        
        if(HAL_CAN_AddTxMessage(&hcan1, &TxHead, buf, (uint32_t *)&canTxMailbox) == HAL_OK )
        {
            
        }
    }
}
static void CanTransmit_can2(uint8_t *buf, uint8_t len,uint16_t id)
{
    CAN_TxHeaderTypeDef TxHead;             /**!< canͨ�ŷ���Э��ͷ */
    uint32_t canTxMailbox;
    
    if((buf != NULL) && (len != 0))
    {
        TxHead.StdId    = id;     /* ָ����׼��ʶ������ֵ��0x00-0x7FF */
        TxHead.IDE      = CAN_ID_STD;       /* ָ����Ҫ������Ϣ�ı�ʶ������ */
        TxHead.RTR      = CAN_RTR_DATA;     /* ָ����Ϣ����֡���� */
        TxHead.DLC      = len;              /* ָ����Ҫ�����֡���� */
        
        if(HAL_CAN_AddTxMessage(&hcan2, &TxHead, buf, (uint32_t *)&canTxMailbox) == HAL_OK )
        {
            
        }
    }
}
/**
  * @brief  MIT�����Can���߷��Ϳ��Ʋ���
  * @param
  * @retval 
  */
void CanComm_SendControlPara_can1(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint16_t id)
{
    uint16_t p, v, kp, kd, t;
    uint8_t buf[8];
	  uint16_t MIT_id_can1;
	  MIT_id_can1 = id;
    
    /* ��������Ĳ����ڶ���ķ�Χ�� */
    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);   //Ŀ��λ��
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);    //Ŀ���ٶ�
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);    //λ������
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);    //�ٶ�����
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);    //����
    
    /* ����Э�飬��float��������ת�� */
    p = float_to_uint(f_p,      P_MIN,  P_MAX,  16);            
    v = float_to_uint(f_v,      V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp,    KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd,    KD_MIN, KD_MAX, 12);
    t = float_to_uint(f_t,      T_MIN,  T_MAX,  12);
    
    /* ���ݴ���Э�飬������ת��ΪCAN���������ֶ� */   //��Ϊ��8λ8λ���ڷ�������Ҫ�������
    buf[0] = p>>8;
    buf[1] = p&0xFF;
    buf[2] = v>>4;
    buf[3] = ((v&0xF)<<4)|(kp>>8);
    buf[4] = kp&0xFF;
    buf[5] = kd>>4;
    buf[6] = ((kd&0xF)<<4)|(t>>8);
    buf[7] = t&0xff;
    
    /* ͨ��CAN�ӿڰ�buf�е����ݷ��ͳ�ȥ */
    CanTransmit_can1(buf, sizeof(buf),MIT_id_can1);
}
void CanComm_SendControlPara_can2(float f_p2, float f_v2, float f_kp2, float f_kd2, float f_t2,uint16_t id)
{
	uint16_t p2, v2, kp2, kd2, t2;
	uint8_t buf[8];

	uint16_t MIT_id_can2;	
	MIT_id_can2 = id;
	
    /* ��������Ĳ����ڶ���ķ�Χ�� */
    LIMIT_MIN_MAX(f_p2,  P_MIN,  P_MAX);    //Ŀ��λ��
    LIMIT_MIN_MAX(f_v2,  V_MIN,  V_MAX);    //Ŀ���ٶ�
    LIMIT_MIN_MAX(f_kp2, KP_MIN, KP_MAX);   //λ������
    LIMIT_MIN_MAX(f_kd2, KD_MIN, KD_MAX);   //�ٶ�����
    LIMIT_MIN_MAX(f_t2,  T_MIN,  T_MAX);    //����
    
    /* ����Э�飬��float��������ת�� */
    p2  = float_to_uint(f_p2,     P_MIN ,  P_MAX ,  16);            
    v2  = float_to_uint(f_v2,     V_MIN ,  V_MAX ,  12);
    kp2 = float_to_uint(f_kp2,    KP_MIN,  KP_MAX,  12);
    kd2 = float_to_uint(f_kd2,    KD_MIN,  KD_MAX,  12);
    t2  = float_to_uint(f_t2,     T_MIN ,  T_MAX ,  12);
    
    /* ���ݴ���Э�飬������ת��ΪCAN���������ֶ� */   //��Ϊ��8λ8λ���ڷ�������Ҫ�������
    buf[0] = p2>>8;
    buf[1] = p2&0xFF;
    buf[2] = v2>>4;
    buf[3] = ((v2&0xF)<<4)|(kp2>>8);
    buf[4] = kp2&0xFF;
    buf[5] = kd2>>4;
    buf[6] = ((kd2&0xF)<<4)|(t2>>8);
    buf[7] = t2&0xff;
    
    /* ͨ��CAN�ӿڰ�buf�е����ݷ��ͳ�ȥ */
    CanTransmit_can2(buf, sizeof(buf),MIT_id_can2);


}
void CanComm_ControlCmd(uint8_t cmd,uint16_t caan,uint16_t an)   //MIT�����ģʽ
{
    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    switch(cmd)
    {
        case CMD_MOTOR_MODE:  
            buf[7] = 0xFC;
            break;
        
        case CMD_RESET_MODE:
            buf[7] = 0xFD;
        break;
        
        case CMD_ZERO_POSITION:
            buf[7] = 0xFE;
        break;
        
        default:
        return; /* ֱ���˳����� */
    }
 
	  //���ģʽ���ó�һ����
		
		if(caan == 1)
		{
			CanTransmit_can1(buf, sizeof(buf),an);
		}
		if(caan == 2)
		{
			CanTransmit_can2(buf, sizeof(buf),an);
		}		
}




