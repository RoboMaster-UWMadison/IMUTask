#ifndef __MIT_MOTOR_H
#define __MIT_MOTOR_H


#include "stdbool.h"
#include "user_lib.h"
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"

//MIT�������������Ƽ�� 10ms
#define  MIT_MOTOR_CONTROL_TIME_MS 4

//USART1�����ж�Ԥ�����ݳ���
#define USART_RX_BUF_LENGHT     128

//USART1�������ݳ���
#define USART1_RX_BUF_LENGHT   9

//MIT ���͵�����
typedef struct
{
   float can1_01;
	 float can1_02;
	 float can2_01;
	 float can2_02;
} Mit_send_Current_type;

//��Ծ������״̬����
typedef enum
{
  JAF_NORMAL,   				//�����˶�
	
	JAF_JUMP_INIT,				//��Ծ׼��
	JAF_JUMP_INIT_READY,	//�����Ծ׼��
	JAF_JUMP_CANCEL,			//��Ծȡ��
	JAF_JUMP_START,				//��ʼ��Ծ
	JAF_JUMP_AIR,					//��Ծ���б���
	JAF_JUMP_WAIT,        //��ֹTOE�����쳣��ɶ�����Ծ
	
	JAF_FORWARD,					//����
	JAF_FORWARD_AIR,			//���¿��б���
} jump_and_forward_mode;

//TOP������������
typedef struct {
   int distance;
   int strength;
   char receiveComplete;
}TFmini;

void MotorControl_Start(uint16_t can,uint16_t n);
void MotorControl_PositionHandler(void);
void jump_and_forward(void);
void USART1_Callback(void);

 //����
extern void mit_motor_task(void const *argu); 
extern void motor_8120_task(void const *argu);

//
extern uint8_t chassis_init_end;
extern float wheel_x;
extern float wheel_y;
extern float wheel_y_init;
extern float add_roll;

//�Ȳ�����������������
extern float FK_wheel_x;
extern float FK_wheel_y;
extern float FK_wheel_x_last;
extern float FK_wheel_x_v;

//����Ƕȡ����ٶ��Ȳ���Чֵ
extern float titl_leg;
extern float d_titl_leg;

//�Ȳ�������Ʋ���
extern float Mit_send_pos_k;
extern float Mit_send_vel_k;

//��Ծ������״̬��
extern jump_and_forward_mode JAF_mode;

#endif


