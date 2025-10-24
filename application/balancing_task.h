#ifndef BALANCING_TASK
#define BALANCING_TASK

#include "struct_typedef.h"
//�Ȳ�ģ�Ͳ��� ������������ģ��
#define L1_a 0.075f
#define L2_a 0.075f
#define L1_u 0.15f
#define L2_u 0.15f
#define L1_d 0.288f
#define L2_d 0.288f
//remote constant
//��̼ƽṹ��
typedef struct { float x, y; } Odom;
typedef struct 
{ float s,s_dot,
				phi,phi_dot,
				theta_ll,theta_ll_dot, 
				theta_lr,theta_lr_dot,
				theta_b, theta_b_dot,
				s_dot_l,s_dot_r,
				s_l,s_r;
} state;
//MIT ���͵�����
typedef struct
{
   float can1_01;
	 float can1_02;
	 float can2_01;
	 float can2_02;
} Mit_send_Current_type;

void Forward_Kinematic(fp32 phi_1,fp32 phi_2,char side);
void Inverse_Kinematic(fp32 F,fp32 T,char side);
extern void balancing_task(void const *pvParameters);
#endif
