#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "math.h"
#include "MIT_motor.h"
#include "remote_control.h"
#include "pid.h"

//float��tof����
float tof_data_float;
TFmini tof_data;

//�Ȳ�ĩ��λ������ֵ
//x ǰ��  y ����
float wheel_x = 0.0f;
float wheel_y = 220.0f;
float wheel_y_init = 220.0f;

//����roll����Ʋ���
float add_roll= 0.0f;
//�Ȳ����λ�÷�������
Mit_Position_type Mit_send_Position = {0};
Mit_Velocity_type Mit_send_Velocity = {0};
Mit_Current_type Mit_send_Current =   {0};
//�Ȳ��˶�ѧ���������־
uint8_t Inverse_Kinematic_Error = 0;
//�Ȳ�������Ʋ���
float Mit_send_pos_k = 20.0f;
float Mit_send_vel_k = 0.8f;
float Mit_send_torque = 0.0f;


int jump_flag=0;
int watch_mode=0;
int jump_flag2=0;
//�ڷ��͵��λ����ǰ����Ҫ�ѵ�������п��Ʋ�������Ϊ0
static void ZeroPosition(uint16_t can,uint16_t n)
{
	//���ݲ�ͬ�ĵ��ģʽ�����Ͳ�ͬ�ı��ģ�����7λ���������λУ����reset����ģʽ
  CanComm_ControlCmd(CMD_MOTOR_MODE,can,n);  
  osDelay(10);
	
  //��������
	if(can == 1)
	{
		CanComm_SendControlPara_can1(0,0,0,0,0,n);
		osDelay(1);
	}
	if(can == 2)
	{
		CanComm_SendControlPara_can2(0,0,0,0,0,n);
		osDelay(1);
	}
}

//�����������
void MotorControl_Start(uint16_t cann,uint16_t nn)
{
	//�Ƚ�����ģʽ��Ȼ���������Ϣ��0��Ȼ���ٷ���λУ��ģʽ������У��
	ZeroPosition(cann,nn); 
	CanComm_ControlCmd(CMD_ZERO_POSITION,cann,nn);  
}

//���λ�ÿ���
void MotorControl_PositionHandler(void)
{
	//λ�ÿ���	
	if( rc_ctrl.rc.s[0] == RC_SW_DOWN ) //��ͣ
	{
		CanComm_SendControlPara_can1( 0, 0, 0, 0, 0,0x01);	//��ǰ
		CanComm_SendControlPara_can1( 0, 0, 0, 0, 0,0x02);	//���
		CanComm_SendControlPara_can2( 0, 0, 0, 0, 0,0x01);	//��ǰ
		CanComm_SendControlPara_can2( 0, 0, 0, 0, 0,0x02);	//�Һ�
	}
	else if( rc_ctrl.rc.s[1] == RC_SW_MID || rc_ctrl.rc.s[1] == RC_SW_UP ) //�Ȳ�ĩ�˿���
	{
		if(Inverse_Kinematic_Error == 0)//��������󣬽����������
		{
			CanComm_SendControlPara_can1( Mit_send_Position.can1_01, Mit_send_Velocity.can1_01, Mit_send_pos_k, Mit_send_vel_k, -Mit_send_torque,0x01);
			CanComm_SendControlPara_can1( Mit_send_Position.can1_02, Mit_send_Velocity.can1_02, Mit_send_pos_k, Mit_send_vel_k, Mit_send_torque,0x02);
			CanComm_SendControlPara_can2( Mit_send_Position.can2_01, Mit_send_Velocity.can2_01, Mit_send_pos_k, Mit_send_vel_k, Mit_send_torque,0x01);
			CanComm_SendControlPara_can2( Mit_send_Position.can2_02, Mit_send_Velocity.can2_02, Mit_send_pos_k, Mit_send_vel_k, -Mit_send_torque,0x02);
		}
	}
	else //�������
	{
		CanComm_SendControlPara_can1( 0, 0, 0, 0.6f, 0, 0x01);
		osDelay(1);
		CanComm_SendControlPara_can1( 0, 0, 0, 0.6f, 0, 0x02);
		osDelay(1);
		CanComm_SendControlPara_can2( 0, 0, 0, 0.6f, 0, 0x01);
		osDelay(1);
		CanComm_SendControlPara_can2( 0, 0, 0, 0.6f, 0, 0x02);
		osDelay(1);
	}
}

//�Ȳ�������������ɱ�־
uint8_t chassis_init_end = 0;

//�������
void Set_leg_zero(void)
{
	//�������
	float Set_leg_zero_v = 1.0f;
	float Set_leg_zero_kv = 3.0f;
	float Set_leg_zero_I_line = 5.0f;
	
	//��λ��־
	uint8_t can1_01_ok = 0;
	uint8_t can1_02_ok = 0;
	uint8_t can2_01_ok = 0;
	uint8_t can2_02_ok = 0;
	
	
	

	//�����ʼ��
	//���ѭ��ȷ����ʼ���ɹ�
	for(int i=0;i<50;i++)
	{
		MotorControl_Start(1,1);
		MotorControl_Start(2,1);
		MotorControl_Start(1,2);
		MotorControl_Start(2,2);
	}
	
	while( can1_01_ok == 0 || can1_02_ok == 0 || can2_01_ok == 0 || can2_02_ok == 0 )
	{
		//�Ȳ������������ȴ�����������λ
		CanComm_SendControlPara_can1( 0, Set_leg_zero_v, 0, Set_leg_zero_kv, 0,0x01);   
		osDelay(1);
		CanComm_SendControlPara_can1( 0, -Set_leg_zero_v, 0, Set_leg_zero_kv, 0,0x02);   
		osDelay(1);
		CanComm_SendControlPara_can2( 0, -Set_leg_zero_v, 0, Set_leg_zero_kv, 0,0x01);   
		osDelay(1);
		CanComm_SendControlPara_can2( 0, Set_leg_zero_v, 0, Set_leg_zero_kv, 0,0x02);   
		osDelay(1);
		
		//��λ�ж�
		if(Mit_receive_Current.can1_01 > Set_leg_zero_I_line)
		{
			can1_01_ok = 1;
		}
		if(Mit_receive_Current.can1_02 < -Set_leg_zero_I_line)
		{
			can1_02_ok = 1;
		}
		if(Mit_receive_Current.can2_01 < -Set_leg_zero_I_line)
		{
			can2_01_ok = 1;
		}
		if(Mit_receive_Current.can2_02 > Set_leg_zero_I_line)
		{
			can2_02_ok = 1;
		}
	}
	
	//��ʱȷ����λ
	osDelay(200);
	
	//�����
	CanComm_ControlCmd(CMD_ZERO_POSITION,1,1);
	osDelay(1);
	CanComm_ControlCmd(CMD_ZERO_POSITION,1,2);
	osDelay(1);
	CanComm_ControlCmd(CMD_ZERO_POSITION,2,1);
	osDelay(1);
	CanComm_ControlCmd(CMD_ZERO_POSITION,2,2);
	osDelay(1);
	
	chassis_init_end = 1;
}


//�Ȳ����㶨��
/*********************************************
                 y |   / E
                   |  /  L6
                   | / B
                   |/\ 
                   /  \
               L2 /|   \ L3
                 / |    \
              A /  |     \ C
                \   |     /
              L1 \ |    / L4
          ---------|---------->
                  O  L5 D      x
*********************************************/

//�Ȳ�ģ�Ͳ���
float L1 = 150.0f,L4 = 150.0f;
float L2 = 288.0f,L3 = 288.0f;
float L5 = 150.0f;
//�м����
float IK_a,IK_b,IK_c,IK_d,IK_e,IK_f;
float Alpha_11,Alpha_12,Alpha_21,Alpha_22;
float Alpha_1;//ǰ��ؽڽǶ�
float Alpha_2;//���ؽڽǶ�

//�Ȳ��ջ������������
void Inverse_Kinematic(float wheel_leg_x,float wheel_leg_y,float body_roll)
{
	float x = wheel_x;
	float y[2] = {0};
	y[0] = wheel_leg_y + body_roll; //���
	y[1] = wheel_leg_y - body_roll;	//�Ҳ�
	
	for(uint8_t i=0; i<2; i++)
	{
		//����ϵƽ��
		x = wheel_x + L5/2.0f;
		
		//�м����
		IK_a = 2.0f * x * L1;
		IK_b = 2.0f * y[i] * L1;
		IK_c = x*x + y[i]*y[i] + L1*L1 - L2*L2;
		
		//���н�
		Alpha_11 = 2 * atanf( (IK_b + sqrtf(IK_a*IK_a + IK_b*IK_b - IK_c*IK_c))/(IK_a + IK_c) );
		Alpha_12 = 2 * atanf( (IK_b - sqrtf(IK_a*IK_a + IK_b*IK_b - IK_c*IK_c))/(IK_a + IK_c) );
		
		//�м����
		IK_d = 2.0f * L4 * (x - L5);
		IK_e = 2.0f * L4 * y[i];
		IK_f = (x - L5)*(x - L5) + L4*L4 + y[i]*y[i] - L3*L3;
		
		//�Ҳ�н�
		Alpha_21 = 2 * atanf( (IK_e + sqrtf(IK_d*IK_d + IK_e*IK_e - IK_f*IK_f))/(IK_d + IK_f) );
		Alpha_22 = 2 * atanf( (IK_e - sqrtf(IK_d*IK_d + IK_e*IK_e - IK_f*IK_f))/(IK_d + IK_f) );
		
		//�Ȳ��ؽڽǶȽ���
		Alpha_11 = 3.49f - Alpha_11;
		Alpha_12 = 3.49f - Alpha_12;
		Alpha_21 = 0.349f + Alpha_21;
		Alpha_22 = 0.349f + Alpha_22;
		
		//ϥ�ؽ����ݷ�Χ����
		if(Alpha_11 >= 6.2832f)
			{Alpha_11 -= 6.2832f;}
		if(Alpha_22 >= 6.2832f)
			{Alpha_22 -= 6.2832f;}
			
		//�Ϲ�����ѡ��
		Inverse_Kinematic_Error = 0;
		
		if((Alpha_11 > -0.1f) && (Alpha_11 < 1.55f))
			{Alpha_1 = Alpha_11;}
		else
		 {Inverse_Kinematic_Error = 1;}
			 
		if((Alpha_22 > -0.1f) && (Alpha_22 < 1.55f))
			{Alpha_2 = Alpha_22;}
		else
		 {Inverse_Kinematic_Error = 1;}
		
		if(Inverse_Kinematic_Error == 0)//������ȷ������
		{
			if(i == 0)//�������
			{
				Mit_send_Position.can1_01 = -1.0f * Alpha_2;
				Mit_send_Position.can1_02 = Alpha_1;
			}
			else if(i == 1)//�Ҳ�����
			{
				Mit_send_Position.can2_01 = Alpha_2;		
				Mit_send_Position.can2_02 = -1.0f * Alpha_1;
			}
		}
	}
}

//����
float sky_ground_range_1_1 = 8.5f;
float sky_ground_range_1_2 = 9.2f;
float sky_ground_range_2 = 8.5f;
//float sky_ground_range_2 = 12.0f;
float leg_current_total;
float leg_current_total_filter_out;
uint16_t sky_ground_flag = 0;

//�Ȳ������ж�
uint8_t leg_statue_sky_or_ground()
{
	//�����Ȳ������ж�
	leg_current_total = abs_f(Mit_receive_Current.can1_01) + abs_f(Mit_receive_Current.can1_02) + abs_f(Mit_receive_Current.can2_01) + abs_f(Mit_receive_Current.can2_02);
	leg_current_total_filter_out = 1.0f / (1.0f + 0.1f) * leg_current_total_filter_out + 0.1f / (1.0f + 0.1f) * leg_current_total;
	if(JAF_mode == JAF_FORWARD)
	{
		if(leg_current_total_filter_out > sky_ground_range_1_1)
		{
			//����
			sky_ground_flag = 1000;//���Ա�־λ
			return 1;		
		}
		else
		{
			//���
			sky_ground_flag = 0;
			return 0;			
		}
	}
	if(JAF_mode == JAF_FORWARD_AIR)
	{
		if(leg_current_total_filter_out > sky_ground_range_1_2)
		{
			//����
			sky_ground_flag = 1000;//���Ա�־λ
			return 1;		
		}
		else
		{
			//���
			sky_ground_flag = 0;
			return 0;			
		}
	}
	if(JAF_mode == JAF_JUMP_AIR)
	{
		if(leg_current_total_filter_out > sky_ground_range_2)
		{
			//����
			sky_ground_flag = 1000;//���Ա�־λ
			return 1;		
		}
		else
		{
			//���
			sky_ground_flag = 0;
			return 0;			
		}
	}
}


//������Ϣ��־ʱ��
uint32_t jump_time = 0;

//��Ծ������״̬��
jump_and_forward_mode JAF_mode;

//��Ծ������״̬�ж�
void jaf_mode_change()
{
	if(JAF_mode == JAF_JUMP_AIR || JAF_mode == JAF_FORWARD_AIR)//����״̬
	{
		//��⵽���������
		if(leg_statue_sky_or_ground() == 1)
		{
			
			jump_flag++;
			if(jump_flag>200)
			{
				//���TOF�Զ���Ծģʽ
				//��ֹTOF�����쳣���������Ծ
				if(rc_ctrl.rc.ch[4] > 600)
				{
					JAF_mode = JAF_JUMP_WAIT;
				}
				else
				{
					//�˻������˶�ģʽ
					JAF_mode = JAF_NORMAL;
				}
			}
		}
	}
	else//����״̬
	{
		if(JAF_mode == JAF_FORWARD)//����״̬
		{
			//�����⵽�Ȳ��ڿգ��л�Ϊ���б���ģʽ
			if(leg_statue_sky_or_ground() == 0)
			{
				JAF_mode = JAF_FORWARD_AIR;
			}
		}
		
		if(JAF_mode == JAF_JUMP_INIT)//��Ծ��ʼ��
		{
			if(rc_ctrl.rc.ch[4] == 0)
			{
				JAF_mode = JAF_JUMP_CANCEL;
			}
			if( abs_f(Mit_receive_Position.can1_01)+abs_f(Mit_receive_Position.can1_02)+abs_f(Mit_receive_Position.can2_01)+abs_f(Mit_receive_Position.can2_02) < 0.1f )
			{
				JAF_mode = JAF_JUMP_INIT_READY;
			}
		}
		
		if(JAF_mode == JAF_JUMP_INIT_READY)//��Ծ׼�����,��ʼ��һ��ָ���б�
		{
			if(rc_ctrl.rc.ch[4] > 600  ||  rc_ctrl.rc.ch[4] < -600 )//���㲦�ֻ���ʱ��
			{
				jump_time = 0;
			}
			else
			{
				jump_time ++;
				jump_flag=0;
			}
			
			if(rc_ctrl.rc.ch[4] == 0)
			{
				if(jump_time > 40)//�����ɲ��֣���Ծȡ��
				{
					JAF_mode = JAF_JUMP_CANCEL;
					jump_time = 0;
					
				}
				else//�����ɲ��֣���ʼ��Ծ
				{
					JAF_mode = JAF_JUMP_START;
					jump_time = 0;
				}
			}
		}
		
		if(JAF_mode == JAF_JUMP_WAIT)//�˳�TOF����ģʽ
		{
			if(rc_ctrl.rc.ch[4] == 0)
			{
				JAF_mode = JAF_NORMAL;
			}
		}
		
		//ң����״̬��ת
		if(rc_ctrl.rc.ch[4] > 600 && JAF_mode != JAF_JUMP_INIT_READY  && JAF_mode != JAF_JUMP_WAIT)//������������
		{
			//ģʽ����
			//���˴�ΪJAF_JUMP_INITģʽ������������Ϊ�Ȳ���£���ײ�������Ϊȡ����Ծ������Ϊ��Ծ
			//���˴�ΪJAF_JUMP_STARTģʽ������Ծ��ʼ״̬Ϊ����������ƽ��״̬���������ּ�Ϊ��Ծ����Ҫע����Ծʱ���ֱ������֣������������Ծ
			
			//JAF_mode = JAF_JUMP_INIT;
			watch_mode=3;
			jump_flag2++;
			if(jump_flag2>200)
			{
			if(tof_data_float <= 38)
			{
				JAF_mode = JAF_JUMP_START;
			}
			}
			
			
		}
			else if(rc_ctrl.rc.ch[4] < -600 && JAF_mode != JAF_JUMP_INIT_READY  && JAF_mode != JAF_JUMP_WAIT)//������������
		{
			JAF_mode = JAF_JUMP_START;
			watch_mode=1;
			
		}
		
//		else if(rc_ctrl.rc.ch[4] < - 600 && JAF_mode != JAF_FORWARD_AIR)//�������Ƶ���,�л�Ϊ����ģʽ
//		{
//			JAF_mode = JAF_FORWARD;
//		}
		else if(rc_ctrl.rc.ch[4] == 0 && JAF_mode == JAF_FORWARD)//���־���,�л�Ϊ�����˶�
		{
			JAF_mode = JAF_NORMAL;
		}
		else
		{
			watch_mode=0;
			jump_flag2=0;
		}
	}
}

//����
uint8_t jump_finish_flag = 0;
uint8_t jump_phase_1 = 0;
uint8_t jump_phase_2 = 0;

//������
//���������Ϊ��ԾԤ������������Ϊȡ����Ծ������Ϊ��ʼ��Ծ
//��ǰ�Ƶ���Ϊ����ģʽ�����ƻ����ٶ�
void jump_and_forward(void)
{
	//״̬ת��
	jaf_mode_change();
	
	//��������
	if(JAF_mode == JAF_NORMAL)
	{
		Mit_send_pos_k = 20.0f;
		Mit_send_vel_k = 1.0f;
		wheel_y = wheel_y_init;
	}
	if(JAF_mode == JAF_FORWARD)
	{
		Mit_send_pos_k = 20.0f;
		Mit_send_vel_k = 2.0f;
		wheel_y = wheel_y_init+80;
	}
	if(JAF_mode == JAF_FORWARD_AIR)
	{
		Mit_send_pos_k = 20.0f;
		Mit_send_vel_k = 0.0f;
		wheel_y = wheel_y_init+30;
	}
	if(JAF_mode == JAF_JUMP_INIT || JAF_mode == JAF_JUMP_INIT_READY)
	{
		Mit_send_pos_k = 0.0f;
		Mit_send_vel_k = 1.5f;
	}
	if(JAF_mode == JAF_JUMP_AIR)
	{
		Mit_send_pos_k = 20.0f;
		Mit_send_vel_k = 2.0f;
		wheel_y = wheel_y_init - 30.0f;
	}
	if(JAF_mode == JAF_JUMP_CANCEL)
	{
		//��������
		Mit_send_pos_k = 20.0f;
		Mit_send_vel_k = 3.0f;
		wheel_y = wheel_y_init;
		//��λ���ָ������˶�״̬
		if(abs_f(FK_wheel_y - wheel_y_init) < 25)
		{
			JAF_mode = JAF_NORMAL;
		}
	}
	if(JAF_mode == JAF_JUMP_WAIT)
	{
		Mit_send_pos_k = 20.0f;
		Mit_send_vel_k = 1.0f;
		wheel_y = wheel_y_init;
	}
	if(JAF_mode == JAF_JUMP_START)
	{
		if(jump_finish_flag == 0)
		{
			if(jump_phase_1 == 0 && FK_wheel_y < 360)//���������
			{
				Mit_send_pos_k = 0.0f;
				Mit_send_vel_k = 0.0f;
				Mit_send_torque = 18.0f;
			}
			if(jump_phase_1 == 0 && FK_wheel_y >= 360)//���ȵ�λ����ʼ����
			{
				jump_phase_1 = 1;
			}
			
			if(jump_phase_1 == 1 && FK_wheel_y > 205)//��λ�ÿ�������
			{
				Mit_send_pos_k = 20.0f;
				Mit_send_vel_k = 0.0f;
				Mit_send_torque = 0.0f;
				wheel_y = 185.0f;
			}
			if(jump_phase_1 == 1 && FK_wheel_y <= 205)//��λ�ÿ������ȵ�λ����ʼ������λ�ÿ�������
			{
				jump_phase_2 = 1;
			}
			
			if(jump_phase_2 == 1)
			{
				Mit_send_pos_k = 20.0f;
				Mit_send_vel_k = 1.5f;
				Mit_send_torque = 0.0f;
				wheel_y = 185.0f;
				//��Ծ������ɣ����ȵ�λ���ȶ���������л���
				if( abs_f(Mit_receive_Velocity.can1_01)+abs_f(Mit_receive_Velocity.can1_02)+abs_f(Mit_receive_Velocity.can2_01)+abs_f(Mit_receive_Velocity.can2_02) < 0.3f )
				{
					jump_finish_flag = 1;
				}
			}
		}
		
		if(jump_finish_flag == 1)
		{
			JAF_mode = JAF_JUMP_AIR;
			//��־λ���㣬׼����һ����Ծ
			jump_finish_flag = 0;
			jump_phase_1 = 0;
			jump_phase_2 = 0;
		}
	}
}

//�м����
float f_angle1,f_angle2;
float xa,ya,xc,yc;
float LengthAC,theta1;
float Aa,Bb,Cc;
//���˶�ѧ����������
float FK_wheel_x,FK_wheel_y;
float FK_wheel_x_last,FK_wheel_x_v;
//����Ƕȡ����ٶ��Ȳ���Чֵ
float titl_leg_last,titl_leg,d_titl_leg;

//�Ȳ����˶�ѧ����
void Forward_Kinematic(float angle1_forward,float angle_back)
{
	//�Ƕȱ任
	f_angle1 = 3.49f - angle_back;
	f_angle2 = abs_f(angle1_forward) - 0.349f;
	
	//�м����
	xa = L1 * cos(f_angle1);
	ya = L1 * sin(f_angle1);
	xc = L4 * cos(f_angle2) + L5;
	yc = L4 * sin(f_angle2);
	
	LengthAC = sqrtf( (xc-xa)*(xc-xa) + (yc-ya)*(yc-ya) );
	
	Aa = 2.0f * L2 * (xc - xa);
	Bb = 2.0f * L2 * (yc - ya);
	Cc = L2*L2 + LengthAC*LengthAC - L3*L3;
	theta1 = 2 * atanf( (Bb + sqrtf( Aa*Aa + Bb*Bb - Cc*Cc )) / (Aa + Cc) );
	
	//���˶�ѧ������
	FK_wheel_x_last = FK_wheel_x;
	
	FK_wheel_x = xa + L2*cos(theta1) - L5/2.0f;
	FK_wheel_y = ya + L2*sin(theta1);
	
	FK_wheel_x_v = (FK_wheel_x - FK_wheel_x_last) * 100.0f;
	
	//����Ƕȡ����ٶ��Ȳ���Чֵ
	titl_leg_last = titl_leg;
	titl_leg = (-1.0f) * atanf( FK_wheel_x/FK_wheel_y );
	d_titl_leg = 100.0f * (titl_leg - titl_leg_last );
}




//TOF���ݻ�ȡ
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
static uint8_t usart1_buf[2][USART_RX_BUF_LENGHT];

static void tof_data_receive(uint8_t *rx_buf, TFmini* data)
{
	unsigned char j = 0;
  unsigned int checksum = 0;
	for(j = 0; j < 8; j++) 
	{
		checksum += rx_buf[j];
	}
	if(rx_buf[8] == (checksum % 256))
	{
		data->distance = rx_buf[2] + rx_buf[3] * 256;
		data->strength = rx_buf[4] + rx_buf[5] * 256;
		data->receiveComplete = 1;
	}
	else
	{
		data->receiveComplete = 0;
	}
}

void usart1_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //ʹ��DMA���ڽ��պͷ���
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_LISR_TCIF1);

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
    //�ڴ滺����1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //�ڴ滺����2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //���ݳ���
    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, dma_buf_num);

    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);


    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);

}

//USART1���ڽ��ջص�����
void USART1_Callback(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = USART_RX_BUF_LENGHT - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart1_rx.Instance->NDTR = USART_RX_BUF_LENGHT;


            //set memory buffer 1
            //�趨������1
						hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == USART1_RX_BUF_LENGHT)
            {
							tof_data_receive(usart1_buf[0], &tof_data);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = USART_RX_BUF_LENGHT - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart1_rx.Instance->NDTR = USART_RX_BUF_LENGHT;

            //set memory buffer 0
            //�趨������0
						hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == USART1_RX_BUF_LENGHT)
            {
							tof_data_receive(usart1_buf[1], &tof_data);
            }
        }
    }
}

//MIT���task
void mit_motor_task(void const *argu)
{
	//������������ʱ��������ֵ
	static portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	//����ͨѶ��ʼ��
	usart1_init(usart1_buf[0], usart1_buf[1], USART_RX_BUF_LENGHT);

	//������У��
	Set_leg_zero();

	while(1)
	{
		//tof����
		tof_data_float = (float)tof_data.distance;
		
		//tof�Դ���Ծ�����ٶ�
		if(rc_ctrl.rc.ch[4] > 600 && JAF_mode != JAF_JUMP_INIT_READY  && JAF_mode != JAF_JUMP_WAIT)//������������
		{
			tof_jump_flag = 1;
		}
		else
		{
			tof_jump_flag = 0;
		}
		
		//��Ծ�����¿���
		if(chassis_rc->rc.s[1] == RC_SW_MID && body_banlance_init == 1)
		{
			jump_and_forward();
		}
		
		//���˶�ѧ���㣬����Ȳ��������ָ��
		Inverse_Kinematic(wheel_x,wheel_y,add_roll);
		
		//�Ȳ�����ָ���
		MotorControl_PositionHandler();
		
		//���˶�ѧ���㣬��⵱ǰ�ֶ�λ��
		Forward_Kinematic(Mit_receive_Position.can1_01,Mit_receive_Position.can1_02);

		//ϵͳ��ʱ
		vTaskDelayUntil(&xLastWakeTime,MIT_MOTOR_CONTROL_TIME_MS);
	}
}



////USART1���ڽ��ջص�����
//void USART1_Callback(void)
//{
//    if(huart1.Instance->SR & UART_FLAG_RXNE)//���յ�����
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart1);
//    }
//    else if(USART1->SR & UART_FLAG_IDLE)
//    {
//        static uint16_t this_time_rx_len = 0;

//        __HAL_UART_CLEAR_PEFLAG(&huart1);

//        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            /* Current memory buffer used is Memory 0 */
//    
//            //disable DMA
//            //ʧЧDMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
//            this_time_rx_len = USART_RX_BUF_LENGHT - hdma_usart1_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //�����趨���ݳ���
//            hdma_usart1_rx.Instance->NDTR = USART_RX_BUF_LENGHT;


//            //set memory buffer 1
//            //�趨������1
//						hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
//            
//            //enable DMA
//            //ʹ��DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);

//            if(this_time_rx_len == USART1_RX_BUF_LENGHT)
//            {
//							tof_data_receive(usart1_buf[0], &tof_data);
//            }
//        }
//        else
//        {
//            /* Current memory buffer used is Memory 1 */
//            //disable DMA
//            //ʧЧDMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
//            this_time_rx_len = USART_RX_BUF_LENGHT - hdma_usart1_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //�����趨���ݳ���
//            hdma_usart1_rx.Instance->NDTR = USART_RX_BUF_LENGHT;

//            //set memory buffer 0
//            //�趨������0
//						hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
//            
//            //enable DMA
//            //ʹ��DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);

//            if(this_time_rx_len == USART1_RX_BUF_LENGHT)
//            {
//							tof_data_receive(usart1_buf[1], &tof_data);
//            }
//        }
//    }
//}

