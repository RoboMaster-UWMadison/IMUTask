#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "math.h"
#include "MIT_motor.h"
#include "remote_control.h"
#include "pid.h"

//float型tof距离
float tof_data_float;
TFmini tof_data;

//腿部末端位置期望值
//x 前后  y 上下
float wheel_x = 0.0f;
float wheel_y = 220.0f;
float wheel_y_init = 220.0f;

//机身roll轴控制参数
float add_roll= 0.0f;
//腿部电机位置发送数据
Mit_Position_type Mit_send_Position = {0};
Mit_Velocity_type Mit_send_Velocity = {0};
Mit_Current_type Mit_send_Current =   {0};
//腿部运动学逆解算错误标志
uint8_t Inverse_Kinematic_Error = 0;
//腿部电机控制参数
float Mit_send_pos_k = 20.0f;
float Mit_send_vel_k = 0.8f;
float Mit_send_torque = 0.0f;


int jump_flag=0;
int watch_mode=0;
int jump_flag2=0;
//在发送电机位置零前，需要把电机的所有控制参数设置为0
static void ZeroPosition(uint16_t can,uint16_t n)
{
	//根据不同的电机模式，发送不同的报文，给第7位，电机，零位校正，reset三种模式
  CanComm_ControlCmd(CMD_MOTOR_MODE,can,n);  
  osDelay(10);
	
  //参数置零
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

//启动电机控制
void MotorControl_Start(uint16_t cann,uint16_t nn)
{
	//先进入电机模式，然后把所有信息变0，然后再发零位校正模式，进行校正
	ZeroPosition(cann,nn); 
	CanComm_ControlCmd(CMD_ZERO_POSITION,cann,nn);  
}

//电机位置控制
void MotorControl_PositionHandler(void)
{
	//位置控制	
	if( rc_ctrl.rc.s[0] == RC_SW_DOWN ) //急停
	{
		CanComm_SendControlPara_can1( 0, 0, 0, 0, 0,0x01);	//左前
		CanComm_SendControlPara_can1( 0, 0, 0, 0, 0,0x02);	//左后
		CanComm_SendControlPara_can2( 0, 0, 0, 0, 0,0x01);	//右前
		CanComm_SendControlPara_can2( 0, 0, 0, 0, 0,0x02);	//右后
	}
	else if( rc_ctrl.rc.s[1] == RC_SW_MID || rc_ctrl.rc.s[1] == RC_SW_UP ) //腿部末端控制
	{
		if(Inverse_Kinematic_Error == 0)//逆解算无误，解算结果可输出
		{
			CanComm_SendControlPara_can1( Mit_send_Position.can1_01, Mit_send_Velocity.can1_01, Mit_send_pos_k, Mit_send_vel_k, -Mit_send_torque,0x01);
			CanComm_SendControlPara_can1( Mit_send_Position.can1_02, Mit_send_Velocity.can1_02, Mit_send_pos_k, Mit_send_vel_k, Mit_send_torque,0x02);
			CanComm_SendControlPara_can2( Mit_send_Position.can2_01, Mit_send_Velocity.can2_01, Mit_send_pos_k, Mit_send_vel_k, Mit_send_torque,0x01);
			CanComm_SendControlPara_can2( Mit_send_Position.can2_02, Mit_send_Velocity.can2_02, Mit_send_pos_k, Mit_send_vel_k, -Mit_send_torque,0x02);
		}
	}
	else //阻尼输出
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

//腿部电机零点设置完成标志
uint8_t chassis_init_end = 0;

//零点设置
void Set_leg_zero(void)
{
	//回零参数
	float Set_leg_zero_v = 1.0f;
	float Set_leg_zero_kv = 3.0f;
	float Set_leg_zero_I_line = 5.0f;
	
	//到位标志
	uint8_t can1_01_ok = 0;
	uint8_t can1_02_ok = 0;
	uint8_t can2_01_ok = 0;
	uint8_t can2_02_ok = 0;
	
	
	

	//电机初始化
	//多次循环确保初始化成功
	for(int i=0;i<50;i++)
	{
		MotorControl_Start(1,1);
		MotorControl_Start(2,1);
		MotorControl_Start(1,2);
		MotorControl_Start(2,2);
	}
	
	while( can1_01_ok == 0 || can1_02_ok == 0 || can2_01_ok == 0 || can2_02_ok == 0 )
	{
		//腿部匀速收缩，等待到达上置限位
		CanComm_SendControlPara_can1( 0, Set_leg_zero_v, 0, Set_leg_zero_kv, 0,0x01);   
		osDelay(1);
		CanComm_SendControlPara_can1( 0, -Set_leg_zero_v, 0, Set_leg_zero_kv, 0,0x02);   
		osDelay(1);
		CanComm_SendControlPara_can2( 0, -Set_leg_zero_v, 0, Set_leg_zero_kv, 0,0x01);   
		osDelay(1);
		CanComm_SendControlPara_can2( 0, Set_leg_zero_v, 0, Set_leg_zero_kv, 0,0x02);   
		osDelay(1);
		
		//到位判断
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
	
	//延时确保到位
	osDelay(200);
	
	//赋零点
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


//腿部各点定义
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

//腿部模型参数
float L1 = 150.0f,L4 = 150.0f;
float L2 = 288.0f,L3 = 288.0f;
float L5 = 150.0f;
//中间参数
float IK_a,IK_b,IK_c,IK_d,IK_e,IK_f;
float Alpha_11,Alpha_12,Alpha_21,Alpha_22;
float Alpha_1;//前侧关节角度
float Alpha_2;//后侧关节角度

//腿部闭环五连杆逆解算
void Inverse_Kinematic(float wheel_leg_x,float wheel_leg_y,float body_roll)
{
	float x = wheel_x;
	float y[2] = {0};
	y[0] = wheel_leg_y + body_roll; //左侧
	y[1] = wheel_leg_y - body_roll;	//右侧
	
	for(uint8_t i=0; i<2; i++)
	{
		//坐标系平移
		x = wheel_x + L5/2.0f;
		
		//中间参数
		IK_a = 2.0f * x * L1;
		IK_b = 2.0f * y[i] * L1;
		IK_c = x*x + y[i]*y[i] + L1*L1 - L2*L2;
		
		//左侧夹角
		Alpha_11 = 2 * atanf( (IK_b + sqrtf(IK_a*IK_a + IK_b*IK_b - IK_c*IK_c))/(IK_a + IK_c) );
		Alpha_12 = 2 * atanf( (IK_b - sqrtf(IK_a*IK_a + IK_b*IK_b - IK_c*IK_c))/(IK_a + IK_c) );
		
		//中间变量
		IK_d = 2.0f * L4 * (x - L5);
		IK_e = 2.0f * L4 * y[i];
		IK_f = (x - L5)*(x - L5) + L4*L4 + y[i]*y[i] - L3*L3;
		
		//右侧夹角
		Alpha_21 = 2 * atanf( (IK_e + sqrtf(IK_d*IK_d + IK_e*IK_e - IK_f*IK_f))/(IK_d + IK_f) );
		Alpha_22 = 2 * atanf( (IK_e - sqrtf(IK_d*IK_d + IK_e*IK_e - IK_f*IK_f))/(IK_d + IK_f) );
		
		//腿部关节角度解算
		Alpha_11 = 3.49f - Alpha_11;
		Alpha_12 = 3.49f - Alpha_12;
		Alpha_21 = 0.349f + Alpha_21;
		Alpha_22 = 0.349f + Alpha_22;
		
		//膝关节数据范围修正
		if(Alpha_11 >= 6.2832f)
			{Alpha_11 -= 6.2832f;}
		if(Alpha_22 >= 6.2832f)
			{Alpha_22 -= 6.2832f;}
			
		//合规数据选择
		Inverse_Kinematic_Error = 0;
		
		if((Alpha_11 > -0.1f) && (Alpha_11 < 1.55f))
			{Alpha_1 = Alpha_11;}
		else
		 {Inverse_Kinematic_Error = 1;}
			 
		if((Alpha_22 > -0.1f) && (Alpha_22 < 1.55f))
			{Alpha_2 = Alpha_22;}
		else
		 {Inverse_Kinematic_Error = 1;}
		
		if(Inverse_Kinematic_Error == 0)//数据正确，发送
		{
			if(i == 0)//左侧数据
			{
				Mit_send_Position.can1_01 = -1.0f * Alpha_2;
				Mit_send_Position.can1_02 = Alpha_1;
			}
			else if(i == 1)//右侧数据
			{
				Mit_send_Position.can2_01 = Alpha_2;		
				Mit_send_Position.can2_02 = -1.0f * Alpha_1;
			}
		}
	}
}

//参数
float sky_ground_range_1_1 = 8.5f;
float sky_ground_range_1_2 = 9.2f;
float sky_ground_range_2 = 8.5f;
//float sky_ground_range_2 = 12.0f;
float leg_current_total;
float leg_current_total_filter_out;
uint16_t sky_ground_flag = 0;

//腿部触地判断
uint8_t leg_statue_sky_or_ground()
{
	//根据腿部电流判断
	leg_current_total = abs_f(Mit_receive_Current.can1_01) + abs_f(Mit_receive_Current.can1_02) + abs_f(Mit_receive_Current.can2_01) + abs_f(Mit_receive_Current.can2_02);
	leg_current_total_filter_out = 1.0f / (1.0f + 0.1f) * leg_current_total_filter_out + 0.1f / (1.0f + 0.1f) * leg_current_total;
	if(JAF_mode == JAF_FORWARD)
	{
		if(leg_current_total_filter_out > sky_ground_range_1_1)
		{
			//触地
			sky_ground_flag = 1000;//调试标志位
			return 1;		
		}
		else
		{
			//离地
			sky_ground_flag = 0;
			return 0;			
		}
	}
	if(JAF_mode == JAF_FORWARD_AIR)
	{
		if(leg_current_total_filter_out > sky_ground_range_1_2)
		{
			//触地
			sky_ground_flag = 1000;//调试标志位
			return 1;		
		}
		else
		{
			//离地
			sky_ground_flag = 0;
			return 0;			
		}
	}
	if(JAF_mode == JAF_JUMP_AIR)
	{
		if(leg_current_total_filter_out > sky_ground_range_2)
		{
			//触地
			sky_ground_flag = 1000;//调试标志位
			return 1;		
		}
		else
		{
			//离地
			sky_ground_flag = 0;
			return 0;			
		}
	}
}


//起跳信息标志时间
uint32_t jump_time = 0;

//跳跃、飞坡状态机
jump_and_forward_mode JAF_mode;

//跳跃、飞坡状态判断
void jaf_mode_change()
{
	if(JAF_mode == JAF_JUMP_AIR || JAF_mode == JAF_FORWARD_AIR)//空中状态
	{
		//检测到机器人落地
		if(leg_statue_sky_or_ground() == 1)
		{
			
			jump_flag++;
			if(jump_flag>200)
			{
				//针对TOF自动跳跃模式
				//防止TOF数据异常造成连续跳跃
				if(rc_ctrl.rc.ch[4] > 600)
				{
					JAF_mode = JAF_JUMP_WAIT;
				}
				else
				{
					//退回正常运动模式
					JAF_mode = JAF_NORMAL;
				}
			}
		}
	}
	else//地面状态
	{
		if(JAF_mode == JAF_FORWARD)//飞坡状态
		{
			//如果检测到腿部腾空，切换为空中保护模式
			if(leg_statue_sky_or_ground() == 0)
			{
				JAF_mode = JAF_FORWARD_AIR;
			}
		}
		
		if(JAF_mode == JAF_JUMP_INIT)//跳跃初始化
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
		
		if(JAF_mode == JAF_JUMP_INIT_READY)//跳跃准备完成,开始进一步指令判别
		{
			if(rc_ctrl.rc.ch[4] > 600  ||  rc_ctrl.rc.ch[4] < -600 )//计算拨轮回零时间
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
				if(jump_time > 40)//慢速松波轮，跳跃取消
				{
					JAF_mode = JAF_JUMP_CANCEL;
					jump_time = 0;
					
				}
				else//快速松波轮，开始跳跃
				{
					JAF_mode = JAF_JUMP_START;
					jump_time = 0;
				}
			}
		}
		
		if(JAF_mode == JAF_JUMP_WAIT)//退出TOF防误触模式
		{
			if(rc_ctrl.rc.ch[4] == 0)
			{
				JAF_mode = JAF_NORMAL;
			}
		}
		
		//遥控器状态跳转
		if(rc_ctrl.rc.ch[4] > 600 && JAF_mode != JAF_JUMP_INIT_READY  && JAF_mode != JAF_JUMP_WAIT)//波轮下拉到底
		{
			//模式控制
			//若此处为JAF_JUMP_INIT模式，则下拉波轮为腿部收拢至底部，缓放为取消跳跃，松手为跳跃
			//若此处为JAF_JUMP_START模式，则跳跃初始状态为正常机体自平衡状态，下拉波轮即为跳跃，但要注意跳跃时波轮必须松手，否则会连续跳跃
			
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
			else if(rc_ctrl.rc.ch[4] < -600 && JAF_mode != JAF_JUMP_INIT_READY  && JAF_mode != JAF_JUMP_WAIT)//波轮下拉到底
		{
			JAF_mode = JAF_JUMP_START;
			watch_mode=1;
			
		}
		
//		else if(rc_ctrl.rc.ch[4] < - 600 && JAF_mode != JAF_FORWARD_AIR)//波轮上推到顶,切换为飞坡模式
//		{
//			JAF_mode = JAF_FORWARD;
//		}
		else if(rc_ctrl.rc.ch[4] == 0 && JAF_mode == JAF_FORWARD)//波轮居中,切换为正常运动
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

//参数
uint8_t jump_finish_flag = 0;
uint8_t jump_phase_1 = 0;
uint8_t jump_phase_2 = 0;

//侧向波轮
//向后拉到底为跳跃预备动作，慢放为取消跳跃，松手为开始跳跃
//向前推到底为飞坡模式，控制机器速度
void jump_and_forward(void)
{
	//状态转换
	jaf_mode_change();
	
	//动作处理
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
		//缓速上升
		Mit_send_pos_k = 20.0f;
		Mit_send_vel_k = 3.0f;
		wheel_y = wheel_y_init;
		//到位，恢复正常运动状态
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
			if(jump_phase_1 == 0 && FK_wheel_y < 360)//最大功率伸腿
			{
				Mit_send_pos_k = 0.0f;
				Mit_send_vel_k = 0.0f;
				Mit_send_torque = 18.0f;
			}
			if(jump_phase_1 == 0 && FK_wheel_y >= 360)//伸腿到位，开始缩腿
			{
				jump_phase_1 = 1;
			}
			
			if(jump_phase_1 == 1 && FK_wheel_y > 205)//纯位置控制缩腿
			{
				Mit_send_pos_k = 20.0f;
				Mit_send_vel_k = 0.0f;
				Mit_send_torque = 0.0f;
				wheel_y = 185.0f;
			}
			if(jump_phase_1 == 1 && FK_wheel_y <= 205)//纯位置控制缩腿到位，开始带阻尼位置控制缩腿
			{
				jump_phase_2 = 1;
			}
			
			if(jump_phase_2 == 1)
			{
				Mit_send_pos_k = 20.0f;
				Mit_send_vel_k = 1.5f;
				Mit_send_torque = 0.0f;
				wheel_y = 185.0f;
				//跳跃流程完成，收腿到位且稳定，进入空中环节
				if( abs_f(Mit_receive_Velocity.can1_01)+abs_f(Mit_receive_Velocity.can1_02)+abs_f(Mit_receive_Velocity.can2_01)+abs_f(Mit_receive_Velocity.can2_02) < 0.3f )
				{
					jump_finish_flag = 1;
				}
			}
		}
		
		if(jump_finish_flag == 1)
		{
			JAF_mode = JAF_JUMP_AIR;
			//标志位清零，准备下一次跳跃
			jump_finish_flag = 0;
			jump_phase_1 = 0;
			jump_phase_2 = 0;
		}
	}
}

//中间参数
float f_angle1,f_angle2;
float xa,ya,xc,yc;
float LengthAC,theta1;
float Aa,Bb,Cc;
//正运动学解算结果数据
float FK_wheel_x,FK_wheel_y;
float FK_wheel_x_last,FK_wheel_x_v;
//解算角度、角速度腿部等效值
float titl_leg_last,titl_leg,d_titl_leg;

//腿部正运动学解算
void Forward_Kinematic(float angle1_forward,float angle_back)
{
	//角度变换
	f_angle1 = 3.49f - angle_back;
	f_angle2 = abs_f(angle1_forward) - 0.349f;
	
	//中间变量
	xa = L1 * cos(f_angle1);
	ya = L1 * sin(f_angle1);
	xc = L4 * cos(f_angle2) + L5;
	yc = L4 * sin(f_angle2);
	
	LengthAC = sqrtf( (xc-xa)*(xc-xa) + (yc-ya)*(yc-ya) );
	
	Aa = 2.0f * L2 * (xc - xa);
	Bb = 2.0f * L2 * (yc - ya);
	Cc = L2*L2 + LengthAC*LengthAC - L3*L3;
	theta1 = 2 * atanf( (Bb + sqrtf( Aa*Aa + Bb*Bb - Cc*Cc )) / (Aa + Cc) );
	
	//正运动学解算结果
	FK_wheel_x_last = FK_wheel_x;
	
	FK_wheel_x = xa + L2*cos(theta1) - L5/2.0f;
	FK_wheel_y = ya + L2*sin(theta1);
	
	FK_wheel_x_v = (FK_wheel_x - FK_wheel_x_last) * 100.0f;
	
	//解算角度、角速度腿部等效值
	titl_leg_last = titl_leg;
	titl_leg = (-1.0f) * atanf( FK_wheel_x/FK_wheel_y );
	d_titl_leg = 100.0f * (titl_leg - titl_leg_last );
}




//TOF数据获取
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
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_LISR_TCIF1);

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);

}

//USART1串口接收回调函数
void USART1_Callback(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
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
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = USART_RX_BUF_LENGHT - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = USART_RX_BUF_LENGHT;


            //set memory buffer 1
            //设定缓冲区1
						hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
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
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = USART_RX_BUF_LENGHT - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = USART_RX_BUF_LENGHT;

            //set memory buffer 0
            //设定缓冲区0
						hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == USART1_RX_BUF_LENGHT)
            {
							tof_data_receive(usart1_buf[1], &tof_data);
            }
        }
    }
}

//MIT电机task
void mit_motor_task(void const *argu)
{
	//任务间隔绝对延时函数赋初值
	static portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	//串口通讯初始化
	usart1_init(usart1_buf[0], usart1_buf[1], USART_RX_BUF_LENGHT);

	//电机零点校正
	Set_leg_zero();

	while(1)
	{
		//tof距离
		tof_data_float = (float)tof_data.distance;
		
		//tof自带跳跃控制速度
		if(rc_ctrl.rc.ch[4] > 600 && JAF_mode != JAF_JUMP_INIT_READY  && JAF_mode != JAF_JUMP_WAIT)//波轮下拉到底
		{
			tof_jump_flag = 1;
		}
		else
		{
			tof_jump_flag = 0;
		}
		
		//跳跃、飞坡控制
		if(chassis_rc->rc.s[1] == RC_SW_MID && body_banlance_init == 1)
		{
			jump_and_forward();
		}
		
		//逆运动学解算，求解腿部电机控制指令
		Inverse_Kinematic(wheel_x,wheel_y,add_roll);
		
		//腿部控制指令发送
		MotorControl_PositionHandler();
		
		//正运动学解算，求解当前轮端位置
		Forward_Kinematic(Mit_receive_Position.can1_01,Mit_receive_Position.can1_02);

		//系统延时
		vTaskDelayUntil(&xLastWakeTime,MIT_MOTOR_CONTROL_TIME_MS);
	}
}



////USART1串口接收回调函数
//void USART1_Callback(void)
//{
//    if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
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
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = USART_RX_BUF_LENGHT - hdma_usart1_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //重新设定数据长度
//            hdma_usart1_rx.Instance->NDTR = USART_RX_BUF_LENGHT;


//            //set memory buffer 1
//            //设定缓冲区1
//						hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
//            
//            //enable DMA
//            //使能DMA
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
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = USART_RX_BUF_LENGHT - hdma_usart1_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //重新设定数据长度
//            hdma_usart1_rx.Instance->NDTR = USART_RX_BUF_LENGHT;

//            //set memory buffer 0
//            //设定缓冲区0
//						hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
//            
//            //enable DMA
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);

//            if(this_time_rx_len == USART1_RX_BUF_LENGHT)
//            {
//							tof_data_receive(usart1_buf[1], &tof_data);
//            }
//        }
//    }
//}

