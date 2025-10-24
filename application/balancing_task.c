#include "balancing_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "math.h"
#include <arm_math.h>  
#include "HT04.h"
#include "motor_CAN.h"
#include "remote_control.h"
#include "pid_dji.h"
#include "INS_task.h"
#include "volt.h"
//state vector
fp32 phi,phi_dot,s,s_dot;
state x;
Odom odom;
void balancing_task(void const *pvParameters);
void balancing_feedback(void);
void Set_leg_zero(void);

const RC_ctrl_t *ctrl;
int16_t current_chassis[2]={0,0};
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

void MotorControl_Start(uint16_t cann,uint16_t n)
{
	//先进入电机模式，然后把所有信息变0，然后再发零位校正模式，进行校正
  CanComm_ControlCmd(CMD_RESET_MODE,cann,n);  
	ZeroPosition(cann,n); 
	CanComm_ControlCmd(CMD_ZERO_POSITION,cann,n);  
}

//零点设置
	uint8_t can1_01_ok = 0;
	uint8_t can1_02_ok = 0;
	uint8_t can2_01_ok = 0;
	uint8_t can2_02_ok = 0;	
void Set_leg_zero(void)
{
	//回零参数
	float Set_leg_zero_v = 0.5f;
	float Set_leg_zero_kv = 5.0f;
	float Set_leg_zero_I_line = 5.5f;	
	
		for(int i=0;i<50;i++)
	{
		MotorControl_Start(1,1);
		MotorControl_Start(1,2);
    MotorControl_Start(2,1);
		MotorControl_Start(2,2);
    osDelay(1);
	}	
	
	while( can1_01_ok == 0 || can1_02_ok == 0 || can2_01_ok == 0 || can2_02_ok == 0)
	{
		
		CanComm_SendControlPara_can1( 0, -Set_leg_zero_v, 0, Set_leg_zero_kv, 0,0x01);   
		osDelay(1);
		CanComm_SendControlPara_can1( 0, Set_leg_zero_v, 0, Set_leg_zero_kv, 0,0x02);   
		osDelay(1);
		CanComm_SendControlPara_can2( 0, -Set_leg_zero_v, 0, Set_leg_zero_kv, 0,0x01);   
		osDelay(1);
		CanComm_SendControlPara_can2( 0, Set_leg_zero_v, 0, Set_leg_zero_kv, 0,0x02);   
		osDelay(1);		
		//到位判断
		if(Mit_receive_Current.can1_01 < - Set_leg_zero_I_line)
		{
			can1_01_ok = 1;
		}
		if(Mit_receive_Current.can1_02 > Set_leg_zero_I_line)
		{
			can1_02_ok = 1;
		}
		if(Mit_receive_Current.can2_01 < - Set_leg_zero_I_line)
		{
			can2_01_ok = 1;
		}
		if(Mit_receive_Current.can2_02 > Set_leg_zero_I_line)
		{
			can2_02_ok = 1;
		}
		
	}	
	for(int i=0;i<10;i++)
	{
		CanComm_ControlCmd(CMD_ZERO_POSITION,1,1);
		CanComm_ControlCmd(CMD_ZERO_POSITION,1,2);
    CanComm_ControlCmd(CMD_ZERO_POSITION,2,1);
		CanComm_ControlCmd(CMD_ZERO_POSITION,2,2);
    osDelay(10);
	}	
	
}
void Balancing_init()
{
	ctrl = get_remote_control_point();
	while(1)
	{
		CanComm_SendControlPara_can1( 0.0f, 0.0f, 0.0f ,0.0f, 0.0f,0x01);   
		osDelay(1);
		CanComm_SendControlPara_can1( 0.0f, 0.0f,  0.0f,0.0f, 0.0f,0x02);   
		osDelay(1); 
    CanComm_SendControlPara_can2( 0.0f, 0.0f,  0.0f ,0.0f, 0.0f,0x01);   
		osDelay(1);
		CanComm_SendControlPara_can2( 0.0f, 0.0f,  0.0f,0.0f, 0.0f,0x02);   
		osDelay(1); 
		if(ctrl->rc.s[1] == 1) break;
	}
  Set_leg_zero();  
}




//中间变量
fp32 sigma;
fp32 x1,x2,y1,y2;
fp32 y1m2,y1a2,x1a2,x1m2;//m:minus a:add
fp32 y1m2s,x1m2s;//s: suqare

#define L1_as L1_a*L1_a
#define L2_as L2_a*L2_a
#define L1_ds L1_d*L1_d
#define L2_ds L2_d*L2_d
#define RIGHT 1
#define LEFT  0

  
  
//腿部末端相对机器人坐标
fp32 xe[2],ye[2],L[2],theta[2],theta_degree[2];
fp32 J_r[2][2] = {{0,0},{0,0}};
fp32 J_l[2][2] = {{0,0},{0,0}};
fp32 J_r_invT[2][2] = {{0,0},{0,0}};
fp32 J_l_invT[2][2] = {{0,0},{0,0}};
fp32 Tor11=0,Tor12=0,Tor21=0,Tor22=0;
fp32 sin_phi1,cos_phi1,sin_phi2,cos_phi2;
static inline void invT_2x2(fp32 J[2][2], fp32 J_invT[2][2])
{
    fp32 a = J[0][0], b = J[0][1];
    fp32 c = J[1][0], d = J[1][1];

    fp32 det = a * d - b * c; // 行列式
    if (fabsf(det) < 1e-6f) det = (det >= 0 ? 1e-6f : -1e-6f); // 防止除零

    fp32 inv_det = 1.0f / det;

    J_invT[0][0] =  d * inv_det;
    J_invT[0][1] = -c * inv_det;
    J_invT[1][0] = -b * inv_det;
    J_invT[1][1] =  a * inv_det;
}
void Forward_Kinematic(fp32 phi_1, fp32 phi_2, char side)
{
  phi_1 = 3.54159f - phi_1;
  phi_2 = 3.54159f - phi_2;

  x1 = -L1_a + L1_u * cosf(phi_1);
  x2 =  L2_a - L2_u * cosf(phi_2);
  y1 =  L1_u * sinf(phi_1);
  y2 =  L2_u * sinf(phi_2);

  y1m2  = y1 - y2;
  y1a2  = y1 + y2;
  x1a2  = x1 + x2;
  x1m2  = x1 - x2;

  y1m2s = y1m2 * y1m2;
  x1m2s = x1m2 * x1m2;

  if (side) {
    fp32 sigma = sqrtf(L1_ds / (y1m2s + x1m2s) - 0.25f);

    xe[0] = 0.5f * x1a2 + sigma * y1m2;
    ye[0] = 0.5f * y1a2 - sigma * x1m2;
    L[0]  = sqrtf(xe[0] * xe[0] + ye[0] * ye[0]);

    theta[0] = atan2f(xe[0], ye[0]);
    theta_degree[0] = theta[0] * 57.3f;
    x.theta_ll = x.phi + theta[0];
    
    fp32 den10 = (-(xe[0] + x1) * sinf(phi_1) + (ye[0] - y1) * cosf(phi_1));
    fp32 den11 = ((xe[0] - x2) * sinf(phi_2) + (ye[0] - y2) * cosf(phi_2));
    fp32 sin_theta_ll = sinf(x.theta_ll);
    fp32 cos_theta_ll = cosf(x.theta_ll);

    J_l[0][0] = ((xe[0] + x1) * sin_theta_ll + (ye[0] - y1) * cos_theta_ll) / den10 / L1_u;
    J_l[0][1] =  L[0] * ((xe[0] + x1) * cos_theta_ll - (ye[0] - y1) * sin_theta_ll) / den10 / L1_u;
    J_l[1][0] = ((xe[0] - x2) * sin_theta_ll + (ye[0] - y2) * cos_theta_ll) / den11 / L2_u;
    J_l[1][1] =  L[0] * ((xe[0] - x2) * cos_theta_ll - (ye[0] - y2) * sin_theta_ll) / den11 / L2_u;
    invT_2x2(J_l, J_l_invT);

  } else {
    fp32 sigma = sqrtf(L1_ds / (y1m2s + x1m2s) - 0.25f);

    xe[1] = -(0.5f * x1a2 + sigma * y1m2);                                                                                                   
    ye[1] = 0.5f * y1a2 - sigma * x1m2;
    L[1]  = sqrtf(xe[1] * xe[1] + ye[1] * ye[1]);

    theta[1] = atan2f(xe[1], ye[1]);
    theta_degree[1] = theta[1] * 57.3f;
    x.theta_lr = x.phi + theta[1];

    fp32 den10 = (-(xe[1] + x1) * sinf(phi_1) + (ye[1] - y1) * cosf(phi_1));
    fp32 den11 = ((xe[1] - x2) * sinf(phi_2) + (ye[1] - y2) * cosf(phi_2));
    fp32 sin_theta_lr = sinf(x.theta_lr);
    fp32 cos_theta_lr = cosf(x.theta_lr);

    J_r[0][0] = ((xe[1] + x1) * sin_theta_lr + (ye[1] - y1) * cos_theta_lr) / den10 / L1_u;
    J_r[0][1] =  L[1] * ((xe[1] + x1) * cos_theta_lr - (ye[1] - y1) * sin_theta_lr) / den10 / L1_u;
    J_r[1][0] = ((xe[1] - x2) * sin_theta_lr + (ye[1] - y2) * cos_theta_lr) / den11 / L2_u;
    J_r[1][1] =  L[1] * ((xe[1] - x2) * cos_theta_lr - (ye[1] - y2) * sin_theta_lr) / den11 / L2_u;
    invT_2x2(J_r, J_r_invT);
  }
}


void Inverse_Kinematic(fp32 F,fp32 T,char side)
{
    if(side)
  {
    Tor21 = F*J_l_invT[0][0] + T*J_l_invT[0][1];
    Tor22 = F*J_l_invT[1][0] + T*J_l_invT[1][1];
  }
  else
  {
    Tor11 = F*J_r_invT[0][0] + T*J_r_invT[0][1];
    Tor12 = F*J_r_invT[1][0] + T*J_r_invT[1][1];
  }
  
}
// 9025编码器里程计计数
int32_t n_r = 0; //cycle count
int32_t n_l = 0; //cycle count
static inline fp32 s_9025(int32_t *n, int16_t ecd, int16_t ecd_last)
{
    int32_t d = (int32_t)ecd - (int32_t)ecd_last;

    if (d < -8192)
        (*n)++;
    else if (d > 8192)
        (*n)--;

    return ((*n) * 16384 + ecd)*MOTOR_ECD_TO_ANGLE;  
}
//static inline void odom_update(Odom *o, float dL, float dR)
//{
//    const float yaw = INS_angle_fusion[1];   // IMU 融合角（弧度）
//    float ds = 0.5f * (dL + dR);             // 车体中心位移 [m]
//    o->x += ds * cosf(yaw);
//    o->y += ds * sinf(yaw);
//}

void Balancing_feedback()
{
		
		x.s = 0; 
		x.s_l =  s_9025(&n_l,motor_chassis[0].ecd,motor_chassis[0].last_ecd);
   	x.s_r = -s_9025(&n_r,motor_chassis[1].ecd,motor_chassis[1].last_ecd);
		x.s_dot_l = motor_chassis[0].speed_rpm * LK9025_RPM_TO_MS;
		x.s_dot_r = -motor_chassis[1].speed_rpm * LK9025_RPM_TO_MS;
		x.phi = INS_angle_fusion[1]/57.3f;
		x.phi_dot = INS_gyro[0];
    
    
    Forward_Kinematic(Mit_receive_Position.can2_01,-Mit_receive_Position.can2_02,RIGHT);
    Forward_Kinematic(Mit_receive_Position.can1_01,-Mit_receive_Position.can1_02,LEFT);
    Inverse_Kinematic(1.0f,0.0f,RIGHT);
    Inverse_Kinematic(1.0f,0.0f,LEFT);
	if(ctrl->rc.s[0] == 3)
    {
			 x.s_r = 0.0f;
			 x.s_l = 0.0f;
    }
}
float loop_us;
static uint8_t cnt = 0;
static inline void delay_us(uint32_t us){
    uint32_t t0 = DWT->CYCCNT;
    uint32_t dt = us * (SystemCoreClock/1000000u);
    while ((DWT->CYCCNT - t0) < dt) { __NOP(); }
}

void balancing_task(void const *pvParameters)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  const TickType_t T = pdMS_TO_TICKS(1); // 1kHz
  TickType_t last = xTaskGetTickCount();
	Balancing_init();
  
  while(1)
  {
		fp32 kp = 1.0f;
    uint32_t t0 = DWT->CYCCNT;  // 记录开始
      
    Balancing_feedback();
		current_chassis[0] =  (int16_t)(-63.2f*x.s_l - 90.155f*x.s_dot_l - 700.0f*x.phi - 285.3f*x.phi_dot);
		current_chassis[1] = -(int16_t)(-63.2f*x.s_r - 90.155f*x.s_dot_r - 700.0f*x.phi - 285.3f*x.phi_dot);
    if (++cnt >= 3) 
    {          // 每3个1ms周期 = 约300Hz
      CanComm_SendControlPara_can1(0,0,kp,0,0,0x01);
      CanComm_SendControlPara_can1(0,0,kp,0,0,0x02);
      CanComm_SendControlPara_can2(0,0,kp,0,0,0x01);
      CanComm_SendControlPara_can2(0,0,kp,0,0,0x02);
    }
    
    if(ctrl->rc.s[0] == 1)
    {
      
			CAN_cmd_chassis(current_chassis[0],current_chassis[1]);
		  //CAN_cmd_chassis(0,0);
    }
		else
    {
      CAN_cmd_chassis(0,0);
    }
		volt.Volt_Data.Data[0] = Tor11;
		volt.Volt_Data.Data[1] = Tor12;
    volt.Volt_Data.Data[2] = Tor21;
    volt.Volt_Data.Data[3] = Tor22;
    uint32_t t1 = DWT->CYCCNT;  // 记录结束    
    loop_us = (float)(t1 - t0) / (SystemCoreClock / 1000000.0f);
		vTaskDelayUntil(&last, T);
  }
}



