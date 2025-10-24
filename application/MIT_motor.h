#ifndef __MIT_MOTOR_H
#define __MIT_MOTOR_H


#include "stdbool.h"
#include "user_lib.h"
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"

//MIT电机控制任务控制间隔 10ms
#define  MIT_MOTOR_CONTROL_TIME_MS 4

//USART1接收中断预留数据长度
#define USART_RX_BUF_LENGHT     128

//USART1返回数据长度
#define USART1_RX_BUF_LENGHT   9

//MIT 发送的力矩
typedef struct
{
   float can1_01;
	 float can1_02;
	 float can2_01;
	 float can2_02;
} Mit_send_Current_type;

//跳跃、飞坡状态定义
typedef enum
{
  JAF_NORMAL,   				//正常运动
	
	JAF_JUMP_INIT,				//跳跃准备
	JAF_JUMP_INIT_READY,	//完成跳跃准备
	JAF_JUMP_CANCEL,			//跳跃取消
	JAF_JUMP_START,				//开始跳跃
	JAF_JUMP_AIR,					//跳跃空中保护
	JAF_JUMP_WAIT,        //防止TOE数据异常造成二次跳跃
	
	JAF_FORWARD,					//飞坡
	JAF_FORWARD_AIR,			//飞坡空中保护
} jump_and_forward_mode;

//TOP反馈数据类型
typedef struct {
   int distance;
   int strength;
   char receiveComplete;
}TFmini;

void MotorControl_Start(uint16_t can,uint16_t n);
void MotorControl_PositionHandler(void);
void jump_and_forward(void);
void USART1_Callback(void);

 //任务
extern void mit_motor_task(void const *argu); 
extern void motor_8120_task(void const *argu);

//
extern uint8_t chassis_init_end;
extern float wheel_x;
extern float wheel_y;
extern float wheel_y_init;
extern float add_roll;

//腿部闭链五连杆正解算
extern float FK_wheel_x;
extern float FK_wheel_y;
extern float FK_wheel_x_last;
extern float FK_wheel_x_v;

//解算角度、角速度腿部等效值
extern float titl_leg;
extern float d_titl_leg;

//腿部电机控制参数
extern float Mit_send_pos_k;
extern float Mit_send_vel_k;

//跳跃、飞坡状态机
extern jump_and_forward_mode JAF_mode;

#endif


