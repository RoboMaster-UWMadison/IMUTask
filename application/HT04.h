#ifndef HT04_H
#define HT04_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

//MIT电机所需一下量
#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03
#define P_MIN -95.5f    // Radians
#define P_MAX 95.5f        
#define V_MIN -45.0f    // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f
//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
  
    int16_t given_current;
    int16_t real_current;
  
    uint8_t temperate;
    int16_t last_ecd;
  
    
    uint8_t hall;
    uint16_t angle;      // abs angle range:[0,8191]
    uint16_t last_angle; // abs angle range:[0,8191]
    uint16_t offset_angle;
    int32_t round_cnt;

    int32_t total_ecd;
    int32_t total_angle;
    uint32_t msg_cnt;
    float vx;
    float vy;
    float vz;
    uint16_t offset_ecd;

    int32_t ecd_raw_rate;
    int32_t rate_buf[5];
    uint8_t buf_cut;
    int32_t filter_rate;
} ht04_measure_t;


typedef struct{
	int8_t 		temperature;
	fp32	 		speed_rpm;
  int16_t  	real_current;
  uint16_t  position;
	fp32   	total_angle;
	fp32   	total_angle_last;
	
}m_rmd_t;

typedef struct
{
    float position;
    float velocity;
    float current;

} Moto_Dreame_Float_t;

//MIT 电机的速度
typedef struct
{
   float can1_01;
	 float can1_02;
	 float can2_01;
	 float can2_02;
} Mit_Velocity_type;
//MIT 电机的位置
typedef struct
{
   float can1_01;
	 float can1_02;
	 float can2_01;
	 float can2_02;
} Mit_Position_type;
//MIT 电机的电流
typedef struct
{
   float can1_01;
	 float can1_02;
	 float can2_01;
	 float can2_02;
} Mit_Current_type;

// MIT LQR 两个的轮子的速度
extern volatile float CurVelocity1; 
extern volatile float CurVelocity2;
//腿电机
extern Mit_Velocity_type Mit_receive_Velocity ; //四个电机的速度
extern Mit_Position_type Mit_receive_Position ;   //四个电机的位置
extern Mit_Current_type Mit_receive_Current;     //四个电机的电流


extern void CanComm_SendControlPara_can1(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint16_t id);
extern void CanComm_SendControlPara_can2(float f_p2, float f_v2, float f_kp2, float f_kd2, float f_t2,uint16_t id);
extern void CanComm_ControlCmd(uint8_t cmd,uint16_t caan,uint16_t an);
extern float CanComm_GetCurVelocity(void);

#endif
