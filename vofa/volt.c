/*
 * @Name: lolt.c
 * @Description: volt+��λ���ײ����,����Volt_Sendware����DATA�ṹ��+VOLT�����幩����
 * @Author: source
 * @Copyright: SixuanRobomasterLab
 */
#include "volt.h"
#include "stdio.h"
volt_un volt;
/**
 * @Name: Volt_Sendware
 * @Description:���� Э��     ��scheduler_task�ļ����е���Volt_Sendware());����
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void Volt_Sendware()
{
		volt.Volt_Data.End=0x7f800000;/*��ƽ�βЭ��*/		
	HAL_UART_Transmit(&huart1,volt.Sent,sizeof(volt.Sent),HAL_MAX_DELAY);/*��������*/
}
