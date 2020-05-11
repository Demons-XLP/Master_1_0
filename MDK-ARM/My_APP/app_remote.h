#ifndef __APP_REMOTE_H
#define __APP_REMOTE_H

#include "bsp_motor.hpp"
void Remote_Control_Handle();  //ң�������ƺ������
void Master_To_Slaver(void);  //���ظ���̨����Ϣ
void Master_To_Cloud();  //���ظ����ط���Ϣ

extern Motor_t DJI_Motor_3508;  //�������
extern pid PID_Chassis_Speed;  //���̵��PID
extern pid PID_Chassis_Follow_OUT;  //���̸���PID�⻷���������ǽǶȣ���������
extern pid PID_Chassis_Follow_IN;  //���̸���PID�ڻ����������ǽǼ��ٶ�
extern chassis Chassis_Engineer;  //�������������

#endif
