#ifndef __GLOBAL_DEFINE_H
#define __GLOBAL_DEFINE_H
#include "can.h"

#define pi 3.1415926f
/*���Ӽ�ͨ�����*/
#define COMMUNICATE_CAN  hcan2
#define Master_To_Cloud_ID 0x100 //���ظ���̨��
#define Master_To_Slaver_ID 0x101  //���ظ����ط�
#define SlaverID 0x102  //���ر�ʶ��ID
#define CloudID  0x103  //��̨��ʶ��ID
/*��Դ����λ��غ궨��*/
#define Distance_To_Island  10.f   //ȡ��ʱ��������Դ��֮��ľ���
#define Error_Between_Sharp   //������������ȡ�����������ֵ����Ϊ�Ƿ�Ҫת��yaw����ж�����

/*ң����ģʽ�ַ����*/
#define STARTING  0  //����ʱ����
#define RUNNING   1  //����ʱ����
#define ENDING    2  //����ʱ����

#endif
