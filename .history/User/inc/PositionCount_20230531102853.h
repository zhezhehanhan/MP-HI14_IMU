#ifndef _POSITIONCOUNT_H
#define _POSITIONCOUNT_H

#include "MyMath.h"
#include "filter.h"
#include "stm32f4xx.h"
#include "math.h"
#include "kalman.h"
#include "time2.h"
#include "QEPtime.h"
#include "led.h"
#include "ucos_ii.h"

//����ṹ����������
typedef struct pot
{
    float X;            //��������������ϵԭ���ڳ�������ϵ�е�X���꣬cm
    float Y;            //��������������ϵԭ���ڳ�������ϵ�е�Y���꣬cm
    float AngleDeg;     //�����˵���̬��(��������ϵX��ת����������ϵX��)���Ƕ��ƣ�˳������
    float Sum_Encoder1; //����1�߹���·��
    float Sum_Encoder2; //����2�߹���·��
} POSITION;

typedef struct spd
{
    float Vx; //�������ڳ�������ϵX�᷽��ķ��ٶȣ�cm/s
    float Vy; //�������ڳ�������ϵY�᷽��ķ��ٶȣ�cm/s
    float Vw; //�����˽��ٶȣ���/s,˳������
    float V;  //�����˵ĺ��ٶ�
} SPEED;

typedef struct correct
{
    float real_x;
    float dt_del_x;
    float real_y;
    float dt_del_y;
} CORRECT;

extern int ang_div;
数据会精确到mm，所以先将position->X乘以10，再加上0.5（也就是进行四舍五入），然后将结果强制转换为int16_t数据会精确到mm，所以先将position->X乘以10，再加上0.5（也就是进行四舍五入），然后将结果强制转换为int16_t
extern float CM_PER_CNT_1; // 2*pi*Radius_Wheel_1/(4*����������) //��λϵͳX��
extern float CM_PER_CNT_2; // 2*pi*Radius_Wheel_2/(4*����������) //��λϵͳY��

//����Ĳ���ȡ�������̵İ�װλ��
extern float OO_distance; //��������ϵԭ���붨λϵͳ����ϵԭ��ľ��룬��λcm
extern float Angle_XX;    //��������ϵx��ת��λϵͳ����ϵx��ĽǶȣ������ƣ�˳������
extern float Angle_OO;    //��������ϵX��ת����������ϵԭ��ָ��λϵͳ����ϵԭ�����ߵĽǶȣ������ƣ�˳������

//������λ�����ٶ�����
extern POSITION Position_Now_mp;
extern POSITION Position_Now_rob;
extern SPEED Speed_Now;

extern float speed_V, speedv;
extern INT32U time;
extern float Accelerated_Speed;

extern float SCALE_TAB[1501];

void CountAngel(void);
void CountPosition(void);
void UpdatePosition(void);
void CountSpeed(void);
void GYRO_Reset(void);

#endif
