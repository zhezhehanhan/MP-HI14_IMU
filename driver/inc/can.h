/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-05-25 23:08:16
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-05-31 10:07:27
 * @FilePath: \ER_改通信版\driver\inc\can.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _CAN_H
#define _CAN_H
#include "PositionCount.h"
#include "stm32f4xx.h"

//#define ID_MASTER_HB 0X03010301     //这是主控的心跳包ID，心跳包ID是指示主控是否在线的包
//#define ID_RESPOND_HB 0x01010301    //这是从控的心跳包ID，心跳包ID是指示主控是否在线的包
//#define ID_MASTER_HB 0X03010101     //这是主控的心跳包ID，心跳包ID是指示主控是否在线的包
#define ID_RESPOND_HB 0x00000100   //这是从控的心跳包ID，心跳包ID是指示主控是否在线的包
#define ID_MASTER_HB  0X00000200    //这是主控的心跳包ID，心跳包ID是指示主控是否在线的包

// 主控
#define ID_MASTER_ACC 0X01010304  //这是主控的加速度ID，加速度ID是指示主控是否要求从控加速度的包
#define ID_MASTER_RESET_POSITION 0x01010301   //这是主控的复位位置ID，复位位置ID是指示主控是否要求从控复位位置的包

// 从控反馈上报
#define ID_RESPOND_RESET_POSITION 0X03010101 
#define ID_POSITION 0x03010102  //这是从控的位置ID，位置ID是指示从控发送位置的包
#define ID_V 0x03010103  //这是从控的速度ID，速度ID是指示从控发送速度的包
#define ID_RESPOND_ACC 0X03010104  //这是从控的加速度ID，加速度ID是指示主控是否要求从控加速度的包




extern CORRECT Position_Correct; // ʵ�������������

u8 Position_Send(POSITION *position);
u8 V_Send(SPEED *speed);
u8 ACC_Send(float Accelerated_Speed);

void can_init(void);
void Answer_Master(int length, uint32_t ID, char Data1, char Data2, char Data3, char Data4, char Data5, char Data6, char Data7, char Data8);
/***********************************/
#endif
