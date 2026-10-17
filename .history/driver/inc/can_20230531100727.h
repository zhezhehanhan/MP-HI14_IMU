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

#define ID_MASTER_HB 0X01030000
#define ID_RESPOND_HB 0X00030100
#define ID_MASTER_RESET_POSITION 0X01010300
#define ID_RESPOND_RESET_POSITION 0X03010100
#define ID_MASTER_ACC 0X01010301
#define ID_RESPOND_ACC 0X03010101
#define ID_POSITION 0X03010102
#define ID_V 0X03010103

extern CORRECT Position_Correct; // ʵ�������������

u8 Position_Send(POSITION *position);
u8 V_Send(SPEED *speed);
u8 ACC_Send(float Accelerated_Speed);

void can_init(void);
void Answer_Master(int length, uint32_t ID, char Data1, char Data2, char Data3, char Data4, char Data5, char Data6);
/***********************************/
#endif
