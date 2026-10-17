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
