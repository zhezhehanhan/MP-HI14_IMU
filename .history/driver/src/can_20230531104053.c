#include "main.h"
#include "can.h"

void can_init(void) // can的初始化
{
	CAN_InitTypeDef can;
	CAN_FilterInitTypeDef can_filter;
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 打开GPIOA的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);  // 打开CAN1的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);  // 打开CAN2的时钟

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1); // PA11的CAN1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1); // PA12的CAN1

	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &gpio);

	nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	CAN_DeInit(CAN1);
	CAN_StructInit(&can);

	can.CAN_TTCM = DISABLE;
	can.CAN_ABOM = DISABLE;
	can.CAN_AWUM = DISABLE;
	can.CAN_NART = DISABLE;
	can.CAN_RFLM = DISABLE;
	can.CAN_TXFP = ENABLE;
	can.CAN_Mode = CAN_Mode_Normal;
	can.CAN_SJW = CAN_SJW_1tq;
	can.CAN_BS1 = CAN_BS1_9tq;
	can.CAN_BS2 = CAN_BS2_4tq;
	can.CAN_Prescaler = 3; // CAN BaudRate 42/(1+9+4)/3=1Mbps
	CAN_Init(CAN1, &can);

	can_filter.CAN_FilterNumber = 0;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh = (u16)(((ID_MASTER_HB << 3)) >> 16);
	can_filter.CAN_FilterIdLow = (u16)(((ID_MASTER_HB << 3)) >> 0);
	can_filter.CAN_FilterMaskIdHigh = (uint16_t)((uint32_t)(0X1FFFFFFFu << 3) >> 16); // 1111 1111 1111 1111
	can_filter.CAN_FilterMaskIdLow = (uint16_t)((uint32_t)(0X1FFFFFFFu << 3) >> 0);	  // 1111 1111 1111 1000
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&can_filter);

	can_filter.CAN_FilterNumber = 1;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;	// 标识符屏蔽模式
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit; // 宽度
	can_filter.CAN_FilterIdHigh = (u16)(((0X01010300 << 3)) >> 16);
	can_filter.CAN_FilterIdLow = (u16)(((0X01010300 << 3)) >> 0);
	can_filter.CAN_FilterMaskIdHigh = (uint16_t)((uint32_t)(0X1FFFFF00u << 3) >> 16);
	can_filter.CAN_FilterMaskIdLow = (uint16_t)((uint32_t)(0X1FFFFF00u << 3) >> 0);
	// 以上4句，限制只接受ID为0x0100的报文
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&can_filter);

	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

u8 Position_Send(POSITION *position) // 给主控发送位置数据
{
	CanTxMsg tx_message;
	tx_message.ExtId = ID_POSITION; // 报文ID号,位置信息反馈
	tx_message.DLC = 0x08;			// 数据字节数
	tx_message.RTR = CAN_RTR_Data;	// 数据帧
	tx_message.IDE = CAN_Id_Extended;

	tx_message.Data[1] = (uint8_t)((int16_t)(position->X * 10+ 0.5f)); // 精确到mm
	tx_message.Data[0] = (uint8_t)((int16_t)(position->X * 10+ 0.5f) >> 8);
	tx_message.Data[3] = (uint8_t)((int16_t)(position->Y * 10 + 0.5f));
	tx_message.Data[2] = (uint8_t)((int16_t)(position->Y * 10 + 0.5f) >> 8);
	tx_message.Data[5] = (uint8_t)((int16_t)(position->AngleDeg * 10 + 0.5f));
	tx_message.Data[4] = (uint8_t)((int16_t)(position->AngleDeg * 10 + 0.5f) >> 8);
	if (CAN_Transmit(CAN1, &tx_message) == CAN_TxStatus_NoMailBox)
		return 1;
	else
		return 0;
}

u8 V_Send(SPEED *speed) // 给主控发送速度数据
{
	CanTxMsg tx_message;
	tx_message.ExtId = ID_V;	   // 报文ID号
	tx_message.DLC = 0x08;		   // 数据字节数
	tx_message.RTR = CAN_RTR_Data; // 数据帧
	tx_message.IDE = CAN_Id_Extended;

	tx_message.Data[1] = (uint8_t)((int16_t)(speed->Vx * 10 + 0.5f)); // 精确到mm
	tx_message.Data[0] = (uint8_t)((int16_t)(speed->Vx * 10 + 0.5f) >> 8);
	tx_message.Data[3] = (uint8_t)((int16_t)(speed->Vy * 10 + 0.5f));
	tx_message.Data[2] = (uint8_t)((int16_t)(speed->Vy * 10 + 0.5f) >> 8);
	tx_message.Data[5] = (uint8_t)((int16_t)(speed->V * 10 + 0.5f));
	tx_message.Data[4] = (uint8_t)((int16_t)(speed->V * 10 + 0.5f) >> 8);
	tx_message.Data[7] = (uint8_t)((int16_t)(speed->Vw * 10 + 0.5f));
	tx_message.Data[6] = (uint8_t)((int16_t)(speed->Vw * 10 + 0.5f) >> 8);
	if (CAN_Transmit(CAN1, &tx_message) == CAN_TxStatus_NoMailBox)
		return 1;
	else
		return 0;
}

u8 ACC_Send(float Accelerated_Speed) // 给主控发送加速度数据,100ms
{
	CanTxMsg tx_message;
	tx_message.ExtId = ID_RESPOND_ACC; // 报文ID号
	tx_message.DLC = 0x08;			   // 数据字节数
	tx_message.RTR = CAN_RTR_Data;	   // 数据帧
	tx_message.IDE = CAN_Id_Extended;

	tx_message.Data[1] = (uint8_t)((int16_t)(Accelerated_Speed + 0.5f)); // 精确到mm
	tx_message.Data[0] = (uint8_t)((int16_t)(Accelerated_Speed + 0.5f) >> 8);
	if (CAN_Transmit(CAN1, &tx_message) == CAN_TxStatus_NoMailBox)
		return 1;
	else
		return 0;
}

void Answer_Master(int length, uint32_t ID, char Data0, char Data1, char Data2, char Data3, char Data5, char Data6)
{
	CanTxMsg tx_message;
	tx_message.ExtId = ID;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.IDE = CAN_Id_Extended;
	tx_message.DLC = length;
	tx_message.Data[0] = Data1;
	tx_message.Data[1] = Data2;
	tx_message.Data[2] = Data3;
	tx_message.Data[3] = Data4;
	tx_message.Data[4] = Data5;
	tx_message.Data[5] = Data6;
	CAN_Transmit(CAN1, &tx_message);
}

u16 HB_count = 0;
u8 acc_flag = 0;
// FIXME : 之前的位置信息乘10发送，这次的通信协议里没有乘10
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg rx_message;
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
		HB_count++;
		switch (rx_message.ExtId)
		{
		case ID_MASTER_HB:
			Answer_Master(6, ID_MASTER_HB, 0, 0, (uint8_t)(HB_count >> 8), (uint8_t)(HB_count & 0xff), 'H', 'B');
			HB_count = 0;
			break;

		case ID_MASTER_ACC:
			if (rx_message.Data[2] == 1)
				acc_flag = 1;
			if (rx_message.Data[2] == 0)
				acc_flag = 0;
			break;

		case ID_MASTER_RESET_POSITION:
			if (rx_message.Data[1] & 0x01)
			{
				Position_Now_mp.AngleDeg = (double)((s16)(((u16)rx_message.Data[7] | (rx_message.Data[6] << 8))) * 0.1);
			}
			float temp, temp_sin, temp_cos; // 用于寄存公用的三角函数运算值，减少重复计算
			temp = (Position_Now_mp.AngleDeg) * PI / 180 + Angle_OO;
			temp_cos = cos(temp);
			temp_sin = sin(temp);
			if (rx_message.Data[1] & 0x04)
				Position_Now_mp.X = (double)((s16)(((u16)(rx_message.Data[2]) << 8) | rx_message.Data[3])) + OO_distance * temp_cos;
			if (rx_message.Data[1] & 0x02)
				Position_Now_mp.Y = (double)((s16)(((u16)(rx_message.Data[4]) << 8) | rx_message.Data[5])) + OO_distance * temp_sin;
			Answer_Master(8, ID_MASTER_RESET_POSITION, rx_message.Data[0], rx_message.Data[1], rx_message.Data[2], rx_message.Data[3], rx_message.Data[4], rx_message.Data[5]);
			break;

		default:
			break;
		}
	}
}
