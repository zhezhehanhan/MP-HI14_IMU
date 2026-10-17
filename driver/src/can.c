#include "main.h"
#include "can.h"

void can_init(void) // can�ĳ�ʼ��
{
	CAN_InitTypeDef can;
	CAN_FilterInitTypeDef can_filter;
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // ��GPIOA��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);  // ��CAN1��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);  // ��CAN2��ʱ��

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1); // PA11��CAN1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1); // PA12��CAN1

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
	can.CAN_ABOM = ENABLE;
	can.CAN_AWUM = DISABLE;
	can.CAN_NART = DISABLE;
	can.CAN_RFLM = DISABLE;
	can.CAN_TXFP = DISABLE; // ?
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
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;	// ��ʶ������ģʽ
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit; // ����
	can_filter.CAN_FilterIdHigh = (uint16_t)(((0X01010300 << 3)) >> 16);
	can_filter.CAN_FilterIdLow = (uint16_t)(((0X01010300 << 3)) >> 0);
	can_filter.CAN_FilterMaskIdHigh = (s16)((uint32_t)(0X1FFFFF00u << 3) >> 16);
	can_filter.CAN_FilterMaskIdLow = (s16)((uint32_t)(0X1FFFFF00u << 3) >> 0);
	// ����4�䣬����ֻ����IDΪ0x0100�ı���
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&can_filter);

	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

u8 Position_Send(POSITION *position) // �����ط���λ������
{
	CanTxMsg tx_message;
	tx_message.ExtId = ID_POSITION; // ����ID��,λ����Ϣ����
	tx_message.DLC = 0x08;			// �����ֽ���
	tx_message.RTR = CAN_RTR_Data;	// ����֡
	tx_message.IDE = CAN_Id_Extended;

	tx_message.Data[0] = (uint8_t)((s16)(position->X * 10 + 0.5f)); // ��ȷ��mm
	tx_message.Data[1] = (uint8_t)((s16)(position->X * 10 + 0.5f) >> 8);
	tx_message.Data[2] = (uint8_t)((s16)(position->Y * 10 + 0.5f));
	tx_message.Data[3] = (uint8_t)((s16)(position->Y * 10 + 0.5f) >> 8);
//	tx_message.Data[0] = (uint8_t)((s16)(position->X)); // ��ȷ��mm
//	tx_message.Data[1] = (uint8_t)((s16)(position->X) >> 8);
//	tx_message.Data[2] = (uint8_t)((s16)(position->Y));
//	tx_message.Data[3] = (uint8_t)((s16)(position->Y) >> 8);
	s32 tempAngle = position->AngleDeg * 100;
	// tx_message.Data[4] = (uint8_t)((s32)(position->AngleDeg * 100) >> 24);
	// tx_message.Data[5] = (uint8_t)((s32)(position->AngleDeg * 100)>> 16);
	// tx_message.Data[6] = (uint8_t)((s32)(position->AngleDeg * 100)>> 8);
	// tx_message.Data[7] = (uint8_t)((s32)(position->AngleDeg * 100));
	tx_message.Data[7] = (uint8_t)((s32)(tempAngle) >> 24);
	tx_message.Data[6] = (uint8_t)((s32)(tempAngle) >> 16);
	tx_message.Data[5] = (uint8_t)((s32)(tempAngle) >> 8);
	tx_message.Data[4] = (uint8_t)((s32)(tempAngle));
	if (CAN_Transmit(CAN1, &tx_message) == CAN_TxStatus_NoMailBox)
		return 1;
	else
		return 0;
}

u8 V_Send(SPEED *speed) // �����ط����ٶ�����
{
	CanTxMsg tx_message;
	tx_message.ExtId = ID_V;	   // ����ID��
	tx_message.DLC = 0x08;		   // �����ֽ���
	tx_message.RTR = CAN_RTR_Data; // ����֡
	tx_message.IDE = CAN_Id_Extended;

	tx_message.Data[0] = (uint8_t)((s16)(speed->Vx * 10 + 0.5f)); // ��ȷ��mm
	tx_message.Data[1] = (uint8_t)((s16)(speed->Vx * 10 + 0.5f) >> 8);
	tx_message.Data[2] = (uint8_t)((s16)(speed->Vy * 10 + 0.5f));
	tx_message.Data[3] = (uint8_t)((s16)(speed->Vy * 10 + 0.5f) >> 8);
	tx_message.Data[4] = (uint8_t)((s16)(speed->V * 10 + 0.5f));
	tx_message.Data[5] = (uint8_t)((s16)(speed->V * 10 + 0.5f) >> 8);
	tx_message.Data[6] = (uint8_t)((s16)(speed->Vw * 10 + 0.5f));
	tx_message.Data[7] = (uint8_t)((s16)(speed->Vw * 10 + 0.5f) >> 8);
	if (CAN_Transmit(CAN1, &tx_message) == CAN_TxStatus_NoMailBox)
		return 1;
	else
		return 0;
}

u8 ACC_Send(float Accelerated_Speed) // �����ط��ͼ��ٶ�����,100ms
{
	CanTxMsg tx_message;
	tx_message.ExtId = ID_RESPOND_ACC; // ����ID��
	tx_message.DLC = 0x08;			   // �����ֽ���
	tx_message.RTR = CAN_RTR_Data;	   // ����֡
	tx_message.IDE = CAN_Id_Extended;

	tx_message.Data[1] = (uint8_t)((s16)(Accelerated_Speed * 10 + 0.5f)); // ��ȷ��mm
	tx_message.Data[0] = (uint8_t)((s16)(Accelerated_Speed * 10 + 0.5f) >> 8);
	if (CAN_Transmit(CAN1, &tx_message) == CAN_TxStatus_NoMailBox)
		return 1;
	else
		return 0;
}

void Answer_Master(int length, uint32_t ID, char Data0, char Data1, char Data2, char Data3, char Data4, char Data5, char Data6, char Data7)//这个函数是用来回复主控的
{
	CanTxMsg tx_message;
	tx_message.ExtId = ID;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.IDE = CAN_Id_Extended;
	tx_message.DLC = length;
	tx_message.Data[0] = Data0;
	tx_message.Data[1] = Data1;
	tx_message.Data[2] = Data2;
	tx_message.Data[3] = Data3;
	tx_message.Data[4] = Data4;
	tx_message.Data[5] = Data5;
	tx_message.Data[6] = Data6;
	tx_message.Data[7] = Data7;
	CAN_Transmit(CAN1, &tx_message);
}

u16 HB_count = 0;
u8 acc_flag = 0;
// FIXME : ֮ǰ��λ����Ϣ��10���ͣ���ε�ͨ��Э����û�г�10
void CAN1_RX0_IRQHandler(void) // 这个函数是CAN1的接收中断函数
{
	CanRxMsg rx_message;
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
		HB_count++;
		switch (rx_message.ExtId)
		{
		case ID_MASTER_HB://这里的作用是回复主控的心跳包，回复内容是从控的心跳包 
			Answer_Master(6, ID_MASTER_HB, 0, 0, 'H', 'B', (uint8_t)(HB_count >> 8), (uint8_t)(HB_count & 0xff), 0, 0);
			HB_count = 0;
			break;

		case ID_MASTER_ACC:
			if (rx_message.Data[1] == 1)
				acc_flag = 1;
			if (rx_message.Data[1] == 0)
				acc_flag = 0;
			break;

		case ID_MASTER_RESET_POSITION:
			if (rx_message.Data[1] & 0x01) // 0000 0001
			{
				Position_Now_mp.AngleDeg = (double)((s16)(((u16)(rx_message.Data[7]) << 8) | rx_message.Data[6]) / 10); //(float)((s16)(((u16)(rx_message.Data[7]) | (rx_message.Data[6] << 8))) * 0.1);
			}
			float temp, temp_sin, temp_cos; // ���ڼĴ湫�õ����Ǻ�������ֵ�������ظ�����
			temp = (Position_Now_mp.AngleDeg) * degree2rad + Angle_OO;
			temp_cos = cos(temp);
			temp_sin = sin(temp);
			if (rx_message.Data[1] & 0x04) // 0000 0100
				Position_Now_mp.X = (double)((s16)(((u16)(rx_message.Data[3]) << 8) | rx_message.Data[2]) ) / 10  + OO_distance * temp_cos;
			if (rx_message.Data[1] & 0x02) // 0000 0010
				Position_Now_mp.Y = (double)((s16)(((u16)(rx_message.Data[5]) << 8) | rx_message.Data[4]) ) / 10 + OO_distance * temp_sin;
			Answer_Master(8, ID_RESPOND_RESET_POSITION, rx_message.Data[0], rx_message.Data[1], rx_message.Data[2], rx_message.Data[3], rx_message.Data[4], rx_message.Data[5], rx_message.Data[6], rx_message.Data[7]);
			break;

		default:
			break;
		}
	}
}
