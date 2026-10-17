#include "time2.h"
#include "ADS1255.h"
#include "can.h"

/*动态更新零点的全局变量*/
void time2_config(void)
{
	TIM_TimeBaseInitTypeDef tim;
	NVIC_InitTypeDef nvic;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	tim.TIM_Prescaler = 84 - 1;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	tim.TIM_Period = 66; // 重载值 67us进行一次定时器中断
	TIM_TimeBaseInit(TIM2, &tim);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

	TIM_Cmd(TIM2, ENABLE);
}

void time3_config(void)
{
	TIM_TimeBaseInitTypeDef tim;
	NVIC_InitTypeDef nvic;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	nvic.NVIC_IRQChannel = TIM3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	tim.TIM_Prescaler = 84 - 1;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	tim.TIM_Period = 100000 - 1; // 重载值 100ms进行一次定时器中断
	TIM_TimeBaseInit(TIM3, &tim);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);

	TIM_Cmd(TIM3, ENABLE);
}

static char i = 0;
int n;
float time_click;

void TIM2_IRQHandler(void) // 定时读取数据用于组合滤波
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // 清除tim2的中断挂起位
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);		// 清除tim2的更新标志位
		n++;
		time_click += 0.067; // 67um进入一次中断，进一次中断就加一次67um，这是个计时器
		if (time_click >= 100000)
			time_click = 0;
		if (i < MAX_NUM) // 5个数一组进行处理
		{
			comb_flt.data[i] = Read_AD_Value(); // 读取AD的值 赋给复合滤波器的data数组
			i++;
		}
		if (i == MAX_NUM)
		{
			i = 0;
			comb_flt.comb_filter_flag = 2;
		}
		if (n == 7) // 7*67um
		{
			V_Send(&Speed_Now); // 给主控发送速度数据
		}
		if (n = 29) // 1.943ms
		{
			Position_Send(&Position_Now_rob); // 给主控发送位置数据
		}
		if(n>=)
	}
}

extern u8 acc_flag;
extern float Accelerated_Speed;
void TIM3_IRQHandler(void) // 加速度,100ms
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update); // 清除tim3的中断挂起位
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);		// 清除tim3的更新标志位
		if (acc_flag == 0)
			return;

		ACC_Send(Accelerated_Speed);
	}
}