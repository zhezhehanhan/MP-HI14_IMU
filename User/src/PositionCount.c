#include "PositionCount.h"

/*****************************************************************/
float x_del;				// 定位坐标X增量
float y_del;				// 定位坐标Y增量
float angle_del;			// 定位坐标角度增量
float Gyro_angle_delta = 0; // 每一次的定位坐标角度增量
extern int ANGLE_FLAG;
volatile uint8_t imu_angle_ready = 0;

POSITION Position_Now_mp = {0, 0, 0, 0, 0};	  // 码盘坐标(cm)
POSITION Position_Now_rob = {0, 0, 0, 0, 0};  // 小车坐标
POSITION Position_Last_rob = {0, 0, 0, 0, 0}; // 小车上一次解算坐标
SPEED Speed_Now = {0, 0, 0, 0};
SPEED Speed_temp = {0, 0, 0, 0};
SPEED Speed_comb; // 组合滤波之后的速度加速度

int zp_now_position = 0; // 零点目前所在位置
double zp_sum = 0;
double zp_data[MAX_ZP_SAMPLE] = {0};
double angle = 0;
double filted_rate = 0;
u8 init_flag = 0;

int ang_div = 1;

int speed_i = 0;
comb_filter Vx_comb_flt;
comb_filter Vy_comb_flt;
comb_filter Vw_comb_flt;
comb_filter V_comb_flt;
comb_filter a_comb_flt;

/****************************标定参数*************************************/
float POS_SCALE = 15000; /* VG910B 陀螺仪正向转的积分系数*/
float NEG_SCALE = 15000; /* VG910B 陀螺仪反向转的积分系数*/

float CM_PER_CNT_1 = 0.003982069; // 轮径系数
float CM_PER_CNT_2 = 0.004009497;

float OO_distance = 0;//31.969;			  //L(cm)
float Angle_XX = 0 * degree2rad;//(-45.08741683 - 180 ) * degree2rad; // β
float Angle_OO = 90 * degree2rad;//72.56 * degree2rad;             //(90.04574718) * degree2rad; //α

float deviation_angle_mp = (-0.491551998 - 90)  * degree2rad; // 码盘夹角 - 90°
float tan_deviation_angle_mp = -0.008579411;
float cos_deviation_angle_mp = 0.999963199;
//此时码盘坐标和机器人坐标原点Z轴投影重合，且码盘坐标和机器人坐标的x轴平行，y轴平行
/*****************************中间变量****************************************/
float course_sum_1 = 0;
float course_sum_2 = 0;

INT32U time = 0, time_last = 0;
float time_det = 0;

float speed_Vnow = 0, speed_Vlast = 0;
float Accelerated_Speed;
float Accelerated_comb;
/*****************************************************************/

void CountAngel(void) // 运行 146.25us
{
	if (comb_flt.comb_filter_flag == 2) // 5us 执行时间IIR_input
	{
		kal_input = combination_filter(&comb_flt);			// comb_fit.data[] 为读取的AD值 ，对读取的输入值使用组合滤波
		kal_output = kalman_filter(&kal_filter, kal_input); // 卡尔曼滤波，加了一阶滞后滤波
		second_order_exkalman_output = second_order_exkalman_filter(&EXkal_filter, &second_order_flt, kal_input);
		if (init_flag == 0) // 开机时先采集MAX_ZP_SAMPLE = 2000个数据，用于零点的更新
		{
			if (zp_now_position < (MAX_ZP_SAMPLE - 1))
			{
				LED_RED_ON();
				LED_GREEN_OFF();
				zp_data[zp_now_position] = kal_output;
				zp_sum += zp_data[zp_now_position++]; // 运算完后zp_now_position（零点目前所在位置）+1
			}
			else // 初始化数完毕 init_flag置1
			{
				LED_RED_OFF();
				init_flag = 1;
				zp_data[zp_now_position] = kal_output;
				zp_sum += zp_data[zp_now_position];						 // 恰好读取了2000次AD值
				kal_filter.zero_point = (float)(zp_sum / MAX_ZP_SAMPLE); // 初始化完成，零点第一次被更新 -- 取平均值，用卡尔曼滤波取静态的零点
			}
		}
		else // 首次零点更新完成
		{
			cha = fabs(kal_output - kal_filter.zero_point); // 使用卡尔曼滤波读取的AD值减去零点值

			if (cha < ZERO_THREDHOLD) // 阈值内的值用于更新零点
			{
				if (zp_now_position < MAX_ZP_SAMPLE) // 如果零点目前所在位置<2000，则先减去更新完成时对应位置的值，再加上新读取的值，直到 == 2000
				{
					zp_sum -= zp_data[zp_now_position];
					zp_data[zp_now_position] = kal_output;
					zp_sum += zp_data[zp_now_position];
					kal_filter.zero_point = (float)(zp_sum / MAX_ZP_SAMPLE);
					zp_now_position++;
				}
				else // 如果零点目前所在位置>2000，则先将零点所在位置置零，从第0个值开始更新
				{
					zp_now_position = 0;
					zp_sum = zp_sum - zp_data[zp_now_position];
					zp_data[zp_now_position] = kal_output;
					zp_sum += zp_data[zp_now_position];
					kal_filter.zero_point = (float)(zp_sum / MAX_ZP_SAMPLE);
					zp_now_position++;
				}
			}
			else // 阈值外的值用于积分
			{
				filted_rate = -(double)((second_order_exkalman_output - kal_filter.zero_point)) * SAMP_TIME; // 用卡尔曼滤波与二阶低通滤波结合处理动态问题
				if (filted_rate > 0)
				{
					if (filted_rate > 1500)
						POS_SCALE = SCALE_TAB[1500] * ang_div * 1.000555555555; // 正转系数*积分系数，ang_div还不明白是啥
					else
						POS_SCALE = SCALE_TAB[(int)(filted_rate)] * ang_div  * 1.000555555555;
					Gyro_angle_delta += filted_rate / POS_SCALE;
				}
				else // 负角速度
				{
					if (filted_rate < -1500)
						NEG_SCALE = 1.000383066 * SCALE_TAB[1500] * ang_div;
					else
						NEG_SCALE =  1.000383066 * SCALE_TAB[(int)(-filted_rate)] * ang_div;
					Gyro_angle_delta += filted_rate / NEG_SCALE; // 每次的Δθ
				}
			}
			Position_Now_mp.AngleDeg += Gyro_angle_delta; // 增量式更新姿态角,角度值
			angle_del += Gyro_angle_delta;
			Gyro_angle_delta = 0;
		}
		ANGLE_FLAG++;
	}
}


void CountAngel_new(void)
{
	static uint8_t yaw_initialized = 0;
    static float yaw_zero = 0.0f;
    static float yaw_last = 0.0f;

    float yaw_now;
    float yaw_delta;

    /* 确保至少收到过一帧有效的 IMU 数据 */
    if (hi14_imu_data.tag != 0x91)
    {
        return;
    }

    /* 读取当前航向角，暂按单位为度处理 */
    yaw_now = hi14_imu_data.eul[2];

    /* 首次获得有效姿态角：建立初始航向基准 */
    if (yaw_initialized == 0)
    {
        yaw_zero = yaw_now;
        yaw_last = yaw_now;

        Position_Now_mp.AngleDeg = 0.0f;
        angle_del = 0.0f;

        yaw_initialized = 1;
		imu_angle_ready = 1;
        ANGLE_FLAG++;
        return;
    }

    /* 计算相邻两次航向角的变化量 */
    yaw_delta = yaw_now - yaw_last;

    /* 处理 +180° / -180° 附近的角度跳变 */
    if (yaw_delta > 180.0f)
    {
        yaw_delta -= 360.0f;
    }
    else if (yaw_delta < -180.0f)
    {
        yaw_delta += 360.0f;
    }

    /*
     * 将绝对航向角转换为相对于启动时的角度。
     * 使用 yaw_delta 累积，可避免直接相减时跨越 ±180° 的问题。
     */
    Position_Now_mp.AngleDeg += yaw_delta;

    /* 保留原有的角度增量接口，供 CountSpeed() 计算角速度 */
    angle_del += yaw_delta;

    yaw_last = yaw_now;

    ANGLE_FLAG++;
}


void CountPosition(void)
{
	/*该函数中所用角度均为弧度制*/
	float Angle_Sum; // 姿态角

	float Angle_XX_Last; // 码盘x轴与世界x轴(即小车初始x轴)夹角
	float cos_Angle_XX_Last, sin_Angle_XX_Last;

	float delta_1 = 0, delta_2 = 0;		  // 码盘1,2的距离增量
	float delta_x_mp = 0, delta_y_mp = 0; // 码盘x,y轴距离增量

	Angle_Sum = Position_Now_mp.AngleDeg * degree2rad; // 上一次的弧度

	Angle_XX_Last = Angle_XX + Angle_Sum; // 求的是（β+Σθ）
	cos_Angle_XX_Last = cos(Angle_XX_Last);
	sin_Angle_XX_Last = sin(Angle_XX_Last);

	Encoder_Get_CNT(); // 获取码盘 1 和码盘 2 的脉冲数
	
	// 当码盘1，2轮累计脉冲数正负号反了，在这里更改正负号
  DeltaCntEncoder1 = -DeltaCntEncoder1;
	DeltaCntEncoder2 = -DeltaCntEncoder2;

	delta_1 = CM_PER_CNT_1 * ((float)(DeltaCntEncoder1)); // 将脉冲数转换为码盘 1 走过的路程，Δ1
	delta_2 = CM_PER_CNT_2 * ((float)(DeltaCntEncoder2)); // 将脉冲数转换为码盘 2 走过的路程，Δ2

	/*直线标定时取消注释*/
	course_sum_1 = CM_PER_CNT_1 * ((float)(Position_Now_mp.Sum_Encoder1));
	course_sum_2 = CM_PER_CNT_2 * ((float)(Position_Now_mp.Sum_Encoder2));


	
	Position_Now_mp.Sum_Encoder1 += DeltaCntEncoder1; // 增量式更新码盘 1 走过的总路程

	Position_Now_mp.Sum_Encoder2 += DeltaCntEncoder2; // 增量式更新码盘 2 走过的总路程

	/**/

	/*考虑码盘误差角的影响，把码盘1,2轴的距离增量转化到码盘x,y轴上*/
	delta_x_mp = delta_1;
	delta_y_mp = delta_2 * cos_deviation_angle_mp * (1 + tan_deviation_angle_mp * tan_deviation_angle_mp) + delta_1 * tan_deviation_angle_mp;

	x_del = delta_x_mp * cos_Angle_XX_Last - delta_y_mp * sin_Angle_XX_Last; // 将码盘Δx转换为世界坐标Δx的增量，为Δx*cos（Σθ+β） - Δy*sin（Σθ+β）
	y_del = delta_x_mp * sin_Angle_XX_Last + delta_y_mp * cos_Angle_XX_Last; // 将码盘Δy转换为世界坐标Δy的增量，为Δx*sin（Σθ+β） - Δy*cos（Σθ+β）

	Position_Now_mp.X += x_del; // 计算X
	Position_Now_mp.Y += y_del; // 计算Y
	// 前一步过程在main（）函数里面完成，后续过程在CountSpeed（）中完成
	x_del = 0; // 世界坐标增量重新置零
	y_del = 0;
}

void CountSpeed(void) // 第一个速度值不要
{
	float X_del, Y_del;

	float Angle_OO_Now; // 码盘中心和小车中心连线与世界x轴(即小车初始x轴)夹角
	float cos_Angle_OO_Now, sin_Angle_OO_Now;

	INT32U ticks;

	ticks = OSTimeGet(); // 得到当前的系统节拍
	time = ticks;
	time_det = (float)(time - time_last) / 10000.0f; // 强制从INT32U类型转化为float类型
	time_last = time;

	Angle_OO_Now = Position_Now_mp.AngleDeg * degree2rad + Angle_OO; // （Σθ+α）
	cos_Angle_OO_Now = cos(Angle_OO_Now);
	sin_Angle_OO_Now = sin(Angle_OO_Now);

	Position_Last_rob.X = Position_Now_rob.X;
	Position_Last_rob.Y = Position_Now_rob.Y;
	Position_Now_rob.X = Position_Now_mp.X - OO_distance * cos_Angle_OO_Now;	// 车的位置x坐标，减的是L*cos（Σθ+α）
	Position_Now_rob.Y = Position_Now_mp.Y - OO_distance * sin_Angle_OO_Now; // 车的位置y坐标，减的是L*cos（Σθ+α）
	Position_Now_rob.X *= -1.f;
	Position_Now_rob.AngleDeg = Position_Now_mp.AngleDeg;					 // 1ms 更新一次坐标

	X_del = Position_Now_rob.X - Position_Last_rob.X;
	Y_del = Position_Now_rob.Y - Position_Last_rob.Y; // cm
													  //	Speed_Now.Vx = KAL_filter(&kalman_Vx, (double)(X_del / time_det)); // 计算x方向速度 cm/s
													  //	Speed_Now.Vy = KAL_filter(&kalman_Vy, (double)(Y_del / time_det)); // 计算y方向速度
													  //	Speed_Now.Vw = KAL_filter(&kalman_Vw, (double)(angle_del / time_det));
	Speed_Now.Vx = (double)(X_del / time_det);		  // 计算x方向速度 cm/s
	Speed_Now.Vy = (double)(Y_del / time_det);		  // 计算y方向速度
	Speed_Now.Vw = (double)(angle_del / time_det);
	Speed_Now.V = sqrt(Speed_Now.Vx * Speed_Now.Vx + Speed_Now.Vy * Speed_Now.Vy);

	//	Accelerated_Speed = KAL_filter(&kalman_Accelerate, (double)((speed_Vnow - speed_Vlast) / time_det));
	Accelerated_Speed = (double)((speed_Vnow - speed_Vlast) / time_det);

	Vx_comb_flt.data[speed_i] = Speed_Now.Vx;
	Vy_comb_flt.data[speed_i] = Speed_Now.Vy;
	Vw_comb_flt.data[speed_i] = Speed_Now.Vw;
	V_comb_flt.data[speed_i] = Speed_Now.V;
	a_comb_flt.data[speed_i] = Accelerated_Speed;
	speed_i++;

	speed_Vlast = speed_Vnow;
	speed_Vnow = Speed_Now.V;

	if (speed_i >= 5)
	{
		Speed_comb.Vx = combination_filter(&Vx_comb_flt);
		Speed_comb.Vy = combination_filter(&Vy_comb_flt);
		Speed_comb.Vw = combination_filter(&Vw_comb_flt);
		Speed_comb.V = combination_filter(&V_comb_flt);
		Accelerated_comb = combination_filter(&a_comb_flt);
		speed_i = 0;
	}

	angle_del = 0;
}

void GYRO_Reset(void)
{
	__set_FAULTMASK(1); /*把FAULTMASK置位，即关闭所有的中断执行复位时不被中断打断*/
	NVIC_SystemReset(); /*系统软件复位，配置好的外设寄存器也一起复位*/
}
