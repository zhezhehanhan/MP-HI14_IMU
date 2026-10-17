#ifndef __KALMAN_H__
#define __KALMAN_H__

typedef struct KALMAN
{
    double Q; //过程噪声的协方差
    double R; //观测噪声的协方差

    double x_last; //上次的最优值，本次测量值，本次最优值
    double p_last; //上次协方差，本次预测协方差，本次协方差

    double kg; // kalman增益

    double A; //系统转移矩阵，
    double H; //观测转移矩阵

    double best;

} KALMAN_filter;

extern KALMAN_filter kalman_Vx;
extern KALMAN_filter kalman_Vy;
extern KALMAN_filter kalman_Vw;
extern KALMAN_filter kalman_Accelerate;

void kalman_init(KALMAN_filter *kalman);
double KAL_filter(KALMAN_filter *kalman, double input);

#endif
