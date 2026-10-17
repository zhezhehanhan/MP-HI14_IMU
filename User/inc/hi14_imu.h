#ifndef _HI14_IMU_H
#define _HI14_IMU_H

#include "stm32f4xx.h"

// 协议常量
#define HI14_FRAME_HEAD1      0x5A
#define HI14_FRAME_HEAD2      0xA5
#define HI14_MAX_PAYLOAD_LEN  128

// 错误状态反馈
typedef enum {
    HI14_OK = 0,
    HI14_ERR_CRC,
    HI14_ERR_LEN,
    HI14_ERR_TAG
} HI14_Status_t;

// 解析后的IMU数据结构体 (基于0x91数据帧)
typedef struct {
    uint8_t  tag;               // 数据标签 (0x91)
    uint16_t main_status;       // 状态字
    int8_t   temperature;       // 温度 (℃)
    float    air_pressure;      // 气压 (Pa)
    uint32_t system_time;       // 时间戳 (ms)
    float    acc_b[3];          // 加速度 XYZ (G)
    float    gyr_b[3];          // 角速度 XYZ (deg/s)
    float    mag_b[3];          // 磁强度 XYZ (uT)
    float    eul[3];            // 欧拉角 Roll, Pitch, Yaw (deg)
    float    quat[4];           // 四元数 W, X, Y, Z
} HI14_IMU_Data_t;

extern HI14_IMU_Data_t hi14_imu_data;        // 主循环使用的数据
extern volatile uint8_t hi14_rx_frame_ready; // 帧接收完成标志
extern volatile HI14_Status_t hi14_rx_status;// 最近一次解析状态
extern volatile uint32_t hi14_err_cnt;       // 错误计数器

void HI14_IMU_Init(void);
void HI14_Rx_Byte(uint8_t byte);             // 放入串口中断调用
void HI14_Process(void);                     // 放入主循环调用

#endif