#include "hi14_imu.h"
#include <string.h>

HI14_IMU_Data_t hi14_imu_data;
volatile uint8_t hi14_rx_frame_ready = 0;
volatile HI14_Status_t hi14_rx_status = HI14_OK;
volatile uint32_t hi14_err_cnt = 0;
static uint32_t hi14_ok_cnt = 0;

// 接收状态机
typedef enum {
    STATE_HEAD1 = 0,
    STATE_HEAD2,
    STATE_LEN_L,
    STATE_LEN_H,
    STATE_CRC_L,   // CRC在数据域之前！
    STATE_CRC_H,
    STATE_PAYLOAD
} HI14_RxState_t;


static HI14_RxState_t rx_state = 0;
static uint8_t rx_buf[HI14_MAX_PAYLOAD_LEN + 6]; // 5A A5 LEN(2) CRC(2) + payload
static uint16_t rx_index = 0;
static uint16_t payload_len = 0;
static uint16_t rx_crc = 0;

// 安全读取宏 (解决非对齐访问导致的HardFault)
static inline uint8_t Safe_U1(uint8_t *p) { uint8_t v; memcpy(&v, p, 1); return v; }
static inline uint16_t Safe_U2(uint8_t *p) { uint16_t v; memcpy(&v, p, 2); return v; }
static inline uint32_t Safe_U4(uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static inline float Safe_R4(uint8_t *p) { float v; memcpy(&v, p, 4); return v; }

// CRC-16/CCITT 更新函数 (手册第24页)
static void crc16_update(uint16_t *crc, uint8_t *data, uint32_t len)
{
    uint16_t crc_val = *crc;
    for (uint32_t i = 0; i < len; i++) {
        crc_val ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc_val & 0x8000) {
                crc_val = (crc_val << 1) ^ 0x1021;
            } else {
                crc_val <<= 1;
            }
        }
    }
    *crc = crc_val;
}

void HI14_IMU_Init(void)
{
    rx_state = STATE_HEAD1;
    rx_index = 0;
    payload_len = 0;
    hi14_rx_frame_ready = 0;
    hi14_rx_status = HI14_OK;
    hi14_err_cnt = 0;
    hi14_ok_cnt = 0;
}

/**
 * @brief 字节接收状态机，放入 USART1_IRQHandler 中调用
 */
void HI14_Rx_Byte(uint8_t byte)
{
    switch (rx_state)
    {
        case STATE_HEAD1:
            if (byte == HI14_FRAME_HEAD1) {
                rx_buf[0] = byte;
                rx_state = STATE_HEAD2;
            }
            break;
            
        case STATE_HEAD2:
            if (byte == HI14_FRAME_HEAD2) {
                rx_buf[1] = byte;
                rx_state = STATE_LEN_L;
            } else {
                rx_state = STATE_HEAD1;
            }
            break;
            
        case STATE_LEN_L:
            rx_buf[2] = byte;
            payload_len = byte;
            rx_state = STATE_LEN_H;
            break;
            
        case STATE_LEN_H:
            rx_buf[3] = byte;
            payload_len |= ((uint16_t)byte << 8);
            if (payload_len > HI14_MAX_PAYLOAD_LEN || payload_len == 0) {
                rx_state = STATE_HEAD1; // 长度异常，重置
                hi14_rx_status = HI14_ERR_LEN;
                hi14_err_cnt++;
            } else {
                rx_state = STATE_CRC_L; // 长度合法，接下来读CRC
            }
            break;

        case STATE_CRC_L:
            rx_buf[4] = byte;
            rx_crc = byte;
            rx_state = STATE_CRC_H;
            break;

        case STATE_CRC_H:
            rx_buf[5] = byte;
            rx_crc |= ((uint16_t)byte << 8);
            rx_index = 6; // 数据域从buf[6]开始
            rx_state = STATE_PAYLOAD;
            break;
            
        case STATE_PAYLOAD:
            rx_buf[rx_index++] = byte;
            if (rx_index >= payload_len + 6) {
                // 一帧接收完成，开始校验CRC
                uint16_t calc_crc = 0;
                // 1. 计算帧头和长度 (buf[0] ~ buf[3])
                crc16_update(&calc_crc, rx_buf, 4);
                // 2. 计算数据域 (buf[6] 开始，长度为 payload_len)
                crc16_update(&calc_crc, rx_buf + 6, payload_len);
                
                if (calc_crc == rx_crc) {
                    hi14_rx_frame_ready = 1; 
                    hi14_rx_status = HI14_OK;
                } else {
                    hi14_rx_status = HI14_ERR_CRC;
                    hi14_err_cnt++;
                }
                rx_state = STATE_HEAD1; // 准备接收下一帧
            }
            break;
            
        default:
            rx_state = STATE_HEAD1;
            break;
    }
}

/**
 * @brief 主循环中调用，将中断接收到的数据解析并更新到全局变量
 */
void HI14_Process(void)
{
    if (hi14_rx_frame_ready) 
    {
        hi14_rx_frame_ready = 0; // 清除标志
        
        uint8_t *buf = rx_buf + 6; // 指向数据域起始位置
        uint16_t offset = 0;
        uint8_t tag = Safe_U1(buf + offset); offset += 1;

        if (tag == 0x91) // 浮点型IMU数据帧
        {
            hi14_imu_data.tag = tag;
            hi14_imu_data.main_status = Safe_U2(buf + offset); offset += 2;
            hi14_imu_data.temperature = (int8_t)Safe_U1(buf + offset); offset += 1;
            hi14_imu_data.air_pressure = Safe_R4(buf + offset); offset += 4;
            hi14_imu_data.system_time = Safe_U4(buf + offset); offset += 4;

            for (int i = 0; i < 3; i++) { hi14_imu_data.acc_b[i] = Safe_R4(buf + offset); offset += 4; }
            for (int i = 0; i < 3; i++) { hi14_imu_data.gyr_b[i] = Safe_R4(buf + offset); offset += 4; }
            for (int i = 0; i < 3; i++) { hi14_imu_data.mag_b[i] = Safe_R4(buf + offset); offset += 4; }
            for (int i = 0; i < 3; i++) { hi14_imu_data.eul[i]   = Safe_R4(buf + offset); offset += 4; }
            for (int i = 0; i < 4; i++) { hi14_imu_data.quat[i]  = Safe_R4(buf + offset); offset += 4; }
            
            hi14_ok_cnt++;
        }
        else
        {
            hi14_rx_status = HI14_ERR_TAG;
            hi14_err_cnt++;
        }
    }
}
