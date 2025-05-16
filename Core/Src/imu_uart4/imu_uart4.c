/**
 * @file    imu_uart4.c
 * @brief   USART4 中断方式解析 0x84‑IMU 数据帧
 */

#include "imu_uart4.h"
#include "usart.h"
#include "stm32f1xx_hal.h"
#include "../buzzer/buzzer.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*                 配置区：根据 IMU 协议修改                           */
/* ------------------------------------------------------------------ */
#define IMU_FRAME_LEN     29U
#define IMU_FRAME_HEAD    0x84
#define IMU_CKS_INDEX     28U

/* ------------------------------------------------------------------ */
/*                 接收状态机私有变量                                  */
/* ------------------------------------------------------------------ */
static uint8_t  g_rxByte        = 0;                 /* 单字节缓存          */
static uint8_t  s_rxBuf[IMU_FRAME_LEN];              /* 帧缓冲             */
static uint8_t  s_rxIndex       = 0;                 /* 当前写入下标       */

/* ------------------------------------------------------------------ */
/*                 公开数据（°×100 / (°/s)×100）                       */
/* ------------------------------------------------------------------ */
volatile imu_t imu_roll_cdeg   = 0;
volatile imu_t imu_pitch_cdeg  = 0;
volatile imu_t imu_yaw_cdeg    = 0;

volatile imu_t imu_gyro_x_cdps = 0;
volatile imu_t imu_gyro_y_cdps = 0;
volatile imu_t imu_gyro_z_cdps = 0;

/* ================================================================== */
/* 3‑Byte 帧字段解码 ➜ int16_t ×100                                    */
/* ================================================================== */
static inline imu_t decode3_int(const uint8_t *p)
{
    uint8_t  neg = (p[0] & 0xF0) ? 1U : 0U;                /* 1 = 负 */
    uint16_t val =
            ((p[0] & 0x0F) * 10000U) +
            ((p[1] >> 4)   * 1000U) +
            ((p[1] & 0x0F) *  100U) +
            ((p[2] >> 4)   *   10U) +
            ( p[2] & 0x0F);
    return neg ? -(imu_t)val : (imu_t)val;
}

/* [-18000,+18000] → [0,36000] */
//static inline imu_t wrap360(imu_t v) { return (v < 0) ? (v + 36000) : v; }
static inline imu_t wrap360(imu_t v) {
    return (v >= 0) ? v : (v + 36000);
}

/* ================================================================== */
/* 解析完整 29‑byte 流                                                */
/* ================================================================== */
static void IMU_ParseFrame(const uint8_t *frame)
{
    const uint8_t *p = &frame[1];     /* Skip head 0x84 */

    /* 2. 姿态角 (ROLL/PITCH/YAW) */
    imu_roll_cdeg  = wrap360( decode3_int(p) ); p += 3;
    imu_pitch_cdeg = wrap360( decode3_int(p) ); p += 3;
    imu_yaw_cdeg   = wrap360( decode3_int(p) ); p += 3;

    /* 3. 跳过 3×ACC (9B) */
    p += 9;

    /* 4. 角速度 */
    imu_gyro_x_cdps = decode3_int(p); p += 3;
    imu_gyro_y_cdps = decode3_int(p); p += 3;
    imu_gyro_z_cdps = decode3_int(p);
}

/* ================================================================== */
/* 公共函数                                                            */
/* ================================================================== */
void IMU_UART4_Init(void)
{
    HAL_UART_Receive_IT(&huart4, &g_rxByte, 1);           /* 启动首字节 */
}

/**
 * @brief 单字节中断分发 —— 在 RxCplt 回调里调用
 */
void IMU_UART4_RxISR(void)
{
    /* 1. 帧头同步 */
    if (s_rxIndex == 0U) {
        if (g_rxByte != IMU_FRAME_HEAD) goto _next;       /* 不是 0x84 */
    }

    /* 2. 写入缓冲 */
    s_rxBuf[s_rxIndex++] = g_rxByte;

    /* 3. 收满 29 B 时尝试解析 */
    if (s_rxIndex >= IMU_FRAME_LEN) {
        IMU_ParseFrame(s_rxBuf);
        s_rxIndex = 0U;/* 重新同步   */
    }

    _next:
    HAL_UART_Receive_IT(&huart4, &g_rxByte, 1);           /* 继续收字节 */
}

/* ---------- 清零方位角命令 ----------------------------------------- */
void IMU_ResetHeading(void)
{
    static const uint8_t kResetCmd[5] = { 0x68, 0x04, 0x00, 0x28, 0x2C };

    /* 典型 IMU 要求 115 200 bps，阻塞 1 ms 足够 */
    HAL_UART_Transmit(&huart4,
                      (uint8_t *)kResetCmd,
                      sizeof kResetCmd,
                      HAL_MAX_DELAY);
}