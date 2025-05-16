/**
 * @file    imu_uart4.h
 * @brief   USART4 解析 0x84‑IMU (29 B) 数据帧 —— 整数 ×100 角度/角速度
 * @author  SCU@王泓俨
 * @date    2025‑05‑06
 *
 * 数据说明
 * ──────────────────────────────────────────────────────────
 * • 固定帧长 29 B：0x84 + 9×3 B + Checksum
 * • 3‑Byte 编码：b0[7:4] = Sign(0:+,1:‑)，其余 5 个十进制位 → ×0.01
 * • 本驱动只保留 ROLL/PITCH/YAW + Gyro XYZ（跳过 ACC）
 * • 解析后统一放大 100 倍存入有符号整数：
 *      -180.00° … +180.00°  ⇒  -18000 … +18000
 *      转成 0 … 36000（°×100）存储，便于整型环绕运算
 *      角速度 ±180.00°/s 直接存 -18000…+18000
 */

#ifndef IMU_UART4_H_
#define IMU_UART4_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------- 类型别名（°×100 / (°·s⁻¹)×100） -------- */
typedef int32_t imu_t;

/* ---------------- 公开解析结果 -------------------- */
extern volatile imu_t imu_roll_cdeg;     /**< 0‑36000，单位 0.01° */
extern volatile imu_t imu_pitch_cdeg;    /**< 0‑36000，单位 0.01° */
extern volatile imu_t imu_yaw_cdeg;      /**< 0‑36000，单位 0.01° */

extern volatile imu_t imu_gyro_x_cdps;   /**< ±18000，单位 0.01 °/s */
extern volatile imu_t imu_gyro_y_cdps;   /**< ±18000，单位 0.01 °/s */
extern volatile imu_t imu_gyro_z_cdps;   /**< ±18000，单位 0.01 °/s */

/* ---------------- 公共 API ------------------------ */
/**
 * @brief 初始化并启动首次单字节接收
 * @note  应在 `MX_USART4_UART_Init()` 之后调用
 */
void IMU_UART4_Init(void);

/**
 * @brief 在 `HAL_UART_RxCpltCallback()` 内部调用（仅 USART4 分支）
 * @param None
 */
void IMU_UART4_RxISR(void);

/**
 * @brief  发送“清零方位角”命令 0x68 04 00 28 2C
 * @note   默认阻塞发送；若需 DMA/IT 请自行替换底层 HAL 调用
 */
void IMU_ResetHeading(void);

#ifdef __cplusplus
}
#endif
#endif /* IMU_UART4_H_ */
