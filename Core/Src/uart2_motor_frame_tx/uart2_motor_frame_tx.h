/**
 ******************************************************************************
 * @file    uart2_motor_frame_tx.h
 * @brief   UART2 电机控制 11 字节帧 —— 发送端 API
 *
 * 帧格式 (“# … !” 无校验)：
 *   Byte 0   0x23 '#'             —— 帧头
 *   Byte 1‑4 目标速度  int8[4]    —— 电机 A‑D
 *   Byte 5   控制字               —— bit0 = 1 → 梯形加减速
 *   Byte 6‑9 角度环速度 int8[4]   —— 电机 A‑D
 *   Byte 10  0x21 '!'             —— 帧尾
 *
 * 使用流程：
 *   1) 可选调用 MotorFrame_UART2_TxInit() 复位 busy 标志
 *   2) MotorFrame_SetSpeedAndCtrl(...)         // 仅写缓存，不发送
 *   3) MotorFrame_SetAngleVelocity(...)        //           ”
 *   4) MotorFrame_UART2_Send() 或 SendIT()     // 真正发送
 *
 * 作者：王泓俨 2025‑05‑03
 ******************************************************************************
 */
#ifndef __UART2_MOTOR_FRAME_TX_H
#define __UART2_MOTOR_FRAME_TX_H

/*====================  头文件  ====================*/
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*====================  API  =======================*/
void MotorFrame_UART2_TxInit(void);                       /* 复位 busy 标志                     */

/* 写入 Byte 1‑5：speedA‑D & 梯形加减速位（只改变量，不立即发送）         */
void MotorFrame_SetSpeedAndCtrl(int8_t speedA,
                                int8_t speedB,
                                int8_t speedC,
                                int8_t speedD,
                                bool   trapezoid);

/* 写入 Byte 6‑9：角度环速度 A‑D（只改变量，不立即发送）                  */
void MotorFrame_SetAngleVelocity(int8_t velA,
                                 int8_t velB,
                                 int8_t velC,
                                 int8_t velD);

/* 发送整帧 -------------------------------------------------------------*/
HAL_StatusTypeDef MotorFrame_UART2_Send   (void); /* 阻塞（10 ms 超时） */
HAL_StatusTypeDef MotorFrame_UART2_SendIT (void); /* 非阻塞（中断）     */
void MotorFrame_UART2_TxCallback(UART_HandleTypeDef *huart);/* 在 HAL 回调里调用 */
void MotorFrame_CheckAndSend(void);
void Encoder_Reset(void);
extern int8_t last_set_speed[4];
#endif /* __UART2_MOTOR_FRAME_TX_H */
