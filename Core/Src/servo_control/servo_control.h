/* =============================================================================
 *  servo_control.h
 *  270° 舵机 (50 Hz PWM) 驱动模块头文件
 *  适用 MCU : STM32F1 系列（无硬件 FPU）
 * -----------------------------------------------------------------------------
 *  功能接口：
 *    - servo_init()               启动 TIM4_CH1/CH2 PWM
 *    - servo_set_angle(channel, angle_deg)
 *                                设置 0–270° 舵机角度
 * -----------------------------------------------------------------------------
 */

#ifndef __SERVO_CONTROL_H
#define __SERVO_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 舵机初始化，必须调用一次以启动 PWM
 */
void servo_init(void);

/**
 * @brief 设置指定通道舵机角度 (0–270°)
 * @param channel   TIM_CHANNEL_1 或 TIM_CHANNEL_2
 * @param angle_deg 目标角度，范围 0–270
 */
void servo_set_angle(uint32_t channel, uint16_t angle_deg);

#endif /* __SERVO_CONTROL_H */

