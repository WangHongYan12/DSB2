/* =============================================================================
 *  line_follow_pid.h
 *  8 路红外寻迹 PID 计算模块头文件（位置式 PID）
 *  适用 MCU : STM32F1 系列（无硬件 FPU）
 * -----------------------------------------------------------------------------
 *  功能接口声明：
 *    - void Line_Follow_PID_Control(bool use_ramp);
 *    - bool Line_Follow_PID_IsFinished(void);
 *  依赖外部符号：
 *    - extern volatile uint8_t lt_digital[8];
 *    - extern void yaw_ramp_set_goal(int32_t new_goal_cdeg, bool use_ramp);
 * -----------------------------------------------------------------------------
 */

#ifndef __LINE_FOLLOW_PID_H
#define __LINE_FOLLOW_PID_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  主控制函数，周期调用以完成巡线位置式 PID 并设置朝向环目标
 * @param  use_ramp 是否使能斜坡限制
 */
void Line_Follow_PID_Control(bool use_ramp);

/**
 * @brief  查询是否到达终点 (检测到 >= 5 路黑线)
 * @return true 表示已到达终点
 */
bool Line_Follow_PID_IsFinished(void);
void Remake_reached_end_flag(void);
extern bool Line_Follow_On;
extern int32_t yaw_goal;
void Start_Line_Follow(void);
void Stop_Line_Follow(void);
#endif /* __LINE_FOLLOW_PID_H */