/**
 * @file    vision_align.h
 * @brief   视觉 PID 控制对准接口声明
 */

#ifndef __VISION_ALIGN_H__
#define __VISION_ALIGN_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 周期性调用此函数进行视觉对准控制
 */
void vision_alignment_update(void);

/**
 * @brief 设置视觉 PID 控制参数
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param setpoint 目标视觉 x 坐标（通常是图像中心）
 * @param limit 控制量输出最大绝对值（限幅）
 * @param deadband 死区阈值（如 5 表示±5以内不控制）
 */
void vision_alignment_set_parameters(float kp, float ki, float kd, int setpoint, int limit, int deadband);

#ifdef __cplusplus
}
#endif

#endif // __VISION_ALIGN_H__
