/**
 * @file    vision_align.c
 * @brief   视觉 PID 控制对准模块（用于车体横向位置调整）
 */

#include "vision_align.h"
#include "vision_parser.h"
#include "../yaw_pid_control/yaw_pid_control.h"

#include <stdlib.h>
#include <stdbool.h>

/// 控制参数结构体
typedef struct {
    float kp;
    float ki;
    float kd;
    int setpoint;
    int output_limit;
    int deadband;
    float integral;
    int last_error;
} VisionAlignPID;

static VisionAlignPID vision_pid = {
        .kp = -0.2f,
        .ki = 0.0f,
        .kd = 0.0f,
        .setpoint = 320,
        .output_limit = 10,
        .deadband = 5,
        .integral = 0.0f,
        .last_error = 0
};

/**
 * @brief 设置视觉 PID 参数（运行时可调用）
 */
void vision_alignment_set_parameters(float kp, float ki, float kd, int setpoint, int limit, int deadband) {
    vision_pid.kp = kp;
    vision_pid.ki = ki;
    vision_pid.kd = kd;
    vision_pid.setpoint = setpoint;
    vision_pid.output_limit = limit;
    vision_pid.deadband = deadband;
    vision_pid.integral = 0;
    vision_pid.last_error = 0;
}

/**
 * @brief 计算 PID 控制量
 */
static int vision_pid_compute(int current) {
    int error = current - vision_pid.setpoint;

    // 死区判断
    if (abs(error) < vision_pid.deadband) {
        return 0;
    }

    // 积分项
    vision_pid.integral += error;

    // 微分项
    int derivative = error - vision_pid.last_error;
    vision_pid.last_error = error;

    // PID 公式
    float output = vision_pid.kp * error +
                   vision_pid.ki * vision_pid.integral +
                   vision_pid.kd * derivative;

    // 限幅
    if (output > vision_pid.output_limit) output = vision_pid.output_limit;
    if (output < -vision_pid.output_limit) output = -vision_pid.output_limit;

    return (int)output;
}

/**
 * @brief 主控制更新函数，根据视觉坐标执行 PID 控制对准
 */
void vision_alignment_update(void) {
    int control = vision_pid_compute((int)vision_x_coord);

    // 发送控制量（此处为控制横向速度）
    Speed_Control(0, control, 0, false);
}
