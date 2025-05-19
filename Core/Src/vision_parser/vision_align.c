#include "vision_align.h"
#include "vision_parser.h"
#include "../yaw_pid_control/yaw_pid_control.h"

#include <stdlib.h>
#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    int setpoint;
    int output_limit;
    int deadband;
    float integral;
    int last_error;
} VisionAxisPID;

static VisionAxisPID vision_pid_x = {
        .kp = -0.2f,
        .ki = 0.0f,
        .kd = 0.0f,
        .setpoint = 320,
        .output_limit = 100,
        .deadband = 5,
        .integral = 0,
        .last_error = 0
};

static VisionAxisPID vision_pid_y = {
        .kp = 0.15f,
        .ki = 0.0f,
        .kd = 0.0f,
        .setpoint = 220,
        .output_limit = 100,
        .deadband = 5,
        .integral = 0,
        .last_error = 0
};

void vision_alignment_set_x_params(float kp, float ki, float kd, int setpoint, int limit, int deadband) {
    vision_pid_x.kp = kp;
    vision_pid_x.ki = ki;
    vision_pid_x.kd = kd;
    vision_pid_x.setpoint = setpoint;
    vision_pid_x.output_limit = limit;
    vision_pid_x.deadband = deadband;
    vision_pid_x.integral = 0;
    vision_pid_x.last_error = 0;
}

void vision_alignment_set_y_params(float kp, float ki, float kd, int setpoint, int limit, int deadband) {
    vision_pid_y.kp = kp;
    vision_pid_y.ki = ki;
    vision_pid_y.kd = kd;
    vision_pid_y.setpoint = setpoint;
    vision_pid_y.output_limit = limit;
    vision_pid_y.deadband = deadband;
    vision_pid_y.integral = 0;
    vision_pid_y.last_error = 0;
}

static int vision_pid_compute(VisionAxisPID *pid, int current) {
    int error = current - pid->setpoint;

    if (abs(error) < pid->deadband) return 0;

    pid->integral += error;
    int derivative = error - pid->last_error;
    pid->last_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return (int)output;
}

void vision_alignment_update(void) {
    int vx = vision_pid_compute(&vision_pid_x, (int)vision_x_coord); // 左右速度
    int vy = vision_pid_compute(&vision_pid_y, (int)vision_y_coord); // 前后速度

    Speed_Control(vy, vx, 0, false);
}
