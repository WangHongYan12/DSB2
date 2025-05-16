#ifndef __YAW_PID_CONTROL_H
#define __YAW_PID_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

//—— 外部变量 ——
// 当前偏航角（单位：0.01°），在陀螺仪驱动模块中定义并实时更新

// 目标偏航角（单位：0.01°），可在主程序中设置

extern bool yaw_pid_control_on;
//—— 函数声明 ——

// 定时器中断回调，每次中断执行一次 PID 计算并下发到电机
void yaw_pid_control_callback(void);

// 底盘运动控制函数
// @param xspeed 前后线速度
// @param yspeed 左右线速度
// @param rspeed 旋转速度（正为顺时针，负为逆时针）
void Vehicle_movement(int xspeed, int yspeed, int rspeed);
void Speed_Control(int xspeed, int yspeed, int rspeed , bool ramp);
#endif // __YAW_PID_CONTROL_H
