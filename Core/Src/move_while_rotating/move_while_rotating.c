// move_while_rotating.c

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "../imu_uart4/imu_uart4.h"
#include "../buzzer/buzzer.h"

// ------------------ 常量定义 ------------------
#define DEG2RAD (0.01f * 3.1415926f / 180.0f)  // cdeg 转弧度

// ------------------ 外部变量与函数 ------------------

void Speed_Control(int xspeed, int yspeed, int rspeed, bool ramp);  // 底盘控制接口

// ------------------ 控制参数 ------------------
int world_x = 100;     // 世界坐标系 x 方向速度
int world_y = 0;       // 世界坐标系 y 方向速度
int rotate_speed = 30; // 自旋速度（顺时针为正）
bool ramp_enable = true;  // 是否启用加减速
bool move_enable_flag = false;  // 控制执行的标志位

// ------------------ 主功能函数 ------------------
void MoveWhileRotating(void) {
    // 仅在标志位开启时执行
    if (!move_enable_flag) return;

    // 获取当前角度（弧度）
    float theta = ((float)imu_yaw_cdeg) * DEG2RAD;

    // 世界坐标转车体坐标
    float fx = world_x * cosf(theta) + world_y * sinf(theta);
    float fy = -world_x * sinf(theta) + world_y * cosf(theta);

    // 转换为整型速度
    int xspeed = (int)(fx);
    int yspeed = (int)(fy);

    // 调用底盘速度控制
    Speed_Control(xspeed, yspeed, rotate_speed, ramp_enable);
}

/**
 * @brief 设置运动控制参数（世界坐标速度、自旋速度、加减速开关、是否启动运动）
 *
 * @param vx 世界坐标系 X 方向速度（向前）
 * @param vy 世界坐标系 Y 方向速度（向右）
 * @param rs 旋转速度（顺时针为正）
 * @param ramp 是否启用加减速
 * @param enable 是否启用运动控制
 */
void SetMoveParameters(int vx, int vy, int rs, bool ramp, bool enable) {
    world_x = vx;
    world_y = vy;
    rotate_speed = rs;
    ramp_enable = ramp;
    move_enable_flag = enable;
}
