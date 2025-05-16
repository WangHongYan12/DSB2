#include "yaw_pid_control.h"  // 确保包含了函数声明
#include "F:/Project/DSB2/Core/Src/uart2_motor_frame_tx/uart2_motor_frame_tx.h"  // 电机控制函数头文件

// —— PID 参数 ——
// 目标偏航角和当前偏航角由外部更新
volatile int32_t target_yaw_cmd_cdeg = 0;   // 用户或上层给定的目标角度（硬目标）
volatile int32_t target_yaw_cdeg = 0;       // 经过软目标斜坡后的目标角度（实际传递给 PID）
extern volatile int32_t imu_yaw_cdeg; // 当前偏航角（单位：0.01°）
bool yaw_pid_control_on = false;
// PID 控制参数
static const int32_t yaw_pid_kp = 5000;
static const int32_t yaw_pid_ki = 10;
static const int32_t yaw_pid_kd = 100;

// 输出限幅
#define MAX_RSPEED   127
#define MIN_RSPEED  -127

// 积分限幅，避免积分过度
#define MAX_INTEGRAL 10000  // 可根据实际情况调整
#define MIN_INTEGRAL -10000


/**
 * @brief  每次定时器中断调用：PID 计算并下发给底盘
 */
void yaw_pid_control_callback(void)
{
    // 1. 计算偏航误差（环形误差，单位 centi-deg）
    int32_t error = target_yaw_cdeg - imu_yaw_cdeg;

    // 把误差收敛到 [-18000, +18000] 范围内（环形误差）
    while (error >  18000) error -= 36000;
    while (error < -18000) error += 36000;

    // 2. PID 计算
    static int64_t integral   = 0;  // 使用 int64_t 防止溢出
    static int32_t prev_error = 0;

    integral += error;                           // 积分
    if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;  // 防止积分过度
    if (integral < MIN_INTEGRAL) integral = MIN_INTEGRAL;

    int32_t derivative = error - prev_error;     // 微分

    // 用 int64_t 存储中间结果，避免溢出
    int64_t rspeed = -(yaw_pid_kp * (int64_t)error  // 强制转换为 int64_t
                       + yaw_pid_ki * integral
                       + yaw_pid_kd * derivative) / 100000;  // 计算结果

    // 3. 限幅
    if (rspeed > MAX_RSPEED) rspeed = MAX_RSPEED;
    else if (rspeed < MIN_RSPEED) rspeed = MIN_RSPEED;

    prev_error = error;

    // 4. 下发给底盘：前后 xspeed，左右 yspeed，旋转 rspeed
    Vehicle_movement(0, 0, (int)rspeed);  // 转换为 int 类型传递给控制函数
}

/**
 * @brief  车体运动控制
 * @param xspeed  前后线速度
 * @param yspeed  左右线速度
 * @param rspeed  旋转速度（正为顺时针，负为逆时针）
 */
void Vehicle_movement(int xspeed, int yspeed, int rspeed)
{
    int speeda = xspeed - yspeed - rspeed;
    int speedb = xspeed + yspeed - rspeed;
    int speedc = xspeed + yspeed + rspeed;
    int speedd = xspeed - yspeed + rspeed;

    MotorFrame_SetAngleVelocity(speeda, speedb, speedc, speedd);
}

void Speed_Control(int xspeed, int yspeed, int rspeed , bool ramp)
{
    int speeda = xspeed - yspeed - rspeed;
    int speedb = xspeed + yspeed - rspeed;
    int speedc = xspeed + yspeed + rspeed;
    int speedd = xspeed - yspeed + rspeed;

    MotorFrame_SetSpeedAndCtrl(speeda, speedb, speedc, speedd , ramp);
}
