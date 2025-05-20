/* =============================================================================
 *  line_follow_pid.c
 *  8 路红外寻迹 PID 模块实现（位置式 PID）
 * =============================================================================*/
#include "stm32f1xx_hal.h"
#include "line_follow_pid.h"
#include "../yaw_pid_control/yaw_ramp.h"
#include "../linetracker_uart5/linetracker_uart5.h"
#include "../yaw_pid_control/yaw_pid_control.h"
#include "../imu_uart4/imu_uart4.h"

bool Line_Follow_On = false;

#define NUM_SENSORS        8
#define END_LINE_THRESHOLD 7
#define LINE_FOLLOW_KP     8      /* P 系数 */
#define LINE_FOLLOW_KI     0       /* I 系数 */
#define LINE_FOLLOW_KD     0       /* D 系数 */
#define OUTPUT_MAX         32767   /* 输出限幅 */
#define OUTPUT_MIN       -32768
#define INTEGRAL_MAX       10000   /* 积分限幅防风暴 */

static const int8_t weight[NUM_SENSORS] = { -80, -50, -15, -2, 2, 15, 50, 80 };

static int16_t  sum_weight;               /* 权重和 */
static int8_t   sum_count;                /* 检测到的黑线数量 */
static int16_t  err_prev = 0;             /* 上次误差 */
static int32_t  err_integral = 0;         /* 积分项累计 */
static int32_t  pid_output = 0;           /* 最终 PID 输出 (centi-degree) */
static bool     reached_end_flag = false; /* 终点标志 */

static void    Line_nspection_data_synthesis(void);
static int32_t Line_Follow_PID_Compute(void);

int32_t yaw_goal;
void Line_Follow_PID_Control(bool use_ramp)
{
    /* 数据合成 */
    Line_nspection_data_synthesis();
    /* 计算位置式 PID 输出 */
    yaw_goal += Line_Follow_PID_Compute();
    /* 如果未到终点，应用朝向目标 */
    yaw_ramp_set_goal(yaw_goal, use_ramp);

}

bool Line_Follow_PID_IsFinished(void)
{
    return reached_end_flag;
}

void Remake_reached_end_flag(void){
    reached_end_flag = false;
}

static void Line_nspection_data_synthesis(void)
{
    sum_weight = 0;
    sum_count  = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        if (lt_digital[i])
        {
            sum_weight += weight[i];
            sum_count++;
        }
    }
    /* 终点检测 */
    if (sum_count >= END_LINE_THRESHOLD)
    {
        reached_end_flag = true;
    }
}

static int32_t Line_Follow_PID_Compute(void)
{
    int16_t error;
    error = sum_weight / sum_count;

    /* P 项 */
    int32_t P_out = (int32_t)LINE_FOLLOW_KP * error;
    /* I 项累加并限幅 */
    err_integral += error;
    if (err_integral >  INTEGRAL_MAX) err_integral =  INTEGRAL_MAX;
    if (err_integral < -INTEGRAL_MAX) err_integral = -INTEGRAL_MAX;
    int32_t I_out = (int32_t)LINE_FOLLOW_KI * err_integral;
    /* D 项 */
    int32_t D_out = (int32_t)LINE_FOLLOW_KD * (error - err_prev);
    /* 位置式 PID 输出合成 */
    pid_output = P_out + I_out + D_out;
    /* 限幅 */
    if (pid_output >  OUTPUT_MAX) pid_output =  OUTPUT_MAX;
    if (pid_output <  OUTPUT_MIN) pid_output =  OUTPUT_MIN;
    /* 更新历史误差 */
    err_prev = error;
    return pid_output;
}

void Start_Line_Follow(void){
    yaw_goal = imu_yaw_cdeg;
    Line_Follow_On = true;
    Speed_Control(64,0,0,true);
    yaw_pid_control_on = true;
}

void Stop_Line_Follow(void){
    Line_Follow_On = false;
    Speed_Control(0,0,0,true);
    HAL_Delay(300);
    Speed_Control(0,0,0,false);
}

void Start_Line_Follow_Fast(void){
    yaw_goal = imu_yaw_cdeg;
    Line_Follow_On = true;
    Speed_Control(100,0,0,true);
    yaw_pid_control_on = true;
}

void Stop_Line_Follow_Fast(void){
    Line_Follow_On = false;
    Speed_Control(0,0,0,true);
    HAL_Delay(300);
    Speed_Control(0,0,0,true);
}
/* =============================================================================
 *  使用示例：主循环调用
 * =============================================================================
 * #include "line_follow_pid.h"
 * extern volatile uint8_t lt_digital[8];
 * extern void yaw_ramp_set_goal(int32_t, bool);
 *
 * int main(void)
 * {
 *     HAL_Init();
 *     // 其他外设初始化
 *     while (1)
 *     {
 *         Line_Follow_PID_Control(true);
 *         if (Line_Follow_PID_IsFinished())
 *         {
 *             // 终点逻辑
 *         }
 *     }
 * }
 */
