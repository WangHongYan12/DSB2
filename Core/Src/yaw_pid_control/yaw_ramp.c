#include "yaw_ramp.h"
#include "yaw_pid_control.h"
#include <stdbool.h>
#include <stdint.h>

/*──────────── 内部状态 ────────────*/
static int32_t  ramp_start_cdeg = 0;        /* 起始软目标 */
static int32_t  ramp_delta_cdeg = 0;        /* 目标差值 (-18000..+18000) */
static uint16_t ramp_tick       = 0;        /* 进度 0..ramp_ticks_total */
static bool     ramp_active     = false;

/* 默认值定义，只在本文件中使用 */
uint16_t ramp_ticks_total = RAMP_TICKS_TOTAL_DEFAULT;

/*------------------------------------------------------------
 * 环形角差: 把 diff 收敛到 [-18000, +18000]
 *-----------------------------------------------------------*/
static int32_t shortest_diff(int32_t diff)
{
    while (diff >  18000) diff -= 36000;
    while (diff < -18000) diff += 36000;
    return diff;
}

/*------------------------------------------------------------
 * ease‑in‑out quadratic  (二次 S 曲线)
 * t∈[0,1] → s∈[0,1]  前半段 y=x²  后半段 y=-x²
 *-----------------------------------------------------------*/
static float ease_in_out_quad(float t)
{
    return (t < 0.5f) ? 2.0f * t * t
                      : (-2.0f * t * t + 4.0f * t - 1.0f);
}

/*------------------------------------------------------------
 * ① 外部 API : 提交新的硬目标，并决定是否平滑斜坡过渡
 *-----------------------------------------------------------*/
void yaw_ramp_set_goal(int32_t new_goal_cdeg, bool use_ramp)
{
    /* 如果目标没变且正在斜坡，则无需重启 */
    if (ramp_active && new_goal_cdeg == target_yaw_cmd_cdeg)
        return;

    target_yaw_cmd_cdeg = new_goal_cdeg;

    if (!use_ramp)
    {
        /* 立即跳转到目标，无需斜坡 */
        ramp_active     = false;
        ramp_tick       = ramp_ticks_total;
        target_yaw_cdeg = new_goal_cdeg;
        return;
    }

    /* 启动斜坡 */
    ramp_start_cdeg = target_yaw_cdeg;
    ramp_delta_cdeg = shortest_diff(new_goal_cdeg - ramp_start_cdeg);
    ramp_tick       = 0;
    ramp_active     = true;
}

/*------------------------------------------------------------
 * ② 每 tick 调用：推进斜坡
 *-----------------------------------------------------------*/
void yaw_ramp_update(void)
{
    if (!ramp_active) return;

    float t = (float)ramp_tick / (float)ramp_ticks_total;   /* 0..1 */
    if (t > 1.0f) t = 1.0f;

    float s = ease_in_out_quad(t);                          /* 曲线映射 */
    target_yaw_cdeg = ramp_start_cdeg + (int32_t)(s * (float)ramp_delta_cdeg);

    if (++ramp_tick >= ramp_ticks_total)
    {
        ramp_active     = false;
        target_yaw_cdeg = ramp_start_cdeg + ramp_delta_cdeg; /* 精确落点 */
    }
}

/*------------------------------------------------------------
 * ③ 查询接口：是否仍在 ramp 中
 *-----------------------------------------------------------*/
bool yaw_ramp_is_active(void)
{
    return ramp_active;
}

/*------------------------------------------------------------
 * ④ 外部 API: 强制跳转至目标，立即终止 ramp
 *-----------------------------------------------------------*/
void yaw_ramp_force_done(void)
{
    ramp_active     = false;
    ramp_tick       = ramp_ticks_total;
    target_yaw_cdeg = target_yaw_cmd_cdeg;
}

/*------------------------------------------------------------
 * ⑤ 修改斜坡总时长接口（可动态设置）
 *-----------------------------------------------------------*/
void Ramp_SetTicksTotal(uint16_t ticks)
{
    if (ticks > 0)
        ramp_ticks_total = ticks;
}

