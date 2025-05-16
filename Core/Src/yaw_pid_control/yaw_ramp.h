#ifndef YAW_RAMP_H
#define YAW_RAMP_H

#include <stdint.h>
#include <stdbool.h>

#define RAMP_TICKS_TOTAL_DEFAULT   80u     /* 默认斜坡 tick 总数 */
#define YAW_RAMP_TICK_HZ           100u    /* 默认 100Hz 更新频率 */

extern int32_t target_yaw_cdeg;
extern int32_t target_yaw_cmd_cdeg;
extern uint16_t ramp_ticks_total;

/* 主要接口 */
void yaw_ramp_set_goal(int32_t new_goal_cdeg, bool use_ramp);  // 设置目标 & 是否使用 ramp
void yaw_ramp_update(void);                                    // 每 tick 调用
bool yaw_ramp_is_active(void);                                 // 是否还在 ramp 中
void yaw_ramp_force_done(void);                                // 强制终止 ramp，立即跳转
void Ramp_SetTicksTotal(uint16_t ticks);                       // 修改 ramp 时长

#endif /* YAW_RAMP_H */
