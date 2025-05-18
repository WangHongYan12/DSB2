#ifndef MOVE_WHILE_ROTATING_H
#define MOVE_WHILE_ROTATING_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 世界坐标系下的目标速度（单位：用户自定义，如 mm/s）
 * 可在主程序中设置这些变量以改变运动方向与速度
 */
extern int world_x;            ///< 世界坐标系 X 方向速度（向前）
extern int world_y;            ///< 世界坐标系 Y 方向速度（向右）
extern int rotate_speed;       ///< 车体旋转速度（右旋为正）
extern bool ramp_enable;       ///< 是否启用梯形加减速
extern bool move_enable_flag;  ///< 使能标志位，控制是否执行运动逻辑
/**
 * @brief 设置运动控制参数（速度、旋转、梯形加减速、是否使能）
 *
 * @param vx 世界坐标系 X 方向速度（向前）
 * @param vy 世界坐标系 Y 方向速度（向右）
 * @param rs 旋转速度（右旋为正）
 * @param ramp 是否启用梯形加减速
 * @param enable 是否启用运动控制
 */
void SetMoveParameters(int vx, int vy, int rs, bool ramp, bool enable);

/**
 * @brief 执行运动控制：使车体在世界坐标中保持直线运动，同时自旋。
 * 该函数会读取 imu_yaw_cdeg，并通过 Speed_Control 控制底盘速度。
 * 仅当 move_enable_flag 为 true 时执行。
 */
void MoveWhileRotating(void);

/**
 * @brief 定时器中断服务函数，调用 MoveWhileRotating。
 * 应在定时器中断中调用，并清除中断标志（需用户根据平台实现）。
 */

#ifdef __cplusplus
}
#endif

#endif // MOVE_WHILE_ROTATING_H
