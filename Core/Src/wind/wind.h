#ifndef WIND_H
#define WIND_H
/* -----------------------------------------------------------------------
 * TB6612 风扇电机驱动 – 头文件
 * ---------------------------------------------------------------
 * 硬件绑定：
 *   IN1 → PA15   (GPIO_Output)
 *   IN2 → PA14   (GPIO_Output)
 *   PWMA→ PA1    (TIM2‑CH2 PWM)
 * PWM 频率与分辨率由 CubeMX 的 TIM2 设置决定
 * -----------------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----------- 方向枚举 -------------------------------------------------- */
typedef enum
{
    WIND_CW  = 0,   /* 顺时针  */
    WIND_CCW = 1    /* 逆时针  */
} WindDir_t;

/* ----------- API 原型 -------------------------------------------------- */
/**
 * @brief 初始化风扇驱动：启动 PWM 并默认停止电机
 * @note  请在 MX_GPIO_Init()、MX_TIM2_Init() 之后调用一次
 */
void Wind_Init(void);

/**
 * @brief 设定旋转方向
 * @param dir  WIND_CW / WIND_CCW
 */
void Wind_SetDirection(WindDir_t dir);

/**
 * @brief 设定 PWM 占空比
 * @param duty 0‑1000 对应 0‑100 %（假设 TIM.Period=1000）
 */
void Wind_SetSpeed(uint16_t duty);

/**
 * @brief 快速制动：IN1=IN2=High，PWM=0
 */
void Wind_Brake(void);

/**
 * @brief 断电滑行停机：IN1=IN2=Low，PWM=0
 */
void Wind_Stop(void);

#ifdef __cplusplus
}
#endif
#endif /* WIND_H */
