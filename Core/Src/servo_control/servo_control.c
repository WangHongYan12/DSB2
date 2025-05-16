
/* =============================================================================
 *  servo_control.c
 *  270° 舵机 (50 Hz PWM) 驱动模块实现
 * =============================================================================*/
#include "stm32f1xx_hal.h"
#include "servo_control.h"

/* 外部定时器句柄，由 CubeMX 生成 */
extern TIM_HandleTypeDef htim4;

/* ----- 舵机参数 ----- */
#define SERVO_PULSE_MIN    500U    /* 最小时脉宽 0.5 ms */
#define SERVO_PULSE_MAX   2500U    /* 最大时脉宽 2.5 ms */
#define SERVO_ANGLE_MAX   270U     /* 舵机最大转角 270° */


/**
 * @brief 启动 TIM4 CH1/CH2 PWM 输出
 */
void servo_init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

/**
 * @brief 设置舵机角度 (0–270°)
 */
void servo_set_angle(uint32_t channel, uint16_t angle_deg)
{
    uint32_t pulse_range = SERVO_PULSE_MAX - SERVO_PULSE_MIN;
    uint32_t pulse;

    /* 限幅角度 */
    if (angle_deg > SERVO_ANGLE_MAX)
        angle_deg = SERVO_ANGLE_MAX;

    /* 线性映射: 0° → 0.5ms; 270° → 2.5ms */
    pulse = SERVO_PULSE_MIN + (pulse_range * angle_deg) / SERVO_ANGLE_MAX;

    /* 更新 CCR，单位为计数 tick (1 tick = 1 µs) */
    __HAL_TIM_SET_COMPARE(&htim4, channel, pulse);
}


/* =============================================================================
 *  使用示例：主循环中调用
 * =============================================================================
 * #include "servo_control.h"
 *
 * int main(void)
 * {
 *     HAL_Init();
 *     SystemClock_Config();
 *     MX_GPIO_Init();
 *     MX_TIM4_Init();
 *
 *     servo_init();
 *
 *     while (1)
 *     {
 *         // 0°
 *         servo_set_angle(TIM_CHANNEL_1, 0);
 *         HAL_Delay(1000);
 *         // 135°
 *         servo_set_angle(TIM_CHANNEL_1, 135);
 *         HAL_Delay(1000);
 *         // 270°
 *         servo_set_angle(TIM_CHANNEL_1, 270);
 *         HAL_Delay(1000);
 *     }
 * }
 */
