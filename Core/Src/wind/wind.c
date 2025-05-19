/* wind.c – TB6612 驱动的风扇电机控制
 * 王泓俨；25/5/6
 * 硬件映射
 * ┌────────────┬──────────┬──────────────────┐
 * │ TB6612 引脚│  STM32F103│  说明            │
 * ├────────────┼──────────┼──────────────────┤
 * │  IN1       │ PA15      │ 方向 A（正转）   │
 * │  IN2       │ PA14      │ 方向 B（反转）   │
 * │  PWMA      │ PA1 (TIM2‑CH2) │ PWM 调速   │
 * └────────────┴──────────┴──────────────────┘
 *
 * CubeMX 已经做的配置
 *  • PA15 / PA14 → GPIO_Output, 推挽, 50 MHz
 *  • TIM2‑CH2    → PWM Generation CH2
 *      Prescaler = 71   (72 MHz / 72 = 1 MHz 计数频率)
 *      Period    = 1000 (PWM = 1 kHz，分辨率 0‥1000)
 *
 * 使  用  步  骤
 * --------------------------------------------------------------------------
 * 1) MX_GPIO_Init(); MX_TIM2_Init();  // CubeMX 生成
 * 2) Wind_Init();                     // 启动 PWM，并默认停止电机
 * 3) Wind_SetDirection(WIND_CW);      // 设定转向
 *    Wind_SetSpeed(800);              // 80% 占空比，加速
 * 4) Wind_Stop();                     // 停止
 */

#include "main.h"      /* 包含 GPIO/TIM 句柄 */
#include <stdbool.h>

/* ====================== 本地宏 ====================== */
#define WIND_TIMER         htim2
#define WIND_PWM_CHANNEL   TIM_CHANNEL_2

/* 方向枚举 */
typedef enum { WIND_CW = 0, WIND_CCW = 1 } WindDir_t;

/* ====================== 外部句柄 ==================== */
extern TIM_HandleTypeDef WIND_TIMER;   /* 由 CubeMX 生成 */

/* ====================== 初始化 ====================== */
void Wind_Init(void)
{
    /* 关闭 SWJ（保险起见） */
    //__HAL_AFIO_REMAP_SWJ_DISABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG(); // 保留 SWD

    /* 关闭输出脚，防止突跳 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 | GPIO_PIN_14, GPIO_PIN_RESET);

    /* 启动 PWM 输出（占空先 0） */
    __HAL_TIM_SET_COMPARE(&WIND_TIMER, WIND_PWM_CHANNEL, 0);
    HAL_TIM_PWM_Start(&WIND_TIMER, WIND_PWM_CHANNEL);
}

/* ====================== 设定方向 ==================== */
void Wind_SetDirection(WindDir_t dir)
{
    switch (dir) {
        case WIND_CW:   /* 顺时针：IN1 = 1, IN2 = 0 */
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET);
            break;

        case WIND_CCW:  /* 逆时针：IN1 = 0, IN2 = 1 */
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_SET);
            break;
    }
}

/* ====================== 调速函数 ==================== */
/**
 * @brief  设置占空比（0‑1000 对应 0‑100%）
 */
void Wind_SetSpeed(uint16_t duty)
{
    if (duty > 1000) duty = 1000;
    __HAL_TIM_SET_COMPARE(&WIND_TIMER, WIND_PWM_CHANNEL, duty);
}

/* ====================== 刹车 / 停止 ================= */
/**
 * @brief  快速制动：IN1=IN2=1, PWM=0
 */
void Wind_Brake(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 | GPIO_PIN_14, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&WIND_TIMER, WIND_PWM_CHANNEL, 0);
}

/**
 * @brief  断电滑行：IN1=IN2=0, PWM=0
 */
void Wind_Stop(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 | GPIO_PIN_14, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&WIND_TIMER, WIND_PWM_CHANNEL, 0);
}

/* ====================== 示例 ======================== */
/*
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM2_Init();

    Wind_Init();

    Wind_SetDirection(WIND_CW);
    Wind_SetSpeed(700);   // 70% 占空比

    while (1) {
        HAL_Delay(3000);
        Wind_SetSpeed(300);   // 转慢
        HAL_Delay(3000);
        Wind_Stop();
        HAL_Delay(3000);
        Wind_SetDirection(WIND_CCW);
        Wind_SetSpeed(800);   // 逆时针全速
    }
}
*/
