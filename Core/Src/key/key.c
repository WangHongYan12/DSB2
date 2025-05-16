/**
 * @file    key.c
 * @brief   EXTI‑based key driver (PA4‒PA7)
 */

#include "key.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"          /* MX_GPIO_Init() 原型 */

/* ---- 本地常量 --------------------------------------------------------- */
static const uint16_t s_keyPin [KEY_NUM]  = {K1_Pin, K2_Pin,
                                             K3_Pin, K4_Pin};
static GPIO_TypeDef * s_keyPort[KEY_NUM]  = {GPIOA, GPIOA, GPIOA, GPIOA};

/* ---- 内部状态 --------------------------------------------------------- */
static volatile uint8_t g_keyLevel[KEY_NUM];      /* 当前稳定电平 0/1 */
static volatile uint8_t g_keyClick[KEY_NUM];      /* 单击标志         */
static uint32_t         g_keyLastMs[KEY_NUM];     /* 上次中断时间戳   */

/* ---- 内部工具函数 ----------------------------------------------------- */
static inline void KEY_NVIC_Config(void)
{
    /* 优先级分配：此处给中断组 5，可根据系统需要调整 */
    HAL_NVIC_SetPriority(EXTI4_IRQn,   5, 0);
    HAL_NVIC_EnableIRQ  (EXTI4_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ  (EXTI9_5_IRQn);
}

/*---------------------------------------------------------------------------*/
/*                                API 实现                                   */
/*---------------------------------------------------------------------------*/
void Key_Init(void)
{
    /* 清状态 */
    for (uint8_t i = 0; i < KEY_NUM; ++i)
    {
        g_keyLevel[i]  = 0U;
        g_keyClick[i]  = 0U;
        g_keyLastMs[i] = 0U;
    }
    KEY_NVIC_Config();
}

bool Key_IsPressed(KeyId_t id)
{
    if (id >= KEY_NUM) return false;

    /* 若记录为按下但引脚已回到高电平，则复位 */
    if (g_keyLevel[id] &&
        HAL_GPIO_ReadPin(s_keyPort[id], s_keyPin[id]) == GPIO_PIN_SET)
    {
        g_keyLevel[id] = 0U;
    }
    return (bool)g_keyLevel[id];
}

bool Key_GetClick(KeyId_t id)
{
    if (id >= KEY_NUM) return false;
    bool clicked = (bool)g_keyClick[id];
    g_keyClick[id] = 0U;
    return clicked;
}

/*---------------------------------------------------------------------------*/
/*                         HAL 回调 & IRQHandlers                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief  HAL EXTI 通用回调
 * @note   仅处理下降沿（按下）事件；上升沿依靠轮询复位
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < KEY_NUM; ++i)
    {
        if (GPIO_Pin == s_keyPin[i])
        {
            /* 软件去抖：两次中断间隔 >= KEY_DEBOUNCE_MS 才算有效 */
            if (now - g_keyLastMs[i] >= KEY_DEBOUNCE_MS)
            {
                g_keyLastMs[i] = now;
                g_keyLevel[i]  = 1U;   /* 记录为“按下”          */
                g_keyClick[i]  = 1U;   /* 产生一次单击事件       */
            }
            return;
        }
    }
}

/* ---- EXTI IRQHandlers -------------------------------------------------- */


