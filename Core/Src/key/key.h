/**
 * @file    key.h
 * @brief   按键驱动（PA4‒PA7，下降沿外部中断 + 软去抖）
 * @details
 * ┌──────┬────┬───────────┐
 * │ 枚举 │ 管脚│ 典型标识 │
 * ├──────┼────┼───────────┤
 * │ KEY1 │ PA4│ “K1”      │
 * │ KEY2 │ PA5│ “K2”      │
 * │ KEY3 │ PA6│ “K3”      │
 * │ KEY4 │ PA7│ “K4”      │
 * └──────┴────┴───────────┘
 *
 * - 建议 PCB 接法：常开按键 → 引脚，另一端 → GND；管脚内部上拉。
 * - CubeMX 配置：
 *   * Pin  : PA4‒PA7 → GPIO_Input
 *   * Mode : External Interrupt  **Falling Edge**
 *   * Pull : Pull‑up
 *   * NVIC : 使能 **EXTI4** 与 **EXTI9_5**
 *
 * @note    使用步骤
 *          1. `MX_GPIO_Init()`（CubeMX 生成）
 *          2. `Key_Init()` 初始化
 *          3. 在循环或任务里用 `Key_IsPressed()` / `Key_GetClick()` 读取
 *
 * @author  王泓俨
 * @date    2025‑05‑06
 */

#ifndef KEY_H_
#define KEY_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 按键枚举（从 0 开始，方便作数组索引） */
typedef enum
{
    KEY1 = 0,
    KEY2,
    KEY3,
    KEY4,
    KEY_NUM   /**< 按键数量，放最后 */
} KeyId_t;

/*---------------------------------------------------------------------------*/
/*                                配置项                                     */
/*---------------------------------------------------------------------------*/

/** @brief 软去抖窗口（毫秒）。修改后需重新编译 */
#define KEY_DEBOUNCE_MS   10U

/*---------------------------------------------------------------------------*/
/*                          公共 API                                          */
/*---------------------------------------------------------------------------*/

/**
 * @brief  初始化按键驱动（清状态 + 使能 NVIC）
 * @note   应在 `MX_GPIO_Init()` 之后且 OS 调度开启之前调用
 */
void Key_Init(void);

/**
 * @brief  实时读取按键电平
 * @param  id   按键编号 #KeyId_t
 * @retval true  按键处于按下（低电平）稳定状态
 * @retval false 按键弹起
 */
bool Key_IsPressed(KeyId_t id);

/**
 * @brief  读取并清除“一次性单击事件”
 * @param  id  按键编号
 * @return 如果自上次调用后检测到一次按下跳变，则返回 true；否则 false
 * @note   典型用法：菜单翻页、参数 ++
 */
bool Key_GetClick(KeyId_t id);

#ifdef __cplusplus
}
#endif
#endif /* KEY_H_ */
