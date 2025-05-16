/* linetracker_uart5.c –– 8路巡线模块 UART‑5 “数字型” 驱动
 *
 * 协议：主控发送 $0,0,1# 后，模块持续输出
 *       $D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#
 * 其中 xN 取 0/1 表示黑/白线（或反之，视模块而定）
 */

#include "stm32f1xx_hal.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>



/* ====== 用户可直接读取的全局数据 ====================================== */
volatile uint8_t lt_digital[8];   /* 8 路数字值：0/1 */
volatile bool    lt_new_digital = false;

/* ====== 私有接收状态机 ================================================== */
#define RX_BUF_LEN  64
static uint8_t  g_rxByte;                 /* 单字节 Rx 缓冲 */
static uint8_t  rx_buf[RX_BUF_LEN];
static uint16_t rx_idx     = 0;
static bool     frame_ready = false;

/* ====== 内部函数 ======================================================== */
static void LT_ParseDigitalFrame(const char *frame);

/* ---------------------------------------------------------------------- */
/* 1. 初始化：在 MX_USART5_UART_Init() 之后调用一次                        */
/*    功能：① 启动单字节中断接收                                          */
/*         ② 发送数字型指令 $0,0,1#                                       */
void LT_UART5_Init(void)
{
    /* ① 先启动接收 */
    HAL_UART_Receive_IT(&huart5, &g_rxByte, 1);

    /* ② 立刻让模块进入数字型输出 */
    const char cmd[] = "$0,0,1#";
    HAL_UART_Transmit(&huart5, (uint8_t *)cmd, sizeof(cmd) - 1, HAL_MAX_DELAY);
}

/* ---------------------------------------------------------------------- */
/* 2. 在 USART5 Rx 完成中断里调用：拼接一帧                               */
void LT_UART5_RxISR(void)
{
    if (g_rxByte == '$')                 /* 帧头 */
        rx_idx = 0;

    if (rx_idx < RX_BUF_LEN)
        rx_buf[rx_idx++] = g_rxByte;

    if (g_rxByte == '#')                 /* 帧尾 */
    {
        rx_buf[rx_idx] = '\0';           /* 方便用 C 字符串解析 */
        frame_ready = true;
    }

    /* 重新挂起下一字节接收 */
    HAL_UART_Receive_IT(&huart5, &g_rxByte, 1);
}

/* ---------------------------------------------------------------------- */
/* 3. 在定时器（100 Hz TIM6）或主循环里调用：解析数字帧                   */
void LT_Process(void)
{
    if (!frame_ready) return;

    frame_ready = false;
    LT_ParseDigitalFrame((char *)rx_buf);
}

/* ====== 解析数字型帧 ==================================================== */
static void LT_ParseDigitalFrame(const char *frame)
{
    if (frame[0] != '$' || frame[1] != 'D') return;  /* 只处理数字型 */

    /* 样例形如: $D,x1:0,x2:1,...,x8:0# */
    const char *p = strchr(frame, ',');              /* 指向第一个 ',' */

    for (int i = 0; i < 8 && p; ++i)
    {
        const char *colon = strchr(p, ':');
        if (!colon) break;

        lt_digital[i] = abs((uint8_t)atoi(colon + 1) - 1);    /* 转成 0 / 1 */
        p = strchr(colon, ',');                      /* 寻找下一个 ',' */
    }


    lt_new_digital = true;                           /* 通知上层有新数据 */
}

/* ---------------------------------------------------------------------- */
/* 把以下示例代码复制到对应文件                                */

/*
#include "linetracker_uart5.c"  // 或将其拆为 .h/.c 分离

// stm32f1xx_it.c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
        LT_UART5_RxISR();
}

// timer 100 Hz 回调
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        LT_Process();

        if (lt_new_digital)
        {
            lt_new_digital = false;
            // 这里安全使用 lt_digital[0..7]
        }
    }
}
*/
