#include "motor_rx.h"
#include "usart.h"          /* 由 CubeMX 生成，包含 huart2 */
#include <string.h>
#include "F:\Project\DSB2\Core\Src\buzzer\buzzer.h"
/* ---------- 私有接收状态机 ---------- */
static uint8_t  rx_buf[MOTOR_RX_PKT_LEN];
static uint8_t  rx_idx  = 0;                 /* 正在累积的下标 */
static uint8_t  frame_buf[MOTOR_RX_PKT_LEN];
static volatile bool frame_ready = false;    /* 收到完整帧标志 */

/* 单字节 IT 接收缓冲 */
static uint8_t g_rxByte = 0;

/* ---------- 全局解析结果 ---------- */
volatile int16_t motor_target_speed[MOTOR_RX_NUM_AXES];
volatile int32_t motor_odometers   [MOTOR_RX_NUM_AXES];
volatile int32_t motor_sum_odometers_x;
volatile int32_t motor_sum_odometers_y;
volatile int32_t motor_real_speed  [MOTOR_RX_NUM_AXES];
volatile bool    motor_data_ready = false;

/* ========== ① 初始化：启动首次单字节接收 ========== */
void MotorRx_Init(void)
{
    /* 建议先显式清零所有状态，再挂接收 */
    rx_idx      = 0;
    frame_ready = false;
    memset(rx_buf,   0, sizeof(rx_buf));
    memset(frame_buf,0, sizeof(frame_buf));

    /* ★若可能，让上位机上电后延时 50~100 ms 再发首帧 */
    HAL_UART_Receive_IT(&huart2, &g_rxByte, 1);
}

/* ========== ② 串口 Rx 完成中断里调用 ========== */
void MotorRx_RxISR(void)
{
        if (rx_idx == 0)/* 寻找帧头 '#' */
        {
            if (g_rxByte != '#')
                goto _restart;
        }

        rx_buf[rx_idx++] = g_rxByte;

        if (rx_idx >= MOTOR_RX_PKT_LEN)
        {
            if (rx_buf[MOTOR_RX_PKT_LEN - 1] == '!')
            {

                memcpy(frame_buf, rx_buf, MOTOR_RX_PKT_LEN);
                frame_ready = true;
            }
            rx_idx = 0;                  /* 成功或失败都重同步 */
        }


    _restart:
    /* 继续收下一个字节 */
    HAL_UART_Receive_IT(&huart2, &g_rxByte, 1);
}

/* ========== ③ 定时器 20 Hz / 主循环里调用 ========== */
void MotorRx_Parse(void)
{
    //if (!frame_ready) return;            /* 没有完整帧 */

    const uint8_t *p = frame_buf + 1;    /* 跳过 '#' */

    for (int i = 0; i < MOTOR_RX_NUM_AXES; ++i)
    {
        memcpy((void*)&motor_target_speed[i], p, sizeof(int16_t));
        p += sizeof(int16_t);
    }
    for (int i = 0; i < MOTOR_RX_NUM_AXES; ++i)
    {
        memcpy((void*)&motor_odometers[i], p, sizeof(int32_t));
        motor_sum_odometers_x = ( (-motor_odometers[MOTOR_A]) + (-motor_odometers[MOTOR_B]) + motor_odometers[MOTOR_C] + motor_odometers[MOTOR_D]) / MOTOR_RX_NUM_AXES;
        motor_sum_odometers_y = ( -(-motor_odometers[MOTOR_A]) + (-motor_odometers[MOTOR_B]) + motor_odometers[MOTOR_C] - motor_odometers[MOTOR_D]) / MOTOR_RX_NUM_AXES;
        p += sizeof(int32_t);
    }
    for (int i = 0; i < MOTOR_RX_NUM_AXES; ++i)
    {
        memcpy((void*)&motor_real_speed[i], p, sizeof(int32_t));
        p += sizeof(int32_t);

    }

    //motor_data_ready = true;             /* 上层可读取数据 */
    //frame_ready      = false;            /* 允许下一帧进入 */
}

void Encoder_ResetAll(void){
    uint8_t cmd[] = "!@!";                      /* 3 字节：0x21 0x40 0x21 */

    HAL_UART_Transmit(&huart2,                 /* UART 句柄        */
                      cmd,                     /* 数据指针        */
                      sizeof(cmd) - 1U,        /* 发送长度：3     */
                      HAL_MAX_DELAY);          /* 阻塞直到完成    */
}