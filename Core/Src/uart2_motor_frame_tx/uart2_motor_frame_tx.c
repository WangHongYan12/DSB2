/**
 ******************************************************************************
 * @file    uart2_motor_frame_tx.c
 * @brief   UART2 “# … !” 协议发送实现
 ******************************************************************************
 */
#include "uart2_motor_frame_tx.h"
#include "usart.h"          /* CubeMX 生成，声明 extern UART_HandleTypeDef huart2 */
#include "../motor_rx/motor_rx.h"
#include "../buzzer/buzzer.h"

/*===================== 协议常量 ====================*/
#define FRAME_LEN        11u
#define FRAME_HEAD       0x23u
#define FRAME_TAIL       0x21u
#define CTRL_TRAP_ENABLE 0x01u    /* bit0 = 梯形加减速使能 */

/*===================== 发送缓冲区 ==================*/
static uint8_t  txBuf[FRAME_LEN] = {FRAME_HEAD, 0,0,0,0, 0, 0,0,0,0, FRAME_TAIL};
static volatile bool txBusy = false;          /* 非阻塞发送中的状态标志 */

int8_t last_set_speed[4] = {0};
/*--------------------------------------------------
 * 可选初始化：清 busy 标志
 *-------------------------------------------------*/
void MotorFrame_UART2_TxInit(void)
{
    txBusy = false;
}

/*--------------------------------------------------
 * 设置 1‑4 字节 + 控制字（不立即发送）
 *-------------------------------------------------*/
void MotorFrame_SetSpeedAndCtrl(int8_t speedA,
                                int8_t speedB,
                                int8_t speedC,
                                int8_t speedD,
                                bool   trapezoid)
{
    txBuf[1] = (uint8_t)(-speedA);
    txBuf[2] = (uint8_t)(-speedB);
    txBuf[3] = (uint8_t)speedC;
    txBuf[4] = (uint8_t)speedD;
    txBuf[5] = trapezoid ? CTRL_TRAP_ENABLE : 0u;

    last_set_speed[0] = speedA;
    last_set_speed[1] = speedB;
    last_set_speed[2] = speedC;
    last_set_speed[3] = speedD;
}

/*--------------------------------------------------
 * 设置 6‑9 字节
 *-------------------------------------------------*/
void MotorFrame_SetAngleVelocity(int8_t velA,
                                 int8_t velB,
                                 int8_t velC,
                                 int8_t velD)
{
    txBuf[6] = (uint8_t)(-velA);
    txBuf[7] = (uint8_t)(-velB);
    txBuf[8] = (uint8_t)velC;
    txBuf[9] = (uint8_t)velD;

    MotorFrame_UART2_SendIT();
}

/*--------------------------------------------------
 * 阻塞方式发送（HAL_UART_Transmit）
 *-------------------------------------------------*/
HAL_StatusTypeDef MotorFrame_UART2_Send(void)
{
    return HAL_UART_Transmit(&huart2, txBuf, FRAME_LEN, 10);
}

/*--------------------------------------------------
 * 非阻塞方式发送（HAL_UART_Transmit_IT）
 *-------------------------------------------------*/
HAL_StatusTypeDef MotorFrame_UART2_SendIT(void)
{
    if (txBusy) return HAL_BUSY;          /* 上一次还没发完 */
    txBusy = true;
    return HAL_UART_Transmit_IT(&huart2, txBuf, FRAME_LEN);
}

/*--------------------------------------------------
 * HAL 中断完成回调：在全局 Tx 完成中断里调用
 *-------------------------------------------------*/
void MotorFrame_UART2_TxCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        txBusy = false;                   /* 发送结束，允许下一帧 */
    }
}

void MotorFrame_CheckAndSend(void)
{
    bool all_equal = true;


    for (int i = 0; i < 4; ++i)
    {
        if (motor_target_speed[i] != last_set_speed[i])
        {
            all_equal = false;
            break;
        }
    }
    if (!all_equal)
    {
        MotorFrame_UART2_SendIT();
    }
}

void Encoder_Reset(void){
    HAL_UART_Transmit(&huart2, "!@!", 3, HAL_MAX_DELAY);}