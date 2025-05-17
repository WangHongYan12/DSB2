/**
 * @file    vision_parser.c
 * @brief   实现视觉数据帧的串口接收与解析（通过 USART3）
 * @author  王泓俨
 * @version 1.0
 */

#include "vision_parser.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/// 数据帧总长度（包括帧头和帧尾）
#define VISION_FRAME_LENGTH     13
#define VISION_FRAME_HEAD       '!'
#define VISION_FRAME_TAIL       '#'

/// 串口接收的单字节缓存
static uint8_t vision_uart_rx_byte;
/// 数据帧缓冲区
static uint8_t vision_frame_buffer[VISION_FRAME_LENGTH];
/// 当前写入帧缓冲区的索引
static uint8_t vision_frame_index = 0;
/// 标志位：表示当前帧已接收完成
static uint8_t vision_frame_ready = 0;

/// 当前帧解析出的标签名（3字符 + '\0'）
char vision_tag[4] = {0};
/// 当前帧解析出的 X 坐标
uint16_t vision_x_coord = 0;
/// 当前帧解析出的 Y 坐标
uint16_t vision_y_coord = 0;

/// 使用串口 USART3 与视觉模块通信
extern UART_HandleTypeDef huart3;

/**
 * @brief 初始化视觉数据串口接收（开启 USART3 中断接收）
 */
void vision_uart_init(void) {
    HAL_UART_Receive_IT(&huart3, &vision_uart_rx_byte, 1);
}

/**
 * @brief 串口接收中断回调函数（由 HAL 调用）
 * @param huart 串口句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        if (vision_uart_rx_byte == VISION_FRAME_HEAD) {
            vision_frame_index = 0;
            vision_frame_buffer[vision_frame_index++] = vision_uart_rx_byte;
        } else if (vision_frame_index > 0 && vision_frame_index < VISION_FRAME_LENGTH) {
            vision_frame_buffer[vision_frame_index++] = vision_uart_rx_byte;

            if (vision_frame_index == VISION_FRAME_LENGTH) {
                if (vision_frame_buffer[VISION_FRAME_LENGTH - 1] == VISION_FRAME_TAIL) {
                    vision_frame_ready = 1;
                }
                vision_frame_index = 0;
            }
        }

        // 继续下一字节接收
        HAL_UART_Receive_IT(&huart3, &vision_uart_rx_byte, 1);
    }
}

/**
 * @brief 解析已接收完成的一帧数据，提取标签与坐标
 * @note  仅当 vision_frame_ready 被置位时执行解析
 */
void vision_frame_parse(void) {
    if (!vision_frame_ready) return;
    vision_frame_ready = 0;

    // 校验帧头和帧尾
    if (vision_frame_buffer[0] != VISION_FRAME_HEAD || vision_frame_buffer[12] != VISION_FRAME_TAIL) return;

    // 标签名
    memcpy(vision_tag, &vision_frame_buffer[1], 3);
    vision_tag[3] = '\0';

    // 坐标字符串临时缓冲
    char vision_x_str[5] = {0};
    char vision_y_str[5] = {0};

    memcpy(vision_x_str, &vision_frame_buffer[4], 4);
    memcpy(vision_y_str, &vision_frame_buffer[8], 4);

    // 字符串转整数
    vision_x_coord = (uint16_t)atoi(vision_x_str);
    vision_y_coord = (uint16_t)atoi(vision_y_str);
}
