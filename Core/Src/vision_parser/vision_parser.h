/**
 * @file    vision_parser.h
 * @brief   声明视觉数据帧的解析接口
 */

#ifndef __VISION_PARSER_H__
#define __VISION_PARSER_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 当前帧解析结果：标签名（3字符 + '\0'）
 */
extern char vision_tag[4];

/**
 * @brief 当前帧解析结果：X 坐标（4位数）
 */
extern uint16_t vision_x_coord;

/**
 * @brief 当前帧解析结果：Y 坐标（4位数）
 */
extern uint16_t vision_y_coord;

/**
 * @brief 初始化视觉串口接收（启用 USART3 接收中断）
 */
void vision_uart_init(void);

/**
 * @brief 解析视觉模块发送的一帧数据
 * @note  应在主循环中周期性调用，用于提取 vision_tag、vision_x_coord、vision_y_coord
 */
void vision_frame_parse(void);

void Vision_RxISR(void);

extern uint8_t version_check;
#ifdef __cplusplus
}
#endif

#endif // __VISION_PARSER_H__
