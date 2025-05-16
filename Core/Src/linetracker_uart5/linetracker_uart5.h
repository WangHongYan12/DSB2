#ifndef LINETRACKER_UART5_H
#define LINETRACKER_UART5_H
/* -------------------------------------------------------------------------
 * 8 路巡线传感模块 —— UART5 数字型驱动（只解析 $D,…# 帧）
 * Author: scu@why / 25/5/4
 * -------------------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======== 公共全局数据 ================================================= */
extern volatile uint8_t lt_digital[8];   /* 8 路 0/1 数字值 */
extern volatile bool    lt_new_digital;  /* 有新数据时置 true */

/* ======== 公共 API ===================================================== */
/**
 * @brief  初始化驱动：
 *         1) 启动 UART5 单字节中断接收
 *         2) 发送 “数字型数据” 命令 $0,0,1#
 */
void LT_UART5_Init(void);

/**
 * @brief  在 HAL_UART_RxCpltCallback() 里调用，把单字节喂给状态机
 */
void LT_UART5_RxISR(void);

/**
 * @brief  在 100 Hz TIM6 中断或主循环里调用，解析完整数字帧
 *         解析完成后 lt_digital[] 写入新的 8 路数值，并把 lt_new_digital 置 true
 */
void LT_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* LINETRACKER_UART5_H */
