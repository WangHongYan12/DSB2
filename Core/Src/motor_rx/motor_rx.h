#ifndef MOTOR_RX_H
#define MOTOR_RX_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 固定参数 */
#define MOTOR_RX_NUM_AXES  4
#define MOTOR_RX_PKT_LEN   42      /* '#' + 4*int16 + 8*int32 + '!' */


#define MOTOR_A 0
#define MOTOR_B 1
#define MOTOR_C 2
#define MOTOR_D 3
/* =========== 解析结果：全局可直接访问 =========== */
extern volatile int16_t motor_target_speed[MOTOR_RX_NUM_AXES];
extern volatile int32_t motor_odometers   [MOTOR_RX_NUM_AXES];
extern volatile int32_t motor_sum_odometers_x;
extern volatile int32_t motor_sum_odometers_y;
extern volatile int32_t motor_real_speed  [MOTOR_RX_NUM_AXES];
extern volatile bool    motor_data_ready;          /* 解析完成置 1 */
/* =========== 仅需 3 个外部函数 =========== */
/* ①  初始化：在 MX_USART2_UART_Init() 之后调用一次 */
void MotorRx_Init(void);

/* ②  串口接收完成中断里调用（仅 USART2 分支） */
void MotorRx_RxISR(void);

/* ③  20 Hz 定时器中断（或主循环）里调用，解析一帧 */
void MotorRx_Parse(void);

void Encoder_ResetAll(void);
#ifdef __cplusplus
}
#endif
#endif /* MOTOR_RX_H */
