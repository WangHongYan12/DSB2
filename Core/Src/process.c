#include <stdio.h>
#include <stdlib.h>
#include "buzzer/buzzer.h"
#include "oled/oled.h"
#include "motor_rx/motor_rx.h"
#include "imu_uart4/imu_uart4.h"
#include "uart2_motor_frame_tx/uart2_motor_frame_tx.h"
#include "linetracker_uart5/linetracker_uart5.h"
#include "yaw_pid_control/yaw_ramp.h"
#include "yaw_pid_control/yaw_pid_control.h"
#include "key/key.h"
#include "servo_control/servo_control.h"
#include "line_follow_pid/line_follow_pid.h"
#include "tim.h"
#include "vision_parser/vision_parser.h"
#include "wind/wind.h"
#include "move_while_rotating/move_while_rotating.h"
#include "vision_parser/vision_align.h"

//
// Created by 王泓俨 on 25-5-15.
//

int target_position = 3;
void process_Init(void){
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim5);

    HAL_Delay(200);
    OLED_Init();
    MotorFrame_UART2_TxInit();
    Buzzer_Init();
    servo_init();
    MotorRx_Init();          /* ① 启动单字节接收 */
    Key_Init();             /* ← 初始化按键驱动（必须在 GPIO 后） */
    IMU_UART4_Init();
    LT_UART5_Init();
    vision_uart_init();
    Wind_Init();
    Wind_SetDirection(WIND_CCW);
    Wind_SetSpeed(0);
    Buzzer_PlayMelody(MELODY_STARTUP);
}

char buffer[40];
void process_0(void){
    servo_set_angle(SERVO_1, 50);
    servo_set_angle(SERVO_2, 225);
    while(1){
    OLED_NewFrame();
    OLED_PrintString(4, 0, "电赛智能车 鸭敏电组", &font12x12, OLED_COLOR_REVERSED);
    OLED_DrawLine(0,14,128,14,OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(0, 16, "KEY1:", &afont12x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(0, 28, "KEY2:", &afont12x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(0, 40, "KEY3:", &afont12x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(0, 52, "KEY4:", &afont12x6, OLED_COLOR_NORMAL);

    OLED_PrintString(24, 16, "朝向环", &font12x12, OLED_COLOR_NORMAL);
    OLED_PrintString(24, 28, "清零角", &font12x12, OLED_COLOR_NORMAL);
    snprintf(buffer, sizeof(buffer), "VS:%d",version_check);
    OLED_PrintASCIIString(24, 40, buffer, &afont12x6, OLED_COLOR_NORMAL);
    OLED_PrintString(24, 52, "发车", &font12x12, OLED_COLOR_REVERSED);

    OLED_PrintString(64, 16, "里程", &font12x12, OLED_COLOR_NORMAL);
    snprintf(buffer, sizeof(buffer), "%d",target_position);
    OLED_PrintASCIIString(100, 16, buffer, &afont12x6, OLED_COLOR_NORMAL);
    OLED_PrintString(64, 28, "角度", &font12x12, OLED_COLOR_NORMAL);
    snprintf(buffer, sizeof(buffer), "%.2f",(float)imu_yaw_cdeg/100.0f);
    OLED_PrintASCIIString(88, 28, buffer, &afont12x6, OLED_COLOR_NORMAL);

    OLED_PrintString(64, 40, "速度", &font12x12, OLED_COLOR_NORMAL);
    snprintf(buffer, sizeof(buffer), "%d",last_set_speed[1]);
    OLED_PrintASCIIString(88, 40, buffer, &afont12x6, OLED_COLOR_NORMAL);

    OLED_PrintString(64, 52, "线数", &font12x12, OLED_COLOR_NORMAL);
    snprintf(buffer, sizeof(buffer), "%d",lt_digital[0]+lt_digital[1]+lt_digital[2]+lt_digital[3]+lt_digital[4]+lt_digital[5]+lt_digital[6]+lt_digital[7]);
    OLED_PrintASCIIString(100, 52, buffer, &afont12x6, OLED_COLOR_NORMAL);
    OLED_ShowFrame();

        if(Key_GetClick(KEY1)){
            target_position++;
            if(target_position == 7){target_position = 1;}}
        if (Key_GetClick(KEY2)){
            IMU_ResetHeading();}
        if (Key_GetClick(KEY3)){
            Buzzer_PlayMelody(MELODY_MAX);}
        if (Key_GetClick(KEY4)){
            yaw_pid_control_on = true;
            HAL_Delay(200);
            break; }
    }
}

void process_1(void){
    yaw_ramp_set_goal(18000,true);
    servo_set_angle(SERVO_1, 50);
    servo_set_angle(SERVO_2, 130);
    for(int i = 0 ; i < 64 ; i+=1){
        SetMoveParameters(-i,0,0,false,true);
        HAL_Delay(1);
    }
    while(yaw_ramp_is_active());
    SetMoveParameters(0,0,0,true,false);
    Speed_Control(64,0,0,false);
    Start_Line_Follow();
    while(motor_sum_odometers_x < 4000){}
    servo_set_angle(SERVO_1, 50);
    servo_set_angle(SERVO_2, 270);
    while(motor_sum_odometers_x < 12000){}
    Remake_reached_end_flag();
    while(1){
        if(Line_Follow_PID_IsFinished()){
            Stop_Line_Follow();
            break;
        }
    }
}

void process_2(void){
    servo_set_angle(SERVO_1, 50);
    servo_set_angle(SERVO_2, 225);
    yaw_ramp_set_goal(9000,true);
    HAL_Delay(500);
    Buzzer_PlayMelody(MELODY_PIRATES);
}
int yaw_123 = 0;
void process_3(void){
    switch(target_position){
        case 1:
            yaw_ramp_set_goal(0,true);
            for(int i = 0 ; i < 64 ; i++){
                SetMoveParameters(-i,0,0,false,true);
            }
            HAL_Delay(2500);
            SetMoveParameters(0,0,0,false,false);
            Speed_Control(-64,0,0,true);
            HAL_Delay(1200);
            Speed_Control(0,0,0,true);
            HAL_Delay(500);
            Speed_Control(0,-100,0,true);
            HAL_Delay(1200);
            yaw_123 = 0;
            Speed_Control(0,0,0,true);
            break;
        case 2:
            yaw_ramp_set_goal(0,true);
            for(int i = 0 ; i < 64 ; i++){
                SetMoveParameters(-i,0,0,false,true);
            }
            HAL_Delay(2500);
            SetMoveParameters(0,0,0,false,false);
            Speed_Control(-64,0,0,true);
            HAL_Delay(1200);
            Speed_Control(0,0,0,true);
            HAL_Delay(500);
            Speed_Control(0,-100,0,true);
            HAL_Delay(3400);
            Speed_Control(0,0,0,true);
            yaw_123 = 0;
            break;
        case 3:
            yaw_ramp_set_goal(0,true);
            for(int i = 0 ; i < 64 ; i++){
                SetMoveParameters(-i,0,0,false,true);
            }
            HAL_Delay(2500);
            SetMoveParameters(0,0,0,false,false);
            Speed_Control(-64,0,0,true);
            HAL_Delay(1200);
            Speed_Control(0,0,0,true);
            HAL_Delay(500);
            Speed_Control(0,-100,0,true);
            HAL_Delay(6000);
            Speed_Control(0,0,0,true);
            yaw_123 = 0;
            break;
        case 4:
            yaw_ramp_set_goal(18000,true);
            for(int i = 0 ; i < 64 ; i++){
                SetMoveParameters(-i,0,0,false,true);
            }
            HAL_Delay(2500);
            SetMoveParameters(0,0,0,false,false);
            Speed_Control(-64,0,0,true);
            HAL_Delay(700);
            Speed_Control(0,0,0,true);
            HAL_Delay(500);
            Speed_Control(0,100,0,true);
            HAL_Delay(1200);
            Speed_Control(0,0,0,true);
            yaw_123 = 18000;
            break;
        case 5:
            yaw_ramp_set_goal(18000,true);
            for(int i = 0 ; i < 64 ; i++){
                SetMoveParameters(-i,0,0,false,true);
            }
            HAL_Delay(2500);
            SetMoveParameters(0,0,0,false,false);
            Speed_Control(-64,0,0,true);
            HAL_Delay(700);
            Speed_Control(0,0,0,true);
            HAL_Delay(500);
            Speed_Control(0,100,0,true);
            HAL_Delay(3400);
            Speed_Control(0,0,0,true);
            yaw_123 = 18000;
            break;
        case 6:
            yaw_ramp_set_goal(18000,true);
            for(int i = 0 ; i < 64 ; i++){
                SetMoveParameters(-i,0,0,false,true);
            }
            HAL_Delay(2500);
            SetMoveParameters(0,0,0,false,false);
            Speed_Control(-64,0,0,true);
            HAL_Delay(700);
            Speed_Control(0,0,0,true);
            HAL_Delay(500);
            Speed_Control(0,100,0,true);
            HAL_Delay(6000);
            Speed_Control(0,0,0,true);
            yaw_123 = 18000;
            break;
    }
}
void process_4(void){
    vision_alignment_set_x_params(-0.5f, 0.0f, 0.0f, 325, 80, 3);
    vision_alignment_set_y_params(0.4f, 0.0f, 0.0f, 210, 80, 3);
    while (1) {
        vision_alignment_update();  // 双轴控制
        OLED_NewFrame();
        OLED_PrintString(64, 40, "速度", &font12x12, OLED_COLOR_NORMAL);
        snprintf(buffer, sizeof(buffer), "%d",vision_x_coord);
        OLED_PrintASCIIString(88, 40, buffer, &afont12x6, OLED_COLOR_NORMAL);

        OLED_PrintString(64, 52, "线数", &font12x12, OLED_COLOR_NORMAL);
        snprintf(buffer, sizeof(buffer), "%d",vision_y_coord);
        OLED_PrintASCIIString(100, 52, buffer, &afont12x6, OLED_COLOR_NORMAL);
        OLED_ShowFrame();
        if( (abs(vision_x_coord - 325) <= 5) && (abs(vision_y_coord - 210) <= 5) ){
            break;
        }
    }
    Speed_Control(-30,0,0,true);
    HAL_Delay(1800);
    Speed_Control(0,0,0,true);
    Wind_SetSpeed(1000);
    HAL_Delay(1000);
    Buzzer_PlayTone(TONE_DESCEND);
    HAL_Delay(1000);
    Buzzer_PlayTone(TONE_DESCEND);
    HAL_Delay(1000);
    Buzzer_PlayTone(TONE_DESCEND);
    HAL_Delay(1000);
    Wind_SetSpeed(0);
}

void process_5(void){
    Speed_Control(60,0,0,false);
    HAL_Delay(900);
    Speed_Control(0,0,0,false);
    HAL_Delay(50);
    switch(target_position){
        case 1:
            Speed_Control(0,100,0,true);
            HAL_Delay(1200);
            Speed_Control(0,0,0,true);
            HAL_Delay(500);
            Speed_Control(64,0,0,true);
            HAL_Delay(1200);
            yaw_ramp_set_goal(27000,true);
            SetMoveParameters(64,0,0,false,true);
            HAL_Delay(3200);
            SetMoveParameters(0,0,0,false,false);
            Speed_Control(0,0,0,true);
            break;
        case 2:
            Speed_Control(0,100,0,true);
            HAL_Delay(3400);
            Speed_Control(0,0,0,true);
            HAL_Delay(500);
            Speed_Control(64,0,0,true);
            HAL_Delay(1200);
            yaw_ramp_set_goal(27000,true);
            SetMoveParameters(64,0,0,false,true);
            HAL_Delay(3200);
            SetMoveParameters(0,0,0,false,false);
            Speed_Control(0,0,0,true);
            break;
        case 3:
            Speed_Control(0,100,0,true);
            HAL_Delay(5500);
            Speed_Control(0,0,0,true);
            HAL_Delay(500);
            Speed_Control(64,0,0,true);
            HAL_Delay(1200);
            yaw_ramp_set_goal(27000,true);
            SetMoveParameters(64,0,0,false,true);
            HAL_Delay(3200);
            SetMoveParameters(0,0,0,false,false);
            Speed_Control(0,0,0,true);
            break;
        case 4:
            Speed_Control(0,-100,0,true);
            HAL_Delay(1200);
            Speed_Control(0,0,0,true);
            HAL_Delay(500);
            Speed_Control(-64,0,0,true);
            HAL_Delay(700);
            yaw_ramp_set_goal(27000,true);
            SetMoveParameters(64,0,0,false,true);
            HAL_Delay(2400);
            SetMoveParameters(0,0,0,false,false);
            Speed_Control(0,0,0,true);
            break;
        case 5:
            Speed_Control(0,-100,0,true);
            HAL_Delay(3100);
            Speed_Control(0,0,0,true);
            HAL_Delay(500);
            Speed_Control(-64,0,0,true);
            HAL_Delay(700);
            yaw_ramp_set_goal(27000,true);
            SetMoveParameters(64,0,0,false,true);
            HAL_Delay(2400);
            SetMoveParameters(0,0,0,false,false);
            Speed_Control(0,0,0,true);
            break;
        case 6:
            Speed_Control(0,-100,0,true);
            HAL_Delay(5500);
            Speed_Control(0,0,0,true);
            HAL_Delay(500);
            Speed_Control(-64,0,0,true);
            HAL_Delay(700);
            yaw_ramp_set_goal(27000,true);
            SetMoveParameters(64,0,0,false,true);
            HAL_Delay(2400);
            SetMoveParameters(0,0,0,false,false);
            Speed_Control(0,0,0,true);
            break;
    }
    HAL_Delay(700);
}

void process_6(void) {
    Encoder_Reset();
    Remake_reached_end_flag();
    Start_Line_Follow_Fast();
    //while(motor_sum_odometers_x < 12000){}
    HAL_Delay(10000);
    Remake_reached_end_flag();
    while(1){
        if(Line_Follow_PID_IsFinished()){
            Stop_Line_Follow_Fast();
            break;
        }
    }
}