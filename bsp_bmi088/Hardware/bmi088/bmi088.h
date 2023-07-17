/**
 ****************************(C) COPYRIGHT 2022 ACE****************************
 * @file bmi088.h
 * @brief 
 * 
 * @note
 * @history
 * Version  Date         Author         Modification
    1.0     2023-7-17    郭嘉源         完成陀螺仪收发基本功能
 *
 *@verbatim
 ==============================================================================
 相关CMD宏定义，函数声明，数据结构体
 具体CMD位数含义查看请查看数据手册
 ==============================================================================
 @endverbatim
 ****************************(C) COPYRIGHT 2022 ACE****************************
 */

#ifndef __BMI088_H
#define __BMI088_H

#include "main.h"

//用于处理原生浮点数据
typedef union{
     float real_data;
     uint8_t temp_data[4];
}float_t;

//命令指令
enum CMD{
    NONE=0x00,
    CMD_RESET=0x01,
    CMD_CALIVRATE=0x02,
    CMD_EULER=0x04,
    CMD_TOTALYAW=0x08,
    CMD_SENDQ=0x10,
    CMD_DIVL=0x20,
    CMD_DIVH=0x40,
    CMD_FULL=0xFF
};

#define TOTAL_EULAR_BYTE 12
#define TOTAL_TOTALYAW_BYTE 4
#define TOTAL_Q_BYTE 16

/* 陀螺仪接收数据结构体*/
typedef struct{

    uint8_t bmi088_frame_length;
    uint8_t cmd;
    
    struct{
        float_t pitch;
        float_t yaw;
        float_t roll;
    }Euler_Angle;
    
    float_t total_yaw;
    
    struct{
        float_t q1;
        float_t q2;
        float_t q3;
        float_t q4;
    }Q;
    
}Bmi088_t;

extern Bmi088_t bmi088_data;
extern uint8_t BMI088_Cmd;
void BMI088_Init(uint8_t cmd);
void BMI088_UART_IRQHandler(UART_HandleTypeDef *huart);

#endif
