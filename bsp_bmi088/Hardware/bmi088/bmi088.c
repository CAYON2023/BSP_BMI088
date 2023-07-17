/**
 ****************************(C) COPYRIGHT 2022 ACE****************************
 * @file bmi088.c
 * @brief 
 * 
 * @note
 * @history
 * Version  Date         Author         Modification
    1.0     2023-7-17    郭嘉源         完成陀螺仪收发基本功能
 *
 *@verbatim
 ==============================================================================
 使用方法：1、将BMI088_Init(cmd)置于串口初始化后(cmd中为命令指令，相关宏定义存储于bmi088.h中),
              初始化后会根据cmd初始化陀螺仪
           2、将BMI088_UART_IRQHandler(UART_HandleTypeDef *huart)至于相关串口中断函数中
           3、经以上调用后，陀螺仪所有数据便会存储于bmi088_data结构体中，需要使用直接调用即可
           
 注意事项：1、初始化后会有几秒钟的陀螺仪调整时间，切勿在此时开启数据接收，
           当调整结束后，陀螺仪蓝灯会闪烁，此时可以开启数据调用
           2、若需使用多个陀螺仪，请在BMI088_Init中添加BMI088_Usart_Init函数，
           同时添加BMI088_SendData函数内的发送函数，发送相应命令到相应串口，
           并添加相应数量接收缓存区与数据结构体
 ==============================================================================
 @endverbatim
 ****************************(C) COPYRIGHT 2022 ACE****************************
 */
#include "bmi088.h"
#include "bmi088reg.h"
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"


#define BMI088_RX_Buffer_Num 72
#define BMI088_FRAME_LENGTH_DEFALUT 36

// 接收数据缓存,缓冲区有36个字节，为防止DMA传输越界给72
uint8_t RX_Buffer[BMI088_RX_Buffer_Num];

// 陀螺仪数据结构体变量
Bmi088_t bmi088_data;
uint8_t BMI088_Cmd=0x00;

//函数声明
static void BMI088_SendData(uint8_t cmd);
static void BMI088_DataProcess(volatile const uint8_t *pData, Bmi088_t *bmi088_rev);

void BMI088_Usart_Init(UART_HandleTypeDef *huart, uint8_t *Rx_Buff, uint16_t Data_Buff_Lenth)
{
    // 使能DMA串口接收
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    // 使能空闲中断
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    // 失效DMA
    __HAL_DMA_DISABLE(huart->hdmarx);
    while (huart->hdmarx->Instance->CCR & DMA_CCR_EN)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);
    }
    huart->hdmarx->Instance->CPAR = (uint32_t) & (huart->Instance->DR);
    // 内存缓冲区
    huart->hdmarx->Instance->CMAR = (uint32_t)(Rx_Buff);
    // 数据长度
    huart->hdmarx->Instance->CNDTR = Data_Buff_Lenth;
    // 使能双缓冲区
    SET_BIT(huart->hdmarx->Instance->CCR, DMA_CCR_EN);
    // 使能DMA
    __HAL_DMA_ENABLE(huart->hdmarx);
}

void BMI088_Init(uint8_t cmd) 
{
  
    BMI088_Cmd|=cmd;
    bmi088_data.cmd=BMI088_Cmd;//获取命令
    bmi088_data.bmi088_frame_length=BMI088_FRAME_LENGTH_DEFALUT;//默认接收数据长度(所有数据)
    /*通过命令位获得所需接收的数据长度*/
    if((bmi088_data.cmd&CMD_EULER)==CMD_EULER)
    {
        bmi088_data.bmi088_frame_length=bmi088_data.bmi088_frame_length;
    }
    else
    {
        bmi088_data.bmi088_frame_length=bmi088_data.bmi088_frame_length-TOTAL_EULAR_BYTE;
    }
    if((bmi088_data.cmd&CMD_TOTALYAW)==CMD_TOTALYAW)
    {
        bmi088_data.bmi088_frame_length=bmi088_data.bmi088_frame_length;
    }
    else
    {
        bmi088_data.bmi088_frame_length=bmi088_data.bmi088_frame_length-TOTAL_TOTALYAW_BYTE;
    }
    if((bmi088_data.cmd&CMD_SENDQ)==CMD_SENDQ)
    {
        bmi088_data.bmi088_frame_length=bmi088_data.bmi088_frame_length;
    }
    else
    {
        bmi088_data.bmi088_frame_length=bmi088_data.bmi088_frame_length-TOTAL_Q_BYTE;
    }
    BMI088_Usart_Init(&huart1, RX_Buffer,BMI088_RX_Buffer_Num);
    BMI088_SendData(BMI088_Cmd);
} 

static uint16_t this_time_rx_len = 0;
void BMI088_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if (huart->Instance->SR &
        UART_FLAG_RXNE) // 接收到数据
                        // //SR寄存器是状态寄存器，若其与UART_FLAG_RXNE（00010100）与运算有1，则说明有接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(huart);
    }
    else if (huart->Instance->SR & UART_FLAG_IDLE) // 串口处于空闲状态  （UART_FLAG_IDLE =
                                                   // 0：未检测到空闲线路 1：检测到空闲线路）
    {                                              // 在空闲中断里判断数据帧的传送是否正确
        // 当串口开始接收数据后，检测到1字节数据的时间内没有数据发送，则认为串口空闲了。

        __HAL_UART_CLEAR_PEFLAG(huart);

        // disable DMA
        // 失效DMA
        __HAL_DMA_DISABLE(huart->hdmarx);

        // get receive data length, length = set_data_length - remain_length
        // 获取接收数据长度,长度 = 设定长度 - 剩余长度
        this_time_rx_len = BMI088_RX_Buffer_Num - huart->hdmarx->Instance->CNDTR;
        // reset set_data_lenght
        // 重新设定数据长度
        huart->hdmarx->Instance->CNDTR = BMI088_RX_Buffer_Num;

        // enable DMA
        // 使能DMA
        __HAL_DMA_ENABLE(huart->hdmarx);

        if (this_time_rx_len == bmi088_data.bmi088_frame_length)
        {
            //对数据进行处理
            BMI088_DataProcess(RX_Buffer, &bmi088_data);
        }
    }
}

//数据处理函数
void BMI088_DataProcess(volatile const uint8_t *pData, Bmi088_t *bmi088_rev)
{
    uint8_t temp_num=0;
 
    for(uint8_t i=bmi088_rev->bmi088_frame_length;i!=0;i--)
    {
        //检验帧尾
        if(pData[i-3]|pData[i-2]|pData[i-1]|pData[i]==0xFF)
        {
            /*根据cmd确定接收内容*/
            if((bmi088_rev->cmd&CMD_EULER)==CMD_EULER)
            {
               bmi088_rev->Euler_Angle.pitch.temp_data[0]=pData[temp_num++];
               bmi088_rev->Euler_Angle.pitch.temp_data[1]=pData[temp_num++];
               bmi088_rev->Euler_Angle.pitch.temp_data[2]=pData[temp_num++];
               bmi088_rev->Euler_Angle.pitch.temp_data[3]=pData[temp_num++];
                
               bmi088_rev->Euler_Angle.yaw.temp_data[0]=pData[temp_num++];
               bmi088_rev->Euler_Angle.yaw.temp_data[1]=pData[temp_num++];
               bmi088_rev->Euler_Angle.yaw.temp_data[2]=pData[temp_num++];
               bmi088_rev->Euler_Angle.yaw.temp_data[3]=pData[temp_num++];
                
               bmi088_rev->Euler_Angle.roll.temp_data[0]=pData[temp_num++];
               bmi088_rev->Euler_Angle.roll.temp_data[1]=pData[temp_num++];
               bmi088_rev->Euler_Angle.roll.temp_data[2]=pData[temp_num++];
               bmi088_rev->Euler_Angle.roll.temp_data[3]=pData[temp_num++];
            }
            if((bmi088_rev->cmd&CMD_TOTALYAW)==CMD_TOTALYAW)
            {
                bmi088_rev->total_yaw.temp_data[0]=pData[temp_num++];
                bmi088_rev->total_yaw.temp_data[1]=pData[temp_num++];
                bmi088_rev->total_yaw.temp_data[2]=pData[temp_num++];
                bmi088_rev->total_yaw.temp_data[3]=pData[temp_num++];
            }
            if((bmi088_rev->cmd&CMD_SENDQ)==CMD_SENDQ)
            {
                bmi088_rev->Q.q1.temp_data[0]=pData[temp_num++];
                bmi088_rev->Q.q1.temp_data[1]=pData[temp_num++];
                bmi088_rev->Q.q1.temp_data[2]=pData[temp_num++];
                bmi088_rev->Q.q1.temp_data[3]=pData[temp_num++];

                bmi088_rev->Q.q2.temp_data[0]=pData[temp_num++];
                bmi088_rev->Q.q2.temp_data[1]=pData[temp_num++];
                bmi088_rev->Q.q2.temp_data[2]=pData[temp_num++];
                bmi088_rev->Q.q2.temp_data[3]=pData[temp_num++];
                
                bmi088_rev->Q.q3.temp_data[0]=pData[temp_num++];
                bmi088_rev->Q.q3.temp_data[1]=pData[temp_num++];
                bmi088_rev->Q.q3.temp_data[2]=pData[temp_num++];
                bmi088_rev->Q.q3.temp_data[3]=pData[temp_num++];
                
                bmi088_rev->Q.q4.temp_data[0]=pData[temp_num++];
                bmi088_rev->Q.q4.temp_data[1]=pData[temp_num++];
                bmi088_rev->Q.q4.temp_data[2]=pData[temp_num++];
                bmi088_rev->Q.q4.temp_data[3]=pData[temp_num++];
            }
            temp_num=0;
        }
    }
}

void BMI088_SendData(uint8_t cmd)
{
    uint8_t data[3]={0xFF,cmd,0xEE};
    HAL_UART_Transmit(&huart1,data,3,1);
}
