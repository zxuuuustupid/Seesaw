#ifndef __USART_H__
#define __USART_H__
#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <Motor.h>
#include <Encoder.h>
#include "Controll.h"
#define UART_SEND_BUFF_SIZE 512
#define UART_RECV_BUFF_SIZE 512

typedef struct _UART_TypeDef
{
    uint8_t CommNum;

    uint16_t SendBuffSize;
    uint16_t RecvBuffSize;
    uint8_t SendBuff[UART_SEND_BUFF_SIZE];
    uint8_t RecvBuff[UART_RECV_BUFF_SIZE];

    uint16_t SendPushIndex;
    uint16_t SendPopIndex;
    uint16_t SendBuffAvailablePoints;

    uint16_t RecvPushIndex;
    uint16_t RecvPopIndex;
    uint16_t RecvBuffAvailablePoints;
} UARTTypeDef;
void StartTaskUART1Send(void const *argument);
int UART_PopDataFromSendFIFOBuffer(UARTTypeDef *uart, uint8_t *pdata, uint16_t len);
int UART_PushDataToSendFIFOBuffer(UARTTypeDef *uart, uint8_t *pdata, uint16_t len);
void UART_BuffInit(UARTTypeDef *uart, uint8_t commNum);
uint8_t Get_id_Flag(void);
float Pow_invert(uint8_t X, uint8_t n);
float RxPacket_Data_Handle(void);
#endif
