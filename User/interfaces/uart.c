/*
 * uart.c
 *
 *  Created on: May 19, 2023
 *      Author: Dmitriy Kosiuchik
 */
#include "uart.h"
#include <string.h>


uint8_t txBuffer[UART_BUFFER_SIZE] = {0};
uint8_t rxBuffer[UART_BUFFER_SIZE] = {0};

uint16_t rxBufferPos = 0;

void USART2_IRQHandler() __attribute__((interrupt(/*"WCH-Interrupt-fast"*/)));

void UART_DMA_Init()
{
    DMA_InitTypeDef DMA_InitStructure = {0};
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // Common
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_DeInit(DMA1_Channel7);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)txBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = UART_BUFFER_SIZE;
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel7, ENABLE); /* USART2 Tx */
}

void UART_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef  GPIO_InitStructure = {0};


    /* USART2 TX-->A.2   RX-->A.3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    UART_DMA_Init();

    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_ERR, ENABLE);

    NVIC_EnableIRQ(USART2_IRQn);

    USART_Init(USART2, &USART_InitStructure);

    USART_DMACmd(USART2, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);

    USART_Cmd(USART2, ENABLE);
}

void UART_WriteData(uint8_t* data_ptr, uint16_t len)
{
    while(DMA1_Channel7->CNTR) {}

    memcpy(txBuffer, data_ptr, len);
    DMA_SetCurrDataCounter(DMA1_Channel7, len);
}

void UART_WriteByte(uint8_t data)
{
    USART2->DATAR = data;
}

void USART2_IRQHandler()
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
        rxBuffer[rxBufferPos] = USART2->DATAR;
        if(rxBufferPos < UART_BUFFER_SIZE){
            rxBufferPos++;
        }
        else
        {
            rxBufferPos=0;
            printf("UART buffer overflow\r\n");
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }

    if(USART_GetITStatus(USART2, USART_IT_FE) == SET)
    {
        printf("Framing error!\r\n");
        USART_ClearITPendingBit(USART2, USART_IT_FE);
    }

    if(USART_GetITStatus(USART2, USART_IT_ORE_ER) == SET)
    {
        printf("Buffer overrun!\r\n");
        USART_ClearITPendingBit(USART2, USART_IT_ORE_ER);
    }
}


