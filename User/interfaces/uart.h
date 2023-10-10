#ifndef UART_H
#define UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32v30x.h"

#define UART_BUFFER_SIZE 1024

typedef enum
{
    UART_NUM1,
    UART_NUM2
}UART_Type;

void UART_Init(void);
void UART_WriteData(uint8_t* data_ptr, uint16_t len);
void UART_WriteByte(uint8_t data);

extern uint8_t txBuffer[UART_BUFFER_SIZE];
extern uint8_t rxBuffer[UART_BUFFER_SIZE];
extern uint16_t rxBufferPos;

#ifdef __cplusplus
}
#endif

#endif
