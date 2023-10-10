#include <malloc.h>

#include "debug.h"

#include "usb_driver.h"
#include "usb_device_classes.h"

#include "uart.h"

void USBHS_IRQHandler()  __attribute__((interrupt("WCH-Interrupt-fast")));

#define PANGAEA_CDC_INTERFACE_NUM 1

typedef enum
{
    DISCONNECTED,
    VCOM_PORT,
    PACNGAEA_CP16,
    FLASH_DRIVE,
    OTHER
}CONNECTED_TYPE;
CONNECTED_TYPE connectedType = DISCONNECTED;

USBDEV_INFO* lastConnectedDevice_ptr;

void checkConnectedDevice()
{
    if(lastConnectedDevice_ptr->devClass == USB_CLASS_MSD) connectedType = FLASH_DRIVE;

    if(lastConnectedDevice_ptr->devClass == USB_CLASS_CDC)
    {
        connectedType = VCOM_PORT;
        if(lastConnectedDevice_ptr->VID == 0x483 && lastConnectedDevice_ptr->PID == 0x5740)
        {
            connectedType = PACNGAEA_CP16;
            printf("Pangaea device found!\r\n");
        }
    }
}

int main(void)
{
	Delay_Init();
	USART_Printf_Init(115200);
	printf("BLE-USB Host converter\r\n");
	printf("SystemClk:%d\r\n", SystemCoreClock);

	USB_HostInit(ENABLE);
	USBHSH->INT_EN |= USBHS_DETECT_EN;
	NVIC_EnableIRQ(USBHS_IRQn);

	UART_Init();

	while(1)
	{
//        uint32_t recievedBytes = UART_BUFFER_SIZE - DMA1_Channel6->CNTR;
//	    uint16_t recievedBytes = rxBufferPos;

	    if(connectedType == PACNGAEA_CP16)
	    {
	        uint8_t retVal=0;

            if(rxBufferPos > 0)
            {
                uint8_t bleTxBuf[UART_BUFFER_SIZE] = {0};

                NVIC_DisableIRQ(USART2_IRQn);
                uint16_t recievedBytes = rxBufferPos;
                memcpy(bleTxBuf, rxBuffer, recievedBytes);
                rxBufferPos = 0;
                NVIC_EnableIRQ(USART2_IRQn);

                if(lastConnectedDevice_ptr)retVal = USB_SendEndpData(&(lastConnectedDevice_ptr->itfInfo[PANGAEA_CDC_INTERFACE_NUM].endpInfo[0]),
                                                      bleTxBuf, recievedBytes);

                printf("BLE text: %s \n", bleTxBuf);
            }
            Delay_Ms(25);

            uint8_t bleRxBuf[MAX_PACKET_SIZE] = {0};
            uint16_t readLen = 0;
            uint16_t iter = 0;
            do
            {
                if(lastConnectedDevice_ptr) retVal = USB_GetEndpData(&(lastConnectedDevice_ptr->itfInfo[PANGAEA_CDC_INTERFACE_NUM].endpInfo[1]),
                                         bleRxBuf, &readLen);
                if(retVal == ERR_SUCCESS)
                {
                    UART_WriteData(bleRxBuf, readLen);
//                    printf("USB answer: %s\r\n", bleRxBuf);
                }
                iter++;
                Delay_Ms(1);
            }
            while(retVal == ERR_SUCCESS);
	    }

	    DMA_Cmd(DMA1_Channel6, DISABLE);
	    DMA_SetCurrDataCounter(DMA1_Channel6, UART_BUFFER_SIZE);
	    DMA_Cmd(DMA1_Channel6, ENABLE);
	    Delay_Ms(25);
	}
}

void USBHS_IRQHandler()
{
    USBHSH->INT_FG = USBHS_DETECT_FLAG;
    if(USBHSH->MIS_ST & USBHS_ATTCH)
    {
        printf("\r\nNew device connected.\n");

        USB_FreeDevStruct(lastConnectedDevice_ptr); // clear old ptr
        lastConnectedDevice_ptr = (USBDEV_INFO*)malloc(sizeof(USBDEV_INFO));
        uint8_t retVal = USB_HostEnum(lastConnectedDevice_ptr);

        if(retVal == ERR_SUCCESS)
        {
            printf("Enum success\n");

            USB_PrintDevInfo(lastConnectedDevice_ptr);
            checkConnectedDevice();
        }
        else
        {
            printf("Enum error\n");
            connectedType = DISCONNECTED;
            USB_FreeDevStruct(lastConnectedDevice_ptr);
        }
    }
    else
    {
        USB_HostInit(DISABLE);
        USB_HostInit(ENABLE);

        connectedType = DISCONNECTED;
        USB_FreeDevStruct(lastConnectedDevice_ptr);

        printf("Disconnect\n");
    }
}
