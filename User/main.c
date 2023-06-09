#include "debug.h"

#include "usb_driver.h"
#include "usb_device_classes.h"

void USBHS_IRQHandler()  __attribute__((interrupt("WCH-Interrupt-fast")));

#define PANGAEA_CDC_INTERFACE_NUM 1

// free memory in function if using pointer to devices
USBDEV_INFO lastConnectedDevice;

typedef enum
{
    DISCONNECTED,
    VCOM_PORT,
    PACNGAEA_CP16,
    FLASH_DRIVE,
    OTHER
}CONNECTED_TYPE;
CONNECTED_TYPE connectedType = DISCONNECTED;

void checkConnectedDevice()
{
    if(lastConnectedDevice.devClass == USB_CLASS_MSD) connectedType = FLASH_DRIVE;

    if(lastConnectedDevice.devClass == USB_CLASS_CDC)
    {
        connectedType = VCOM_PORT;
        if(lastConnectedDevice.VID == 0x483 && lastConnectedDevice.PID == 0x5740)
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

	while(1)
	{
	    if(connectedType == PACNGAEA_CP16)
	    {
	        uint8_t retVal;

            uint8_t buf[] = "amtdev\r\n";
            retVal = USB_SendEndpData(lastConnectedDevice.itfInfo[PANGAEA_CDC_INTERFACE_NUM].endpInfo[0].endpAddress,
                                      &lastConnectedDevice.itfInfo[PANGAEA_CDC_INTERFACE_NUM].endpInfo[0].toggle,
                                      buf, strlen(buf));

            printf("Transfer res: %X", retVal);

            uint8_t readBuf[256]={0};
            uint16_t readLen;
            retVal = USB_GetEndpData(lastConnectedDevice.itfInfo[PANGAEA_CDC_INTERFACE_NUM].endpInfo[1].endpAddress,
                                     &lastConnectedDevice.itfInfo[PANGAEA_CDC_INTERFACE_NUM].endpInfo[1].toggle,
                                     readBuf, &readLen);

            printf(" recieve res: %X\r\n", retVal);

            if(readLen > 0)
            {
                printf("%s", readBuf);
            }
	    }
	    Delay_Ms(2000);
	}
}

void USBHS_IRQHandler()
{
    USBHSH->INT_FG = USBHS_DETECT_FLAG;
    if(USBHSH->MIS_ST & USBHS_ATTCH)
    {
        printf("\r\nNew device connected.\n");
        uint8_t retVal = USB_HostEnum(&lastConnectedDevice);
        if(retVal == ERR_SUCCESS)
        {
            printf("Enum success\n");
            checkConnectedDevice();
        }
        else
        {
            printf("Enum error\n");
        }
    }
    else
    {
        USB_HostInit(DISABLE);
        USB_HostInit(ENABLE);

        connectedType = DISCONNECTED;
        USB_FreeDevStruct(&lastConnectedDevice);

        printf("Disconnect\n");
    }
}
