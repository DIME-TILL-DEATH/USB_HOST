#include "debug.h"
#include "ch32v30x_usbhs_host.h"

#include "usb_device_classes.h"

void USBHS_IRQHandler()  __attribute__((interrupt("WCH-Interrupt-fast")));

//USBDEV_INFO actualDevice;

typedef enum
{
    DISCONNECTED,
    VCOM_PORT,
    PACNGAEA_CP16,
    FLASH_DRIVE,
    OTHER
}CONNECTED_TYPE;
CONNECTED_TYPE connectedType;

void checkConnectedDevice()
{
    if(thisUsbDev.DeviceClass == USB_CLASS_MSD) connectedType = FLASH_DRIVE;

    if(thisUsbDev.DeviceClass == USB_CLASS_CDC)
    {
        connectedType = VCOM_PORT;
        if(thisUsbDev.VID == 0x483 && thisUsbDev.PID == 0x5740)
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
	    if(connectedType == VCOM_PORT)
	    {

	    }

	    Delay_Ms(5);
	}
}

void USBHS_IRQHandler()
{
    USBHSH->INT_FG = USBHS_DETECT_FLAG;
    if(USBHSH->MIS_ST & USBHS_ATTCH)
    {
        printf("\r\nNew device connected.\n");
        uint8_t retVal = USB_HostEnum();
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
        printf("Disconnect\n");
    }
}
