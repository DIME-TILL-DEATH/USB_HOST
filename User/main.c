#include "debug.h"
#include "ch32v30x_usbhs_host.h"

void USBHS_IRQHandler()  __attribute__((interrupt("WCH-Interrupt-fast")));

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
//        if(USBHSH->INT_FG & USBHS_DETECT_FLAG)
//        {
//            USBHSH->INT_FG = USBHS_DETECT_FLAG; // Clear event (Maybe function)
//            if(USBHSH->MIS_ST & USBHS_ATTCH)
//            {
//                printf("\r\nNew device connected.\n");
//                ret = USB_HostEnum();
//                if(ret == ERR_SUCCESS)
//                {
//                    printf("Enum success\n");
//                }
//                else
//                {
//                    printf("Enum error\n");
//                }
//            }
//            else
//            {
//                USB_HostInit(DISABLE);
//                USB_HostInit(ENABLE);
//                printf("Disconnect\n");
//            }
//        }
	}
}

void USBHS_IRQHandler()
{
//    printf("IRQ USB");
//    USBHSH->INT_FG = USBHS_DETECT_FLAG; // Clear event (Maybe function)

    USBHSH->INT_FG = USBHS_DETECT_FLAG; // Clear event (Maybe function)
    if(USBHSH->MIS_ST & USBHS_ATTCH)
    {
        printf("\r\nNew device connected.\n");
        uint8_t retVal = USB_HostEnum();
        if(retVal == ERR_SUCCESS)
        {
            printf("Enum success\n");
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
        printf("Disconnect\n");
    }
}


