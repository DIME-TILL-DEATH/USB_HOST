#include "debug.h"
#include "ch32v30x_usbhs_host.h"

int main(void)
{
    uint8_t ret;

	Delay_Init();
	USART_Printf_Init(115200);
	printf("BLE-USB Host converter\r\n");
	printf("SystemClk:%d\r\n", SystemCoreClock);

	USBHS_HostInit(ENABLE);

	while(1)
	{
        if(USBHSH->INT_FG & USBHS_DETECT_FLAG)
        {
            USBHSH->INT_FG = USBHS_DETECT_FLAG; // Clear event (Maybe function)
            if(USBHSH->MIS_ST & USBHS_ATTCH)
            {
                ret = USBHS_HostEnum();
                if(ret == ERR_SUCCESS)
                {
                    printf("New device. Enum success\n");
                }
                else
                {
                    printf("enum error\n");
                }
            }
            else
            {
                USBHS_HostInit(DISABLE);
                USBHS_HostInit(ENABLE);
                printf("disconnect\n");
            }
        }
	}
}




