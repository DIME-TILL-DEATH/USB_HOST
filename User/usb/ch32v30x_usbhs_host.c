#include "ch32v30x_usbhs_host.h"

/******************************** IN-OUT BUFFERS ********************************/
__attribute__ ((aligned(4))) uint8_t   endpTXbuf[MAX_PACKET_SIZE];  // OUT, must even address
__attribute__ ((aligned(4))) uint8_t   endpRXbuf[MAX_PACKET_SIZE];  // OUT, must even address

/*********************************************************************
 * @fn      USB20_RCC_Init
 *
 * @brief   USB RCC initialized
 *
 * @return  none
 */
void USB_RCC_Init()
{
    RCC->CFGR2 = USBHS_PLL_SRC_HSE | USBHS_PLL_SRC_PRE_DIV2 | USBHS_PLL_CKREF_4M;//PLL REF = HSE/2 = 4MHz
    RCC->CFGR2 |= USBHS_CLK_SRC_PHY|USBHS_PLLALIVE;
    RCC->AHBPCENR |= RCC_AHBPeriph_USBHS;                  //USB clock enable
    Delay_Us(200);
    USBHSH->HOST_CTRL |= PHY_SUSPENDM;
    Delay_Us(5);
}

/*********************************************************************
 * @fn      USBHS_HostInit
 *
 * @brief   USB host mode initialized.
 *
 * @param   state - ENABLE or DISABLE
 *
 * @return  none
 */
void USB_HostInit(FunctionalState state)
{
    if(state==ENABLE)
    {
        USB_RCC_Init();

        USBHSH->CONTROL = HOST_MODE | FULL_SPEED | INT_BUSY_EN | DMA_EN;
        USBHSH->HOST_CTRL = PHY_SUSPENDM | SEND_SOF_EN;
        USBHSH->HOST_EP_CONFIG = HOST_TX_EN | HOST_RX_EN ;
        USBHSH->HOST_RX_MAX_LEN = 512;
        USBHSH->HOST_RX_DMA = (uint32_t)endpRXbuf;
        USBHSH->HOST_TX_DMA = (uint32_t)endpTXbuf;
    }
    else
    {
        USBHSH->CONTROL = USB_FORCE_RST | USB_ALL_CLR;
    }
}

/*********************************************************************
 * @fn      SetBusReset
 *
 * @brief   Reset USB bus
 *
 * @return  none
 */
void  USB_SetBusReset()
{
    USBHSH->HOST_CTRL |= SEND_BUS_RESET;                              //bus reset
    Delay_Ms(15);
    USBHSH->HOST_CTRL &= ~SEND_BUS_RESET;
    while(!(USBHSH->HOST_CTRL & UH_SOFT_FREE));                     //wait bus idle;
    USBHSH->HOST_CTRL |= SEND_SOF_EN;                                 //sof enable
}

/*********************************************************************
 * @fn      USB_RawTransaction
 *
 * @brief   General host transaction
 *
 * @param   endpPid - PID of the transaction  and the number of endpoint
 *          endpToggle - sync  trigger bit
 *          timeout - value of timeout
 *
 * @return  Error state
 */
uint8_t USB_RawTransaction(uint8_t endpPid, uint8_t endpToggle, uint32_t timeout)
{
    USBHSH->HOST_TX_CTRL = USBHSH->HOST_RX_CTRL = endpToggle;

    uint8_t transRetry = 0;
    do
    {
        USBHSH->HOST_EP_PID = endpPid;
        USBHSH->INT_FG = USBHS_ACT_FLAG;

        for (uint32_t i = WAIT_USB_TOUT_200US; i != 0 && ((USBHSH->INT_FG) & USBHS_ACT_FLAG) == 0; i--)
        {
            Delay_Us(1);
        }

        USBHSH->HOST_EP_PID = 0x00;

        if((USBHSH->INT_FG & USBHS_ACT_FLAG) == 0) return ERR_USB_UNKNOWN;

        if(USBHSH->INT_FG & USBHS_DETECT_FLAG)
        {
            USBHSH->INT_FG = USBHS_DETECT_FLAG;
            Delay_Us(200);
            if(USBHSH->MIS_ST & USBHS_ATTCH)
            {
                if(USBHSH->HOST_CTRL & SEND_SOF_EN)
                {
                    return ERR_USB_CONNECT;
                }
            }
            else return ERR_USB_DISCON;
        }
        else if(USBHSH->INT_FG & USBHS_ACT_FLAG)
        {
            uint8_t responsePID = USBHSH->INT_ST & USBHS_HOST_RES;

            if (responsePID == USB_PID_STALL) return responsePID|ERR_USB_TRANSFER;

            if((endpPid >> 4) == USB_PID_IN)
            {
                if (USBHSH->INT_ST & USBHS_TOGGLE_OK) return ERR_SUCCESS;
            }
            else
            {
                if ((responsePID == USB_PID_ACK) || (responsePID == USB_PID_NYET)) return ERR_SUCCESS;
            }

            if (responsePID == USB_PID_NAK)
            {
                if (timeout == 0) return responsePID|ERR_USB_TRANSFER;
                if (timeout < 0xFFFF) timeout--;
                --transRetry;
            }
            else switch (endpPid >> 4)
            {
                case USB_PID_SETUP:
                    break;
                case USB_PID_OUT:
                    if (responsePID) return responsePID|ERR_USB_TRANSFER;
                    break;
                case USB_PID_IN:                                        //2b
                    if ((responsePID == USB_PID_DATA0) || (responsePID == USB_PID_DATA1))
                    {
                    }
                    else if (responsePID)
                    {
                        return responsePID | ERR_USB_TRANSFER;
                    }
                    break;
                default:
                {
                    return ERR_USB_UNKNOWN;
                }
            }
        }
        else
        {
            USBHSH->INT_FG = 0x3F;
        }
        Delay_Us(15);
    } while (++transRetry < 3);

    return ERR_USB_TRANSFER;
}

/*********************************************************************
 * @fn      USB_HostCtrlTransfer
 *
 * @brief   Host control transfer.
 *
 * @param   usbDevice_ptr - target USB device pointer
 *          request_ptr - USB setup request pointer
 *          replyBuf_ptr - pointer to answer
 *
 * @return  Error state
 */
uint8_t USB_HostCtrlTransfer(USBDEV_INFO* usbDevice_ptr, USB_SETUP_REQ* request_ptr, uint8_t** replyBuf_ptr)
{
    uint8_t   retVal;
    uint8_t   tog = 1;

    uint32_t len_ptr = 0;

    *replyBuf_ptr = 0;

    memcpy(endpTXbuf, request_ptr, sizeof(USB_SETUP_REQ));
    Delay_Us(100);

    USBHSH->DEV_AD = usbDevice_ptr->devAddress;

    // setup stage
    USBHSH->HOST_TX_LEN = sizeof(USB_SETUP_REQ);
    retVal = USB_RawTransaction((USB_PID_SETUP<<4)|DEF_ENDP_0, 0, 200000);
    if(retVal != ERR_SUCCESS) return retVal;

    // data stage
    uint16_t requestLen = request_ptr->wLength;
    if(requestLen && endpRXbuf)
    {
       if((request_ptr->bRequestType) & USB_REQ_TYP_IN)            // device to host
       {
           while(requestLen)
           {
               USBHSH->HOST_RX_DMA = (uint32_t)endpRXbuf + len_ptr;
               retVal = USB_RawTransaction((USB_PID_IN<<4)| DEF_ENDP_0, tog<<3, 20000);
               if(retVal != ERR_SUCCESS) return retVal;

               tog ^=1;

               uint32_t rxlen = (USBHSH->RX_LEN < requestLen) ? USBHSH->RX_LEN : requestLen;
               requestLen -= rxlen;
               len_ptr += rxlen;

               if((USBHSH->RX_LEN == 0) || (USBHSH->RX_LEN & (usbDevice_ptr->endp0Size - 1)))  break;
            }
            USBHSH->HOST_TX_LEN = 0 ;
         }
       else
       {                                                           // host to device
          while(requestLen)
          {
               USBHSH->HOST_TX_DMA = (uint32_t)endpTXbuf + len_ptr;
               USBHSH->HOST_TX_LEN = (requestLen >= usbDevice_ptr->endp0Size) ? usbDevice_ptr->endp0Size : requestLen;

               retVal = USB_RawTransaction((USB_PID_OUT<<4)|DEF_ENDP_0,  tog<<3,  20000);
               if(retVal != ERR_SUCCESS) return retVal;
               tog ^=1;
               requestLen -= USBHSH->HOST_TX_LEN;
               len_ptr += USBHSH->HOST_TX_LEN;
          }
        }
    }

    // status stage
    retVal = USB_RawTransaction(((USBHSH->HOST_TX_LEN) ? USB_PID_IN<<4|DEF_ENDP_0 : USB_PID_OUT<<4|DEF_ENDP_0),
            UH_R_TOG_1|UH_T_TOG_1, 20000 );

    if(retVal != ERR_SUCCESS) return retVal;

    if(len_ptr < request_ptr->wLength) return ERR_USB_BUF_OVER;

    if(replyBuf_ptr) *replyBuf_ptr = endpRXbuf;

    if (USBHSH->HOST_TX_LEN == 0) return ERR_SUCCESS;    //status stage is out, send a zero-length packet.
    if (USBHSH->RX_LEN == 0) return ERR_SUCCESS;    //status stage is in, a zero-length packet is returned indicating success.

    return ERR_USB_BUF_OVER;
}



/*********************************************************************
 * @fn      USB_GetEndpData
 *
 * @brief   Get data from USB device input endpoint.
 *
 * @param    endpInfo_ptr: Endpoint pointer
 *          buf_ptr: Data Buffer
 *          len_ptr: Data length
 *
 * @return  The result of getting data.
 */
uint8_t USB_GetEndpData(USBENDP_INFO* endpInfo_ptr, uint8_t *buf_ptr, uint16_t *len_ptr)
{
    if(!endpInfo_ptr || !buf_ptr) return 0xFF;

    if(len_ptr) *len_ptr = 0;

    memset(endpRXbuf, '\0', MAX_PACKET_SIZE);

    USBHSH->HOST_RX_DMA = (uint32_t)endpRXbuf;
    uint8_t retVal = USB_RawTransaction((USB_PID_IN << 4) | endpInfo_ptr->endpAddress, endpInfo_ptr->toggle,  2000);
    if(retVal == ERR_SUCCESS)
    {
        endpInfo_ptr->toggle ^= UH_R_TOG_1;
        *len_ptr = USBHSH->RX_LEN;
        memcpy(buf_ptr, endpRXbuf, *len_ptr);
      //  printf("USB answer: %s\r\n", endpRXbuf);
    }
    return retVal;
}

/*********************************************************************
 * @fn      USBHSH_SendEndpData
 *
 * @brief   Send data to the USB device output endpoint.
 *
 * @para    endpInfo_ptr: Endpoint pointer
 *          pbuf: Data Buffer
 *          plen: Data length
 *
 * @return  The result of sending data.
 */
uint8_t USB_SendEndpData(USBENDP_INFO* endpInfo_ptr, uint8_t *buf_ptr, int16_t len)
{
    if(!endpInfo_ptr || !buf_ptr) return 0xFF;

    uint16_t packetSize = (endpInfo_ptr->endpMaxSize <= MAX_PACKET_SIZE) ? endpInfo_ptr->endpMaxSize : MAX_PACKET_SIZE;
    uint32_t len_ptr = 0;
    uint8_t retVal = 0xFF;

    memcpy(endpTXbuf, buf_ptr, len); // no more than MAX_PACKET_SIZE!!!!!!!

    while(len>0)
    {
         USBHSH->HOST_TX_DMA = (uint32_t)endpTXbuf + len_ptr;
         USBHSH->HOST_TX_LEN = (len >= endpInfo_ptr->endpMaxSize) ? packetSize : len;

         retVal = USB_RawTransaction(( USB_PID_OUT << 4 ) | endpInfo_ptr->endpAddress, endpInfo_ptr->toggle,  2000);
         if(retVal != ERR_SUCCESS)
         {
             return retVal;
         }

         endpInfo_ptr->toggle ^= UH_T_TOG_1;
         len -= USBHSH->HOST_TX_LEN;
         len_ptr += USBHSH->HOST_TX_LEN;
    }

    return retVal;
}
