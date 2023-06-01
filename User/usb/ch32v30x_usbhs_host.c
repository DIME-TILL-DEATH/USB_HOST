#include "ch32v30x_usbhs_host.h"

USBDEV_INFO thisUsbDev;

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
void USB_RCC_Init( void )
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
 * @param   sta - ENABLE or DISABLE
 *
 * @return  none
 */
void USB_HostInit (FunctionalState sta)
{
    if(sta==ENABLE)
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
void  USB_SetBusReset(void)
{
    USBHSH->HOST_CTRL |= SEND_BUS_RESET;                              //bus reset
    Delay_Ms(15);
    USBHSH->HOST_CTRL &= ~SEND_BUS_RESET;
    while(!(USBHSH->HOST_CTRL & UH_SOFT_FREE));                     //wait bus idle;
    USBHSH->HOST_CTRL |= SEND_SOF_EN;                                 //sof enable
}

/*********************************************************************
 * @fn      USBHostTransact
 *
 * @brief   General host transaction
 *
 * @param   endp_pid - PID of the transaction  and the number of endpoint
 *          toggle - sync  trigger bit
 *          timeout - value of timeout
 *
 * @return  Error state
 */
uint8_t USB_RawTransaction(uint8_t endp_pid, uint8_t toggle, uint32_t timeout)
{
    USBHSH->HOST_TX_CTRL = USBHSH->HOST_RX_CTRL = toggle;

    uint8_t transRetry = 0;
    do
    {
        USBHSH->HOST_EP_PID = endp_pid;
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
                if(USBHSH->HOST_CTRL & SEND_SOF_EN) return ERR_USB_CONNECT;
            }
            else return ERR_USB_DISCON;
        }
        else if(USBHSH->INT_FG & USBHS_ACT_FLAG)
        {
            uint8_t responsePID = USBHSH->INT_ST & USBHS_HOST_RES;
            if((endp_pid >> 4) == USB_PID_IN)
            {
                if (USBHSH->INT_ST & USBHS_TOGGLE_OK) return ERR_SUCCESS;
            }
            else
            {
                if ((responsePID == USB_PID_ACK) || (responsePID == USB_PID_NYET)) return ERR_SUCCESS;
            }

            if (responsePID == USB_PID_STALL) return responsePID|ERR_USB_TRANSFER;

            if (responsePID == USB_PID_NAK)
            {
                if (timeout == 0) return responsePID|ERR_USB_TRANSFER;
                if (timeout < 0xFFFF) timeout--;
                --transRetry;
            }
            else switch (endp_pid >> 4)
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
                    else if (responsePID) return responsePID | ERR_USB_TRANSFER;
                    break;
                default:
                    return ERR_USB_UNKNOWN;
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
 * @fn      HostCtrlTransfer
 *
 * @brief   Host control transfer.
 *
 * @param   request - USB setup request
 *
 * @return  Error state
 */
uint8_t HostCtrlTransfer(USB_SETUP_REQ* request)
{
    uint8_t   retVal;
    uint8_t   tog = 1;

    uint32_t len_ptr = 0;

    memcpy(endpTXbuf, request, sizeof(USB_SETUP_REQ));
    Delay_Us(100);

    // setup stage
    USBHSH->HOST_TX_LEN = 8;
    retVal = USB_RawTransaction((USB_PID_SETUP<<4)|DEF_ENDP_0, 0, 200000);
    if(retVal != ERR_SUCCESS) return retVal;

    // data stage
    uint16_t requestLen = request->wLength;
    if(requestLen && endpRXbuf)
    {
       if((request->bRequestType) & USB_REQ_TYP_IN)            // device to host
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
               if((USBHSH->RX_LEN == 0) || (USBHSH->RX_LEN & (thisUsbDev.DeviceEndp0Size - 1)))  break;
            }
            USBHSH->HOST_TX_LEN = 0 ;
         }
       else
       {                                                           // host to device
          while(requestLen)
          {
               USBHSH->HOST_TX_DMA = (uint32_t)endpTXbuf + len_ptr;
               USBHSH->HOST_TX_LEN = (requestLen >= thisUsbDev.DeviceEndp0Size) ? thisUsbDev.DeviceEndp0Size : requestLen;

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

    if(len_ptr < request->wLength) return ERR_USB_BUF_OVER;

    if (USBHSH->HOST_TX_LEN == 0) return ERR_SUCCESS;    //status stage is out, send a zero-length packet.

    if (USBHSH->RX_LEN == 0) return ERR_SUCCESS;    //status stage is in, a zero-length packet is returned indicating success.

    return ERR_USB_BUF_OVER;
}

/*********************************************************************
 * @fn      CtrlGetDevDescr
 *
 * @brief   Get device descriptor
 *
 * @return  Error state
 */
uint8_t USB_CtrlGetDevDescr(USB_DEV_DESCR* devDescriptor)
{
    USB_SETUP_REQ request;

    request.bRequestType = USB_REQ_TYP_IN;
    request.bRequest = USB_GET_DESCRIPTOR;
    request.wValue = USB_DESCR_TYP_DEVICE<<8 | 0x0000;
    request.wIndex = 0;
    request.wLength = USB_DESCR_SIZE_DEVICE;

    uint8_t retVal = HostCtrlTransfer(&request);
    if(retVal != ERR_SUCCESS) return retVal;

    thisUsbDev.DeviceEndp0Size = ((USB_DEV_DESCR*)endpRXbuf)->bMaxPacketSize0;

    memcpy(devDescriptor, (USB_DEV_DESCR*)endpRXbuf, sizeof(USB_DEV_DESCR));

    return retVal;
}

/*********************************************************************
 * @fn      CtrlGetConfigDescr
 *
 * @brief   Get configuration descriptor.
 *
 * @return  Error state
 */
uint8_t USB_CtrlGetConfigDescr()
{
    uint8_t retVal;

    USB_SETUP_REQ request;

    request.bRequestType = USB_REQ_TYP_IN;
    request.bRequest = USB_GET_DESCRIPTOR;
    request.wValue = USB_DESCR_TYP_CONFIG<<8 | 0x0000;
    request.wIndex = 0;
    request.wLength = USB_DESCR_SIZE_CONFIG;

    retVal = HostCtrlTransfer(&request);
    if(retVal != ERR_SUCCESS)  return retVal;

    request.wLength = ((USB_CFG_DESCR*)endpRXbuf)->wTotalLength;
    retVal = HostCtrlTransfer(&request);
    if(retVal != ERR_SUCCESS) return retVal;

    thisUsbDev.DeviceCfgValue = ((USB_CFG_DESCR*)endpRXbuf)->bConfigurationValue;

    USB_AnalyseCfgDescriptor(&thisUsbDev, endpRXbuf, request.wLength);

    return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      CtrlSetUsbAddress
 *
 * @brief   Set USB device address.
 *
 * @param   addr - Device address.
 *
 * @return  Error state
 */
uint8_t USB_CtrlSetAddress(uint8_t addr)
{
    uint8_t retVal;

    USB_SETUP_REQ request ={0};

    request.bRequestType = USB_REQ_TYP_OUT;
    request.bRequest = USB_SET_ADDRESS;
    request.wValue = addr;

    retVal = HostCtrlTransfer(&request);
    if(retVal != ERR_SUCCESS) return retVal;

    // SET ADDRESS
    USBHSH->DEV_AD = addr;
    thisUsbDev.DeviceAddress = addr ;

    Delay_Ms(5);
    return  ERR_SUCCESS;
}

/*********************************************************************
 * @fn      CtrlSetUsbConfig
 *
 * @brief   Set usb configration.
 *
 * @param   cfg_val - device configuration value
 *
 * @return  Error state
 */
uint8_t USB_CtrlSetUsbConfig(uint8_t cfg_val)
{
    USB_SETUP_REQ request ={0};

    request.bRequestType = USB_REQ_TYP_OUT;
    request.bRequest = USB_SET_CONFIGURATION;
    request.wValue = cfg_val;

    return HostCtrlTransfer(&request);
}

/*********************************************************************
 * @fn      CtrlClearEndpStall
 *
 * @brief   clear endpoint stall status.
 *
 * @param   endp - number of endpoint
 *
 * @return  Error state
 */
uint8_t USB_CtrlClearEndpStall(uint8_t endp)
{
    USB_SETUP_REQ request ={0};

    request.bRequestType = USB_REQ_TYP_OUT | USB_REQ_RECIP_ENDP;
    request.bRequest = USB_CLEAR_FEATURE;
    request.wIndex = endp;

    return HostCtrlTransfer(&request);
}

/*********************************************************************
 * @fn      CtrlSetUsbIntercace
 *
 * @brief   Set USB Interface configuration.
 *
 * @param   cfg - configuration value
 *
 * @return  Error state
 */
uint8_t USB_CtrlSetUsbIntercace(uint8_t cfg)
{
    USB_SETUP_REQ request ={0};

    request.bRequestType = USB_REQ_RECIP_INTERF;
    request.bRequest = USB_SET_INTERFACE;
    request.wValue = cfg;

    return HostCtrlTransfer(&request);
}

/*********************************************************************
 * @fn      HubGetPortStatus
 *
 * @brief   Get string descriptor
 *
 * @param   out resBuf - result buffer address
 *          out resLen - result len
 *
 * @return  Error state
 */
uint8_t USB_CtrlGetStringDescr(uint8_t* resBuf, uint16_t* resLen, uint16_t indexId, uint8_t languageIndex)
{
    uint8_t retVal;

    USB_SETUP_REQ request = {0};

    request.bRequestType = USB_REQ_TYP_IN;
    request.bRequest = USB_GET_DESCRIPTOR;
    request.wValue = USB_DESCR_TYP_STRING<<8 | indexId;
    request.wIndex = languageIndex;
    request.wLength = 2;

    retVal = HostCtrlTransfer(&request);
    if(retVal != ERR_SUCCESS)  return retVal;

    request.wLength = ((USB_STRING_DESCR*)endpRXbuf)->bLength;
    retVal = HostCtrlTransfer(&request);
    if(retVal != ERR_SUCCESS) return retVal;

    uint16_t length = ((USB_STRING_DESCR*)endpRXbuf)->bLength-2;

    if(resBuf) memcpy(resBuf, ((USB_STRING_DESCR*)endpRXbuf)->string, length);
    if(resLen) *resLen = length;

    return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      HubGetPortStatus
 *
 * @brief   Check the port of hub,and return the port's status
 *
 * @param   HubPortIndex - index of the hub port index
 *
 * @return  Error state
 */
uint8_t USB_HubGetPortStatus(uint8_t HubPortIndex)
{
    USB_SETUP_REQ request;

    request.bRequestType = HUB_GET_PORT_STATUS;
    request.bRequest = HUB_GET_STATUS;
    request.wValue = 0x0000;
    request.wIndex = 0x0000|HubPortIndex;
    request.wLength = 0x0004;

    uint8_t retVal = HostCtrlTransfer(&request);
    if (retVal != ERR_SUCCESS) return retVal ;

    return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      HubSetPortFeature
 *
 * @brief   set the port feature of hub
 *
 * @param   HubPortIndex - index of the hub port index
 *          FeatureSelt - feature selector
 *
 * @return  Error state
 */
uint8_t USB_HubSetPortFeature(uint8_t HubPortIndex, uint8_t FeatureSelt)
{
    USB_SETUP_REQ request;

    request.bRequestType = HUB_SET_PORT_FEATURE;
    request.bRequest = HUB_SET_FEATURE;
    request.wValue = 0x0000|FeatureSelt;
    request.wIndex = 0x0000|HubPortIndex;
    request.wLength = 0x0000;

    return HostCtrlTransfer(&request);
}

/*********************************************************************
 * @fn      HubClearPortFeature
 *
 * @brief   clear the port feature of hub
 *
 * @param   HubPortIndex - index of the hub port index
 *          FeatureSelt - feature selector
 *
 * @return  Error state
 */
uint8_t USB_HubClearPortFeature(uint8_t HubPortIndex, uint8_t FeatureSelt)
{
    USB_SETUP_REQ request;

    request.bRequestType = HUB_CLEAR_PORT_FEATURE;
    request.bRequest = HUB_CLEAR_FEATURE;
    request.wValue = 0x0000|FeatureSelt;
    request.wIndex = 0x0000|HubPortIndex;
    request.wLength = 0x0000;

    return HostCtrlTransfer(&request);
}

/*********************************************************************
 * @fn      USBOTG_HostEnum
 *
 * @brief   Host enumerated device.
 *
 * @return  Error state
 */
uint8_t USB_HostEnum()
{
  uint8_t retVal;

  USB_SetBusReset();
  Delay_Ms(10);

  USB_DEV_DESCR deviceDescriptor;

  thisUsbDev.DeviceEndp0Size = 8;

  // for PANGAEA CP16 need to set address first
  retVal = USB_CtrlSetAddress(USB_DEVICE_ADDR);
  if(retVal != ERR_SUCCESS)
  {
      printf("set address:%02x\n",retVal);
      return retVal;
  }

  retVal = USB_CtrlGetDevDescr(&deviceDescriptor);
  if(retVal != ERR_SUCCESS)
  {
      printf("Get device descriptor error. Code:%02x\n", retVal);
      return retVal;
  }
  else
  {
      thisUsbDev.DeviceClass = deviceDescriptor.bDeviceClass;
      thisUsbDev.DeviceSubClass = deviceDescriptor.bDeviceSubClass;
      thisUsbDev.VID = deviceDescriptor.idVendor;
      thisUsbDev.PID = deviceDescriptor.idProduct;
      printf("Device VID: %X PID: %X\r\n", thisUsbDev.VID, thisUsbDev.PID);
  }

  if(deviceDescriptor.iManufacturer !=0 )
  {
      uint8_t buf[256];
      uint16_t len;
      USB_CtrlGetStringDescr(buf, &len, deviceDescriptor.iManufacturer, 0);

      for(uint16_t i=0; i<len; ++i)
      {
        printf("%c", buf[i]);
      }
      printf("\r\n");
  }

  if(deviceDescriptor.iProduct !=0 )
  {
      uint8_t buf[256];
      uint16_t len;
      USB_CtrlGetStringDescr(buf, &len, deviceDescriptor.iProduct, 0);

      for(uint16_t i=0; i<len; ++i)
      {
        printf("%c", buf[i]);
      }
      printf("\r\n");
  }

  retVal = USB_CtrlGetConfigDescr();
  if(retVal != ERR_SUCCESS)
  {
      printf("get configuration descriptor:%02x\n", retVal);
      return retVal;
  }

  retVal = USB_CtrlSetUsbConfig(thisUsbDev.DeviceCfgValue);
  if(retVal != ERR_SUCCESS)
  {
      printf("set configuration:%02x\n", retVal);
      return retVal;
  }

  return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      Anaylisys_Descr
 *
 * @brief   Descriptor analysis.
 *
 * @param   pusbdev - device information variable.
 *          pdesc - descriptor buffer to analyze
 *          l - length
 *
 * @return  none
 */
void USB_AnalyseCfgDescriptor(USBDEV_INFO* pusbdev, uint8_t* pdesc, uint16_t len)
{
    for(uint16_t i=0; i<len; i++)
    {
         if((pdesc[i]==USB_DESCR_SIZE_CONFIG)&&(pdesc[i+1]==USB_DESCR_TYP_CONFIG))
         {
             USB_CFG_DESCR* currentCFG_ptr = (USB_CFG_DESCR*)&(pdesc[i]);
             printf("bNumInterfaces:%02x \n", currentCFG_ptr->bNumInterfaces);
         }

         if((pdesc[i]==USB_DESCR_SIZE_ITF)&&(pdesc[i+1]==USB_DESCR_TYP_ITF))
         {
             USB_ITF_DESCR* currentITF_ptr = (USB_ITF_DESCR*)&(pdesc[i]);
             printf("Interface class:%02x num endpoints:%02d\n",
                     currentITF_ptr->bInterfaceClass,
                     currentITF_ptr->bNumEndpoints);

             if(currentITF_ptr->iInterface != 0)
             {
                 uint8_t buf[256];
                 uint16_t len;
                 USB_CtrlGetStringDescr(buf, &len, currentITF_ptr->iInterface, 0);

                 for(uint16_t i=0; i<len; ++i)
                 {
                   printf("%c", buf[i]);
                 }
                 printf("\r\n");
             }
         }

         if((pdesc[i]==USB_DESCR_SIZE_ENDP)&&(pdesc[i+1]==USB_DESCR_TYP_ENDP))
         {
            USB_ENDP_DESCR* currentEndp_ptr = (USB_ENDP_DESCR*)&(pdesc[i]);

            if(currentEndp_ptr->bEndpointAddress & USB_ENDP_DIR_MASK)
            {
                 pusbdev->DevEndp.InEndpNum = currentEndp_ptr->bEndpointAddress & 0x0f;
                 pusbdev->DevEndp.InEndpCount++;
                 pusbdev->DevEndp.InEndpMaxSize = currentEndp_ptr->wMaxPacketSize;
                 pusbdev->DevEndp.InType = currentEndp_ptr->bmAttributes & USB_ENDP_TYPE_MASK;

                 printf("endpIN:%02d maxSize:%02d type:%02x\n",
                         pusbdev->DevEndp.InEndpNum,
                         pusbdev->DevEndp.InEndpMaxSize,
                         pusbdev->DevEndp.InType);
            }
            else
            {
                pusbdev->DevEndp.OutEndpNum = currentEndp_ptr->bEndpointAddress & 0x0f;
                pusbdev->DevEndp.OutEndpCount++;
                pusbdev->DevEndp.OutEndpMaxSize = currentEndp_ptr->wMaxPacketSize;
                pusbdev->DevEndp.OutType = currentEndp_ptr->bmAttributes & USB_ENDP_TYPE_MASK;

                printf("endpOUT:%02d maxSize:%02d type:%02x\n",
                        pusbdev->DevEndp.InEndpNum,
                        pusbdev->DevEndp.InEndpMaxSize,
                        pusbdev->DevEndp.InType);
            }
        }
  }
}
