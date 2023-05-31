/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_usbhs_host.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : This file provides all the USB firmware functions.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/ 
#include "ch32v30x_usbhs_host.h"

uint8_t  FoundNewDev = 0;
uint8_t  Endp0MaxSize = 8;
uint8_t  UsbDevEndp0Size = 8;
uint16_t EndpnMaxSize = 8;
USBDEV_INFO  thisUsbDev;

/******************************** IN-OUT BUFFERS ********************************/
__attribute__ ((aligned(4))) uint8_t   endpTXbuf[MAX_PACKET_SIZE];  // OUT, must even address
__attribute__ ((aligned(4))) uint8_t   endpRXbuf[MAX_PACKET_SIZE];  // OUT, must even address

/******************************** HOST DEVICE **********************************/
__attribute__ ((aligned(4))) const uint8_t  RequestDeviceDescriptor[]={USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_DEVICE, 0x00, 0x00, 0x12, 0x00};
__attribute__ ((aligned(4))) const uint8_t  RequestConfigDescriptor[]= {USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_CONFIG, 0x00, 0x00, 0x04, 0x00};
__attribute__ ((aligned(4))) const uint8_t  SetAddress[]={USB_REQ_TYP_OUT, USB_SET_ADDRESS, USB_DEVICE_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00};
__attribute__ ((aligned(4))) const uint8_t  SetConfig[]={USB_REQ_TYP_OUT, USB_SET_CONFIGURATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
__attribute__ ((aligned(4))) const uint8_t  Clear_EndpStall[]={USB_REQ_RECIP_INTERF, USB_SET_INTERFACE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
__attribute__ ((aligned(4))) const uint8_t  SetupSetUsbInterface[] = { USB_REQ_RECIP_INTERF, USB_SET_INTERFACE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
__attribute__ ((aligned(4))) const uint8_t  SetupClrEndpStall[] = { USB_REQ_TYP_OUT | USB_REQ_RECIP_ENDP, USB_CLEAR_FEATURE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

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
uint8_t USBHostTransact(uint8_t endp_pid, uint8_t toggle, uint32_t timeout)
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
 * @param   databuf - Receiving or send data buffer.
 *          RetLen - Data length.
 *
 * @return  Error state
 */
uint8_t HostCtrlTransfer(uint8_t* dataBuf_ptr, uint8_t* len_ptr)
{
    uint8_t   retVal;
    uint8_t   tog = 1;

    if(len_ptr)  *len_ptr = 0;

    Delay_Us(100);

    // setup stage
    USBHSH->HOST_TX_LEN = 8;
    retVal = USBHostTransact((USB_PID_SETUP<<4)|DEF_ENDP_0, 0, 200000);
    if(retVal != ERR_SUCCESS) return retVal;

    // data stage
    uint16_t requestLen = pSetupReq->wLength;
    if(requestLen && dataBuf_ptr)
    {
       if((pSetupReq->bRequestType) & USB_REQ_TYP_IN)            // device to host
       {
           while(requestLen)
           {
               USBHSH->HOST_RX_DMA = (uint32_t)dataBuf_ptr + *len_ptr;
               retVal = USBHostTransact((USB_PID_IN<<4)| DEF_ENDP_0, tog<<3, 20000);
               if(retVal != ERR_SUCCESS) return retVal;

               tog ^=1;
               uint32_t rxlen = (USBHSH->RX_LEN < requestLen) ? USBHSH->RX_LEN : requestLen;
               requestLen -= rxlen;
               if(len_ptr) *len_ptr += rxlen;
               if((USBHSH->RX_LEN == 0) || (USBHSH->RX_LEN & (UsbDevEndp0Size - 1)))  break;
            }
            USBHSH->HOST_TX_LEN = 0 ;
         }
       else
       {                                                           // host to device
          while(requestLen)
          {
               USBHSH->HOST_TX_DMA = (uint32_t)dataBuf_ptr + *len_ptr;
               USBHSH->HOST_TX_LEN = (requestLen >= UsbDevEndp0Size) ? UsbDevEndp0Size : requestLen;

               retVal = USBHostTransact((USB_PID_OUT<<4)|DEF_ENDP_0,  tog<<3,  20000);
               if(retVal != ERR_SUCCESS) return retVal;
               tog ^=1;
               requestLen -= USBHSH->HOST_TX_LEN;
               if(len_ptr) *len_ptr += USBHSH->HOST_TX_LEN;
          }
        }
    }

    // status stage
    retVal = USBHostTransact(((USBHSH->HOST_TX_LEN) ? USB_PID_IN<<4|DEF_ENDP_0 : USB_PID_OUT<<4|DEF_ENDP_0),
            UH_R_TOG_1|UH_T_TOG_1, 20000 );

    if(retVal != ERR_SUCCESS) return retVal;

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
uint8_t CtrlGetDevDescr(USB_DEV_DESCR* devDescriptor)
{
    uint8_t retVal;
    uint8_t len;

    memcpy(pSetupReq, RequestDeviceDescriptor, sizeof(USB_SETUP_REQ));

    retVal = HostCtrlTransfer(endpRXbuf, &len);
    if(retVal != ERR_SUCCESS) return retVal;
    if(len < pSetupReq->wLength) return ERR_USB_BUF_OVER;

    UsbDevEndp0Size = ((USB_DEV_DESCR*)endpRXbuf)->bMaxPacketSize0;

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
uint8_t CtrlGetConfigDescr()
{
    uint8_t retVal;
    uint8_t len;

    memcpy(pSetupReq, RequestConfigDescriptor, sizeof(USB_SETUP_REQ));
    retVal = HostCtrlTransfer(endpRXbuf, &len);
    if(retVal != ERR_SUCCESS)  return retVal;
    if(len < pSetupReq->wLength) return ERR_USB_BUF_OVER;

    memcpy(pSetupReq, RequestConfigDescriptor, sizeof(USB_SETUP_REQ));
    pSetupReq->wLength = ((PUSB_CFG_DESCR)endpRXbuf)->wTotalLength;
    retVal = HostCtrlTransfer(endpRXbuf, &len);
    if(retVal != ERR_SUCCESS) return retVal;

    thisUsbDev.DeviceCongValue = ((PUSB_CFG_DESCR)endpRXbuf)->bConfigurationValue;

    AnalyseDescriptor(&thisUsbDev, endpRXbuf, pSetupReq->wLength);

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
uint8_t CtrlSetAddress(uint8_t addr)
{
    uint8_t retVal;

    memcpy(pSetupReq, SetAddress, sizeof(USB_SETUP_REQ));
    pSetupReq->wValue = addr;
    retVal = HostCtrlTransfer(NULL, NULL);
    if(retVal != ERR_SUCCESS) return retVal;

    USB_CurrentAddress(addr);
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
uint8_t CtrlSetUsbConfig(uint8_t cfg_val)
{
    memcpy(pSetupReq, SetConfig, sizeof(USB_SETUP_REQ));
    pSetupReq->wValue = cfg_val;
    return HostCtrlTransfer(NULL, NULL);
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
uint8_t CtrlClearEndpStall(uint8_t endp)
{
    memcpy(pSetupReq, SetupClrEndpStall, sizeof(USB_SETUP_REQ));
    pSetupReq -> wIndex = endp;
    return HostCtrlTransfer(NULL, NULL);
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
uint8_t CtrlSetUsbIntercace(uint8_t cfg)
{
    memcpy(pSetupReq, SetupSetUsbInterface, sizeof(USB_SETUP_REQ));
    pSetupReq -> wValue = cfg;
    return HostCtrlTransfer( NULL, NULL );
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
uint8_t   HubGetPortStatus(uint8_t HubPortIndex)
{
    uint8_t   retVal;
    uint8_t  len;

    pSetupReq -> bRequestType = HUB_GET_PORT_STATUS;
    pSetupReq -> bRequest = HUB_GET_STATUS;
    pSetupReq -> wValue = 0x0000;
    pSetupReq -> wIndex = 0x0000|HubPortIndex;
    pSetupReq -> wLength = 0x0004;
    retVal = HostCtrlTransfer(endpRXbuf, &len);
    if (retVal != ERR_SUCCESS) return retVal ;
    if (len < 4) return ERR_USB_BUF_OVER;

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
uint8_t HubSetPortFeature(uint8_t HubPortIndex, uint8_t FeatureSelt)
{
    pSetupReq->bRequestType = HUB_SET_PORT_FEATURE;
    pSetupReq->bRequest = HUB_SET_FEATURE;
    pSetupReq->wValue = 0x0000|FeatureSelt;
    pSetupReq->wIndex = 0x0000|HubPortIndex;
    pSetupReq->wLength = 0x0000;
    return(HostCtrlTransfer(NULL, NULL ));
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
uint8_t HubClearPortFeature(uint8_t HubPortIndex, uint8_t FeatureSelt)
{
    pSetupReq->bRequestType = HUB_CLEAR_PORT_FEATURE;
    pSetupReq->bRequest = HUB_CLEAR_FEATURE;
    pSetupReq->wValue = 0x0000|FeatureSelt;
    pSetupReq->wIndex = 0x0000|HubPortIndex;
    pSetupReq->wLength = 0x0000;
    return(HostCtrlTransfer(NULL, NULL));
}

/*********************************************************************
 * @fn      USBOTG_HostEnum
 *
 * @brief   Host enumerated device.
 *
 * @return  Error state
 */
uint8_t USBHS_HostEnum()
{
  uint8_t retVal;

  USB_SetBusReset();
  Delay_Ms(10);

  USB_DEV_DESCR deviceDescriptor;

  UsbDevEndp0Size = 8;

  // for PANGAEA CP16 need to set address first
  retVal = CtrlSetAddress(((USB_SETUP_REQ*)SetAddress)->wValue);
  if(retVal != ERR_SUCCESS)
  {
      printf("set address:%02x\n",retVal);
      return retVal;
  }

  retVal = CtrlGetDevDescr(&deviceDescriptor);
  if(retVal != ERR_SUCCESS)
  {
      printf("Get device descriptor error. Code:%02x\n", retVal);
      return retVal;
  }
  else
  {
      printf("Device VID: %X PID: %X\r\n", deviceDescriptor.idVendor, deviceDescriptor.idProduct);
  }

  retVal = CtrlGetConfigDescr();
  if(retVal != ERR_SUCCESS)
  {
      printf("get configuration descriptor:%02x\n", retVal);
      return retVal;
  }

  retVal = CtrlSetUsbConfig(thisUsbDev.DeviceCongValue);
  if(retVal != ERR_SUCCESS)
  {
      printf("set configuration:%02x\n", retVal);
      return retVal;
  }
  return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      USBHS_CurrentAddr
 *
 * @brief   Current device address.
 *
 * @param   address - Endpoint address.
 *
 * @return  none
 */
void USB_CurrentAddress(uint8_t address)
{
    USBHSH->DEV_AD = address;                  // SET ADDRESS
    thisUsbDev.DeviceAddress = address ;
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
void AnalyseDescriptor(USBDEV_INFO* pusbdev, uint8_t* pdesc, uint16_t len)
{
    uint16_t i;
    for( i=0; i<len; i++ )                                                //分析描述符
    {
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x02))
         {
                printf("bNumInterfaces:%02x \n",pdesc[i+4]);            //配置描述符里的接口数-第5个字节
         }

         if((pdesc[i]==0x07)&&(pdesc[i+1]==0x05))
         {
            if((pdesc[i+2])&0x80)
            {
                 printf("endpIN:%02x \n",pdesc[i+2]&0x0f);              //取in端点号
                 pusbdev->DevEndp.InEndpNum = pdesc[i+2]&0x0f;
                 pusbdev->DevEndp.InEndpCount++;
                 EndpnMaxSize = ((uint16_t)pdesc[i+5]<<8)|pdesc[i+4];     //取端点大小
                 pusbdev->DevEndp.InEndpMaxSize = EndpnMaxSize;
                 printf("In_endpmaxsize:%02x \n", EndpnMaxSize);
            }
            else
            {
                printf("endpOUT:%02x \n",pdesc[i+2]&0x0f);              //取out端点号
                pusbdev->DevEndp.OutEndpNum = pdesc[i+2]&0x0f;
                pusbdev->DevEndp.OutEndpCount++;
                EndpnMaxSize =((uint16_t)pdesc[i+5]<<8)|pdesc[i+4];        //取端点大小
                pusbdev->DevEndp.OutEndpMaxSize = EndpnMaxSize;
                printf("Out_endpmaxsize:%02x \n", EndpnMaxSize);
            }
        }
  }
}


