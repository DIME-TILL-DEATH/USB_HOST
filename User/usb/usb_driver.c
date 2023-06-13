#include <malloc.h>

#include "usb_driver.h"

/*********************************************************************
 * @fn      CtrlGetDevDescr
 *
 * @brief   Get device descriptor
 *
 * @param   target device
 *
 * @return  Error state
 */
uint8_t USB_CtrlGetDevDescr(USBDEV_INFO* usbDevice_ptr)
{
    USB_SETUP_REQ request;

    request.bRequestType = USB_REQ_TYP_IN;
    request.bRequest = USB_GET_DESCRIPTOR;
    request.wValue = USB_DESCR_TYP_DEVICE<<8 | 0x0000;
    request.wIndex = 0;
    request.wLength = USB_DESCR_SIZE_DEVICE;

    uint8_t* replyBuf_ptr;

    uint8_t retVal = USB_HostCtrlTransfer(usbDevice_ptr, &request, &replyBuf_ptr);
    if(retVal != ERR_SUCCESS) return retVal;

    USB_DEV_DESCR* devDescriptor = (USB_DEV_DESCR*)replyBuf_ptr;

    usbDevice_ptr->endp0Size = devDescriptor->bMaxPacketSize0;

    usbDevice_ptr->devClass = devDescriptor->bDeviceClass;
    usbDevice_ptr->devSubClass = devDescriptor->bDeviceSubClass;
    usbDevice_ptr->VID = devDescriptor->idVendor;
    usbDevice_ptr->PID = devDescriptor->idProduct;

    uint8_t iManufacturerDesc = devDescriptor->iManufacturer;
    uint8_t iProductDesc = devDescriptor->iProduct;

    uint8_t buf[256];
    uint16_t len = 0;

    usbDevice_ptr->manufacturerString = NULL;
    usbDevice_ptr->manufacturerStringLen = 0;
    if(iManufacturerDesc != 0)
    {
        USB_CtrlGetStringDescr(usbDevice_ptr, buf, &len, iManufacturerDesc, 0);

        usbDevice_ptr->manufacturerString = (char*)malloc(len);
        memcpy(usbDevice_ptr->manufacturerString, buf, len);
        usbDevice_ptr->manufacturerStringLen = len;
    }

    usbDevice_ptr->productString = NULL;
    usbDevice_ptr->productStringLen = 0;
    if(iProductDesc != 0)
    {
        USB_CtrlGetStringDescr(usbDevice_ptr, buf, &len, iProductDesc, 0);

        usbDevice_ptr->productString = (char*)malloc(len);
        memcpy(usbDevice_ptr->productString, buf, len);
        usbDevice_ptr->productStringLen = len;
    }
    return retVal;
}
/*********************************************************************
 * @fn      CtrlGetConfigDescr
 *
 * @brief   Get configuration descriptor.
 *
 * @return  Error state
 */
uint8_t USB_CtrlGetConfigDescr(USBDEV_INFO* usbDevice_ptr)
{
    uint8_t retVal;

    USB_SETUP_REQ request;

    request.bRequestType = USB_REQ_TYP_IN;
    request.bRequest = USB_GET_DESCRIPTOR;
    request.wValue = USB_DESCR_TYP_CONFIG<<8 | 0x0000;
    request.wIndex = 0;
    request.wLength = USB_DESCR_SIZE_CONFIG;

    uint8_t* replyBuf_ptr;

    retVal = USB_HostCtrlTransfer(usbDevice_ptr, &request, &replyBuf_ptr);
    if(retVal != ERR_SUCCESS)  return retVal;

    USB_CFG_DESCR* cfgDescriptor_ptr = (USB_CFG_DESCR*)replyBuf_ptr;

    request.wLength = cfgDescriptor_ptr->wTotalLength;

    retVal = USB_HostCtrlTransfer(usbDevice_ptr, &request, &replyBuf_ptr);
    if(retVal != ERR_SUCCESS) return retVal;

    usbDevice_ptr->devCfgValue = ((USB_CFG_DESCR*)replyBuf_ptr)->bConfigurationValue;

    USB_ParseFullCfgDescriptor(usbDevice_ptr, replyBuf_ptr, request.wLength);

    return ERR_SUCCESS;
}
/*********************************************************************
 * @fn      USB_ParseFullCfgDescriptor
 *
 * @brief   Full config descriptor parsing.
 *
 *
 * @return  none
 */
void USB_ParseFullCfgDescriptor(USBDEV_INFO* usbDevice_ptr, uint8_t* descriptorBuf_ptr, uint16_t len)
{
    uint8_t curItfNum1 = 0;
    uint8_t curItfNum2 = 0;
    uint8_t curEndpNum = 0;

    for(uint16_t i=0; i<len; i++)
    {
         if((descriptorBuf_ptr[i]==USB_DESCR_SIZE_CONFIG)&&(descriptorBuf_ptr[i+1]==USB_DESCR_TYP_CONFIG))
         {
             USB_CFG_DESCR* currentCFG_ptr = (USB_CFG_DESCR*)&(descriptorBuf_ptr[i]);

             usbDevice_ptr->itfNum = currentCFG_ptr->bNumInterfaces;
             usbDevice_ptr->itfInfo = (USBITF_INFO*)calloc(usbDevice_ptr->itfNum, sizeof(USBITF_INFO));
         }

         if((descriptorBuf_ptr[i]==USB_DESCR_SIZE_ITF)&&(descriptorBuf_ptr[i+1]==USB_DESCR_TYP_ITF))
         {
             USB_ITF_DESCR* currentITF_ptr = (USB_ITF_DESCR*)&(descriptorBuf_ptr[i]);
             usbDevice_ptr->itfInfo[curItfNum1].itfNumber = currentITF_ptr->bInterfaceNumber;
             usbDevice_ptr->itfInfo[curItfNum1].itfClass = currentITF_ptr->bInterfaceClass;
             usbDevice_ptr->itfInfo[curItfNum1].endpCount = currentITF_ptr->bNumEndpoints;
             usbDevice_ptr->itfInfo[curItfNum1].endpInfo = (USBENDP_INFO*)calloc(usbDevice_ptr->itfInfo[curItfNum1].endpCount, sizeof(USBENDP_INFO));

             curItfNum1++;
         }

         if((descriptorBuf_ptr[i]==USB_DESCR_SIZE_ENDP)&&(descriptorBuf_ptr[i+1]==USB_DESCR_TYP_ENDP))
         {
            USB_ENDP_DESCR* currentEndp_ptr = (USB_ENDP_DESCR*)&(descriptorBuf_ptr[i]);

            usbDevice_ptr->itfInfo[curItfNum2].endpInfo[curEndpNum].direction = currentEndp_ptr->bEndpointAddress & USB_ENDP_DIR_MASK;
            usbDevice_ptr->itfInfo[curItfNum2].endpInfo[curEndpNum].endpAddress = currentEndp_ptr->bEndpointAddress & USB_ENDP_ADDR_MASK;
            usbDevice_ptr->itfInfo[curItfNum2].endpInfo[curEndpNum].endpMaxSize = currentEndp_ptr->wMaxPacketSize;
            usbDevice_ptr->itfInfo[curItfNum2].endpInfo[curEndpNum].type = currentEndp_ptr->bmAttributes & USB_ENDP_TYPE_MASK;
            usbDevice_ptr->itfInfo[curItfNum2].endpInfo[curEndpNum].toggle = 0;

            curEndpNum++;
            if(curEndpNum == usbDevice_ptr->itfInfo[curItfNum2].endpCount)
            {
                curItfNum2++;
                curEndpNum = 0;
            }
        }
  }
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
uint8_t USB_CtrlSetAddress(USBDEV_INFO* usbDevice_ptr, uint8_t addr)
{
    uint8_t retVal;

    USB_SETUP_REQ request ={0};

    request.bRequestType = USB_REQ_TYP_OUT;
    request.bRequest = USB_SET_ADDRESS;
    request.wValue = addr;

    usbDevice_ptr->devAddress = 0;
    retVal = USB_HostCtrlTransfer(usbDevice_ptr, &request, NULL);
    if(retVal != ERR_SUCCESS) return retVal;

    // SET ADDRESS
    USBHSH->DEV_AD = addr;
    usbDevice_ptr->devAddress = addr ;

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
uint8_t USB_CtrlSetUsbConfig(USBDEV_INFO* usbDevice_ptr, uint8_t cfgVal)
{
    USB_SETUP_REQ request ={0};

    request.bRequestType = USB_REQ_TYP_OUT;
    request.bRequest = USB_SET_CONFIGURATION;
    request.wValue = cfgVal;

    return USB_HostCtrlTransfer(usbDevice_ptr, &request, NULL);
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
uint8_t USB_CtrlClearEndpStall(USBDEV_INFO* usbDevice_ptr, uint8_t endp)
{
    USB_SETUP_REQ request ={0};

    request.bRequestType = USB_REQ_TYP_OUT | USB_REQ_RECIP_ENDP;
    request.bRequest = USB_CLEAR_FEATURE;
    request.wIndex = endp;

    return USB_HostCtrlTransfer(usbDevice_ptr, &request, NULL);
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
uint8_t USB_CtrlSetUsbIntercace(USBDEV_INFO* usbDevice_ptr, uint8_t cfg)
{
    USB_SETUP_REQ request ={0};

    request.bRequestType = USB_REQ_RECIP_INTERF;
    request.bRequest = USB_SET_INTERFACE;
    request.wValue = cfg;

    return USB_HostCtrlTransfer(usbDevice_ptr, &request, NULL);
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
uint8_t USB_CtrlGetStringDescr(USBDEV_INFO* usbDevice_ptr,
                                uint8_t* resultBuf_ptr, uint16_t* resultrLen_ptr,
                                uint16_t indexId, uint8_t languageIndex)
{
    uint8_t retVal;

    USB_SETUP_REQ request = {0};

    request.bRequestType = USB_REQ_TYP_IN;
    request.bRequest = USB_GET_DESCRIPTOR;
    request.wValue = USB_DESCR_TYP_STRING<<8 | indexId;
    request.wIndex = languageIndex;
    request.wLength = 2;

    uint8_t* replyBuf_ptr;

    retVal = USB_HostCtrlTransfer(usbDevice_ptr, &request, &replyBuf_ptr);
    if(retVal != ERR_SUCCESS)  return retVal;

    request.wLength = ((USB_STRING_DESCR*)replyBuf_ptr)->bLength;
    retVal = USB_HostCtrlTransfer(usbDevice_ptr, &request, &replyBuf_ptr);
    if(retVal != ERR_SUCCESS) return retVal;

    uint16_t length = ((USB_STRING_DESCR*)replyBuf_ptr)->bLength-2;

    if(resultBuf_ptr) memcpy(resultBuf_ptr, ((USB_STRING_DESCR*)replyBuf_ptr)->string, length);
    if(resultrLen_ptr) *resultrLen_ptr = length;

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
uint8_t USB_HubGetPortStatus(USBDEV_INFO* usbDevice_ptr, uint8_t fubPortIndex)
{
//    USB_SETUP_REQ request;
//
//    request.bRequestType = HUB_GET_PORT_STATUS;
//    request.bRequest = HUB_GET_STATUS;
//    request.wValue = 0x0000;
//    request.wIndex = 0x0000|fubPortIndex;
//    request.wLength = 0x0004;
//
//    uint8_t retVal = USB_HostCtrlTransfer(usbDevice_ptr, &request);
//    if (retVal != ERR_SUCCESS) return retVal ;

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
uint8_t USB_HubSetPortFeature(USBDEV_INFO* usbDevice_ptr, uint8_t hubPortIndex, uint8_t featureSelt)
{
//    USB_SETUP_REQ request;
//
//    request.bRequestType = HUB_SET_PORT_FEATURE;
//    request.bRequest = HUB_SET_FEATURE;
//    request.wValue = 0x0000|featureSelt;
//    request.wIndex = 0x0000|hubPortIndex;
//    request.wLength = 0x0000;

//    return USB_HostCtrlTransfer(usbDevice_ptr, &request);
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
uint8_t USB_HubClearPortFeature(USBDEV_INFO* usbDevice_ptr, uint8_t hubPortIndex, uint8_t featureSel)
{
//    USB_SETUP_REQ request;
//
//    request.bRequestType = HUB_CLEAR_PORT_FEATURE;
//    request.bRequest = HUB_CLEAR_FEATURE;
//    request.wValue = 0x0000|featureSel;
//    request.wIndex = 0x0000|hubPortIndex;
//    request.wLength = 0x0000;

//    return USB_HostCtrlTransfer(usbDevice_ptr, &request);
}

/*********************************************************************
 * @fn      USBOTG_HostEnum
 *
 * @brief   Host enumerated device.
 *
 * @return  Error state
 */
uint8_t USB_HostEnum(USBDEV_INFO* usbDevice_ptr)
{
  uint8_t retVal;

  USB_SetBusReset();
  Delay_Ms(10);

  usbDevice_ptr->endp0Size = 8;

  // for PANGAEA CP16 need to set address first
  retVal = USB_CtrlSetAddress(usbDevice_ptr, USB_DEVICE_ADDR);
  if(retVal != ERR_SUCCESS)
  {
      printf("set address:%02x\n",retVal);
      return retVal;
  }

  retVal = USB_CtrlGetDevDescr(usbDevice_ptr);
  if(retVal != ERR_SUCCESS)
  {
      printf("Get device descriptor error. Code:%02x\n", retVal);
      return retVal;
  }

  retVal = USB_CtrlGetConfigDescr(usbDevice_ptr);
  if(retVal != ERR_SUCCESS)
  {
      printf("get configuration descriptor:%02x\n", retVal);
      return retVal;
  }

  retVal = USB_CtrlSetUsbConfig(usbDevice_ptr, usbDevice_ptr->devCfgValue);
  if(retVal != ERR_SUCCESS)
  {
      printf("set configuration:%02x\n", retVal);
      return retVal;
  }

  return ERR_SUCCESS;
}

/*****************************************************************
 * @fn USB_PrintDevInfo
 *
 * @brief print all device info to debug console
 */
void USB_PrintDevInfo(USBDEV_INFO* devInfoStruct)
{
    printf("Device VID: %X PID: %X\r\n", devInfoStruct->VID, devInfoStruct->PID);

    for(uint16_t i=0; i<devInfoStruct->manufacturerStringLen; ++i)
    {
      printf("%c", devInfoStruct->manufacturerString[i]);
    }
    printf("\r\n");

    for(uint16_t i=0; i<devInfoStruct->productStringLen; ++i)
    {
      printf("%c", devInfoStruct->productString[i]);
    }
    printf("\r\n");

    printf("bNumInterfaces:%02x \n", devInfoStruct->itfNum);

    for(uint8_t i=0; i<devInfoStruct->itfNum; i++)
    {
        printf("Interface class:%02x number:%d num endpoints:%02d\n",
                devInfoStruct->itfInfo[i].itfClass,
                devInfoStruct->itfInfo[i].itfNumber,
                devInfoStruct->itfInfo[i].endpCount);

        for(uint8_t eNum=0; eNum<devInfoStruct->itfInfo[i].endpCount; eNum++)
        {
            printf("Endp dir:%02x addr:%02d maxSize:%02d type:%02x\n",
                    devInfoStruct->itfInfo[i].endpInfo[eNum].direction,
                    devInfoStruct->itfInfo[i].endpInfo[eNum].endpAddress,
                    devInfoStruct->itfInfo[i].endpInfo[eNum].endpMaxSize,
                    devInfoStruct->itfInfo[i].endpInfo[eNum].type);
        }
    }
}

/*****************************************************************
* @fn USB_FreeDevStruct
*
* @brief free all memory used for USBDEV_INFO struct
*/

void USB_FreeDevStruct(USBDEV_INFO* devInfoStruct)
{
    if(devInfoStruct)
    {
        if(devInfoStruct->manufacturerString)
        {
            free(devInfoStruct->manufacturerString);
            devInfoStruct->manufacturerString = NULL;
        }

        if(devInfoStruct->productString)
        {
            free(devInfoStruct->productString);
            devInfoStruct->productString = NULL;
        }

        USBITF_INFO* itfInfo_ptr = devInfoStruct->itfInfo;
        if(itfInfo_ptr)
        {
            USBENDP_INFO* endpInfo_ptr = devInfoStruct->itfInfo->endpInfo;
            if(endpInfo_ptr)
            {
                free(endpInfo_ptr);
                endpInfo_ptr = NULL;
            }

            free(itfInfo_ptr);
            itfInfo_ptr = NULL;
        }
        free(devInfoStruct);
        devInfoStruct = NULL;
    }
}
