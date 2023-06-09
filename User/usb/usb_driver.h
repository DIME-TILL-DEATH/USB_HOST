#ifndef USER_USB_USB_DRIVER_H_
#define USER_USB_USB_DRIVER_H_

#include <string.h>

#include "debug.h"
#include "usb_defines.h"

#include "ch32v30x_usbhs_host.h"

uint8_t USB_CtrlGetDevDescr(USBDEV_INFO* usbDevice_ptr);
uint8_t USB_CtrlGetConfigDescr(USBDEV_INFO* usbDevice_ptr);
void    USB_ParseFullCfgDescriptor(USBDEV_INFO* usbDevice_ptr, uint8_t* descriptorBuf_ptr, uint16_t len);

uint8_t USB_CtrlGetStringDescr(USBDEV_INFO* usbDevice_ptr,
                                uint8_t* resultBuf_ptr, uint16_t* resultrLen_ptr,
                                uint16_t indexId, uint8_t languageIndex);

uint8_t USB_CtrlSetAddress(USBDEV_INFO* usbDevice_ptr, uint8_t addr);
uint8_t USB_CtrlSetUsbConfig(USBDEV_INFO* usbDevice_ptr, uint8_t cfgVal);
uint8_t USB_CtrlSetUsbIntercace(USBDEV_INFO* usbDevice_ptr, uint8_t cfg);

uint8_t USB_CtrlClearEndpStall(USBDEV_INFO* usbDevice_ptr, uint8_t endp);

uint8_t USB_HubGetPortStatus(USBDEV_INFO* usbDevice_ptr, uint8_t fubPortIndex);
uint8_t USB_HubSetPortFeature(USBDEV_INFO* usbDevice_ptr, uint8_t hubPortIndex, uint8_t featureSelt);
uint8_t USB_HubClearPortFeature(USBDEV_INFO* usbDevice_ptr, uint8_t hubPortIndex, uint8_t featureSel);


uint8_t USB_HostEnum(USBDEV_INFO* usbDevice_ptr);

void USB_PrintDevInfo(USBDEV_INFO* devInfoStruct);
void USB_FreeDevStruct(USBDEV_INFO* devInfoStruct);

#endif /* USER_USB_USB_DRIVER_H_ */
