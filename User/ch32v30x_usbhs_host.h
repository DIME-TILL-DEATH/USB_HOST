/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32v30x_usbhs_host.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : This file contains all the functions prototypes for the USB 
*                      Host firmware library.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/ 
#ifndef __CH32V30x_USBHS_HOST_H
#define __CH32V30x_USBHS_HOST_H

#include "debug.h"
#include "string.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* USB constant and structure define */

/* USB PID */
#ifndef USB_PID_SETUP
#define USB_PID_NULL            0x00    /* reserved PID */
#define USB_PID_SOF             0x05
#define USB_PID_SETUP           0x0D
#define USB_PID_IN              0x09
#define USB_PID_OUT             0x01
#define USB_PID_ACK             0x02
#define USB_PID_NYET            0x06
#define USB_PID_NAK             0x0A
#define USB_PID_STALL           0x0E
#define USB_PID_DATA0           0x03
#define USB_PID_DATA1           0x0B
#define USB_PID_PRE             0x0C
#endif

/* USB standard device request code */
#ifndef USB_GET_DESCRIPTOR
#define USB_GET_STATUS          0x00
#define USB_CLEAR_FEATURE       0x01
#define USB_SET_FEATURE         0x03
#define USB_SET_ADDRESS         0x05
#define USB_GET_DESCRIPTOR      0x06
#define USB_SET_DESCRIPTOR      0x07
#define USB_GET_CONFIGURATION   0x08
#define USB_SET_CONFIGURATION   0x09
#define USB_GET_INTERFACE       0x0A
#define USB_SET_INTERFACE       0x0B
#define USB_SYNCH_FRAME         0x0C
#endif

/* USB hub class request code */
#ifndef HUB_GET_DESCRIPTOR
#define HUB_GET_STATUS          0x00
#define HUB_CLEAR_FEATURE       0x01
#define HUB_GET_STATE           0x02
#define HUB_SET_FEATURE         0x03
#define HUB_GET_DESCRIPTOR      0x06
#define HUB_SET_DESCRIPTOR      0x07
#endif

/* USB HID class request code */
#ifndef HID_GET_REPORT
#define HID_GET_REPORT          0x01
#define HID_GET_IDLE            0x02
#define HID_GET_PROTOCOL        0x03
#define HID_SET_REPORT          0x09
#define HID_SET_IDLE            0x0A
#define HID_SET_PROTOCOL        0x0B
#endif

/* Bit define for USB request type */
#ifndef USB_REQ_TYP_MASK
#define USB_REQ_TYP_IN          0x80            /* control IN, device to host */
#define USB_REQ_TYP_OUT         0x00            /* control OUT, host to device */
#define USB_REQ_TYP_READ        0x80            /* control read, device to host */
#define USB_REQ_TYP_WRITE       0x00            /* control write, host to device */
#define USB_REQ_TYP_MASK        0x60            /* bit mask of request type */
#define USB_REQ_TYP_STANDARD    0x00
#define USB_REQ_TYP_CLASS       0x20
#define USB_REQ_TYP_VENDOR      0x40
#define USB_REQ_TYP_RESERVED    0x60
#define USB_REQ_RECIP_MASK      0x1F            /* bit mask of request recipient */
#define USB_REQ_RECIP_DEVICE    0x00
#define USB_REQ_RECIP_INTERF    0x01
#define USB_REQ_RECIP_ENDP      0x02
#define USB_REQ_RECIP_OTHER     0x03
#endif

/* USB request type for hub class request */
#ifndef HUB_GET_HUB_DESCRIPTOR
#define HUB_CLEAR_HUB_FEATURE   0x20
#define HUB_CLEAR_PORT_FEATURE  0x23
#define HUB_GET_BUS_STATE       0xA3
#define HUB_GET_HUB_DESCRIPTOR  0xA0
#define HUB_GET_HUB_STATUS      0xA0
#define HUB_GET_PORT_STATUS     0xA3
#define HUB_SET_HUB_DESCRIPTOR  0x20
#define HUB_SET_HUB_FEATURE     0x20
#define HUB_SET_PORT_FEATURE    0x23
#endif

/* Hub class feature selectors */
#ifndef HUB_PORT_RESET
#define HUB_C_HUB_LOCAL_POWER   0
#define HUB_C_HUB_OVER_CURRENT  1
#define HUB_PORT_CONNECTION     0
#define HUB_PORT_ENABLE         1
#define HUB_PORT_SUSPEND        2
#define HUB_PORT_OVER_CURRENT   3
#define HUB_PORT_RESET          4
#define HUB_PORT_POWER          8
#define HUB_PORT_LOW_SPEED      9
#define HUB_C_PORT_CONNECTION   16
#define HUB_C_PORT_ENABLE       17
#define HUB_C_PORT_SUSPEND      18
#define HUB_C_PORT_OVER_CURRENT 19
#define HUB_C_PORT_RESET        20
#endif

/* USB descriptor type */
#ifndef USB_DESCR_TYP_DEVICE
#define USB_DESCR_TYP_DEVICE    0x01
#define USB_DESCR_TYP_CONFIG    0x02
#define USB_DESCR_TYP_STRING    0x03
#define USB_DESCR_TYP_INTERF    0x04
#define USB_DESCR_TYP_ENDP      0x05
#define USB_DESCR_TYP_QUALIF    0x06
#define USB_DESCR_TYP_SPEED     0x07
#define USB_DESCR_TYP_OTG       0x09
#define USB_DESCR_TYP_BOS       0X0F
#define USB_DESCR_TYP_HID       0x21
#define USB_DESCR_TYP_REPORT    0x22
#define USB_DESCR_TYP_PHYSIC    0x23
#define USB_DESCR_TYP_CS_INTF   0x24
#define USB_DESCR_TYP_CS_ENDP   0x25
#define USB_DESCR_TYP_HUB       0x29
#endif

/* USB device class */
#ifndef USB_DEV_CLASS_HUB
#define USB_DEV_CLASS_RESERVED  0x00
#define USB_DEV_CLASS_AUDIO     0x01
#define USB_DEV_CLASS_COMMUNIC  0x02
#define USB_DEV_CLASS_HID       0x03
#define USB_DEV_CLASS_MONITOR   0x04
#define USB_DEV_CLASS_PHYSIC_IF 0x05
#define USB_DEV_CLASS_POWER     0x06
#define USB_DEV_CLASS_PRINTER   0x07
#define USB_DEV_CLASS_STORAGE   0x08
#define USB_DEV_CLASS_HUB       0x09
#define USB_DEV_CLASS_VEN_SPEC  0xFF
#endif

/* USB endpoint type and attributes */
#ifndef USB_ENDP_TYPE_MASK
#define USB_ENDP_DIR_MASK       0x80
#define USB_ENDP_ADDR_MASK      0x0F
#define USB_ENDP_TYPE_MASK      0x03
#define USB_ENDP_TYPE_CTRL      0x00
#define USB_ENDP_TYPE_ISOCH     0x01
#define USB_ENDP_TYPE_BULK      0x02
#define USB_ENDP_TYPE_INTER     0x03
#endif

#ifndef USB_DEVICE_ADDR
#define USB_DEVICE_ADDR         0x02
#endif

#ifndef DEFAULT_ENDP0_SIZE
#define DEFAULT_ENDP0_SIZE      8       /* default maximum packet size for endpoint 0 */
#endif

#ifndef MAX_PACKET_SIZE
#define MAX_PACKET_SIZE         512      /* maximum packet size */
#endif

#ifndef USB_BO_CBW_SIZE
#define USB_BO_CBW_SIZE         0x1F
#define USB_BO_CSW_SIZE         0x0D
#endif

#ifndef USB_BO_CBW_SIG0
#define USB_BO_CBW_SIG0         0x55
#define USB_BO_CBW_SIG1         0x53
#define USB_BO_CBW_SIG2         0x42
#define USB_BO_CBW_SIG3         0x43
#define USB_BO_CSW_SIG0         0x55
#define USB_BO_CSW_SIG1         0x53
#define USB_BO_CSW_SIG2         0x42
#define USB_BO_CSW_SIG3         0x53
#endif

#ifndef __PACKED
#define __PACKED  __attribute__((packed))
#endif

typedef struct __PACKED _USB_SETUP_REQ {
    uint8_t bRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
}USB_SETUP_REQ;

typedef struct __PACKED _USB_DEVICE_DESCR {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
}USB_DEV_DESCR;

typedef struct __PACKED _USB_CONFIG_DESCR {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t MaxPower;
}USB_CFG_DESCR, *PUSB_CFG_DESCR;

typedef struct __PACKED _USB_INTERF_DESCR {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
}USB_ITF_DESCR;

typedef struct __PACKED _USB_ENDPOINT_DESCR {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
}USB_ENDP_DESCR;

typedef struct __PACKED _USB_CONFIG_DESCR_LONG {
    USB_CFG_DESCR   cfg_descr;
    USB_ITF_DESCR   itf_descr;
    USB_ENDP_DESCR  endp_descr[1];
}USB_CFG_DESCR_LONG;

typedef struct __PACKED _USB_HUB_DESCR {
    uint8_t bDescLength;
    uint8_t bDescriptorType;
    uint8_t bNbrPorts;
    uint8_t wHubCharacteristicsL;
    uint8_t wHubCharacteristicsH;
    uint8_t bPwrOn2PwrGood;
    uint8_t bHubContrCurrent;
    uint8_t DeviceRemovable;
    uint8_t PortPwrCtrlMask;
}USB_HUB_DESCR;

typedef struct __PACKED _USB_HID_DESCR {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdHID;
    uint8_t bCountryCode;
    uint8_t bNumDescriptors;
    uint8_t bDescriptorTypeX;
    uint8_t wDescriptorLengthL;
    uint8_t wDescriptorLengthH;
}USB_HID_DESCR;

typedef struct __PACKED _UDISK_BOC_CBW {/* command of BulkOnly USB-FlashDisk */
    uint32_t mCBW_Sig;
    uint32_t mCBW_Tag;
    uint32_t mCBW_DataLen;                /* uppest byte of data length, always is 0 */
    uint8_t mCBW_Flag;                    /* transfer direction and etc. */
    uint8_t mCBW_LUN;
    uint8_t mCBW_CB_Len;                  /* length of command block */
    uint8_t mCBW_CB_Buf[16];              /* command block buffer */
}UDISK_BOC_CBW, *PXUDISK_BOC_CBW;

typedef struct __PACKED _UDISK_BOC_CSW {/* status of BulkOnly USB-FlashDisk */
    uint32_t mCBW_Sig;
    uint32_t mCBW_Tag;
    uint32_t mCSW_Residue;                /* return: remainder bytes */         /* uppest byte of remainder length, always is 0 */
    uint8_t mCSW_Status;                  /* return: result status */
}UDISK_BOC_CSW;

/* USBHS PHY Clock Config (RCC_CFGR2) USBHS PHY*/
#ifndef  USBHS_EXIST
#define USB_48M_CLK_SRC_MASK   (1<<31)
#define USB_48M_CLK_SRC_SYS    (0<<31)
#define USB_48M_CLK_SRC_PHY    (1<<31)

#define USBHS_PLL_ALIVE        (1<<30)

#define USBHS_PLL_CKREF_MASK   (3<<28)
#define USBHS_PLL_CKREF_3M     (0<<28)
#define USBHS_PLL_CKREF_4M     (1<<28)
#define USBHS_PLL_CKREF_8M     (2<<28)
#define USBHS_PLL_CKREF_5M     (3<<28)

#define USBHS_PLL_SRC_MASK     (1<<27)
#define USBHS_PLL_SRC_HSE      (0<<27)
#define USBHS_PLL_SRC_HSI      (1<<27)

#define USBHS_PLL_SRC_PRE_MASK (7<<24)
#define USBHS_PLL_SRC_PRE_DIV1 (0<<24)
#define USBHS_PLL_SRC_PRE_DIV2 (1<<24)
#define USBHS_PLL_SRC_PRE_DIV3 (2<<24)
#define USBHS_PLL_SRC_PRE_DIV4 (3<<24)
#define USBHS_PLL_SRC_PRE_DIV5 (4<<24)
#define USBHS_PLL_SRC_PRE_DIV6 (5<<24)
#define USBHS_PLL_SRC_PRE_DIV7 (6<<24)

/*******************RCC_CFG2***********************/
#define USBHS_CLK_SRC_PHY     (1<<31)
#define USBHS_CLK_SRC_SYS     (0<<31)
#define USBHS_PLLALIVE        (1<<30)

#define USBHS_PLL_CKREF_3M    (0<<28)
#define USBHS_PLL_CKREF_4M    (1<<28)
#define USBHS_PLL_CKREF_8M    (2<<28)
#define USBHS_PLL_CKREF_5M    (3<<28)

#define USBHS_PLL_SRC_HSE      (0<<27)
#define USBHS_PLL_SRC_HSI      (1<<27)

#define USBHS_PLL_SRC_PRE_DIV1 (0<<24)
#define USBHS_PLL_SRC_PRE_DIV2 (1<<24)
#define USBHS_PLL_SRC_PRE_DIV3 (2<<24)
#define USBHS_PLL_SRC_PRE_DIV4 (3<<24)
#define USBHS_PLL_SRC_PRE_DIV5 (4<<24)
#define USBHS_PLL_SRC_PRE_DIV6 (5<<24)
#define USBHS_PLL_SRC_PRE_DIV7 (6<<24)
#define USBHS_PLL_SRC_PRE_DIV8 (7<<24)
/*******************RCC_AHBEBR***********************/
#define USBHS_CLK_EN        (1<<11)

/******************************************************************************/
/*                         USB HD Host Mode Peripheral Register map           */
/******************************************************************************/
// R8_USB_CTRL
#define DMA_EN                  (1<<0)
#define USB_ALL_CLR             (1<<1)
#define USB_FORCE_RST           (1<<2)
#define INT_BUSY_EN             (1<<3)
#define DEV_PU_EN               (1<<4)
#define FULL_SPEED              (0<<5)
#define HIGH_SPEED              (1<<5)
#define LOW_SPEED               (2<<5)
#define HOST_MODE               (1<<7)

//R8_UHOST_CTRL
#define SEND_BUS_RESET          (1<<0)
#define SEND_BUS_SUSPEND        (1<<1)
#define SEND_BUS_RESUME         (1<<2)
#define REMOTE_WAKE             (1<<3)
#define PHY_SUSPENDM            (1<<4)
#define UH_SOFT_FREE            (1<<6)
#define SEND_SOF_EN             (1<<7)

// R8_USB_INT_EN
#define USBHS_DETECT_EN          (1<<0)
#define USBHS_ACT_EN             (1<<1)
#define USBHS_SUSP_EN            (1<<2)
#define USBHS_SOF_EN             (1<<3)
#define USBHS_OVER_EN            (1<<4)
#define USBHS_SETUP_EN           (1<<5)
#define USBHS_ISO_EN             (1<<6)
#define USBHS_DEV_NAK_EN         (1<<7)

// USB SUSPENED
#define HOST_TESTMODE_MASK       (3<<0)
#define DEV_REMOTE_WAKEUP        (1<<2)
#define USB_LINESTATE            (3<<4)

// R8_USB_SPEED_TYPE
#define USBSPEED_MASK            (0x03)

// R8_USB_MIS_ST
#define USBHS_SPLIT_CAN          (1<<0)
#define USBHS_ATTCH              (1<<1)
#define USBHS_SUSPEND            (1<<2)
#define USBHS_BUS_RESET          (1<<3)
#define USBHS_FIFO_RDY           (1<<4)
#define USBHS_SIE_FREE           (1<<5)
#define USBHS_SOF_ACT            (1<<6)
#define USBHS_SOF_PRES           (1<<7)

// R8_USB_INT_FLAG
#define USBHS_DETECT_FLAG        (1<<0)
#define USBHS_ACT_FLAG           (1<<1)
#define USBHS_SUSP_FLAG          (1<<2)
#define USBHS_SOF_FLAG           (1<<3)
#define USBHS_OVER_FLAG          (1<<4)
#define USBHS_SETUP_FLAG         (1<<5)
#define USBHS_ISO_FLAG           (1<<6)

// R8_USUB_INT_ST
#define USBHS_TOGGLE_OK          (0x40)
#define USBHS_HOST_RES           (0x0f)

// R32_UH_EP_MOD
#define HOST_TX_EN           (1<<3)
#define HOST_RX_EN           (1<<18)

// HOST_EP_TYPE
#define USBHS_ENDP_TX_ISO     (1<<3)
#define USBHS_ENDP_RX_ISO     (1<<(16+2))

// R32_UH_EP_PID
#define HOST_MASK_TOKEN      (0x0f)
#define HOST_MASK_ENDP       (0x0f<<4)

//R8_UH_RX_CTRL
#define EP_R_RES_MASK        (3<<0)
#define EP_R_RES_ACK         (0<<0)
#define EP_R_RES_NYET        (1<<0)
#define EP_R_RES_NAK         (2<<0)
#define EP_R_RES_STALL       (3<<0)

#define UH_R_RES_NO          (1<<2)
#define UH_R_TOG_1           (1<<3)
#define UH_R_TOG_2           (2<<3)
#define UH_R_TOG_3           (3<<3)
#define UH_R_TOG_AUTO        (1<<5)
#define UH_R_DATA_NO         (1<<6)

//R8_UH_TX_CTRL
#define UH_T_RES_MASK        (3<<0)
#define UH_T_RES_ACK         (0<<0)
#define UH_T_RES_NYET        (1<<0)
#define UH_T_RES_NAK         (2<<0)
#define UH_T_RES_STALL       (3<<0)

#define UH_T_RES_NO          (1<<2)
#define UH_T_TOG_1           (1<<3)
#define UH_T_TOG_2           (2<<3)
#define UH_T_TOG_3           (3<<3)
#define UH_T_TOG_AUTO        (1<<5)
#define UH_T_DATA_NO         (1<<6)

#define DEV_TYPE_KEYBOARD     (USB_DEV_CLASS_HID | 0x20)
#define DEV_TYPE_MOUSE        (USB_DEV_CLASS_HID | 0x30)
#define DEF_AOA_DEVICE        0xF0
#define DEV_TYPE_UNKNOW       0xFF
/******************************************************/
 //endpoint
#define DEF_ENDP_0     0
#define DEF_ENDP_1     1
#define DEF_ENDP_2     2
#define DEF_ENDP_3     3
#define DEF_ENDP_4     4
#define DEF_ENDP_5     5
#define DEF_ENDP_6     6
#define DEF_ENDP_7     7
#define DEF_ENDP_8     8
#define DEF_ENDP_9     9
#define DEF_ENDP_10    10
#define DEF_ENDP_11    11
#define DEF_ENDP_12    12
#define DEF_ENDP_13    13
#define DEF_ENDP_14    14
#define DEF_ENDP_15    15

#define USBHS_MAX_PACK_SIZE     512
#define WAIT_TIME               1000000
#define WAIT_USB_TOUT_200US     3000

#ifndef ERR_SUCCESS
#define ERR_SUCCESS           (0x00)
#define ERR_USB_CONNECT       (0x15)
#define ERR_USB_DISCON        (0x16)
#define ERR_USB_BUF_OVER      (0x17)
#define ERR_USB_DISK_ERR      (0x1F)
#define ERR_USB_TRANSFER      (0x20)
#define ERR_USB_UNSUPPORT     (0xFB)
#define ERR_USB_UNKNOWN       (0xFE)
#define ERR_AOA_PROTOCOL      (0x41)
#endif

typedef struct
{
    uint16_t  OutEndpMaxSize;
    uint16_t  InEndpMaxSize;          // IN
    uint8_t   InEndpNum;              // IN
    uint8_t   Intog;                  // IN
    uint8_t   InEndpCount;            // IN
    uint8_t   OutEndpNum;             // OUT
    uint8_t   Outtog;                 // OUT
    uint8_t   OutEndpCount;           // OUT
}DEVENDP;

typedef struct  __attribute__((packed))  _DEV_INFO
{
     DEVENDP DevEndp;
     uint8_t   DeviceStatus;           //
     uint8_t   DeviceAddress;          //
     uint8_t   DeviceSpeed;            //
     uint8_t   DeviceType;             // 0x30-HID  0x31-KEYBOARD  0x32-MOUSE
     uint8_t   DeviceEndp0Size;        // USB0
     uint8_t   DeviceCongValue;
 }USBDEV_INFO;

 #define pSetupReq ((USB_SETUP_REQ*)endpTXbuf)

 /*********************************************************/
 void USB_RCC_Init(void);
 void USB_HostInit(FunctionalState sta);
 void USB_CurrentAddress(uint8_t address);
 void USB_SetBusReset(void);

 uint8_t USBHS_HostEnum();
 uint8_t CtrlGetDevDescr();
 uint8_t CtrlGetConfigDescr();
 uint8_t CtrlSetAddress(uint8_t addr);
 uint8_t CtrlSetUsbConfig(uint8_t cfg_val);
 uint8_t CtrlClearEndpStall(uint8_t endp);
 uint8_t CtrlSetUsbIntercace(uint8_t cfg);

 void AnalyseDescriptor(USBDEV_INFO* pusbdev, uint8_t* pdesc, uint16_t len);

 uint8_t HubGetPortStatus(uint8_t HubPortIndex);
 uint8_t HubSetPortFeature(uint8_t HubPortIndex, uint8_t FeatureSelt);
 uint8_t HubClearPortFeature(uint8_t HubPortIndex, uint8_t FeatureSelt);

 uint8_t HostCtrlTransfer(uint8_t* databuf, uint8_t* len);
 uint8_t USBHostTransact(uint8_t endp_pid, uint8_t toggle, uint32_t timeout);
#ifdef __cplusplus
}
#endif

#endif  
#endif
