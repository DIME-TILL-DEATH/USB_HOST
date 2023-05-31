#ifndef __CH32V30x_USBHS_HOST_H
#define __CH32V30x_USBHS_HOST_H

#include <string.h>

#include "debug.h"
#include "usb_defines.h"

#ifdef __cplusplus
 extern "C" {
#endif

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
    uint8_t   InType;                  // IN
    uint8_t   InEndpCount;            // IN
    uint8_t   OutEndpNum;             // OUT
    uint8_t   OutType;                 // OUT
    uint8_t   OutEndpCount;           // OUT
}USBDEV_ENDP;

typedef struct  __attribute__((packed))  _DEV_INFO
{
     USBDEV_ENDP DevEndp;
     uint8_t   DeviceStatus;           //
     uint8_t   DeviceAddress;          //
     uint8_t   DeviceSpeed;            //
     uint8_t   DeviceClass;             // 0x20-CDC 0x30-HID  0x31-KEYBOARD  0x32-MOUSE
     uint8_t   DeviceEndp0Size;        // USB0
     uint8_t   DeviceCfgValue;

   //  USB_DEV_DESCR  deviceDescriptor;
   //  USB_CFG_DESCR  cfgDescriptor;
}USBDEV_INFO;

 /*********************************************************/
void USB_HostInit(FunctionalState sta);
uint8_t USB_HostEnum();

uint8_t USB_CtrlGetDevDescr(USB_DEV_DESCR* devDescriptor);
uint8_t USB_CtrlGetConfigDescr();
uint8_t USB_CtrlSetAddress(uint8_t addr);
uint8_t USB_CtrlSetUsbConfig(uint8_t cfg_val);
uint8_t USB_CtrlClearEndpStall(uint8_t endp);
uint8_t USB_CtrlSetUsbIntercace(uint8_t cfg);
uint8_t USB_CtrlGetStringDescr(uint8_t* resBuf, uint16_t* resLen, uint16_t indexId, uint8_t languageIndex);

uint8_t USB_HubGetPortStatus(uint8_t HubPortIndex);
uint8_t USB_HubSetPortFeature(uint8_t HubPortIndex, uint8_t FeatureSelt);
uint8_t USB_HubClearPortFeature(uint8_t HubPortIndex, uint8_t FeatureSelt);

void USB_AnalyseCfgDescriptor(USBDEV_INFO* pusbdev, uint8_t* pdesc, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif  
#endif
