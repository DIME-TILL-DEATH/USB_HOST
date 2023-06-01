#ifndef USB_USB_DEVICE_CLASSES_H_
#define USB_USB_DEVICE_CLASSES_H_

#define USB_CLASS_DEVICE    0x00 // Use class information in the Interface Descriptors
#define USB_CLASS_AUDIO     0x01 // Audio
#define USB_CLASS_CDC       0x02 // Communications and Communications Device Class (CDC) Control
#define USB_CLASS_HID       0x03 // Human Interface Device (HID)
#define USB_CLASS_PHYSICAL  0x05 // Physical
#define USB_CLASS_IMAGE     0x06 // Image
#define USB_CLASS_PRINTER   0x07 // Printer
#define USB_CLASS_MSD       0x08 // Mass Storage (MSD)
#define USB_CLASS_HUB       0x09 // Hub
#define USB_CLASS_CDC_DATA 0x0A // CDC-Data
#define USB_CLASS_SCARD     0x0B // Smart Card
#define USB_CLASS_CSEC      0x0D // Content Security
#define USB_CLASS_VIDEO     0x0E // Video
#define USB_CLASS_HEALTH    0x0F // Personal Healthcare
#define USB_CLASS_AUD_VID   0x10 // Audio/Video Devices
#define USB_CLASS_BILLBOARD 0x11 // Billboard Device Class
#define USB_CLASS_DIAGN     0xDC // Diagnostic Device
#define USB_CLASS_WIRELESS  0x0E // Wireless Controller
#define USB_CLASS_MISC      0xEF // Miscellaneous
#define USB_CLASS_APPSCPEC  0xFE // Application Specific
#define USB_CLASS_VENDOR    0xFF // Vendor Specific

#endif /* USB_USB_DEVICE_CLASSES_H_ */
