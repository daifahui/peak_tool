#include "usbd_core.h"
#include "usbd_conf.h"
#include "pcan_identity.h"

/* PEAK 设备身份由 pcan_identity.h 的编译分支决定 */
#define USBD_VID                    PCAN_USB_VID
#define USBD_PID_FS                 PCAN_USB_PID
#define USBD_LANGID_STRING          1033
#define USBD_MAX_STR_DESC_SIZ       0x100U

uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
  0x12,                       /*bLength */
  USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
  0x00,                       /*bcdUSB = 2.00 */
  0x02,
  0x00,                       /*bDeviceClass*/
  0x00,                       /*bDeviceSubClass*/
  0x00,                       /*bDeviceProtocol*/
  USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
  LOBYTE(USBD_VID),           /*idVendor*/
  HIBYTE(USBD_VID),           /*idVendor*/
  LOBYTE(USBD_PID_FS),        /*idProduct*/
  HIBYTE(USBD_PID_FS),        /*idProduct*/
  0x00,                       /*bcdDevice = 0.00 */
  0x00,
  1,                          /*Index of manufacturer  string*/
  2,                          /*Index of product string*/
  3,                          /*Index of serial number string*/
  USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
};

__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
     USB_LEN_LANGID_STR_DESC,
     USB_DESC_TYPE_STRING,
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING)
};

/* "PEAK-System" */
__ALIGN_BEGIN static const uint8_t usbd_mfc_desc[] __ALIGN_END =
{
  0x18, USB_DESC_TYPE_STRING,
  'P', 0, 'E', 0, 'A', 0, 'K', 0, '-', 0, 'S', 0, 'y', 0, 's', 0, 't', 0, 'e', 0, 'm', 0
};

#if PCAN_USE_PLIN
/* "PCAN-USB Pro FD" */
__ALIGN_BEGIN static const uint8_t usbd_product_desc[] __ALIGN_END =
{
  0x20, USB_DESC_TYPE_STRING,
  'P', 0, 'C', 0, 'A', 0, 'N', 0, '-', 0, 'U', 0, 'S', 0, 'B', 0,
  ' ', 0, 'P', 0, 'r', 0, 'o', 0, ' ', 0, 'F', 0, 'D', 0
};
#else
/* "PCAN-USB" */
__ALIGN_BEGIN static const uint8_t usbd_product_desc[] __ALIGN_END =
{
  0x12, USB_DESC_TYPE_STRING,
  'P', 0, 'C', 0, 'A', 0, 'N', 0, '-', 0, 'U', 0, 'S', 0, 'B', 0
};
#endif

/* "00000001" */
__ALIGN_BEGIN static const uint8_t usbd_serial_desc[] __ALIGN_END =
{
  0x12, USB_DESC_TYPE_STRING,
  '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '1', 0
};

/* "PEAK Tool" */
__ALIGN_BEGIN static const uint8_t usbd_config_desc[] __ALIGN_END =
{
  0x14, USB_DESC_TYPE_STRING,
  'P', 0, 'E', 0, 'A', 0, 'K', 0, ' ', 0, 'T', 0, 'o', 0, 'o', 0, 'l', 0
};

uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_FS_DeviceDesc);
  return USBD_FS_DeviceDesc;
}

uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(usbd_mfc_desc);
  return (uint8_t*)usbd_mfc_desc;
}

uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(usbd_product_desc);
  return (uint8_t*)usbd_product_desc;
}

uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(usbd_serial_desc);
  return (uint8_t*)usbd_serial_desc;
}

uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(usbd_config_desc);
  return (uint8_t*)usbd_config_desc;
}

USBD_DescriptorsTypeDef FS_Desc =
{
  .GetDeviceDescriptor = USBD_FS_DeviceDescriptor,
  .GetLangIDStrDescriptor = USBD_FS_LangIDStrDescriptor,
  .GetManufacturerStrDescriptor = USBD_FS_ManufacturerStrDescriptor,
  .GetProductStrDescriptor = USBD_FS_ProductStrDescriptor,
  .GetSerialStrDescriptor = USBD_FS_SerialStrDescriptor,
  .GetConfigurationStrDescriptor = USBD_FS_ConfigStrDescriptor,
  .GetInterfaceStrDescriptor = 0,
};
