#include <stm32f0xx_hal.h>
#include <assert.h>
#include "usbd_ctlreq.h"
#include "usbd_ioreq.h"
#include "usbd_conf.h"
#include "usbd_helper.h"
#include "pcan_protocol.h"
#include "pcan_led.h"
#include "pcan_usb.h"
#include "pcan_identity.h"
#if PCAN_USE_PLIN
#include "plin_protocol.h"
#endif

USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_DescriptorsTypeDef FS_Desc;
static struct t_class_data pcan_data = { 0 };

/* PCAN-USB Pro FD 复合设备：MI_00=CAN（6 端点），MI_01=LIN（6 端点）
 * PCAN-USB 经典设备：仅 MI_00=CAN（4 端点），不描述 LIN 接口 */
struct t_peak_description
{
  USB_CONFIGURATION_DESCRIPTOR con0;
  USB_INTERFACE_DESCRIPTOR     if0;
  USB_ENDPOINT_DESCRIPTOR      if0_ep[PCAN_CAN_EP_COUNT];
#if PCAN_USE_PLIN
  USB_INTERFACE_DESCRIPTOR     if1;
  USB_ENDPOINT_DESCRIPTOR      if1_ep[6];
#endif
};

__ALIGN_BEGIN static const USB_DEVICE_QUALIFIER_DESCRIPTOR dev_qua __ALIGN_END = 
{
  .bLength            = sizeof( USB_DEVICE_QUALIFIER_DESCRIPTOR ),
  .bDescriptorType    = USB_QUALIFIER_DESCRIPTOR_TYPE,
  .bcdUSB             = 0x0200, /* 2.0 */
  .bDeviceClass       = 0,
  .bDeviceSubClass    = 0,
  .bDeviceProtocol    = 0,
  .bMaxPacketSize0    = 64,
  .bNumConfigurations = 1,
  .bReserved          = 0,
};

__ALIGN_BEGIN static struct t_peak_description peak_usb_dev __ALIGN_END = 
{
  .con0 =
  {
    .bLength              = sizeof( USB_CONFIGURATION_DESCRIPTOR ),
    .bDescriptorType      = USB_CONFIGURATION_DESCRIPTOR_TYPE,
    .wTotalLength         = sizeof( struct t_peak_description ), 
    .bNumInterfaces       = PCAN_USB_IFACE_COUNT,
    .bConfigurationValue  = 1,
    .iConfiguration       = 4,
    .bmAttributes         = USB_CONFIG_BUS_POWERED,
    .MaxPower             = 120, /* = 240mA */
  },
  .if0 =
  {
    .bLength              = sizeof( USB_INTERFACE_DESCRIPTOR ),
    .bDescriptorType      = USB_INTERFACE_DESCRIPTOR_TYPE,
    .bInterfaceNumber     = 0,
    .bAlternateSetting    = 0,
    .bNumEndpoints        = 6,
    .bInterfaceClass      = 0,
    .bInterfaceSubClass   = 0,
    .bInterfaceProtocol   = 0,
    .iInterface           = 0,
  },
  .if0_ep =
  {
    {
      .bLength            = sizeof( USB_ENDPOINT_DESCRIPTOR ),
      .bDescriptorType    = USB_ENDPOINT_DESCRIPTOR_TYPE,
      .bEndpointAddress   = PCAN_USB_EP_CMDIN,
      .bmAttributes       = USB_ENDPOINT_TYPE_BULK,
      .wMaxPacketSize     = PCAN_USB_EP_SIZE,
      .bInterval          = 0,
    },
    {
      .bLength            = sizeof( USB_ENDPOINT_DESCRIPTOR ),
      .bDescriptorType    = USB_ENDPOINT_DESCRIPTOR_TYPE,
      .bEndpointAddress   = PCAN_USB_EP_CMDOUT,
      .bmAttributes       = USB_ENDPOINT_TYPE_BULK,
      .wMaxPacketSize     = PCAN_USB_EP_SIZE,
      .bInterval          = 0,
    },
    {
      .bLength            = sizeof( USB_ENDPOINT_DESCRIPTOR ),
      .bDescriptorType    = USB_ENDPOINT_DESCRIPTOR_TYPE,
      .bEndpointAddress   = PCAN_USB_EP_MSGIN,
      .bmAttributes       = USB_ENDPOINT_TYPE_BULK,
      .wMaxPacketSize     = PCAN_USB_EP_SIZE,
      .bInterval          = 0,
    },
    {
      .bLength            = sizeof( USB_ENDPOINT_DESCRIPTOR ),
      .bDescriptorType    = USB_ENDPOINT_DESCRIPTOR_TYPE,
      .bEndpointAddress   = PCAN_USB_EP_MSGOUT,
      .bmAttributes       = USB_ENDPOINT_TYPE_BULK,
      .wMaxPacketSize     = PCAN_USB_EP_SIZE,
      .bInterval          = 0,
    },
    {
      .bLength            = sizeof( USB_ENDPOINT_DESCRIPTOR ),
      .bDescriptorType    = USB_ENDPOINT_DESCRIPTOR_TYPE,
      .bEndpointAddress   = PCAN_USB_EP_MSG1IN,
      .bmAttributes       = USB_ENDPOINT_TYPE_BULK,
      .wMaxPacketSize     = PCAN_USB_EP_SIZE,
      .bInterval          = 0,
    },
    {
      .bLength            = sizeof( USB_ENDPOINT_DESCRIPTOR ),
      .bDescriptorType    = USB_ENDPOINT_DESCRIPTOR_TYPE,
      .bEndpointAddress   = PCAN_USB_EP_MSG1OUT,
      .bmAttributes       = USB_ENDPOINT_TYPE_BULK,
      .wMaxPacketSize     = PCAN_USB_EP_SIZE,
      .bInterval          = 0,
    },
  },
#if PCAN_USE_PLIN
  .if1 =
  {
    .bLength              = sizeof( USB_INTERFACE_DESCRIPTOR ),
    .bDescriptorType      = USB_INTERFACE_DESCRIPTOR_TYPE,
    .bInterfaceNumber     = 1,
    .bAlternateSetting    = 0,
    .bNumEndpoints        = 6,
    .bInterfaceClass      = 0,
    .bInterfaceSubClass   = 0,
    .bInterfaceProtocol   = 0,
    .iInterface           = 0,
  },
  .if1_ep =
  {
    {
      .bLength            = sizeof( USB_ENDPOINT_DESCRIPTOR ),
      .bDescriptorType    = USB_ENDPOINT_DESCRIPTOR_TYPE,
      .bEndpointAddress   = PLIN_EP_CMDIN,
      .bmAttributes       = USB_ENDPOINT_TYPE_BULK,
      .wMaxPacketSize     = PCAN_USB_EP_SIZE,
      .bInterval          = 0,
    },
    {
      .bLength            = sizeof( USB_ENDPOINT_DESCRIPTOR ),
      .bDescriptorType    = USB_ENDPOINT_DESCRIPTOR_TYPE,
      .bEndpointAddress   = PLIN_EP_CMDOUT,
      .bmAttributes       = USB_ENDPOINT_TYPE_BULK,
      .wMaxPacketSize     = PCAN_USB_EP_SIZE,
      .bInterval          = 0,
    },
    {
      .bLength            = sizeof( USB_ENDPOINT_DESCRIPTOR ),
      .bDescriptorType    = USB_ENDPOINT_DESCRIPTOR_TYPE,
      .bEndpointAddress   = PLIN_EP_DATA0IN,
      .bmAttributes       = USB_ENDPOINT_TYPE_BULK,
      .wMaxPacketSize     = PCAN_USB_EP_SIZE,
      .bInterval          = 0,
    },
    {
      .bLength            = sizeof( USB_ENDPOINT_DESCRIPTOR ),
      .bDescriptorType    = USB_ENDPOINT_DESCRIPTOR_TYPE,
      .bEndpointAddress   = PLIN_EP_DATA0OUT,
      .bmAttributes       = USB_ENDPOINT_TYPE_BULK,
      .wMaxPacketSize     = PCAN_USB_EP_SIZE,
      .bInterval          = 0,
    },
    {
      .bLength            = sizeof( USB_ENDPOINT_DESCRIPTOR ),
      .bDescriptorType    = USB_ENDPOINT_DESCRIPTOR_TYPE,
      .bEndpointAddress   = PLIN_EP_DATA1IN,
      .bmAttributes       = USB_ENDPOINT_TYPE_BULK,
      .wMaxPacketSize     = PCAN_USB_EP_SIZE,
      .bInterval          = 0,
    },
    {
      .bLength            = sizeof( USB_ENDPOINT_DESCRIPTOR ),
      .bDescriptorType    = USB_ENDPOINT_DESCRIPTOR_TYPE,
      .bEndpointAddress   = PLIN_EP_DATA1OUT,
      .bmAttributes       = USB_ENDPOINT_TYPE_BULK,
      .wMaxPacketSize     = PCAN_USB_EP_SIZE,
      .bInterval          = 0,
    },
  },
#endif
};

static uint8_t *peak_ep_out_buffer( uint8_t epnum )
{
  switch( epnum )
  {
    case PCAN_USB_EP_CMDOUT: return pcan_data.buffer_cmd;
    case PCAN_USB_EP_MSGOUT: return pcan_data.buffer_data;
    case PCAN_USB_EP_MSG1OUT: return pcan_data.buffer_data1;
#if PCAN_USE_PLIN
    case PLIN_EP_CMDOUT: return pcan_data.buffer_lin_cmd;
    case PLIN_EP_DATA0OUT: return pcan_data.buffer_lin_data0;
    case PLIN_EP_DATA1OUT: return pcan_data.buffer_lin_data1;
#endif
    default: return 0;
  }
}

static uint16_t peak_ep_out_size( uint8_t epnum )
{
#if PCAN_USE_PLIN
  /* LIN 数据 OUT 只承载 36 字节消息，PMA 缓冲按 40 字节分配 */
  if( epnum == PLIN_EP_DATA0OUT || epnum == PLIN_EP_DATA1OUT )
    return PLIN_RX_MSG_SIZE;
#endif
  return PCAN_USB_EP_SIZE;
}

static void device_ep_list_set( USBD_HandleTypeDef *pdev,
                                const USB_ENDPOINT_DESCRIPTOR *eps,
                                uint8_t count, uint8_t open )
{
  for( uint8_t i = 0; i < count; i++ )
  {
    uint8_t ep_addr = eps[i].bEndpointAddress;

    if( open )
    {
      USBD_LL_OpenEP( pdev, ep_addr, eps[i].bmAttributes, eps[i].wMaxPacketSize );
      if( ( ep_addr & 0x80 ) != 0 )
        pdev->ep_in[ep_addr & EP_ADDR_MSK].is_used = 1;
      else
        pdev->ep_out[ep_addr & EP_ADDR_MSK].is_used = 1;
    }
    else
    {
      USBD_LL_CloseEP( pdev, ep_addr );
      if( ( ep_addr & 0x80 ) != 0 )
        pdev->ep_in[ep_addr & EP_ADDR_MSK].is_used = 0;
      else
        pdev->ep_out[ep_addr & EP_ADDR_MSK].is_used = 0;
    }
  }
}

static uint8_t device_init( USBD_HandleTypeDef *pdev, uint8_t cfgidx )
{
  UNUSED( cfgidx );

  device_ep_list_set( pdev, peak_usb_dev.if0_ep, PCAN_CAN_EP_COUNT, 1 );
#if PCAN_USE_PLIN
  device_ep_list_set( pdev, peak_usb_dev.if1_ep, 6, 1 );
#endif

  pdev->pClassData = (void*)&pcan_data;

  /* 只接收实际使用的 OUT 端点：CAN 命令/数据、LIN 命令/数据。
   * EP3 OUT 不使用，避免共享 PMA 缓冲被意外写入。 */
  {
    static const uint8_t out_eps[] =
    {
      PCAN_USB_EP_CMDOUT,
      PCAN_USB_EP_MSGOUT,
#if PCAN_USE_PLIN
      PLIN_EP_CMDOUT,
      PLIN_EP_DATA0OUT,
      PLIN_EP_DATA1OUT,
#endif
    };

    for( uint8_t i = 0; i < sizeof( out_eps ); i++ )
    {
      USBD_LL_PrepareReceive( pdev, out_eps[i],
                              peak_ep_out_buffer( out_eps[i] ),
                              peak_ep_out_size( out_eps[i] ) );
    }
  }

  return USBD_OK;
}

static uint8_t device_deinit( USBD_HandleTypeDef *pdev, uint8_t cfgidx )
{
  UNUSED( cfgidx );

  device_ep_list_set( pdev, peak_usb_dev.if0_ep, PCAN_CAN_EP_COUNT, 0 );
#if PCAN_USE_PLIN
  device_ep_list_set( pdev, peak_usb_dev.if1_ep, 6, 0 );
#endif

  pdev->pClassData = (void*)0;
  return USBD_OK;
}

uint16_t pcan_usb_send_command_buffer( const void *p, uint16_t size )
{
  USBD_HandleTypeDef *pdev = &hUsbDeviceFS;

  if( pdev->ep_in[PCAN_USB_EP_CMDIN & 0xFU].total_length )
    return 0;
  pdev->ep_in[PCAN_USB_EP_CMDIN & 0xFU].total_length = size;
  
  if( USBD_LL_Transmit( pdev, PCAN_USB_EP_CMDIN, (void*)p, size ) == USBD_OK )
    return size;
  return 0;
}

uint16_t pcan_usb_send_data_buffer( const void *p, uint16_t size )
{
  USBD_HandleTypeDef *pdev = &hUsbDeviceFS;

  if( pdev->ep_in[PCAN_USB_EP_MSGIN & 0xFU].total_length )
    return 0;
  pdev->ep_in[PCAN_USB_EP_MSGIN & 0xFU].total_length = size;
  
  if( USBD_LL_Transmit( pdev, PCAN_USB_EP_MSGIN, (void*)p, size ) == USBD_OK )
    return size;
  return 0;
}

void pcan_usb_poll( void )
{
  HAL_PCD_IRQHandler( &hpcd_USB_FS );
}

static uint8_t  device_setup( USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req )
{
  switch( req->bmRequest & USB_REQ_TYPE_MASK )
  {
    case USB_REQ_TYPE_VENDOR:
      /* PCAN 引导信息查询 */
      if( req->bRequest == 0 && req->wValue == 0 )
      {
        static const uint8_t bootloader_info[] = { 
                                0x00, 0x00, 0x08, 0x04, 0x00, 0x08, 0x07, 0x00,
                                0x04, 0x02, 0xe0, 0x07, 0x01, 0x00, 0x00, 0x00
                              };
        USBD_CtlSendData( pdev, (void*)bootloader_info, sizeof( bootloader_info ) );

        pcan_led_set_mode( LED_CH0_RX, LED_MODE_ON, 0 );
        pcan_led_set_mode( LED_CH0_TX, LED_MODE_ON, 0 );

        /* reset endpoints */
        pcan_flush_ep( PCAN_USB_EP_MSGIN );
        pcan_flush_ep( PCAN_USB_EP_CMDIN );
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch( req->bRequest )
      {
        case USB_REQ_GET_INTERFACE:
        {
          static uint8_t alt_setting = 0;
          USBD_CtlSendData( pdev, &alt_setting, 1 );
          break;
        }

        case USB_REQ_SET_INTERFACE:
          break;

        case USB_REQ_GET_STATUS:
        {
          static uint8_t itf_status[2] = { 0, 0 };
          USBD_CtlSendData( pdev, itf_status, 2 );
          break;
        }

        default:
          break;
      }
      break;

    default:
      break;
  }
  return USBD_OK;
}

static uint8_t device_ep0_rx_ready( USBD_HandleTypeDef *pdev )
{
  UNUSED( pdev );
  return USBD_OK;
}

/* data was sent to PC */
static uint8_t device_data_in( USBD_HandleTypeDef *pdev, uint8_t epnum )
{
  struct t_class_data *p_data = (void*)pdev->pClassData;
  
  if( pdev->pClassData == 0 )
    return USBD_FAIL;

  pdev->ep_in[epnum].total_length = 0U;
  p_data->ep_tx_in_use[epnum] = 0;

#if PCAN_USE_PLIN
  if( epnum == ( PLIN_EP_DATA0IN & 0x0F ) )
    plin_protocol_on_tx_done( 0 );
  else if( epnum == ( PLIN_EP_DATA1IN & 0x0F ) )
    plin_protocol_on_tx_done( 1 );
#endif

  return USBD_OK;
}

/* data was received from PC */
static uint8_t device_data_out( USBD_HandleTypeDef *pdev, uint8_t epnum )
{
  int size;
  uint8_t *buffer;
   
  if( pdev->pClassData == 0 )
    return USBD_FAIL;
  
  size = USBD_LL_GetRxDataSize( pdev, epnum );

  switch( epnum )
  {
    case PCAN_USB_EP_CMDOUT:
      pcan_protocol_process_command( pcan_data.buffer_cmd, (uint16_t)size );
      break;
    case PCAN_USB_EP_MSGOUT:
      pcan_protocol_process_data( pcan_data.buffer_data, (uint16_t)size );
      break;
#if PCAN_USE_PLIN
    case PLIN_EP_CMDOUT:
      plin_protocol_process_command( pcan_data.buffer_lin_cmd, (uint16_t)size );
      break;
    case PLIN_EP_DATA0OUT:
      plin_protocol_process_data( 0, pcan_data.buffer_lin_data0, (uint16_t)size );
      break;
    case PLIN_EP_DATA1OUT:
      plin_protocol_process_data( 1, pcan_data.buffer_lin_data1, (uint16_t)size );
      break;
#endif
    default:
      /* 0x03 预留数据通道，丢弃 */
      break;
  }

  buffer = peak_ep_out_buffer( epnum );
  if( buffer == 0 )
    return USBD_FAIL;

  USBD_LL_PrepareReceive( pdev, epnum, buffer, peak_ep_out_size( epnum ) );
  return USBD_OK;
}

static uint8_t *device_get_hs_cfg( uint16_t *length )
{
  *length = sizeof( struct t_peak_description );
  return (void*)&peak_usb_dev;
}

static uint8_t *device_get_fs_cfg( uint16_t *length )
{
  *length = sizeof( struct t_peak_description );
  return (void*)&peak_usb_dev;
}

static uint8_t *device_get_other_speed_cfg( uint16_t *length )
{
  *length = sizeof( struct t_peak_description );
  return (void*)&peak_usb_dev;
}

static uint8_t *device_get_device_qualifier( uint16_t *length )
{
  *length = sizeof( USB_DEVICE_QUALIFIER_DESCRIPTOR );
  
  return (void*)&dev_qua;
}


static uint8_t *device_get_user_string( USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length )
{
  UNUSED( pdev );
  if( index == 10 )
  {
    __ALIGN_BEGIN static const uint16_t vendor_descriptor[1+24] __ALIGN_END = 
    {
      0x0332,
      ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '
    };
    *length = sizeof( vendor_descriptor );
    return (uint8_t*)vendor_descriptor;
  }
  return 0;
}

USBD_ClassTypeDef usbd_pcan =
{
  .Init = device_init,
  .DeInit = device_deinit,
  .Setup = device_setup,
  .EP0_TxSent = 0,
  .EP0_RxReady = device_ep0_rx_ready,
  .DataIn = device_data_in,
  .DataOut = device_data_out,
  .SOF = 0,
  .IsoINIncomplete = 0,
  .IsoOUTIncomplete = 0,
  .GetHSConfigDescriptor = device_get_hs_cfg,
  .GetFSConfigDescriptor = device_get_fs_cfg,
  .GetOtherSpeedConfigDescriptor = device_get_other_speed_cfg,
  .GetDeviceQualifierDescriptor = device_get_device_qualifier,
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  .GetUsrStrDescriptor = device_get_user_string,
#endif
};

void pcan_usb_init( void )
{
  GPIO_InitTypeDef gpio = { 0 };

  /* drive D+ low for a while so the host detects a disconnect and re-enumerates
     (some boards keep D+ pulled up externally) */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin   = GPIO_PIN_12;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init( GPIOA, &gpio );
  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_12, GPIO_PIN_RESET );
  HAL_Delay( 300 );
  gpio.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init( GPIOA, &gpio );

  if( USBD_Init( &hUsbDeviceFS, &FS_Desc, DEVICE_FS ) != USBD_OK )
  {
    assert( 0 );
  }

  if( USBD_RegisterClass( &hUsbDeviceFS, &usbd_pcan ) != USBD_OK )
  {
    assert( 0 );
  }

  if( USBD_Start(&hUsbDeviceFS) != USBD_OK )
  {
    assert( 0 );
  }
}

int pcan_flush_ep( uint8_t ep )
{
  USBD_HandleTypeDef *pdev = &hUsbDeviceFS;
  struct t_class_data *p_data = (void*)pdev->pClassData;

  p_data->ep_tx_in_use[ep&0x0F] = 0;
  return USBD_LL_FlushEP( pdev, ep ) == USBD_OK;
}

int pcan_flush_data( struct t_m2h_fsm *pfsm, void *src, int size )
{
  USBD_HandleTypeDef *pdev = &hUsbDeviceFS;
  struct t_class_data *p_data = (void*)pdev->pClassData;

  if( !p_data )
    return 0;

  switch( pfsm->state )
  {
    case 1:
      if( p_data->ep_tx_in_use[pfsm->ep_addr&0x0F] )
      {
        p_data->ep_tx_data_pending[pfsm->ep_addr&0x0F] = size;
        return 0;
      }
      pfsm->state = 0;
      /* fall through */
    case 0:
      assert( p_data->ep_tx_in_use[pfsm->ep_addr&0x0F] == 0 );
      if( size > pfsm->dbsize )
        break;
      memcpy( pfsm->pdbuf, src, size );
      p_data->ep_tx_in_use[pfsm->ep_addr&0x0F] = 1;
      /* prepare data transmit */
      pdev->ep_in[pfsm->ep_addr & EP_ADDR_MSK].total_length = size;
      /* no pending data */
      p_data->ep_tx_data_pending[pfsm->ep_addr&0x0F] = 0;
      USBD_LL_Transmit( pdev, pfsm->ep_addr, pfsm->pdbuf, size );
      
      pfsm->total_tx += size;
      pfsm->state = 1;
      return 1;
  }
  
  return 0;
}
