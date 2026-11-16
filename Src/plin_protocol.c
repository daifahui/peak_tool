#include <string.h>
#include "plin_protocol.h"
#include "plin_lin.h"
#include "pcan_usb.h"
#include "pcan_led.h"
#include "usbd_core.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

/* PLIN-USB 命令字（来自 USBPcap 实测） */
enum
{
  PLIN_CMD_INITIALIZE_HW     = 0x00,
  PLIN_CMD_RST_HW_CONFIG     = 0x01,
  PLIN_CMD_SET_FRAME_ENTRY   = 0x02,
  PLIN_CMD_GET_FRAME_ENTRY   = 0x03,
  PLIN_CMD_GET_BAUDRATE      = 0x05,
  PLIN_CMD_SET_ID_FILTER     = 0x06,
  PLIN_CMD_GET_ID_FILTER     = 0x07,
  PLIN_CMD_GET_MASTER_MODE   = 0x08,
  PLIN_CMD_GET_HW_NAME       = 0x0A,
  PLIN_CMD_IDENTIFY_HW       = 0x0B,
  PLIN_CMD_GET_FW_VERSION    = 0x0C,
  PLIN_CMD_ADD_SLOT          = 0x10,
  PLIN_CMD_DELETE_SCHEDULE   = 0x11,
  PLIN_CMD_GET_SLOT_COUNT    = 0x12,
  PLIN_CMD_START_SCHEDULE    = 0x15,
  PLIN_CMD_RESUME_SCHEDULE   = 0x16,
  PLIN_CMD_SUSPEND_SCHEDULE  = 0x17,
  PLIN_CMD_GET_STATUS        = 0x18,
  PLIN_CMD_RST_HW            = 0x1E,
  PLIN_CMD_UPDATE_BYTE_ARRAY = 0x1F,
  PLIN_CMD_XMT_WAKE_UP       = 0x21,
  PLIN_CMD_GET_SERIAL_NUMBER = 0x23,
  PLIN_CMD_GET_HARDWARE_TYPE = 0x24,
  PLIN_CMD_GET_TARGET_TIME   = 0x26,
  PLIN_CMD_RESPONSE_REMAP    = 0x27,
  PLIN_CMD_BREAK_LEN         = 0x29,
  PLIN_CMD_TERMINATION       = 0x2A,
};

enum
{
  PLIN_MODE_NONE   = 0,
  PLIN_MODE_SLAVE  = 1,
  PLIN_MODE_MASTER = 2,
};

#define PLIN_MAX_FRAMES     64
#define PLIN_MAX_SLOTS      8
#define PLIN_RX_QUEUE_LEN   8
#define PLIN_FRAME_ENTRY_SIZE 16

struct plin_frame_entry
{
  uint8_t raw[PLIN_FRAME_ENTRY_SIZE];
};

struct plin_channel
{
  uint8_t  mode;
  uint16_t baudrate;
  uint8_t  filter[8];

  uint8_t  frame_valid[PLIN_MAX_FRAMES];
  struct plin_frame_entry frames[PLIN_MAX_FRAMES];

  uint8_t  slot_count;
  uint8_t  schedule_active;
  uint8_t  slots[PLIN_MAX_SLOTS][PLIN_FRAME_ENTRY_SIZE];

  uint16_t rx_overrun;
  uint8_t  rx_head;
  uint8_t  rx_tail;
  uint8_t  rx_count;
  uint8_t  rx_queue[PLIN_RX_QUEUE_LEN][PLIN_RX_MSG_SIZE];
};

static struct plin_channel plin_channels[PLIN_CHANNELS];
static uint8_t plin_cmd_tx[PLIN_EP_SIZE];
static uint8_t plin_rx_tx[PLIN_CHANNELS][PLIN_RX_MSG_SIZE];

static void plin_send_raw( const uint8_t *data, uint16_t size )
{
  USBD_HandleTypeDef *pdev = &hUsbDeviceFS;

  if( pdev->ep_in[PLIN_EP_CMDIN & 0x0F].total_length )
    return;
  if( size > sizeof( plin_cmd_tx ) )
    size = sizeof( plin_cmd_tx );

  memcpy( plin_cmd_tx, data, size );
  pdev->ep_in[PLIN_EP_CMDIN & 0x0F].total_length = size;
  USBD_LL_Transmit( pdev, PLIN_EP_CMDIN, plin_cmd_tx, size );
}

/* 响应头：[cmd][0x01][channel][client]，后跟 payload */
static void plin_reply( const uint8_t *req, const uint8_t *payload, uint16_t size )
{
  uint8_t rsp[PLIN_EP_SIZE];

  if( size > ( sizeof( rsp ) - 4 ) )
    size = sizeof( rsp ) - 4;

  rsp[0] = req[0];
  rsp[1] = 0x01;
  rsp[2] = req[2];
  rsp[3] = req[3];
  if( size && payload )
    memcpy( &rsp[4], payload, size );

  plin_send_raw( rsp, (uint16_t)( 4 + size ) );
}

static void plin_reset_config( struct plin_channel *chan )
{
  chan->mode = PLIN_MODE_NONE;
  chan->baudrate = 0;
  chan->slot_count = 0;
  chan->schedule_active = 0;
  memset( chan->filter, 0, sizeof( chan->filter ) );
  memset( chan->frame_valid, 0, sizeof( chan->frame_valid ) );
  memset( chan->frames, 0, sizeof( chan->frames ) );
  memset( chan->slots, 0, sizeof( chan->slots ) );
}

static void plin_reset_queues( struct plin_channel *chan )
{
  chan->rx_head = 0;
  chan->rx_tail = 0;
  chan->rx_count = 0;
  chan->rx_overrun = 0;
}

static void plin_kick_rx( uint8_t channel )
{
  USBD_HandleTypeDef *pdev = &hUsbDeviceFS;
  struct plin_channel *chan = &plin_channels[channel];
  uint8_t ep = ( channel == 0 ) ? PLIN_EP_DATA0IN : PLIN_EP_DATA1IN;

  if( pdev->ep_in[ep & 0x0F].total_length )
    return;
  if( chan->rx_count == 0 )
    return;

  memcpy( plin_rx_tx[channel], chan->rx_queue[chan->rx_tail], PLIN_RX_MSG_SIZE );
  chan->rx_tail = (uint8_t)( ( chan->rx_tail + 1 ) % PLIN_RX_QUEUE_LEN );
  chan->rx_count--;

  pdev->ep_in[ep & 0x0F].total_length = PLIN_RX_MSG_SIZE;
  USBD_LL_Transmit( pdev, ep, plin_rx_tx[channel], PLIN_RX_MSG_SIZE );
}

void plin_protocol_init( void )
{
  memset( plin_channels, 0, sizeof( plin_channels ) );
}

void plin_protocol_poll( void )
{
  plin_kick_rx( 0 );
  plin_kick_rx( 1 );
}

void plin_protocol_on_tx_done( uint8_t channel )
{
  if( channel >= PLIN_CHANNELS )
    return;
  plin_kick_rx( channel );
}

void plin_protocol_report_msg( uint8_t channel, const uint8_t *msg, uint16_t size )
{
  struct plin_channel *chan;

  if( channel >= PLIN_CHANNELS || size == 0 )
    return;

  chan = &plin_channels[channel];
  if( size > PLIN_RX_MSG_SIZE )
    size = PLIN_RX_MSG_SIZE;

  if( chan->rx_count >= PLIN_RX_QUEUE_LEN )
  {
    chan->rx_overrun++;
    return;
  }

  memset( chan->rx_queue[chan->rx_head], 0, PLIN_RX_MSG_SIZE );
  memcpy( chan->rx_queue[chan->rx_head], msg, size );
  chan->rx_head = (uint8_t)( ( chan->rx_head + 1 ) % PLIN_RX_QUEUE_LEN );
  chan->rx_count++;
}

void plin_protocol_process_command( const uint8_t *buf, uint16_t size )
{
  struct plin_channel *chan;
  uint8_t command;
  uint8_t channel;

  if( size < 4 )
    return;

  command = buf[0];
  channel = buf[2] & 0x01;
  chan = &plin_channels[channel];

  switch( command )
  {
    case PLIN_CMD_INITIALIZE_HW:
      if( size >= 8 )
      {
        chan->baudrate = (uint16_t)( buf[4] | ( buf[5] << 8 ) );
        chan->mode = ( buf[6] == PLIN_MODE_MASTER ) ? PLIN_MODE_MASTER :
                     ( buf[6] == PLIN_MODE_SLAVE )  ? PLIN_MODE_SLAVE  :
                                                      PLIN_MODE_NONE;
        plin_lin_set_config( channel, chan->baudrate, chan->mode );
      }
      break;

    case PLIN_CMD_RST_HW_CONFIG:
      plin_reset_config( chan );
      plin_lin_set_config( channel, 19200, PLIN_MODE_NONE );
      break;

    case PLIN_CMD_SET_FRAME_ENTRY:
      if( size >= ( 4 + PLIN_FRAME_ENTRY_SIZE ) )
      {
        uint8_t id = buf[4] & 0x3F;
        memcpy( chan->frames[id].raw, &buf[4], PLIN_FRAME_ENTRY_SIZE );
        chan->frame_valid[id] = 1;
      }
      break;

    case PLIN_CMD_GET_FRAME_ENTRY:
    {
      uint8_t entry[PLIN_FRAME_ENTRY_SIZE];
      uint8_t id = ( size >= 5 ) ? ( buf[4] & 0x3F ) : 0;

      memset( entry, 0, sizeof( entry ) );
      if( chan->frame_valid[id] )
        memcpy( entry, chan->frames[id].raw, PLIN_FRAME_ENTRY_SIZE );

      plin_reply( buf, entry, sizeof( entry ) );
      break;
    }

    case PLIN_CMD_SET_ID_FILTER:
      if( size >= 12 )
        memcpy( chan->filter, &buf[4], sizeof( chan->filter ) );
      break;

    case PLIN_CMD_GET_ID_FILTER:
      plin_reply( buf, chan->filter, sizeof( chan->filter ) );
      break;

    case PLIN_CMD_GET_HW_NAME:
    {
      /* Manager 挂载硬件时读取：固定 48 字节名称缓冲 */
      uint8_t name[48] = { 0 };
      memcpy( name, "STM32F072-LIN", 13 );
      plin_reply( buf, name, sizeof( name ) );
      break;
    }

    case PLIN_CMD_GET_MASTER_MODE:
    {
      uint8_t mode[4] = { chan->mode, 0, 0, 0 };
      plin_reply( buf, mode, sizeof( mode ) );
      break;
    }

    case PLIN_CMD_IDENTIFY_HW:
      pcan_led_set_mode( LED_CH0_RX, LED_MODE_BLINK_FAST, 20 );
      pcan_led_set_mode( LED_CH0_TX, LED_MODE_BLINK_FAST, 20 );
      break;

    case PLIN_CMD_GET_FW_VERSION:
    {
      /* 保持 1.4.1.0：驱动走 NoFunc 认证路径 */
      static const uint8_t version[4] = { 1, 4, 1, 0 };
      plin_reply( buf, version, sizeof( version ) );
      break;
    }

    case PLIN_CMD_ADD_SLOT:
      if( size >= 24 )
      {
        uint8_t rsp[32];

        if( chan->slot_count < PLIN_MAX_SLOTS )
        {
          memcpy( chan->slots[chan->slot_count], &buf[8], PLIN_FRAME_ENTRY_SIZE );
          chan->slot_count++;
        }

        memcpy( rsp, buf, 24 );
        memset( &rsp[24], 0, 8 );
        plin_send_raw( rsp, sizeof( rsp ) );
      }
      break;

    case PLIN_CMD_DELETE_SCHEDULE:
      if( size >= 8 )
      {
        uint8_t rsp[20];

        chan->slot_count = 0;
        chan->schedule_active = 0;
        memset( chan->slots, 0, sizeof( chan->slots ) );
        plin_lin_set_active( channel, 0 );

        memset( rsp, 0, sizeof( rsp ) );
        memcpy( rsp, buf, 8 );
        plin_send_raw( rsp, sizeof( rsp ) );
      }
      break;

    case PLIN_CMD_GET_SLOT_COUNT:
    {
      uint8_t count[4] = { chan->slot_count, 0, 0, 0 };
      plin_reply( buf, count, sizeof( count ) );
      break;
    }

    case PLIN_CMD_START_SCHEDULE:
    case PLIN_CMD_RESUME_SCHEDULE:
      if( size >= 8 )
      {
        chan->schedule_active = 1;
        plin_lin_set_active( channel, 1 );
        plin_send_raw( buf, 8 );
      }
      break;

    case PLIN_CMD_SUSPEND_SCHEDULE:
      if( size >= 12 )
      {
        chan->schedule_active = 0;
        plin_lin_set_active( channel, 0 );
        plin_send_raw( buf, 12 );
      }
      break;

    case PLIN_CMD_GET_STATUS:
    {
      uint8_t status[20] = { 0 };

      status[0] = chan->mode;
      status[1] = 16;                         /* FreeOnSendQueue */
      status[2] = 0x00;                       /* FreeOnSchedulePool = 256 */
      status[3] = 0x01;
      status[4] = (uint8_t)( chan->baudrate & 0xFF );
      status[5] = (uint8_t)( chan->baudrate >> 8 );
      status[6] = (uint8_t)( chan->rx_overrun & 0xFF );
      status[7] = (uint8_t)( chan->rx_overrun >> 8 );
      status[16] = ( chan->mode != PLIN_MODE_NONE ) ? 2 : 0; /* Status: Active */
      plin_reply( buf, status, sizeof( status ) );
      break;
    }

    case PLIN_CMD_RST_HW:
      plin_reset_queues( chan );
      break;

    case PLIN_CMD_UPDATE_BYTE_ARRAY:
      if( size >= 16 )
      {
        uint8_t id = buf[4] & 0x3F;
        uint8_t length = buf[5];
        uint16_t offset = (uint16_t)( buf[6] | ( buf[7] << 8 ) );

        if( chan->frame_valid[id] && length <= 8 && offset < 8 )
        {
          uint8_t count = length;

          if( ( offset + count ) > 8 )
            count = (uint8_t)( 8 - offset );
          memcpy( &chan->frames[id].raw[8 + offset], &buf[8], count );
        }
      }
      break;

    case PLIN_CMD_GET_HARDWARE_TYPE:
    {
      uint8_t info[60] = { 0 };

      info[0] = 2;                            /* type = 2 */
      info[4] = 0x9F;                         /* 0x0000DF9F */
      info[5] = 0xDF;
      memcpy( &info[20], "STM32F072-LIN", 13 );
      info[44] = 0x78;                        /* 0x12345678 */
      info[45] = 0x56;
      info[46] = 0x34;
      info[47] = 0x12;
      plin_reply( buf, info, sizeof( info ) );
      break;
    }

    case PLIN_CMD_XMT_WAKE_UP:
      plin_lin_send_wakeup( channel );
      break;

    case PLIN_CMD_GET_BAUDRATE:
    case PLIN_CMD_GET_SERIAL_NUMBER:
    case PLIN_CMD_GET_TARGET_TIME:
    case PLIN_CMD_RESPONSE_REMAP:
    case PLIN_CMD_BREAK_LEN:
    case PLIN_CMD_TERMINATION:
    default:
      /* 无数据响应：只等待 OUT URB 完成 */
      break;
  }
}

int plin_protocol_get_frame( uint8_t channel, uint8_t id, uint8_t *entry )
{
  if( channel >= PLIN_CHANNELS || id >= PLIN_MAX_FRAMES || entry == 0 )
    return 0;
  if( !plin_channels[channel].frame_valid[id] )
    return 0;

  memcpy( entry, plin_channels[channel].frames[id].raw, PLIN_FRAME_ENTRY_SIZE );
  return 1;
}

int plin_protocol_get_slot( uint8_t channel, uint8_t index, uint8_t *slot )
{
  if( channel >= PLIN_CHANNELS || index >= PLIN_MAX_SLOTS || slot == 0 )
    return 0;
  if( index >= plin_channels[channel].slot_count )
    return 0;

  memcpy( slot, plin_channels[channel].slots[index], PLIN_FRAME_ENTRY_SIZE );
  return 1;
}

uint8_t plin_protocol_get_slot_count( uint8_t channel )
{
  if( channel >= PLIN_CHANNELS )
    return 0;
  return plin_channels[channel].slot_count;
}
