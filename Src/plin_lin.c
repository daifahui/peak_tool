#include <string.h>
#include "stm32f0xx_hal.h"
#include "plin_lin.h"
#include "plin_protocol.h"
#include "pcan_usb.h"
#include "pcan_timestamp.h"

#define LIN_RX_RING_SIZE  128
#define LIN_RX_RING_MASK  ( LIN_RX_RING_SIZE - 1 )
#define LIN_TXQ_LEN       8

struct lin_tx_request
{
  uint8_t pid;
  uint8_t length;
  uint8_t direction;
  uint8_t checksum_type;
  uint8_t data[8];
};

enum
{
  LIN_PARSE_WAIT_BREAK = 0,
  LIN_PARSE_WAIT_SYNC,
  LIN_PARSE_WAIT_PID,
  LIN_PARSE_DATA,
  LIN_PARSE_CHECKSUM,
};

/* 每通道的帧阶段：IDLE 空闲，TX 中断发送中，RX 主机等待从机应答 */
enum
{
  LIN_PHASE_IDLE = 0,
  LIN_PHASE_TX,
  LIN_PHASE_TX_SLAVE,
  LIN_PHASE_RX,
};

struct lin_channel
{
  USART_TypeDef *usart;
  uint8_t  mode;
  uint16_t baudrate;
  uint8_t  active;

  uint8_t  slot_index;
  uint32_t next_slot_ms;
  uint16_t slot_delay;

  volatile uint8_t  rx_ring[LIN_RX_RING_SIZE];
  volatile uint8_t  rx_head;
  volatile uint8_t  rx_tail;
  volatile uint8_t  break_flag;

  /* 中断发送引擎 */
  volatile uint8_t  tx_break_wait;
  volatile uint8_t  tx_done;
  uint8_t  tx_buf[16];
  uint8_t  tx_len;
  uint8_t  tx_pos;

  /* LIN_Write 立即发送请求队列 */
  struct lin_tx_request txq[LIN_TXQ_LEN];
  volatile uint8_t txq_head;
  volatile uint8_t txq_tail;

  /* 当前帧上下文 */
  uint8_t  phase;
  uint8_t  frame_id;
  uint8_t  frame_pid;
  uint8_t  frame_len;
  uint8_t  frame_dir;
  uint8_t  frame_cs;
  uint8_t  frame_report;
  uint16_t frame_error;
  uint8_t  rx_expected;
  uint8_t  rx_count;
  uint32_t rx_deadline_ms;
  uint8_t  rx_data[8];

  /* 从机解析 */
  uint8_t  parse_state;
  uint8_t  pid;
  uint8_t  rx_len;
  uint8_t  parse_count;
  uint8_t  last_rx_ms;

  uint16_t sequence;
};

static struct lin_channel lin_channels[2] =
{
  { .usart = USART1 },
  { .usart = USART2 },
};

volatile uint32_t plin_lin_debug[8];

static uint8_t lin_pid( uint8_t id )
{
  uint8_t p0 = (uint8_t)( ( ( id >> 0 ) ^ ( id >> 1 ) ^ ( id >> 2 ) ^ ( id >> 4 ) ) & 1u );
  uint8_t p1 = (uint8_t)( ~( ( id >> 1 ) ^ ( id >> 3 ) ^ ( id >> 4 ) ^ ( id >> 5 ) ) & 1u );

  return (uint8_t)( id | ( p0 << 6 ) | ( p1 << 7 ) );
}

static uint8_t lin_checksum( uint8_t id, uint8_t pid, uint8_t type,
                             const uint8_t *data, uint8_t length )
{
  uint16_t sum = 0;
  int enhanced = ( type == 2 ) || ( type == 3 && id != 0x3C && id != 0x3D );

  for( uint8_t i = 0; i < length; i++ )
    sum += data[i];

  if( enhanced )
    sum += pid;

  return (uint8_t)( 0xFF - ( sum & 0xFF ) );
}

static void lin_push_rx( struct lin_channel *chan, uint8_t data )
{
  uint8_t next = (uint8_t)( ( chan->rx_head + 1 ) & LIN_RX_RING_MASK );

  if( next != chan->rx_tail )
  {
    chan->rx_ring[chan->rx_head] = data;
    chan->rx_head = next;
  }
}

static int lin_pop_rx( struct lin_channel *chan, uint8_t *data )
{
  if( chan->rx_tail == chan->rx_head )
    return 0;

  *data = chan->rx_ring[chan->rx_tail];
  chan->rx_tail = (uint8_t)( ( chan->rx_tail + 1 ) & LIN_RX_RING_MASK );
  return 1;
}

static void lin_flush_rx( struct lin_channel *chan )
{
  while( chan->usart->ISR & USART_ISR_RXNE )
    (void)chan->usart->RDR;

  chan->usart->ICR = USART_ICR_ORECF | USART_ICR_FECF;
  chan->rx_head = 0;
  chan->rx_tail = 0;
  chan->break_flag = 0;
}

static void lin_report( struct lin_channel *chan, uint8_t pid, uint8_t length,
                        uint8_t direction, uint8_t checksum_type,
                        uint16_t error_flags, uint8_t checksum,
                        const uint8_t *data )
{
  uint8_t msg[PLIN_RX_MSG_SIZE] = { 0 };
  uint32_t ts = pcan_timestamp_us();

  (void)checksum;

  if( length > 8 )
    length = 8;

  /* 36 字节线格式（USBPcap 实测）：
   * +0 type, +1 seq(u16), +8 PID, +9 len, +10 dir, +11 checksum type,
   * +16 error flags, +18 checksum(恒 0), +20 timestamp(u32 µs), +28 data[8] */
  msg[0]  = 0;
  msg[1]  = (uint8_t)( chan->sequence & 0xFF );
  msg[2]  = (uint8_t)( chan->sequence >> 8 );
  chan->sequence++;

  msg[8]  = pid;
  msg[9]  = length;
  msg[10] = direction;
  msg[11] = checksum_type;
  msg[16] = (uint8_t)( error_flags & 0xFF );
  msg[17] = (uint8_t)( error_flags >> 8 );
  msg[18] = 0;
  msg[20] = (uint8_t)( ts & 0xFF );
  msg[21] = (uint8_t)( ( ts >> 8 ) & 0xFF );
  msg[22] = (uint8_t)( ( ts >> 16 ) & 0xFF );
  msg[23] = (uint8_t)( ( ts >> 24 ) & 0xFF );

  if( data )
    memcpy( &msg[28], data, length );

  plin_protocol_report_msg( (uint8_t)( chan - lin_channels ), msg, sizeof( msg ) );
}

/* ---- 中断发送引擎 ------------------------------------------------------ */

static void lin_tx_start( struct lin_channel *chan, uint8_t with_break )
{
  USART_TypeDef *usart = chan->usart;

  chan->tx_pos = 0;
  chan->tx_done = 0;
  chan->tx_break_wait = 0;

  usart->CR1 &= ~( USART_CR1_TXEIE | USART_CR1_TCIE );
  usart->ICR = USART_ICR_TCCF;

  if( with_break )
  {
    /* 先发 break，等 TC 确认 break 完整送出后再灌数据 */
    usart->RQR = USART_RQR_SBKRQ;
    chan->tx_break_wait = 1;
    usart->CR1 |= USART_CR1_TCIE;
  }
  else
  {
    usart->CR1 |= USART_CR1_TXEIE;
  }
}

static void lin_tx_stop( struct lin_channel *chan )
{
  chan->usart->CR1 &= ~( USART_CR1_TXEIE | USART_CR1_TCIE );
  chan->tx_break_wait = 0;
  chan->tx_done = 0;
  chan->tx_len = 0;
  chan->tx_pos = 0;
}

static void lin_frame_done( struct lin_channel *chan )
{
  chan->phase = LIN_PHASE_IDLE;

  if( chan->mode == PLIN_LIN_MODE_MASTER && chan->active )
    chan->next_slot_ms = HAL_GetTick() + chan->slot_delay;
}

/* ---- 主机调度 ---------------------------------------------------------- */

/* LIN_Write 立即发送：宿主直接下发一帧，不依赖调度表 */
static void lin_master_write( struct lin_channel *chan )
{
  struct lin_tx_request *req = &chan->txq[chan->txq_tail];
  uint8_t id = (uint8_t)( req->pid & 0x3F );
  uint8_t length = req->length;
  uint8_t checksum = 0;

  chan->txq_tail = (uint8_t)( ( chan->txq_tail + 1 ) % LIN_TXQ_LEN );

  if( length > 8 )
    length = 8;

  chan->frame_id = id;
  chan->frame_pid = req->pid;
  chan->frame_len = length;
  chan->frame_dir = req->direction;
  chan->frame_cs = req->checksum_type;
  chan->frame_error = 0;
  chan->frame_report = 0;
  chan->rx_expected = 0;
  chan->rx_count = 0;

  lin_flush_rx( chan );

  chan->tx_buf[0] = 0x55;
  chan->tx_buf[1] = req->pid;

  if( req->direction == 1 )
  {
    checksum = lin_checksum( id, req->pid, req->checksum_type, req->data, length );
    memcpy( &chan->tx_buf[2], req->data, length );
    chan->tx_buf[2 + length] = checksum;
    chan->tx_len = (uint8_t)( 3 + length );
    chan->frame_report = 1;
  }
  else
  {
    /* direction>=2：只发报头并接收应答 */
    chan->tx_len = 2;
    chan->rx_expected = (uint8_t)( length + 1 );
  }

  lin_tx_start( chan, 1 );
  chan->phase = LIN_PHASE_TX;
}

static void lin_master_start( struct lin_channel *chan )
{
  uint8_t channel = (uint8_t)( chan - lin_channels );
  uint8_t count = plin_protocol_get_slot_count( channel );
  uint8_t slot[16];
  uint8_t entry[16];
  uint8_t id;
  uint16_t delay;
  uint8_t length;
  uint8_t direction;
  uint8_t checksum_type;
  uint16_t flags;
  uint8_t pid;
  uint8_t checksum = 0;
  uint8_t data[8] = { 0 };

  if( count == 0 )
  {
    chan->active = 0;
    lin_flush_rx( chan );
    return;
  }

  if( !plin_protocol_get_slot( channel, chan->slot_index, slot ) )
  {
    chan->slot_index = 0;
    return;
  }

  id = (uint8_t)( slot[4] & 0x3F );
  delay = (uint16_t)( slot[2] | ( slot[3] << 8 ) );
  if( delay < 4 )
    delay = 4;

  chan->slot_delay = delay;
  chan->slot_index = (uint8_t)( ( chan->slot_index + 1 ) % count );

  if( !plin_protocol_get_frame( channel, id, entry ) )
  {
    chan->next_slot_ms = HAL_GetTick() + delay;
    return;
  }

  length = entry[1];
  direction = entry[2];
  checksum_type = entry[3];
  flags = (uint16_t)( entry[4] | ( entry[5] << 8 ) );

  if( direction == 0 )
  {
    chan->next_slot_ms = HAL_GetTick() + delay;
    return;
  }

  if( length > 8 )
    length = 8;

  pid = lin_pid( id );

  chan->frame_id = id;
  chan->frame_pid = pid;
  chan->frame_len = length;
  chan->frame_dir = direction;
  chan->frame_cs = checksum_type;
  chan->frame_error = 0;
  chan->frame_report = 0;
  chan->rx_expected = 0;
  chan->rx_count = 0;

  plin_lin_debug[0]++;
  plin_lin_debug[3] = pid;
  plin_lin_debug[4] = length;

  lin_flush_rx( chan );

  chan->tx_buf[0] = 0x55;
  chan->tx_buf[1] = pid;

  if( direction == 1 && ( flags & 0x0001 ) )
  {
    /* 主机发布：报头 + 数据 + 校验和一次发完 */
    memcpy( data, &entry[8], length );
    checksum = lin_checksum( id, pid, checksum_type, data, length );

    memcpy( &chan->tx_buf[2], data, length );
    chan->tx_buf[2 + length] = checksum;
    chan->tx_len = (uint8_t)( 3 + length );
    chan->frame_report = 1;

    lin_tx_start( chan, 1 );
    chan->phase = LIN_PHASE_TX;
  }
  else if( direction == 1 )
  {
    /* 未使能 RESPONSE_ENABLE：只发报头，按无应答上报 */
    chan->tx_len = 2;
    chan->frame_report = 1;
    chan->frame_len = 0;
    chan->frame_error = 0x0008;

    lin_tx_start( chan, 1 );
    chan->phase = LIN_PHASE_TX;
  }
  else
  {
    /* 主机订阅：报头发完后等待从机应答 */
    chan->tx_len = 2;
    chan->rx_expected = (uint8_t)( length + 1 );

    lin_tx_start( chan, 1 );
    chan->phase = LIN_PHASE_TX;
  }
}

static void lin_master_poll( struct lin_channel *chan )
{
  uint8_t byte;

  if( chan->phase == LIN_PHASE_RX )
  {
    while( chan->rx_count < chan->rx_expected && lin_pop_rx( chan, &byte ) )
      chan->rx_data[chan->rx_count++] = byte;

    if( chan->rx_count >= chan->rx_expected )
    {
      uint8_t checksum = chan->rx_data[chan->rx_expected - 1];
      uint8_t expect = lin_checksum( chan->frame_id, chan->frame_pid, chan->frame_cs,
                                     chan->rx_data, chan->frame_len );

      lin_report( chan, chan->frame_pid, chan->frame_len, chan->frame_dir,
                  chan->frame_cs,
                  ( expect == checksum ) ? 0 : 0x0020, checksum,
                  chan->rx_data );
      lin_frame_done( chan );
    }
    else if( (int32_t)( HAL_GetTick() - chan->rx_deadline_ms ) >= 0 )
    {
      lin_report( chan, chan->frame_pid, 0, chan->frame_dir, chan->frame_cs,
                  0x0008, 0, 0 );
      lin_frame_done( chan );
    }
    return;
  }

  if( chan->phase != LIN_PHASE_IDLE )
    return;

  /* LIN_Write 优先于周期调度，保证立即发送 */
  if( chan->txq_head != chan->txq_tail )
  {
    lin_master_write( chan );
    return;
  }

  if( (int32_t)( HAL_GetTick() - chan->next_slot_ms ) < 0 )
    return;

  lin_master_start( chan );
}

/* ---- 从机 -------------------------------------------------------------- */

static void lin_slave_start_response( struct lin_channel *chan, uint8_t id,
                                      const uint8_t *entry )
{
  uint8_t length = entry[1];
  uint8_t checksum_type = entry[3];
  uint8_t pid = lin_pid( id );
  uint8_t checksum;

  if( length > 8 )
    length = 8;

  memcpy( chan->tx_buf, &entry[8], length );
  checksum = lin_checksum( id, pid, checksum_type, chan->tx_buf, length );
  chan->tx_buf[length] = checksum;

  chan->frame_id = id;
  chan->frame_pid = pid;
  chan->frame_len = length;
  chan->frame_dir = 1;
  chan->frame_cs = checksum_type;
  chan->frame_error = 0;

  chan->tx_len = (uint8_t)( length + 1 );
  lin_tx_start( chan, 0 );
  chan->phase = LIN_PHASE_TX_SLAVE;
}

static void lin_slave_poll( struct lin_channel *chan )
{
  uint8_t channel = (uint8_t)( chan - lin_channels );
  uint8_t byte;

  if( chan->phase == LIN_PHASE_TX_SLAVE )
    return;

  if( chan->break_flag )
  {
    chan->break_flag = 0;
    chan->parse_state = LIN_PARSE_WAIT_SYNC;
    chan->parse_count = 0;
  }

  while( lin_pop_rx( chan, &byte ) )
  {
    chan->last_rx_ms = HAL_GetTick();

    switch( chan->parse_state )
    {
      case LIN_PARSE_WAIT_SYNC:
        chan->parse_state = ( byte == 0x55 ) ? LIN_PARSE_WAIT_PID : LIN_PARSE_WAIT_BREAK;
        break;

      case LIN_PARSE_WAIT_PID:
      {
        uint8_t id = (uint8_t)( byte & 0x3F );
        uint8_t entry[16];

        chan->pid = byte;
        chan->parse_state = LIN_PARSE_WAIT_BREAK;
        chan->parse_count = 0;

        if( plin_protocol_get_frame( channel, id, entry ) )
        {
          uint8_t length = entry[1];
          uint16_t flags = (uint16_t)( entry[4] | ( entry[5] << 8 ) );

          if( length > 8 )
            length = 8;

          if( entry[2] == 1 && ( flags & 0x0001 ) )
          {
            lin_slave_start_response( chan, id, entry );
            return;
          }
          else if( entry[2] >= 2 )
          {
            chan->rx_len = length;
            chan->parse_state = ( length > 0 ) ? LIN_PARSE_DATA : LIN_PARSE_CHECKSUM;
          }
        }
        break;
      }

      case LIN_PARSE_DATA:
        chan->rx_data[chan->parse_count++] = byte;
        if( chan->parse_count >= chan->rx_len )
          chan->parse_state = LIN_PARSE_CHECKSUM;
        break;

      case LIN_PARSE_CHECKSUM:
      {
        uint8_t id = chan->pid & 0x3F;
        uint8_t entry[16];

        if( plin_protocol_get_frame( channel, id, entry ) )
        {
          uint8_t expect = lin_checksum( id, chan->pid, entry[3],
                                         chan->rx_data, chan->rx_len );
          lin_report( chan, chan->pid, chan->rx_len, entry[2], entry[3],
                      ( expect == byte ) ? 0 : 0x0020, byte,
                      chan->rx_data );
        }
        chan->parse_state = LIN_PARSE_WAIT_BREAK;
        break;
      }

      default:
        chan->parse_state = LIN_PARSE_WAIT_BREAK;
        break;
    }
  }

  if( chan->parse_state != LIN_PARSE_WAIT_BREAK &&
      ( HAL_GetTick() - chan->last_rx_ms ) > 20 )
  {
    chan->parse_state = LIN_PARSE_WAIT_BREAK;
  }
}

/* ---- 公共接口 ---------------------------------------------------------- */

static void lin_gpio_init( void )
{
  GPIO_InitTypeDef gpio = { 0 };

  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF1_USART1;
  HAL_GPIO_Init( GPIOA, &gpio );

  gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  gpio.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init( GPIOA, &gpio );
}

static void lin_usart_config( struct lin_channel *chan, uint16_t baudrate )
{
  USART_TypeDef *usart = chan->usart;
  uint32_t pclk = ( usart == USART1 ) ? HAL_RCC_GetHCLKFreq() : HAL_RCC_GetPCLK1Freq();

  lin_tx_stop( chan );

  usart->CR1 = 0;
  usart->CR2 = 0;
  usart->CR3 = 0;

  usart->BRR = ( pclk + baudrate / 2u ) / baudrate;
  usart->CR2 = USART_CR2_LINEN | USART_CR2_LBDL | USART_CR2_LBDIE;
  usart->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_LBDCF;
  usart->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

  lin_flush_rx( chan );
}

void plin_lin_init( void )
{
  lin_gpio_init();

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  NVIC_SetPriority( USART1_IRQn, 2 );
  NVIC_EnableIRQ( USART1_IRQn );
  NVIC_SetPriority( USART2_IRQn, 2 );
  NVIC_EnableIRQ( USART2_IRQn );

  for( int i = 0; i < 2; i++ )
  {
    struct lin_channel *chan = &lin_channels[i];

    chan->mode = PLIN_LIN_MODE_NONE;
    chan->active = 0;
    chan->phase = LIN_PHASE_IDLE;
    chan->slot_index = 0;
    chan->next_slot_ms = 0;
    chan->parse_state = LIN_PARSE_WAIT_BREAK;
    chan->sequence = 0;
    chan->txq_head = 0;
    chan->txq_tail = 0;
    lin_usart_config( chan, 19200 );
  }
}

void plin_lin_set_config( uint8_t channel, uint16_t baudrate, uint8_t mode )
{
  struct lin_channel *chan;

  if( channel >= 2 )
    return;
  chan = &lin_channels[channel];

  if( baudrate < 1000 || baudrate > 20000 )
    baudrate = 19200;

  chan->baudrate = baudrate;
  chan->mode = mode;
  chan->active = 0;
  chan->phase = LIN_PHASE_IDLE;
  chan->sequence = 0;
  chan->parse_state = LIN_PARSE_WAIT_BREAK;
  chan->txq_head = 0;
  chan->txq_tail = 0;
  lin_usart_config( chan, baudrate );
}

void plin_lin_set_active( uint8_t channel, uint8_t active )
{
  struct lin_channel *chan;

  if( channel >= 2 )
    return;
  chan = &lin_channels[channel];

  chan->active = active;
  chan->phase = LIN_PHASE_IDLE;
  chan->slot_index = 0;
  chan->next_slot_ms = HAL_GetTick();
  lin_flush_rx( chan );
}

void plin_lin_send_wakeup( uint8_t channel )
{
  struct lin_channel *chan;

  if( channel >= 2 )
    return;
  chan = &lin_channels[channel];
  if( chan->mode == PLIN_LIN_MODE_NONE || chan->phase != LIN_PHASE_IDLE )
    return;

  /* 仅发送 break，不跟随数据 */
  chan->tx_len = 0;
  chan->frame_report = 0;
  chan->rx_expected = 0;

  lin_tx_start( chan, 1 );
  chan->phase = LIN_PHASE_TX;
}

void plin_lin_write( uint8_t channel, uint8_t pid, uint8_t length, uint8_t direction,
                     uint8_t checksum_type, const uint8_t *data )
{
  struct lin_channel *chan;
  struct lin_tx_request *req;
  uint8_t next;

  if( channel >= 2 || data == 0 )
    return;

  chan = &lin_channels[channel];
  if( chan->mode != PLIN_LIN_MODE_MASTER )
    return;   /* 从机数据已写入帧表，等主机报头 */

  next = (uint8_t)( ( chan->txq_head + 1 ) % LIN_TXQ_LEN );
  if( next == chan->txq_tail )
  {
    /* 队列满时丢弃最旧请求，保留最新写入 */
    chan->txq_tail = (uint8_t)( ( chan->txq_tail + 1 ) % LIN_TXQ_LEN );
    plin_lin_debug[2]++;
  }

  req = &chan->txq[chan->txq_head];
  req->pid = pid;
  req->length = ( length > 8 ) ? 8 : length;
  req->direction = direction;
  req->checksum_type = checksum_type;
  memcpy( req->data, data, req->length );

  chan->txq_head = (uint8_t)( ( chan->txq_head + 1 ) % LIN_TXQ_LEN );
}

void plin_lin_poll( void )
{
  for( int i = 0; i < 2; i++ )
  {
    struct lin_channel *chan = &lin_channels[i];

    if( chan->tx_done )
    {
      chan->tx_done = 0;

      if( chan->phase == LIN_PHASE_TX )
      {
        if( chan->frame_report )
        {
          uint8_t length = chan->frame_len;
          const uint8_t *data = ( length && chan->frame_error == 0 ) ?
                                &chan->tx_buf[2] : 0;

          lin_report( chan, chan->frame_pid, length, chan->frame_dir,
                      chan->frame_cs, chan->frame_error, 0, data );
          lin_frame_done( chan );
        }
        else if( chan->rx_expected )
        {
          /* 报头已发完，转入应答接收窗口；ring 里可能已有先到的应答字节 */
          chan->phase = LIN_PHASE_RX;
          chan->rx_count = 0;
          chan->rx_deadline_ms = HAL_GetTick() + 5u + chan->frame_len * 2u;
        }
        else
        {
          lin_frame_done( chan );
        }
      }
      else if( chan->phase == LIN_PHASE_TX_SLAVE )
      {
        lin_report( chan, chan->frame_pid, chan->frame_len, 1,
                    chan->frame_cs, chan->frame_error, 0, chan->tx_buf );
        lin_frame_done( chan );
        chan->parse_state = LIN_PARSE_WAIT_BREAK;
        lin_flush_rx( chan );
      }
    }

    if( chan->mode == PLIN_LIN_MODE_MASTER && chan->active )
      lin_master_poll( chan );
    else if( chan->mode == PLIN_LIN_MODE_SLAVE )
      lin_slave_poll( chan );
    else if( chan->phase == LIN_PHASE_IDLE )
      lin_flush_rx( chan );
  }
}

/* ---- 中断 -------------------------------------------------------------- */

static void lin_irq_handler( struct lin_channel *chan )
{
  USART_TypeDef *usart = chan->usart;
  uint32_t isr = usart->ISR;
  uint32_t cr1 = usart->CR1;

  if( isr & USART_ISR_LBDF )
  {
    usart->ICR = USART_ICR_LBDCF;
    chan->break_flag = 1;
  }

  if( isr & USART_ISR_ORE )
  {
    usart->ICR = USART_ICR_ORECF;
    (void)usart->RDR;
  }
  else if( isr & USART_ISR_RXNE )
  {
    lin_push_rx( chan, (uint8_t)usart->RDR );
  }

  if( isr & USART_ISR_FE )
    usart->ICR = USART_ICR_FECF;

  if( ( isr & USART_ISR_TXE ) && ( cr1 & USART_CR1_TXEIE ) )
  {
    if( chan->tx_pos < chan->tx_len )
    {
      usart->TDR = chan->tx_buf[chan->tx_pos++];
    }
    else
    {
      usart->CR1 &= ~USART_CR1_TXEIE;
      usart->CR1 |= USART_CR1_TCIE;
    }
  }

  if( ( isr & USART_ISR_TC ) && ( cr1 & USART_CR1_TCIE ) )
  {
    usart->ICR = USART_ICR_TCCF;

    if( chan->tx_break_wait )
    {
      /* break 已完整送出，开始发送后续字节 */
      chan->tx_break_wait = 0;
      usart->CR1 &= ~USART_CR1_TCIE;
      usart->CR1 |= USART_CR1_TXEIE;
    }
    else
    {
      usart->CR1 &= ~USART_CR1_TCIE;
      chan->tx_done = 1;
    }
  }
}

void USART1_IRQHandler( void )
{
  lin_irq_handler( &lin_channels[0] );
}

void USART2_IRQHandler( void )
{
  lin_irq_handler( &lin_channels[1] );
}
