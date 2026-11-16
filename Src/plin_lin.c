#include <string.h>
#include "stm32f0xx_hal.h"
#include "plin_lin.h"
#include "plin_protocol.h"

#define LIN_RX_RING_SIZE  128
#define LIN_RX_RING_MASK  ( LIN_RX_RING_SIZE - 1 )

enum
{
  LIN_PARSE_WAIT_BREAK = 0,
  LIN_PARSE_WAIT_SYNC,
  LIN_PARSE_WAIT_PID,
  LIN_PARSE_DATA,
  LIN_PARSE_CHECKSUM,
};

struct lin_channel
{
  USART_TypeDef *usart;
  uint8_t  mode;
  uint16_t baudrate;
  uint8_t  active;

  uint8_t  slot_index;
  uint32_t next_slot_ms;

  volatile uint8_t  rx_ring[LIN_RX_RING_SIZE];
  volatile uint8_t  rx_head;
  volatile uint8_t  rx_tail;
  volatile uint8_t  break_flag;

  uint8_t  parse_state;
  uint8_t  pid;
  uint8_t  rx_len;
  uint8_t  rx_count;
  uint8_t  rx_data[8];
  uint32_t last_rx_ms;

  uint16_t sequence;
};

static struct lin_channel lin_channels[2] =
{
  { .usart = USART1 },
  { .usart = USART2 },
};

static void lin_putc( struct lin_channel *chan, uint8_t data )
{
  while( ( chan->usart->ISR & USART_ISR_TXE ) == 0 )
    ;
  chan->usart->TDR = data;
}

static void lin_wait_tc( struct lin_channel *chan )
{
  while( ( chan->usart->ISR & USART_ISR_TC ) == 0 )
    ;
}

static void lin_send_break( struct lin_channel *chan )
{
  chan->usart->RQR = USART_RQR_SBKRQ;
  lin_wait_tc( chan );
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

static void lin_report( struct lin_channel *chan, uint8_t pid, uint8_t length,
                        uint8_t direction, uint8_t checksum_type,
                        uint16_t error_flags, uint8_t checksum,
                        const uint8_t *data )
{
  uint8_t msg[PLIN_RX_MSG_SIZE] = { 0 };
  uint32_t ts = HAL_GetTick() * 1000u;

  msg[0]  = 0; /* RX_MSG */
  msg[1]  = (uint8_t)( chan->sequence & 0xFF );
  msg[2]  = (uint8_t)( chan->sequence >> 8 );
  chan->sequence++;

  msg[8]  = pid;
  msg[9]  = length;
  msg[10] = direction;
  msg[11] = checksum_type;
  msg[16] = (uint8_t)( error_flags & 0xFF );
  msg[17] = (uint8_t)( error_flags >> 8 );
  msg[18] = checksum;
  msg[20] = (uint8_t)( ts & 0xFF );
  msg[21] = (uint8_t)( ( ts >> 8 ) & 0xFF );
  msg[22] = (uint8_t)( ( ts >> 16 ) & 0xFF );
  msg[23] = (uint8_t)( ( ts >> 24 ) & 0xFF );

  if( length > 8 )
    length = 8;
  if( data )
    memcpy( &msg[28], data, length );

  plin_protocol_report_msg( (uint8_t)( chan - lin_channels ), msg, sizeof( msg ) );
}

static void lin_master_frame( struct lin_channel *chan, uint8_t id, const uint8_t *entry )
{
  uint8_t length = entry[1];
  uint8_t direction = entry[2];
  uint8_t checksum_type = entry[3];
  uint16_t flags = (uint16_t)( entry[4] | ( entry[5] << 8 ) );
  uint8_t pid = lin_pid( id );
  uint8_t data[8] = { 0 };
  uint8_t checksum = 0;
  uint16_t error = 0;

  if( direction == 0 )
    return;
  if( length > 8 )
    length = 8;

  chan->usart->CR1 &= ~USART_CR1_RXNEIE;
  lin_flush_rx( chan );

  lin_send_break( chan );
  lin_putc( chan, 0x55 );
  lin_putc( chan, pid );
  lin_wait_tc( chan );

  if( direction == 1 && ( flags & 0x0001 ) )
  {
    memcpy( data, &entry[8], length );
    checksum = lin_checksum( id, pid, checksum_type, data, length );

    for( uint8_t i = 0; i < length; i++ )
      lin_putc( chan, data[i] );
    lin_putc( chan, checksum );
    lin_wait_tc( chan );
  }
  else if( direction == 1 )
  {
    /* 未使能 RESPONSE_ENABLE：只有报头，按无应答上报 */
    error = 0x0008;
    length = 0;
  }
  else
  {
    uint8_t rx[9];
    uint8_t count = 0;
    uint32_t start = HAL_GetTick();
    uint32_t timeout = (uint32_t)( 5 + length * 2 );

    HAL_Delay( 1 );
    lin_flush_rx( chan );

    while( count < (uint8_t)( length + 1 ) )
    {
      if( chan->usart->ISR & USART_ISR_RXNE )
      {
        rx[count++] = (uint8_t)chan->usart->RDR;
        start = HAL_GetTick();
      }
      else if( ( HAL_GetTick() - start ) > timeout )
      {
        break;
      }
    }

    if( count == (uint8_t)( length + 1 ) )
    {
      memcpy( data, rx, length );
      checksum = rx[length];
    }
    else
    {
      error = 0x0008;
      length = 0;
    }
  }

  HAL_Delay( 1 );
  lin_flush_rx( chan );
  chan->usart->CR1 |= USART_CR1_RXNEIE;

  lin_report( chan, pid, length, direction, checksum_type, error, checksum, data );
}

static void lin_master_poll( struct lin_channel *chan )
{
  uint8_t channel = (uint8_t)( chan - lin_channels );
  uint8_t count = plin_protocol_get_slot_count( channel );
  uint8_t slot[16];
  uint8_t entry[16];
  uint8_t id;
  uint16_t delay;

  if( count == 0 )
  {
    chan->active = 0;
    lin_flush_rx( chan );
    return;
  }

  if( (int32_t)( HAL_GetTick() - chan->next_slot_ms ) < 0 )
    return;

  if( !plin_protocol_get_slot( channel, chan->slot_index, slot ) )
  {
    chan->slot_index = 0;
    return;
  }

  id = (uint8_t)( slot[4] & 0x3F );
  delay = (uint16_t)( slot[2] | ( slot[3] << 8 ) );
  if( delay < 4 )
    delay = 4;

  if( plin_protocol_get_frame( channel, id, entry ) )
    lin_master_frame( chan, id, entry );

  chan->slot_index = (uint8_t)( ( chan->slot_index + 1 ) % count );
  chan->next_slot_ms = HAL_GetTick() + delay;
}

static void lin_send_slave_response( struct lin_channel *chan, uint8_t id, const uint8_t *entry )
{
  uint8_t length = entry[1];
  uint8_t checksum_type = entry[3];
  uint8_t pid = lin_pid( id );
  uint8_t data[8] = { 0 };
  uint8_t checksum;

  if( length > 8 )
    length = 8;

  memcpy( data, &entry[8], length );
  checksum = lin_checksum( id, pid, checksum_type, data, length );

  chan->usart->CR1 &= ~USART_CR1_RXNEIE;
  for( uint8_t i = 0; i < length; i++ )
    lin_putc( chan, data[i] );
  lin_putc( chan, checksum );
  lin_wait_tc( chan );
  HAL_Delay( 1 );
  lin_flush_rx( chan );
  chan->usart->CR1 |= USART_CR1_RXNEIE;

  lin_report( chan, pid, length, 1, checksum_type, 0, checksum, data );
}

static void lin_slave_poll( struct lin_channel *chan )
{
  uint8_t channel = (uint8_t)( chan - lin_channels );
  uint8_t byte;

  if( chan->break_flag )
  {
    chan->break_flag = 0;
    chan->parse_state = LIN_PARSE_WAIT_SYNC;
    chan->rx_count = 0;
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
        chan->rx_count = 0;

        if( plin_protocol_get_frame( channel, id, entry ) )
        {
          uint8_t length = entry[1];
          uint16_t flags = (uint16_t)( entry[4] | ( entry[5] << 8 ) );

          if( length > 8 )
            length = 8;

          if( entry[2] == 1 && ( flags & 0x0001 ) )
          {
            lin_send_slave_response( chan, id, entry );
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
        chan->rx_data[chan->rx_count++] = byte;
        if( chan->rx_count >= chan->rx_len )
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
                      ( expect == byte ) ? 0 : 0x0004, byte,
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
    chan->slot_index = 0;
    chan->next_slot_ms = 0;
    chan->parse_state = LIN_PARSE_WAIT_BREAK;
    chan->sequence = 0;
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
  chan->parse_state = LIN_PARSE_WAIT_BREAK;
  lin_usart_config( chan, baudrate );
}

void plin_lin_set_active( uint8_t channel, uint8_t active )
{
  struct lin_channel *chan;

  if( channel >= 2 )
    return;
  chan = &lin_channels[channel];

  chan->active = active;
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
  if( chan->mode == PLIN_LIN_MODE_NONE )
    return;

  lin_send_break( chan );
}

void plin_lin_poll( void )
{
  for( int i = 0; i < 2; i++ )
  {
    struct lin_channel *chan = &lin_channels[i];

    if( chan->mode == PLIN_LIN_MODE_MASTER && chan->active )
      lin_master_poll( chan );
    else if( chan->mode == PLIN_LIN_MODE_SLAVE )
      lin_slave_poll( chan );
    else
      lin_flush_rx( chan );
  }
}

static void lin_irq_handler( struct lin_channel *chan )
{
  USART_TypeDef *usart = chan->usart;
  uint32_t isr = usart->ISR;

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
}

void USART1_IRQHandler( void )
{
  lin_irq_handler( &lin_channels[0] );
}

void USART2_IRQHandler( void )
{
  lin_irq_handler( &lin_channels[1] );
}
