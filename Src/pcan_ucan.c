#include <string.h>
#include "pcan_identity.h"

#if PCAN_CAN_USE_UCAN

#include "stm32f0xx_hal.h"
#include "pcan_ucan.h"
#include "pcan_usb.h"
#include "pcan_can.h"
#include "pcan_led.h"
#include "pcan_timestamp.h"
#include "usbd_core.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

#if PCAN_USB_STRESS
#include "pcan_usb_dbg.h"
#define UCAN_DBG_INC( f ) ( g_usb_dbg.f++ )
#else
#define UCAN_DBG_INC( f ) do { } while( 0 )
#endif

/* uCAN 命令 opcode（opcode_channel 低 10 位） */
#define UCAN_CMD_RESET_MODE       0x001
#define UCAN_CMD_NORMAL_MODE      0x002
#define UCAN_CMD_LISTEN_ONLY_MODE 0x003
#define UCAN_CMD_TIMING_SLOW      0x004
#define UCAN_CMD_TIMING_FAST      0x005
#define UCAN_CMD_FILTER_STD       0x008
#define UCAN_CMD_TX_ABORT         0x009
#define UCAN_CMD_WR_ERR_CNT       0x00A
#define UCAN_CMD_SET_EN_OPTION    0x00B
#define UCAN_CMD_CLR_DIS_OPTION   0x00C
#define UCAN_CMD_RX_BARRIER       0x010
#define UCAN_CMD_CLK_SET          0x080
#define UCAN_CMD_DEVID_SET        0x081
#define UCAN_CMD_LED_SET          0x086
#define UCAN_CMD_END_OF_COLLECTION 0x3FF

/* 设备 → 主机记录类型 */
#define UCAN_MSG_CAN_RX     0x0001
#define UCAN_MSG_STATUS     0x0003
#define UCAN_MSG_BUSLOAD    0x0004
#define UCAN_MSG_CALIBRATION 0x0100

/* 主机 → 设备记录类型 */
#define UCAN_MSG_CAN_TX     0x1000

/* CAN 报文 flags */
#define UCAN_FLAG_RTR           0x0001
#define UCAN_FLAG_EXT_ID        0x0002
#define UCAN_FLAG_EXT_DATA_LEN  0x0010
#define UCAN_FLAG_SELF_RECEIVE  0x0080

/* 状态记录 channel_p_w_b 位 */
#define UCAN_STATUS_BUS_PASSIVE 0x20
#define UCAN_STATUS_BUS_WARNING 0x40
#define UCAN_STATUS_BUS_BUSOFF  0x80

/* 选项位 */
#define UCAN_OPTION_ERROR    0x0001
#define UCAN_OPTION_BUSLOAD  0x0002
#define UCAN_OPTION_CALIB    0x0010

#define UCAN_REPORT_PERIOD_MS 1000u
#define UCAN_TXQ_LEN          8u
#define UCAN_PKT_SIZE         64u

struct ucan_state
{
  uint8_t  bus_active;
  uint8_t  busload_enabled;
  uint8_t  error_enabled;
  uint8_t  calibration_enabled;
  uint8_t  clock_mode;
  uint8_t  tx_busy;
  uint32_t tx_busy_ms;

  uint8_t  q_head;
  uint8_t  q_tail;
  uint8_t  q_count;
  uint8_t  q[UCAN_TXQ_LEN][UCAN_PKT_SIZE];

  uint8_t  pkt[UCAN_PKT_SIZE];
  uint8_t  pkt_len;

  uint16_t frame_index;
  uint32_t bitrate;
  uint32_t last_report_ms;
  uint32_t rx_frames;
  uint32_t tx_frames;
};

static struct ucan_state ucan;

static void ucan_report_can( const can_message_t *msg );
static void ucan_report_status( uint8_t channel );

static void ucan_put_ts( uint8_t *p, uint64_t ts )
{
  p[0] = (uint8_t)( ts );
  p[1] = (uint8_t)( ts >> 8 );
  p[2] = (uint8_t)( ts >> 16 );
  p[3] = (uint8_t)( ts >> 24 );
  p[4] = (uint8_t)( ts >> 32 );
  p[5] = (uint8_t)( ts >> 40 );
  p[6] = (uint8_t)( ts >> 48 );
  p[7] = (uint8_t)( ts >> 56 );
}

static void ucan_queue_packet( const uint8_t *pkt )
{
  if( ucan.q_count >= UCAN_TXQ_LEN )
  {
    UCAN_DBG_INC( ucan_txq_drop );
    return;
  }

  memcpy( ucan.q[ucan.q_head], pkt, UCAN_PKT_SIZE );
  ucan.q_head = (uint8_t)( ( ucan.q_head + 1u ) % UCAN_TXQ_LEN );
  ucan.q_count++;
}

static void ucan_flush_pkt( void )
{
  if( ucan.pkt_len == 0 )
    return;

  memset( &ucan.pkt[ucan.pkt_len], 0, UCAN_PKT_SIZE - ucan.pkt_len );
  ucan_queue_packet( ucan.pkt );
  ucan.pkt_len = 0;
}

static void ucan_append( const uint8_t *rec, uint8_t size )
{
  if( size == 0 || size > UCAN_PKT_SIZE )
    return;

  if( (uint16_t)( ucan.pkt_len + size ) > UCAN_PKT_SIZE )
    ucan_flush_pkt();

  memcpy( &ucan.pkt[ucan.pkt_len], rec, size );
  ucan.pkt_len = (uint8_t)( ucan.pkt_len + size );
}

static void ucan_kick( void )
{
  USBD_HandleTypeDef *pdev = &hUsbDeviceFS;
  uint8_t ep = PCAN_USB_EP_MSGIN & 0x0Fu;

  if( ucan.tx_busy || ucan.q_count == 0 )
    return;
  if( pdev->ep_in[ep].total_length )
    return;

  pdev->ep_in[ep].total_length = UCAN_PKT_SIZE;
  if( USBD_LL_Transmit( pdev, PCAN_USB_EP_MSGIN,
                        ucan.q[ucan.q_tail], UCAN_PKT_SIZE ) == USBD_OK )
  {
    ucan.tx_busy = 1;
    ucan.tx_busy_ms = HAL_GetTick();
    ucan.q_tail = (uint8_t)( ( ucan.q_tail + 1u ) % UCAN_TXQ_LEN );
    ucan.q_count--;
  }
  else
  {
    pdev->ep_in[ep].total_length = 0;
  }
}

void pcan_ucan_on_tx_done( void )
{
  ucan.tx_busy = 0;
  ucan_kick();
}

/* USB 重新配置/复位：丢弃旧会话残留的 IN 传输与队列，
 * 避免主机取消 URB 后 tx_busy 永久卡住。 */
void pcan_ucan_usb_reset( void )
{
  USBD_HandleTypeDef *pdev = &hUsbDeviceFS;

  pdev->ep_in[PCAN_USB_EP_MSGIN & 0x0Fu].total_length = 0;
  ucan.tx_busy = 0;
  ucan.q_head = 0;
  ucan.q_tail = 0;
  ucan.q_count = 0;
  ucan.pkt_len = 0;
}

static void ucan_report_can( const can_message_t *msg )
{
  uint8_t  rec[36];
  uint8_t  dlc = msg->dlc > 8 ? 8 : msg->dlc;
  uint16_t size = (uint16_t)( 28u + dlc );
  uint16_t aligned = (uint16_t)( ( size + 3u ) & ~3u );
  uint16_t flags = 0;
  uint64_t ts = pcan_timestamp_us64();

  if( msg->flags & CAN_FLAG_EXTID )
    flags |= UCAN_FLAG_EXT_ID;
  if( msg->flags & CAN_FLAG_RTR )
    flags |= UCAN_FLAG_RTR;
  if( msg->flags & CAN_FLAG_ECHO )
    flags |= UCAN_FLAG_SELF_RECEIVE;

  memset( rec, 0, sizeof( rec ) );
  rec[0] = (uint8_t)( aligned );
  rec[1] = (uint8_t)( aligned >> 8 );
  rec[2] = (uint8_t)( UCAN_MSG_CAN_RX );
  rec[3] = (uint8_t)( UCAN_MSG_CAN_RX >> 8 );
  ucan_put_ts( &rec[4], ts );
  /* +12 tag_low, +16 tag_high 保持 0 */
  rec[20] = (uint8_t)( ( dlc << 4 ) | 0u );
  rec[21] = 0; /* client */
  rec[22] = (uint8_t)( flags );
  rec[23] = (uint8_t)( flags >> 8 );
  rec[24] = (uint8_t)( msg->id );
  rec[25] = (uint8_t)( msg->id >> 8 );
  rec[26] = (uint8_t)( msg->id >> 16 );
  rec[27] = (uint8_t)( msg->id >> 24 );
  if( !( msg->flags & CAN_FLAG_RTR ) )
    memcpy( &rec[28], msg->data, dlc );

  ucan_append( rec, (uint8_t)aligned );
}

static uint8_t ucan_bus_flags( void )
{
  uint32_t esr;

  if( !ucan.bus_active )
    return 0;

  esr = pcan_can_esr();
  if( esr & CAN_ESR_BOFF )
    return UCAN_STATUS_BUS_BUSOFF;
  if( esr & CAN_ESR_EPVF )
    return UCAN_STATUS_BUS_PASSIVE;
  if( esr & CAN_ESR_EWGF )
    return UCAN_STATUS_BUS_WARNING;
  return 0;
}

static void ucan_report_status( uint8_t channel )
{
  uint8_t  rec[16];
  uint64_t ts = pcan_timestamp_us64();

  memset( rec, 0, sizeof( rec ) );
  rec[0] = 16;
  rec[2] = (uint8_t)( UCAN_MSG_STATUS );
  rec[3] = (uint8_t)( UCAN_MSG_STATUS >> 8 );
  ucan_put_ts( &rec[4], ts );
  rec[12] = (uint8_t)( ( channel & 0x0Fu ) |
                       ( channel == 0 ? ucan_bus_flags() : 0 ) );
  ucan_append( rec, sizeof( rec ) );
}

static void ucan_report_calibration( void )
{
  uint8_t  rec[16];
  uint64_t ts = pcan_timestamp_us64();

  memset( rec, 0, sizeof( rec ) );
  rec[0] = 16;
  rec[2] = (uint8_t)( UCAN_MSG_CALIBRATION );
  rec[3] = (uint8_t)( UCAN_MSG_CALIBRATION >> 8 );
  ucan_put_ts( &rec[4], ts );
  rec[12] = (uint8_t)( ucan.frame_index );
  rec[13] = (uint8_t)( ucan.frame_index >> 8 );
  ucan.frame_index++;
  ucan_append( rec, sizeof( rec ) );
}

static void ucan_report_busload( uint8_t channel, uint8_t load )
{
  uint8_t  rec[16];
  uint64_t ts = pcan_timestamp_us64();

  memset( rec, 0, sizeof( rec ) );
  rec[0] = 16;
  rec[2] = (uint8_t)( UCAN_MSG_BUSLOAD );
  rec[3] = (uint8_t)( UCAN_MSG_BUSLOAD >> 8 );
  ucan_put_ts( &rec[4], ts );
  rec[12] = (uint8_t)( channel & 0x0Fu );
  rec[13] = load;
  ucan_append( rec, sizeof( rec ) );
}

static void ucan_set_options( uint16_t options, uint8_t on )
{
  if( options & UCAN_OPTION_ERROR )
    ucan.error_enabled = on;
  if( options & UCAN_OPTION_BUSLOAD )
    ucan.busload_enabled = on;
  if( options & UCAN_OPTION_CALIB )
    ucan.calibration_enabled = on;
  /* FD ISO 及其它 USB 选项对经典 bxCAN 无意义，忽略 */
}

static void ucan_set_led( uint8_t mode )
{
  int led_mode;

  switch( mode )
  {
    case 1: led_mode = LED_MODE_BLINK_FAST; break;
    case 2: led_mode = LED_MODE_BLINK_SLOW; break;
    case 3: led_mode = LED_MODE_ON;         break;
    case 4: led_mode = LED_MODE_OFF;        break;
    default: return;
  }

  pcan_led_set_mode( LED_CH0_TX, led_mode, 0 );
  pcan_led_set_mode( LED_CH0_RX, led_mode, 0 );
}

static void ucan_set_timing_slow( const uint8_t *p )
{
  /* p[0..1]=opcode, p[2]=ewl, p[3]=sjw_t, p[4]=tseg2, p[5]=tseg1, p[6..7]=brp */
  uint32_t tseg1 = (uint32_t)p[5] + 1u;
  uint32_t tseg2 = (uint32_t)( p[4] & 0x7Fu ) + 1u;
  uint32_t brp = ( (uint32_t)( p[6] | ( p[7] << 8 ) ) & 0x3FFu ) + 1u;
  uint8_t  sjw = (uint8_t)( ( p[3] & 0x7Fu ) + 1u );
  uint32_t total = 1u + tseg1 + tseg2;
  uint32_t bps;

  if( brp == 0 || total == 0 )
    return;

  /* 真机 uCAN 以 80 MHz 为位时序基准 */
  bps = 80000000u / ( brp * total );
  ucan.bitrate = bps;
  pcan_can_set_timing_bps( bps, tseg1, tseg2, sjw );
}

static void ucan_handle_command( uint16_t opcode, uint8_t channel, const uint8_t *p )
{
  switch( opcode )
  {
    case UCAN_CMD_RESET_MODE:
      if( channel == 0 )
      {
        pcan_can_set_bus_active( 0 );
        ucan.bus_active = 0;
      }
      break;

    case UCAN_CMD_NORMAL_MODE:
      if( channel == 0 )
      {
        pcan_can_set_silent( 0 );
        pcan_can_set_bus_active( 1 );
        ucan.bus_active = 1;
      }
      break;

    case UCAN_CMD_LISTEN_ONLY_MODE:
      if( channel == 0 )
      {
        pcan_can_set_silent( 1 );
        pcan_can_set_bus_active( 1 );
        ucan.bus_active = 1;
      }
      break;

    case UCAN_CMD_TIMING_SLOW:
      if( channel == 0 )
        ucan_set_timing_slow( p );
      break;

    case UCAN_CMD_TIMING_FAST:
      /* F072 bxCAN 只支持经典 CAN，数据段时序忽略 */
      break;

    case UCAN_CMD_FILTER_STD:
      /* 驱动未下发过滤命令时默认全通，这里保持全通 */
      break;

    case UCAN_CMD_TX_ABORT:
      if( channel == 0 )
        pcan_can_abort_tx();
      break;

    case UCAN_CMD_WR_ERR_CNT:
      break;

    case UCAN_CMD_SET_EN_OPTION:
      ucan_set_options( (uint16_t)( p[2] | ( p[3] << 8 ) ), 1 );
      break;

    case UCAN_CMD_CLR_DIS_OPTION:
      ucan_set_options( (uint16_t)( p[2] | ( p[3] << 8 ) ), 0 );
      break;

    case UCAN_CMD_RX_BARRIER:
      /* 回一条状态，RX_BARRIER 位由主机侧自行判断 */
      ucan_report_status( channel );
      ucan_flush_pkt();
      break;

    case UCAN_CMD_CLK_SET:
      ucan.clock_mode = p[2];
      break;

    case UCAN_CMD_DEVID_SET:
      break;

    case UCAN_CMD_LED_SET:
      ucan_set_led( p[2] );
      break;

    default:
      break;
  }
}

void pcan_ucan_process_command( const uint8_t *buf, uint16_t size )
{
  uint16_t off;

  for( off = 0; (uint16_t)( off + 8u ) <= size; off = (uint16_t)( off + 8u ) )
  {
    uint16_t opcode_channel = (uint16_t)( buf[off] | ( buf[off + 1] << 8 ) );
    uint16_t opcode = opcode_channel & 0x3FFu;
    uint8_t  channel = (uint8_t)( ( opcode_channel >> 12 ) & 0x0Fu );

    if( opcode == UCAN_CMD_END_OF_COLLECTION )
      break;

    ucan_handle_command( opcode, channel, &buf[off] );
  }
}

static void ucan_process_tx_record( const uint8_t *rec, uint16_t size )
{
  can_message_t msg;
  uint8_t  channel;
  uint8_t  dlc;
  uint16_t flags;
  uint32_t id;

#if PCAN_USB_STRESS
  pcan_usb_dbg_can_rec( rec, size );
#endif

  if( size < 20u )
    return;

  channel = (uint8_t)( rec[12] & 0x0Fu );
  dlc = (uint8_t)( rec[12] >> 4 );
  flags = (uint16_t)( rec[14] | ( rec[15] << 8 ) );
  id = (uint32_t)rec[16] | ( (uint32_t)rec[17] << 8 ) |
       ( (uint32_t)rec[18] << 16 ) | ( (uint32_t)rec[19] << 24 );

  /* 单路 CAN 硬件：CAN2 暂不发送 */
  if( channel != 0 )
  {
    UCAN_DBG_INC( can_tx_ch1 );
    return;
  }

  /* 经典 bxCAN 不支持 CAN FD */
  if( flags & UCAN_FLAG_EXT_DATA_LEN )
    return;
  if( dlc > 8u )
    return;
  /* RTR 帧没有数据区，记录长度固定为 20（16 字节头 + 4 字节 ID） */
  if( !( flags & UCAN_FLAG_RTR ) && (uint16_t)( 20u + dlc ) > size )
    return;

  memset( &msg, 0, sizeof( msg ) );
  msg.id = id;
  msg.dlc = dlc;
  if( flags & UCAN_FLAG_EXT_ID )
    msg.flags |= CAN_FLAG_EXTID;
  if( flags & UCAN_FLAG_RTR )
    msg.flags |= CAN_FLAG_RTR;
  if( !( flags & UCAN_FLAG_RTR ) )
    memcpy( msg.data, &rec[20], dlc );

  if( pcan_can_send_message( &msg ) == 0 )
  {
    UCAN_DBG_INC( can_tx_records );
    ucan.tx_frames++;

    if( flags & UCAN_FLAG_SELF_RECEIVE )
    {
      msg.flags |= CAN_FLAG_ECHO;
      ucan_report_can( &msg );
    }
  }
  else
  {
    UCAN_DBG_INC( can_tx_fail );
  }
}

void pcan_ucan_process_data( const uint8_t *buf, uint16_t size, uint8_t channel )
{
  uint16_t off = 0;

  (void)channel; /* 记录内的 channel 字段才是权威值 */

  while( (uint16_t)( off + 4u ) <= size )
  {
    uint16_t rec_size = (uint16_t)( buf[off] | ( buf[off + 1] << 8 ) );
    uint16_t type;
    uint16_t step;

    if( rec_size == 0 )
      break;
    if( (uint32_t)off + rec_size > size )
      break;

    type = (uint16_t)( buf[off + 2] | ( buf[off + 3] << 8 ) );
    if( type == UCAN_MSG_CAN_TX )
      ucan_process_tx_record( &buf[off], rec_size );

    step = (uint16_t)( ( rec_size + 3u ) & ~3u );
    if( step == 0 )
      break;
    off = (uint16_t)( off + step );
  }
}

static void pcan_ucan_can_rx( can_message_t *msg )
{
  if( !ucan.bus_active )
    return;

  UCAN_DBG_INC( can_rx_records );
  ucan.rx_frames++;
  pcan_led_set_mode( LED_CH0_RX, LED_MODE_BLINK_FAST, 237 );
  ucan_report_can( msg );
}

static void pcan_ucan_can_error( uint8_t err, uint8_t rx_err, uint8_t tx_err )
{
  /* 总线状态在 1 Hz 状态记录中通过 ESR 上报 */
  (void)err;
  (void)rx_err;
  (void)tx_err;
}

void pcan_ucan_init( void )
{
  memset( &ucan, 0, sizeof( ucan ) );
  ucan.last_report_ms = HAL_GetTick() - UCAN_REPORT_PERIOD_MS;

  pcan_can_init();
  pcan_can_install_rx_callback( pcan_ucan_can_rx );
  pcan_can_install_error_callback( pcan_ucan_can_error );

  pcan_led_set_mode( LED_CH0_RX, LED_MODE_BLINK_SLOW, 0 );
  pcan_led_set_mode( LED_CH0_TX, LED_MODE_BLINK_SLOW, 0 );
}

void pcan_ucan_poll( void )
{
  uint32_t now;
  uint32_t load = 0;

  /* 持续推进 64 位 µs 计数器，避免 TIM2 回绕丢时间 */
  (void)pcan_timestamp_us64();

  pcan_can_poll();

  now = HAL_GetTick();
  if( (uint32_t)( now - ucan.last_report_ms ) >= UCAN_REPORT_PERIOD_MS )
  {
    ucan.last_report_ms = now;

    if( ucan.bitrate != 0 )
    {
      load = ( ucan.rx_frames + ucan.tx_frames ) * 130u * 100u / ucan.bitrate;
      if( load > 100u )
        load = 100u;
    }

    ucan_report_calibration();
    ucan_report_status( 0 );
    ucan_report_status( 1 );
    if( ucan.busload_enabled )
    {
      ucan_report_busload( 0, (uint8_t)load );
      ucan_report_busload( 1, 0 );
    }

    ucan.rx_frames = 0;
    ucan.tx_frames = 0;
  }

  /* 主机取消 IN URB 时不会回调 DataIn，超时后主动解锁 */
  if( ucan.tx_busy &&
      (uint32_t)( HAL_GetTick() - ucan.tx_busy_ms ) > 200u )
  {
    USBD_HandleTypeDef *pdev = &hUsbDeviceFS;

    pdev->ep_in[PCAN_USB_EP_MSGIN & 0x0Fu].total_length = 0;
    ucan.tx_busy = 0;
  }

  ucan_flush_pkt();
  ucan_kick();

#if PCAN_USB_STRESS
  g_usb_dbg.ucan_txq_count = ucan.q_count;
#endif
}

#endif /* PCAN_CAN_USE_UCAN */
