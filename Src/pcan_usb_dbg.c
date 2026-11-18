#include "pcan_identity.h"

#if PCAN_USB_STRESS

#include "pcan_usb_dbg.h"

struct pcan_usb_dbg_s g_usb_dbg;

void pcan_usb_dbg_out( uint8_t ep, const uint8_t *data, uint16_t size )
{
  uint8_t idx = (uint8_t)( ep & 0x0Fu );
  uint8_t n;
  uint32_t sum = 0;

  if( idx >= 7u || data == 0 )
    return;

  g_usb_dbg.out_pkts[idx]++;
  g_usb_dbg.out_bytes[idx] += size;
  g_usb_dbg.out_last_len[idx] = size;

  n = ( size > 40u ) ? 40u : (uint8_t)size;
  for( uint8_t i = 0; i < n; i++ )
    sum += data[i];

  /* 只保留最后一个非零数据包，避免主机断开时的全零包覆盖快照 */
  if( sum != 0 )
  {
    for( uint8_t i = 0; i < n; i++ )
      g_usb_dbg.out_snap[idx][i] = data[i];
    for( uint8_t i = n; i < 40u; i++ )
      g_usb_dbg.out_snap[idx][i] = 0;

    g_usb_dbg.out_sum[idx] = sum;
  }
}

void pcan_usb_dbg_in( uint8_t ep )
{
  uint8_t idx = (uint8_t)( ep & 0x0Fu );

  if( idx < 7u )
    g_usb_dbg.in_done[idx]++;
}

void pcan_usb_dbg_cmd( const uint8_t *data, uint16_t size )
{
  uint8_t idx = (uint8_t)( g_usb_dbg.cmd_log_idx & 0x1Fu );
  uint8_t n = ( size > 8u ) ? 8u : (uint8_t)size;

  for( uint8_t i = 0; i < 8u; i++ )
    g_usb_dbg.cmd_log[idx][i] = ( i < n ) ? data[i] : 0;
  g_usb_dbg.cmd_log_idx++;
}

void pcan_usb_dbg_rsp( const uint8_t *data, uint16_t size )
{
  uint8_t idx = (uint8_t)( g_usb_dbg.rsp_log_idx & 0x1Fu );
  uint8_t n = ( size > 8u ) ? 8u : (uint8_t)size;

  for( uint8_t i = 0; i < 8u; i++ )
    g_usb_dbg.rsp_log[idx][i] = ( i < n ) ? data[i] : 0;
  g_usb_dbg.rsp_log_idx++;
}

void pcan_usb_dbg_can_out( uint8_t ep, const uint8_t *data, uint16_t size )
{
  uint8_t idx = (uint8_t)( g_usb_dbg.can_out_idx & 0x1Fu );
  uint8_t n = ( size > 8u ) ? 8u : (uint8_t)size;

  (void)ep;
  for( uint8_t i = 0; i < 8u; i++ )
    g_usb_dbg.can_out_log[idx][i] = ( i < n ) ? data[i] : 0;
  g_usb_dbg.can_out_len[idx] = (uint8_t)( size > 255u ? 255u : size );
  g_usb_dbg.can_out_idx++;
}

void pcan_usb_dbg_can_rec( const uint8_t *rec, uint16_t size )
{
  uint8_t idx = (uint8_t)( g_usb_dbg.can_rec_idx & 0x1Fu );

  /* 记录 dlc/ch、flags、ID（rec[12..19]），便于定位丢失的帧 */
  for( uint8_t i = 0; i < 8u; i++ )
    g_usb_dbg.can_rec_log[idx][i] = rec[12 + i];
  g_usb_dbg.can_rec_size[idx] = (uint8_t)( size > 255u ? 255u : size );
  g_usb_dbg.can_rec_idx++;
}

#endif

/* 非压测构建保留一个类型声明，避免 -Wpedantic 报空编译单元 */
typedef int pcan_usb_dbg_not_used_t;
