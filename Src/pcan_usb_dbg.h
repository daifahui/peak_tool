#pragma once
#include <stdint.h>

/* 仅 PCAN_USB_STRESS 调试构建启用的 USB 端点观测结构。
 * 正式固件不包含任何计数器，不影响时序与 RAM。 */
#if PCAN_USB_STRESS

struct pcan_usb_dbg_s
{
  volatile uint32_t out_pkts[7];
  volatile uint32_t out_bytes[7];
  volatile uint32_t out_last_len[7];
  volatile uint32_t out_sum[7];
  volatile uint8_t  out_snap[7][40];

  volatile uint32_t in_done[7];

  volatile uint32_t can_tx_records;
  volatile uint32_t can_tx_ch1;
  volatile uint32_t can_tx_fail;
  volatile uint32_t can_rx_records;
  volatile uint32_t ucan_txq_drop;
  volatile uint32_t ucan_txq_count;

  /* PLIN 命令/响应环形日志：定位 Manager 与设备的交互序列 */
  volatile uint8_t  cmd_log[32][8];
  volatile uint8_t  cmd_log_idx;
  volatile uint8_t  rsp_log[32][8];
  volatile uint8_t  rsp_log_idx;

  /* LIN USART 中断/发送计数 */
  volatile uint32_t lin_irq[2];
  volatile uint32_t lin_txe[2];
  volatile uint32_t lin_tc[2];
  volatile uint32_t lin_txd[2];
  volatile uint32_t lin_cr1[2];
  volatile uint32_t lin_isr[2];
  volatile uint32_t lin_brr[2];
  volatile uint32_t lin_cr2[2];
  volatile uint32_t lin_cr3[2];
  volatile uint32_t lin_nvic;

  /* CAN 数据 OUT 回调与解析记录（定位丢帧） */
  volatile uint8_t  can_out_log[32][8];
  volatile uint8_t  can_out_len[32];
  volatile uint8_t  can_out_idx;
  volatile uint8_t  can_rec_log[32][8];
  volatile uint8_t  can_rec_size[32];
  volatile uint8_t  can_rec_idx;
};

extern struct pcan_usb_dbg_s g_usb_dbg;

void pcan_usb_dbg_out( uint8_t ep, const uint8_t *data, uint16_t size );
void pcan_usb_dbg_in( uint8_t ep );
void pcan_usb_dbg_cmd( const uint8_t *data, uint16_t size );
void pcan_usb_dbg_rsp( const uint8_t *data, uint16_t size );
void pcan_usb_dbg_can_out( uint8_t ep, const uint8_t *data, uint16_t size );
void pcan_usb_dbg_can_rec( const uint8_t *rec, uint16_t size );

#endif
