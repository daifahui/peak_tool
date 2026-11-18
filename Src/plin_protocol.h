#pragma once
#include <stdint.h>

/* PLIN-USB (MI_01) 端点，映射自 PCAN-USB Pro FD 实测描述符 */
#define PLIN_EP_CMDOUT    0x04
#define PLIN_EP_CMDIN     0x84
#define PLIN_EP_DATA0OUT  0x05
#define PLIN_EP_DATA0IN   0x85
#define PLIN_EP_DATA1OUT  0x06
#define PLIN_EP_DATA1IN   0x86

#define PLIN_EP_SIZE      64
#define PLIN_CHANNELS     2
/* Pro FD 紧凑报文：与 LIN_Write 相同的 24 字节布局，数据在 +16 */
#define PLIN_RX_MSG_SIZE  24
/* 兼容 PLIN-USB 36 字节写报文，OUT 端点接收缓冲按 36 字节挂起 */
#define PLIN_WRITE_MSG_SIZE 36

#define PLIN_MAX_FRAMES       64
#define PLIN_MAX_SLOTS        8
#define PLIN_MAX_SCHEDULES    2
#define PLIN_FRAME_ENTRY_SIZE 16
/* Pro FD 调度槽：schedule(u32) + type + count + delay(u16) + id[8] + handle(u32) */
#define PLIN_SLOT_SIZE        20

/* 一个通道的完整可持久化配置（帧表 + 双调度槽 + 总线参数） */
struct plin_config
{
  uint8_t  frames[PLIN_MAX_FRAMES][PLIN_FRAME_ENTRY_SIZE];
  uint8_t  frame_valid[PLIN_MAX_FRAMES];
  uint8_t  slots[PLIN_MAX_SCHEDULES][PLIN_MAX_SLOTS][PLIN_SLOT_SIZE];
  uint8_t  slot_count[PLIN_MAX_SCHEDULES];
  uint8_t  active_schedule;
  uint16_t baudrate;
  uint8_t  mode;
};

#define PLIN_RX_QUEUE_LEN   32

void plin_protocol_init( void );
void plin_protocol_poll( void );
void plin_protocol_process_command( const uint8_t *buf, uint16_t size );
void plin_protocol_process_data( uint8_t channel, const uint8_t *buf, uint16_t size );
void plin_protocol_on_tx_done( uint8_t channel );
void plin_protocol_report_msg( uint8_t channel, const uint8_t *msg, uint16_t size );
void plin_protocol_set_frame_data( uint8_t channel, uint8_t id, const uint8_t *data, uint8_t length );

/* 最近一次命令/写报文中的设备号与客户端号（RX 报文头需要） */
uint8_t plin_protocol_get_device_no( void );
uint8_t plin_protocol_get_client_no( void );

/* 供 LIN 驱动读取配置 */
int plin_protocol_get_frame( uint8_t channel, uint8_t id, uint8_t *entry );
int plin_protocol_get_slot( uint8_t channel, uint8_t index, uint8_t *slot );
uint8_t plin_protocol_get_slot_count( uint8_t channel );

/* 配置导入/导出（供 Flash 调度表存储使用） */
int plin_protocol_get_config( uint8_t channel, struct plin_config *cfg );
int plin_protocol_set_config( uint8_t channel, const struct plin_config *cfg );
void plin_protocol_set_schedule_active( uint8_t channel, uint8_t active );
