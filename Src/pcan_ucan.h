#pragma once
#include <stdint.h>

/* PCAN-USB Pro FD (uCAN over USB) 协议实现。
 * 命令通道 EP 0x01，CAN1/CAN2 数据 OUT 为 0x02/0x03，数据 IN 为 0x82。 */
void pcan_ucan_init( void );
void pcan_ucan_poll( void );
void pcan_ucan_process_command( const uint8_t *buf, uint16_t size );
void pcan_ucan_process_data( const uint8_t *buf, uint16_t size, uint8_t channel );
void pcan_ucan_on_tx_done( void );
void pcan_ucan_usb_reset( void );
