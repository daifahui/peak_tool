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
#define PLIN_RX_MSG_SIZE  36

void plin_protocol_init( void );
void plin_protocol_poll( void );
void plin_protocol_process_command( const uint8_t *buf, uint16_t size );
void plin_protocol_on_tx_done( uint8_t channel );
void plin_protocol_report_msg( uint8_t channel, const uint8_t *msg, uint16_t size );
