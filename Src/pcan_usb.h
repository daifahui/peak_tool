#pragma once
#include <stdint.h>

/* PCAN-USB Pro FD 复合设备：MI_00 = CAN，MI_01 = LIN */
#define PCAN_USB_EP_CMDOUT  0x01
#define PCAN_USB_EP_CMDIN   0x81
#define PCAN_USB_EP_MSGOUT  0x02
#define PCAN_USB_EP_MSGIN   0x82
#define PCAN_USB_EP_MSG1OUT 0x03
#define PCAN_USB_EP_MSG1IN  0x83

#define PCAN_USB_EP_SIZE    64

struct t_class_data
{
  uint16_t  ep_tx_data_pending[15];
  uint8_t   ep_tx_in_use[15];
  uint8_t   buffer_cmd[PCAN_USB_EP_SIZE];
  uint8_t   buffer_data[PCAN_USB_EP_SIZE];
  uint8_t   buffer_data1[PCAN_USB_EP_SIZE];
  uint8_t   buffer_lin_cmd[PCAN_USB_EP_SIZE];
  uint8_t   buffer_lin_data0[PCAN_USB_EP_SIZE];
  uint8_t   buffer_lin_data1[PCAN_USB_EP_SIZE];
};

struct t_m2h_fsm
{
  uint8_t   state;
  uint8_t   ep_addr;
  uint8_t   *pdbuf;
  int       dbsize;
  uint32_t  total_tx;
};

int pcan_flush_ep( uint8_t ep );
int pcan_flush_data( struct t_m2h_fsm *pfsm, void *src, int size );

uint16_t pcan_usb_send_command_buffer( const void *p, uint16_t size );
uint16_t pcan_usb_send_data_buffer( const void *p, uint16_t size );
void pcan_usb_init( void );
void pcan_usb_poll( void );
