#pragma once
#include <stdint.h>

/* LIN 通道引脚分配（usbcan_f072 板型上空闲引脚）
 * LIN1: USART1  TX=PA9   RX=PA10  AF1
 * LIN2: USART2  TX=PA2   RX=PA3   AF1
 */

#define PLIN_LIN_MODE_NONE   0
#define PLIN_LIN_MODE_SLAVE  1
#define PLIN_LIN_MODE_MASTER 2

void plin_lin_init( void );
void plin_lin_poll( void );
void plin_lin_set_config( uint8_t channel, uint16_t baudrate, uint8_t mode );
void plin_lin_set_active( uint8_t channel, uint8_t active );
void plin_lin_send_wakeup( uint8_t channel );
