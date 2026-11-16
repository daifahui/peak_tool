#pragma once
#include <stdint.h>

/* LIN 通道引脚分配（usbcan_f072 板型上空闲引脚）
 * LIN1: USART1  TX=PA9   RX=PA10  AF1
 * LIN2: USART2  TX=PA2   RX=PA3   AF1
 */

#define PLIN_LIN_MODE_NONE   0
#define PLIN_LIN_MODE_SLAVE  1
#define PLIN_LIN_MODE_MASTER 2

/* 调试计数：[0]主机帧调用 [1]主机帧完成 [2]发送超时 [3]最后 PID
 * [4]最后长度 [5]从机应答 [6]break 发送 [7]最后帧时刻(ms) */
extern volatile uint32_t plin_lin_debug[8];

void plin_lin_init( void );
void plin_lin_poll( void );
void plin_lin_set_config( uint8_t channel, uint16_t baudrate, uint8_t mode );
void plin_lin_set_active( uint8_t channel, uint8_t active );
void plin_lin_send_wakeup( uint8_t channel );

/* LIN_Write：master 模式下排队立即发送一帧；slave 模式由帧表数据应答 */
void plin_lin_write( uint8_t channel, uint8_t pid, uint8_t length, uint8_t direction,
                     uint8_t checksum_type, const uint8_t *data );
