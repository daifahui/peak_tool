#pragma once
#include <stdint.h>

#define PCAN_TICKS_FROM_US( _us ) (((uint32_t)_us*1000u)/42666u)

void pcan_timestamp_init( void );
uint16_t pcan_timestamp_ticks( void );
uint16_t pcan_timestamp_millis( void );

/* 1 MHz 自由运行计数器（TIM2，32 位，µs 分辨率，无中断） */
uint32_t pcan_timestamp_us( void );

/* 由 TIM2 扩展出的 64 位 µs 计数（供 Pro FD uCAN 时间戳使用） */
uint64_t pcan_timestamp_us64( void );
