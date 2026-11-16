#pragma once
#include <stdint.h>

/* Flash 离线调度表：每个 LIN 通道一张表，共 2 张。
 * 存储区：0x0801F000 - 0x0801FFFF（2 个 2 KB 页，链接脚本已保留）。 */

#define PLIN_STORE_TABLES  2

/* 上电校验两张表：有效则载入对应通道的帧表/调度槽；
 * 表头 autostart=1 时立即启动调度，实现无上位机离线收发。 */
void plin_store_init( void );

/* 保存/加载指定通道的当前配置，table 0/1 对应 LIN1/LIN2。 */
int plin_store_save( uint8_t table, uint8_t autostart );
int plin_store_load( uint8_t table, uint8_t start );

/* 8 字节状态：[valid0][valid1][auto0][auto1][mode0][mode1][slots0][slots1] */
void plin_store_info( uint8_t *info );
