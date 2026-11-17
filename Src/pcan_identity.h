#pragma once

/* 设备身份 / 协议编译分支
 *
 * PCAN_IDENTITY_SJA1000:
 *   经典 PCAN-USB（0C72:000C），单接口、4 个 bulk 端点，
 *   CAN 使用 SJA1000 命令协议（pcan_protocol.c），无 LIN。
 *
 * PCAN_IDENTITY_PRO_FD:
 *   PCAN-USB Pro FD（0C72:0011），双接口复合设备（MI_00 CAN + MI_01 LIN），
 *   12 个 bulk 端点，CAN 仍复用 SJA1000 命令协议，LIN 走 PLIN 协议。
 */
#if defined( PCAN_IDENTITY_SJA1000 )
  #define PCAN_USB_VID          0x0C72
  #define PCAN_USB_PID          0x000C
  #define PCAN_USE_PLIN         0
  #define PCAN_CAN_USE_UCAN     0
  #define PCAN_USB_IFACE_COUNT  1
  #define PCAN_USB_PRODUCT_NAME "PCAN-USB"
#elif defined( PCAN_IDENTITY_PRO_FD )
  #define PCAN_USB_VID          0x0C72
  #define PCAN_USB_PID          0x0011
  #define PCAN_USE_PLIN         1
  #define PCAN_CAN_USE_UCAN     1
  #define PCAN_USB_IFACE_COUNT  2
  #define PCAN_USB_PRODUCT_NAME "PCAN-USB Pro FD"
#else
  #error "请选择编译身份：-DPCAN_IDENTITY_SJA1000 或 -DPCAN_IDENTITY_PRO_FD"
#endif

/* CAN 接口端点数量：经典 PCAN-USB 只有命令/数据两对端点，
 * Pro FD 复合设备保留 6 个（其中 MSG1 预留）。 */
#if PCAN_USE_PLIN
  #define PCAN_CAN_EP_COUNT  6
#else
  #define PCAN_CAN_EP_COUNT  4
#endif
