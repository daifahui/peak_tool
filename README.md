# PEAK Tool —— STM32 版 PEAK PCAN + PLIN 兼容工具

把一块 STM32 小板做成 PEAK PCAN-USB Pro FD 的软件替代品：CAN 口能被 **PCAN-View** 直接识别并使用，LIN 口能被 **PLIN-View Pro** 直接识别并使用，LIN 收发由 STM32 的 USART LIN 外设完成（主机/从机均可）。

USB 行为来自对真实 PLIN-USB 盒子的 USBPcap 抓包与 PLinApi 调用逐条对照（命令字、帧表/调度表线格式、36 字节异步 RX 消息、500 ms 状态轮询、NoFunc 认证路径）。

## 当前进度

- [x] M1 工程骨架：以 pcan_cantact 为基础，STM32F072 目标可编译
- [x] M2 USB 复合设备：VID/PID `0C72:0011`，MI_00=CAN、MI_01=LIN，12 个 bulk 端点
- [x] M3 PLIN 命令通道：`0x04/0x84` 命令、`0x18` 状态轮询、`0x85/0x86` 异步 RX 队列
- [ ] M4 LIN 主/从：帧表、调度表、Slave 自动应答、`UPDATE_BYTE_ARRAY`
- [ ] M5 与 PLIN-View Pro 联调

## 目标硬件

- MCU：STM32F072C8/CB（USB FS + bxCAN + USART LIN）
- 板型：基于 CANtact / CANable / Entrée 的 F072 派生板（`make usbcan_f072`）
- 收发器：CAN 用 TJA1050/TJA1051；LIN 用 TJA1021/SN65HVDA195

## 编译

需要 GNU Arm Embedded Toolchain：

```
make usbcan_f072
```

产物在 `build-usbcan_f072/`：`.elf` / `.hex` / `.bin`。

## 兼容性目标

| 上位机 | 接口 | 状态 |
|---|---|---|
| PCAN-View | CAN (MI_00) | 沿用 pcan_cantact 实现 |
| PLIN-View Pro 3.1.x | LIN (MI_01) | 开发中 |

## 参考与授权

- CAN 部分基于 [moonglow/pcan_cantact](https://github.com/moonglow/pcan_cantact)。
- 授权：WTFPL，见 `LICENSE`。
- PEAK 名称与协议仅用于兼容性研究；商用/量产前请自行确认授权。
