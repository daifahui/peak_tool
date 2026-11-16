# PEAK Tool —— STM32 版 PEAK PCAN + PLIN 兼容工具

把一块 STM32 小板做成 PEAK PCAN-USB Pro FD 的软件替代品：CAN 口能被 **PCAN-View** 直接识别并使用，LIN 口能被 **PLIN-View Pro** 直接识别并使用，LIN 收发由 STM32 的 USART LIN 外设完成（主机/从机均可）。

USB 行为来自对真实 PLIN-USB 盒子的 USBPcap 抓包与 PLinApi 调用逐条对照（命令字、帧表/调度表线格式、36 字节异步 RX 消息、500 ms 状态轮询、NoFunc 认证路径）。

## 当前进度

- [x] M1 工程骨架：以 pcan_cantact 为基础，STM32F072 目标可编译
- [x] M2 USB 复合设备：VID/PID `0C72:0011`，MI_00=CAN、MI_01=LIN，12 个 bulk 端点
- [x] M3 PLIN 命令通道：`0x04/0x84` 命令、`0x18` 状态轮询、`0x85/0x86` 异步 RX 队列
- [x] M4 LIN 主/从驱动：USART1(PA9/PA10)=LIN1、USART2(PA2/PA3)=LIN2；break/报头/数据全部由 TXE/TC 中断发送，主循环不再阻塞
- [ ] M5 与 PLIN-View Pro 联调：设备枚举、连接、初始化、帧表、调度、状态已通过；RX 报文投递待解决

## 引脚分配

| 功能 | 外设 | TX | RX | 备注 |
|---|---|---|---|---|
| LIN1 | USART1 | PA9 | PA10 | AF1，当前未接收发器 |
| LIN2 | USART2 | PA2 | PA3 | AF1，当前未接收发器 |
| CAN | bxCAN | PB9 | PB8 | 复用 pcan_cantact 板型 |
| USB | USB FS | PA12 | PA11 | |

## 目标硬件

- MCU：STM32F072C8/CB（USB FS + bxCAN + USART LIN）
- 板型：基于 CANtact / CANable / Entrée 的 F072 派生板（`make usbcan_f072`）
- 收发器：CAN 用 TJA1050/TJA1051；LIN 用 TJA1021/SN65HVDA195（主机端需 1 kΩ 终端电阻和 12 V VBAT）

## 编译与下载

需要 GNU Arm Embedded Toolchain。固件通过编译宏区分两种设备身份/协议：

| 目标 | 身份 | USB | CAN 协议 | LIN |
|---|---|---|---|---|
| `make usbcan_f072` | PCAN-USB Pro FD | `0C72:0011` 复合设备（2 接口 / 12 端点） | SJA1000 命令协议 | PLIN |
| `make usbcan_f072_sja1000` | 经典 PCAN-USB | `0C72:000C` 单接口（4 端点） | SJA1000 命令协议 | 无 |

编译分支由 `Src/pcan_identity.h` 的 `PCAN_IDENTITY_PRO_FD` / `PCAN_IDENTITY_SJA1000` 控制，
Makefile 中的两个目标分别传入对应宏。

```
make usbcan_f072                # Pro FD 身份，产物在 build-usbcan_f072/
make usbcan_f072_sja1000        # 经典 PCAN-USB 身份，产物在 build-usbcan_f072_sja1000/
```

产物为 `.elf` / `.hex` / `.bin`。

使用 WCH-Link/DAPLink 下载（Keil STM32F0xx DFP pack）：

```powershell
pyocd flash --pack "D:\work_space\mcu\stm32\.daplink_pack\Keil.STM32F0xx_DFP.2.1.0.pack" `
  -t stm32f072cb build-usbcan_f072/pcan_usbcan_f072_hw.hex
```

烧录后设备会重新枚举；PLIN Manager 不会自动重新挂载，需要管理员权限重启一次：

```powershell
Restart-Service -Name 'PLIN Manager' -Force
```

## 兼容性目标

| 上位机 | 接口 | 状态 |
|---|---|---|
| PCAN-View | CAN (MI_00) | Pro FD 身份下不可用（驱动报错 10）；经典身份固件 `usbcan_f072_sja1000` 可被 PCAN-USB 驱动识别 |
| PLIN-View Pro 3.1.x | LIN (MI_01) | 枚举/连接/初始化/帧表/调度/状态通过；RX 投递待解决 |

## 实现说明

- LIN 发送采用 USART TXE/TC 中断状态机：主机调度、从机应答都只把字节序列交给发送引擎，
  主循环不再按字节忙等，USB 命令/数据通道始终有人服务。
- LIN 接收由 RXNE 中断写入 128 字节环形缓冲，主循环解析；主机等待应答带超时窗口。
- LIN→USB 上报走每通道 32 条 × 36 字节的异步队列，USB IN 提交失败时保留队首重试，队列满只累加溢出计数。
- 时间戳使用 TIM2 1 MHz 计数器（见下）。

### Flash 离线调度表

- 链接脚本把末尾 4 KB（`0x0801F000`–`0x0801FFFF`，2 个 2 KB 页）留给调度表，每通道一页。
- 单表有效载荷 1184 B（表头 32 B + 64 帧 × 16 B + 8 槽 × 16 B），2 KB 页余量约 40%，空间足够。
- 表头含 magic/version/波特率/模式/帧数/有效位图/autostart，整表 CRC32 校验；掉电写入半途的表会被判为无效。
- 上电时 `plin_store_init()` 自动校验并把有效表载入对应通道；`autostart=1` 的表立即启动调度，无需上位机。
- 自定义命令（PEAK 未占用区间）：
  - `0x30 STORE_SCHEDULE`：`+4`=表号(0/1)，`+5`=autostart，保存当前通道配置。
  - `0x31 LOAD_SCHEDULE`：`+4`=表号，载入并立即启动。
  - `0x32 GET_SCHEDULE_INFO`：返回 8 字节 `[valid0][valid1][auto0][auto1][mode0][mode1][slots0][slots1]`。

## 已知问题 / 待办

### 阻塞项

- **PLIN RX 投递**：0x85 的 36 字节报文线上内容已正确（PID/长度/方向/数据/序号/时间戳），但当前
  `PLinMng 3.0.0.219`（驱动 `1.7.0.1122`）不把报文投递给客户端；抓包文档环境是
  `3.0.1.227 / 1.7.1.1147`，怀疑版本差异。需要真实 PLIN 盒子在当前环境抓一次 0x85 对比，或升级 PEAK 驱动。
- **CAN / PCAN-View**：MI_00 仍是 PCAN-USB 的 SJA1000 协议；Pro FD 身份下驱动无法启动。
  需要适配 Pro FD 的 CAN 协议，或重新考虑设备身份方案。

### LIN / PLIN 功能

- 未接 LIN 收发器，主/从实际总线收发未验证；主机端 1 kΩ 终端电阻和 12 V VBAT 未接。
- `LIN_Write`（Simple Master）走 Data OUT 管道（`0x05/0x06`），当前固件未解析。
- 仅支持单个调度表（忽略 scheduleIndex 0..7）；未实现 `GET_SLOT (0x13)`、`SET_SCHEDULE_BREAK (0x14)`。
- 未实现：`START_AUTOBAUD (0x04)`、`SET_HW_NAME (0x09)`、KeepAlive（`0x0D`-`0x0F`）、
  `GET_HARDWARE_VERSION (0x25)`、`GET_TARGET_TIME (0x26)`、`RESPONSE_REMAP` 生效、
  `LEDS_ON_OFF (0x28)`、`BREAK_LEN (0x29)` / `TERMINATION (0x2A)`、高级从机/Flash 模式命令。
- `SET_ID_FILTER` 只保存未生效；从机应答重映射未实现。
- 错误位映射需修正：校验错误应为 `0x020`，当前从机路径用了 `0x004`；同步/奇偶校验未检查。
- 时间戳使用 TIM2 1 MHz 自由运行计数器（32 位 µs，无中断，约 71.6 分钟回绕）。
- 调试点 `plin_lin_debug` / `plin_proto_debug` 暂留，后续可移除或加宏开关。
- F072 的 USB PMA 实际可用区约 `0x2FC`，EP3/EP6 OUT 与 EP2/EP5 共享缓冲；若后续使用 CAN2 需重新分配。

### 工程

- `.github/workflows` 只做编译，不检查 pack/烧录。
- 授权提醒：PEAK 名称与协议仅用于兼容性研究，商用/量产前请确认授权。

## 参考与授权

- CAN 部分基于 [moonglow/pcan_cantact](https://github.com/moonglow/pcan_cantact)。
- 授权：WTFPL，见 `LICENSE`。
- PEAK 名称与协议仅用于兼容性研究；商用/量产前请自行确认授权。
