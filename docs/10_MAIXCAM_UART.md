# MaixCam 预留口串口配置

当前底层工程的主控是 MSPM0G3507。资源分配表中的 MaixCam 预留口已经配置为 UART1，参数统一为 `115200, 8-N-1, 无流控`。

## 接线

| MaixCam | 主控预留口 | 方向 |
|---|---|---|
| A19 / UART1_TX | PA9 / UART1_RX | MaixCam 发送到主控 |
| A18 / UART1_RX | PA8 / UART1_TX | 主控发送到 MaixCam |
| GND | GND | 必须共地 |

两端都是 3.3 V TTL 串口，TX 与 RX 必须交叉连接，不要把 5 V 信号直接接到 MaixCam IO。

如果对端换成真正的 STM32，MaixCam 侧配置不变，只需把 A19 接到 STM32 的 USART_RX、A18 接到 STM32 的 USART_TX，并共地；STM32 也设置成 115200、8 数据位、无校验、1 停止位、无硬件流控。

## 程序入口

- MaixCam 示例：`maixcam/uart1_stm32.py`
- 主控驱动：`drivers/stm32_uart.c` 和 `drivers/stm32_uart.h`

主控接收采用 128 字节中断环形缓冲，主循环可用 `MaixCamUART_ReadByte()` 或 `MaixCamUART_Read()` 取数据；发送可用 `MaixCamUART_Write()` 或 `MaixCamUART_WriteString()`。

## 10 ms IMU 数据包

主控以 115200、8N1 每 10 ms 发送一个固定 8 字节小端数据包：

| 偏移 | 字段 | 说明 |
|---:|---|---|
| 0 | `0xAA` | 帧头 1 |
| 1 | `0x55` | 帧头 2 |
| 2 | `SEQ` | 每帧加 1，8 位自然回绕 |
| 3..4 | `AY` | 车头 +Y 方向加速度，`int16_t`，单位 mg，低字节在前 |
| 5..6 | `ANGLE_X` | 绕车左 +X 方向俯仰角，`int16_t`，单位 0.01°，低字节在前 |
| 7 | `XOR` | 第 0 至第 6 字节逐字节异或 |

协议的车头 +Y 对应驱动内部的车体前向 `accel_x_g`，协议的车左 +X 俯仰角对应驱动内部的 `pitch_deg`。角度以 IMU 完成校准时的姿态为零点。

示例：

```c
uint8_t command;

if (MaixCamUART_ReadByte(&command)) {
    /* 处理来自 MaixCam 的命令 */
}

MaixCamUART_WriteString("MSPM0_READY\r\n");
```
