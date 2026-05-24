# ATK-MS53L0M 激光测距模块接入说明

本文档记录 ATK-MS53L0M 激光测距模块在 STM32/CubeMX 工程中的典型接入方式、串口协议、查询流程和移植要点，便于在不同 MCU 工程中复用。

## 1. 硬件连接

模块线序以正点原子 ATK-MS53L0M 用户手册为准：

| 模块线色 | 引脚 | 连接到 STM32 |
| --- | --- | --- |
| 红色 | VCC | 3.3V 或 5V，推荐 5V |
| 黑色 | GND | GND |
| 黄色 | TX | MCU 串口 RX，例如 USART1_RX / PA10 |
| 白色 | RX | MCU 串口 TX，例如 USART1_TX / PA9 |
| 蓝色 | SCL | 仅 IIC 模式使用，串口模式不接 |
| 绿色 | SDA | 仅 IIC 模式使用，串口模式不接 |

注意事项：

- 模块串口电平是 3.3V TTL，可直接接入 3.3V TTL 电平的 MCU 串口。
- TX/RX 必须交叉连接：模块 TX -> MCU RX，模块 RX -> MCU TX。
- 示例连接可使用 `USART1`：PA9 为 TX，PA10 为 RX；实际工程按所选 UART 和引脚调整。
- 若同时使用 OLED，可将其放在独立 I2C 引脚上，避免与串口引脚冲突。

## 2. 串口参数

ATK-MS53L0M 默认串口参数：

```text
波特率：115200
数据位：8
停止位：1
校验位：None
流控：None
```

如果 CubeMX 生成的 `MX_USARTx_UART_Init()` 默认波特率不是 115200，建议优先在 CubeMX 中把对应 UART 配置为 115200。也可以在用户代码区提供 `Lidar_ReconfigureUart()`，并在初始化阶段调用，将 `huartx.Init.BaudRate` 重设为 115200。

为避免重新生成代码时被覆盖，手动修改应放在 `USER CODE BEGIN ...` / `USER CODE END ...` 区域内。

## 3. 工作模式与关键结论

模块支持 Normal、Modbus、IIC 三种工作模式。

常见实测现象是，模块串口可能会自动输出：

```text
State:0 , Range Valid
```

这不是距离值，而是测量状态。`State:0` 表示 `Range Valid`，即测距状态有效。如果直接从文本里提取数字，会错误地把状态码 `0` 当成距离，导致显示端固定显示 `0 cm`。

正确做法是：主动发送“读取距离值”的查询命令，解析模块返回帧中的距离字段。

## 4. 读取距离命令

手册中“测量数据”功能码是 `0x05`，读取距离值，两字节，高位在前，单位 mm。

设备默认地址为 `0x0001`。读取距离命令为：

```text
51 0A 00 01 00 05 02 00 63
```

字段含义：

| 字节 | 含义 |
| --- | --- |
| `51` | 主机发送帧头 |
| `0A` | 帧长度相关字段，示例固定为 0x0A |
| `00 01` | 设备地址，默认 0x0001 |
| `00` | 读操作 |
| `05` | 功能码：读取距离 |
| `02` | 数据长度：2 字节 |
| `00 63` | 校验和 |

建议每 200 ms 发送一次该命令，相当于 5 Hz 查询，通常可与模块默认回传速率匹配。

## 5. 返回帧格式

实测返回示例：

```text
55 0A 00 01 00 00 05 02 01 20 00 B9
```

字段解析：

| 字节 | 含义 |
| --- | --- |
| `55` | 模块返回帧头 |
| `0A` | 帧长度相关字段 |
| `00 01` | 设备地址 |
| `00` | 读操作返回 |
| `00` | 状态码，0x00 表示正常 |
| `05` | 功能码：距离值 |
| `02` | 数据长度：2 字节 |
| `01 20` | 距离值，高位在前 |
| `00 B9` | 校验和 |

距离计算：

```c
distance_mm = ((uint16_t)frame[8] << 8) | frame[9];
distance_cm = (distance_mm + 5) / 10;
```

以上示例中：

```text
0x0120 = 288 mm ~= 29 cm
```

## 6. 校验和算法

手册示例使用简单累加和，校验字段是两个字节，高位在前。

校验计算范围：从帧头开始，到校验字段前一个字节结束。

示例返回帧：

```text
55 0A 00 01 00 00 05 02 01 20 00 B9
```

累加：

```text
55 + 0A + 00 + 01 + 00 + 00 + 05 + 02 + 01 + 20 = 00B9
```

C 语言校验函数：

```c
static uint8_t Lidar_FrameChecksumValid(const uint8_t *frame, uint8_t frameLength)
{
  uint8_t i;
  uint16_t checksum = 0U;
  uint16_t receivedChecksum;

  if (frameLength < 4U)
  {
    return 0U;
  }

  for (i = 0U; i < (uint8_t)(frameLength - 2U); i++)
  {
    checksum = (uint16_t)(checksum + frame[i]);
  }

  receivedChecksum = ((uint16_t)frame[frameLength - 2U] << 8U) | frame[frameLength - 1U];
  return (checksum == receivedChecksum) ? 1U : 0U;
}
```

## 7. 推荐的软件结构

建议将传感器逻辑封装成以下几个函数，便于移植到不同工程：

```c
static HAL_StatusTypeDef Lidar_ReconfigureUart(void);
static void Lidar_Task(void);
static void Lidar_SendDistanceQuery(void);
static void Lidar_ProcessRxByte(uint8_t byte);
static uint8_t Lidar_TryParseDistanceFrame(uint16_t *distanceMm);
static uint8_t Lidar_FrameChecksumValid(const uint8_t *frame, uint8_t frameLength);
static void Lidar_DiscardRxBytes(uint8_t count);
```

主循环只需要周期调用：

```c
while (1)
{
  Lidar_Task();
}
```

`Lidar_Task()` 内部完成：

1. 每 200 ms 发送一次读取距离命令。
2. 轮询接收 UART 字节。
3. 将字节放入接收缓冲区。
4. 从缓冲区查找 `0x55` 返回帧。
5. 校验返回帧累加和。
6. 提取 `frame[8]` 和 `frame[9]` 组成距离，单位 mm。
7. 转换成 cm 后显示到 OLED。

## 8. 关键代码片段

查询命令：

```c
static void Lidar_SendDistanceQuery(void)
{
  static const uint8_t query[] = {
    0x51U, 0x0AU, 0x00U, 0x01U, 0x00U, 0x05U, 0x02U, 0x00U, 0x63U
  };

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)query, sizeof(query), 20U);
}
```

解析距离返回帧：

```c
if ((frameLength >= 12U) &&
    (Lidar_RxBuffer[5] == 0x00U) &&
    (Lidar_RxBuffer[6] == 0x05U) &&
    (Lidar_RxBuffer[7] == 0x02U))
{
  distanceMm = ((uint16_t)Lidar_RxBuffer[8] << 8U) | Lidar_RxBuffer[9];
}
```

显示厘米：

```c
distanceCm = (uint16_t)((distanceMm + 5U) / 10U);
```

## 9. 移植到其他项目的步骤

1. 在 CubeMX 中启用一个 UART，例如 USART1。
2. 配置 UART 为 `115200, 8N1, no flow control`。
3. 确认模块 TX 接 MCU RX，模块 RX 接 MCU TX。
4. 在工程中准备一个 `uint8_t` 接收缓冲区，长度建议 64 字节。
5. 周期发送读取距离命令：

   ```text
   51 0A 00 01 00 05 02 00 63
   ```

6. 接收返回数据并寻找 `0x55` 帧头。
7. 根据第二字节计算帧长：

   ```c
   frameLength = frame[1] + 2;
   ```

8. 对整帧做累加和校验。
9. 检查：

   ```text
   frame[5] == 0x00
   frame[6] == 0x05
   frame[7] == 0x02
   ```

10. 提取距离：

    ```c
    distanceMm = ((uint16_t)frame[8] << 8) | frame[9];
    ```

11. 如果只需要 cm 精度：

    ```c
    distanceCm = (distanceMm + 5) / 10;
    ```

## 10. 常见问题

### OLED 固定显示 0

原因通常是把状态文本里的 `State:0` 误解析成距离。

解决方法：不要从 `State:0 , Range Valid` 文本中提取距离，必须主动发送 `0x05` 功能码读取距离。

### 收不到距离，只显示 No distance

检查：

- UART 波特率是否为 115200。
- TX/RX 是否交叉连接。
- 模块供电是否稳定。
- 模块地址是否仍是默认 `0x0001`。
- 是否把模块改成了 IIC 模式。IIC 模式下串口协议不可用。

### 只能看到 State:0 , Range Valid

这是状态信息，不是距离。说明模块正在工作并且测距状态有效，但 MCU 仍需要主动发送读取距离命令。

### 距离跳动或不准

检查：

- 目标物体是否太暗、太透明、反光太强或角度太斜。
- 距离是否小于 40 mm 或超过有效量程。
- 模块镜片是否干净。
- 是否需要按手册进行 10 cm 白色目标校准。

## 11. 参考验证记录

以下内容为一次参考验证环境，实际工程可按所用 MCU、调试器和 IDE 调整。

参考工具链：

```text
Keil uVision 5
OpenOCD xPack
ST-LINK V2
STM32F103C8T6
```

参考 ST-LINK 调试结果：

```text
Target voltage: about 3.23 V
Cortex-M3 detected
Flash verified OK
```

参考串口返回：

```text
55 0A 00 01 00 00 05 02 01 20 00 B9
```

参考解析结果：

```text
0x0120 = 288 mm ~= 29 cm
```

参考 Keil 构建结果：

```text
0 Error(s), 0 Warning(s)
```

## 12. CubeMX 工程注意事项

在 CubeMX 工程中，业务代码必须写在以下区域中：

```c
/* USER CODE BEGIN ... */
/* USER CODE END ... */
```

传感器代码建议放在 `main.c` 的用户代码区内，例如：

- `USER CODE BEGIN Includes`
- `USER CODE BEGIN PD`
- `USER CODE BEGIN PV`
- `USER CODE BEGIN PFP`
- `USER CODE BEGIN 2`
- `USER CODE BEGIN 3`
- `USER CODE BEGIN 4`

这样重新用 CubeMX 生成代码时，传感器逻辑不会被覆盖。

