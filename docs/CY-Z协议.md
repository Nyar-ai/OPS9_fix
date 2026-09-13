# CY-Z 通信协议

## 选择串口或 I2C

示例工程通过一个带上拉的 GPIO 选择通信方式：

| 选择引脚电平 | 通信方式 |
|---|---|
| 高电平或悬空 | 串口，默认方式 |
| 低电平，接 GND | I2C |

容易踩坑：

- 选择引脚已经配置内部上拉，悬空时默认使用串口。
- 示例主循环会读取选择引脚，再调用 `CYZ_ReadAngle()`；切换电平后会自动改用对应接口。
- 串口方式由模块主动上报；I2C 方式由主机周期性读取，两者的数据产生方式不同。
- 只使用底层 I2C 接口时，需要主动、周期性调用 `CYZ_I2C_ReadData()`。
- HAL 工程必须先执行 CubeMX 生成的 `MX_I2Cx_Init()`，再调用 `CYZ_Init()`；标准库工程由 `CYZ_Init(CYZ_INTERFACE_I2C)` 初始化 I2C1。
- 选择错误接口时，`CYZ_I2C_*` 会返回 `CYZ_I2C_ERROR_INTERFACE`，串口回调也不会处理数据，便于尽早发现配置错误。
- `CYZ_ZeroAngle()` 和 `CYZ_ReestimateBias()` 会根据所选接口自动走串口或 I2C；其他查询、比例校准和升级接口目前是串口专用。
- 不需要在主循环中反复调用 `CYZ_Init()`；`CYZ_ReadAngle()` 检测到接口变化时会完成切换。

## 串口参数

- 接口：USART1
- 波特率：115200
- 数据位：8
- 校验位：无
- 停止位：1
- 流控：无

## 数据读取

CY-Z 使用固定 16 字节二进制帧输出陀螺仪角度和角速度数据。一帧内同时包含当前积分角度和滤波后的输出角速度。

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|---:|---:|---|---|---|
| 0 | 2 | Header | uint8[2] | 固定 `AA 55` |
| 2 | 2 | Seq | uint16 little-endian | 帧序号，每发送一帧加 1，溢出后回到 0 |
| 4 | 4 | AngleDeg | float32 little-endian | 陀螺仪积分角度，单位 deg |
| 8 | 4 | GyroDps | float32 little-endian | 陀螺仪角速度，单位 deg/s |
| 12 | 2 | CRC16 | uint16 little-endian | CRC-16/MODBUS |
| 14 | 2 | Tail | uint8[2] | 固定 `55 AA` |

帧结构：

```text
AA 55  Seq[2]  AngleDeg[4]  GyroDps[4]  CRC16[2]  55 AA
```

CRC 计算范围：

```text
Seq[2] + AngleDeg[4] + GyroDps[4]
```

CRC 不包含 `Header`、`CRC16` 和 `Tail`。

## CRC16 规则

- 算法：CRC-16/MODBUS
- 初值：`0xFFFF`
- 多项式：`0xA001`
- 输出：低字节在前，高字节在后

## Python 解析示例

```python
import struct


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


frame = bytes.fromhex("AA 55 01 00 00 00 80 3F 00 00 00 00 2A 07 55 AA")

if len(frame) == 16 and frame[0:2] == b"\xAA\x55" and frame[14:16] == b"\x55\xAA":
    payload = frame[2:12]
    crc_recv = struct.unpack_from("<H", frame, 12)[0]
    if crc16_modbus(payload) == crc_recv:
        seq, angle_deg, gyro_dps = struct.unpack_from("<Hff", frame, 2)
        print(seq, angle_deg, gyro_dps)
```

## 下行命令帧

上位机向 CY-Z 发送固定 8 字节命令帧，用于读取数据、角度清零和校准。

```text
A5 5A  Cmd[1]  Param[1]  Seq[1]  CRC16[2]  5A
```

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|---:|---:|---|---|---|
| 0 | 2 | Header | uint8[2] | 固定 `A5 5A` |
| 2 | 1 | Cmd | uint8 | 命令号 |
| 3 | 1 | Param | uint8 | 命令参数 |
| 4 | 1 | Seq | uint8 | 命令序号，响应帧原样返回 |
| 5 | 2 | CRC16 | uint16 little-endian | CRC-16/MODBUS |
| 7 | 1 | Tail | uint8 | 固定 `5A` |

CRC 计算范围：

```text
Cmd[1] + Param[1] + Seq[1]
```

## 命令定义

| Cmd | Param | 功能 | 静止要求 | 响应 |
|---:|---:|---|---|---|
| `0x01` | `0x01` | 角度清零，只清除当前积分角度 | 必须静止 | ACK 帧 |
| `0x01` | `0x02` | 零偏校准，重新估计软件零偏并清除积分角度 | 必须静止 | ACK 帧 |
| `0x02` | `0x00` | 查询当前上报频率 | 无 | ACK 帧，Result 为频率值 |
| `0x03` | 见下表 | 设置上报频率 | 无 | ACK 帧 |
| `0x04` | `0x00` | 读取陀螺仪角度和角速度 | 无 | 16 字节数据帧 |
| `0x05` | `1/2/3/6` | 开始比例校准，参数为旋转圈数 | 开始前必须静止 | ACK 帧 |
| `0x06` | `0x00` | 完成比例校准并保存因子 | 完成时必须静止 | ACK 帧 |
| `0x07` | `0x00` | 取消比例校准，不保存 | 无 | ACK 帧 |
| `0x08` | `0x00` | 读取当前比例因子 | 无 | 比例因子响应帧 |

上报频率参数：

| Param | 上报方式 |
|---:|---:|
| `0x00` | 50 Hz |
| `0x01` | 20 Hz |
| `0x02` | 10 Hz |
| `0x03` | 询问模式，不主动上报 |
| `0x04` | 100 Hz |
| `0x05` | 200 Hz |

## ACK 帧

CY-Z 收到有效命令并执行后，返回固定 8 字节 ACK 帧。

```text
A5 5B  Cmd[1]  Result[1]  Seq[1]  CRC16[2]  5B
```

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|---:|---:|---|---|---|
| 0 | 2 | Header | uint8[2] | 固定 `A5 5B` |
| 2 | 1 | Cmd | uint8 | 原命令号 |
| 3 | 1 | Result | uint8 | 执行结果 |
| 4 | 1 | Seq | uint8 | 原命令序号 |
| 5 | 2 | CRC16 | uint16 little-endian | CRC-16/MODBUS |
| 7 | 1 | Tail | uint8 | 固定 `5B` |

CRC 计算范围：

```text
Cmd[1] + Result[1] + Seq[1]
```

Result 定义：

| Result | 含义 |
|---:|---|
| `0x00` | 成功 |
| `0x01` | 设备未静止，拒绝执行 |
| `0x02` | 命令或参数错误 |
| `0x03` | 执行失败 |

## 角度清零

角度清零用于把当前积分角度置为 `0 deg`，不重新估计零偏。

命令：

```text
Cmd = 0x01
Param = 0x01
```

执行要求：

- 发送命令前保持 CY-Z 静止。
- 清零成功后，后续数据帧中的 `AngleDeg` 从 `0 deg` 附近重新积分。
- 如果设备未静止，返回 `Result=0x01`，角度不清零。

## 零偏校准

零偏校准用于重新估计陀螺仪静止零偏，并同时清除当前积分角度。

命令：

```text
Cmd = 0x01
Param = 0x02
```

执行要求：

- 校准期间必须保持 CY-Z 静止。
- 校准过程约持续 2 秒，期间数据输出可能短暂停顿。
- 校准成功后，后续角速度输出以新的零偏为基准，`AngleDeg` 从 `0 deg` 附近重新积分。
- 如果设备未静止或校准失败，返回对应错误码。

## 比例校准

比例校准用于修正角速度比例因子，使积分角度更接近实际旋转角度。

流程：

1. 保持 CY-Z 静止，发送开始比例校准命令 `Cmd=0x05`。
2. `Param` 选择校准圈数：`1`、`2`、`3` 或 `6`。
3. 收到成功 ACK 后，按选定圈数连续同方向旋转。
4. 旋转完成后停止并保持静止，发送完成比例校准命令 `Cmd=0x06, Param=0x00`。
5. CY-Z 根据累计角度计算并保存新的比例因子。
6. 建议发送 `Cmd=0x08, Param=0x00` 读取当前比例因子，确认保存结果。

计算逻辑：

```text
measured = abs(累计角度)
expected = 圈数 * 360.0
new_factor = old_factor * expected / measured
```

失败条件：

- 开始或完成校准时设备未静止。
- 测量角度明显偏离目标角度。
- 新比例因子超出允许范围。
- 校准过程中发送了取消命令 `Cmd=0x07`。

## 比例因子响应帧

读取比例因子命令 `Cmd=0x08` 返回固定 12 字节比例因子响应帧。

```text
A5 5C  Cmd[1]  Seq[1]  GyroScaleFactor[4]  Reserved[1]  CRC16[2]  5C
```

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|---:|---:|---|---|---|
| 0 | 2 | Header | uint8[2] | 固定 `A5 5C` |
| 2 | 1 | Cmd | uint8 | 原命令号，当前为 `0x08` |
| 3 | 1 | Seq | uint8 | 原命令序号 |
| 4 | 4 | GyroScaleFactor | float32 little-endian | 当前角速度比例因子 |
| 8 | 1 | Reserved | uint8 | 固定 `0x00` |
| 9 | 2 | CRC16 | uint16 little-endian | CRC-16/MODBUS |
| 11 | 1 | Tail | uint8 | 固定 `5C` |

CRC 计算范围：

```text
Cmd[1] + Seq[1] + GyroScaleFactor[4] + Reserved[1]
```

# CY-Z I2C 寄存器协议

## I2C 总线配置

CY-Z 作为 I2C 从机，STM32 作为 I2C 主机。

| 项目 | 配置 |
|---|---|
| 从机 7-bit 地址 | `0x42` |
| 推荐总线速率 | 不高于 `400 kHz` |
| 寄存器地址长度 | 1 字节 |
| 多字节字段字节序 | little-endian，低字节在前 |
| 读寄存器方式 | 写 1 字节寄存器地址，再 repeated-start 连续读取 |

注意：

- `0x42` 是 7-bit 地址，不包含读写位。
- STM32 HAL 的 `HAL_I2C_Mem_Read/Write()` 要传左移一位后的地址，即 `0x42 << 1`。
- MSPM0 DriverLib 接口直接传 7-bit 地址 `0x42`，不要手动左移。
- SDA 和 SCL 是开漏信号，必须外接上拉电阻，常用 `2.2 kΩ ~ 4.7 kΩ` 上拉到双方兼容的 I/O 电压。
- 上拉电阻应装在主机板一侧。否则拔掉模块后，主机的 SDA、SCL 会悬空，容易一直判断为总线忙。
- STM32 与 CY-Z 必须共地。
- 完整数据块应一次连续读取，不能拆成多次单寄存器读取，否则更新瞬间可能得到不同采样周期的数据和 CRC。

## I2C 访问时序

读取寄存器：

```text
START
发送从机地址 0x42 + Write
发送 1 字节寄存器地址
REPEATED START
发送从机地址 0x42 + Read
连续读取 N 字节
STOP
```

读取身份字节：

```text
I2C write 0x42: [0x00]
I2C read  0x42: 1 byte
返回: 0x71
```

读取完整数据块：

```text
I2C write 0x42: [0x00]
I2C read  0x42: 20 bytes
返回寄存器 0x00 ~ 0x13
```

如果读取超出当前寄存器表，超出部分返回 `0x00`。

## I2C 寄存器表

| 地址 | 名称 | 类型 | 说明 |
|---:|---|---|---|
| `0x00` | WHO_AM_I | u8 | 固定 `0x71` |
| `0x01` | PROTOCOL_VER | u8 | 当前固定 `0x01` |
| `0x02` | STATUS | u16 | 状态位，低字节在前 |
| `0x04` | DATA_SEQ | u16 | 每次成功采样递增 |
| `0x06` | SAMPLE_HZ | u16 | 当前固定 `500` |
| `0x08` | ANGLE_CDEG | i32 | 积分角度乘以 100，单位 `0.01 deg` |
| `0x0C` | GYRO_CDPS | i32 | 角速度乘以 100，单位 `0.01 dps` |
| `0x10` | TEMP_CENTI_C | i16 | 温度乘以 100，单位 `0.01 ℃` |
| `0x12` | CRC16 | u16 | 对寄存器 `0x00 ~ 0x11` 共 18 字节计算 |
| `0x20` | COMMAND | u8 | 写命令寄存器 |
| `0x21` | COMMAND_STATUS | u8 | 最近一次 I2C 命令状态 |

未定义的空洞寄存器当前返回 `0x00`，主机不能依赖保留地址的含义。

## I2C 数据换算

```text
angle_deg = ANGLE_CDEG / 100.0
gyro_dps  = GYRO_CDPS / 100.0
temp_c    = TEMP_CENTI_C / 100.0
```

`ANGLE_CDEG` 和 `GYRO_CDPS` 随成功采样更新；`DATA_SEQ` 可用于判断是否拿到了新数据。

## I2C STATUS 状态位

| bit | 名称 | 含义 |
|---:|---|---|
| 0 | DATA_VALID | 数据有效 |
| 1 | CALIBRATED | 已完成零偏校准 |
| 2 | ZERO_RATE | 当前处于零速状态 |
| 3 | SENSOR_ERROR | 采样或初始化异常 |
| 4 | SAMPLE_OVERRUN | 采样处理出现过堆积或超时 |

正常读取后至少应满足：

```text
(STATUS & 0x0001) != 0
```

静止时执行零偏重估前，建议先确认：

```text
(STATUS & 0x0004) != 0
```

## I2C 数据块 CRC16

`CRC16` 位于 `0x12`，校验寄存器 `0x00 ~ 0x11` 共 18 字节。

| 项目 | 值 |
|---|---|
| 算法 | CRC-16/MODBUS |
| 初值 | `0xFFFF` |
| 多项式 | `0xA001` |
| CRC 字节序 | little-endian |

推荐主机从 `0x00` 开始一次读取 20 字节，对前 18 字节计算 CRC，并与最后 2 字节比较。

## I2C 命令

命令通过写 `COMMAND` 寄存器触发：

| 写入数据 | 功能 | 说明 |
|---|---|---|
| `[0x20, 0x01]` | 角度归零 | 将当前积分角度清零 |
| `[0x20, 0x02]` | 零偏重估 | 静止时重新执行零偏校准 |

命令收到后，`COMMAND_STATUS` 先变为 `BUSY`，耗时动作在主循环中执行。零偏重估可能持续约 2 秒。

`COMMAND_STATUS` 定义：

| 值 | 名称 | 含义 |
|---:|---|---|
| `0x00` | OK | 命令完成或最近一次命令成功 |
| `0x01` | BUSY | 命令已接收，正在执行或等待执行 |
| `0x02` | PARAM | 命令值非法 |
| `0x03` | MOVING | 当前不满足静止条件 |
| `0x04` | FAILED | 命令执行失败 |

触发角度归零：

```text
I2C write 0x42: [0x20, 0x01]
I2C write 0x42: [0x21]
I2C read  0x42: 1 byte
```

触发零偏重估后，应循环读取 `0x21`：

```text
0x01: 继续等待
0x00: 成功
0x03: 设备未静止
0x04: 执行失败
```

## 主机推荐读取流程

1. 从寄存器 `0x00` 开始连续读取 20 字节。
2. 检查 `WHO_AM_I == 0x71`。
3. 检查 `PROTOCOL_VER == 0x01`。
4. 对前 18 字节计算 CRC16。
5. 检查 `STATUS.DATA_VALID`。
6. 用 `DATA_SEQ` 判断数据是否更新。
7. 按有符号 little-endian 整数解析角度、角速度和温度。

本仓库主机工程提供以下接口：

```c
CYZ_I2C_Data data;
CYZ_I2C_Result result;

result = CYZ_I2C_ReadData(&data);
if (result == CYZ_I2C_OK) {
    /* data.angle_deg、data.gyro_dps、data.temp_c 可直接使用 */
}
```

命令示例：

```c
uint8_t command_status;

if (CYZ_I2C_ZeroAngle() == CYZ_I2C_OK) {
    do {
        /* 实际工程应在两次查询间适当延时 */
        (void)CYZ_I2C_ReadCommandStatus(&command_status);
    } while (command_status == 0x01U);
}
```

## 各主机工程 I2C 引脚

| 工程/芯片 | I2C 外设 | SCL | SDA | 接口选择 | 速率 |
|---|---|---|---|---|---:|
| STM32F103C8Tx | I2C1 | PB6 | PB7 | PA0 | 100 kHz |
| STM32F407VETx | I2C1 | PB6 | PB7 | PA0 | 100 kHz |
| STM32F407ZGTx | I2C3 | PA8 | PC9 | PA0 | 100 kHz |
| STM32F427IIHx（A 板） | I2C2 | PF1 | PF0 | PF10 | 100 kHz |
| STM32F407 标准库工程 | I2C1 | PB6 | PB7 | PA0 | 100 kHz |
| MSPM0G3507 | I2C0 | PA1 | PA0 | PA3 | 100 kHz |

## I2C 断线与恢复

主机读取失败后不能停在本次错误上，应继续周期性重试。当前示例的恢复处理包括：

1. 检测 NACK、仲裁丢失、超时和总线忙。
2. 清除上一次传输留下的完成标志和错误标志。
3. SDA 被拉低时，用 SCL 输出 9 个恢复时钟并生成 STOP。
4. 重新初始化 I2C 外设。
5. 下一次主循环继续读取，重新插线后恢复通信。

检查顺序：

1. 先确认主机板一侧的 SDA、SCL 各有上拉电阻。
2. 断开模块时，测量主机 SDA、SCL，空闲电压应接近 3.3V。
3. 确认主机与模块共地。
4. 确认使用 7-bit 地址 `0x42`，只在 STM32 HAL 参数中左移一位。
5. 读取寄存器时必须使用 repeated-start，不能在写寄存器地址后等待总线完全空闲。
