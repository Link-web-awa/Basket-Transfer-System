# RS01 STM32H7 电机控制

使用 STM32H723 的 FDCAN2，通过 RobStride 私有协议控制 RS01。保留 C++ 私有协议、MIT 方法，并提供 `main.c` 可调用的 C 接口。

## 当前版本

- 支持指定电机 ID、正反向圈数及速度的单段位置运动。
- 支持首次配置使能、连续执行相对运动段、整组结束后失能。段间保持使能，避免重复配置造成的停顿。
- 位置运动限流 **1 A**，加速度 **1 rad/s²**，在头文件中统一配置。
- 已实机确认正常 24 V 供电、1 A 限流下可以运动。清理后版本编译、链接通过，尚未重新上板验证。
- 已移除自动往返、固定一圈测试接口、多电机扫描及临时 Watch/报文记录。保留主动确认的单电机 ID 查询。

**当前 `main.c` 上电后只初始化并待机，不发送使能或运动指令。PC13 仍拉高开启板上电机供电。**

## 硬件与构建

| 项目 | 配置 |
| --- | --- |
| MCU | STM32H723VGTx |
| CAN 外设 | FDCAN2 |
| CAN RX / TX | PB5 / PB6，经 CAN 收发器连接总线 |
| 电机电源开关 | PC13，高电平开启 |
| CAN 格式 | 经典 CAN，8 字节；私有协议使用扩展帧 |
| 波特率 | 1 Mbps |
| 时钟与位时序 | PLL1Q 129 MHz；Prescaler=1，Seg1=102，Seg2=26 |
| 主机 ID | `0xFD` |

手册规定使用电压范围为 24～50 V。此前内部母线约 11.975 V 时出现欠压，使用正常 24 V 电源后恢复运动。电源设定值不能代替电机接口实际电压。

```text
Core/                       初始化、主循环、中断
Drivers/                    STM32 HAL 与 CMSIS
RS/Robstride.h              电机 ID、运动配置、C/C++ 接口
RS/Robstride01.cpp          电机驱动、FDCAN 适配、位置运动流程
MDK-ARM/                    CubeMX 生成的 Keil 工程和启动文件
RS01_sample_H7.ioc          CubeMX 配置
```

当前验证环境使用相邻的 `../RS01_sample_H7_EIDE/` 工程和 ARM Compiler 5.06。EIDE 配置 `.eide/eide.yml` 引用本仓库源码，产物位于其 `build/RS01_sample_H7/` 下，包括 `.axf` 和 `.hex`。EIDE 配置目录不属于本 Git 仓库，单独克隆不会得到该配置。

`MDK-ARM/RS01_sample_H7.uvprojx` 尚未包含 `RS/Robstride01.cpp`。使用 Keil 时需把该源文件按 C++ 加入工程并配置头文件路径；`main.c` 保持 C 编译。源码与 README 使用 UTF-8，编译产物由 `.gitignore` 排除。

## 配置

修改 `RS/Robstride.h`：

```c
#define RS01_MOTOR_ID              0x05U
#define RS01_MOTOR_1_ID            0x05U
#define RS01_MOTOR_2_ID            0x07U
#define RS01_MOVE_ACCEL_RAD_S2     1.0f
#define RS01_MOVE_CURRENT_LIMIT_A  1.0f
```

ID 是软件目标地址，不会修改电机内部地址。位置接口显式传入 ID；兼容接口使用 `RS01_MOTOR_ID`。加速度、限流由所有位置会话共用；限流有效范围 `(0, 23] A`，启动时读回校验。速度接口另有独立的限流参数。

速度单位 `rad/s`，不是每秒圈数或 rpm：`rpm = rad/s × 60 / (2π)`。圈数为减速器输出轴圈数，支持小数，正负表示电机坐标增减。应用需根据安装方向、外接传动比和主动轮周长换算机构位移。

## 初始化与调用

主函数完成 HAL、时钟、GPIO、FDCAN2 初始化，配置过滤器并启动 CAN，等待 3 秒，随后循环处理默认 ID 的反馈。

下面的运动示例需由应用显式加入 CAN 启动后的代码。放在无限循环内会反复运动；放在初始化段则每次上电或复位执行一次。

### 单段运动

```c
HAL_StatusTypeDef status = RS01_MoveTurns(0x05U, 1.0f, 0.2f);
if (status != HAL_OK) Error_Handler();
```

函数配置 PP 位置模式、速度、加速度和限流，再执行相对当前位置的运动；阻塞到完成或出错，最后尝试失能。负圈数表示反向，例如 `-0.25f` 为反向 90°。

### 连续运动段

```c
const uint8_t id = RS01_MOTOR_1_ID;
HAL_StatusTypeDef status = RS01_MotionBegin(id, 2.5f);
if (status == HAL_OK) status = RS01_MotionMoveTurns(id, 2.0f);
if (status == HAL_OK) status = RS01_MotionMoveTurns(id, -2.0f);
HAL_StatusTypeDef stop_status = RS01_MotionEnd(id);
if (status != HAL_OK || stop_status != HAL_OK) Error_Handler();
```

`Begin` 只配置、使能一次。各段相对上一个目标累计圈数，避免因每段当前位置容差而逐次漂移；到位后保持位置，下一段只更新目标。`End` 失能后不再保持力矩。

段间仍需减速到位和反馈确认，不保证零时间换向。到位要求位置误差在 ±0.01 rad（约 ±0.57°）内、速度绝对值不超过 0.03 rad/s，连续确认 3 次，正常轮询间隔 20 ms。这是软件判断容差，不代表机械精度指标。

## C 接口

| 接口 | 用途 |
| --- | --- |
| `RS01_MoveTurns(id, turns, speed)` | 单段运动，结束后尝试失能 |
| `RS01_MotionBegin(id, speed)` | 配置并使能，保留会话 |
| `RS01_MotionMoveTurns(id, turns)` | 阻塞执行当前段；成功后保持使能，失败后尝试失能 |
| `RS01_MotionEnd(id)` | 发送失能并释放会话，可作异常清理 |
| `RS01_FindMotorId(&id)` | 单独接入一台电机时主动确认 ID，不运动、不修改配置 |
| `RS01_PollFeedback()` | 处理默认 ID 的反馈 |
| `RS01_ReadPosition(&position)` | 读取默认 ID 的计圈位置，单位 rad |
| `RS01_SpeedControl(speed, current_limit)` | 默认 ID 的持续速度模式，不持续监控堵转 |
| `RS01_SetCspMode()` / `RS01_SetSpeedLimit(speed)` | 默认 ID 的 CSP 配置；速度上限接口当前限制 0.15 rad/s |
| `RS01_Enable()` / `RS01_SetPosition(position)` | 默认 ID 的底层使能、绝对位置指令 |

返回值为 `HAL_OK`、`HAL_ERROR`、`HAL_TIMEOUT`；重复打开同一 ID 或两个会话槽均占用时，`Begin` 返回 `HAL_BUSY`。应用需检查返回值，不应在故障后无条件反复使能。

ID 查询发送前清除排队报文，要求两次回复的地址、8 字节唯一标识一致，再成功读回运行模式。失败时输出 ID 为 `0xFF`。单帧上电广播（包括 `0x7F`）不能直接当作工作 ID；相同 ID 的多台设备需分别接入查询。

## 保护与限制

- 位置流程保留参数读回、使能反馈、电机故障、反向越界、过冲及运动超时检查。参数查询最长 500 ms，期间每 100 ms 重试。停止指令最多尝试 3 次；通信中断时无法保证停止指令到达。
- 运动超时根据圈数、速度、加速度估算；短行程可能达不到速度上限。当前没有机构行程数据，应用必须自行限制可用行程。
- **尚未实现双轴同步、统一反馈分发或共同停机。** 两个会话槽只保存独立状态，运动仍阻塞，接收按当前目标筛选；不能在两个任务并发调用，也不能据此同时驱动机械连接的两条皮带。
- 速度模式没有应用层堵转停机逻辑，`HAL_OK` 不表示运行过程持续正常。电机内置热保护不能替代机构行程和同步误差保护。
- 当前未实现上电回零、堵转寻零或掉电多圈恢复。遗留 C++ `RobStride_Motor_Set_Zero_control()` 的模式值不在当前手册有效模式列表内，不应直接用于装置回零。
- 后续双皮带联动需比较两侧相对起点的实际位移，超差或任一故障时共同停机。讨论中的约 5 mm 容许差仍需主动轮周长、传动比和机构实测确认，本版没有启用该保护参数。
