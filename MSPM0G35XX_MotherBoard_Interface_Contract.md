# MSPM0G35XX 主板接口契约

> 用途：作为 GPT 生成/修改代码时的主板接口唯一参考，同时供用户维护。
>
> 硬件：MSPM0G35XX MotherBoard V1.1；原理图修订号 V1.1.1.1（2026-06-10）。
>
> 适用工程：本仓库 `Example/Motherboard_Demo` 下的 MSPM0G3519/MSPM0G3507 例程。
>
> 文档状态：`usable_with_exceptions`。已确认大部分接口；编码器 2 和 DL1B 硬件 IIC 保留明确风险，见第 8 节。

## 0. GPT 使用规则（必须遵守）

1. MCU 引脚名必须写成 `A0`、`A22`、`B13` 这种格式；不要把连接器针脚号当成 MCU 引脚号。
2. 生成 C 代码时优先使用库中完整枚举 token，例如 `PWM_TIM_A0_CH0_A0`、`SPI0_SCK_A12`、`UART7_TX_B15`，不要手写复用功能值。
3. 表格中的“方向”均以 MCU 视角描述：`TX` 是 MCU 输出，`RX` 是 MCU 输入；外设模块的 TX/RX 标签可能与此相反。
4. 一个引脚已经被主板外设占用时，不得在同一应用中再分配给普通 GPIO、另一外设或另一种复用功能，除非明确说明“互斥使用”。
5. 状态为 `CONFLICT` 或 `NEEDS_MEASURE` 的记录不能被 GPT 当作已确认接口。生成代码前应保留 TODO 或先请求实测结果。
6. 修改接口后，必须同时更新“状态、来源、变更记录”；不得只改正文中的某一个数字。
7. 本文档中的表格是接口数据源；说明性文字不能覆盖表格中的 `mcu_pin/token/status`。

### 状态标记

| 状态 | 含义 |
|---|---|
| `CONFIRMED` | 原理图与代码/头文件一致，或已由代码和原理图共同确认。 |
| `CODE_CONFIRMED` | 例程实际使用的宏和初始化已确认；连接器物理对应关系仍需按原理图理解。 |
| `SCHEMATIC_ONLY` | 原理图已确认，但仓库没有对应的完整初始化例程。 |
| `CONFLICT` | 原理图、头文件或例程存在不一致；禁止自动猜测。 |
| `NEEDS_MEASURE` | 需要万用表、示波器或导通测试确认。 |

## 1. 可信来源和版本

按以下优先级核对接口：

1. [主板 V1.1 原理图 PDF](<【原理图】原理图 丝印图 尺寸图 位号图/主板/V1.1/MSPM0G35XX_MotherBoard V1.1.pdf>)，重点看第 2、3、4、5 页。
2. `Example/Motherboard_Demo/*/user/src/main.c` 中实际编译例程的宏和初始化调用。
3. 与目标例程同目录的 `libraries/zf_device/*.h`、`libraries/zf_common/zf_common_debug.h`、`libraries/zf_driver/*.h`。
4. [README.md](README.md) 和 [主板端口介绍图](assets/主板端口介绍.png) 仅用于物理名称和供电说明，不覆盖原理图/代码。

主板例程包含多份复制的 `libraries`。修改某个例程时，应使用该例程旁边的库；本文引用的“基准头文件”是：

`Example/Motherboard_Demo/E1_motherboard/libraries/`

## 2. 电源与连接器总览

| 标识/接口 | 信号或电源 | MCU 引脚/token | 说明 | 状态 |
|---|---|---|---|---|
| P1 电源输入 | `VBAT`, `GND` | 无 | XT60 电池输入；README 推荐直流 7.2-26 V。必须先确认电源极性。 | `SCHEMATIC_ONLY` |
| S1 主电源开关 | `VBAT` 开关 | 无 | 控制主板整体电源。 | `SCHEMATIC_ONLY` |
| P2/P3 电机电源 | `VCCBAT`, `GND` | 无 | 给外部电机驱动模块供电，电压约等于电池电压；不是 MCU PWM 信号。 | `SCHEMATIC_ONLY` |
| P4/P5 舵机接口 | 信号、`VCCServo`、`GND` | 信号分别 `B3`、`B4` | 舵机电源由电位器调节；默认约 6 V。不要把 `VCCServo` 当作 3.3 V GPIO 电源。 | `CONFIRMED` |
| 5 V 电源 | `VCC5V` | 无 | 无线串口、ToF、串口、屏幕等接口使用。 | `CONFIRMED` |
| 3.3 V 电源 | `VCC3V3` | 无 | 按键、蜂鸣器、编码器、姿态传感器和一般屏幕接口使用。 | `CONFIRMED` |
| 摄像头/灰度电源 | `VCCCAMOP` | 无 | 原理图由独立 3.3 V 稳压输出，供 CCD/灰度传感器一类接口使用。 | `CONFIRMED` |

## 3. 主板接口矩阵（逻辑信号）

### 3.1 调试串口、普通串口、无线串口

| 接口 ID | 板上接口 | MCU 方向 | MCU 引脚/token | 默认参数/备注 | 状态 |
|---|---|---|---|---|---|
| `DEBUG_UART` | 核心板 USB-TTL/调试口 | TX/RX | TX `A10`=`UART0_TX_A10`；RX `A11`=`UART0_RX_A11` | `UART_0`，`115200`；由 `debug_init()` 初始化。 | `CONFIRMED` |
| `SERIAL_P8` | 主板“串口接口” P8 | TX/RX | TX `B15`=`UART7_TX_B15`；RX `B16`=`UART7_RX_B16` | P8 同时提供 `VCC5V/GND`；这是 `UART_7`，不是默认调试串口。 | `CONFIRMED` |
| `WIRELESS_UART_P12` | 主板“无线模块接口-串口” P12 | MCU TX -> 模块 RX | `B6`=`UART1_TX_B6` | 模块 TX 线应接 MCU `B5`；模块 RX 线应接 MCU `B6`。 | `CONFIRMED` |
| `WIRELESS_UART_P12` | 同上 | MCU RX <- 模块 TX | `B5`=`UART1_RX_B5` | RTS `B2`；默认波特率 `115200`。 | `CONFIRMED` |
| `WIFI_UART_P12` | 同一 P12，无线 WiFi 串口模式 | MCU TX/RX | TX `B6`，RX `B5` | `UART_1`；RTS `B2`；当前 `WIFI_UART_HARDWARE_RST=0`，默认不占用硬件复位脚。 | `CONFIRMED` |
| `WIFI_UART_RST` | WiFi 串口模块硬件复位（可选） | MCU 输出 | `B3` | 只有将 `WIFI_UART_HARDWARE_RST` 改为 `1` 才启用；会与舵机 1 信号 `B3` 冲突。 | `CONFLICT` |

基准代码：

- `zf_common_debug.h`：`DEBUG_UART_INDEX=UART_0`、TX `A10`、RX `A11`、波特率 `115200`。
- `zf_device_wireless_uart.h`：`UART_1`、TX `B6`、RX `B5`、RTS `B2`。
- `zf_device_wifi_uart.h`：TX `B6`、RX `B5`、RTS `B2`、可选 RST `B3`。
- `zf_driver_uart.h`：P8 使用 `UART7_TX_B15` 和 `UART7_RX_B16`。

### 3.2 屏幕接口 P10

P10 是 SPI 屏幕接口。逻辑信号如下，表中 `DC/INT` 是同一根主板信号在不同屏幕上的不同用途。

| 屏幕信号 | MCU 引脚/token | OLED/TFT180/IPS114/IPS200 | IPS200PRO | 备注 |
|---|---|---|---|---|
| `SCK` | `A12`=`SPI0_SCK_A12` | 使用 | 使用 | SPI0 时钟。 |
| `MOSI/SDA` | `A9`=`SPI0_MOSI_A9` | 使用 | 使用 | SPI0 主机输出。 |
| `MISO` | `A13`=`SPI0_MISO_A13` | 通常不使用 | 使用 | 基本屏幕驱动通常设置 `SPI_MISO_NULL`。 |
| `RST/RES` | `A7` | 使用 | 使用 | GPIO 复位。 |
| `DC` 或 `INT` | `A15` | `DC` | `INT` | 不同屏幕驱动语义不同。 |
| `CS` | `A8` | 使用 | 使用 | GPIO 软件片选。 |
| `BL/BLK` | `A13` | TFT180/IPS114/IPS200 用作背光 | IPS200PRO 不把它当背光 | `A13` 是 `MISO/BL` 复用资源，不可同时按两种用途使用。 |
| 电源 | `VCC5V/GND` | 使用 | 使用 | 以屏幕模块实际规格为准。 |

代码头文件中的默认配置：SPI0、30 MHz、`A12/A9/A7/A15/A8/A13`。对应设备头文件为 `zf_device_oled.h`、`zf_device_tft180.h`、`zf_device_ips114.h`、`zf_device_ips200.h`、`zf_device_ips200pro.h`。

### 3.3 CCD 接口 P7/P9

| 逻辑信号 | MCU 引脚/token | CCD1/CCD2 | 说明 | 状态 |
|---|---|---|---|---|
| `CLK` | `A28` | 共用 | `TSL1401_CLK_PIN=A28`。 | `CONFIRMED` |
| `SI` | `A29` | 共用 | `TSL1401_SI_PIN=A29`。 | `CONFIRMED` |
| `AO` 模拟输出 1 | `A25`=`ADC0_CH2_A25` | CCD1 | 12 位 ADC。 | `CONFIRMED` |
| `AO` 模拟输出 2 | `A24`=`ADC0_CH3_A24` | CCD2 | 12 位 ADC。 | `CONFIRMED` |
| 电源 | `VCCCAMOP/GND` | 共用 | 不要接错为 `VCCServo`。 | `CONFIRMED` |

`TSL1401_DATA_LEN=128`，例程使用 `tsl1401_init(0)` 和 `tsl1401_init(1)`。

### 3.4 灰度传感器接口 P11

主板原理图给出的灰度接口网络为：

| 网络 | MCU 引脚 | GS08RA 驱动用途 | 备注 |
|---|---|---|---|
| `OUT` | `B25`=`ADC0_CH4_B25` | 8 路模拟量输出 | `GS08RA_OUT_PIN`；驱动 ADC 分辨率默认 8 bit。 |
| `S0` | `A16` | 通道选择 | `GS08RA_S0_PIN`。 |
| `S1` | `A17` | 通道选择 | `GS08RA_S1_PIN`。 |
| `S2` | `B17` | 通道选择 | `GS08RA_S2_PIN`。 |
| 额外模拟网络 | `B20`、`B18` | 当前 GS08RA 驱动未使用 | 原理图引脚表列为灰度传感器 ADC 网络；使用前需确认对应传感器型号。 |
| 电源/地 | `VCCCAMOP/GND` | 传感器供电 | 以模块规格为准。 |

重要：`A16/A17/B17/B25` 不能被 GPT 当作普通空闲 GPIO；GS08RA 连接时，前三根是选择线，`B25` 是 ADC 输入。

#### 本车 GS08RA 循迹约定（实测）

- 白底黑线：`gs08ra_bin_val=0` 表示黑线，`1` 表示白底，当前阈值为 `30`。
- 车体从左到右对应通道 `0..7`；实测黑线在左侧为 `BIN=00111111`、中央为 `BIN=11110111`、右侧为 `BIN=11111110`。
- 偏差公式为 `error=left_black_index+right_black_index-7`；左侧为负，右侧为正。
- `error<0` 时必须让物理左轮减速、物理右轮加速；`error>0` 时相反。
- 2026-07-25 新电机复测：本车物理左轮是 `CH1/TIMG8`，物理右轮是 `CH2/TIMG9`，不得沿用旧电机的交叉映射。
- GS08RA 扫描在主循环执行，10ms PIT 回调只执行编码器读取和双轮速度 PI。
- KEY1（`A30`）短按切换运行状态：停止时按下启动，运行时再次按下停止；停止后允许重新启动。
- 无线串口 V2.4 使用 P12：`UART_1`，MCU TX=`B6`、MCU RX=`B5`、RTS=`B2`、`115200` 波特率；接收使用库内中断 FIFO，命令解析在主循环执行。
- 无线控制为单字节协议：`S`/`s`/`1` 启动，`P`/`p`/`0` 停止，`?` 查询运行状态，`H`/`h` 返回帮助；回车和换行会被忽略。
- 连续 150 ms 全白丢线时强制停机；取消固定运行时长不能取消该保护。
- 直角弯在偏差绝对值达到 `5` 并连续确认 20 ms，或黑线从明显偏向的一侧消失时触发。
- 左直角弯使用“左轮反转、右轮正转”，右直角弯相反；转向目标速度绝对值为 `12 count/10ms`。
- 直角转向时暂停普通丢线判断，中间通道 3/4 连续重新看到黑线 30 ms 后恢复普通循迹；2.5 s 内未重新捕线则强制停机。
- 全黑图案不再表示终点，也不会触发停车；当前版本通过 KEY1、无线停止命令、丢线保护或直角转向超时停止。

### 3.5 ToF 接口 P6

| ToF 信号 | MCU 引脚/token | 默认库方式 | 备注 | 状态 |
|---|---|---|---|---|
| `SCL` | `B8` | 软件 IIC `DL1A_SCL_PIN` / `DL1B_SCL_PIN` | DL1A/DL1B 默认配置使用。 | `CONFIRMED` |
| `SDA` | `B26` | 软件 IIC `DL1A_SDA_PIN` / `DL1B_SDA_PIN` | DL1A/DL1B 默认配置使用。 | `CONFIRMED` |
| `IO/XS` | `B14` | `DL1A_XS_PIN` / `DL1B_XS_PIN` | 默认 `INT_ENABLE=0`，通常不依赖中断。 | `CONFIRMED` |
| 电源/地 | `VCC5V/GND` | 无 | ToF 接口不是 3.3 V 电源接口。 | `CONFIRMED` |

DL1B 头文件还提供硬件 IIC 备选：`IIC1_SCL_B8`、`IIC1_SDA_B9`。但 P6 原理图的 SDA 是 `B26`，因此硬件 IIC 备选与主板连接存在不一致；保持 `DL1B_USE_SOFT_IIC=1`，除非已经修改硬件或完成导通确认。

### 3.6 按键和蜂鸣器

| 设备 | MCU 引脚 | 默认电平/逻辑 | 代码宏 |
|---|---|---|---|
| KEY1 | `A30` | 释放高、按下低 | `KEY_LIST[0]` |
| KEY2 | `A31` | 释放高、按下低 | `KEY_LIST[1]` |
| KEY3 | `B0` | 释放高、按下低 | `KEY_LIST[2]` |
| KEY4 | `B1` | 释放高、按下低 | `KEY_LIST[3]` |
| 蜂鸣器 | `A18` | NPN 三极管驱动；代码中低电平初始化，通常高电平响 | `BEEP` |

按键默认消抖 10 ms，长按判定 1000 ms。基准头文件为 `zf_device_key.h`；蜂鸣器例程为 `E1_01_button_buzzer_demo/user/src/main.c`。

## 4. 电机、舵机、编码器

### 4.1 有刷电机驱动信号 P13

主板 P13 输出四路 3.3 V 逻辑 PWM 信号，不提供电机功率。

| 主板 PWM 资源 | MCU token | MCU 引脚 | 常见用途 |
|---|---|---|---|
| PWM0 | `PWM_TIM_A0_CH0_A0` | `A0` | 电机 1 PWM 或 HIP4082 通道 1。 |
| PWM1 | `PWM_TIM_A0_CH1_A1` | `A1` | 电机 1 反向 PWM，或 DRV8701E 电机 1 DIR。 |
| PWM2 | `PWM_TIM_A0_CH2_B12` | `B12` | 电机 2 PWM。 |
| PWM3 | `PWM_TIM_A0_CH3_B13` | `B13` | 电机 2 反向 PWM，或 DRV8701E 电机 2 DIR。 |

例程默认电机 PWM 频率为 17 kHz，库占空比范围为 `0..PWM_DUTY_MAX`，其中 `PWM_DUTY_MAX=10000`。

DRV8701E 双电机例程实际使用：

```c
#define MOTOR1_DIR  (A1)
#define MOTOR1_PWM  (PWM_TIM_A0_CH0_A0)
#define MOTOR2_DIR  (B13)
#define MOTOR2_PWM  (PWM_TIM_A0_CH2_B12)
```

#### 本车闭环通道实测（当前：2026-07-25）

> 新电机的手动编码器、单通道正反转和电机/编码器配对已经完成。以下 2026-07-25 表格是当前唯一有效映射。工程仍启用 `NEW_MOTOR_SAFE_WIRELESS_TEST=1`。开环映射、单/双轮速度 PI 和低速循迹分别保留为 `SAFE_TEST_STAGE_OPEN_LOOP`、`SAFE_TEST_STAGE_SINGLE_WHEEL_PI`、`SAFE_TEST_STAGE_LOW_SPEED_TRACK` 三个可切换阶段。当前启用低速循迹阶段：`S`/`1` 启动并持续运行，`P`/`0` 主动停止，`G` 在停车时读取灰度快照，`?` 查询状态；根据转向修正绝对值分级选择基础速度，修正为0时直道目标 `8 count/10ms`，修正为1时轻弯目标 `7`，修正不小于2时弯道目标 `6`；转向修正上限 `4`、PWM 上限 2000，不设固定运行时长，连续丢线 150 ms 强制停止，不启用终点、陀螺仪或直角弯逻辑。单轮架空首测平均为左 `8.62`、右 `8.64 count/10ms`，无反向和饱和；据末段稳态 PWM 将测试前馈修正为 `250 + 105 × target`。双轮架空测试为左 `8.06`、右 `8.11`，落地直行测试为左 `7.50`、右 `7.52`，稳定阶段均保持约 `8 count/10ms`，实车确认直行。

2026-07-28 更换损坏排线后 GS08RA 静态复测通过：全白 `BIN=11111111, det=0`；黑线在左、中、右分别得到 `BIN=10011111, error=-4`、`BIN=11100111, error=0`、`BIN=11111101, error=+5`。阈值继续使用 `30`，黑白归一化值分离明显。

同日首次5秒低速循迹实测通过：起步 `error=+7` 时目标为左 `10`、右 `2`，约 510 ms 回到中心，短暂越过至约 `error=-3` 后稳定；全程无丢线、无持续摆动或偏离，PWM 未饱和。随后持续运行缓弯复测也通过，误差主要保持在 `-1..-2`，两轮速度能跟随 `5/7`、`4/8` 等差速目标，最终赛道结束进入全白后由150 ms丢线保护正常停车。验证完成后已删除5秒测试上限并启用直道8、轻弯7、明显弯道6的分级速度。

2026-07-25 新电机完整映射复测结果：

| 逻辑车轮 | DRV8701E 通道 | 正交编码器 | 车辆前进 DIR | 前进时原始计数 | 控制反馈符号 | PWM=1600、前进 400 ms |
|---|---|---|---|---|---|---|
| 左轮 / CH1 | `A0 PWM` + `A1 DIR` | `TIM_G8`：`A26/A27` | `GPIO_HIGH` | 负数 | `-raw_count` | `-439`（归一化约 `+11.0 count/10ms`） |
| 右轮 / CH2 | `B12 PWM` + `B13 DIR` | `TIM_G9`：`B7/B9` | `GPIO_HIGH` | 正数 | `+raw_count` | `+450`（约 `+11.3 count/10ms`） |

静止时两通道稳定为 0。反向测试为左轮 `+426`、右轮 `-452`，与前进计数方向相反且幅值接近。新电机接线下电机与编码器不再交叉：`CH1=物理左轮/TIMG8`，`CH2=物理右轮/TIMG9`。以下 2026-07-11 表格只保留为旧接线历史记录。

开环前进标定（每次 400 ms，表内为包含起步过程的归一化平均值）：PWM=1000 时左/右约 `5.95/6.13 count/10ms`；PWM=1200 时约 `6.53/7.83`；PWM=1400 时约 `9.15/9.50`；PWM=1600 时约 `10.98/11.25`。单轮 PI 初始前馈采用 `PWM = 350 + 115 × target`。

以下配对由旧电机逐路运行测试得到，2026-07-25 更换电机后已经失效，不得用于当前闭环代码：

| 逻辑车轮 | DRV8701E 通道 | 正交编码器 | 车辆前进 DIR | 前进时原始计数 | 控制反馈符号 |
|---|---|---|---|---|---|
| 左轮 / CH2 | `B12 PWM` + `B13 DIR` | `TIM_G8`：`A26/A27` | `GPIO_HIGH` | 负数 | `-raw_count` |
| 右轮 / CH1 | `A0 PWM` + `A1 DIR` | `TIM_G9`：`B7/B9` | `GPIO_HIGH` | 正数 | `+raw_count` |

补充测量：映射测试在 `DIR=HIGH, PWM=2000` 时，CH1/TIMG9 约为 `+277 count/100ms`，CH2/TIMG8 约为 `-269 count/100ms`。物理复测确认该状态下左右轮均驱动车辆前进；不能仅依据编码器正负判断车辆前进方向。

旧记录约束（已失效）：2026-07-11 曾为 `CH1=物理右轮`、`CH2=物理左轮` 的交叉连接。当前代码禁止使用该关系；速度控制器按 2026-07-25 表格统一为“车辆前进为正”，左轮反馈取反、右轮反馈保持原符号。

### 4.2 舵机接口 P4/P5

| 舵机 | PWM token | MCU 引脚 | 例程默认 |
|---|---|---|---|
| 舵机 1 / P4 | `PWM_TIM_A1_CH0_B4` | `B4` | 50 Hz；代码活动角度范围示例 75-105°。 |
| 舵机 2 / P5 | `PWM_TIM_A1_CH1_B3` | `B3` | 50 Hz；代码允许频率范围注释为 50-300 Hz。 |

舵机信号是 3.3 V MCU 逻辑，舵机电源来自可调 `VCCServo`。不要用 `VCCServo` 给其他 3.3 V 外设供电。

### 4.3 编码器 P15/P16

#### 代码实际使用的资源

| 模式 | 编码器 | 定时器 | A/LSB | B/DIR | 例程宏 |
|---|---|---|---|---|---|
| 正交 | 编码器 1 / P15 | `TIM_G8` | `TIMG8_ENCODER1_CH1_A26` | `TIMG8_ENCODER1_CH2_A27` | `E2_01_encoder_quadrature_demo` |
| 正交 | 编码器 2 / P16 | `TIM_G9` | `TIMG9_ENCODER1_CH1_B7` | `TIMG9_ENCODER1_CH2_B9` | `E2_01_encoder_quadrature_demo` |
| 方向 | 编码器 1 / P15 | `TIM_G7` | `TIMG7_ENCODER1_CH1_A26` | `B27` | `E2_02_encoder_dir_demo` |
| 方向 | 编码器 2 / P16 | `TIM_G6` | `TIMG6_ENCODER1_CH1_B10` | `B11` | `E2_02_encoder_dir_demo` |

#### 编码器风险说明

原理图第 3 页引脚表同时出现编码器 2 的 `B10/B7`、`B11`、`B9`，第 5 页 P16 的局部网络标注与例程的正交/方向两种宏未能完全一一对应。例程源码明确使用上表四种 token，但 P16 连接器的物理信号对应需要导通测试确认。

因此：

- 生成编码器代码时只能复制上表对应例程的 timer/token 组合；不要自行把 `B7/B9/B10/B11` 重排。
- 如果编码器 2 读数为零、方向异常或只在一种模式下工作，应先测 P16 到核心板的导通关系，再修改文档状态。
- 在该问题关闭前，编码器 2 记录保持 `CONFLICT`，不能标为完全确认。

## 5. 姿态传感器接口 P14

主板 P14 的默认姿态传感器总线是 SPI1。

| 信号 | MCU token/引脚 | 说明 |
|---|---|---|
| `SCK/SPC` | `SPI1_SCK_B23` / `B23` | SPI1 时钟。 |
| `MOSI/SDI` | `SPI1_MOSI_B22` / `B22` | MCU 输出。 |
| `MISO/SDO` | `SPI1_MISO_B21` / `B21` | MCU 输入。 |
| `CS` | 软件 GPIO `B19` | IMU660RA/RB/RC、IMU963RA 头文件默认使用。 |
| `INT` | `B24` | 原理图 P14 提供；IMU660RC 的 `INT2` 使用，其他型号按各自驱动需求。 |
| 电源/地 | `VCC3V3/GND` | 姿态传感器使用 3.3 V。 |

默认 SPI 速度：IMU 设备头文件为 8 MHz；当前 `IMU660RA_USE_IIC=0`，所以 IMU660RA 例程默认走硬件 SPI。头文件中存在的软件 IIC/硬件 IIC 备选宏，不代表主板已经为该模式重新布线。

基准设备头文件：`zf_device_imu660ra.h`、`zf_device_imu660rb.h`、`zf_device_imu660rc.h`、`zf_device_imu963ra.h`。

## 6. 资源冲突表

| 资源/引脚 | 已占用接口 | 冲突对象 | 处理规则 |
|---|---|---|---|
| SPI0 `A12/A9/A13` | P10 屏幕 | WiFi SPI | 屏幕与 WiFi SPI 共享 SPI0 总线，理论上可通过不同 CS 工作，但仓库未提供组合验证；默认按互斥外设处理。 |
| `A8` | 屏幕 CS | 其他 SPI0 CS | 不得把 A8 当通用 CS，除非屏幕不连接。 |
| `A13` | IPS200PRO MISO，或 TFT/IPS114/IPS200 BL | 另一种屏幕语义 | 只能按当前屏幕型号选择 MISO 或背光。 |
| SPI1 `B23/B22/B21` | 姿态传感器 | 其他 SPI1 设备 | 可共享总线的前提是 CS、初始化和时序均单独管理；不要重新初始化成另一套引脚。 |
| `B19` | 姿态传感器 CS | 其他 GPIO/CS | IMU 连接时保留。 |
| `A16/A17` | GS08RA 的 S0/S1 | WiFi SPI 的 RST/INT | GS08RA 与 WiFi SPI 默认控制线冲突，按互斥处理。 |
| `B17/B25` | GS08RA 的 S2/OUT | 其他 GPIO/ADC | GS08RA 连接时保留为选择线/ADC。 |
| `B8/B26` | ToF 软件 IIC | 其他 GPIO/IIC | ToF 连接时保留；DL1B 硬件 IIC 备选的 SDA `B9` 不符合 P6 默认接线。 |
| `B3` | 舵机 1 PWM | WiFi UART 硬件 RST | 只能二选一。 |
| `B6/B5/B2` | 无线串口 TX/RX/RTS | 其他 UART/GPIO | 无线串口连接时保留。 |
| `A30/A31/B0/B1` | 四个按键 | 普通 GPIO/复用功能 | 按键连接时为低有效输入。 |
| `A18` | 蜂鸣器 | 普通 GPIO/复用功能 | 蜂鸣器连接时为输出控制脚。 |

## 7. 不建议使用的引脚

工程中的“尽量不要使用的引脚”文件列出：

`A19, A20, A5, A6, A4, A3`

另：`A14` 与核心板板载 LED 有关联，建议也不要用于关键功能；如果必须使用，必须接受 LED 负载/状态指示的影响。来源：[project/尽量不要使用的引脚.txt](SeekFree_MSPM0G3519_Opensource_Library/project/尽量不要使用的引脚.txt)。

## 8. 已知风险和不得自动修正的差异

### R1：编码器 2 P16 映射

例程分别使用 `B7/B9`（正交）和 `B10/B11`（方向），但原理图连接器局部标注与引脚表存在模式相关的歧义。状态：方向模式仍为 `CONFLICT`；本车使用的 `TIMG9 B7/B9` 正交模式已通过计数实测。处理：本车正交代码使用第 4.1 节实测配对；若改为方向模式，仍须导通测试。

### R2：DL1B 硬件 IIC 备选

DL1B 头文件的硬件 IIC 备选为 `SCL=B8`、`SDA=B9`，但主板 P6 默认 ToF 接口表为 `SCL=B8`、`SDA=B26`。状态：`CONFLICT`。处理：保持 `DL1B_USE_SOFT_IIC=1`，不要仅修改宏就切换硬件 IIC。

### R3：电池电压换算

电池检测为 `ADC0_CH7_A22`。原理图分压为上臂 10 kΩ、下臂 1 kΩ，因此：

```text
V_A22 = V_BAT × 1/11
V_BAT ≈ V_ADC × 11
12-bit ADC、3.3 V 满量程时，理论电池满量程约 36.3 V
```

例程中的 `36.3 * adc / 4096` 与该公式一致；不要根据例程注释中的“1/4”字样重新改成四分之一分压。

### R4：无线串口 TX/RX 宏命名

`WIRELESS_UART_TX_PIN` 在头文件注释中按“模块 TX 要接 MCU RX”的语境命名，容易与 MCU 视角混淆。生成代码时以本表为准：`MCU B6=TX -> 模块 RX`，`MCU B5=RX <- 模块 TX`。

### R5：部分电机/舵机例程的“直接接线说明”不是主板 P13/P4/P5 的最终映射

电机例程注释中有面向“核心板直接接线”的 `B13/B12/B9/B8` 说明，而同一例程的主板代码宏使用 `A0/A1/B12/B13`。舵机例程的旧注释写成 `B4/B5`，但代码宏和主板原理图使用 `B4/B3`。生成主板代码时以本契约的连接器矩阵和实际 `#define` 为准，不要复制这些旧注释中的核心板直连引脚。

## 9. 推荐的代码修改模板

最小系统初始化通常保留：

```c
clock_init(SYSTEM_CLOCK_80M);
debug_init();
```

主板电机 PWM 示例：

```c
pwm_init(PWM_TIM_A0_CH0_A0, 17000, 0); // A0，17 kHz，初始占空比 0
pwm_set_duty(PWM_TIM_A0_CH0_A0, duty);
```

主板 P8 串口示例：

```c
uart_init(UART_7, 115200, UART7_TX_B15, UART7_RX_B16);
```

修改时应优先修改目标例程的 `user/src/main.c` 或对应设备头文件中的宏，不要在应用代码中复制一套新的引脚编号。

## 10. GPT 生成代码前检查清单

- [ ] 已明确目标主板版本为 `MSPM0G35XX MotherBoard V1.1 / schematic V1.1.1.1`。
- [ ] 已明确使用的是 P6/P8/P10/P11/P12/P13/P14/P15/P16 中哪个物理接口。
- [ ] 已把连接器信号转换成 MCU 引脚名和完整库 token。
- [ ] 已检查该引脚是否出现在第 6 节资源冲突表或第 7 节禁用列表。
- [ ] 已检查 UART 的 TX/RX 是否按 MCU 视角连接。
- [ ] 已检查供电是 `VCC5V`、`VCC3V3`、`VCCCAMOP` 还是可调 `VCCServo`。
- [ ] 若使用编码器 2 或 DL1B 硬件 IIC，已先处理 `CONFLICT`，没有直接猜测。
- [ ] 已在代码中使用现有库枚举，而不是手写 GPIO 复用编号。
- [ ] 初始化后已用示波器/串口/LED/万用表完成至少一项硬件验证。

## 11. 用户维护区

新增或修改接口时，按以下格式追加记录。不要删除旧记录。

```text
日期：YYYY-MM-DD
硬件版本：
修改对象：接口 ID / MCU 引脚 / token / 电气逻辑
修改内容：
来源：原理图页码、文件路径和行号
验证方式：代码编译 / 导通测试 / 示波器 / 实测外设
状态：CONFIRMED / CODE_CONFIRMED / SCHEMATIC_ONLY / CONFLICT / NEEDS_MEASURE
维护者：
```

### 2026-07-11 初始整理

- 建立主板 V1.1.1.1 逻辑接口矩阵。
- 根据例程和头文件补充了 UART、SPI、PWM、ADC、编码器和设备宏 token。
- 显式记录编码器 2 P16、DL1B 硬件 IIC、电池分压注释三个风险点。
- 记录本车 DRV8701E 与正交编码器的逐路实测配对、方向极性和架空计数。
