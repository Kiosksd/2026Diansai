# MaixCAM2 钢珠位置与速度测量程序

本程序负责钢珠识别、轨道标定、位置/速度滤波，并向 MSPM0G3507 提供串级 PID 所需的反馈量。

- 不启动 RTSP、JPEG 或 WebRTC 网络图传。
- MaixCAM2 通过右侧 UART3 向 MSPM0G3507 的 UART2 发送视觉测量值；3507 使用位置和速度进行串级 PID 控制。
- 识别画面通过 `disp.show(img)` 显示在 MaixCAM2 屏幕；连接 MaixVision 调试时也能看到。
- 使用 MaixCAM2 的 YOLO26 `640x160` 模型。
- 使用低延迟单缓冲检测，使检测框与当前输入画面对应。
- 每 500ms 向终端打印一次 FPS、检测耗时、位置、误差和滤波状态。

运行前重点检查 `main.py` 中的参数：

```python
AXIS_START_PX = (97, 122)
AXIS_ZERO_PX = (318, 123)
AXIS_END_PX = (551, 124)
AXIS_START_CM = -10.0
AXIS_ZERO_CM = 0.0
AXIS_END_CM = 10.0
DISPLAY_TARGET_CM = 0.0
```

以上三点来自相机固定后的实测数据：刻度`-10cm`、`0cm`和`+10cm`。
程序对左右半轴分别换算，补偿相机透视和安装偏差。

坐标定义为刻度中心`0cm`，左侧为负、右侧为正。

## 视觉串口联调

- MaixCAM2 右侧 `B2/UART3_TX` -> 3507 主板串口插座 `B16/RX`
- MaixCAM2 右侧 `B3/UART3_RX` <- 3507 主板串口插座 `B15/TX`（本阶段可不接）
- MaixCAM2 `GND` <-> 3507 `GND`
- 115200 baud，8N1；不要连接主板串口插座上的 `5V`
- 约 50Hz 发送一帧；即使暂时识别不到球，也会发送 `valid=0` 帧用于区分“丢球”和“串口断线”

固定 13 字节协议：

```text
AA 55 TYPE FLAGS SEQ POS_L POS_H VEL_L VEL_H TIME_L TIME_H CRC8 0D
```

`TYPE=0x01`，`FLAGS bit0` 为识别有效位；位置和速度分别采用有符号
`int16` 小端格式，单位为 `0.01cm` 和 `0.01cm/s`。CRC-8 使用多项式
`0x07`、初值 `0x00`，计算范围为 `TYPE` 到 `TIME_H`。

上电后电机保持禁用。依次执行 `Z`、`E`、`V` 后，3507 才会启用视觉串级闭环；
控制目标由无线命令 `X<位置cm>` 设置，例如 `X+5`、`X-5` 或 `X0`。
`DISPLAY_TARGET_CM` 只影响 MaixCAM2 屏幕上的目标标记，不会改变 3507 的控制目标。
