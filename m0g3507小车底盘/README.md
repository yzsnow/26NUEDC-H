# 2026 电赛 H 题天猛星底盘主控

本工程用于“车载平衡滚球运动控制系统”的天猛星 MSPM0G3507 底盘控制。
当前阶段只实现小车运动部分，地猛星视觉控球板将在底盘调通后单独开发。

## 已实现功能

- 感维八路红外循迹与分级 PID 差速控制，不使用 I2C。
- 循迹核心仿照 `E:\A-2026diansai\715test1`：外侧探头优先、普通弯/急弯分级降速、出弯低速保持和短时丢线找线。
- 任务2使用独立高速曲线：直道400、普通弯360、急弯340、出弯保持350、终点前250 PWM；任务4/5/6保持原低速曲线。
- 半圆弯道自动降速、出弯低速保持和短时丢线恢复。
- G356 陀螺仪直道航向辅助；进入弯道或数据掉线时自动退出，不影响灰度循迹转弯。
- 左右编码器距离测量。
- 压住 A 点横线启动，驶出 300mm 后才武装停车判断；之后任意 4 路及以上灰度同时识别到黑线就立即制动。
- AB 段编码器定距停车。
- SysTick 独立毫秒计时和 0.96 英寸 OLED 状态显示。
- 运行中确认键或返回键立即制动停车。
- 丢线、超时和超距离安全停车。

## 比赛模式

上电默认选择任务 2，使用上、下按键切换模式，板载 PB21 确认启动或进入诊断页。

1. `T2 FAST LAP`：任务 2，高速循迹一圈并在 A 点停车。
2. `T4 A TO B`：任务 4，不使用灰度循迹，锁定启动时航向并按半余弦 S 曲线行驶 1.5 米后停车。
3. `T5/6 STEADY`：任务 5/6 共用的稳速单圈底盘流程。
4. `GYRO STATUS`：G356 通信、航向、角速度和帧计数诊断，不启动电机。

任务 3 是静止控球，不需要底盘运动，由后续地猛星控球板完成。

## 操作方式

- `PB15`：向上选择模式。
- `PB16`：向下选择模式。
- `PB21`：确认启动；运行中再次按下为紧急停车。
- `PB17`：运行中紧急停车。

在 `GYRO STATUS` 中：

- `PB21`：将当前绝对 Yaw 记录为软件零点。
- `PB17`：退出诊断页，返回模式选择。
- `G` 表示是否收到有效帧，`Z` 表示是否完成软件归零；`Y` 为绝对航向，`O` 为归零偏移，`W` 为 Z 轴角速度。
- `B/F/E` 分别表示累计接收字节数、有效帧数和错误帧数，可用于区分接线错误与协议解析错误。

OLED 显示当前模式、运行状态、比赛计时、编码器距离和 G356 状态。
运行时采用分页刷新；单圈里程达到 5000mm 后暂停软件 SPI 发送，避免终点横线漏采样。SysTick 计时不受影响，停车后会全屏显示真实最终用时。

## 关键标定参数

所有调车参数集中在 `App/vehicle_config.h`。

- `DRIVE_WHEEL_CIRCUMFERENCE_MILLIMETERS`：驱动轮实测周长。
- `MOTOR_ENCODER_COUNTS_PER_WHEEL_REVOLUTION`：车轮一圈对应编码器计数。
- `COMPETITION_TASK4_GYRO_TARGET_DISTANCE_MM`：任务 4 陀螺仪直行目标距离。
- `COMPETITION_TASK4_GYRO_ACCELERATION_DISTANCE_MM`、`COMPETITION_TASK4_GYRO_DECELERATION_START_MM`：任务 4 S 曲线加减速距离。
- `COMPETITION_FINISH_DETECTION_WINDOW_MM`：以 5832mm 圈长基准计算的终点识别窗口。
- `COMPETITION_FINISH_DISPLAY_SUSPEND_DISTANCE_MM`：接近终点后暂停 OLED 硬件刷新的距离。
- `COMPETITION_TASK2_BASE_PWM`：任务 2 基础速度。
- `COMPETITION_TASK4_GYRO_CRUISE_PWM`：任务 4 陀螺仪直行巡航速度。
- `COMPETITION_LAP_STEADY_BASE_PWM`：任务 5/6 稳球基础速度。
- `LINE_TRACKING_PID_PROPORTIONAL_GAIN`、`LINE_TRACKING_PID_INTEGRAL_GAIN`、`LINE_TRACKING_PID_DERIVATIVE_GAIN`：循迹 PID。

停车精度必须在实际底盘上标定。尤其是灰度板到车体中轴测试标记点的距离、轮胎打滑和 TB6612 制动惯性，无法仅靠理论尺寸确定。

## 接线

### 感维八路红外循迹

- L4～R2：`PA12`～`PA17`。
- R3：`PA0`。
- R4：`PA1`。
- 当前按低电平检测到黑线处理。

### TB6612 与编码器

- 左右 PWM：`PB2`、`PB3`。
- 左电机方向：`PB18`、`PB19`。
- 右电机方向：`PB20`、`PB24`。
- 左编码器：`PB0`、`PB1`。
- 右编码器：`PB7`、`PB6`。

### G356

- G356 TX 接 `PA25`，即天猛星 UART3 RX。
- G356 RX 接 `PA26`，只接收遥测时可以不连接。
- 串口参数为 115200、8N1。

### OLED

- SCK：`PB9`。
- MOSI：`PB8`。
- RES：`PB10`。
- DC：`PB11`。
- CS：`PB14`。

所有模块必须与天猛星共地。电机和大功率负载应独立供电，并做好电源滤波。

## 工程入口

- Keil 工程：`keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`。
- 主程序：`User/empty.c`。
- 比赛状态机：`App/competition_control.c`。
- 循迹与航向融合：`Control/motion.c`。
- 外设配置源：`empty.syscfg`。

`ti_msp_dl_config.c` 和 `ti_msp_dl_config.h` 是 SysConfig 生成文件，不应手工修改。

## 下一阶段

底盘实车完成以下标定后，再开始地猛星控球板：

1. 八路探头方向与 PID 参数。
2. AB 段停车距离。
3. A 点横线识别武装距离和立即制动停车精度。
4. 任务 2 单圈时间与停车偏差。
5. 任务 4/5/6 稳球所需的低加速度速度曲线。
6. 天猛星与地猛星 UART 通信协议。
