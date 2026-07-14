# 全局核心变量分层说明

本文用于整理旧工程中几个核心全局变量的职责边界，避免后续重构时继续让 APP、MCS、PWM/FOC 状态交错在一起。

## 1) 总体原则

重构后的数据流建议保持单向：

```text
UART / KEY
  -> APP command
  -> APP business state
  -> MCS public API
  -> MCS internal command / state
  -> FOC / PWM
  -> MCS status snapshot
  -> APP report / protection decision
```

层间原则：

- APP 发出业务意图，不直接改 PWM、FOC 状态机和底层命令位。
- MCS 负责执行电机启动、停止、开环、闭环和故障停机。
- APP 读取 MCS 的状态快照做业务判断和串口上报。
- 直接驱动 PWM 的变量和状态必须留在 MCS 内部。

## 2) 变量归属总表

| 变量 | 建议归属层 | 当前/旧工程作用 | 重构建议 |
| --- | --- | --- | --- |
| `gDeskMianMbr` | APP 用户应用层 | 升降桌业务状态：当前高度、目标高度、限位、记忆位、复位流程、业务系统状态 | 保留在 APP，不能直接控制 PWM/FOC |
| `gAppCmd` | APP -> MCS 命令边界 | 旧代码中表达电机正转、反转、停止、目标频率等命令 | 后续改为 `MCS_Command_t` 或用 `MCS_Motor_Start/Stop` API 替代 |
| `gMotorMainCmd` | MCS 内部命令邮箱 | MCS 内部运行命令：`RunCmd`、`DirCmd`、`PwmCmd`、重启请求等 | APP 不直接访问，只由 MCS API 或 MCS 状态机写入 |
| `gMainStatus` | MCS 内部状态机 | 电机控制实际状态：`DriveSts`、`MotorSts`、`Error` | MCS 内部读写；APP 只通过状态快照或查询接口间接读取 |
| `gMcsStatus` | MCS -> APP 状态快照 | 给 APP 看电机反馈：故障、电压、频率、电流、功率等 | 作为只读快照或替换为 `MCS_MotorStatus` |

## 3) `gDeskMianMbr`

`gDeskMianMbr` 是升降桌业务对象，属于 APP 层。

它回答的问题是：

```text
桌子当前高度是多少？
目标高度是多少？
最大/最小限位是多少？
当前是正常、复位、过热、故障还是老化模式？
业务运动状态是加速、运行、减速还是停止？
记忆位置是多少？
```

典型字段：

- `currentHeight`
- `targetHeight`
- `initHeight`
- `Target_gear_01` / `Target_gear_02` ...
- `maxHeightLimit`
- `minHeightLimit`
- `systemStatus`
- `MotorState`
- `ResState`

注意：

- 它不应该直接表达 FOC 内部状态。
- 它不应该直接控制 `PwmAOutputs()`。
- 它可以决定是否请求 MCS 启动或停止。

## 4) `gAppCmd`

`gAppCmd` 在旧工程中名字叫 APP 命令，但定义和处理在 `Drivers/MotorControlSystem/Source/AppFunction.c`，所以它是一个混层变量。

旧作用：

```text
APP 侧运行意图
  -> gAppCmd.eAppCmdCode
  -> gAppCmd.FreqCmd
  -> ControlMotorRunningProc()
  -> gMotorMainCmd
```

它实际表达的是：

- 电机运行命令：正转、反转、停止
- 目标频率/速度
- 命令来源

重构建议：

短期可以保留概念，但不要让 APP 直接写 MCS 内部结构。建议改成更明确的边界：

```c
MCS_Motor_StartOpenLoop(MCS_MOTOR_DIR_FORWARD);
MCS_Motor_StartOpenLoop(MCS_MOTOR_DIR_REVERSE);
MCS_Motor_Stop();
```

后续闭环恢复时可以扩展成：

```c
MCS_Motor_Start(const MCS_MotorCommand *cmd);
MCS_Motor_SetTargetSpeed(...);
MCS_Motor_Stop(...);
```

## 5) `gMotorMainCmd`

`gMotorMainCmd` 属于 MCS 内部命令邮箱。

旧结构中包含：

- `ActReq`：动作请求
- `RunCmd`：运行/停止命令
- `DirCmd`：方向
- `CtrlMethod`：控制模式
- `PwmCmd`：PWM 开启阶段控制
- `UnVolReStartReq` / `OvVolReStartReq`：欠压/过压恢复重启请求

它回答的问题是：

```text
MCS 状态机下一步要执行什么？
是否请求运行？
方向是什么？
PWM 是否已经打开？
```

重构建议：

- 只允许 MCS 内部写。
- APP 层不能直接写 `RunCmd`、`DirCmd`、`PwmCmd`。
- APP 调用 `MCS_Motor_*` 接口，由 MCS 接口内部更新命令邮箱。

## 6) `gMainStatus`

`gMainStatus` 属于 MCS 内部状态机。

当前重构工程定义在 `include/mcs/mcs_state_type.h`：

```c
typedef struct STRUCT_DRIVE_MAIN_STS {
    Drive_Status_ENU DriveSts;
    Motor_Status_ENU MotorSts;
    uint32_t Error;
} DRIVE_MAIN_STR;
```

它回答的问题是：

```text
电机控制当前处于 IDLE / INIT / RUN / STOP / FAULT？
电机子状态处于 RESET / OPEN / PRE_CLOSE / SPEED_RUN / BRAKE？
当前 MCS 故障位是什么？
```

字段含义：

- `DriveSts`：驱动主状态，例如 `IDLE`、`INIT`、`RUN`、`STOP`、`FAULT`。
- `MotorSts`：电机控制子状态，例如 `OPEN`、`SPEED_RUN`、`BRAKE`。
- `Error`：MCS 故障位。

注意：

- 它不是 APP 业务状态。
- 它不表示桌子的高度状态或记忆位置。
- APP 层不应直接修改它。
- 目前开环测试模块临时写 `gMainStatus`，后续应收进正式 MCS 状态机。

## 7) `gMcsStatus`

`gMcsStatus` 属于 MCS 到 APP 的状态快照。

旧定义中包含：

- `SysError`
- `SysRunStatus`
- `MotorFreq`
- `MotorPower100mw`
- `Vdc`
- `phaseA`

它回答的问题是：

```text
APP 层需要看到的电机状态是什么？
母线电压是多少？
频率/电流/功率是多少？
当前故障是什么？
```

重构建议：

- MCS 周期更新。
- APP 只读。
- 后续可以替换为更清晰的 `MCS_MotorStatus`：

```c
typedef struct
{
    uint32_t error;
    int32_t speed;
    int32_t current;
    int32_t voltage;
    uint8_t running;
} MCS_MotorStatus;
```

## 8) 旧工程实际链路

旧工程大致链路如下：

```text
UART / KEY
  -> KeyState / Uart_TargetHeight
  -> gDeskMianMbr
  -> gAppCmd
  -> ControlMotorRunningProc()
  -> gMotorMainCmd
  -> DriverMainHandle()
  -> gMainStatus
  -> FOC_Model() / PwmAOutputs()
  -> gMcsStatus / AppCommData
  -> USER_APP 业务判断和串口上报
```

这条链路的问题：

- `gAppCmd` 名义像 APP 层，实际在 MCS 文件里处理。
- `gMainStatus.Error` 被 APP 层直接读写。
- `gMcsStatus` 和 `gMainStatus` 都表达故障，含义重复。
- `gDeskMianMbr.MotorState` 和 `gMainStatus.MotorSts` 都叫 MotorState，容易混淆。

## 9) 当前重构工程建议链路

当前已验证六步开环可运行，建议继续保持以下分层：

```text
app_uart_comm.c
  -> 解析串口帧
  -> 保存 APP_UART_COMMAND

user_app.c
  -> 读取 APP_UART_COMMAND
  -> 判断业务状态
  -> 调用 MCS_Motor_StartOpenLoop() / MCS_Motor_Stop()

mcs_motor.c
  -> MCS 对外电机接口
  -> 内部调用开环/闭环实现

mcs_task.c
  -> MCS 内部周期任务
  -> 推进 MCS_Motor_Task1ms()

mcs_open_loop_test.c
  -> 临时六步开环实现
  -> 后续被正式开环启动流程替换
```

最终目标：

```text
APP 层只处理业务
MCS 层只处理电机控制
BSP 层只处理硬件外设
```

## 10) 后续重构顺序建议

1. 把 `gMainStatus` 的读写集中到 MCS 内部。
2. 把 `gMotorMainCmd` 恢复为 MCS 内部命令邮箱，不向 APP 暴露。
3. 用 `MCS_MotorStatus` 替代或包裹 `gMcsStatus`。
4. 把 `gDeskMianMbr` 仅保留为 APP 业务对象。
5. 移除 `gAppCmd` 或改名为明确的 MCS 命令结构。
6. 逐步将开环测试替换为正式开环启动，再接入闭环切换。

## 11) 当前已落地的分层实现

### 2026-05-14：APP 最小状态机 + MCS 状态快照

本次更新加入了 APP 业务状态机和 MCS 对外状态查询接口。

新增文件：

- `include/app/app_control.h`
- `src/user/app_control.c`
- `include/mcs/mcs_motor.h`
- `src/mcs/mcs_motor.c`

当前链路：

```text
app_uart_comm.c
  -> 解析串口帧
  -> 设置 APP_UART_COMMAND pending

user_app.c
  -> 检查 pending 命令
  -> 调用 AppControl_HandleUartCommand()

app_control.c
  -> 维护 APP_CONTROL_IDLE / RUNNING_UP / RUNNING_DOWN / FAULT
  -> 根据串口命令决定业务状态
  -> 调用 MCS_Motor_StartOpenLoop() / MCS_Motor_Stop()
  -> 通过 MCS_Motor_GetStatus() 获取 MCS 状态快照

mcs_motor.c
  -> 作为 MCS 对外电机接口
  -> 当前内部包装 mcs_open_loop_test.c 六步开环测试
  -> 集中读取 gMainStatus 并填充 MCS_MOTOR_STATUS

mcs_task.c
  -> 调用 MCS_Motor_Task1ms()
  -> 推进开环换向
```

这一步完成后，APP 层不再直接调用 `MCS_OpenLoopTest_*()`，也不需要直接读取 `gMainStatus`。

当前仍保留的临时实现：

- `mcs_open_loop_test.c` 仍然直接写 `gMainStatus.DriveSts` 和 `gMainStatus.MotorSts`，这是六步开环验证阶段的临时做法。
- 后续正式接入 MCS 状态机时，应把这些状态写入收回到 `mcs_motor.c` 或新的 MCS 状态机模块。

### 2026-05-14：APP 限位判断骨架

本次更新在 `app_control.c` 中加入了最小限位判断框架。

新增职责：

- `AppControl_InitTravelRange()`：如果 NVM/高度模块尚未初始化行程范围，则使用 `MIN_TRAVEL_DISTANCE` 和 `MAX_TRAVEL_DISTANCE` 填充默认范围。
- `AppControl_CanRunUp()`：判断当前高度是否允许继续上升。
- `AppControl_CanRunDown()`：判断当前高度是否允许继续下降。
- `AppControl_CheckLimitWhileRunning()`：运行中持续检查上下限，触发后请求 MCS 停机。
- `AppControl_EnterMaxLimit()` / `AppControl_EnterMinLimit()`：到达限位后的 APP 业务处理。

当前限位链路：

```text
APP_UART_CMD_UP
  -> AppControl_EnterRunningUp()
  -> AppControl_CanRunUp()
  -> MCS_Motor_StartOpenLoop(FORWARD) 或进入最大限位状态

APP_UART_CMD_DOWN
  -> AppControl_EnterRunningDown()
  -> AppControl_CanRunDown()
  -> MCS_Motor_StartOpenLoop(REVERSE) 或进入最小限位状态

AppControl_TaskAlways()
  -> AppControl_CheckLimitWhileRunning()
  -> 到达限位后 MCS_Motor_Stop()
```

当前仍是骨架：

- `gDeskMianMbr.currentHeight` 还没有接入真实霍尔高度计算。
- 为避免未初始化行程范围导致限位全部失效，当前会在 APP 初始化时补默认行程范围，并把当前高度放到行程中间。
- 下一步接入霍尔高度后，`currentHeight` 将由高度计算模块周期刷新，限位判断即可真实生效。

### 2026-05-15：APP 自研 Hall 高度验证模块

本次更新生成独立的 APP 高度模块文件，但不修改 Keil 工程文件；`.uvprojx` 由人工在 Keil 中添加。由于原工程 Hall 黑盒库依赖的变量和回调较多，本阶段不再使用 `HallAngle.h` 中的 `HalltoAngle()` / `RespondHALL_TOAPP_TokenWord()` 链路，改为直接根据两个 Hall ADC 原始值做高度验证。

新增文件：

- `include/app/app_height.h`
- `src/user/app_height.c`

涉及文件：

- `include/app/app_height.h`：声明 `AppHeight_Init()`、`AppHeight_Sample()`、`AppHeight_Task100ms()` 等高度接口。
- `include/app/main.h`：包含 `app_height.h`。
- `src/user/manage.c`：在 `User_app_init()` 中先调用 `AppHeight_Init()`。
- `src/user/user_app.c`：在 100ms 任务里调用 `AppHeight_Task100ms()`。
- `src/user/app_height.c`：运行时采集 Hall ADC 的 min/max 计算中心点，把 A/B 两相信号转成 4 个正交状态，累计合法边沿，并刷新 `gDeskMianMbr.currentHeight`。
- `src/mcs/mcs_foc_hw.c`：在 ADC 采样处理中调用 `AppHeight_Sample(ADC_DAT3, ADC_DAT4)`。

当前高度链路：

```text
ADC_IRQHandler()
  -> AdcEocHandler()
  -> AppHeight_Sample(ADC_DAT3, ADC_DAT4)
  -> APP自研Hall高度模块累计正交边沿变化量

User_App_100ms_Task()
  -> AppHeight_Task100ms()
  -> gDeskMianMbr.currentHeight =
     gDeskMianMbr.initHeight + Hall边沿累计量换算行程
```

当前验证策略：

- 先不接 Flash/NVM 读取，`AppHeight_Init()` 使用默认导程、减速比、行程范围和极对数。
- 当前高度初始化到行程中点，并清零自研扇区累计量，方便验证升、降两个方向。
- `AppHeight_Sample()` 在电机运行时采集 512 点，用 `min/max` 估算 Hall A/B 的中心点，之后才开始计数。
- 当前用带迟滞的 A/B 两相信号生成 4 个正交状态，只接受合法相邻状态跳变；非法跳变会丢弃一次状态同步。
- 如果电机没有运行，`AppHeight_Sample()` 不累计扇区，避免静止时 Hall 噪声导致高度在两个值之间跳动。
- 迟滞阈值 `APP_HEIGHT_HYSTERESIS` 当前为 150 个 ADC 计数，用于避免阈值附近抖动造成 `currentHeight` 来回跳。
- 本阶段只验证：开环转动时 `gDeskMianMbr.currentHeight` 是否会变化；方向相反时可调整 `APP_HEIGHT_DIR_SIGN`。

### 2026-05-15：进入 Hall 观测安全阶段，禁用开环驱动

由于 29V 电机在六步开环测试中出现沉闷声音、严重发热和焦味，本阶段禁止继续通过串口升/降命令启动开环 PWM。

涉及文件：

- `src/user/app_control.c`
  - 新增 `APP_CONTROL_ENABLE_OPEN_LOOP_DRIVE`，当前固定为 `0`。
  - `APP_UART_CMD_UP` / `APP_UART_CMD_DOWN` 不再调用真实 `MCS_Motor_StartOpenLoop()` 输出 PWM。
  - 收到升/降时会调用 `MCS_Motor_Stop()` 并回到 `APP_CONTROL_IDLE`，避免误触发继续发热。
- `src/user/app_height.c`
  - Hall 观测不再依赖 `MCS_Motor_IsRunning()`，即不开电机也可以采集 `hal1/hal2`。
  - 新增 `electricAngle` 角度观测值，当前使用 Hall A/B 模拟量相对中心点的整数近似 `atan2` 计算，输出范围为 `0~65535`。2026-05-19 实测确认它表示 Hall 机械角观测值，进入 FOC 前需要乘 `Pole_Pairs`。
  - 位置累计改为对 `electricAngle` 做跨 0 点差分积分，保存到 `cumulativeAngle`；`cumulativeEdge` 仅作为 `cumulativeAngle / 16384` 的调试显示量。
  - `APP_HEIGHT_ANGLE_DELTA_DEADBAND` 调整为 `0`。ADC 中断频率较高，手动慢速转动时单次角度差很小，如果设置死区会导致 `electricAngle` 变化但 `cumulativeAngle` 不累计。
  - `APP_HEIGHT_MIN_RANGE` 提高到 `1000`，避免静止噪声范围误触发校准完成。
  - 校准完成后仍会跟踪 `hal1/hal2` 的新 min/max，并动态修正中心点，避免上电后中心点被早期小范围数据锁死。

当前安全链路：

```text
串口升/降
  -> AppControl_RequestOpenLoop()
  -> APP_CONTROL_ENABLE_OPEN_LOOP_DRIVE == 0
  -> MCS_Motor_Stop()
  -> APP_CONTROL_IDLE

ADC_IRQHandler()
  -> AdcEocHandler()
  -> AppHeight_Sample(ADC_DAT3, ADC_DAT4)
  -> 只观测 Hall 状态、电角度和高度累计，不主动驱动电机
```

下一步目标：

- 先用手动转动或极低风险外部方式观察 `APP_HEIGHT_STATUS.calibrated`、`electricAngle`、`cumulativeEdge` 是否连续。
- 在恢复任何 PWM 输出前，先完成电流采样零点、母线电压和过流保护检查。

### 2026-05-15：补充 Hall 高度算法注释并清理旧边沿累计代码

本次更新主要是说明和清理，不改变 Keil 工程文件。

- `src/user/app_height.c`
  - 补充模块级注释，说明两路模拟 Hall 按近似 `sin/cos` 信号处理。
  - 补充 min/max 校准、中心点、整数近似 `atan2`、跨 0 点差分积分、行程换算等关键注释。
  - 删除不再参与高度换算的旧四状态边沿累计函数，避免误以为 `cumulativeEdge` 仍是主累计来源。
- `include/app/app_height.h`
  - 给 `APP_HEIGHT_STATUS` 字段和接口补充注释。

当前结论：

- `electricAngle` 是主观测角度，范围 `0~65535`。当前实测 Hall A/B 机械转子转 `360°` 才完成一个波形周期，因此该值按一圈机械角观测值使用。
- `cumulativeAngle` 是主位置累计量，由 `electricAngle` 差分积分得到。
- `cumulativeEdge` 只是调试量，等于 `cumulativeAngle / 16384`，不再直接驱动高度计算。

### 2026-05-15：补充母线电压换算和基础保护

本次更新根据当前硬件母线采样电路补充 `gBUS_Vol_ADC` 到实际电压的换算，并把欠压/过压状态接入 MCS 10ms 任务。

硬件采样关系：

```text
VIN0 -> 20K -> VBUS_AD -> 1K -> GND

VBUS_AD = VIN0 * 1K / (20K + 1K)
VIN0(mV) = ADC / 32768 * 3600mV * 21
```

示例：

```text
gBUS_Vol_ADC = 12597
母线电压约 = 12597 / 32768 * 3600 * 21 = 29065mV
```

新增变量：

| 变量 | 归属层 | 作用 |
| --- | --- | --- |
| `gBUS_Vol_ADC` | MCS / ADC采样层 | ADC中断中更新的母线电压原始采样滤波值 |
| `gBusVoltageMv` | MCS状态层 | 由 `gBUS_Vol_ADC` 换算得到的母线电压，单位 mV |
| `gBusVoltageStatus` | MCS状态层 | 母线电压状态：未知、正常、欠压、过压 |

涉及文件：

- `include/mcs/mcs_foc_hw_type.h`
  - 新增 `MCS_BUS_VOLTAGE_STATUS`。
  - 新增 `MCS_BUS_VOLTAGE_INFO`。
  - 声明母线电压换算和查询接口。
- `include/mcs/mcs_variable.h`
  - 声明 `gBusVoltageMv` 和 `gBusVoltageStatus`。
- `include/mcs/mcs_prototype.h`
  - 补充 `ClearSysErrorFlag()` 声明。
- `src/mcs/mcs_foc_hw.c`
  - 新增 `MCS_BusVoltage_AdcToMv()`。
  - 新增 `MCS_BusVoltage_Task10ms()`。
  - 10ms 周期根据电压状态置位/清除 `LOW_VOL_ERROR` 和 `HIG_VOL_ERROR`。
  - 检测到欠压或过压时立即关闭 PWM 输出。
- `src/mcs/mcs_task.c`
  - 在 MCS 10ms 任务中调用 `MCS_BusVoltage_Task10ms()`。

当前保护阈值：

| 状态 | 触发阈值 | 恢复阈值 |
| --- | --- | --- |
| 欠压 | `<= 18.0V` | `>= 20.0V` |
| 过压 | `>= 33.0V` | `<= 31.0V` |

注意：

- 这是基础母线保护，不等于完整闭环控制。
- 目前只根据母线电压设置 MCS 错误位并关闭 PWM，后续闭环启动前还需要继续补电流采样零点、过流保护、方向和角度对应关系。
- 本次只修改源码文件，不修改 Keil 工程文件；需要人工把新增/修改过的文件加入工程或确认工程中已有这些文件。

### 2026-05-15：补充软件相电流过流保护

本次更新在已有电流零漂校准基础上增加软件过流保护。硬件 CMP/MCPWM 故障保护仍然保留；软件保护作为额外观察和兜底，便于闭环前确认电流采样是否可信。

保护链路：

```text
ADC_IRQHandler()
  -> AdcEocHandler()
  -> MCS_PhaseCurrent_CheckFast()
  -> 读取 A/B 相电流 ADC 原始值
  -> 减去 hPhaseAOffset / hPhaseBOffset
  -> 计算 A/B 绝对值峰值
  -> RUN 状态下超过阈值则立即关闭 PWM 并置 E_FAULT_SHORT_ERROR
```

新增变量：

| 变量 | 归属层 | 作用 |
| --- | --- | --- |
| `hPhaseAOffset` / `hPhaseBOffset` | MCS采样校准层 | 上电零漂校准得到的 A/B 相电流零点 |
| `gPhaseCurrentAAdc` / `gPhaseCurrentBAdc` | MCS采样状态层 | 原始电流采样减零点后的 ADC 计数 |
| `gPhaseCurrentPeakAbsAdc` | MCS采样状态层 | A/B 相电流绝对值历史峰值，方便 Keil 在线观察 |
| `gPhaseCurrentStatus` | MCS保护状态层 | 软件相电流保护状态 |

涉及文件：

- `include/mcs/mcs_foc_hw_type.h`
  - 新增 `MCS_PHASE_CURRENT_STATUS`。
  - 新增 `MCS_PHASE_CURRENT_INFO`。
  - 声明 `MCS_PhaseCurrent_CheckFast()` 和 `MCS_GetPhaseCurrentInfo()`。
- `include/mcs/mcs_variable.h`
  - 声明电流零点和软件过流调试变量。
- `src/mcs/mcs_foc_hw.c`
  - 在 ADC 快路径中执行相电流过流判断。
  - 当前阈值沿用旧工程业务过载参数：
    `((Max_OVLOAD_CURRENT / (U_RATED_CUR * 1.414)) * 100) * 41`。
  - 旧工程低压系统 `U_RATED_CUR = 2.0A`，当前 `Max_OVLOAD_CURRENT = 14`，换算后约为 `20295`。
  - 过流后置位 `E_FAULT_OVER_LOAD_ERROR`，并立即 `PwmAOutputs(DISABLE)`。
  - 后续恢复 FOC 代码时，FOC 快路径也会被 `gPhaseCurrentStatus == MCS_PHASE_CURRENT_NORMAL` 门控，避免过流后继续进入控制计算。

注意：

- 软件过流错误当前按锁存处理，不自动清除 `E_FAULT_OVER_LOAD_ERROR`。
- 当前阈值参考旧工程 `g_OvLoadCur` 的核心电流单位；硬件级短路保护仍由 CMP/MCPWM 负责，并继续使用 `E_FAULT_SHORT_ERROR`。
- 因 `AdcSampleCal()` 依赖 `MotorCtrlPar.uShuntSamplingMode == 2` 才更新 `iAdcRes1/iAdcRes2`，本次软件保护直接读取电流 ADC 通道原始值再减零点，避免保护被采样模式变量挡住。

### 2026-05-15：补充相电流 mA 换算和状态快照

本次更新在软件相电流保护基础上，增加 A/B 相电流的 mA 换算和查询快照。mA 结果当前只用于观察，不改变已有保护阈值。

换算参数沿用旧工程低压 29V 硬件配置：

```text
ADC满量程 = 32768
ADC基准电压 = 3.6V
运放增益 = 18.18
采样电阻 = 0.01ohm
```

换算关系：

```text
I(mA) = adcDiff / 32768 * 3.6V / 18.18 / 0.01ohm * 1000
满量程电流约 = 19802mA
```

新增变量：

| 变量 | 归属层 | 作用 |
| --- | --- | --- |
| `gPhaseCurrentAMa` / `gPhaseCurrentBMa` | MCS采样状态层 | A/B 相电流，单位 mA |
| `gPhaseCurrentPeakAbsMa` | MCS采样状态层 | A/B 相电流绝对值历史峰值，单位 mA |

涉及文件：

- `include/mcs/mcs_foc_hw_type.h`
  - `MCS_PHASE_CURRENT_INFO` 增加 `phaseAMa`、`phaseBMa`、`peakAbsMa`。
  - 声明 `MCS_PhaseCurrent_AdcToMa()`。
- `include/mcs/mcs_variable.h`
  - 声明 `gPhaseCurrentAMa`、`gPhaseCurrentBMa`、`gPhaseCurrentPeakAbsMa`。
- `src/mcs/mcs_foc_hw.c`
  - 新增 `MCS_PhaseCurrent_AdcToMa()`。
  - 在 `MCS_PhaseCurrent_CheckFast()` 中同步刷新 mA 观测值。
  - `MCS_GetPhaseCurrentInfo()` 返回 ADC 和 mA 两套观测值。

注意：

- 当前 mA 换算按旧工程 `LOW_VOL_SYS` 的 `RSHUNT = 0.01ohm` 处理。
- `MC_MY/include/bsp/bsp_user.h` 里 `RSHUNT` 仍受 `LOW_VOL_SYS` 宏影响；后续应统一低压系统宏定义，避免硬件配置和 MCS 换算参数分叉。
- 软件过流阈值仍沿用旧工程核心电流单位，不直接用 mA 阈值触发。

### 2026-05-16：MCS 层硬禁用六步开环，并加入 Hall FOC 安全入口骨架

本次更新响应开环转动发热严重的问题，把六步开环从“APP 层禁用”提升为“MCS 层默认硬禁用”。这样即使后续有人绕过 `app_control.c` 直接调用 `MCS_Motor_StartOpenLoop()`，也不会再次打开 PWM。

新增启动结果枚举：

| 枚举 | 含义 |
| --- | --- |
| `MCS_MOTOR_START_OK` | 启动检查通过 |
| `MCS_MOTOR_START_BLOCKED_FAULT` | `gMainStatus.Error` 非零，禁止启动 |
| `MCS_MOTOR_START_BLOCKED_BUS_VOLTAGE` | 母线电压状态不是正常，禁止启动 |
| `MCS_MOTOR_START_BLOCKED_PHASE_CURRENT` | 软件相电流状态不是正常，禁止启动 |
| `MCS_MOTOR_START_BLOCKED_OPEN_LOOP_DISABLED` | 六步开环测试入口被安全策略禁用 |
| `MCS_MOTOR_START_NOT_IMPLEMENTED` | Hall FOC 测试入口已建立，但真实 FOC 输出尚未接入 |

涉及文件：

- `include/mcs/mcs_motor.h`
  - 新增 `MCS_MOTOR_START_RESULT`。
  - 新增 `MCS_Motor_StartHallFocSafetyTest()`。
  - 新增 `MCS_Motor_GetLastStartResult()`，方便 Keil watch 或后续串口上报启动被拦截的原因。
- `src/mcs/mcs_motor.c`
  - 新增 `MCS_Motor_CheckStartSafety()`，集中检查故障位、母线电压状态和软件相电流状态。
  - 新增 `MCS_Motor_BlockStart()`，所有被拦截的启动都会统一关 PWM、回中性比较值并记录原因。
  - 新增 `MCS_ENABLE_DANGEROUS_OPEN_LOOP_TEST`，默认 `0`。只有显式改为 `1` 时，六步开环才可能继续运行。
  - `MCS_Motor_StartOpenLoop()` 在默认配置下只会停机并记录 `MCS_MOTOR_START_BLOCKED_OPEN_LOOP_DISABLED`。
  - `MCS_Motor_StartHallFocSafetyTest()` 当前只做安全门控，不输出 PWM；待旧 FOC 电流环和 Hall 电角度链路迁回后再接真实驱动。

当前安全链路：

```text
APP 或调试入口请求启动
  -> MCS_Motor_StartOpenLoop() / MCS_Motor_StartHallFocSafetyTest()
  -> MCS_Motor_CheckStartSafety()
  -> 检查 gMainStatus.Error / gBusVoltageStatus / gPhaseCurrentStatus
  -> 不满足则 MCS_Motor_BlockStart()
  -> 满足但 Hall FOC 尚未实现时仍不输出 PWM
```

注意：

- 六步开环模块文件 `mcs_open_loop_test.c` 暂时保留，用于保留已有验证代码和中性 PWM 写入逻辑，但默认不能启动。
- 下一步恢复驱动时，不应重新启用六步开环；应迁入 Hall 定向 FOC 的最小电流环，并继续复用本次新增的启动安全门控。
- 之后每次结构性更新仍需同步更新本文档，并在源码中补充必要注释。

### 2026-05-16：Hall 角度/高度方向观测结论

本次实验通过手动顺时针转动转子观察 Hall 与高度链路，确认当前 APP 高度观测方向具备一致性。

观测结果：

- 顺时针转动转子时，`APP_HEIGHT_STATUS.electricAngle` 变大。
- 顺时针转动转子时，`APP_HEIGHT_STATUS.cumulativeAngle` 变大。
- 顺时针转动转子时，`gDeskMianMbr.currentHeight` 变大。
- `hal1` 与 `hal2` 表现为两路近似正交的正弦波/余弦波信号，随转动周期性变大、变小。

当前结论：

- `AppHeight_CalcAnalogElectricAngle()` 的角度方向与当前高度累计方向一致。
- `APP_HEIGHT_DIR_SIGN` 当前不需要调整。
- 该结论只证明“高度增量方向”和“Hall 观测角度连续性”基本正确，不等于已经确定 FOC 电角度零偏。

下一步重点：

- 继续确认 `electricAngle` 跨 `0/65535` 时 `cumulativeAngle` 是否连续，不出现异常大跳变。
- 通过低电流静态定向实验确认 `Hall机械角观测值 -> FOC rotor electrical angle` 的零偏 `offset`。
- 恢复驱动时必须先用小电流、短超时、母线/相电流/故障门控，不再使用六步开环拖转。

### 2026-05-16：确认 4030 电机参数并加入 Hall FOC 零偏变量

本次更新确认当前工程使用 `4030` 电机默认参数。`include/mcs/mcs_bp_customer.h` 中当前配置为：

```c
#define Motor_Model 4030
```

对应默认参数：

| 参数 | 当前值 |
| --- | --- |
| `Pole_Pairs` | `4` |
| `Rs` | `0.43` |
| `LD` | `0.74` |
| `LQ` | `0.765` |

新增变量：

| 变量 | 归属层 | 作用 |
| --- | --- | --- |
| `gHallFocOffset` | MCS / Hall FOC 标定层 | Hall 观测角度乘极对数后的电角度到 FOC 转子电角度的零偏，当前按反向角度公式固化为 `10632`，可在 Keil watch 中修改 |
| `gHallFocMechAngle` | MCS / Hall FOC 观测层 | Hall 原始观测角度，未乘极对数 |
| `gHallFocElecAngle` | MCS / Hall FOC 观测层 | Hall 原始观测角度乘 `Pole_Pairs` 后的电角度 |

涉及文件：

- `include/mcs/mcs_variable.h`
  - 声明 `extern volatile u16 gHallFocOffset`。
- `include/mcs/mcs_foc_hw_type.h`
  - 声明 `MCS_HallFoc_ApplyOffset()`。
- `src/mcs/mcs_foc_hw.c`
  - 定义 `MCS_HALL_FOC_USE_POLE_PAIRS = 1U`。
  - 定义 `MCS_HALL_FOC_REVERSE_ANGLE = 1U`，当前按 `offset - hallAngle * Pole_Pairs` 计算 FOC 电角度。
  - 定义 `MCS_HALL_FOC_FIXED_OFFSET = 10632U`。
  - 定义 `volatile u16 gHallFocOffset = MCS_HALL_FOC_FIXED_OFFSET`。
  - 新增 `MCS_HallFoc_ApplyOffset(u16 hallAngle)`，先计算 `hallAngle * Pole_Pairs`，再通过 16 位自然回绕得到 `gHallFocOffset - hallElecAngle` 后的 FOC 电角度。

当前用途：

```text
Hall angle
  -> hallAngle * Pole_Pairs
  -> gHallFocOffset - hallElecAngle
  -> MCS_HallFoc_ApplyOffset(hallAngle)
  -> FOC rotor electrical angle
```

注意：

- `gHallFocOffset` 已按 2026-05-18 连续复位学习结果、`Pole_Pairs = 4` 和反向 Hall 角度公式重新换算后固化为 `10632`。
- 如果更换电机、相线顺序、Hall A/B 接线或角度算法，需要重新学习该 offset。
- 高度累计只依赖角度增量连续；FOC 驱动必须额外确认这个零偏。

### 2026-05-19：确认 Hall 角度周期与 FOC 角度换算关系

本次通过手动转动机械一圈并观察 `hal1/hal2` 波形，确认当前两路模拟 Hall 是一组机械周期信号：

```text
机械角 360°
  -> Hall A/B 原始波形完成 1 个周期
  -> gHallLearnElectricAngle 完成 0..65535 一圈
```

因此虽然变量名仍沿用 `electricAngle`，但当前物理含义按 Hall 机械角观测值处理。FOC 使用前必须乘极对数：

```text
Hall机械角 = gHallLearnElectricAngle
Hall电角度 = Hall机械角 * Pole_Pairs
4030电机 Pole_Pairs = 4
```

当前实测 Hall A/B 的角度增长方向与 FOC 需要的电角度方向相反，所以最终 FOC 角度链路固定为：

```text
FOC电角度 = gHallFocOffset - gHallLearnElectricAngle * Pole_Pairs
```

当前固化值：

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `Pole_Pairs` | `4` | 机械一圈对应 4 个电周期 |
| `MCS_HALL_FOC_REVERSE_ANGLE` | `1U` | 使用反向 Hall 角度公式 |
| `gHallFocOffset` | `10632` | 按反向公式和零偏学习结果折算后的固定值 |

注意事项：

- 后续文档中若提到 `gHallLearnElectricAngle`，应理解为“Hall 机械角观测值”，不是 FOC 可直接使用的电角度。
- 如果后续更换 Hall 安装位置、A/B 接线、相线顺序或角度算法，必须重新确认方向和 offset。
- 当前 Hall 波形不是理想正弦/余弦，存在畸变和幅值不一致；`center/range` 归一化是必要处理，后续追求更顺滑时可考虑角度查表校正。

### 2026-05-16：加入手动请求式低电流静态定向测试

本次更新加入 Hall FOC 零偏标定前的低电流静态定向测试入口。该入口默认不自动运行，只能通过函数调用或 Keil watch 手动置位请求变量触发。

目的：

```text
给一个很小、固定方向的三相 PWM 矢量
  -> 转子应轻微吸附到固定位置
  -> 不要求连续旋转
  -> 用于后续标定 gHallFocOffset
```

新增变量：

| 变量 | 归属层 | 作用 |
| --- | --- | --- |
| `gHallFocAlignRequest` | MCS / 调试请求 | Keil watch 中置 `1`，请求执行一次静态定向脉冲 |
| `gHallFocAlignActive` | MCS / 调试状态 | 当前静态定向脉冲是否正在输出 PWM |
| `gHallFocAlignTick` | MCS / 调试状态 | 当前/最近一次静态定向脉冲持续的 1ms tick 数；结束后保留最后值 |
| `gHallFocAlignDoneCount` | MCS / 调试状态 | 每次静态定向脉冲停止时自增，用于确认一次请求已完成 |
| `gHallFocAlignResult` | MCS / 调试状态 | 最近一次静态定向启动或中途保护拦截原因 |

涉及文件：

- `include/mcs/mcs_motor.h`
  - 新增 `MCS_Motor_RequestHallFocAlignTest()`。
  - 新增 `MCS_Motor_AbortHallFocAlignTest()`。
- `include/mcs/mcs_variable.h`
  - 声明 `gHallFocAlignRequest`、`gHallFocAlignActive`、`gHallFocAlignTick`、`gHallFocAlignDoneCount`、`gHallFocAlignResult`。
- `src/mcs/mcs_motor.c`
  - 新增 `MCS_HALL_FOC_ALIGN_TICKS_MS = 300`，静态定向最长持续约 300ms。
  - 静态定向矢量幅值当前使用 `MCS_HALL_FOC_ALIGN_DEFAULT_REF = 96`，用于让零偏学习有可感知吸附力。
  - `MCS_Motor_Task1ms()` 中调用静态定向服务函数，只有 `gHallFocAlignRequest == 1` 时才尝试启动。
  - 启动和运行中都会复用 `MCS_Motor_CheckStartSafety()`，检查 `gMainStatus.Error`、`gBusVoltageStatus`、`gPhaseCurrentStatus`。
  - 超时、故障、电压异常或相电流异常都会 `PwmAOutputs(DISABLE)` 并回到中性 PWM 比较值。

当前使用方式：

```text
1. 上电后确认:
   gBusVoltageStatus == MCS_BUS_VOLTAGE_NORMAL
   gPhaseCurrentStatus == MCS_PHASE_CURRENT_NORMAL
   gMainStatus.Error == 0

2. Keil watch 中把:
   gHallFocAlignRequest = 1

3. 观察 300ms 内现象:
   gHallFocAlignActive 变为 1 后自动回 0
   gHallFocAlignTick 从 0 增加，结束后保留约 300
   gHallFocAlignDoneCount 每完成一次请求自增
   转子应轻微吸附，不应连续旋转
   电流不应明显升高
```

异常现象与处理：

- 如果出现明显嗡鸣、抖动、发热、电流升高，立即断电或调用 `MCS_Motor_AbortHallFocAlignTest()`。
- 如果 `gHallFocAlignResult != MCS_MOTOR_START_OK`，先看结果枚举确认是故障位、母线电压还是相电流状态拦截。
- 本测试只用于静态吸附观察，不代表 FOC 闭环已经恢复。

### 2026-05-17：加入手动请求式低 q 轴 Hall 角度跟随测试

本次更新在静态定向之后加入一个更接近后续 FOC 的低风险测试：使用 `APP_HEIGHT_STATUS.electricAngle`，经过 `gHallFocOffset` 修正后，输出一个短时间、低幅值、`d=0` 的 q 轴电压矢量。

目的：

```text
gHallFocOffset - Hall angle * Pole_Pairs
  -> 查表得到 sin/cos
  -> d=0, q=gHallFocQRef
  -> alpha/beta
  -> 三相 PWM 小矢量
```

现阶段它仍然不是电流闭环，也不是速度闭环；它只是用 Hall 观测角度给一个很小的旋转力矩，用来观察 offset、方向和电流是否正常。

新增变量：

| 变量 | 归属层 | 作用 |
| --- | --- | --- |
| `gHallFocQTestRequest` | MCS / 调试请求 | Keil watch 中置 `1`，请求执行一次低 q 轴矢量测试；当前也可由上电自动测试置位 |
| `gHallFocQTestActive` | MCS / 调试状态 | 当前低 q 轴测试是否正在输出 PWM |
| `gHallFocQRef` | MCS / 调试给定 | q 轴电压矢量幅值，单位是 PWM compare counts，默认 `64`，软件限制 `-96..96` |
| `gHallFocQTestTick` | MCS / 调试状态 | 当前/最近一次低 q 轴测试持续的 1ms tick 数；结束后保留最后值 |
| `gHallFocQTestDoneCount` | MCS / 调试状态 | 每次低 q 轴测试停止时自增 |
| `gHallFocQTestResult` | MCS / 调试状态 | 最近一次低 q 轴测试启动或中途保护拦截原因 |
| `gHallFocQTestAngle` | MCS / 调试状态 | 最近一次输出使用的 `gHallFocOffset - hallAngle * Pole_Pairs` 后电角度 |
| `gHallFocQTestAutoEnable` | MCS / 调试开关 | 上电自动低 q 轴 Hall 测试开关，当前默认 `0` |
| `gHallFocQTestAutoDone` | MCS / 调试状态 | 上电自动低 q 轴 Hall 测试是否已经触发 |
| `gHallFocQTestAutoTick` | MCS / 调试状态 | 上电自动低 q 轴 Hall 测试等待计时 |

涉及文件：

- `include/mcs/mcs_motor.h`
  - 新增 `MCS_MOTOR_START_BLOCKED_HALL_NOT_READY`，用于 Hall 角度校准未完成时拦截启动。
  - 新增 `MCS_Motor_RequestHallFocQTest()`。
  - 新增 `MCS_Motor_AbortHallFocQTest()`。
- `include/mcs/mcs_variable.h`
  - 声明 `gHallFocQTestRequest`、`gHallFocQTestActive`、`gHallFocQRef`、`gHallFocQTestTick`、`gHallFocQTestDoneCount`、`gHallFocQTestResult`、`gHallFocQTestAngle`。
- `src/mcs/mcs_motor.c`
  - 从原项目 `learn_p` 的 `Trig_Functions()` 迁移 256 点 Q15 正弦表，只用于当前调试测试。
  - 新增 `MCS_HALL_FOC_Q_TEST_TICKS_MS = 3000`，测试最长持续约 3s。
  - 新增 `MCS_HALL_FOC_Q_TEST_DEFAULT_REF = 64`，默认 q 轴幅值与前面能转动的开环正弦测试同量级。
  - 新增 `MCS_HALL_FOC_Q_TEST_MAX_ABS = 96`，限制手动给定，避免误写过大。
  - `MCS_HALL_FOC_Q_TEST_AUTO_ENABLE_DEFAULT` 当前重新设为 `1`。本次已在 Hall 角度进入 FOC 前补上 `Pole_Pairs` 换算，自动测试恢复为 3s、q 轴幅值 64 的验证。
  - `MCS_Motor_Task1ms()` 中调用低 q 轴测试服务函数；静态定向测试、零偏学习和低 q 轴测试互斥运行。

当前使用方式：

```text
1. 上电后先确认:
   APP_HEIGHT_STATUS.calibrated == 1
   gBusVoltageStatus == MCS_BUS_VOLTAGE_NORMAL
   gPhaseCurrentStatus == MCS_PHASE_CURRENT_NORMAL
   gMainStatus.Error == 0

2. 当前默认上电自动触发一次:
   gHallFocQRef = 64
   gHallFocQTestAutoEnable = 1

3. 观察 3s 内现象:
   gHallFocQTestActive 变为 1 后自动回 0
   gHallFocQTestTick 从 0 增加，结束后保留约 3000
   gHallFocQTestDoneCount 每完成一次请求自增
   gHallFocQTestAngle 应随 Hall 机械角观测值和 gHallFocOffset 改变
   转子应出现轻微、连续的力矩趋势，方向由 gHallFocQRef 正负决定
   gPhaseCurrentPeakAbsAdc 不应明显冲高，电机不应快速发热
```

调试判断：

- `gHallFocQTestResult == MCS_MOTOR_START_BLOCKED_HALL_NOT_READY`：Hall min/max 中心点校准还没完成，先手动转动让 `APP_HEIGHT_STATUS.calibrated` 变为 `1`。
- `gHallFocQRef > 0` 和 `< 0` 应产生相反方向的力矩趋势；如果两者现象不对称或方向反，需要继续调 `gHallFocOffset` 或确认角度方向。
- 如果只有吸住不连续、或者电流升高明显，先不要加大 `gHallFocQRef`，应继续排查 offset 和相序。

## 12) 文档同步约定

后续每次做结构性更新时，同步更新本文档：

- 新增模块：记录模块职责和归属层。
- 调整变量归属：记录变量由哪一层读写。
- 修改调用链路：更新当前链路图。
- 临时验证代码：标注后续替换方向。

### 2026-05-18：固化 Hall 参数并加入角度归一化

本次更新把上一轮学习到的 Hall A/B 参数固化到 `src/user/app_height.c`，默认不再运行 min/max 学习流程。

固定参数：

| 参数 | 当前值 |
| --- | --- |
| `APP_HEIGHT_FIXED_MIN_A` | `12976` |
| `APP_HEIGHT_FIXED_MAX_A` | `19104` |
| `APP_HEIGHT_FIXED_CENTER_A` | `16040` |
| `APP_HEIGHT_FIXED_RANGE_A` | `6128` |
| `APP_HEIGHT_FIXED_MIN_B` | `12672` |
| `APP_HEIGHT_FIXED_MAX_B` | `19456` |
| `APP_HEIGHT_FIXED_CENTER_B` | `16064` |
| `APP_HEIGHT_FIXED_RANGE_B` | `6784` |

默认配置：

```c
/* #define APP_HEIGHT_ENABLE_HALL_LEARN */
```

该宏保持注释时，`gHallLearnMin/Max/Center/Range` 不再跟随采样变化，只显示固化参数。需要重新学习时再取消注释。

同时，`AppHeight_CalcAnalogElectricAngle()` 已从直接使用原始 A/B 差值改为先按幅值归一化：

```text
x = (hallA - centerA) / (rangeA / 2)
y = (hallB - centerB) / (rangeB / 2)
electricAngle = atan2(y, x)
```

这里的 `electricAngle` 是历史命名，当前按 Hall 机械角观测值理解。这样可以减小 A/B 幅值不一致导致的椭圆误差。新增 `gHallNormX`、`gHallNormY` 供 Keil watch 观察归一化后的轨迹。

### 2026-05-18：补充统一代码风格文档

新增文档：

- [CODING_STYLE.md](CODING_STYLE.md)

后续源码重构按该文档保持命名、注释、编码、分层边界和调试宏风格一致。源码中文注释继续按 GB2312/GBK 保存，文档按 UTF-8 保存。

### 2026-05-18：加入 Hall -> FOC 电角度零偏自动学习

本次更新加入上电自动零偏学习流程，目标是得到：

```text
FOC电角度 = gHallFocOffset - Hall机械角 * Pole_Pairs
```

核心流程：

```text
上电等待 1000ms
  -> 输出一个固定 d 轴小矢量，幅值 96，FOC 定向角度为 49152
  -> 定向保持 600ms
  -> 第 500ms 读取当前 Hall electricAngle（历史命名，物理上按机械角使用）
  -> gHallFocOffset = 49152 + Hall机械角观测值 * Pole_Pairs
  -> 关闭 PWM，只保存 offset，不进入闭环
```

新增/调整变量：

| 变量 | 归属层 | 作用 |
| --- | --- | --- |
| `gHallFocAlignAngle` | MCS / Hall FOC 标定层 | 静态定向使用的 FOC 电角度，当前固定为 `49152` |
| `gHallFocOffsetLearnRequest` | MCS / 调试请求 | 置 `1` 手动请求一次零偏学习 |
| `gHallFocOffsetLearnActive` | MCS / 调试状态 | 零偏学习是否正在输出 PWM |
| `gHallFocOffsetLearnCaptured` | MCS / 调试状态 | 本次是否已经采到 offset |
| `gHallFocOffsetLearnTick` | MCS / 调试状态 | 零偏学习已经运行的 1ms 计数 |
| `gHallFocOffsetLearnDoneCount` | MCS / 调试状态 | 每次零偏学习结束时自增 |
| `gHallFocOffsetLearnAlignAngle` | MCS / 标定结果 | 本次学习使用的 FOC 定向角度 |
| `gHallFocOffsetLearnHallAngle` | MCS / 标定结果 | 本次学习采到的 Hall 机械角观测值 |
| `gHallFocOffsetLearnOffset` | MCS / 标定结果 | 本次学习得到的 offset |
| `gHallFocOffsetLearnResult` | MCS / 调试状态 | 最近一次零偏学习启动或运行结果 |
| `gHallFocOffsetLearnAutoEnable` | MCS / 调试开关 | 上电自动零偏学习开关，当前默认 `0` |
| `gHallFocOffsetLearnAutoDone` | MCS / 调试状态 | 上电自动零偏学习是否已触发 |
| `gHallFocOffsetLearnAutoTick` | MCS / 调试状态 | 上电自动零偏学习等待计时 |

当前 offset 已按 `Pole_Pairs = 4` 和反向 Hall 角度公式换算后固化为 `10632`，所以上电自动零偏学习默认关闭；需要重新学习时，可手动置 `gHallFocOffsetLearnAutoEnable = 1` 或 `gHallFocOffsetLearnRequest = 1`。开环正弦测试仍可通过 `gOpenLoopSineRequest = 1` 手动触发。

观察重点：

- `gHallFocOffsetLearnDoneCount` 自增，说明自动学习完成。
- `gHallFocOffsetLearnCaptured == 1`，说明已经采到 Hall 角度。
- `gHallFocOffsetLearnHallAngle` 是吸附稳定后的 Hall 角度。
- `gHallFocOffsetLearnOffset` 会同步写入 `gHallFocOffset`。
- 如果 `gHallFocOffsetLearnResult != MCS_MOTOR_START_OK`，先看母线电压、相电流状态、故障位或 Hall 校准状态。
