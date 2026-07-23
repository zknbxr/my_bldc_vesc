# MC_MY_V2.0 电机控制代码阅读与修改指南

> 文档对应当前工程状态，整理日期：2026-07-23。
>
> 本工程是从 VESC FOC 思路移植到 LKS32MC03x 的定点实现。源码采用 GB2312/CP936 和 CRLF；控制核心不使用浮点数。

## 1. 当前系统做到了什么

当前正式控制链路已经具备以下功能：

- 两电阻相电流采样、第三相电流重构。
- Clarke、Park、d/q 电流 PI、反 Park 和 SVM。
- 无感磁链观测器、非线性磁链幅值校正、角度 PLL。
- 两路线性霍尔的在线学习、Flash 保存、上电加载和运行角度预测。
- 1 ms 速度外环，输出 q 轴电流目标。
- 电流目标斜坡、正反转、串口启停。
- 硬件比较器快速关断和软件持续过流保护。
- 学习模式与正常控制模式分离。

当前默认配置为：

| 项目 | 当前值 | 位置 |
|---|---:|---|
| 上电工作模式 | `MCS_WORK_MODE_CONTROL` | `include/mcs/mcs_const.h` |
| 控制角度来源 | `FOC_SENSOR_MODE_HALL` | `include/mcs/mcs_const.h` |
| 默认目标速度 | 4000 ERPM | `MCS_SPEED_TARGET_DEFAULT_ERPM` |
| PWM/电流环频率 | 14 kHz | `PWM_FREQ` |
| 速度环频率 | 1 kHz | `Mcs_Task_Run()` |
| 霍尔绝对角计算频率 | 3.5 kHz | 14 kHz / 4 |
| 速度 PI 驱动电流上限 | 1500 mA | `gMotorSpeedIqLimitMa` |
| 速度 PI 制动电流上限 | 1000 mA | `MCS_SPEED_BRAKE_IQ_LIMIT_MA` |
| 软件相电流保护 | 2000 mA，连续 3 ms | `gMotorCurrentLimitMa` |
| 电机相电阻 | 254 mOhm | `FOC_MOTOR_R_MOHM` |
| 电机相电感 | 343 uH | `FOC_MOTOR_L_UH` |
| 电机磁链 | 4450 uWb | `FOC_MOTOR_FLUX_LINKAGE_UWB` |

这些值是当前调试值，不等于最终产品参数。修改前先确认单位。

## 2. 一张图看懂代码框架

```text
串口 9 字节命令
    |
    v
User_App_DispatchUartCommand()
    |  gMotorCommand（运行请求、方向、速度）
    v
Motor_FaultTask1ms()                   1 ms 独立故障监控
    |
    v
Motor_ControlTask1ms()                 1 ms 模式和运行状态机
    |  LEARN/CONTROL -> motor_control_request_t
    |  启动延时、停机斜坡、PWM 启停
    v
Motor_SpeedControlUpdate1ms()          1 ms 速度外环
    |  m_iq_set_target (mA)
    v
Motor_CurrentCommandUpdate()           1 ms 电流指令斜坡
    |  m_iq_set (mA)
    +--------------------------------------------------+
                                                       |
PWM 触发 ADC -> ADC_IRQHandler()                       |
                    |                                  |
                    v                                  |
               AdcEocHandler()                         |
                    |                                  |
                    +-> Hall_FastUpdate() / 无感角度   |
                    +-> AdcSampleCal()                  |
                           |                            |
                           +-> ADC 电流换算              |
                           +-> Clarke                   |
                           +-> foc_sensorless_update()  |
                           +-> sin/cos                  |
                           +-> 读取 m_iq_set <-----------+
                           +-> Motor_CurrentLoopRun()
                                  |
                                  +-> Park
                                  +-> d/q 电流 PI
                                  +-> 电压矢量限幅
                                  +-> 反 Park
                                  +-> SVM
                                  +-> 下一周期 PWM 比较值
```

控制系统是标准串级结构：

```text
速度目标 -> 速度 PI -> iq 目标 -> 电流 PI -> 电压矢量 -> PWM -> 电机
                              ^                         |
                              |                         v
                              +------ 电流反馈 <--- ADC
```

速度环在外层，电流环在内层。内环必须显著快于外环，所以电流环运行在 ADC 中断，速度环运行在 1 ms 任务。

## 3. 推荐阅读顺序

第一次阅读不要从最长的算法文件开始，按下面顺序更容易建立整体认识：

1. `include/mcs/mcs_const.h`
   先看工作模式、传感器模式、默认速度和全局换算系数。
2. `include/mcs/mcs_motor_type.h`
   看 `motor_state_t`、`mc_configuration`、`motor_all_state_t` 三层数据结构。
3. `src/mcs/mcs_task.c`
   看 1 ms 和 10 ms 慢速任务的调用顺序。
4. `src/user/interrupt.c`
   看 ADC 快速中断入口、耗时统计和硬件故障处理。
5. `src/mcs/mcs_foc_hw.c`
   看 ADC 电流怎样变成 `i_alpha/i_beta`，以及角度和电流环怎样串起来。
6. `src/mcs/mcs_control.c`
   看状态机、速度 PI、电流目标斜坡和保护。
7. `src/mcs/mcs_pwm_foc.c`
   看 Park、电流 PI、反 Park 和 SVM。
8. `src/mcs/mcs_hall.c`
   看线性霍尔学习、Flash 参数和运行角度估算。
9. `src/mcs/sensorless_ctrl.c`
   看磁链观测器、校正项、CORDIC 和 PLL。
10. `src/mcs/mcs_math.c`、`src/mcs/mcs_motor.c`
    看定点三角函数、SVM 和底层辅助运算。
11. `src/bsp/hardware_init.c`、`include/bsp/bsp_user.h`
    最后核对 PWM、ADC 触发、死区和硬件通道。

`mcs_direction_test.c` 已退出正式调度，只保留调试参考。`active_flux.c`、旧接口和旧应用文件不是当前主控制链路，阅读时不要先陷进去。

## 4. 调度与实时性

### 4.1 14 kHz ADC 快速环

`ADC_IRQHandler()` 每个 PWM 周期执行一次，核心调用是：

```text
ADC_IRQHandler()
  -> AdcEocHandler()
       -> Hall_FastUpdate()
       -> AdcSampleCal()
            -> foc_sensorless_update()
            -> Motor_CurrentLoopRun()
  -> Task_vTickTimerEvent()
  -> MCPWM 硬件故障检查
```

快速环中允许做：

- 读取 ADC。
- 简单定点乘加、移位和限幅。
- 角度短周期预测。
- 电流 PI 和 SVM。

快速环中不应做：

- Flash 擦写。
- 串口帧解析或发送。
- 长循环、阻塞延时。
- 无条件的 32 位除法、平方根或大量 CORDIC。
- 与控制无关的显示滤波和统计。

### 4.2 1 ms 慢速控制层

`Mcs_Task_Run()` 的当前顺序很重要：

```text
Motor_FaultTask1ms()
Motor_ControlTask1ms()
Motor_SpeedControlUpdate1ms()
Motor_CurrentCommandUpdate()
Hall_LearnTask1ms()
```

含义是先锁存故障，再由学习模式或操作模式生成统一控制请求；公共运行状态机执行请求，速度环发布电流目标，最后把目标斜坡到快速环实际使用值并推进霍尔学习。

### 4.3 10 ms 慢变量

`Motor_FocSlowUpdate1ms()` 当前在 `tick->ms10` 下调用，实际周期是 10 ms。它负责母线电压换算和低通，不占用电流环时间。

函数名中的 `1ms` 与实际 10 ms 调用不一致，后续可以重命名为 `Motor_FocSlowUpdate()`，但这只是可读性修改，不影响算法。

### 4.4 `errTimer` 的含义

`errTimer` 不是任务周期，它是一次 ADC 中断从进入到结束消耗的 PWM 计数器刻度：

```text
errTimer = MCPWM_CNT0_end - MCPWM_CNT0_start
```

计数器一整个上下计数周期为：

```text
2 * PWM_PERIOD = 2 * 1714 = 3428 ticks
```

如果计数时钟是 48 MHz，可粗略换算：

```text
time_us ~= errTimer / 48
```

`errTimerMax` 越接近 3428，快速环时间裕量越小。调试器观察也可能增加抖动，所以最终判断最好再用 GPIO 翻转配合示波器确认。

## 5. 三层核心数据结构

### 5.1 `motor_state_t`：本控制周期的快速状态

主要字段：

| 字段 | 单位/格式 | 含义 |
|---|---|---|
| `i_alpha`, `i_beta` | mA | Clarke 后的定子电流 |
| `id`, `iq` | mA | Park 后的转子坐标电流 |
| `id_target`, `iq_target` | mA | 电流快速环目标 |
| `phase` | Q16 圈角 | 当前 FOC 电角度 |
| `phase_sin`, `phase_cos` | Q15 | 当前角度正余弦 |
| `vd`, `vq` | Q15 | 电流 PI 输出调制度，不是 mV |
| `vd_int`, `vq_int` | Q15 计数 | PI 积分状态 |
| `mod_alpha_raw`, `mod_beta_raw` | Q15 | 实际送入 SVM 的调制度 |
| `pwm_a/b/c` | 定时器 tick | 三相 PWM 比较值 |
| `v_bus` | mV | 滤波后的母线电压 |

### 5.2 `mc_configuration`：电机与控制参数

这里存放电流 PI、R/L/磁链、电流零偏、最大调制度和电流限制。它描述“这台电机和这套控制器怎样工作”。

### 5.3 `motor_all_state_t`：电机实例与跨层状态

它把配置、快速状态、控制模式、观测器状态和慢速指令串在一起：

```text
m_motor
  +-> m_conf                    配置
  +-> m_motor_state             快速状态
  +-> m_run_state               唯一权威软件运行状态
  +-> m_fault_code              锁存故障
  +-> m_control_mode            SPEED/CURRENT/NONE
  +-> m_id_set_target           慢速层最终 d 轴目标
  +-> m_iq_set_target           速度 PI 输出 q 轴目标
  +-> m_id_set / m_iq_set       斜坡后供快速环使用
  +-> m_observer_state          无感磁链和 PLL
  +-> m_pll_speed               当前速度反馈，ERPM
  +-> m_phase_control_q16       带小数的预测控制角
  +-> p_max_v_mag               初始化缓存的最大电压矢量
```

`m_iq_set_target` 和 `m_iq_set` 不重复：前者是速度环想要的值，后者是经过变化率限制后真正交给电流环的值。

### 5.4 命令、状态和硬件输出不能混用

```text
gMotorCommand.run              应用层“请求运行”
m_motor.m_run_state            软件状态机实际阶段
m_motor.m_control_mode         当前控制算法
Motor_IsPwmEnabled()           MCPWM MOE 是否实际打开
Motor_IsRotorMoving()          估算速度是否超过指定门限
```

`gMotorCommand.run = 1` 不代表电机已经运行，它可能仍在启动延时或已被故障阻止。对外报告软件状态应读取 `m_run_state`；判断功率输出应读取 MOE；判断转子是否真的转动应读取速度。

`motor_control_request_t` 是模式层和公共运行状态机之间的唯一接口。学习模式与操作模式分别填充请求，公共状态机不再包含 `if (learning_control)`。

## 6. 定点格式和物理单位

工程中同样的 C 类型可能代表不同单位，阅读时必须结合字段语义：

| 数据 | 格式 | 实际值 |
|---|---|---|
| 相电流、d/q 电流 | 整数 mA | `600` 表示 600 mA |
| 母线和 alpha/beta 电压 | 整数 mV | `12000` 表示 12 V |
| 相电阻 | mOhm | `254` 表示 0.254 Ohm |
| 相电感 | uH | `343` 表示 343 uH |
| 磁链 | uWb | `4450` 表示 4.45 mWb |
| 电角度 | Q16 圈角 | 0..65535 对应 0..360 度 |
| 正余弦、调制度 | Q15 | 实际值约为整数 / 32768 |
| 霍尔增益 | Q14 | 实际值为整数 / 16384 |
| 速度 | ERPM | 电气转每分钟 |
| 速度 PI 参数 | Q10 | 实际值为整数 / 1024 |

角度换算：

```text
degree = phase_q16 * 360 / 65536
phase_q16 = degree * 65536 / 360
```

机械转速与电气转速：

```text
mechanical_rpm = ERPM / pole_pairs
ERPM = mechanical_rpm * pole_pairs
```

## 7. ADC 电流采样

### 7.1 两相采样与符号

当前硬件使用下桥臂采样电阻，软件采用下面的极性：

```text
i_u = (offset_u - raw_u) * current_scale
i_v = (offset_v - raw_v) * current_scale
i_w = -(i_u + i_v)
```

代码中：

```text
current_scale = MCS_CURRENT_ADC_TO_MA_Q15 / 32768
```

也就是说 ADC 高于零偏时，软件相电流为负；ADC 低于零偏时，软件相电流为正。这个符号已经和当前相序及正 q 轴转矩定义匹配，不要仅凭“低边采样”再次加负号。

### 7.2 第三相重构

星形三相系统忽略中性线电流时：

```text
i_u + i_v + i_w = 0
i_w = -(i_u + i_v)
```

`curr0`、`curr1` 已经在 ADC 转换函数中饱和到 `s16`，第三相由两者相加，范围可能达到 +/-65535，所以 `curr2` 仍必须做一次 `FocHw_SatS16()`。

### 7.3 Clarke 变换

当前使用：

```text
i_alpha = i_u
i_beta  = (i_u + 2*i_v) / sqrt(3)
```

定点常数：

```text
ONE_BY_SQRT3 = 18919  ~= 1/sqrt(3) * 32768
TWO_BY_SQRT3 = 37837  ~= 2/sqrt(3) * 32768
```

### 7.4 零偏校准

`CurrentOffsetCalibration()` 在 PWM 关闭和中断关闭时采 512 次平均：

```text
offset = sum(raw) / 512
```

上电等待不能随意删除，否则运放、ADC 参考或采样电容尚未稳定，零电流偏置会直接变成虚假 d/q 电流。

## 8. FOC 电流内环

### 8.1 Park 变换

使用同一时刻的 `sin/cos` 快照：

```text
id = i_alpha*cos(theta) + i_beta*sin(theta)
iq = i_beta*cos(theta)  - i_alpha*sin(theta)
```

每次 Q15 乘法结果右移 15 位。局部 `trig` 不是无意义复制，它保证两次读取属于同一个角度快照，也便于编译器把值放在寄存器中。

### 8.2 电流 PI

误差：

```text
ed = id_target - id
eq = iq_target - iq
```

比例项：

```text
pd = ed * Kp
pq = eq * Kp
```

积分项：

```text
vd_int[k] = vd_int[k-1] + (ed * Ki >> 15)
vq_int[k] = vq_int[k-1] + (eq * Ki >> 15)
```

代码额外保留 `vd_int_residual/vq_int_residual`，把不足一个整数计数的小数余量留到下次累积。删除它会使小误差附近出现明显死区。

当前离散增益：

```text
Kp = 4    Q15 modulation / mA
Ki = 6933 Q15 modulation / (mA * fast-loop tick)
```

这里的 `Ki` 已经包含 14 kHz 采样周期。改变 PWM 频率后不能原样使用。

### 8.3 电压矢量限幅

为了避免快速环使用平方根，代码用下面公式近似矢量模长：

```text
mag ~= max(|vd|, |vq|) + 27/64 * min(|vd|, |vq|)
```

超出上限时，d/q 两轴等比例缩小，以保持电压矢量方向。

最大电压矢量在初始化时缓存：

```text
max_v_mag = l_max_duty * foc_overmod_factor * sqrt(3)/2
```

三项均为 Q15。当前约为：

```text
l_max_duty        = 30000
foc_overmod_factor = 32767
max_v_mag         ~= 25980
```

如果运行中修改 `l_max_duty` 或 `foc_overmod_factor`，必须重新计算 `p_max_v_mag`，否则配置值和实际限幅不一致。

### 8.4 反 Park

```text
v_alpha = vd*cos(theta) - vq*sin(theta)
v_beta  = vd*sin(theta) + vq*cos(theta)
```

这里得到的是 Q15 调制度，保存到 `mod_alpha_raw/mod_beta_raw`，不是 mV。无感观测器需要电压时，再结合母线电压换算：

```text
v_alpha_mV = mod_alpha * (2/3 * v_bus) / 32768
v_beta_mV  = mod_beta  * (2/3 * v_bus) / 32768
```

### 8.5 SVM 与 PWM

`FOC_SVM_Q15()` 根据 alpha/beta 电压矢量判断扇区，计算两个有效矢量时间和零矢量居中时间，最后得到 `tA/tB/tC`。

比较值范围：

```text
0 <= tA, tB, tC <= PWM_PERIOD
PWM_PERIOD = 1714
```

`Foc_WriteSvmPwm()` 写入的是下一 PWM 周期使用的比较值，这就是采样和执行天然相差一个控制周期的原因。

## 9. 速度外环

### 9.1 速度环输出不是 PWM

速度 PI 不直接计算占空比，它输出 q 轴电流目标：

```text
speed error -> speed PI -> iq_target -> current PI -> voltage -> PWM
```

负载增加后速度下降，速度误差增大，PI 自动提高 `iq_target`；达到电流上限后，即使速度还不足也不能继续增加转矩。

### 9.2 目标速度和方向

`gMotorCommand.speed_target_erpm` 保存速度幅值，方向由 `gMotorCommand.direction` 决定：

```text
target_signed = abs(target) * direction
direction = +1 正转
direction = -1 反转
```

正反转不应修改 ADC 电流符号，也不应镜像霍尔原始输入，只改变有符号速度/转矩命令。

### 9.3 速度目标斜坡

```text
delta_speed_per_call = ramp_erpm_per_s * elapsed_ms / 1000
```

余数会累计，避免低斜率因整数除法永久变成 0。当前 `6000 ERPM/s`，从 0 到 4000 ERPM 理论上约需 0.67 s。

### 9.4 速度反馈滤波

速度反馈使用 Q8 状态低通，当前时间尺度约 8 ms。反馈过度滤波会使负载响应慢；滤波过弱会把霍尔角噪声送入 PI，引起电流和速度来回摆动。

### 9.5 速度 PI

```text
e = speed_target_ramped - speed_feedback
P_q10 = e * Kp_q10
I_q10 += e * Ki_q10 * dt_ms / 1000
iq_command_mA = (P_q10 + I_q10) / 1024
```

当前：

```text
Kp_q10 = 410  -> 0.400 mA/ERPM
Ki_q10 = 96   -> 0.09375 mA/(ERPM*s)
```

积分带抗饱和：当输出已经达到限幅且误差还会把输出推得更深时，暂停积分；反向误差仍允许积分退出饱和。

驱动与制动限幅按方向区分：

- 正转：正 iq 最大 1500 mA，反向制动最大 1000 mA。
- 反转：负 iq 最大 -1500 mA，正向制动最大 1000 mA。

### 9.6 电流指令斜坡

速度 PI 发布 `m_iq_set_target` 后，`Motor_CurrentCommandUpdate()` 以当前 20 mA/ms 逼近：

```text
m_iq_set[k] = move_towards(m_iq_set, m_iq_set_target, 20 * elapsed_ms)
```

它用于抑制 q 轴电流突变。值太小会导致负载变化时速度恢复慢；值太大会让启动、反转和制动冲击变大。

## 10. 两种角度来源

### 10.1 统一出口

无论角度来自霍尔还是无感，最终都必须写入：

```text
m_motor.m_motor_state.phase
```

电流环只认这个统一角度，不需要知道角度来源。

### 10.2 无感模式

`foc_sensorless_update()` 只在 `foc_sensor_mode == SENSORLESS` 时工作。观测器被分时执行，减少单次中断峰值：

- 累计四个 PWM 周期的电压和电流。
- 更新磁链积分。
- 在另一个时隙做磁链幅值校正。
- 再在一个时隙做 `atan2` 和 PLL。
- 中间 PWM 周期用 PLL 速度外推控制角。

当前无 HFI、初始位置检测和可靠开环拖动，因此零速时没有足够反电动势信息。直接无感启动可能先摆动，或者需要扰动才能建立方向。这个限制属于算法信息不足，不是简单提高 PI 就能完全解决。

### 10.3 霍尔模式

当前使用两路线性霍尔。学习完成后，运行时每四个 PWM 周期计算一次绝对电角度，其余周期按估算速度外推。

绝对角更新：

```text
hall_a/b -> 一级中心和增益校正
         -> x = hall_a_norm - hall_b_norm
         -> y = hall_a_norm + hall_b_norm
         -> 二级中心和增益校正
         -> mechanical_phase = atan2(y, x)
         -> electrical_phase = direction * pole_pairs * mechanical_phase
                               + electrical_offset
```

速度来自相邻绝对角的有符号环绕差：

```text
delta_phase = int16(phase_now - phase_last)
speed_step_target_q16 = delta_phase * 65536
speed_step += (speed_step_target - speed_step) / 16
ERPM = speed_step_q16 / 20452
```

中间三个 PWM 周期：

```text
phase_control += speed_step_q16 / 4
```

`20452` 依赖 14 kHz PWM 和四分频。PWM 频率或分频变化后必须重算。

## 11. 无感磁链观测器数学

### 11.1 电压模型

观测器状态近似为：

```text
eta = integral(v - R*i) dt - L*i
```

离散到 alpha/beta 两轴：

```text
eta_alpha[k] = eta_alpha[k-1]
             + (v_alpha - R*i_alpha) * dt
             - L * (i_alpha[k] - i_alpha[k-1])

eta_beta[k]  = eta_beta[k-1]
             + (v_beta - R*i_beta) * dt
             - L * (i_beta[k] - i_beta[k-1])
```

代码使用 mV、mA、mOhm、uH、us 和 uWb，并通过 `/1000` 完成量纲缩放。

### 11.2 非线性磁链校正

纯电压积分会因零偏漂移，代码把估算矢量长度拉回已知磁链 `lambda`：

```text
flux_error = (lambda^2 - eta_alpha^2 - eta_beta^2) / lambda^2
eta_alpha += gain * flux_error * eta_alpha
eta_beta  += gain * flux_error * eta_beta
```

当估算磁链过大时误差为负，矢量收缩；过小时误差为正，矢量放大。

### 11.3 原始角度和 PLL

```text
phase_raw = atan2(eta_beta, eta_alpha)
phase_error = wrap(phase_raw - phase_pll)
phase_pll += speed_step + Kp_pll * phase_error
speed_step += Ki_pll * phase_error
```

当前 PLL 参数：

```text
Kp = 4096 Q15
Ki = 64   Q15
```

速度换算同样使用：

```text
ERPM = pll_speed_step_q16 / 20452
```

修改 R、L、磁链时一次只改一类参数并记录波形。先保证电流、电压和方向正确，再调观测器；错误的采样符号无法靠观测器增益修好。

## 12. 霍尔在线学习

### 12.1 顶层模式

编译期宏：

```text
MCS_POWER_ON_WORK_MODE = MCS_WORK_MODE_LEARN 或 CONTROL
MCS_CONTROL_SENSOR_MODE = FOC_SENSOR_MODE_HALL 或 SENSORLESS
```

运行期状态：

```text
gMotorWorkMode
```

学习成功后，本次运行会从 `LEARN` 自动切到 `CONTROL`。

### 12.2 学习模式流程

```text
WAIT_STABLE
  -> RAW
  -> ORTHOGONAL
  -> OFFSET
  -> ALIGN_STOP
  -> ALIGN
  -> COMPLETE
  -> 等待电机完全停止
  -> Flash 保存
  -> CONTROL
```

旋转阶段由公共速度环驱动，不再维护独立学习电流。学习没有固定总超时，每一级都要满足最短时间、机械圈数和连续收敛条件。

当前主要条件：

| 条件 | 当前值 |
|---|---:|
| 每阶段最短时间 | 5000 ms |
| 最小机械圈数 | 12 |
| 连续稳定圈数 | 6 |
| 最低学习速度 | 300 ERPM |
| 最低质量 | 28000 Q15 |
| 固定角稳定样本 | 500 |

### 12.3 第一级：原始霍尔归一化

对两路霍尔分别寻找最大最小值：

```text
center = (max + min) / 2
amplitude = (max - min) / 2
normalized = (raw - center) * gain_q14 / 16384
gain_q14 = target * 16384 / amplitude
```

这一级消除两路霍尔各自的直流偏置和幅值差。

### 12.4 第二级：正交化

90 度安装的两个线性霍尔经过组合：

```text
x_raw = hall_a_normalized - hall_b_normalized
y_raw = hall_a_normalized + hall_b_normalized
```

再对 x/y 做中心和增益归一化，尽量把 XY 轨迹从偏心椭圆修正成以原点为中心的圆。

### 12.5 第三级：电角度方向与动态偏置

```text
theta_mech = atan2(y, x)
theta_elec_candidate = +/- pole_pairs * theta_mech
error = wrap(reference_sensorless_phase - theta_elec_candidate)
```

对误差的 `sin/cos` 做圆周平均，分别评价正向和反向映射，选择向量质量更高的一组，得到 `inverted` 和动态 `electrical_offset`。

这一步依赖无感参考角。如果无感角存在固定相位延迟，动态偏置会吸收大部分固定误差；如果无感角随机跳变或方向错误，学习质量会下降或无法收敛。

### 12.6 固定角校零

旋转学习结束后先停机，再用固定电角度和 d 轴电流吸附转子。稳定后根据霍尔静态角计算最终偏置，消除无感运行相位延迟对零点的影响。

固定角阶段是唯一不走速度环的阶段：

```text
control_mode = CURRENT
phase_override = true
id_target = align_current
iq_target = 0
```

### 12.7 Flash 记录

Flash 记录包含：

- `magic`
- `version`
- 记录长度
- 全部 `hall_calibration_t`
- 学习质量
- CRC32

当前版本是 `HALL_FLASH_VERSION = 3`。只要改变记录结构、字段含义或算法产生的参数定义，就应增加版本号，防止旧参数被新固件误用。

## 13. 学习模式与控制模式

### 13.1 学习模式

- 上电不等待串口。
- 强制使用无感角度和公共速度环旋转。
- 在线学习两路线性霍尔。
- 学完停止并固定角吸附校零。
- 保存 Flash。
- 本次运行自动切到控制模式。

### 13.2 控制模式

- 上电默认不转，`gMotorCommand.run = 0`。
- MCPWM 计数器和 ADC 触发保持运行，但 MOE 关闭，六路功率输出不工作。
- 加载有效霍尔参数；如果宏选择霍尔但 Flash 无效，会自动退回学习模式。
- 串口命令决定正转、反转和停止。
- 霍尔和无感都使用同一个速度环与电流环。

### 13.3 串口协议

固定 9 字节：

| 字节 | 内容 |
|---|---|
| BYTE0 | `0xAA` |
| BYTE1 | `0xAA` |
| BYTE2 | 地址，当前 `0x00` |
| BYTE3 | 命令字 |
| BYTE4 | 速度，当前保留未接入 |
| BYTE5 | 电流，当前保留 |
| BYTE6 | 目标行程高字节 |
| BYTE7 | 目标行程低字节 |
| BYTE8 | BYTE2 到 BYTE7 的 8 位累加和 |

命令字：

```text
0x01 正转
0x02 反转
0x03 停止
```

串口控制按命令变化触发。`LastKeyState` 保存上一次有效命令，相同命令的重复帧只更新通信诊断，不重复执行启动或停止状态转换。启动延时由电机状态机中的 `gMotorStartDelayMs` 完成，串口解析本身不阻塞。

目前 BYTE4 尚未写入 `gMotorCommand.speed_target_erpm`，所以速度始终使用宏定义默认值。后续接入时要先明确 BYTE4 的比例，例如 `1 count = 100 ERPM`，并做上下限校验和目标斜坡。

## 14. 状态机和保护

### 14.1 控制状态

`m_motor.m_run_state`：

```text
MOTOR_RUN_STATE_OFF
MOTOR_RUN_STATE_START_DELAY
MOTOR_RUN_STATE_RUNNING
MOTOR_RUN_STATE_STOPPING
MOTOR_RUN_STATE_FAULT
```

状态转换只执行一次寄存器动作：

```text
OFF          不重复写 MOE
START_DELAY  等待启动延时，随后只开启一次 MOE
RUNNING      收到停止后只发布一次零电流目标
STOPPING     电流归零后只关闭一次 MOE
FAULT        Motor_FaultTrip() 已经立即关闭 MOE
```

正常停止不是立刻关 PWM，而是先把 d/q 电流目标斜坡到 0，随后关闭 MOE。硬件故障走立即停机。

### 14.2 软件过流

三相电流任一相连续超过 `gMotorCurrentLimitMa` 达 3 ms 才停机，以过滤单点 ADC 毛刺。它不是硬件短路保护，阈值和延迟不能用来保护 MOS 管的瞬时短路。

### 14.3 硬件保护

比较器直接连接 MCPWM Fail 输入，硬件可以先关闭 MOE。ADC 中断随后读取故障标志、保存诊断寄存器并调用 `StopMotorImmdly()`。

相关 Watch 变量：

```text
gShortFaultCount
gMcpwmEifAtShort
gMcpwmFail012AtShort
gCmpDataAtShort
m_motor.m_fault_code
```

故障码：

```text
MOTOR_FAULT_NONE
MOTOR_FAULT_SOFTWARE_OVERCURRENT
MOTOR_FAULT_HARDWARE_SHORT
```

PWM 未开启本身不是故障。`Motor_FaultTask1ms()` 只在控制算法已激活且 MOE 已开启时累计软件过流；硬件短路完全由 ADC 中断中的 MCPWM Fail 路径锁存。

故障发生后，MOE 关闭、运行命令清零并进入 `FAULT`。当 MOE 保持关闭、MCPWM 没有新的 Fail 事件且 ADC 电流连续 500 ms 低于保护阈值时，故障自动解锁到 `OFF`，但不会自动启动。

故障期间出现的命令变化只更新 `LastKeyState` 而不执行。恢复后必须收到与 `LastKeyState` 不同的新命令才会重新启动。例如上升时发生故障，恢复后收到新的下降命令即可启动，不要求先发送停止命令。

## 15. 参数在哪里改

### 15.1 工作模式、角度源、速度目标

文件：`include/mcs/mcs_const.h`

```text
MCS_POWER_ON_WORK_MODE
MCS_CONTROL_SENSOR_MODE
MCS_SPEED_TARGET_DEFAULT_ERPM
MCS_SPEED_EST_MAX_ERPM
MCS_SPEED_BRAKE_IQ_LIMIT_MA
```

### 15.2 速度环和启动行为

文件：`src/mcs/mcs_control.c`

```text
gMotorSpeedRampErpmPerS
gMotorSpeedKpQ10
gMotorSpeedKiQ10
gMotorSpeedIqLimitMa
gMotorSpeedStartCurrentMa
gMotorSpeedCurrentRampMaPerMs
gMotorSpeedCloseLoopMinErpm
gMotorSpeedCloseLoopStableMs
gMotorStartDelayMs
gMotorCurrentLimitMa
```

### 15.3 电流环、电机参数和调制度

文件：`src/mcs/mcs_pwm_foc.c`

```text
MCS_CURRENT_KP_Q15_PER_MA
MCS_CURRENT_KI_Q15_PER_MA_TICK
MCS_CURRENT_RAMP_MA_PER_MS
MCS_SVM_MAX_MOD_Q15
MCS_MOTOR_CURRENT_MAX_MA
MCS_OVERMOD_FACTOR_Q15
FOC_MOTOR_R_MOHM
FOC_MOTOR_L_UH
FOC_MOTOR_FLUX_LINKAGE_UWB
```

### 15.4 ADC 比例和极性

文件：

- `include/mcs/mcs_const.h`：ADC 到 mA/mV 的比例。
- `src/mcs/mcs_foc_hw.c`：零偏相减方向和第三相重构。
- `src/mcs/mcs_init.c`：零偏校准。
- `include/bsp/bsp_user.h`：ADC 通道宏。

### 15.5 无感观测器

文件：`src/mcs/sensorless_ctrl.c`

主要参数：磁链校正增益、PLL Kp/Ki、分时周期和速度上限。电机 R/L/磁链仍从 `m_conf` 读取。

### 15.6 霍尔学习和运行预测

文件：`src/mcs/mcs_hall.c`

主要参数：每阶段最短时间、圈数、收敛容差、固定角电流、运行分频和速度滤波。

### 15.7 PWM、ADC 触发和死区

文件：

- `include/bsp/bsp_user.h`
- `src/bsp/hardware_init.c`

这些参数直接影响功率级，不要只靠软件波形判断。修改后要用示波器确认互补 PWM、死区、ADC 触发点和故障关断。

## 16. 常见修改的正确步骤

### 16.1 调速度环

1. 先使用霍尔模式，确保角度和速度反馈稳定。
2. 降低电流上限，空载开始。
3. 暂时减小 `Ki`，逐步增加 `Kp`，直到负载响应快但没有持续振荡。
4. 再逐步增加 `Ki`，消除稳态速度误差。
5. 分别测试正转、反转、加载、卸载和制动。
6. 同时观察 `gMotorSpeedErrorErpm`、`gMotorSpeedIqCommandMa`、实际 `iq` 和电流限幅。

如果速度周期性波动：先判断是速度反馈波动还是 PI 输出自身振荡。不要直接同时改霍尔滤波、速度 Kp、Ki 和电流斜坡。

### 16.2 调电流环

1. 固定可靠角度源。
2. 限制母线电压和电流。
3. 给小 q 轴阶跃，观察 `iq_target`、`iq`、`iq_error`、`vq`。
4. 先调 Kp，再加 Ki。
5. 确认 `vd/vq` 没有长期顶在电压矢量上限。

电流环不稳定时不要先调速度环，因为速度环会把内环问题放大。

### 16.3 修改 PWM 频率

不能只改 `PWM_FREQ`。至少联动检查：

- `PWM_PERIOD` 和死区计数。
- ADC 触发点和采样窗口。
- `FOC_CONTROL_DT_US`。
- 电流 PI 离散 `Ki`。
- 无感观测器积分周期和分时频率。
- `OBSERVER_PLL_STEP_Q16_PER_ERPM`。
- `HALL_RUNTIME_STEP_Q16_PER_ERPM`。
- `PWM_TIME_1MS_COUNTER`。
- `errTimer` 预算。

通用速度换算系数为：

```text
step_q16_per_erpm = 65536 * 65536 / (60 * angle_update_frequency_hz)
```

当前 `angle_update_frequency = 14000/4 = 3500 Hz`，因此约为 20452。

### 16.4 修改霍尔参数结构

1. 修改 `hall_calibration_t`。
2. 同步修改 Flash 记录复制和校验。
3. 增加 `HALL_FLASH_VERSION`。
4. 重新烧录学习固件并重新学习。
5. 再切回控制固件验证正反转。

### 16.5 接入串口速度

建议把 BYTE4 明确定义成速度等级，而不是直接当 ERPM：

```text
target_erpm = byte4 * ERPM_PER_COUNT
```

接入点放在 `User_App_DispatchUartCommand()`。必须做：

- 0 值语义定义。
- 最大 ERPM 限幅。
- 正反转只由命令字决定。
- 更新目标后仍经过速度斜坡。
- 通讯超时是否停车的产品策略。

## 17. 调试变量速查

### 17.1 速度环

```text
gMotorCommand.speed_target_erpm
gMotorSpeedTargetRampErpm
gMotorSpeedFeedbackErpm
gMotorSpeedErrorErpm
gMotorSpeedIqCommandMa
gMotorSpeedClosedLoopActive
m_motor.m_pll_speed
m_motor.m_iq_set_target
m_motor.m_iq_set
```

### 17.2 电流环

```text
ADC_curr_norm_value[0..2]
m_motor.m_motor_state.i_alpha
m_motor.m_motor_state.i_beta
m_motor.m_motor_state.id
m_motor.m_motor_state.iq
m_motor.m_motor_state.id_error
m_motor.m_motor_state.iq_error
m_motor.m_motor_state.vd
m_motor.m_motor_state.vq
m_motor.m_motor_state.mod_alpha_raw
m_motor.m_motor_state.mod_beta_raw
m_motor.m_motor_state.pwm_a/b/c
```

### 17.3 霍尔

```text
gHallRawA / gHallRawB
gHallNormX / gHallNormY
gHallMechanicalPhase
gHallElectricalPhase
gHallControlPhase
gHallPhaseError
gHallLearnState
gHallLearnError
gHallLearnQualityQ15
gHallStorageState
```

### 17.4 无感

```text
gObserverFluxErrorQ13
gObserverCorrectionGainQ13
gObserverPllPhaseError
gObserverPllSpeedStepQ16
gObserverPhase
gObserverPhaseError
m_motor.m_observer_state.x1/x2
```

### 17.5 实时性和保护

```text
errTimer
errTimerMax
m_motor.m_run_state
m_motor.m_fault_code
gMotorCommand
gShortFaultCount
MCPWM_FAIL012
```

## 18. 症状与排查方向

| 现象 | 优先检查 |
|---|---|
| 上电立刻转一下 | `gMotorCommand.run` 默认值、工作模式、学习模式是否自动运行 |
| 电机只吸附不旋转 | 控制角是否更新、`m_pll_speed`、`iq_target`、MOE |
| 正转正常反转振荡 | 速度符号、霍尔角差环绕、反转 iq 限幅、相位映射方向 |
| 速度周期性波动 | 霍尔速度噪声、速度 Kp/Ki、iq 是否饱和、负载周期性变化 |
| 负载变化恢复慢 | 速度 Kp/Ki 太小、iq 上限太低、电流斜坡太慢、反馈滤波过重 |
| 切角度源瞬间卡住 | 两角度源相位差、速度符号、PI 积分未平滑接管 |
| iq 误差大且 vq 饱和 | 电压不足、角度错误、电流 PI 参数、母线换算错误 |
| ADC 偶发巨大毛刺 | 采样点、ADC 通道残留、运放恢复、PWM 开关噪声、接地布局 |
| `errTimerMax` 接近 3428 | 中断超时、调试器开销、无条件除法/CORDIC、任务放错层 |
| 无感零速来回摆动 | 零速不可观测；需要 HFI、开环启动或霍尔绝对角 |

## 19. 当前已知限制

- 无感没有 HFI，零速直接启动不保证每次成功。
- 串口速度 BYTE4 尚未接入，当前只使用固定宏速度。
- 母线过压、欠压和回馈制动管理尚未形成完整闭环。
- 温度保护、堵转判断和通讯失联停车仍需产品化补全。
- 电流 PI 目前未加入交叉耦合、反电动势前馈、MTPA 和弱磁。
- 霍尔速度滤波和速度 PI 仍需要在正反转、不同负载下继续整定。
- `m_pll_speed` 这个名字同时承载霍尔估算速度和无感 PLL 速度，语义上可在后续重构为统一的 `m_speed_est_erpm`。
- 一些 VESC 枚举和字段仍保留但未进入当前路径，删除前必须先用 `rg` 确认没有调试或接口引用。

## 20. 修改前后的检查清单

修改前：

- 记录当前可运行固件和参数。
- 明确要改的是快速环、慢速环、角度源还是硬件配置。
- 确认变量单位和定点格式。
- 确认正转和反转的符号定义。

编译后：

- 必须做到 0 Error、0 Warning。
- 检查代码和 RAM/Flash 占用变化。
- 检查 `errTimerMax` 是否明显增加。

低压空载测试：

- 上电默认不转。
- 停止命令能平滑停机并关闭 PWM。
- 正反转速度符号正确。
- 三相电流和约等于 0。
- `id` 接近 0，`iq` 跟随目标。
- 霍尔相位连续，无 180 度或 360 度错误跳变。

加载测试：

- 负载增加时 iq 增大，速度能恢复。
- iq 不长期顶住限幅。
- 卸载时没有强烈回馈或母线过压。
- MOS、采样电阻和电机温升可接受。
- 硬件保护和软件保护都能正确关断。

## 21. 维护原则

1. 快速环只保留必须在本 PWM 周期完成的运算。
2. 外部命令只发布目标，不直接写 PWM。
3. 速度环只输出电流目标，电流环只输出电压目标。
4. 角度来源可以切换，但进入电流环前必须统一成 `state.phase`。
5. 所有跨中断共享变量都要考虑原子性和快照一致性。
6. 每次只调整一类参数，并保留调试波形。
7. 先验证采样符号、相序和角度，再调 PI。
8. 修改频率必须重新审查全部离散系数。
9. Flash 数据结构变化必须增加版本号。
10. 不为了“少一个局部变量”破坏同周期数据快照和可读性。

这份指南应随控制框架更新。新增速度协议、HFI、弱磁、母线保护或新的角度源时，请同步更新对应章节和当前配置表。
