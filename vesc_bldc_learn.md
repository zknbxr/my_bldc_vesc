# BLDC

## 电流中断函数

### 控制模式预处理

读取当前 duty/vq/speed 状态
读取 id/iq 目标
根据刹车模式先限制 iq
滤波当前 duty
判断当前是不是 duty 控制模式
为后面的刹车逻辑、duty 模式逻辑、电流环目标设置做准备

### 控制模式切换的“无冲击衔接逻辑”

1. duty -> current 时：
   重置 q轴电流环积分 vq_int，让电流环平滑接手。
2. current -> duty 之前：
   持续同步 duty PI 积分 m_duty_i_term，让 duty 环下次接手也平滑。

​	这是控制模式切换的“无冲击衔接”逻辑，主要保护 `vq_int` 和 `m_duty_i_term` 两个积分项，避免切换 duty/电流控制时输出突变。

### duty控制模式

如果当前 duty 小于目标：
    给最大电流，并把 max_duty 限到 duty_set

如果当前 duty 大于目标：
    用 PI 降低 iq，让 duty 平滑降下来

### FOC 的转子位置估算与角度选择模块

先更新 observer 角度
 -> 根据传感器模式选择/融合角度
 -> 根据特殊控制模式覆盖角度
 -> 如果手动 override，再强制覆盖
 -> 计算 sin/cos

### MTPA（最大转矩每安培）模式

这段是在开启 MTPA 时，根据电机磁链和 Ld/Lq 差异，把一部分 q 轴电流转成 d 轴电流，以便用同样总电流获得更好的转矩效率。

### 弱磁控制

这段是在高速电压不够时自动或手动加入负 d 轴弱磁电流，并相应减少 q 轴转矩电流，让电机能超过基速运行，同时避免总电流过大。



## 速度环

### 接收速度命令

```
case COMM_SET_RPM:
    mc_interface_set_pid_speed(rpm);
```

foc电机调用

mcpwm_foc_set_pid_speed(DIR_MULT * rpm);

### mcpwm_foc_set_pid_speed()

#### 保存目标速度

设置速度斜坡后把速度环当前目标设为实际速度：应该也就是距离最近的斜坡速度作为当前速度，防止速度命令变换过大。如果没有斜坡就直接设定目标速度。

#### 切换为速度模式

#### 启动电机运行状态

这里要判断速度命令不得小于某值

### 真正速度环

foc_run_pid_control_speed()

由独立的pid_thread周期调用





### 阅读顺序

mcpwm_foc_set_pid_speed()
    ↓
pid_thread()
    ↓
foc_run_pid_control_speed()
    ↓
mcpwm_foc_adc_int_handler() 中的 iq_set_tmp
    ↓
control_current()
    ↓
foc_svm()
    ↓
TIMER_UPDATE_DUTY_M1()

