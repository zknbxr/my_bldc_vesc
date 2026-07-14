# MC_MY 代码风格约定

本文用于统一后续重构代码的写法，方便在 Keil、Git diff 和在线调试时快速阅读。

## 1. 编码和注释

- 源码注释使用中文，工程源文件按 GB2312/GBK 保存，避免 Keil 中文乱码。
- 文档文件使用 UTF-8 保存。
- 注释解释“为什么这样做”和“当前阶段限制”，不重复描述显而易见的语句。
- 调试/临时代码必须写清楚用途、风险和后续替换方向。

## 2. 命名

| 类型 | 规则 | 示例 |
| --- | --- | --- |
| 模块公开函数 | `模块名_动作` | `AppHeight_Sample()` |
| 模块内部函数 | `static 模块名_动作` | `static AppHeight_CalcAnalogElectricAngle()` |
| 宏定义 | 全大写，带模块前缀 | `APP_HEIGHT_FIXED_CENTER_A` |
| 全局调试变量 | `g` 前缀 | `gHallLearnElectricAngle` |
| 文件内静态状态 | `s_` 前缀 | `s_appHeight` |
| 类型名 | 保持现有工程风格 | `APP_HEIGHT_STATUS` |

## 3. 分层边界

当前重构继续保持：

```text
APP -> MCS -> HAL/BSP
```

- APP 层处理业务状态、高度、串口命令，不直接写 PWM 寄存器。
- MCS 层处理电机控制、安全门控、测试矢量和 FOC 接入。
- ADC 中断快路径只做采样、保护和必要的观测更新，避免塞入业务流程。
- 跨层访问优先通过接口函数；确实需要 Keil watch 的变量，用 `g` 前缀并在头文件注明用途。

## 4. 格式

- 缩进使用 4 个空格。
- 左花括号独占一行，保持现有工程风格。
- `if`、`while`、`switch` 后保留现有写法：`if(condition)`。
- 常量数字带单位或意义说明，避免裸值散落在逻辑中。
- 一个函数只做一类事情；过长函数优先拆成 `static` 小函数。

## 5. 调试宏

- 编译期开关统一放在文件顶部。
- 默认状态必须安全，危险测试必须显式打开。
- 宏名要能看出层级和用途，例如：

```c
/* #define APP_HEIGHT_ENABLE_HALL_LEARN */
#define MCS_ENABLE_DANGEROUS_OPEN_LOOP_TEST    0
```

## 6. Hall/FOC 当前约定

- `AppHeight` 负责把两路模拟 Hall ADC 转成 `0~65535` 的 Hall 机械角观测值。
- Hall A/B 先减中心点，再按各自幅值归一化，最后做整数 `atan2` 近似。
- 当前实测 Hall A/B 机械转子转 `360°` 才完成一个波形周期，因此 `gHallLearnElectricAngle` 命名沿用旧变量，但物理含义按机械角使用。
- 4030 电机 `Pole_Pairs = 4`，FOC 使用前必须把 Hall 机械角乘极对数变成电角度。
- 当前 Hall A/B 角度方向与 FOC 电角度方向相反，FOC 角度公式为 `gHallFocOffset - hallAngle * Pole_Pairs`。
- `gHallLearnMin/Max/Center/Range` 是 Hall 参数观察和固化入口。
- `gHallNormX/gHallNormY` 是归一化后的调试量，用来观察 A/B 是否接近圆形轨迹。
- `MCS_HallFoc_ApplyOffset()` 只负责应用 Hall 到 FOC 的电角度零偏。
- 真正接入 FOC 前，必须先完成 offset 标定、母线电压状态、相电流状态和故障位检查。

## 7. 文档同步

后续每次做结构性改动，同步更新文档：

- 改模块职责：更新 `README_ARCHITECTURE.md`。
- 改全局变量归属：更新 `GLOBAL_VARIABLE_OWNERSHIP.md`。
- 改编码/命名/调试约定：更新本文档。
