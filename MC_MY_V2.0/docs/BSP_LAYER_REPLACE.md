# BSP 替换层说明（LKS32MC03x）

该层用于替换原有 `lks32mc03x_*` 直接散调用方式，特点：
- 所有寄存器访问统一经 `BSP_REG_*` 宏。
- 时钟/GPIO/ADC/MCPWM/CMP/UART/看门狗都支持独立 `Enable/Disable`。
- 支持仿真测试：`BSP_SIMULATION` 编译开关 + `BSP_Sim*` 注入接口。

## 文件

- `include/bsp/bsp_lks32mc03x_if.h`
- `src/bsp/bsp_lks32mc03x_if.c`

## 核心能力

1. 寄存器读写宏
- `BSP_REG_READ32`
- `BSP_REG_WRITE32`
- `BSP_REG_SET_BITS32`
- `BSP_REG_CLR_BITS32`
- `BSP_REG_MODIFY32`
- `BSP_REG_WRITE_FIELD32`

2. 外设独立开关
- `BSP_PeriphEnable(BSP_PERIPH_*)`
- `BSP_PeriphDisable(BSP_PERIPH_*)`
- `BSP_PeriphIsEnabled(BSP_PERIPH_*)`

3. 外设初始化接口
- 时钟：`BSP_ClockInit`
- GPIO：`BSP_GpioInit`
- ADC：`BSP_AdcInit`
- MCPWM：`BSP_McpwmInit`
- CMP：`BSP_CmpInit`
- UART：`BSP_Uart0Init`
- WDG：`BSP_WdgInit`

4. 仿真测试接口
- `BSP_SimReset`
- `BSP_SimSetReg / BSP_SimGetReg`
- `BSP_SimSetPeriphEnabled`
- `BSP_SimSetTimeUs / BSP_SimAdvanceUs`
- `BSP_SimSetAdcFrame`
- `BSP_SimInjectUartRxByte`
- `BSP_SimSetCmpIrqFlag`

## 编译方式

- 真机构建：默认（不定义 `BSP_SIMULATION`）
- 仿真构建：添加宏 `BSP_SIMULATION`

## 迁移建议

1. 先将现有 `hardware_init.c` 中外设初始化替换为 `BSP_*Init`。
2. 将中断中寄存器直接访问替换为 `BSP_REG_*` + `BSP_*` 接口。
3. 在主循环与单测中用 `BSP_Sim*` 注入采样/故障，验证控制与保护逻辑。
