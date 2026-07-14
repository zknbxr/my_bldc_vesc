#include "main.h"

void delay(u16 cnt)
{
    u16 t_cnt;

    for (t_cnt = 0; t_cnt < cnt; t_cnt++)
    {
        __nop();
    }
}
extern void SYS_EnableWatchDog(void);
void InitWatchDog(void)
{

    // 独立看门狗用于防止程序跑飞，关键寄存器写入前需先写密码
    IWDG_PSW = 0xA6B4;
        // 完成看门狗基础参数配置
    IWDG_CFG = 0x3C00;
    SYS_EnableWatchDog();                                                                          /* 使能看门狗*/


}

/*
时钟初始化
**/
void Clock_Init(void)
{

    // 手册说明芯片支持快速系统时钟，此处将主时钟配到 48MHz
    SYS_WR_PROTECT = 0x7a83;
        // 按 5V 供电场景选择模拟模块参数
    SYS_VolSelModule(1);       // 0: 3.3V ; 1: 5.0V	/* 写保护 */
        // 打开 PLL/模拟基准相关电路
    SYS_AFE_REG0 |= 0xa000;                                                                        /* 使能pll 比较器0 */

        // 等待 PLL 和模拟基准稳定
    delay(5000);
    delay(5000);

        // 切换到 48MHz 快速系统时钟
    SYS_CLK_CFG |= 0x000001ff;                                                                      /* select fast clock 48M*/
    SYS_CLK_DIV2  = 0x0000;                                                                         /* UART clock.. */
        // 打开主要外设的时钟门控
    SYS_CLK_FEN = 0x3ff;                                                                            /* 使能外设时钟 */
}

void SystemInit(void)
{

    Clock_Init();                                                                                   /* 系统时钟初始化 */

}

void InitAdcMotor0(void)
{
    // ADC 用于采样相电流、母线电压等快速控制所需模拟量



    SYS_AnalogModuleClockCmd(SYS_AnalogModule_ADC, ENABLE);                                         /* ADC模块使能 */
        // 使能 EOS0 中断，一轮采样结束后进入 ADC IRQ
    ADC_IE = ADC_EOS0_IRQ_EN;

        // ADC_CFG 配成：左对齐 + MCPWM T0 硬件触发 + 单段转换
    ADC_CFG = (ADC_LEFT_ALIGN << 10) | (ADC_HARDWARE_T0_TRG) | (0x00 << 12) |
              (0x00 << 4) | (ADC_1SEG_TRG << 8);

        // 单段采样每轮连续转换 6 个通道
    ADC_CHNT = (0x06) | (0x00 << 4) |
               (0x00 << 8) | (0x00 << 12);




    SYS_WR_PROTECT = 0x7a83;
        // SAMP_TIME 设置 ADC 采样时间
    SYS_AFE_REG2 = 0x04<<8;                                                                         /* sample time ,4+4 */
    SYS_WR_PROTECT = 0;

        // 清 ADC 状态标志并复位状态机
    ADC_IF = 0xff;
    ADC_CFG |= BIT11;

}

void InitEpwmMotor0(void)
{
    // MCPWM 负责三相 PWM 生成、ADC 触发、死区插入和 FAIL 硬件保护

    MCPWM_InitTypeDef MCPWM_InitStructure;
    MCPWM_StructInit(&MCPWM_InitStructure);

    MCPWM_InitStructure.CLK_DIV = 0;                                                                /* MCPWM时钟分频设置 */
    MCPWM_InitStructure.MCLK_EN = ENABLE;                                                           /* 模块时钟开启 */
    MCPWM_InitStructure.MCPWM_Cnt0_EN = ENABLE;                                                     /* 时基0主计数器开始计数使能开关 */
    MCPWM_InitStructure.MCPWM_Cnt1_EN = ENABLE;                                                     /* 时基1主计数器开始计数使能开关 */
        // 三相通道均配为中心对齐 PWM，便于在周期中心附近采样
    MCPWM_InitStructure.MCPWM_WorkModeCH0 = CENTRAL_PWM_MODE;                                       /* 通道工作模式设置，中心对齐或边沿对齐 */
    MCPWM_InitStructure.MCPWM_WorkModeCH1 = CENTRAL_PWM_MODE;
    MCPWM_InitStructure.MCPWM_WorkModeCH2 = CENTRAL_PWM_MODE;

    /* 自动更新使能寄存器 MCPWM_TH00 自动加载使能 MCPWM_TMR0 自动加载使能 MCPWM_0TH 自动加载使能 MCPWM_0CNT 自动加载使能*/
        // 开启自动更新，让新的比较值和触发点在安全时刻同步生效
    MCPWM_InitStructure.AUEN = TH00_AUEN | TH01_AUEN | TH10_AUEN | TH11_AUEN |
                               TH20_AUEN | TH21_AUEN | TMR0_AUEN | TMR1_AUEN |
                               TMR2_AUEN | TMR3_AUEN | TH0_AUEN | TH30_AUEN | TH31_AUEN ;

    MCPWM_InitStructure.GPIO_BKIN_Filter = 0;                                                      /* 急停事件(来自IO口信号)数字滤波器时间设置 */
    MCPWM_InitStructure.CMP_BKIN_Filter = 0;                                                       /* 急停事件(来自比较器信号)数字滤波器时间设置 */

    MCPWM_InitStructure.TimeBase0_PERIOD = PWM_PERIOD;                                              /* 时期0周期设置 */
    MCPWM_InitStructure.TimeBase1_PERIOD = PWM_PERIOD;                                              /* 时期1周期设置 */

        // T0/T1 事件可作为 ADC 硬件触发源，当前工程主要用 T0
    MCPWM_InitStructure.TriggerPoint0 = (u16)(5 - PWM_PERIOD);                                      /* MCPWM_TMR0 ADC触发事件T0 设置 */
    MCPWM_InitStructure.TriggerPoint1 = (u16)(500-PWM_PERIOD);                                      /* MCPWM_TMR1 ADC触发事件T1 设置 */

        // 上下桥臂插入死区时间，防止功率器件直通
    MCPWM_InitStructure.DeadTimeCH0N = DEADTIME;                                                    /* 死区时间设置 */
    MCPWM_InitStructure.DeadTimeCH0P = DEADTIME;
    MCPWM_InitStructure.DeadTimeCH1N = DEADTIME;
    MCPWM_InitStructure.DeadTimeCH1P = DEADTIME;
    MCPWM_InitStructure.DeadTimeCH2N = DEADTIME;
    MCPWM_InitStructure.DeadTimeCH2P = DEADTIME;

#if (PRE_DRIVER_POLARITY == P_HIGH__N_LOW)                                                          /* CHxP 高有效， CHxN低电平有效 */
    MCPWM_InitStructure.CH0N_Polarity_INV = ENABLE;                                                 /* CH0N通道输出极性设置 | 正常输出或取反输出*/
    MCPWM_InitStructure.CH0P_Polarity_INV = DISABLE;                                                /* CH0P通道输出极性设置 | 正常输出或取反输出 */
    MCPWM_InitStructure.CH1N_Polarity_INV = ENABLE;
    MCPWM_InitStructure.CH1P_Polarity_INV = DISABLE;
    MCPWM_InitStructure.CH2N_Polarity_INV = ENABLE;
    MCPWM_InitStructure.CH2P_Polarity_INV = DISABLE;

    MCPWM_InitStructure.Switch_CH0N_CH0P =  DISABLE;                                                /* 通道交换选择设置 | CH0P和CH0N是否选择信号交换 */
    MCPWM_InitStructure.Switch_CH1N_CH1P =  DISABLE;                                                /* 通道交换选择设置 */
    MCPWM_InitStructure.Switch_CH2N_CH2P =  DISABLE;                                                /* 通道交换选择设置 */

    /* 默认电平设置 默认电平输出不受MCPWM_IO01和MCPWM_IO23的 BIT0、BIT1、BIT8、BIT9、BIT6、BIT14
                                                     通道交换和极性控制的影响，直接控制通道输出 */
    MCPWM_InitStructure.CH0P_default_output = LOW_LEVEL;                                            /* CH1P对应引脚在空闲状态输出低电平 */
    MCPWM_InitStructure.CH0N_default_output = HIGH_LEVEL;                                           /* CH1N对应引脚在空闲状态输出高电平 */
    MCPWM_InitStructure.CH1P_default_output = LOW_LEVEL;
    MCPWM_InitStructure.CH1N_default_output = HIGH_LEVEL;
    MCPWM_InitStructure.CH2P_default_output = LOW_LEVEL;
    MCPWM_InitStructure.CH2N_default_output = HIGH_LEVEL;
#else
#if (PRE_DRIVER_POLARITY == P_HIGH__N_HIGH)                                                         /* CHxP 高有效， CHxN高电平有效 */
    MCPWM_InitStructure.CH0N_Polarity_INV = DISABLE;                                                /* CH0N通道输出极性设置 | 正常输出或取反输出*/
    MCPWM_InitStructure.CH0P_Polarity_INV = DISABLE;                                                /* CH0P通道输出极性设置 | 正常输出或取反输出 */
    MCPWM_InitStructure.CH1N_Polarity_INV = DISABLE;
    MCPWM_InitStructure.CH1P_Polarity_INV = DISABLE;
    MCPWM_InitStructure.CH2N_Polarity_INV = DISABLE;
    MCPWM_InitStructure.CH2P_Polarity_INV = DISABLE;

    /* 默认电平设置 默认电平输出不受MCPWM_IO01和MCPWM_IO23的 BIT0、BIT1、BIT8、BIT9、BIT6、BIT14
                                                     通道交换和极性控制的影响，直接控制通道输出 */
    MCPWM_InitStructure.CH0P_default_output = LOW_LEVEL;                                            /* CH1P对应引脚在空闲状态输出低电平 */
    MCPWM_InitStructure.CH0N_default_output = LOW_LEVEL;
    MCPWM_InitStructure.CH1P_default_output = LOW_LEVEL;
    MCPWM_InitStructure.CH1N_default_output = LOW_LEVEL;
    MCPWM_InitStructure.CH2P_default_output = LOW_LEVEL;
    MCPWM_InitStructure.CH2N_default_output = LOW_LEVEL;
#endif
#endif

    MCPWM_InitStructure.DebugMode_PWM_out = ENABLE;                                                 /* 在接上仿真器debug程序时，暂停MCU运行时，选择各PWM通道正常输出调制信号
                                                                                                       还是输出默认电平，保护功率器件 ENABLE:正常输出 DISABLE:输出默认电平*/
    MCPWM_InitStructure.MCPWM_Base0T0_UpdateEN = ENABLE;                                            /* MCPWM 时基0 T0事件更新使能 */
    MCPWM_InitStructure.MCPWM_Base0T1_UpdateEN = DISABLE;                                           /* MCPWM 时基0 T1事件更新 禁止*/

    MCPWM_InitStructure.MCPWM_Base1T0_UpdateEN = ENABLE;                                            /* MCPWM 时基1 T0事件更新使能 */
    MCPWM_InitStructure.MCPWM_Base1T1_UpdateEN = DISABLE;                                           /* MCPWM 时基1 T1事件更新 禁止*/

    MCPWM_InitStructure.CNT0_T1_Update_INT_EN = DISABLE;


        // FAIL0 选择比较器作为信号源，检测到过流后可硬件关断 PWM
    MCPWM_InitStructure.FAIL0_INT_EN = DISABLE;                                                     /* FAIL1事件 中断使能或关闭 */
    MCPWM_InitStructure.FAIL0_INPUT_EN = ENABLE;                                                    /* FAIL0通道急停功能打开或关闭 */
    MCPWM_InitStructure.FAIL0_Signal_Sel = FAIL_SEL_CMP;                                            /* FAIL0事件信号选择，比较器或IO口 */
    MCPWM_InitStructure.FAIL0_Polarity = HIGH_LEVEL_VALID;

    MCPWM_InitStructure.FAIL1_INT_EN = DISABLE;                                                     /* FAIL1事件 中断使能或关闭 */
    MCPWM_InitStructure.FAIL1_INPUT_EN = DISABLE;                                                   /* FAIL1通道急停功能打开或关闭 */
    MCPWM_InitStructure.FAIL1_Signal_Sel = FAIL_SEL_CMP;                                            /* FAIL1事件信号选择，比较器或IO口 */
    MCPWM_InitStructure.FAIL1_Polarity = HIGH_LEVEL_VALID;                                          /* FAIL1事件极性选择，高有效或低有效 */

#ifdef LKSMCU_PREDDRIVE
    /* 使用054D, 057D内置预驱芯片需要打开PWM交换功能 */
    MCPWM_PRT = 0x0000DEAD;                                                                         /* enter password to unlock write protection */
    MCPWM_SWAP = 0x67;

#endif

        // 将上述 PWM、触发、保护等参数写入 MCPWM0 模块
    MCPWM_Init(MCPWM0, &MCPWM_InitStructure);                                                       /* MCPWM0 模块初始化 */
    //mIPD_CtrProc.hDriverPolarity = MCPWM_IO01;                                                      /* 读出驱动极性 */
}


void InitBroadGPIO(void)
{
    // GPIO 分为三类：MCPWM 功率输出、模拟采样引脚、调试引脚

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);

    /* MCPWM GPIO INIT */
        // P0.10~P0.15 后续会被复用成 MCPWM 三相 6 路 PWM 输出
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIO0, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIO0, GPIO_PinSource_10, AF3_MCPWM);
    GPIO_PinAFConfig(GPIO0, GPIO_PinSource_11, AF3_MCPWM);
    GPIO_PinAFConfig(GPIO0, GPIO_PinSource_12, AF3_MCPWM);
    GPIO_PinAFConfig(GPIO0, GPIO_PinSource_13, AF3_MCPWM);
    GPIO_PinAFConfig(GPIO0, GPIO_PinSource_14, AF3_MCPWM);
    GPIO_PinAFConfig(GPIO0, GPIO_PinSource_15, AF3_MCPWM);


        // 模拟采样引脚切到 ANA 模式，避免数字输入缓冲带来干扰
    //VBUS_AD
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_ANA;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIO0, &GPIO_InitStruct);

    //OPA1_IN
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_ANA;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIO1, &GPIO_InitStruct);

    //OPA1_IP
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_ANA;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIO1, &GPIO_InitStruct);

    //OPA0_IP
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_ANA;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIO1, &GPIO_InitStruct);

    //OPA0_IN
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_ANA;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIO1, &GPIO_InitStruct);

//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
//    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_Init(GPIO0, &GPIO_InitStruct);

//    GPIO_SetBits(GPIO0, GPIO_Pin_3);
    //HA1
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_ANA;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIO0, &GPIO_InitStruct);
    //OCP
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_ANA;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIO0, &GPIO_InitStruct);
		
    
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;		
            // P1.4 用作调试 GPIO，可用示波器观察 ADC 中断耗时
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 ;
		GPIO_InitStruct.GPIO_PuPd =  GPIO_PuPd_NOPULL;
    GPIO_Init(GPIO1, &GPIO_InitStruct);

}

void InitDacMotor0(void)
{
    // DAC 在这里不是对外输出模拟量，而是作为 CMP0 的参考阈值
    DAC_InitTypeDef DAC_InitStructure;

    DAC_StructInit(&DAC_InitStructure);
        // 选 3.0V DAC 量程
    DAC_InitStructure.DAC_GAIN   = DAC_RANGE_3V0 ;
    DAC_Init(&DAC_InitStructure);        /* DAC初始化 */

        // 按过流门限和分流电阻值换算比较阈值
    DAC_OutputVoltage(OCP_CUR_THH * RSHUNT * BIT12);  /*  输出 DAC */



}

void InitPgaMotor0(void)
{

    // 片内 OPA/PGA 用于放大分流电阻上的小电压信号
    SYS_AnalogModuleClockCmd(SYS_AnalogModule_OPA, ENABLE);                                         /* opa enable.. */
        // 先清除 PGA 增益配置位
    SYS_AFE_REG0 &= ~0x0f;
	  SYS_WR_PROTECT = 0x7a83;
        // 根据手册中 RES_OPA 配置内置反馈电阻比，这里设为 20 倍增益
    SYS_AFE_REG0 |= PGA_GAIN_20;                                                                      /* OPA增益设置 */
     SYS_WR_PROTECT = 0;
}

void InitCmpMotor0(void)
{

    // 比较器用于硬件级过流/短路保护，本工程只启用 CMP0
    /* for 037E.use CMP0_IP3,HAL */
    CMP_InitTypeDef CMP_InitStruct;
    CMP_StructInit(&CMP_InitStruct);
    Clock_Init();                                                                                   /* 系统时钟初始化 */
        // 打开比较器滤波时钟，兼顾抗干扰和响应速度
    CMP_InitStruct.CLK10_EN =1;                                                                     /* 使能比较器滤波功能 */
    CMP_InitStruct.FIL_CLK10_DIV16 = 0;                                                            /* 比较器 1/0 滤波 */
    CMP_InitStruct.FIL_CLK10_DIV2 = 0;                                                              /* 比较器 1/0 滤波时钟分频 */

    SYS_AnalogModuleClockCmd(SYS_AnalogModule_CMP0, ENABLE); // CMP0模块使能
    SYS_AnalogModuleClockCmd(SYS_AnalogModule_CMP1, DISABLE); // CMP1模块使能
    /* 比较器1 */
        // CMP1 未参与当前保护链路，因此关闭
    CMP_InitStruct.CMP1_IE = DISABLE;                                                               /* 比较器 1 中断使能 */
    CMP_InitStruct.CMP1_IN_EN = DISABLE;                                                            /* 比较器 1 信号输入使能 */
    CMP_InitStruct.CMP1_POL = 0;                                                                    /* 比较器 1 极性选择 */
    CMP_InitStruct.CMP1_SELN = CMP1_SELN_DAC;                                                       /* SELN_DAC; */
    CMP_InitStruct.CMP1_SELP = CMP1_SELP_CMP1_IP1;

    /* 比较器0 */
        // CMP0 为实际保护通道：负端接 DAC，正端接 CMP0_IP3
    // 电平超阈值时可触发 MCPWM FAIL 快速关断
    CMP_InitStruct.CMP0_IE = ENABLE;                                                               /* 比较器 0 中断使能 */
    CMP_InitStruct.CMP0_IN_EN = ENABLE;                                                            /* 比较器 0 信号输入使能 */
    CMP_InitStruct.CMP0_POL = 0;                                                                   /* 比较器 1 极性选择 */
    CMP_InitStruct.CMP0_SELN = CMP0_SELN_DAC;                                                      /* SELN_DAC; */
    CMP_InitStruct.CMP0_SELP = CMP0_SELP_CMP0_IP3;

    CMP_Init(&CMP_InitStruct);

}

void UART_HardWare_Init(void)
{

        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_StructInit(&GPIO_InitStruct);

		/*uart gpio init*/
        /* P1.9 UART1 RXD */
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 ;
        GPIO_PinAFConfig(GPIO0, GPIO_PinSource_4, AF4_UART);
        GPIO_Init(GPIO0, &GPIO_InitStruct);

        /* P1.8 UART1 TXD */
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_5 ;
        GPIO_PinAFConfig(GPIO0, GPIO_PinSource_5, AF4_UART);
        GPIO_Init(GPIO0, &GPIO_InitStruct);

        UART_InitTypeDef UART_InitStruct;
		UART_StructInit(&UART_InitStruct);

		UART_InitStruct.BaudRate = 9600;//115200;                            /* 设置波特率9600 */
		UART_InitStruct.WordLength = UART_WORDLENGTH_8b;                     /* 发送数据长度8位 */
		UART_InitStruct.StopBits = UART_STOPBITS_1b;
		UART_InitStruct.FirstSend = UART_FIRSTSEND_LSB;                      /* 先发送LSB */
		UART_InitStruct.ParityMode = UART_Parity_NO;                         /* 无奇偶校验 */
		UART_InitStruct.IRQEna = UART_IRQEna_RcvOver | UART_IRQEna_SendOver; /* 接受中断使能 */
		UART_Init(UART0, &UART_InitStruct);

        NVIC_SetPriority(UART_IRQn, 2);
        NVIC_EnableIRQ (UART_IRQn);

}

void Hardware_init(void)
{

    // 硬件总初始化入口：完成 ADC、PWM、GPIO、CMP、DAC、PGA、看门狗和中断基础配置
        // 先全局关中断，防止外设尚未配好时被中断打断
    __disable_irq();                                                                               /* 关闭中断 中断总开关 */
        // 解除 SYS/AFE 相关寄存器写保护
    SYS_WR_PROTECT = 0x7a83;
        // 使能 Flash 预取，适配 48MHz 主频下的取指速率
    FLASH_CFG |= 0x00080000;                                                                       /* enable prefetch */
	
    InitAdcMotor0();                                                                               /* 电机控制层ADC初始化 */
    InitEpwmMotor0();                                                                              /* 电机控制层PWM初始化 */
    InitBroadGPIO();                                                                               /* 电机控制层GPIO初始化 */
    InitDacMotor0();                                                                               /* DAC初始化 */
    InitPgaMotor0();                                                                               /* PGA初始化 */
    InitCmpMotor0();                                                                               /* CMP初始化 */
    InitWatchDog();                                                                                /* 看门狗初配置始化 */
	UART_HardWare_Init();
	
	TempSensor_Init();
	
#if (UART_TRACE_DEBUG_ENABLE==1)
    uart_trace_debug_init();
#endif

    delay(100);

        // ADC 是快速控制主中断，CMP 对应硬件保护链路
    NVIC_SetPriority(ADC_IRQn, 1);                                                                 /* 设置ADC中断优先级 */
    NVIC_SetPriority(CMP_IRQn, 0);	                                                               /* 设置CMP中断优先级 */

        // CMP 保护由 MCPWM FAIL 硬件链路处理，这里不单独开 NVIC 中断
    NVIC_DisableIRQ(CMP_IRQn);                                                                      /* 使能CMP中断 */
        // 打开 ADC 转换结束中断，FOC 快速控制由 ADC_IRQHandler 入口执行
    NVIC_EnableIRQ(ADC_IRQn);


}

