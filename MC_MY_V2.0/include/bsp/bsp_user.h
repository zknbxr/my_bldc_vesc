#ifndef BSP_USER_H
#define BSP_USER_H

/********************************************************************************************************************************************************/
/*****************************************MCU时钟及PWM相关配置***********************************************************************************************/
/********************************************************************************************************************************************************/ 
/*----------------------------------------PWM 频率及死区定义(P8)--------------------------------------------------------------------------------------------*/

#define MCU_SERISE                        (03)                                                     /*        MCU系列  目前仅支持 03 07 08 */
#define MCU_MAIN_MCLK                     ((u32)48000000uL)                                        /* P8.00 ,PWM模块运行主频 */
#define PWM_MCLK	                        (MCU_MAIN_MCLK)                                               /*        PWM模块运行主频 */
#define PWM_PRSC                          ((u8)0)                                                  /* P8.02 ,PWM模块运行预分频器 */

#define PWM_FREQ                          ((u16)14000)                                             /* P8.03 ,PWM斩波频率, */
#define DEADTIME_NS                       ((u16)1000)                                              /* P8.04 ,死区时间 单位：ns*/

#define PWM_PERIOD                        ((u16) (PWM_MCLK / (u32)(2 * PWM_FREQ *(PWM_PRSC+1))))   /*        PWM 周期计数器值 */  // 1714
#define DEADTIME                          (u16)(((unsigned long long)PWM_MCLK * (unsigned long long)DEADTIME_NS)/1000000000uL)
	
/*----------------------------------------1ms tick counter 时基--------------------------------------------------------------------------------------------------*/

#define PWM_TIME_1MS_COUNTER              (u16)(PWM_FREQ/1000)
#define PWM_TIME_10MS_COUNTER     PWM_TIME_1MS_COUNTER*10
#define PWM_TIME_100MS_COUNTER    PWM_TIME_10MS_COUNTER*10
#define PWM_TIME_1000MS_COUNTER   PWM_TIME_100MS_COUNTER*10
/*----------------------------------------驱动极性选择(P3)--------------------------------------------------------------------------------------------------*/

#define LKSMCU_PREDDRIVE                                                                           /* P8.05 ,注释是外置预区模式，打开是内置预区模式*/

/*----------------------------------------ADC基准电压及运放放大倍数(P3)------------------------------------------------------------------------------------------------------*/

#define ADC_SUPPLY_VOLTAGE                (3.6)                                                    /* P8.07 ,单位: V  ADC基准电压，3.6或者2.4,大部分应用选择3.6 */
#define AMPLIFICATION_GAIN                (18.18)                                                  /* P8.08 ,运放放大倍数 */

/*----------------------------------------采样电阻及母线电压、反电动势电压分压比配置-----------------------------------------------------------------------------------------*/
#define LOW_VOL_SYS            
#ifdef LOW_VOL_SYS
#define RSHUNT                            (0.01)                                                  /* P8.09 ,单位: Ω  采样电阻阻值 */
#define VOLTAGE_SHUNT_RATIO               (1.0/(20.0+1.0))                                         /* P8.0A ,母线电压分压比 */
#else

#define RSHUNT                            (0.1)                                                    /* P8.09 ,单位: Ω  采样电阻阻值*/
#define VOLTAGE_SHUNT_RATIO               (1.0/(300.0+1.0))                                        /* P8.0A ,母线电压分压比  */

#endif
#define BEMF_SHUNT_RATIO                  (1.5/(10.0+1.5))                                         /* P8.0B ,反电势电压分压比*/
#define OCP_CUR_THH                       (25.0)                                                    /* P8.0C ,OCP短路保护电流，单位：A */
/*----------------------------------------单电阻采样移相参数-----------------------------------------------------------------------------------------*/
#define SAMP_TIME_RISE_TIME               (1200)                                                   /* P8.0D ,单电阻移相时间参数A：单位 nS ,*/ 
#define ADC_RSVD_TIME                     (600)                                                    /* P8.0E ,单电阻移相时间参数B：单位 nS ,*/ 


/*----------------------------------------SVPWM 驱动模式：7段式、5段式选择-----------------------------------------------------------------------------------------*/
#define VECTOR_MODULATION_MODE            (SVPWM_7_SEGMENT)                                        /* P8.0F ,Csvpwm */


/*----------------------------------------电流采样参数------------------------------------------------------------------------------------------------------*/                                 
/* 可以正常采样的最大输出PWM脉冲宽度 */
#define MAX_SAMP_TIME                     (u16)(((unsigned long long)PWM_MCLK * (unsigned long long)(SAMP_TIME_RISE_TIME+DEADTIME_NS))/1000000000uL)
#define MIN_SAMP_TIME                     (u16)(((unsigned long long)PWM_MCLK * (unsigned long long)(SAMP_TIME_RISE_TIME+DEADTIME_NS+ADC_RSVD_TIME))/1000000000uL) 
#define SINGLE_SAMP_TIME                  (u16)(((unsigned long long)PWM_MCLK * (unsigned long long)SAMP_TIME_RISE_TIME)/1000000000uL)  
	                                                                                                  /* 单电阻采样可以执行的脉冲宽度 */
#define FIR_SAMP_TIME                     (15500)                                                   /* 需要避让采样点时刻 16384为全脉宽 */

/*----------------------------------------ADC通道号定义-----------------------------------------------------------------------------------------------------*/    

#define ADC_CHANNEL_OPA0                  ADC_CHANNEL_0
#define ADC_CHANNEL_OPA1                  ADC_CHANNEL_8


#define ADC_CURRETN_A_CHANNEL             (ADC_CHANNEL_OPA0)
#define ADC_CURRETN_B_CHANNEL             (ADC_CHANNEL_OPA1)
#define ADC0_BUS_CUR_CHANNEL              (ADC0_CHANNEL_OPA1)
#define GET_UDC_SAMPLE_RESULT()           ((INT16)ADC_DAT2)                                         /* set udc result chn.. */

#define GET_HALLA_SAMPLE_RESULT()         ((INT16)ADC_DAT3)                                         /* set udc result chn.. */
#define GET_HALLB_SAMPLE_RESULT()         ((INT16)ADC_DAT4)                                         /* set udc result chn.. */

#define GET_TEMP_SAMPLE_RESULT()          ((INT16)ADC_DAT5)  

#define GET_NTCVOLAD_SAMPLE_RESULT()      ((INT16)ADC_DAT6) 

#define GET_CURRENT_U_SAMPLE_RESULT()    ((INT16)ADC_DAT0)                                          /* set phase u result chn.. */
#define GET_CURRENT_V_SAMPLE_RESULT()    ((INT16)ADC_DAT1)                                          /* set phase v result chn.. */


/*----------------------------------------第一次采样顺序MSK-----------------------------------------------------------------------------------------------------*/ 
//采样顺序其实就是一次采样后把采样值放进哪个寄存器，ADC_CHN0 的低4位是第一次采样的通道号，依次类推
#define ADC0_CUR_A_1ST_MSK                (u16)(ADC0_CURRETN_A_CHANNEL)                             /* ADC0先采样A相电流 */
#define ADC0_CUR_B_1ST_MSK                (u16)(ADC0_CURRETN_B_CHANNEL)                             /* ADC0先采样B相电流 */
#define ADC0_CUR_C_1ST_MSK                (u16)(ADC0_CURRETN_C_CHANNEL)                             /* ADC0先采样B相电流 */

#define ADC1_CUR_A_1ST_MSK                (u16)(ADC1_CURRETN_A_CHANNEL<<8)                          /* ADC1先采样B相电流 */
#define ADC1_CUR_C_1ST_MSK                (u16)(ADC1_CURRETN_C_CHANNEL<<8)                          /* ADC1先采样C相电流 */
#define ADC1_CUR_B_1ST_MSK                (u16)(ADC1_CURRETN_B_CHANNEL<<8)                          /* ADC1先采样B相电流 */

/*----------------------------------------第二次采样顺序MSK-----------------------------------------------------------------------------------------------------*/ 


#define ADC0_FIR_SAMPLE_CHN               (u16)(ADC0_CURRETN_A_CHANNEL)
#define ADC0_SEC_SAMPLE_CHN               (u16)(ADC0_CURRETN_B_CHANNEL<<4)


/*----------------------------------------第三次采样顺序MSK-----------------------------------------------------------------------------------------------------*/

#define ADC_DC_VOL_CHN                    (u16)(ADC_CHANNEL_5)                                      /* ADC0采样电压 */

#define ADC0_3TH_SAMPLE_CHN               (u16)(ADC_DC_VOL_CHN<<8)


#define ADC0_4TH_SAMPLE_CHN               (u16)(ADC_CHANNEL_3)
#define ADC0_4TH_SAMPLE2_CHN              (u16)(ADC_CHANNEL_6)

#define ADC0_TEMP_SAMPLE_CHN              (u16)(ADC_CHANNEL_11)

#define ADC1_3th_MSK                      (u16)(ADC0_BUS_CUR_CHANNEL<<8)                            /* ADC1第三次采样母线电流 */

/*----------------------------------------第四次采样顺序MSK-----------------------------------------------------------------------------------------------------*/

#define ADC0_4th_MSK                      (u16)(0x05)                                               /* ADC0采样电压 */
#define ADC1_4th_MSK                      (u16)(0x09<<8)                                             

#define ADC0_4th_CH8_MSK                  (u16)(ADC_CHANNEL_7<<12)
#define ADC1_4th_CH8_MSK                  (u16)(ADC_CHANNEL_7<<12)                                  /* ADC1第四次采样 */

#define ADC_STATE_RESET()                 {ADC_CFG |= BIT11;}                                       /* ADC0 状态机复位,用以极限情况下确定ADC工作状态 */
#define ADC_SOFTWARE_TRIG_ONLY()          {ADC_CFG = 0;}                                            /* ADC设置为仅软件触发 */

#define ADC_DOUBLE_SAMP_RATE()            {ADC0_CFG = ADC_HARDWARE_T0_TRG | ADC_HARDWARE_T1_TRG | BIT11;}
#define ADC_NORMAL_SAMP_RATE()            {ADC0_CFG = ADC_HARDWARE_T0_TRG | BIT11; }

#define BEMF_CH_A                         ADC_CHANNEL_15                                            /* ADC_15 */
#define BEMF_CH_B                         ADC_CHANNEL_16                                            /* ADC_16 */
#define BEMF_CH_C                         ADC_CHANNEL_17                                            /* ADC_17 */

/*----------------------------------------ADC操作相关定义-------------------------------------------------------------------------------------------------------*/

#define ADC_GET_OFFSET_AVG_TIMES          (512)

/*----------------------------------------PGA操作相关定义-------------------------------------------------------------------------------------------------------*/

#define PGA_GAIN_20                       (0)                                                       /* 反馈电阻200:10 */
#define PGA_GAIN_9P5                      (1)                                                       /* 反馈电阻190:20 */
#define PGA_GAIN_6                        (2)                                                       /* 反馈电阻180:30 */
#define PGA_GAIN_4P25                     (3)                                                       /* 反馈电阻170:40 */
                                                                                  
#define OPA0_GIAN                         (PGA_GAIN_20)
#define OPA1_GIAN                         (PGA_GAIN_20 << 2)
#define OPA2_GIAN                         (PGA_GAIN_20 << 4)
#define OPA3_GIAN                         (PGA_GAIN_20 << 6)

#define  P_HIGH__N_HIGH                   (1)
#define  P_HIGH__N_LOW                    (2)

#define  PRE_DRIVER_POLARITY              P_HIGH__N_HIGH                                            /* 预驱预动极性设置 上管高电平有效，下管高电平有效 */


void SystemInit(void);
void Hardware_init(void);
void delay(u16 cnt);
void InitAdcMotor0(void);
#endif
