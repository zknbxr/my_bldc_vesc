#include "main.h"

/*
 * 电机控制系统初始化和ADC零偏校准。
 * 校准期间关闭中断和功率输出，初始化完成后再启动控制中断。
 */


/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define PHASE_OFFSET_MAX        4000
#define PHASE_OFFSET_MIN        -4000
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
/* Offect */
s16 hPhaseAOffset;
s16 hPhaseBOffset;

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/


/* 配置电机控制使用的ADC转换序列。 */
void ConfigAdcModeMotor0(void)
{
    /*
     * ADC_DAT0 is a discarded OPA1 sample. The SAR capacitor retains the last
     * Hall-channel voltage across PWM periods, so sampling OPA1 twice prevents
     * that residue from entering the current loop. Valid V/U samples are kept
     * consecutive and all slow Hall channels remain at the end of the sequence.
     */
    ADC_CHN0 = ADC_CURRETN_B_CHANNEL | (ADC_CURRETN_B_CHANNEL << 4) |
               (ADC_CURRETN_A_CHANNEL << 8) | (ADC_DC_VOL_CHN << 12);
    ADC_CHN1 = ADC0_TEMP_SAMPLE_CHN | (ADC0_4TH_SAMPLE_CHN << 4) |
               (ADC0_4TH_SAMPLE2_CHN << 8);

}

void CurrentOffsetCalibration(void)
{
    volatile u32 t_dlay;
    volatile u16 t_cnt;
    s32 t_offset1,t_offset2;

    /* Offset calibration must run with power stage inactive and interrupts
     * disabled, otherwise ADC ISR/control code could consume unstable samples.
     */
    __disable_irq();

    for (t_dlay = 0; t_dlay < 0x2ffff; t_dlay++);

    t_offset1 = 0;
    t_offset2 = 0;
    // Sequence: OPA1(V), bus voltage, temperature, OPA0(U), Hall A, Hall B.
    ADC_SOFTWARE_TRIG_ONLY();
    ADC_STATE_RESET();
    ConfigAdcModeMotor0();

    /* 等待采样稳定，并累加当前转换结果。 */
    for (t_cnt = 0; t_cnt < ADC_GET_OFFSET_AVG_TIMES; t_cnt++)
    {
        for (t_dlay = 0; t_dlay < 0x800; t_dlay++);
        /* 清除ADC完成标志。 */
        ADC_IF |= BIT1|BIT0;
        /* 复位ADC状态机，准备下一轮采样。 */
        ADC_CFG |= BIT11;
        t_offset1 += GET_CURRENT_U_SAMPLE_RESULT();
        t_offset2 += GET_CURRENT_V_SAMPLE_RESULT();


        /* 软件触发下一轮ADC转换。 */
        ADC_SWT = 0x00005AA5;
    }

    InitAdcMotor0();
    ConfigAdcModeMotor0();

    __enable_irq();

	/* 512次采样求平均，作为两相电流零偏。 */
	hPhaseAOffset = (s16)(t_offset1 >> 9);
	hPhaseBOffset = (s16)(t_offset2 >> 9);

    m_motor.m_conf->foc_offsets_current[0] = hPhaseAOffset;
    m_motor.m_conf->foc_offsets_current[1] = hPhaseBOffset;
    m_motor.m_conf->foc_offsets_current[2] = 0;


}


void mc_sys_init(void)
{
	INT8 ax;

	/* 初始化电机对象、配置指针和控制状态。 */
	Motor_FocInit();

	/* Give analog front-end and ADC references time to settle before offset
     * calibration. Removing this delay can make phase-current zero drift wrong.
     */
	/* 等待模拟前端和ADC参考电压稳定。 */
    for(ax=0; ax<10; ax++)
    {
        delay(60000);
    }
	/* 电流采样零偏校准。 */
    CurrentOffsetCalibration();

	/* 复位无感观测器和PLL状态。 */
	foc_observer_reset(&m_motor.m_observer_state);
	/* 基础状态就绪后加载霍尔参数，或进入在线学习模式。 */
	Hall_LearnInit();

	/* Clear pending MCPWM fail events before exposing the MCS facade to APP. */
	MCPWM_EIF = BIT4|BIT5;	

__enable_irq(); /* 开启中断。 */
}

