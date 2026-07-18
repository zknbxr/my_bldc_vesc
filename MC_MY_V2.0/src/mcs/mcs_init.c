#include "main.h"

/*
 * MCS initialization and ADC zero-offset calibration.
 *
 * 该文件负责电机控制层上电初始化。当前重点是：
 * - 关闭中断后做相电流零漂采样；
 * - 恢复 ADC/MCPWM 到运行配置；
 * - 初始化 MCS_Motor facade，确保所有调试启动入口先处于停止态。
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


/*
	adc采样通道配置
**/
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

    //本轮先读上一次已经完成的结果，再启动下一次转换，下轮再来读
    for (t_cnt = 0; t_cnt < ADC_GET_OFFSET_AVG_TIMES; t_cnt++)
    {
        for (t_dlay = 0; t_dlay < 0x800; t_dlay++);
        //清上一次 ADC 完成标志
        ADC_IF |= BIT1|BIT0;
        //复位ADC状态机，准备下一轮采样
        ADC_CFG |= BIT11;
        t_offset1 += GET_CURRENT_U_SAMPLE_RESULT();
        t_offset2 += GET_CURRENT_V_SAMPLE_RESULT();


        /* Clear the ADC0 JEOC pending flag */
        //开启一次软件采样，由硬件AD采样后将结果放到data中，延时一定时间等待完成后清标志，复位，等下一次软件触发。
        ADC_SWT = 0x00005AA5;
    }

    InitAdcMotor0();
    ConfigAdcModeMotor0();

    __enable_irq();

	//512次采样平均
	hPhaseAOffset = (s16)(t_offset1 >> 9);
	hPhaseBOffset = (s16)(t_offset2 >> 9);

    m_motor.m_conf->foc_offsets_current[0] = hPhaseAOffset;
    m_motor.m_conf->foc_offsets_current[1] = hPhaseBOffset;
    m_motor.m_conf->foc_offsets_current[2] = 0;


}


void mc_sys_init(void)
{
	INT8 ax;

	/* 快速环使用前一次性建立电机配置指针，运行中不再重复检查。 */
	Motor_FocInit();

	/* Give analog front-end and ADC references time to settle before offset
     * calibration. Removing this delay can make phase-current zero drift wrong.
     */
	//延时稳定
    for(ax=0; ax<10; ax++)
    {
        delay(60000);
    }
	/*零飘校准*/
    CurrentOffsetCalibration();

	/* 清零 VESC 风格的磁链观测器状态。 */
	foc_observer_reset(&m_motor.m_observer_state);

	/* Clear pending MCPWM fail events before exposing the MCS facade to APP. */
	MCPWM_EIF = BIT4|BIT5;	

__enable_irq();/* 开启中断 */
}

