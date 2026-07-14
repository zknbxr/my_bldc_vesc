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
s16 hBusCurrentOffset;
s32 i32iAdcRes1Avg1,i32iAdcRes1Avg2;

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

INT16 iAdcRes1,iAdcRes2;

/*
	adc采样通道配置
**/
void ConfigAdcModeMotor0(void)
{
    /* ADC regular sequence:
     * CH0/CH1 sample phase current, CH2 samples DC bus, remaining channels
     * are reserved for Hall/temperature/extra observation channels.
     */
    ADC_CHN0 = ADC_CURRETN_A_CHANNEL | (ADC_CURRETN_B_CHANNEL << 4) | (ADC_DC_VOL_CHN << 8) | (ADC0_4TH_SAMPLE_CHN << 12);
    ADC_CHN1 = ADC0_4TH_SAMPLE2_CHN |  (ADC0_TEMP_SAMPLE_CHN<<4);

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
    //纯软件触发，复位ADC状态机，配置成单次采样模式，连续采样6个通道（A相电流、B相电流、母线电压、4th采样1、4th采样2、温度），每轮采样结束后进入 ADC 中断，由 ADC IRQ 处理函数累加采样结果，最后取平均值作为零漂补偿值
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

    Motor_FocInit();
    m_motor.m_conf->foc_offsets_current[0] = hPhaseAOffset;
    m_motor.m_conf->foc_offsets_current[1] = hPhaseBOffset;
    m_motor.m_conf->foc_offsets_current[2] = 0;


    i32iAdcRes1Avg1=(INT32)hPhaseAOffset<<12;
    i32iAdcRes1Avg2=(INT32)hPhaseBOffset<<12;;
}

void judgement_offset(void)
{
    //实际判断并未生效，
    /* Legacy hook: hBusCurrentOffset is not filled in the current path.
     * Keep the function so later current-offset range checks have one place
     * to report E_FAULT_OFFSET_ERROR.
     */

}

void mc_sys_init(void)
{
	INT8 ax;


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

	/* Clear pending MCPWM fail events before exposing the MCS facade to APP. */
	MCPWM_EIF = BIT4|BIT5;	

__enable_irq();/* 开启中断 */
}

