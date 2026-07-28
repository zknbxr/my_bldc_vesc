#include "main.h"

#define MCS_SQRT3_OVER_2_Q15              28378L
#define MCS_MOTOR_PWM_CMD_MIN             0L
#define MCS_MOTOR_PWM_CMD_MAX             ((s32)PWM_PERIOD)


static const s16 s_sinCosTable[256] =
{
    0x0000,0x00C9,0x0192,0x025B,0x0324,0x03ED,0x04B6,0x057F,
    0x0648,0x0711,0x07D9,0x08A2,0x096B,0x0A33,0x0AFB,0x0BC4,
    0x0C8C,0x0D54,0x0E1C,0x0EE4,0x0FAB,0x1073,0x113A,0x1201,
    0x12C8,0x138F,0x1455,0x151C,0x15E2,0x16A8,0x176E,0x1833,
    0x18F9,0x19BE,0x1A83,0x1B47,0x1C0C,0x1CD0,0x1D93,0x1E57,
    0x1F1A,0x1FDD,0x209F,0x2162,0x2224,0x22E5,0x23A7,0x2467,
    0x2528,0x25E8,0x26A8,0x2768,0x2827,0x28E5,0x29A4,0x2A62,
    0x2B1F,0x2BDC,0x2C99,0x2D55,0x2E11,0x2ECC,0x2F87,0x3042,
    0x30FC,0x31B5,0x326E,0x3327,0x33DF,0x3497,0x354E,0x3604,
    0x36BA,0x3770,0x3825,0x38D9,0x398D,0x3A40,0x3AF3,0x3BA5,
    0x3C57,0x3D08,0x3DB8,0x3E68,0x3F17,0x3FC6,0x4074,0x4121,
    0x41CE,0x427A,0x4326,0x43D1,0x447B,0x4524,0x45CD,0x4675,
    0x471D,0x47C4,0x486A,0x490F,0x49B4,0x4A58,0x4AFB,0x4B9E,
    0x4C40,0x4CE1,0x4D81,0x4E21,0x4EC0,0x4F5E,0x4FFB,0x5098,
    0x5134,0x51CF,0x5269,0x5303,0x539B,0x5433,0x54CA,0x5560,
    0x55F6,0x568A,0x571E,0x57B1,0x5843,0x58D4,0x5964,0x59F4,
    0x5A82,0x5B10,0x5B9D,0x5C29,0x5CB4,0x5D3E,0x5DC8,0x5E50,
    0x5ED7,0x5F5E,0x5FE4,0x6068,0x60EC,0x616F,0x61F1,0x6272,
    0x62F2,0x6371,0x63EF,0x646C,0x64E9,0x6564,0x65DE,0x6657,
    0x66D0,0x6747,0x67BD,0x6832,0x68A7,0x691A,0x698C,0x69FD,
    0x6A6E,0x6ADD,0x6B4B,0x6BB8,0x6C24,0x6C8F,0x6CF9,0x6D62,
    0x6DCA,0x6E31,0x6E97,0x6EFB,0x6F5F,0x6FC2,0x7023,0x7083,
    0x70E3,0x7141,0x719E,0x71FA,0x7255,0x72AF,0x7308,0x735F,
    0x73B6,0x740B,0x7460,0x74B3,0x7505,0x7556,0x75A6,0x75F4,
    0x7642,0x768E,0x76D9,0x7723,0x776C,0x77B4,0x77FB,0x7840,
    0x7885,0x78C8,0x790A,0x794A,0x798A,0x79C9,0x7A06,0x7A42,
    0x7A7D,0x7AB7,0x7AEF,0x7B27,0x7B5D,0x7B92,0x7BC6,0x7BF9,
    0x7C2A,0x7C5A,0x7C89,0x7CB7,0x7CE4,0x7D0F,0x7D3A,0x7D63,
    0x7D8A,0x7DB1,0x7DD6,0x7DFB,0x7E1E,0x7E3F,0x7E60,0x7E7F,
    0x7E9D,0x7EBA,0x7ED6,0x7EF0,0x7F0A,0x7F22,0x7F38,0x7F4E,
    0x7F62,0x7F75,0x7F87,0x7F98,0x7FA7,0x7FB5,0x7FC2,0x7FCE,
    0x7FD9,0x7FE2,0x7FEA,0x7FF1,0x7FF6,0x7FFA,0x7FFE,0x7FFF
};


static s16 Motor_PwmOffsetToModQ15(s32 offset)
{
    int64_t modulation;

    /* Q15 modulation 1.0 represents 2/3 of the DC bus in alpha-beta. */
    modulation = ((int64_t)offset * 3LL * 32768LL) /
                 ((int64_t)PWM_PERIOD * 2LL);
    return McsMath_SatS16((s32)modulation);
}

void Motor_WritePwmCompare(u16 phaseA, u16 phaseB, u16 phaseC)
{
    MCPWM_TH20 = -phaseC;
    MCPWM_TH21 = phaseC;
    
    MCPWM_TH10 = -phaseB;
    MCPWM_TH11 = phaseB;
    
    MCPWM_TH00 = -phaseA;
    MCPWM_TH01 = phaseA;
}

MCS_TRIG_Q15 Motor_GetSinCosQ15(u16 angle)
{
    u16 index;
    u8 pos;
    MCS_TRIG_Q15 trig;

    index = (u16)(angle >> 6);
    pos = (u8)(index & 0x00FFU);
    trig.sin = 0;
    trig.cos = 0;

    switch((u8)(index >> 8))
    {
        case 0U:
            trig.sin = s_sinCosTable[pos];
            trig.cos = s_sinCosTable[(u8)(0xFFU - pos)];
            break;

        case 1U:
            trig.sin = s_sinCosTable[(u8)(0xFFU - pos)];
            trig.cos = (s16)(-s_sinCosTable[pos]);
            break;

        case 2U:
            trig.sin = (s16)(-s_sinCosTable[pos]);
            trig.cos = (s16)(-s_sinCosTable[(u8)(0xFFU - pos)]);
            break;

        default:
            trig.sin = (s16)(-s_sinCosTable[(u8)(0xFFU - pos)]);
            trig.cos = s_sinCosTable[pos];
            break;
    }

    return trig;
}

static void Motor_WriteAlphaBetaVector(s16 alpha, s16 beta)
{
    const s32 center = ((s32)PWM_PERIOD) / 2;
    s32 alphaCmd;
    s32 betaCmd;
    s32 phaseAOffset;
    s32 phaseBOffset;
    s32 phaseCOffset;
    s32 maxOffset;

    alphaCmd = (s32)alpha;
    betaCmd = (s32)beta;

    phaseAOffset = alphaCmd;
    phaseBOffset = -(alphaCmd / 2) + ((betaCmd * MCS_SQRT3_OVER_2_Q15) >> 15);
    phaseCOffset = -(alphaCmd / 2) - ((betaCmd * MCS_SQRT3_OVER_2_Q15) >> 15);

    /*
     * MCPWM_THx0/THx1 使用 -phase/+phase 生成中心对齐 PWM。
     * phase=0 对应 0% 占空比，phase=PWM_PERIOD 对应 100% 占空比，
     * phase=PWM_PERIOD/2 对应三相中性 50% 占空比。
     * 因此三相相电压偏移量必须限制在 [-center, +center] 内。
     */
    maxOffset = McsMath_MaxAbs3S32(phaseAOffset, phaseBOffset, phaseCOffset);
    if(maxOffset > center)
    {
        alphaCmd = (alphaCmd * center) / maxOffset;
        betaCmd = (betaCmd * center) / maxOffset;

        phaseAOffset = alphaCmd;
        phaseBOffset = -(alphaCmd / 2) + ((betaCmd * MCS_SQRT3_OVER_2_Q15) >> 15);
        phaseCOffset = -(alphaCmd / 2) - ((betaCmd * MCS_SQRT3_OVER_2_Q15) >> 15);
    }
    /* 保存下一次 ADC 采样期间实际生效的调制度。 */
    m_motor.m_motor_state.mod_alpha_raw = Motor_PwmOffsetToModQ15(alphaCmd);
    m_motor.m_motor_state.mod_beta_raw = Motor_PwmOffsetToModQ15(betaCmd);

    Motor_WritePwmCompare(
        (u16)McsMath_LimitS32(center + phaseAOffset,
                              MCS_MOTOR_PWM_CMD_MIN,
                              MCS_MOTOR_PWM_CMD_MAX),
        (u16)McsMath_LimitS32(center + phaseBOffset,
                              MCS_MOTOR_PWM_CMD_MIN,
                              MCS_MOTOR_PWM_CMD_MAX),
        (u16)McsMath_LimitS32(center + phaseCOffset,
                              MCS_MOTOR_PWM_CMD_MIN,
                              MCS_MOTOR_PWM_CMD_MAX));
}


void Motor_WriteDqVector(u16 angle, s16 dRef, s16 qRef)
{
    MCS_TRIG_Q15 trig;
    s32 alpha;
    s32 beta;

    trig = Motor_GetSinCosQ15(angle);
    alpha = (((s32)dRef * trig.cos) - ((s32)qRef * trig.sin)) >> 15;
    beta = (((s32)dRef * trig.sin) + ((s32)qRef * trig.cos)) >> 15;

    Motor_WriteAlphaBetaVector((s16)alpha, (s16)beta);
}


void Motor_WriteNeutralPwm(void)
{
    Motor_WritePwmCompare(PWM_PERIOD / 2U, PWM_PERIOD / 2U, PWM_PERIOD / 2U);
}


void MCS_Motor_Stop(void)
{
    PwmAOutputs(DISABLE);
}

