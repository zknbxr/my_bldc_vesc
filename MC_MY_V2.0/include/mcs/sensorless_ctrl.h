#ifndef SENSORLESS_CTRL_H
#define SENSORLESS_CTRL_H

#include "main.h"

/* 定点数参数 */
#define MCS_FLUX_OBS_FIXED_Q              (15L)
#define MCS_FLUX_OBS_FIXED_SCALE          (1L << MCS_FLUX_OBS_FIXED_Q)
#define MCS_FLUX_OBS_UPDATE_DIV            2U

/* 电机真实参数：R/L直接按当前alpha-beta单位参与观测器计算。 */
#define MCS_FLUX_OBS_DEFAULT_FLUX_UW       (4450L)  /* uWb 6675*/
#define MCS_FLUX_OBS_DEFAULT_L_UH          (514L)   /* uH 514L  343L  */
#define MCS_FLUX_OBS_DEFAULT_RS_MOHM       (381L)   /* mΩ 381L  254L  */
#define MCS_FLUX_OBS_KOBS_TS_Q13           (256L)    /* Q13 64 */
#define MCS_FLUX_OBS_ITERATIONS            1U
#define MCS_FLUX_OBS_ERR_BOOST_Q13         ((s32)((MCS_FLUX_OBS_FIXED_SCALE * 2L) / 10L))
#define MCS_FLUX_OBS_ERR_BOOST_GAIN        1L

// 磁链输出最小幅值
#define MCS_FLUX_OBS_MIN_VALID_AMP_UW 1000UL
#define MCS_FLUX_OBS_INVALID_LIMIT      20U
// 多久调用一次，us单位
#define FOC_TICK_US                         ((u16)(1000000UL / PWM_FREQ))


#define MCS_FLUX_PLL_KP_SHIFT        (6L)
#define MCS_FLUX_PLL_KI_SHIFT        (4L)
#define MCS_FLUX_PLL_ERR_LIMIT_Q13   (8192L)
#define MCS_FLUX_PLL_SPEED_LIMIT_Q16 ((s32)(600L << 16))


/* VESC风格无感启动：先用开环角override带转，再切到观测角。 */
#define SENSORLESS_START_SEED_ANGLE         ((u16)0)
#define SENSORLESS_IPD_ENABLE               (1U)
#define SENSORLESS_IPD_DIR_COUNT            (6U)
#define SENSORLESS_IPD_PULSE_CMD            (25)
#define SENSORLESS_IPD_PULSE_TICKS          (4U)
#define SENSORLESS_IPD_SETTLE_TICKS         (4U)
#define SENSORLESS_START_D_REF              (20)
#define SENSORLESS_START_D_TIME_TICKS       ((u16)(PWM_FREQ / 2U))
#define SENSORLESS_OPEN_LOOP_TICKS          ((u16)(PWM_FREQ * 2U))
#define SENSORLESS_OPEN_LOOP_START_STEP     (4U)
#define SENSORLESS_OPEN_LOOP_TARGET_STEP    (32U)
#define SENSORLESS_OPEN_LOOP_RAMP_DIV       (128U)
#define SENSORLESS_RUN_Q_REF                (40)
#define SENSORLESS_START_VQ_CMD            (20)
#define SENSORLESS_RUN_VQ_CMD              (80)
#define SENSORLESS_SOFT_CURRENT_LIMIT_MA   (4000L)
#define FOC_START_BLOCK_NONE                (0U)
#define FOC_START_BLOCK_PHASE_CURRENT       (5U)
#define FOC_START_BLOCK_BUS_VOLTAGE         (6U)
#define FOC_START_BLOCK_MANUAL_RESET        (7U)

#define SENSORLESS_START_D_REF_MA           (30)
#define SENSORLESS_START_IQ_REF_MA          (20)
#define SENSORLESS_RUN_IQ_REF_MA            (80)

typedef enum {
    FOC_START_ALIGN = 0,
    FOC_START_RAMP = 1,
    FOC_START_CLOSED_LOOP = 3
} FOC_START_STATE;

/* 电流结构体 */
typedef struct {
    s16 alphaAdc;
    s16 betaAdc;
    s32 alphaMa;
    s32 betaMa;
} MCS_FLUX_OBS_CURRENT;

/* 电压结构体 */
typedef struct {
    s16 alphaCmd;
    s16 betaCmd;
    s32 alphaMv;
    s32 betaMv;
} MCS_FLUX_OBS_VOLTAGE;

/* 参数 */
typedef struct {
    s32 rsMohm;       /* mΩ */
    s32 lUh;          /* uH */
    s32 fluxRefUw;    /* uWb */
    s32 kobsTsQ13;    /* Q13 */
    s32 tsUs;         /* us */
} MCS_FLUX_OBS_PARAM;

/* 状态 */
typedef struct {
    u16 angle;
    u16 lastAngle;
    s16 angleDelta;
    s32 speedQ16;       /* Q16 */

    s32 yAlphaMv;
    s32 yBetaMv;

    s32 xhatAlphaUw;
    s32 xhatBetaUw;

    s32 etaAlphaUw;
    s32 etaBetaUw;

    u32 phiAmpUw;
    s64 etaNorm2;
    s32 fluxErrQ13;
    s32 lastAlphaMa;
    s32 lastBetaMa;
    u8 invalidCount;

    u8 ready;


    u16 rawAngle;        /* atan2 直接算出来的原始角度 */
    u16 pllAngle;        /* PLL 平滑后的角度 */
    u32 pllAngleQ16;     /* PLL 角度，u16角度的Q16累加形式 */
    s32 pllIntegQ16;     /* PLL PI积分项 */
    s32 pllSpeedQ16;     /* PLL输出速度，单位是 angle-count/update，Q16 */
    s32 pllErrQ13;       /* PLL相位误差，Q13 */
    s32 pllPTermQ16;     /* PLL比例项 */
    u8 pllReady;
} MCS_FLUX_OBS_STATE;

/* 全部观测器 */
typedef struct {
    MCS_FLUX_OBS_CURRENT current;
    MCS_FLUX_OBS_VOLTAGE voltage;
    MCS_FLUX_OBS_PARAM param;
    MCS_FLUX_OBS_STATE state;
} MCS_FLUX_OBS;
extern volatile MCS_FLUX_OBS gFluxObs;
extern u8 MCS_FLUX_OBS_ENABLE;
extern volatile u8 gFocStartStateWatch;
extern volatile u8 gFocStartBlockReason;
extern volatile u16 gFocStartResetCount;
extern volatile u16 gFocStartClosedEnterCount;
extern volatile u16 gFocStartClosedRunCount;
extern volatile u32 gFocStartTaskCount;
extern volatile u16 gFocStartAlignRunCount;
extern volatile u16 gFocStartAlignDwellCount;
extern volatile u16 gFocStartRampRunCount;
extern volatile s16 gOverCurrentPhaseAAdc;
extern volatile s16 gOverCurrentPhaseBAdc;
extern volatile u16 gOverCurrentPeakAdc;
extern volatile u16 gOverCurrentPwmA;
extern volatile u16 gOverCurrentPwmB;
extern volatile u16 gOverCurrentPwmC;
extern volatile u8 gOverCurrentStartState;
extern volatile u16 gSensorlessCompAngle;
extern volatile u16 gSensorlessOpenLoopAngle;
extern volatile u16 gSensorlessOpenLoopStep;
extern volatile u16 gSensorlessSeedAngle;
extern volatile u16 gSensorlessIpdAngle;
extern volatile u8 gSensorlessIpdDir;
extern volatile u8 gSensorlessIpdDone;
extern volatile s32 gSensorlessIpdRespMa;
extern volatile s32 gSensorlessIpdBestRespMa;

/* 函数接口 */
void FluxObs_Init(void);
void FluxObs_Reset(void);
void FluxObs_SeedAngle(u16 angle);
void FluxObs_Update(void);
void FOC_SensorlessStart_Reset(void);
void FOC_SensorlessStart_Task(void);
void FluxObs_SetVoltageAlphaBeta(s16 alpha, s16 beta);
#endif






