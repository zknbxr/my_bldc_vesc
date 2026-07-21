#ifndef MCS_HALL_H
#define MCS_HALL_H

#include "mcs_motor_type.h"

/*
 * 两路线性霍尔在线学习状态。
 * 旋转阶段判断幅值、正交关系和方向是否逐圈收敛，随后固定角吸附校零。
 */
typedef enum {
    HALL_LEARN_STATE_IDLE = 0,
    HALL_LEARN_STATE_WAIT_STABLE,
    HALL_LEARN_STATE_RAW,
    HALL_LEARN_STATE_ORTHOGONAL,
    HALL_LEARN_STATE_OFFSET,
    HALL_LEARN_STATE_ALIGN_STOP,
    HALL_LEARN_STATE_ALIGN,
    HALL_LEARN_STATE_COMPLETE,
    HALL_LEARN_STATE_FAILED
} hall_learn_state_t;

/*
 * 霍尔工作模式：
 * LEARN使用无感角度完成三级学习；
 * NORMAL从Flash读取参数并由霍尔角度直接驱动FOC。
 */
typedef enum {
    HALL_WORK_MODE_LEARN = 0,
    HALL_WORK_MODE_NORMAL = 1
} hall_work_mode_t;

/* 学习模式向正式控制层发布的电机动作，不直接操作PWM。 */
typedef enum {
    HALL_LEARN_DRIVE_STOP = 0,
    HALL_LEARN_DRIVE_SENSORLESS,
    HALL_LEARN_DRIVE_ALIGN
} hall_learn_drive_mode_t;

typedef enum {
    HALL_STORAGE_STATE_IDLE = 0,
    HALL_STORAGE_STATE_WAIT_STOP,
    HALL_STORAGE_STATE_SAVED,
    HALL_STORAGE_STATE_LOADED,
    HALL_STORAGE_STATE_FAILED
} hall_storage_state_t;

/*
 * 三级学习得到的全部参数。
 * gain_xxx_q14为Q14增益，角度一圈对应0...65535。
 */
typedef struct {
    s16 center_a;
    s16 center_b;
    s32 gain_a_q14;
    s32 gain_b_q14;

    s32 center_x;
    s32 center_y;
    s32 gain_x_q14;
    s32 gain_y_q14;

    s16 electrical_offset;
    u8 pole_pairs;
    u8 inverted;
    u8 valid;
} hall_calibration_t;

/* 学习结果和关键中间量，保留为全局变量方便Keil Watch观察。 */
extern volatile hall_calibration_t gHallCalibration;
extern volatile u8 gHallWorkMode;
extern volatile u8 gHallStorageState;
extern volatile u8 gHallLearnState;
extern volatile u8 gHallLearnError;
extern volatile u8 gHallLearnRequest;
extern volatile u16 gHallLearnStableMs;
extern volatile u16 gHallLearnStageElapsedMs;
extern volatile u16 gHallLearnMechanicalTurns;
extern volatile u16 gHallLearnStableTurns;
extern volatile u32 gHallLearnSampleCount;
extern volatile u16 gHallLearnQualityQ15;
extern volatile u16 gHallAlignStableSamples;
extern volatile s16 gHallAlignElectricalRaw;
extern volatile s16 gHallLearnSpinCurrentMa;
extern volatile s16 gHallLearnAlignCurrentMa;
extern volatile s16 gHallLearnAlignPhase;
extern volatile s16 gHallRawA;
extern volatile s16 gHallRawB;
extern volatile s32 gHallNormX;
extern volatile s32 gHallNormY;
extern volatile s16 gHallMechanicalPhase;
extern volatile s16 gHallElectricalPhase;
extern volatile s16 gHallControlPhase;
extern volatile s16 gHallPhaseError;

/* 上电优先加载Flash；没有有效参数时自动进入无感学习模式。 */
void Hall_LearnInit(void);
void Hall_LearnRequest(void);
void Hall_LearnCancel(void);
hall_learn_drive_mode_t Hall_LearnGetDriveMode(void);

/* 学习模式由ADC中断调用：保存同一时刻的两路霍尔值和无感参考角。 */
void Hall_CaptureSample(s16 hall_a, s16 hall_b, s16 reference_phase);

/* 正常模式由ADC中断调用：分频提取霍尔角并逐PWM周期预测控制角。 */
void Hall_FastUpdate(motor_all_state_t *motor, s16 hall_a, s16 hall_b);

/* 1ms慢速任务调用：完成收敛判断、固定角校零和Flash保存。 */
void Hall_LearnTask1ms(u16 elapsed_ms);

bool Hall_CalibrationIsValid(void);
bool Hall_GetElectricalPhase(s16 *phase);

#endif
