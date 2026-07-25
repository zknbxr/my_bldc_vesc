#include "main.h"

/* 第一级和第二级校正后的目标幅值，均采用普通整数表示。 */
#define HALL_LEARN_FIRST_TARGET             (8192L)
#define HALL_LEARN_SECOND_TARGET            (8192L)
#define HALL_LEARN_GAIN_SHIFT               (14U)
#define HALL_LEARN_GAIN_ONE                 (1L << HALL_LEARN_GAIN_SHIFT)
#define HALL_LEARN_GAIN_MAX                 (65535L)

/* 每阶段至少观察5秒且连续多圈收敛；这里只设最短时间，不设学习超时。 */
#define HALL_LEARN_STABLE_TIME_MS            (500U)
#define HALL_LEARN_MIN_STAGE_TIME_MS         (5000U)
#define HALL_LEARN_MIN_ERPM                  (300L)
#define HALL_LEARN_MIN_MECHANICAL_TURNS      (12U)
#define HALL_LEARN_STABLE_TURNS              (6U)
#define HALL_LEARN_RAW_EDGE_TOLERANCE        (24L)
#define HALL_LEARN_ORTHO_EDGE_TOLERANCE      (48L)
#define HALL_LEARN_OFFSET_TOLERANCE          (256L)
#define HALL_LEARN_MIN_RAW_AMPLITUDE         (512L)
#define HALL_LEARN_MIN_ORTHO_AMPLITUDE       (512L)
#define HALL_LEARN_MAX_PHASE_STEP            (8192L)
#define HALL_LEARN_MIN_QUALITY_Q15           (28000U)

/* 固定电角度吸附后，以连续稳定的霍尔角圆周平均计算最终零偏。 */
#define HALL_ALIGN_STABLE_SAMPLES            (500U)
#define HALL_ALIGN_MAX_PHASE_STEP            (128L)
#define HALL_ALIGN_MAX_PHASE_DEVIATION       (256L)
#define HALL_ALIGN_CURRENT_TOLERANCE_MA       (50L)
#define HALL_ALIGN_MEASURED_TOLERANCE_MA      (150L)

/* 正常模式每4个PWM周期提取一次霍尔原始角，其余周期按角度差速度外推。 */
#define HALL_RUNTIME_ANGLE_DIV                (2U)
#define HALL_RUNTIME_MAX_PHASE_STEP           (8192L)
#define HALL_RUNTIME_SPEED_FILTER_DIV         (16L)
/* 14 kHz PWM、每4周期更新时，每1 ERPM对应的Q16角度步进。 */
#define HALL_RUNTIME_STEP_Q16_PER_ERPM        (20452L)

/* 霍尔参数独占主Flash最后一个512字节扇区。 */
#define HALL_FLASH_MAGIC                      (0x48414C4CUL)
#define HALL_FLASH_VERSION                    (3U)
#define HALL_FLASH_ERASE_KEY                  (0x9A0D361FUL)
#define HALL_FLASH_PROGRAM_KEY                (0x9AFDA40CUL)

/* 学习失败原因，数值保存在 gHallLearnError 中。 */
#define HALL_LEARN_ERROR_NONE                (0U)
#define HALL_LEARN_ERROR_RAW_RANGE           (1U)
#define HALL_LEARN_ERROR_RAW_GAIN            (2U)
#define HALL_LEARN_ERROR_ORTHO_RANGE         (3U)
#define HALL_LEARN_ERROR_ORTHO_GAIN          (4U)
#define HALL_LEARN_ERROR_PHASE_QUALITY       (5U)
#define HALL_LEARN_ERROR_FLASH               (6U)

typedef struct {
    s32 min_a;
    s32 max_a;
    s32 min_b;
    s32 max_b;
} hall_min_max_t;

typedef struct {
    u32 magic;
    u16 version;
    u16 size;
    hall_calibration_t calibration;
    u16 quality_q15;
    u16 reserved;
    u32 crc32;
} hall_flash_record_t;

volatile hall_calibration_t gHallCalibration;
/* 默认优先进入正常模式；Flash无有效参数时会自动退回学习模式。 */
volatile u8 gHallStorageState;
volatile u8 gHallLearnState;
volatile u8 gHallLearnError;
volatile u8 gHallLearnRequest;
volatile u16 gHallLearnStableMs;
volatile u16 gHallLearnStageElapsedMs;
volatile u16 gHallLearnMechanicalTurns;
volatile u16 gHallLearnStableTurns;
volatile u32 gHallLearnSampleCount;
volatile u16 gHallLearnQualityQ15;
volatile u16 gHallAlignStableSamples;
volatile s16 gHallAlignElectricalRaw;
volatile s16 gHallLearnAlignCurrentMa = 600;
volatile s16 gHallLearnAlignPhase = 0;
volatile s16 gHallRawA;
volatile s16 gHallRawB;
volatile s32 gHallNormX;
volatile s32 gHallNormY;
volatile s16 gHallMechanicalPhase;
volatile s16 gHallElectricalPhase;
volatile s16 gHallControlPhase;
volatile s16 gHallPhaseError;

/* ADC 中断发布的最近一次同步快照。 */
static volatile s16 s_hallSampleA;
static volatile s16 s_hallSampleB;
static volatile s16 s_hallReferencePhase;

static hall_min_max_t s_hallMinMax;
static hall_min_max_t s_hallMinMaxCheckpoint;
static u32 s_hallElectricalTravel;
static u16 s_hallReferencePhaseLast;
static bool s_hallReferencePhaseValid;
static s8 s_hallLearnDirection;
static u16 s_hallConvergenceTurnLast;
static bool s_hallMinMaxCheckpointValid;

/* 正反两个机械角方向候选的圆周平均累加量。 */
static s32 s_hallOffsetSinPositive;
static s32 s_hallOffsetCosPositive;
static s32 s_hallOffsetSinNegative;
static s32 s_hallOffsetCosNegative;
static u32 s_hallOffsetSamples;
static s16 s_hallOffsetCandidateLast;
static u8 s_hallOffsetInvertedLast;
static bool s_hallOffsetCandidateValid;

/* 固定角吸附阶段的零偏圆周平均状态。 */
static s32 s_hallAlignOffsetSin;
static s32 s_hallAlignOffsetCos;
static s16 s_hallAlignPhaseLast;
static s16 s_hallAlignPhaseAnchor;
static bool s_hallAlignPhaseValid;

/* 正常模式霍尔角度差测速及逐PWM角度预测状态。 */
static u8 s_hallRuntimeCounter;
static bool s_hallRuntimeActive;
static bool s_hallElectricalPhaseValid;
static u16 s_hallElectricalPhaseLast;
static s32 s_hallSpeedStepQ16;

static s32 Hall_AbsS32(s32 value)
{
    return (value >= 0L) ? value : -value;
}

static u32 Hall_Crc32(const u8 *data, u32 length)
{
    u32 crc;
    u32 i;
    u8 bit;

    crc = 0xFFFFFFFFUL;
    for(i = 0UL; i < length; i++)
    {
        crc ^= data[i];
        for(bit = 0U; bit < 8U; bit++)
        {
            if((crc & 1UL) != 0UL)
            {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return ~crc;
}

static void Hall_ClearBytes(void *address, u32 size)
{
    u8 *data;
    u32 i;

    data = (u8 *)address;
    for(i = 0UL; i < size; i++)
    {
        data[i] = 0U;
    }
}

static void Hall_CopyCalibrationToRecord(hall_flash_record_t *record)
{
    Hall_ClearBytes(record, sizeof(*record));
    record->magic = HALL_FLASH_MAGIC;
    record->version = HALL_FLASH_VERSION;
    record->size = (u16)sizeof(*record);
    record->calibration.center_a = gHallCalibration.center_a;
    record->calibration.center_b = gHallCalibration.center_b;
    record->calibration.gain_a_q14 = gHallCalibration.gain_a_q14;
    record->calibration.gain_b_q14 = gHallCalibration.gain_b_q14;
    record->calibration.center_x = gHallCalibration.center_x;
    record->calibration.center_y = gHallCalibration.center_y;
    record->calibration.gain_x_q14 = gHallCalibration.gain_x_q14;
    record->calibration.gain_y_q14 = gHallCalibration.gain_y_q14;
    record->calibration.electrical_offset =gHallCalibration.electrical_offset;
    record->calibration.pole_pairs = gHallCalibration.pole_pairs;
    record->calibration.inverted = gHallCalibration.inverted;
    record->calibration.valid = gHallCalibration.valid;
    record->quality_q15 = gHallLearnQualityQ15;
    record->crc32 = Hall_Crc32(
        (const u8 *)record, (u32)offsetof(hall_flash_record_t, crc32));
}

static bool Hall_RecordIsValid(const hall_flash_record_t *record)
{
    u32 crc;

    if((record->magic != HALL_FLASH_MAGIC) ||
       (record->version != HALL_FLASH_VERSION) ||
       (record->size != sizeof(*record)) ||
       (record->calibration.valid == 0U) ||
       (record->calibration.pole_pairs != (u8)Pole_Pairs) ||
       (record->quality_q15 < HALL_LEARN_MIN_QUALITY_Q15))
    {
        return false;
    }

    if((record->calibration.gain_a_q14 <= 0L) ||
       (record->calibration.gain_b_q14 <= 0L) ||
       (record->calibration.gain_x_q14 <= 0L) ||
       (record->calibration.gain_y_q14 <= 0L) ||
       (record->calibration.gain_a_q14 > HALL_LEARN_GAIN_MAX) ||
       (record->calibration.gain_b_q14 > HALL_LEARN_GAIN_MAX) ||
       (record->calibration.gain_x_q14 > HALL_LEARN_GAIN_MAX) ||
       (record->calibration.gain_y_q14 > HALL_LEARN_GAIN_MAX))
    {
        return false;
    }

    crc = Hall_Crc32(
        (const u8 *)record, (u32)offsetof(hall_flash_record_t, crc32));
    return crc == record->crc32;
}

static void Hall_CopyRecordToCalibration(const hall_flash_record_t *record)
{
    gHallCalibration.center_a = record->calibration.center_a;
    gHallCalibration.center_b = record->calibration.center_b;
    gHallCalibration.gain_a_q14 = record->calibration.gain_a_q14;
    gHallCalibration.gain_b_q14 = record->calibration.gain_b_q14;
    gHallCalibration.center_x = record->calibration.center_x;
    gHallCalibration.center_y = record->calibration.center_y;
    gHallCalibration.gain_x_q14 = record->calibration.gain_x_q14;
    gHallCalibration.gain_y_q14 = record->calibration.gain_y_q14;
    gHallCalibration.electrical_offset =
        record->calibration.electrical_offset;
    gHallCalibration.pole_pairs = record->calibration.pole_pairs;
    gHallCalibration.inverted = record->calibration.inverted;
    gHallCalibration.valid = record->calibration.valid;
    gHallLearnQualityQ15 = record->quality_q15;
}

static bool Hall_LoadCalibration(void)
{
    hall_flash_record_t record;

    Hall_ClearBytes(&record, sizeof(record));
    Read_Flash(HALL_FLASH_Sector_StartAddr,
               (u32 *)&record,
               (sizeof(record) + 3U) >> 2,
               MEMORY_TYPE);
    if(!Hall_RecordIsValid(&record))
    {
        return false;
    }

    Hall_CopyRecordToCalibration(&record);
    return true;
}

static bool Hall_SaveCalibration(void)
{
    hall_flash_record_t record;
    hall_flash_record_t verify;
    int program_result;

    Hall_CopyCalibrationToRecord(&record);
    Hall_ClearBytes(&verify, sizeof(verify));

    /* 擦写期间必须保持PWM关闭并禁止中断，避免从Flash取指被打断。 */
    __disable_irq();
    EraseSector(HALL_FLASH_Sector_StartAddr,
                MEMORY_TYPE,
                HALL_FLASH_ERASE_KEY);
    program_result = ProgramPage(HALL_FLASH_Sector_StartAddr,
                                 sizeof(record),
                                 (u8 *)&record,
                                 MEMORY_TYPE,
                                 HALL_FLASH_PROGRAM_KEY);
    Read_Flash(HALL_FLASH_Sector_StartAddr,
               (u32 *)&verify,
               (sizeof(verify) + 3U) >> 2,
               MEMORY_TYPE);
    __enable_irq();

    return (program_result != 0) && Hall_RecordIsValid(&verify);
}

static void Hall_ResetMinMax(void)
{
    s_hallMinMax.min_a = 2147483647L;
    s_hallMinMax.max_a = -2147483647L;
    s_hallMinMax.min_b = 2147483647L;
    s_hallMinMax.max_b = -2147483647L;
}

static void Hall_UpdateMinMax(s32 value_a, s32 value_b)
{
    if(value_a < s_hallMinMax.min_a)
    {
        s_hallMinMax.min_a = value_a;
    }
    if(value_a > s_hallMinMax.max_a)
    {
        s_hallMinMax.max_a = value_a;
    }
    if(value_b < s_hallMinMax.min_b)
    {
        s_hallMinMax.min_b = value_b;
    }
    if(value_b > s_hallMinMax.max_b)
    {
        s_hallMinMax.max_b = value_b;
    }
}

static void Hall_ClearCalibration(void)
{
    gHallCalibration.center_a = 0;
    gHallCalibration.center_b = 0;
    gHallCalibration.gain_a_q14 = 0L;
    gHallCalibration.gain_b_q14 = 0L;
    gHallCalibration.center_x = 0L;
    gHallCalibration.center_y = 0L;
    gHallCalibration.gain_x_q14 = 0L;
    gHallCalibration.gain_y_q14 = 0L;
    gHallCalibration.electrical_offset = 0;
    gHallCalibration.pole_pairs = (u8)Pole_Pairs;
    gHallCalibration.inverted = 0U;
    gHallCalibration.valid = 0U;
}

static void Hall_ResetStage(hall_learn_state_t state)
{
    Hall_ResetMinMax();
    s_hallMinMaxCheckpoint.min_a = 0L;
    s_hallMinMaxCheckpoint.max_a = 0L;
    s_hallMinMaxCheckpoint.min_b = 0L;
    s_hallMinMaxCheckpoint.max_b = 0L;
    s_hallElectricalTravel = 0UL;
    s_hallReferencePhaseLast = 0U;
    s_hallReferencePhaseValid = false;
    s_hallConvergenceTurnLast = 0U;
    s_hallMinMaxCheckpointValid = false;
    s_hallOffsetSinPositive = 0L;
    s_hallOffsetCosPositive = 0L;
    s_hallOffsetSinNegative = 0L;
    s_hallOffsetCosNegative = 0L;
    s_hallOffsetSamples = 0UL;
    s_hallOffsetCandidateLast = 0;
    s_hallOffsetInvertedLast = 0U;
    s_hallOffsetCandidateValid = false;
    gHallLearnSampleCount = 0UL;
    gHallLearnStageElapsedMs = 0U;
    gHallLearnMechanicalTurns = 0U;
    gHallLearnStableTurns = 0U;
    gHallLearnState = (u8)state;
}

static void Hall_ResetLearning(void)
{
    Hall_ClearCalibration();
    gHallLearnError = HALL_LEARN_ERROR_NONE;
    gHallLearnQualityQ15 = 0U;
    gHallLearnStableMs = 0U;
    gHallRawA = 0;
    gHallRawB = 0;
    gHallNormX = 0L;
    gHallNormY = 0L;
    gHallMechanicalPhase = 0;
    gHallElectricalPhase = 0;
    gHallControlPhase = 0;
    gHallPhaseError = 0;
    gHallAlignStableSamples = 0U;
    gHallAlignElectricalRaw = 0;
    s_hallAlignOffsetSin = 0L;
    s_hallAlignOffsetCos = 0L;
    s_hallAlignPhaseLast = 0;
    s_hallAlignPhaseAnchor = 0;
    s_hallAlignPhaseValid = false;
    gHallStorageState = HALL_STORAGE_STATE_IDLE;
    s_hallLearnDirection = 0;
    s_hallRuntimeCounter = 0U;
    s_hallRuntimeActive = false;
    Hall_ResetStage(HALL_LEARN_STATE_WAIT_STABLE);
}

static void Hall_Fail(u8 error)
{
    gHallCalibration.valid = 0U;
    gHallLearnError = error;
    gHallLearnRequest = 0U;
    gHallLearnState = HALL_LEARN_STATE_FAILED;
}

static s32 Hall_CalculateGain(s32 amplitude, s32 target)
{
    s32 gain;

    if(amplitude <= 0L)
    {
        return 0L;
    }

    gain = (target * HALL_LEARN_GAIN_ONE) / amplitude;
    if((gain <= 0L) || (gain > HALL_LEARN_GAIN_MAX))
    {
        return 0L;
    }

    return gain;
}

static s32 Hall_Normalize(s32 raw, s32 center, s32 gain_q14)
{
    return ((raw - center) * gain_q14) >> HALL_LEARN_GAIN_SHIFT;
}

/* 两级校正：原始 A/B -> 等幅 A/B -> 正交 X/Y。 */
static void Hall_CalculateOrthogonal(s16 raw_a, s16 raw_b,
                                     s32 *hall_x, s32 *hall_y)
{
    s32 hall_a_normalized;
    s32 hall_b_normalized;
    s32 hall_x_raw;
    s32 hall_y_raw;

    hall_a_normalized = Hall_Normalize(
        raw_a, gHallCalibration.center_a, gHallCalibration.gain_a_q14);
    hall_b_normalized = Hall_Normalize(
        raw_b, gHallCalibration.center_b, gHallCalibration.gain_b_q14);

    hall_x_raw = hall_a_normalized - hall_b_normalized;
    hall_y_raw = hall_a_normalized + hall_b_normalized;

    *hall_x = Hall_Normalize(
        hall_x_raw, gHallCalibration.center_x, gHallCalibration.gain_x_q14);
    *hall_y = Hall_Normalize(
        hall_y_raw, gHallCalibration.center_y, gHallCalibration.gain_y_q14);
}

static u16 Hall_MechanicalToElectrical(u16 mechanical_phase, bool inverted)
{
    u16 electrical_phase;

    electrical_phase = (u16)((u32)mechanical_phase * (u32)Pole_Pairs);
    if(inverted)
    {
        electrical_phase = (u16)(0U - electrical_phase);
    }

    return electrical_phase;
}

/*
 * 由无感电角度差分累计转过的距离。只接受学习开始时的旋转方向，
 * 可以同时避开角度跨 0 和短时反向抖动造成的误计数。
 */
static void Hall_UpdateTravel(u16 reference_phase)
{
    s32 phase_step;
    u32 one_mechanical_turn;

    if(!s_hallReferencePhaseValid)
    {
        s_hallReferencePhaseLast = reference_phase;
        s_hallReferencePhaseValid = true;
        return;
    }

    phase_step = (s32)(s16)(reference_phase - s_hallReferencePhaseLast);
    s_hallReferencePhaseLast = reference_phase;
    if(Hall_AbsS32(phase_step) > HALL_LEARN_MAX_PHASE_STEP)
    {
        return;
    }

    if(((s_hallLearnDirection > 0) && (phase_step > 0L)) ||
       ((s_hallLearnDirection < 0) && (phase_step < 0L)))
    {
        s_hallElectricalTravel += (u32)Hall_AbsS32(phase_step);
    }

    one_mechanical_turn = (u32)Pole_Pairs << 16;
    if(one_mechanical_turn != 0UL)
    {
        gHallLearnMechanicalTurns = (u16)(
            s_hallElectricalTravel / one_mechanical_turn);
    }
}

static bool Hall_MinMaxChangedWithin(s32 tolerance)
{
    return (Hall_AbsS32(s_hallMinMax.min_a -
                        s_hallMinMaxCheckpoint.min_a) <= tolerance) &&
           (Hall_AbsS32(s_hallMinMax.max_a -
                        s_hallMinMaxCheckpoint.max_a) <= tolerance) &&
           (Hall_AbsS32(s_hallMinMax.min_b -
                        s_hallMinMaxCheckpoint.min_b) <= tolerance) &&
           (Hall_AbsS32(s_hallMinMax.max_b -
                        s_hallMinMaxCheckpoint.max_b) <= tolerance);
}

static void Hall_SaveMinMaxCheckpoint(void)
{
    s_hallMinMaxCheckpoint = s_hallMinMax;
    s_hallMinMaxCheckpointValid = true;
}

/*
 * 每转完一个机械圈才比较一次极值包络。只有中心和幅值连续多圈不再扩展，
 * 并且已经覆盖足够机械圈数，才认为这一阶段真正收敛。
 */
static bool Hall_MinMaxConverged(s32 tolerance)
{
    if((gHallLearnMechanicalTurns == 0U) ||
       (gHallLearnMechanicalTurns == s_hallConvergenceTurnLast))
    {
        return false;
    }

    s_hallConvergenceTurnLast = gHallLearnMechanicalTurns;
    if(!s_hallMinMaxCheckpointValid)
    {
        Hall_SaveMinMaxCheckpoint();
        gHallLearnStableTurns = 0U;
        return false;
    }

    if(Hall_MinMaxChangedWithin(tolerance))
    {
        if(gHallLearnStableTurns < 65535U)
        {
            gHallLearnStableTurns++;
        }
    }
    else
    {
        gHallLearnStableTurns = 0U;
    }
    Hall_SaveMinMaxCheckpoint();

    return (gHallLearnStageElapsedMs >= HALL_LEARN_MIN_STAGE_TIME_MS) &&
           (gHallLearnMechanicalTurns >= HALL_LEARN_MIN_MECHANICAL_TURNS) &&
           (gHallLearnStableTurns >= HALL_LEARN_STABLE_TURNS);
}

static void Hall_CompleteRawStage(void)
{
    s32 amplitude_a;
    s32 amplitude_b;

    amplitude_a = (s_hallMinMax.max_a - s_hallMinMax.min_a) / 2L;
    amplitude_b = (s_hallMinMax.max_b - s_hallMinMax.min_b) / 2L;
    if((amplitude_a < HALL_LEARN_MIN_RAW_AMPLITUDE) ||
       (amplitude_b < HALL_LEARN_MIN_RAW_AMPLITUDE))
    {
        Hall_Fail(HALL_LEARN_ERROR_RAW_RANGE);
        return;
    }

    gHallCalibration.center_a = (s16)(
        (s_hallMinMax.max_a + s_hallMinMax.min_a) / 2L);
    gHallCalibration.center_b = (s16)(
        (s_hallMinMax.max_b + s_hallMinMax.min_b) / 2L);
    gHallCalibration.gain_a_q14 = Hall_CalculateGain(
        amplitude_a, HALL_LEARN_FIRST_TARGET);
    gHallCalibration.gain_b_q14 = Hall_CalculateGain(
        amplitude_b, HALL_LEARN_FIRST_TARGET);
    if((gHallCalibration.gain_a_q14 == 0L) ||
       (gHallCalibration.gain_b_q14 == 0L))
    {
        Hall_Fail(HALL_LEARN_ERROR_RAW_GAIN);
        return;
    }

    Hall_ResetStage(HALL_LEARN_STATE_ORTHOGONAL);
}

static void Hall_CompleteOrthogonalStage(void)
{
    s32 amplitude_x;
    s32 amplitude_y;

    amplitude_x = (s_hallMinMax.max_a - s_hallMinMax.min_a) / 2L;
    amplitude_y = (s_hallMinMax.max_b - s_hallMinMax.min_b) / 2L;
    if((amplitude_x < HALL_LEARN_MIN_ORTHO_AMPLITUDE) ||
       (amplitude_y < HALL_LEARN_MIN_ORTHO_AMPLITUDE))
    {
        Hall_Fail(HALL_LEARN_ERROR_ORTHO_RANGE);
        return;
    }

    gHallCalibration.center_x =
        (s_hallMinMax.max_a + s_hallMinMax.min_a) / 2L;
    gHallCalibration.center_y =
        (s_hallMinMax.max_b + s_hallMinMax.min_b) / 2L;
    gHallCalibration.gain_x_q14 = Hall_CalculateGain(
        amplitude_x, HALL_LEARN_SECOND_TARGET);
    gHallCalibration.gain_y_q14 = Hall_CalculateGain(
        amplitude_y, HALL_LEARN_SECOND_TARGET);
    if((gHallCalibration.gain_x_q14 == 0L) ||
       (gHallCalibration.gain_y_q14 == 0L))
    {
        Hall_Fail(HALL_LEARN_ERROR_ORTHO_GAIN);
        return;
    }

    Hall_ResetStage(HALL_LEARN_STATE_OFFSET);
}

/* 用 max + 3/8*min 近似圆周累加矢量幅值，避免平方和开方。 */
static s32 Hall_ApproxVectorMagnitude(s32 x, s32 y)
{
    s32 max_value;
    s32 min_value;

    x = Hall_AbsS32(x);
    y = Hall_AbsS32(y);
    if(x >= y)
    {
        max_value = x;
        min_value = y;
    }
    else
    {
        max_value = y;
        min_value = x;
    }

    return max_value + ((min_value * 3L) >> 3);
}

static void Hall_ResetOffsetWindow(void)
{
    s_hallOffsetSinPositive = 0L;
    s_hallOffsetCosPositive = 0L;
    s_hallOffsetSinNegative = 0L;
    s_hallOffsetCosNegative = 0L;
    s_hallOffsetSamples = 0UL;
}

static bool Hall_GetOffsetCandidate(u8 *inverted, s16 *offset,
                                    u16 *quality_q15)
{
    s32 magnitude_positive;
    s32 magnitude_negative;
    s32 selected_sin;
    s32 selected_cos;
    s32 quality;

    if((inverted == 0) || (offset == 0) || (quality_q15 == 0) ||
       (s_hallOffsetSamples == 0UL))
    {
        return false;
    }

    magnitude_positive = Hall_ApproxVectorMagnitude(
        s_hallOffsetCosPositive, s_hallOffsetSinPositive);
    magnitude_negative = Hall_ApproxVectorMagnitude(
        s_hallOffsetCosNegative, s_hallOffsetSinNegative);

    if(magnitude_positive >= magnitude_negative)
    {
        *inverted = 0U;
        selected_sin = s_hallOffsetSinPositive;
        selected_cos = s_hallOffsetCosPositive;
        quality = magnitude_positive / (s32)s_hallOffsetSamples;
    }
    else
    {
        *inverted = 1U;
        selected_sin = s_hallOffsetSinNegative;
        selected_cos = s_hallOffsetCosNegative;
        quality = magnitude_negative / (s32)s_hallOffsetSamples;
    }

    if(quality > 32767L)
    {
        quality = 32767L;
    }
    *quality_q15 = (u16)quality;
    *offset = (s16)Foc_Atan2Q16(selected_sin, selected_cos);
    return true;
}

/* 方向和旋转中相位关系连续多圈一致后，进入固定角吸附校零。 */
static bool Hall_OffsetConverged(void)
{
    u8 inverted;
    s16 offset;
    u16 quality;
    bool stable;

    if((gHallLearnMechanicalTurns == 0U) ||
       (gHallLearnMechanicalTurns == s_hallConvergenceTurnLast))
    {
        return false;
    }
    s_hallConvergenceTurnLast = gHallLearnMechanicalTurns;

    if(!Hall_GetOffsetCandidate(&inverted, &offset, &quality))
    {
        Hall_ResetOffsetWindow();
        return false;
    }
    Hall_ResetOffsetWindow();
    gHallLearnQualityQ15 = quality;

    stable = s_hallOffsetCandidateValid &&
             (inverted == s_hallOffsetInvertedLast) &&
             (Hall_AbsS32((s32)(s16)((u16)offset -
                 (u16)s_hallOffsetCandidateLast)) <=
                 HALL_LEARN_OFFSET_TOLERANCE) &&
             (quality >= HALL_LEARN_MIN_QUALITY_Q15);
    if(stable)
    {
        if(gHallLearnStableTurns < 65535U)
        {
            gHallLearnStableTurns++;
        }
    }
    else
    {
        gHallLearnStableTurns = 0U;
    }

    s_hallOffsetCandidateLast = offset;
    s_hallOffsetInvertedLast = inverted;
    s_hallOffsetCandidateValid = quality >= HALL_LEARN_MIN_QUALITY_Q15;

    if((gHallLearnStageElapsedMs < HALL_LEARN_MIN_STAGE_TIME_MS) ||
       (gHallLearnMechanicalTurns < HALL_LEARN_MIN_MECHANICAL_TURNS) ||
       (gHallLearnStableTurns < HALL_LEARN_STABLE_TURNS))
    {
        return false;
    }

    gHallCalibration.inverted = inverted;
    gHallCalibration.electrical_offset = offset;
    return true;
}

static void Hall_PrepareAlignment(void)
{
    gHallCalibration.valid = 0U;
    gHallAlignStableSamples = 0U;
    gHallAlignElectricalRaw = 0;
    s_hallAlignOffsetSin = 0L;
    s_hallAlignOffsetCos = 0L;
    s_hallAlignPhaseLast = 0;
    s_hallAlignPhaseAnchor = 0;
    s_hallAlignPhaseValid = false;
    gHallLearnState = HALL_LEARN_STATE_ALIGN_STOP;
}

static void Hall_CompleteAlignment(void)
{
    if(gHallAlignStableSamples == 0U)
    {
        return;
    }

    gHallCalibration.electrical_offset = (s16)Foc_Atan2Q16(
        s_hallAlignOffsetSin, s_hallAlignOffsetCos);
    gHallCalibration.pole_pairs = (u8)Pole_Pairs;
    gHallCalibration.valid = 1U;
    gHallLearnState = HALL_LEARN_STATE_COMPLETE;
    gHallLearnRequest = 0U;
    gHallStorageState = HALL_STORAGE_STATE_WAIT_STOP;

    /* 学习控制层会先将吸附电流斜坡降为0，PWM关闭后再写Flash。 */
}

static void Hall_AccumulateOffset(s16 raw_a, s16 raw_b,
                                  s16 reference_phase)
{
    MCS_TRIG_Q15 error_trig;
    u16 mechanical_phase;
    u16 electrical_positive;
    u16 electrical_negative;
    s16 error_positive;
    s16 error_negative;
    s32 hall_x;
    s32 hall_y;

    Hall_CalculateOrthogonal(raw_a, raw_b, &hall_x, &hall_y);
    mechanical_phase = Foc_Atan2Q16(hall_y, hall_x);
    electrical_positive = Hall_MechanicalToElectrical(
        mechanical_phase, false);
    electrical_negative = Hall_MechanicalToElectrical(
        mechanical_phase, true);

    gHallNormX = hall_x;
    gHallNormY = hall_y;
    gHallMechanicalPhase = (s16)mechanical_phase;

    error_positive = (s16)((u16)reference_phase - electrical_positive);
    error_negative = (s16)((u16)reference_phase - electrical_negative);

    error_trig = Motor_GetSinCosQ15((u16)error_positive);
    s_hallOffsetSinPositive += error_trig.sin;
    s_hallOffsetCosPositive += error_trig.cos;

    error_trig = Motor_GetSinCosQ15((u16)error_negative);
    s_hallOffsetSinNegative += error_trig.sin;
    s_hallOffsetCosNegative += error_trig.cos;
    s_hallOffsetSamples++;

    gHallElectricalPhase = (s16)electrical_positive;
    gHallPhaseError = error_positive;
}

static void Hall_ResetAlignmentAverage(void)
{
    gHallAlignStableSamples = 0U;
    s_hallAlignOffsetSin = 0L;
    s_hallAlignOffsetCos = 0L;
}
volatile uint8_t text_check;
static void Hall_ProcessAlignment(s16 raw_a, s16 raw_b)
{
    MCS_TRIG_Q15 offset_trig;
    s32 hall_x;
    s32 hall_y;
    s32 current_error;
    u16 mechanical_phase;
    u16 electrical_phase;
    s16 offset;
    s16 phase_step;

    current_error = Hall_AbsS32((s32)m_motor.m_id_set -
        Hall_AbsS32((s32)gHallLearnAlignCurrentMa));
    if((m_motor.m_run_state != MOTOR_RUN_STATE_RUNNING) ||
       !Motor_IsPwmEnabled() ||
       (m_motor.m_control_mode != CONTROL_MODE_CURRENT) ||
       !m_motor.m_phase_override ||
       (current_error > HALL_ALIGN_CURRENT_TOLERANCE_MA) ||
       (Hall_AbsS32((s32)m_motor.m_motor_state.id -
           Hall_AbsS32((s32)gHallLearnAlignCurrentMa)) >
           HALL_ALIGN_MEASURED_TOLERANCE_MA) ||
       (Hall_AbsS32((s32)m_motor.m_motor_state.iq) >
           HALL_ALIGN_MEASURED_TOLERANCE_MA))
    {
        s_hallAlignPhaseValid = false;
        text_check = 1;
        ///Hall_ResetAlignmentAverage();
        return;
    }

    Hall_CalculateOrthogonal(raw_a, raw_b, &hall_x, &hall_y);
    mechanical_phase = Foc_Atan2Q16(hall_y, hall_x);
    electrical_phase = Hall_MechanicalToElectrical(
        mechanical_phase, gHallCalibration.inverted != 0U);

    gHallNormX = hall_x;
    gHallNormY = hall_y;
    gHallMechanicalPhase = (s16)mechanical_phase;
    gHallAlignElectricalRaw = (s16)electrical_phase;

    if(!s_hallAlignPhaseValid)
    {
        s_hallAlignPhaseLast = (s16)electrical_phase;
        s_hallAlignPhaseAnchor = (s16)electrical_phase;
        s_hallAlignPhaseValid = true;
        text_check = 2;
       // Hall_ResetAlignmentAverage();
        return;
    }

    phase_step = (s16)(electrical_phase - (u16)s_hallAlignPhaseLast);
    s_hallAlignPhaseLast = (s16)electrical_phase;
    if((Hall_AbsS32((s32)phase_step) > HALL_ALIGN_MAX_PHASE_STEP) ||
       (Hall_AbsS32((s32)(s16)(electrical_phase -
           (u16)s_hallAlignPhaseAnchor)) >
           HALL_ALIGN_MAX_PHASE_DEVIATION))
    {
        s_hallAlignPhaseAnchor = (s16)electrical_phase;
        //Hall_ResetAlignmentAverage();
        
        return;
    }
    text_check = 3;
    offset = (s16)((u16)gHallLearnAlignPhase - electrical_phase);
    offset_trig = Motor_GetSinCosQ15((u16)offset);
    s_hallAlignOffsetSin += offset_trig.sin;
    s_hallAlignOffsetCos += offset_trig.cos;
    if(gHallAlignStableSamples < HALL_ALIGN_STABLE_SAMPLES)
    {
        gHallAlignStableSamples++;
    }

    gHallElectricalPhase = (s16)(electrical_phase + (u16)offset);
    gHallPhaseError = offset;
    if(gHallAlignStableSamples >= HALL_ALIGN_STABLE_SAMPLES)
    {
        Hall_CompleteAlignment();
    }
}

hall_learn_drive_mode_t Hall_LearnGetDriveMode(void)
{
    if(gMotorWorkMode != MCS_WORK_MODE_LEARN)
    {
        return HALL_LEARN_DRIVE_STOP;
    }

    switch((hall_learn_state_t)gHallLearnState)
    {
    case HALL_LEARN_STATE_WAIT_STABLE:
    case HALL_LEARN_STATE_RAW:
    case HALL_LEARN_STATE_ORTHOGONAL:
    case HALL_LEARN_STATE_OFFSET:
        return HALL_LEARN_DRIVE_SENSORLESS;

    case HALL_LEARN_STATE_ALIGN:
        return HALL_LEARN_DRIVE_ALIGN;

    default:
        return HALL_LEARN_DRIVE_STOP;
    }
}

/*
 * 学习状态机只生成控制请求，不直接操作 PWM 或公共运行状态。
 * 旋转阶段使用无感速度环，固定角阶段使用 d 轴电流吸附。
 */
void Hall_LearnBuildControlRequest(motor_control_request_t *request)
{
    hall_learn_drive_mode_t drive_mode;
    s32 align_current;

    if((request == 0) || (gMotorWorkMode != MCS_WORK_MODE_LEARN))
    {
        return;
    }

    drive_mode = Hall_LearnGetDriveMode();
    request->sensor_mode = FOC_SENSOR_MODE_SENSORLESS;

    switch(drive_mode)
    {
    case HALL_LEARN_DRIVE_SENSORLESS:
        request->run = true;
        request->control_mode = CONTROL_MODE_SPEED;
        request->speed_target_erpm = MCS_SPEED_TARGET_DEFAULT_ERPM;
        break;

    case HALL_LEARN_DRIVE_ALIGN:
        align_current = Hall_AbsS32((s32)gHallLearnAlignCurrentMa);
        if(align_current > 32767L)
        {
            align_current = 32767L;
        }
        request->run = true;
        request->control_mode = CONTROL_MODE_CURRENT;
        request->phase_override = true;
        request->phase_override_q16 = gHallLearnAlignPhase;
        request->id_target_ma = (s16)align_current;
        request->iq_target_ma = 0;
        break;

    default:
        break;
    }
}

static void Hall_ResetRuntime(motor_all_state_t *motor)
{
    motor_state_t *state;

    state = &motor->m_motor_state;
    foc_observer_reset(&motor->m_observer_state);
    motor->m_phase_control_initialized = false;
    motor->m_phase_control_q16 = (u32)(u16)state->phase << 16;
    motor->m_pll_phase = state->phase;
    motor->m_pll_speed = 0;
    s_hallElectricalPhaseValid = false;
    s_hallElectricalPhaseLast = 0U;
    s_hallSpeedStepQ16 = 0L;
    s_hallRuntimeCounter = HALL_RUNTIME_ANGLE_DIV - 1U;
    s_hallRuntimeActive = true;
}

void Hall_FastUpdate(motor_all_state_t *motor, s16 hall_a, s16 hall_b)
{
    motor_state_t *state;
    observer_state *pll;
    s32 hall_x;
    s32 hall_y;
    s32 phase_step;
    s32 speed_step_target_q16;
    s32 speed_erpm;
    u16 mechanical_phase;
    u16 electrical_phase;
    bool angle_updated;

    if((motor == 0) || (motor->m_conf == 0))
    {
        return;
    }
    // 学习模式
    if((gMotorWorkMode != MCS_WORK_MODE_CONTROL) ||
       (gHallCalibration.valid == 0U) ||
       (motor->m_conf->foc_sensor_mode != FOC_SENSOR_MODE_HALL))
    {
        s_hallRuntimeCounter = 0U;
        s_hallRuntimeActive = false;
        s_hallElectricalPhaseValid = false;
        s_hallSpeedStepQ16 = 0L;
        return;
    }

    if(!s_hallRuntimeActive)
    {
        // 为什么要四个周期提取一次霍尔原始角，有疑问
        Hall_ResetRuntime(motor);
    }

    state = &motor->m_motor_state;
    pll = &motor->m_observer_state;
    angle_updated = false;

    s_hallRuntimeCounter++;
    if(s_hallRuntimeCounter >= HALL_RUNTIME_ANGLE_DIV)
    {
        s_hallRuntimeCounter = 0U;
        Hall_CalculateOrthogonal(hall_a, hall_b, &hall_x, &hall_y);
        mechanical_phase = Foc_Atan2Q16(hall_y, hall_x);
        electrical_phase = Hall_MechanicalToElectrical(
            mechanical_phase, gHallCalibration.inverted != 0U);
        electrical_phase = (u16)(electrical_phase +
            (u16)gHallCalibration.electrical_offset);

        if(!s_hallElectricalPhaseValid)
        {
            s_hallElectricalPhaseLast = electrical_phase;
            s_hallElectricalPhaseValid = true;
            s_hallSpeedStepQ16 = 0L;
            motor->m_pll_speed = 0;
            angle_updated = true;
        }
        else
        {
            phase_step = (s32)(s16)(electrical_phase -
                                    s_hallElectricalPhaseLast);
            s_hallElectricalPhaseLast = electrical_phase;

            /*
             * 霍尔给出绝对角度，控制角无需再经过无感PLL积分。
             * 只对相邻绝对角的差值测速；异常跳点不更新角度和速度。
             */
            if(Hall_AbsS32(phase_step) <= HALL_RUNTIME_MAX_PHASE_STEP)
            {
                speed_step_target_q16 = phase_step * 65536L;
                s_hallSpeedStepQ16 +=
                    (speed_step_target_q16 - s_hallSpeedStepQ16) /
                    HALL_RUNTIME_SPEED_FILTER_DIV;
                speed_erpm = s_hallSpeedStepQ16 /
                             HALL_RUNTIME_STEP_Q16_PER_ERPM;
                motor->m_pll_speed = (s16)speed_erpm;
                angle_updated = true;
            }
        }

        if(angle_updated)
        {
            pll->pll_phase_q16 = (u32)electrical_phase << 16;
            pll->pll_speed_step_q16 = s_hallSpeedStepQ16;
            pll->pll_initialized = true;
            motor->m_phase_control_q16 = pll->pll_phase_q16;
            motor->m_phase_control_initialized = true;
        }

        gHallRawA = hall_a;
        gHallRawB = hall_b;
        gHallNormX = hall_x;
        gHallNormY = hall_y;
        gHallMechanicalPhase = (s16)mechanical_phase;
        gHallElectricalPhase = (s16)electrical_phase;
    }
    if(!angle_updated && motor->m_phase_control_initialized)
    {
        motor->m_phase_control_q16 += (u32)(
            pll->pll_speed_step_q16 / (s32)HALL_RUNTIME_ANGLE_DIV);
    }

    if(motor->m_phase_control_initialized)
    {
        motor->m_pll_phase = (s16)(motor->m_phase_control_q16 >> 16);
        state->phase = motor->m_pll_phase;
        gHallControlPhase = state->phase;
        gHallPhaseError = (s16)((u16)gHallElectricalPhase -
                                (u16)gHallControlPhase);
    }
}

void Hall_LearnInit(void)
{
    s_hallSampleA = 0;
    s_hallSampleB = 0;
    s_hallReferencePhase = 0;
    s_hallRuntimeCounter = 0U;
    s_hallRuntimeActive = false;

    gMotorWorkMode = MCS_POWER_ON_WORK_MODE;

    if((gMotorWorkMode == MCS_WORK_MODE_CONTROL) &&
       (MCS_CONTROL_SENSOR_MODE == FOC_SENSOR_MODE_SENSORLESS))
    {
        gHallLearnRequest = 0U;
        gHallLearnState = HALL_LEARN_STATE_IDLE;
        gHallLearnError = HALL_LEARN_ERROR_NONE;
        gHallStorageState = HALL_STORAGE_STATE_IDLE;
        if(m_motor.m_conf != 0)
        {
            m_motor.m_conf->foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
        }
        return;
    }

    if((gMotorWorkMode == MCS_WORK_MODE_CONTROL) &&
       (MCS_CONTROL_SENSOR_MODE == FOC_SENSOR_MODE_HALL) &&
       Hall_LoadCalibration())
    {
        gHallLearnRequest = 0U;
        gHallLearnState = HALL_LEARN_STATE_COMPLETE;
        gHallLearnError = HALL_LEARN_ERROR_NONE;
        gHallLearnStableMs = 0U;
        gHallLearnMechanicalTurns = 0U;
        gHallLearnSampleCount = 0UL;
        gHallStorageState = HALL_STORAGE_STATE_LOADED;
        if(m_motor.m_conf != 0)
        {
            m_motor.m_conf->foc_sensor_mode = FOC_SENSOR_MODE_HALL;
        }
        return;
    }

    /*
     * 宏指定LEARN，或霍尔控制所需的Flash参数无效时，进入无感学习。
     * 学习旋转由公共速度环控制，不再维护单独的学习电流。
     */
    gMotorWorkMode = MCS_WORK_MODE_LEARN;
    if(m_motor.m_conf != 0)
    {
        m_motor.m_conf->foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
    }
    gHallLearnRequest = 1U;
    Hall_ResetLearning();
}

void Hall_LearnRequest(void)
{
    /* 运行中切换角度源不安全，重新学习请求只能在电机关闭时发出。 */
    if(m_motor.m_run_state != MOTOR_RUN_STATE_OFF)
    {
        return;
    }

    __disable_irq();
    gMotorWorkMode = MCS_WORK_MODE_LEARN;
    if(m_motor.m_conf != 0)
    {
        m_motor.m_conf->foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
    }
    gHallLearnRequest = 1U;
    Hall_ResetLearning();
    __enable_irq();
}

void Hall_LearnCancel(void)
{
    gHallLearnRequest = 0U;
    gHallLearnState = HALL_LEARN_STATE_IDLE;
    gHallLearnStableMs = 0U;
}

void Hall_CaptureSample(s16 hall_a, s16 hall_b, s16 reference_phase)
{
    s_hallSampleA = hall_a;
    s_hallSampleB = hall_b;
    s_hallReferencePhase = reference_phase;
}

void Hall_LearnTask1ms(u16 elapsed_ms)
{
    s16 hall_a;
    s16 hall_b;
    s16 reference_phase;
    s32 speed_abs;
    s32 hall_a_normalized;
    s32 hall_b_normalized;
    s32 hall_x_raw;
    s32 hall_y_raw;
    s8 direction_now;

    if(elapsed_ms == 0U)
    {
        return;
    }

    /* 控制模式的角度在ADC中断更新，不再执行学习任务。 */
    if(gMotorWorkMode == MCS_WORK_MODE_CONTROL)
    {
        return;
    }

    /* 读取一个由 ADC 中断同时发布的霍尔/参考角快照。 */
    __disable_irq();
    hall_a = s_hallSampleA;
    hall_b = s_hallSampleB;
    reference_phase = s_hallReferencePhase;
    __enable_irq();

    gHallRawA = hall_a;
    gHallRawB = hall_b;

    if(gHallCalibration.valid != 0U)
    {
        if(gHallStorageState == HALL_STORAGE_STATE_WAIT_STOP)
        {
            /* 防止保存完成前被新的串口运行命令重新启动。 */
            gMotorCommand.run = 0U;
            if((m_motor.m_control_mode == CONTROL_MODE_NONE) &&
               (m_motor.m_run_state == MOTOR_RUN_STATE_OFF))
            {
                if(Hall_SaveCalibration())
                {
                    gHallStorageState = HALL_STORAGE_STATE_SAVED;
                    gMotorWorkMode = MCS_WORK_MODE_CONTROL;
                    m_motor.m_conf->foc_sensor_mode =
                        MCS_CONTROL_SENSOR_MODE;
                    s_hallRuntimeActive = false;
                    s_hallRuntimeCounter = 0U;
                }
                else
                {
                    gHallStorageState = HALL_STORAGE_STATE_FAILED;
                    gHallLearnState = HALL_LEARN_STATE_FAILED;
                    gHallLearnError = HALL_LEARN_ERROR_FLASH;
                    gMotorWorkMode = MCS_WORK_MODE_LEARN;
                    m_motor.m_conf->foc_sensor_mode =
                        FOC_SENSOR_MODE_SENSORLESS;
                }
            }
        }
        return;
    }

    if(gHallLearnRequest == 0U)
    {
        return;
    }

    if(gHallLearnState == HALL_LEARN_STATE_ALIGN_STOP)
    {
        if((m_motor.m_control_mode == CONTROL_MODE_NONE) &&
           (m_motor.m_run_state == MOTOR_RUN_STATE_OFF))
        {
            Hall_ResetAlignmentAverage();
            s_hallAlignPhaseValid = false;
            gHallLearnState = HALL_LEARN_STATE_ALIGN;
        }
        return;
    }

    if(gHallLearnState == HALL_LEARN_STATE_ALIGN)
    {
        Hall_ProcessAlignment(hall_a, hall_b);
        return;
    }

    speed_abs = Hall_AbsS32((s32)m_motor.m_pll_speed);
    if((m_motor.m_run_state != MOTOR_RUN_STATE_RUNNING) ||
       (m_motor.m_conf == 0) ||
       (m_motor.m_conf->foc_sensor_mode != FOC_SENSOR_MODE_SENSORLESS) ||
       (speed_abs < HALL_LEARN_MIN_ERPM))
    {
        if((gHallLearnState == HALL_LEARN_STATE_RAW) ||
           (gHallLearnState == HALL_LEARN_STATE_ORTHOGONAL) ||
           (gHallLearnState == HALL_LEARN_STATE_OFFSET))
        {
            Hall_ResetLearning();
        }
        gHallLearnStableMs = 0U;
        return;
    }

    direction_now = (m_motor.m_pll_speed >= 0) ? 1 : -1;
    if(gHallLearnState == HALL_LEARN_STATE_WAIT_STABLE)
    {
        if(gHallLearnStableMs < HALL_LEARN_STABLE_TIME_MS)
        {
            gHallLearnStableMs += elapsed_ms;
            return;
        }

        s_hallLearnDirection = direction_now;
        Hall_ResetStage(HALL_LEARN_STATE_RAW);
    }
    else if(direction_now != s_hallLearnDirection)
    {
        Hall_ResetLearning();
        return;
    }

    Hall_UpdateTravel((u16)reference_phase);
    gHallLearnSampleCount++;
    if(gHallLearnStageElapsedMs <
       (u16)(65535U - elapsed_ms))
    {
        gHallLearnStageElapsedMs += elapsed_ms;
    }
    else
    {
        gHallLearnStageElapsedMs = 65535U;
    }

    switch((hall_learn_state_t)gHallLearnState)
    {
    case HALL_LEARN_STATE_RAW:
        Hall_UpdateMinMax(hall_a, hall_b);
        if(Hall_MinMaxConverged(HALL_LEARN_RAW_EDGE_TOLERANCE))
        {
            Hall_CompleteRawStage();
        }
        break;

    case HALL_LEARN_STATE_ORTHOGONAL:
        hall_a_normalized = Hall_Normalize(
            hall_a, gHallCalibration.center_a, gHallCalibration.gain_a_q14);
        hall_b_normalized = Hall_Normalize(
            hall_b, gHallCalibration.center_b, gHallCalibration.gain_b_q14);
        hall_x_raw = hall_a_normalized - hall_b_normalized;
        hall_y_raw = hall_a_normalized + hall_b_normalized;
        Hall_UpdateMinMax(hall_x_raw, hall_y_raw);
        if(Hall_MinMaxConverged(HALL_LEARN_ORTHO_EDGE_TOLERANCE))
        {
            Hall_CompleteOrthogonalStage();
        }
        break;

    case HALL_LEARN_STATE_OFFSET:
        Hall_AccumulateOffset(hall_a, hall_b, reference_phase);
        if(Hall_OffsetConverged())
        {
            Hall_PrepareAlignment();
        }
        break;

    default:
        break;
    }
}

bool Hall_CalibrationIsValid(void)
{
    return gHallCalibration.valid != 0U;
}

bool Hall_GetElectricalPhase(s16 *phase)
{
    if((phase == 0) || (gHallCalibration.valid == 0U))
    {
        return false;
    }

    *phase = gHallElectricalPhase;
    return true;
}
