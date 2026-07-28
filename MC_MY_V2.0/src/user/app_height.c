#include "main.h"

#define APP_POSITION_FLASH_MAGIC              (0x504F5331UL)
#define APP_POSITION_FLASH_VERSION            (1U)
#define APP_POSITION_FLASH_ERASE_KEY          (0x9A0D361FUL)
#define APP_POSITION_FLASH_PROGRAM_KEY        (0x9AFDA40CUL)

#define APP_POSITION_PHASE_PER_TURN            (65536L)
#define APP_POSITION_PHASE_PER_SCREW_TURN      \
    (APP_POSITION_PHASE_PER_TURN * APP_POSITION_REDUCTION_RATIO)
     
#define APP_POSITION_TRAJECTORY_HZ              (100L)  // 更新周期 10ms
#define APP_POSITION_ACCEL_01MM_S2              (80L)   // 加速度 
#define APP_POSITION_TRACK_KP_ERPM_PER_01MM     (672L)  // 位置跟踪比例系数
#define APP_POSITION_TOLERANCE_01MM            (1L)     // 位置误差限位

#define APP_POSITION_SAVE_VOLTAGE_MV           (20000UL) // 掉电保护
#define APP_POSITION_SAVE_ARM_VOLTAGE_MV       (21000UL) // 高于此电压认为已经上电

typedef struct
{
    u32 magic;
    u16 version;
    u16 size;
    s16 position_01mm;
    u16 reserved;
    u32 crc32;
} app_position_flash_record_t;


/* 应用层命令、位置、轨迹和存储状态统一从这个对象观察。 */
volatile APP_HEIGHT_CONTROL gAppHeight;

extern volatile u32 gBusVoltageMv;

/* 仅供本文件使用的算法历史量，不作为应用接口或Keil命令入口。 */
typedef struct
{
    s16 position_base_01mm;
    u16 phase_last;
    bool phase_initialized;
    bool power_save_armed;
    bool save_pending;
    s32 ramp_position_q16;
    s32 ramp_speed_q16;
} app_height_runtime_t;

static app_height_runtime_t s_appHeightRuntime;

static void AppHeight_UpdateTargetControl(void);

static void AppHeight_UpdateTrajectoryMonitor(void)
{
    gAppHeight.trajectory.ramp_01mm = (s16)(s_appHeightRuntime.ramp_position_q16 / 65536L);
    gAppHeight.trajectory.ramp_speed_01mm_s = (s16)(s_appHeightRuntime.ramp_speed_q16 / 65536L);
}

static void AppHeight_ResetTrajectory(void)
{
    s32 ramp_speed_q16;

    s_appHeightRuntime.ramp_position_q16 = (s32)gAppHeight.position.current_01mm * 65536L;
    ramp_speed_q16 = (s32)(((int64_t)m_motor.m_pll_speed * 65536LL) /
        APP_POSITION_ERPM_PER_01MM_S);
    s_appHeightRuntime.ramp_speed_q16 = McsMath_LimitS32(
        ramp_speed_q16,
        -(APP_POSITION_TRAVEL_SPEED_01MM_S * 65536L),
        APP_POSITION_TRAVEL_SPEED_01MM_S * 65536L);
    AppHeight_UpdateTrajectoryMonitor();
}

/*
 * 100 Hz梯形行程轨迹。目标是最终行程，内部参考按限定线速度和加速度移动；
 * 根据v^2/(2a)计算制动距离，在到达最终位置前先把轨迹速度降到零。
 */
static void AppHeight_UpdateTrajectory(void)
{
    int64_t speed_square;
    s32 target_q16;
    s32 remaining_q16;
    s32 remaining_abs_q16;
    s32 speed_abs_q16;
    s32 stopping_distance_q16;
    s32 braking_guard_q16;
    s32 desired_speed_q16;
    s32 speed_step_q16;
    s32 position_next_q16;
    
    // 最终目标
    target_q16 = (s32)gAppHeight.position.target_01mm * 65536L;
    // 剩余距离
    remaining_q16 = target_q16 - s_appHeightRuntime.ramp_position_q16;
    remaining_abs_q16 = McsMath_AbsS32(remaining_q16);
    
    speed_abs_q16 = McsMath_AbsS32(s_appHeightRuntime.ramp_speed_q16);
    
    // 计算制动距离，提前多久减速
    speed_square = (int64_t)speed_abs_q16 * speed_abs_q16;
    stopping_distance_q16 = (s32)(speed_square /
        (2LL * APP_POSITION_ACCEL_01MM_S2 * 65536LL));
    // 理论刹车距离+周期移动距离+位置误差余量
    braking_guard_q16 = stopping_distance_q16 +
        speed_abs_q16 / APP_POSITION_TRAJECTORY_HZ +
        APP_POSITION_TOLERANCE_01MM * 65536L;
    
    /* 如果 已经到了误差限位，停止；
     * 如果速度跟误差反向，或者剩余距离已经到了braking_guard；
     * 否则全速运行
     */ 
    if(remaining_abs_q16 <= (APP_POSITION_TOLERANCE_01MM * 65536L))
    {
        desired_speed_q16 = 0L;
    }
    else if(((remaining_q16 > 0L) && (s_appHeightRuntime.ramp_speed_q16 < 0L)) ||
            ((remaining_q16 < 0L) && (s_appHeightRuntime.ramp_speed_q16 > 0L)) ||
            (remaining_abs_q16 <= braking_guard_q16))
    {
        desired_speed_q16 = 0L;
    }
    else
    {
        desired_speed_q16 =
            (remaining_q16 > 0L) ?
            (APP_POSITION_TRAVEL_SPEED_01MM_S * 65536L) :
            -(APP_POSITION_TRAVEL_SPEED_01MM_S * 65536L);
    }
    // 加速度限制
    speed_step_q16 =
        (APP_POSITION_ACCEL_01MM_S2 * 65536L) /
        APP_POSITION_TRAJECTORY_HZ;
    
    s_appHeightRuntime.ramp_speed_q16 = McsMath_StepTowardsS32(
        s_appHeightRuntime.ramp_speed_q16, desired_speed_q16, speed_step_q16);
    // s_k+1 = s_k + vt;
    position_next_q16 = s_appHeightRuntime.ramp_position_q16 +
        s_appHeightRuntime.ramp_speed_q16 / APP_POSITION_TRAJECTORY_HZ;
    
    if((remaining_abs_q16 <=
        (APP_POSITION_TOLERANCE_01MM * 65536L)) &&
       (s_appHeightRuntime.ramp_speed_q16 == 0L))
    {
        position_next_q16 = target_q16;
    }
    else if(((remaining_q16 > 0L) && (position_next_q16 >= target_q16)) ||
       ((remaining_q16 < 0L) && (position_next_q16 <= target_q16)))
    {
        position_next_q16 = target_q16;
        s_appHeightRuntime.ramp_speed_q16 = 0L;
    }
    s_appHeightRuntime.ramp_position_q16 = position_next_q16;
    AppHeight_UpdateTrajectoryMonitor();
}

static void AppHeight_ClearBytes(void *address, u32 size)
{
    u8 *data;
    u32 index;

    data = (u8 *)address;
    for(index = 0UL; index < size; index++)
    {
        data[index] = 0U;
    }
}

static u32 AppHeight_Crc32(const u8 *data, u32 size)
{
    u32 crc;
    u32 index;
    u8 bit;

    crc = 0xFFFFFFFFUL;
    for(index = 0UL; index < size; index++)
    {
        crc ^= (u32)data[index];
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

static bool AppHeight_RecordIsValid(
    const app_position_flash_record_t *record)
{
    u32 crc;

    if((record->magic != APP_POSITION_FLASH_MAGIC) ||
       (record->version != APP_POSITION_FLASH_VERSION) ||
       (record->size != sizeof(*record)) ||
       (record->position_01mm < APP_POSITION_MIN_01MM) ||
       (record->position_01mm > APP_POSITION_MAX_01MM))
    {
        return false;
    }

    crc = AppHeight_Crc32(
        (const u8 *)record,
        (u32)offsetof(app_position_flash_record_t, crc32));
    return crc == record->crc32;
}

static bool AppHeight_LoadPosition(s16 *position_01mm)
{
    app_position_flash_record_t record;

    AppHeight_ClearBytes(&record, sizeof(record));
    Read_Flash(APP_FLASH_Sector_StartAddr,
               (u32 *)&record,
               (sizeof(record) + 3U) >> 2,
               MEMORY_TYPE);
    if(!AppHeight_RecordIsValid(&record))
    {
        return false;
    }

    *position_01mm = record.position_01mm;
    return true;
}

static bool AppHeight_SavePosition(s16 position_01mm)
{
    app_position_flash_record_t record;
    app_position_flash_record_t verify;
    int program_result;

    AppHeight_ClearBytes(&record, sizeof(record));
    AppHeight_ClearBytes(&verify, sizeof(verify));
    record.magic = APP_POSITION_FLASH_MAGIC;
    record.version = APP_POSITION_FLASH_VERSION;
    record.size = (u16)sizeof(record);
    record.position_01mm = position_01mm;
    record.crc32 = AppHeight_Crc32(
        (const u8 *)&record,
        (u32)offsetof(app_position_flash_record_t, crc32));

    /* Flash擦写期间关闭中断，且调用前必须已经关闭PWM。 */
    __disable_irq();
    EraseSector(APP_FLASH_Sector_StartAddr,
                MEMORY_TYPE,
                APP_POSITION_FLASH_ERASE_KEY);
    program_result = ProgramPage(APP_FLASH_Sector_StartAddr,
                                 sizeof(record),
                                 (u8 *)&record,
                                 MEMORY_TYPE,
                                 APP_POSITION_FLASH_PROGRAM_KEY);
    Read_Flash(APP_FLASH_Sector_StartAddr,
               (u32 *)&verify,
               (sizeof(verify) + 3U) >> 2,
               MEMORY_TYPE);
    __enable_irq();

    return (program_result != 0) && AppHeight_RecordIsValid(&verify) &&
           (verify.position_01mm == position_01mm);
}

static bool AppHeight_PositionFeedbackReady(void)
{
    return (gMotorWorkMode == MCS_WORK_MODE_CONTROL) &&
           Hall_CalibrationIsValid() &&
           (m_motor.m_conf != 0) &&
           (m_motor.m_conf->foc_sensor_mode == FOC_SENSOR_MODE_HALL);
}

// 电机转了多少角度，换算移动了多少mm
static void AppHeight_UpdatePosition(void)
{
    int64_t travel_numerator;
    s32 position_delta;
    s32 phase_delta;
    u16 phase_now;

    if(!AppHeight_PositionFeedbackReady())
    {
        s_appHeightRuntime.phase_initialized = false;
        return;
    }

    phase_now = (u16)gHallMechanicalPhase;
    if(!s_appHeightRuntime.phase_initialized)
    {
        s_appHeightRuntime.phase_last = phase_now;
        s_appHeightRuntime.phase_initialized = true;
        return;
    }

    phase_delta = (s32)(s16)(phase_now - s_appHeightRuntime.phase_last);
    s_appHeightRuntime.phase_last = phase_now;

    /*
     * Hall标定若选择反向角度，正ERPM对应原始机械角减小。
     * 这里统一成正方向增加行程，保证位置环和速度环方向一致。
     */
    if(gHallCalibration.inverted != 0U)
    {
        phase_delta = -phase_delta;
    }

    gAppHeight.position.cumulative_phase += phase_delta;
    travel_numerator =
        (int64_t)gAppHeight.position.cumulative_phase *
        (int64_t)APP_POSITION_SCREW_LEAD_01MM;
    if(travel_numerator >= 0)
    {
        travel_numerator += APP_POSITION_PHASE_PER_SCREW_TURN / 2L;
    }
    else
    {
        travel_numerator -= APP_POSITION_PHASE_PER_SCREW_TURN / 2L;
    }

    position_delta = (s32)(
        travel_numerator / APP_POSITION_PHASE_PER_SCREW_TURN);
    gAppHeight.position.current_01mm = (s16)McsMath_LimitS32(
        (s32)s_appHeightRuntime.position_base_01mm + position_delta, -32768L, 32767L);
}

static void AppHeight_StopCommand(void)
{
    gAppHeight.command.run = 0U;
    gAppHeight.trajectory.speed_request_erpm = 0;
}

/*
 * 普通运行的独立软件限位。该保护每1 ms执行一次，不依赖10 ms位置状态机；
 * 复位模式需要寻找机械端点，因此由后续复位状态机单独管理边界。
 */
static void AppHeight_ApplyTravelLimit(void)
{
    if((gAppHeight.position.mode == APP_POSITION_MODE_RESET) ||
       (gAppHeight.position.valid == 0U) ||
       (gAppHeight.command.run == 0U))
    {
        return;
    }

    if(((gAppHeight.command.direction == MCS_MOTOR_DIRECTION_FORWARD) &&
        (gAppHeight.position.current_01mm >= APP_POSITION_MAX_01MM)) ||
       ((gAppHeight.command.direction == MCS_MOTOR_DIRECTION_REVERSE) &&
        (gAppHeight.position.current_01mm <= APP_POSITION_MIN_01MM)))
    {
        AppHeight_StopCommand();
        gAppHeight.position.mode = APP_POSITION_MODE_IDLE;
    }
}

static void AppHeight_UpdatePowerSave(void)
{
    s16 position_to_save;

    if(s_appHeightRuntime.save_pending)
    {
        AppHeight_StopCommand();
        gAppHeight.position.mode = APP_POSITION_MODE_IDLE;
        if(Motor_IsPwmEnabled())
        {
            return;
        }

        position_to_save = (s16)McsMath_LimitS32(
            (s32)gAppHeight.position.current_01mm,
            APP_POSITION_MIN_01MM,
            APP_POSITION_MAX_01MM);
        if(AppHeight_SavePosition(position_to_save))
        {
            gAppHeight.storage.saved_01mm = position_to_save;
            gAppHeight.storage.storage_state = APP_POSITION_STORAGE_SAVED;
        }
        else
        {
            gAppHeight.storage.storage_state = APP_POSITION_STORAGE_FAILED;
        }
        s_appHeightRuntime.save_pending = false;
        return;
    }

    if(gBusVoltageMv >= APP_POSITION_SAVE_ARM_VOLTAGE_MV)
    {
        s_appHeightRuntime.power_save_armed = true;
        return;
    }

    if((gBusVoltageMv >= APP_POSITION_SAVE_VOLTAGE_MV) ||
       !s_appHeightRuntime.power_save_armed)
    {
        return;
    }

    s_appHeightRuntime.power_save_armed = false;
    if((gAppHeight.position.valid != 0U) &&
       (gAppHeight.position.current_01mm != gAppHeight.storage.saved_01mm))
    {
        s_appHeightRuntime.save_pending = true;
        gAppHeight.storage.storage_state = APP_POSITION_STORAGE_PENDING;
        AppHeight_StopCommand();
        gAppHeight.position.mode = APP_POSITION_MODE_IDLE;
    }
}

static void AppHeight_UpdateManualControl(void)
{
    /*
     * 手动命令没有指定目标行程，但仍以对应机械端点作为内部位置目标。
     * 这样位置外环会在接近0/1100时逐步降低速度，而不是全速撞到限位才停。
     */
    if((gAppHeight.position.valid == 0U) ||
       !AppHeight_PositionFeedbackReady())
    {
        AppHeight_StopCommand();
        gAppHeight.position.mode = APP_POSITION_MODE_IDLE;
        return;
    }

    gAppHeight.position.target_01mm =
        (gAppHeight.command.direction == MCS_MOTOR_DIRECTION_FORWARD) ?
        APP_POSITION_MAX_01MM : APP_POSITION_MIN_01MM;
    AppHeight_UpdateTargetControl();
}

// 位置控制核心
static void AppHeight_UpdateTargetControl(void)
{
    int64_t speed_feedforward;
    int64_t position_correction;
    s32 error;
    s32 tracking_error_q16;
    s32 speed_signed;
    // 位置是否有效，霍尔反馈是否有效
    if((gAppHeight.position.valid == 0U) ||
       !AppHeight_PositionFeedbackReady())
    {
        AppHeight_StopCommand();
        gAppHeight.position.mode = APP_POSITION_MODE_IDLE;
        return;
    }

    error = (s32)gAppHeight.position.target_01mm -
            (s32)gAppHeight.position.current_01mm;
    error = McsMath_LimitS32(error, -32768L, 32767L);
    gAppHeight.position.error_01mm = (s16)error;
    
    /* 误差小于某个限位；
     * 已经达到目标；
     * 速度为0
     */ 
    if((McsMath_AbsS32(error) <= APP_POSITION_TOLERANCE_01MM) &&
       (s_appHeightRuntime.ramp_position_q16 ==
        (s32)gAppHeight.position.target_01mm * 65536L) &&
       (s_appHeightRuntime.ramp_speed_q16 == 0L))
    {
        AppHeight_StopCommand();
        return;
    }
    // 更新轨迹，得到期望位置s_appHeightRuntime.ramp_position_q16，梯度目标位置
    AppHeight_UpdateTrajectory();
    // 梯度误差
    tracking_error_q16 = s_appHeightRuntime.ramp_position_q16 -
        (s32)gAppHeight.position.current_01mm * 65536L;
    // 速度前馈
    speed_feedforward =
        (int64_t)s_appHeightRuntime.ramp_speed_q16 * APP_POSITION_ERPM_PER_01MM_S;
    // 位置误差补偿
    position_correction =
        (int64_t)tracking_error_q16 *
        APP_POSITION_TRACK_KP_ERPM_PER_01MM;
    
    speed_signed = (s32)(
        (speed_feedforward + position_correction) / 65536LL);
    
    speed_signed = McsMath_LimitS32(
        speed_signed,
        -APP_POSITION_MAX_SPEED_ERPM,
        APP_POSITION_MAX_SPEED_ERPM);
    
    /* 速度请求本身带符号，方向字段只保存给应用层观察和限位判断。 */
    if(speed_signed > 0L)
    {
        gAppHeight.command.direction = MCS_MOTOR_DIRECTION_FORWARD;
    }
    else if(speed_signed < 0L)
    {
        gAppHeight.command.direction = MCS_MOTOR_DIRECTION_REVERSE;
    }

    /* 位置环只发布带符号ERPM；速度环再把速度误差变成q轴电流。 */
    gAppHeight.trajectory.speed_request_erpm = (s16)speed_signed;
    gAppHeight.command.run = 1U;
}

void AppHeight_Init(void)
{
    s16 stored_position;

    gAppHeight.position.current_01mm = 0;
    gAppHeight.position.target_01mm = 0;
    gAppHeight.position.error_01mm = 0;
    gAppHeight.trajectory.speed_request_erpm = 0;
    gAppHeight.trajectory.ramp_01mm = 0;
    gAppHeight.trajectory.ramp_speed_01mm_s = 0;
    gAppHeight.storage.saved_01mm = -1;
    gAppHeight.position.cumulative_phase = 0L;
    gAppHeight.position.valid = 0U;
    gAppHeight.position.mode = APP_POSITION_MODE_IDLE;
    gAppHeight.storage.storage_state = APP_POSITION_STORAGE_EMPTY;
    gAppHeight.command.command = Motor_Stop;
    gAppHeight.command.run = 0U;
    gAppHeight.command.direction = MCS_MOTOR_DIRECTION_DEFAULT;
    gAppHeight.command.serial_target_01mm = APP_POSITION_MANUAL_TARGET;
    s_appHeightRuntime.position_base_01mm = 0;
    s_appHeightRuntime.phase_last = 0U;
    s_appHeightRuntime.phase_initialized = false;
    s_appHeightRuntime.power_save_armed = false;
    s_appHeightRuntime.save_pending = false;

    if(AppHeight_LoadPosition(&stored_position))
    {
        gAppHeight.position.current_01mm = stored_position;
        gAppHeight.position.target_01mm = stored_position;
        gAppHeight.storage.saved_01mm = stored_position;
        s_appHeightRuntime.position_base_01mm = stored_position;
        gAppHeight.position.valid = 1U;
        gAppHeight.storage.storage_state = APP_POSITION_STORAGE_LOADED;
    }
    else
    {
        /*
         * 复位状态机尚未接入时，无有效Flash记录暂以机械下限作为位置原点。
         * 这允许首次上电向上运行，同时普通向下命令仍会被0行程限位挡住。
         */
        gAppHeight.position.current_01mm = APP_POSITION_MIN_01MM;
        gAppHeight.position.target_01mm = APP_POSITION_MIN_01MM;
        s_appHeightRuntime.position_base_01mm = APP_POSITION_MIN_01MM;
        gAppHeight.position.cumulative_phase = 0L;
        gAppHeight.position.valid = 1U;
    }
    AppHeight_ResetTrajectory();
}

void AppHeight_Task1ms(u16 elapsed_ms)
{
    if(elapsed_ms == 0U)
    {
        return;
    }

    AppHeight_UpdatePosition();
    AppHeight_ApplyTravelLimit();
    AppHeight_UpdatePowerSave();
}

void AppHeight_Task10ms(void)
{
    if(Motor_FaultIsActive() ||
       (gMotorWorkMode != MCS_WORK_MODE_CONTROL))
    {
        AppHeight_StopCommand();
        gAppHeight.position.mode = APP_POSITION_MODE_IDLE;
        return;
    }

    switch((APP_POSITION_MODE)gAppHeight.position.mode)
    {
    case APP_POSITION_MODE_MANUAL:
        AppHeight_UpdateManualControl();
        break;

    case APP_POSITION_MODE_TARGET:
        AppHeight_UpdateTargetControl();
        break;

    case APP_POSITION_MODE_RESET:
        /* 堵转电流和低速复位后续在这里接入。 */
        AppHeight_StopCommand();
        break;

    case APP_POSITION_MODE_IDLE:
    default:
        break;
    }
}

void AppHeight_HandleCommand(u8 command, u16 target_01mm)
{
    /* 串口层只在命令变化时调用；这里保存完整应用命令快照。 */
    gAppHeight.command.command = command;
    gAppHeight.command.serial_target_01mm = target_01mm;

    switch(command)
    {
    case Motor_Up:
    case Motor_Down:
        if(target_01mm == APP_POSITION_MANUAL_TARGET)
        {
            gAppHeight.position.mode = APP_POSITION_MODE_MANUAL;
            gAppHeight.command.direction =
                (command == Motor_Up) ?
                MCS_MOTOR_DIRECTION_FORWARD :
                MCS_MOTOR_DIRECTION_REVERSE;
            AppHeight_ResetTrajectory();
            AppHeight_UpdateManualControl();
        }
        else
        {
            gAppHeight.position.target_01mm = (s16)McsMath_LimitS32(
                (s32)target_01mm,
                APP_POSITION_MIN_01MM,
                APP_POSITION_MAX_01MM);
            gAppHeight.position.mode = APP_POSITION_MODE_TARGET;
            AppHeight_ResetTrajectory();
            AppHeight_UpdateTargetControl();
        }
        break;

    case Motor_Reset:
        gAppHeight.position.mode = APP_POSITION_MODE_RESET;
        AppHeight_StopCommand();
        break;

    case Motor_Stop:
    default:
        gAppHeight.position.mode = APP_POSITION_MODE_IDLE;
        gAppHeight.position.error_01mm = 0;
        AppHeight_StopCommand();
        break;
    }
}

void AppHeight_SetCurrent(s16 position_01mm)
{
    position_01mm = (s16)McsMath_LimitS32(
        (s32)position_01mm,
        APP_POSITION_MIN_01MM,
        APP_POSITION_MAX_01MM);
    s_appHeightRuntime.position_base_01mm = position_01mm;
    gAppHeight.position.current_01mm = position_01mm;
    gAppHeight.position.target_01mm = position_01mm;
    gAppHeight.position.cumulative_phase = 0L;
    gAppHeight.position.valid = 1U;
    s_appHeightRuntime.phase_initialized = false;
    AppHeight_ResetTrajectory();
}

int AppHeight_GetCurrent(void)
{
    return (int)gAppHeight.position.current_01mm;
}
