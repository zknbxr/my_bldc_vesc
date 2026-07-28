#ifndef APP_HEIGHT_H
#define APP_HEIGHT_H

#include <stdint.h>

/* 推杆行程统一使用0.1 mm，机械位置范围为0...1100。 */
#define APP_POSITION_MIN_01MM              (0)
#define APP_POSITION_MAX_01MM              (1100)
#define APP_POSITION_MANUAL_TARGET         (0xFFFFU)

/*
 * JA11推杆参数。应用层根据推杆机械参数产生速度目标，MCS层不再保存一份
 * 与推杆无关的默认速度。
 */
#define APP_POSITION_SCREW_LEAD_01MM       (25L)  /* 丝杆导程2.5 mm */
#define APP_POSITION_REDUCTION_RATIO       (35L)  /* 电机35圈，丝杆1圈 */
#define APP_POSITION_TRAVEL_SPEED_01MM_S   (40L)  /* 额定线速度4.0 mm/s */
#define APP_POSITION_ERPM_PER_01MM_S       \
    ((60L * APP_POSITION_REDUCTION_RATIO * Pole_Pairs) / \
     APP_POSITION_SCREW_LEAD_01MM)
#define APP_POSITION_MAX_SPEED_ERPM        \
    (APP_POSITION_TRAVEL_SPEED_01MM_S * APP_POSITION_ERPM_PER_01MM_S)
#define APP_HALL_LEARN_SPEED_ERPM          (5000)

typedef enum
{
    APP_POSITION_MODE_IDLE = 0,
    APP_POSITION_MODE_MANUAL,
    APP_POSITION_MODE_TARGET,
    APP_POSITION_MODE_RESET
} APP_POSITION_MODE;

typedef enum
{
    APP_POSITION_STORAGE_EMPTY = 0,
    APP_POSITION_STORAGE_LOADED,
    APP_POSITION_STORAGE_PENDING,
    APP_POSITION_STORAGE_SAVED,
    APP_POSITION_STORAGE_FAILED
} APP_POSITION_STORAGE_STATE;

/* 串口命令层：只记录“用户想做什么”，不表示PWM已经开启。 */
typedef struct
{
    uint8_t command;          /* 最近一次有效命令字 */
    uint8_t run;              /* 1=应用请求运行，0=应用请求停止 */
    int8_t direction;         /* MCS_MOTOR_DIRECTION_FORWARD/REVERSE */
    uint16_t serial_target_01mm; /* 串口原始目标；0xFFFF表示手动升降 */
} APP_HEIGHT_COMMAND_STATE;

/* 位置反馈层：当前位置、最终目标和位置误差，单位均为0.1 mm。 */
typedef struct
{
    int16_t current_01mm;
    int16_t target_01mm;
    int16_t error_01mm;
    int32_t cumulative_phase; /* 从位置基准开始累计的机械角，Q16一圈 */
    uint8_t valid;            /* Flash位置或复位位置有效 */
    uint8_t mode;             /* APP_POSITION_MODE */
} APP_HEIGHT_POSITION_STATE;

/* 轨迹与位置环输出：参考位置经速度/加速度限制后生成带符号ERPM。 */
typedef struct
{
    int16_t ramp_01mm;
    int16_t ramp_speed_01mm_s;
    int16_t speed_request_erpm; /* 送给速度环的带符号目标 */
} APP_HEIGHT_TRAJECTORY_STATE;

/* 掉电存储状态独立归组，便于观察Flash保存过程。 */
typedef struct
{
    int16_t saved_01mm;
    uint8_t storage_state;
} APP_HEIGHT_STORAGE;

/*
 * 应用层唯一持久状态。观察命令看command，观察位置看position，
 * 观察位置环输出看trajectory；电机实际状态仍以m_motor为准。
 */
typedef struct
{
    APP_HEIGHT_COMMAND_STATE command;
    APP_HEIGHT_POSITION_STATE position;
    APP_HEIGHT_TRAJECTORY_STATE trajectory;
    APP_HEIGHT_STORAGE storage;
} APP_HEIGHT_CONTROL;

extern volatile APP_HEIGHT_CONTROL gAppHeight;

void AppHeight_Init(void);
void AppHeight_Task1ms(uint16_t elapsed_ms);
void AppHeight_Task10ms(void);
void AppHeight_HandleCommand(uint8_t command, uint16_t target_01mm);
void AppHeight_SetCurrent(int16_t position_01mm);
int AppHeight_GetCurrent(void);

#endif
