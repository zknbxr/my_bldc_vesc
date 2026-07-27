#ifndef APP_HEIGHT_H
#define APP_HEIGHT_H

#include <stdint.h>

/* 推杆行程统一使用0.1 mm，机械位置范围为0...1100。 */
#define APP_POSITION_MIN_01MM              (0)
#define APP_POSITION_MAX_01MM              (1100)
#define APP_POSITION_MANUAL_TARGET         (0xFFFFU)

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

typedef struct
{
    int16_t current_01mm;
    int16_t target_01mm;
    int16_t error_01mm;
    int16_t speed_command_erpm;
    int16_t ramp_01mm;
    int16_t ramp_speed_01mm_s;
    uint8_t valid;
    uint8_t mode;
    uint8_t storage_state;
} APP_HEIGHT_STATUS;

extern volatile int16_t gAppPositionCurrent01mm;
extern volatile int16_t gAppPositionTarget01mm;
extern volatile int16_t gAppPositionError01mm;
extern volatile int16_t gAppPositionSpeedErpm;
extern volatile int16_t gAppPositionRamp01mm;
extern volatile int16_t gAppPositionRampSpeed01mmS;
extern volatile int16_t gAppPositionSaved01mm;
extern volatile int32_t gAppPositionCumulativePhase;
extern volatile uint8_t gAppPositionValid;
extern volatile uint8_t gAppPositionMode;
extern volatile uint8_t gAppPositionStorageState;

void AppHeight_Init(void);
void AppHeight_Task1ms(uint16_t elapsed_ms);
void AppHeight_Task10ms(void);
void AppHeight_HandleCommand(uint8_t command, uint16_t target_01mm);
void AppHeight_SetCurrent(int16_t position_01mm);
int AppHeight_GetCurrent(void);
void AppHeight_GetStatus(APP_HEIGHT_STATUS *status);

#endif
