#ifndef TYPE_H
#define TYPE_H

#include "stdint.h"
typedef struct STRUCT_GLOBAL_TIMER_STS{
	bool bMsFired;
	bool bTenMsFired;
	bool bHundredMsFired;
	bool bThreeHundredMsFired;
	bool bSecondFired;

}GLOBAL_TIMER_STR;


typedef struct tagUART{
	uint8_t RecStatus;				//状态
	uint8_t MAX_Len;				// 最大数据长度
	uint8_t T_Index;				// 发送当前位置
	uint8_t R_Index;				// 接收当前位置
	uint8_t RXBuffer[UartMaxLen];
	uint8_t TXBuffer[UartMaxLen];
}UART0_Data;


typedef enum
{
    APP_UART_CMD_NONE = 0,
    APP_UART_CMD_UP,
    APP_UART_CMD_DOWN,
    APP_UART_CMD_STOP,
    APP_UART_CMD_RESET,
    APP_UART_CMD_MEMORY1,
    APP_UART_CMD_MEMORY2,
    APP_UART_CMD_MEMORY1_SAVE,
    APP_UART_CMD_MEMORY2_SAVE,
    APP_UART_CMD_SET_PARAM,
    APP_UART_CMD_UNKNOWN
} APP_UART_CMD_TYPE;

typedef struct
{
    APP_UART_CMD_TYPE type;
    u8 rawCode;
    u8 lastrawCode;
    u8 address;
    u16 targetHeight;
    float targetSpeed;
    u16 paramLeadDistance;
    u16 paramReductionRatio;
    u16 paramMaxTravelDistance;
} APP_UART_COMMAND;

typedef struct
{
    u32 rxFrameOkCount;
    u32 rxFrameErrorCount;
    u32 txReplyCount;
    u32 timeoutCount;
    u8 pending;
    u8 lastRawCode;
} APP_UART_COMM_STATUS;


/* 桌子运动子状态枚举, 由 gDeskMianMbr.MotorState 使用。 */
typedef enum {
	STATE_RAMP_STOP,     /* 停止状态: 电机不再继续运行, 通常也是运动流程的起点/终点。 */
    STATE_RAMP_UP,     /* 加速/起动状态: 电机从停止或低速向目标速度过渡。 */
    STATE_RUN,         /* 正常运行状态: 电机按当前目标高度或目标速度运行。 */
    STATE_RAMP_DOWN    /* 减速/缓停状态: 接近目标高度或限位时逐步降速停机。 */
} ENUM_MotorState;

/* 复位流程子状态枚举, 由 gDeskMianMbr.ResState 使用。 */
typedef enum {
	STATE_RES_DOWN,         /* 复位初始/下行阶段: 作为复位流程的起始状态或回到下行处理。 */
    STATE_RES_BACK_F,      /* 缩回堵转后的回退阶段, 用于先脱离堵转/限位位置。 */
	 STATE_RES_BACK_R,      /* 伸出堵转后的回退阶段, 用于先脱离堵转/限位位置。 */
    STATE_RES_RESET_F,     /* 缩回方向复位阶段, 按复位目标继续运行。 */
	STATE_RES_RESET_R,      /* 伸出方向复位阶段, 按复位目标继续运行。 */
	STATE_RES_TARGETHEIGHT  /* 复位到指定目标高度阶段, 完成后返回正常状态。 */
} ENUM_ResState;
/*
 * 升降桌应用层主控状态结构体。
 * 这个结构体不直接控制 FOC 电流环/速度环, 而是保存用户层的高度目标、行程限位、记忆档位和运行状态。
 * USER_APP.c/manage.c 会根据这些字段决定目标高度、启停和对外上报状态。
 */
typedef struct STRUCT_DESK_MAIN_PAR
{
    /* 当前高度/行程, ConvertHeight() 根据霍尔累计量周期刷新。 */
    int currentHeight;      				/*当前行程*/
    /* 目标高度/行程, 按键、串口命令、记忆位和复位流程都会修改它。 */
	  int targetHeight;      				/*目标行程*/ 
    /* 初始高度/行程, 用于将霍尔累计量换算成绝对高度。 */
	  int initHeight;						/*初始行程*/	
    /* 记忆位 1~6 的目标高度, 上电从 NVM 读取, 修改后会写回 NVM。 */
		u16 Target_gear_01;					/*档位1*/
		u16 Target_gear_02;					/*档位2*/
		u16 Target_gear_03;					/*档位3*/
		u16 Target_gear_04;					/*档位4*/
		u16 Target_gear_05;					/*档位5*/
		u16 Target_gear_06;					/*档位6*/
	
	
	
    /* 最大/最小行程限位, Motor_Update() 和 Motor_Current_Status_Judge() 用它做限位判断。 */
		u16 maxHeightLimit;     		/*最大行程*/ 
		u16 minHeightLimit;     		/*最小行程*/ 
    /* 系统主状态, 实际取值对应 ENUM_SystemStatus, USER_APP_vTick100MS() 按它切换正常/复位/过热/故障/老化等流程。 */
		u8 systemStatus;				/*系统状态*/
    /* 桌子运动子状态, 用于控制加速/运行/减速/停止斜坡。 */
		ENUM_MotorState MotorState;		/*电机状态*/
    /* 复位子状态, HighReset() 和堵转/电压恢复等流程用它控制回退、复位和目标高度处理。 */
		ENUM_ResState		ResState;

} DESK_MAIN_CMD_STR;

typedef enum
{
    E_NORMAL_STATUS = 0,    	/*正常状态*/
    E_OBSTRUCTION_STATUS = 1, /*遇阻状态*/
    E_RETRACT_STATUS = 2,    	/*回退状态*/
    E_OVERLOAD_STATUS = 3,  	/*过载状态*/
    E_RESET_STATUS = 4,      	/*复位状态*/
    E_OVERHEAT_STATUS = 5,    /*过热状态*/
    E_HALLCALIB_STATUS = 6,		/*校准状态*/
	E_ERROR_STATUS = 7,				/*错误状态*/
	E_TEST_STATUS = 8				/*校准状态*/
} ENUM_SystemStatus;

typedef struct
{
    u8  u8NVMFlage;
    u16 Memory_gear_01;
    u16 Memory_gear_02;
    u16 Memory_gear_03;
	u16 Memory_gear_04;
    u16 Memory_gear_05;
    u16 Memory_gear_06;
//    u16 Memory_currentHeight;
	int Memory_currentHeight;

    u16   Calibration_flag;

    uint16_t minTravelDistance;       ///< 最小行程，单位：毫米
    uint16_t maxTravelDistance;       ///< 最大行程，单位：毫米

    u16 leadDistance;            ///< 导程，单位：毫米
    u16 reductionRatio;          ///< 减速比

    u8 pointRunDistance;        ///< 点按运行距离，单位：毫米
    u16 upRetreatDistance;    	///< 上升回退距离，单位：毫米
	u8 downRetreatDistance;    ///< 下降回退距离，单位：毫米
    u8 softStopDistance;        ///< 缓停距离，单位：毫米

    u8 travelSpeed;             ///< 行程速度，单位：毫米/秒
    uint16_t motorSpeed;             ///< 电机速度，单位：转/分钟
    uint16_t resetSpeed;              ///< 复位速度，单位：转/分钟


    u8 resetCurrent;            ///< 复位电流，单位：安培
    u8 maxReverseCurrent;       ///< 最大反向电流，单位：安培
    u8 maxForwardCurrent;       ///< 最大正向电流，单位：安培

	s32 run_distance;						/*一圈运行距离*/
	u16 agi_cnt;
} tsAPP_NVM_Data;

#endif
