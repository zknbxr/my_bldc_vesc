/***************************************************************************************/
/*	注 释：		定义一些常数的助记形式、常量																						*/
/****************************************************************************************/
#define MEMORY_TYPE                       (0x000)                                                 /* 0x800 NVM,  0x000 FLASH */
#define HALL_FLASH_Sector_StartAddr       (0x7C00)                                                /* 	FLASH首地址-学习程序*/
#define APP_FLASH_Sector_StartAddr        (0x7A00)  


/*UART相关常量*/
#define UART_STY				0		 // 空闲
#define UART_REC				1		 // 接收中
#define UART_RECD				2		 // 接收完成
#define UART_TRN				3		 // 发送中
#define UART_TRND				4		 // 发送完成
#define UART_ERR				5		 // 串口异常
#define UartMaxLen 				11 		//串口缓存数组长度
/*UART帧头常量*/
#define Head_RxH 				0xAA
#define Head_RxL 				0xAA
#define Head_TxH 				0x55
#define Head_TxL 				0x55




#define SCREW_LEAD_SPEED								80				//导程速度 	（0.1mm/s）
#define RESSPEED 											  1500			//复位速度	（rad/min）
#define	SCREW_LEAD											1200				//导程（0.1mm）注：导程2.5的填246
#define REDUCTION_RARIO										350				//减速比(比实际大10倍)

#define MIN_TRAVEL_DISTANCE 								0000       		// 最小行程，单位：0.1毫米
#define MAX_TRAVEL_DISTANCE 								650      		// 最大行程，单位：0.1毫米

#define POINT_RUN_DISTANCE 									10         		// 点按运行距离，单位：0.1毫米
#define UP_RETREAT_DISTANCE 								30    			// 上升堵转回退距离，单位：0.1毫米(比实际多1mm)
#define DOWN_RETREAT_DISTANCE 							30    			// 下降堵转回退距离，单位：0.1毫米(比实际多1mm)
#define SOFT_STOP_DISTANCE 									150       		// 缓停距离，单位：0.1毫米
#define SOFT_STOP_DISTANCE_ST 								5        		// 缓停距离，单位：0.1毫米

#define RESET_CURRENT 										6         		// 复位电流，单位：安培
#define MAX_REVERSE_CURRENT 								16       		// 最大反向电流，实际电流：17/1.414A
#define MAX_FORWARD_CURRENT 								16     			// 最大正向电流，实际电流：17/1.414A
#define	Max_OVLOAD_CURRENT									14				// 过流保护电流	单位：10/1.414A

#define RUNINGSPEED 							((SCREW_LEAD_SPEED*60*(REDUCTION_RARIO/10.0))/(SCREW_LEAD/10.0))		//运行速度
#define RUN_DISTANCE							((65535*(REDUCTION_RARIO/10.0)*Pole_Pairs)/(SCREW_LEAD/10.0)			//运行距离

/* 电机变速相关 */
#define Speed_Contorl 			0 				//电机变速使能 （1：可调速度  0：恒定速度 若用控制盒控制电机默认采样恒定速度）
#if(Speed_Contorl)
#define Speed_Target 			1250			//电机变速位置	(单位 0.1mm)
#endif
/* 电机记忆行程相关 */
#define Memory1_Target 			0 				//电机位置1		(单位 0.1mm)
#define Memory2_Target 			0			//电机位置2		(单位 0.1mm)
/* 电机复位行程相关 */
#define Reset_Target 			 0 			//复位后运行到哪个位置	(单位 0.1mm)
/* 电机运行方向相关 */
#define Run_Direction 			1				//运行方向（1：正转为手控器升  0：反转）
/* 有锁或无锁相关 */
#define Height_Lock_Flag 		0				//记忆位置自定义使能（0：有锁不可调  1：无锁可调）
