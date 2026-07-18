#ifndef MCS_MOTOR_TYPE_H
#define MCS_MOTOR_TYPE_H

#define MAX_TRAVEL_DISTANCE  650

#define MCS_INV_SQRT3_Q13  4730L

/*推杆运行状态相关常量*/
#define Motor_Address									0x00	//电机地址
#define Stop_Status										0x00	//停止状态
#define	Run_Status										0x01	//运行状态
#define	Reach_Maxheightlimit_Status						0x02	//到达最大行程
#define	Reach_Minheightlimit_Status						0x04	//到达最小行程
#define Fault_Status									0x03	//错误状态
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
/* 电机指令相关 */
#define Motor_Up 				0x01 		//电机上升
#define Motor_Down 				0x02 		//电机下降
#define Motor_Stop 				0x03 		//电机停止
#define Motor_Memory1 			0x16 		//电机记忆1
#define Motor_Memory2 			0x17 		//电机记忆2
#define Motor_Memory1Target 	0x18 		//电机记忆位置1
#define Motor_Memory2Target 	0x19 		//电机记忆位置2
#define Motor_Reset 			0x13 		//电机复位
#define Motor_SET 				0x68 		//电机参数修改
/* 电机错误标志相关 */
#define Motor_Normal  				0 		//电机正常
#define Motor_Overheat 				1 		//电机过温
#define Motor_Overcurrent  			2 		//电机过流
#define Motor_Undervoltage  		3 		//电机欠压
#define Motor_Overvoltage  			4 		//电机过压
#define Motor_Defaultphase			5		//电机缺相
#define Motor_Shortcircuit 			6		//电机短路
#define Motor_Uarterror 			7		//电机通讯异常
#define Motor_Lockedrotor 			8		//电机堵转


// 传感器模式枚举
typedef enum {
    FOC_SENSOR_MODE_SENSORLESS = 0,
    FOC_SENSOR_MODE_ENCODER,
    FOC_SENSOR_MODE_HALL,
    FOC_SENSOR_MODE_HFI,
    FOC_SENSOR_MODE_HFI_START,
    FOC_SENSOR_MODE_HFI_V2,
    FOC_SENSOR_MODE_HFI_V3,
    FOC_SENSOR_MODE_HFI_V4,
    FOC_SENSOR_MODE_HFI_V5,
    FOC_SENSOR_MODE_ENCODER_AB
} mc_foc_sensor_mode;

// FOC current controller decoupling mode.
typedef enum {
	FOC_CC_DECOUPLING_DISABLED = 0,
	FOC_CC_DECOUPLING_CROSS,
	FOC_CC_DECOUPLING_BEMF,
	FOC_CC_DECOUPLING_CROSS_BEMF
} mc_foc_cc_decoupling_mode;

typedef enum {
	FOC_CONTROL_SAMPLE_MODE_V0 = 0,
	FOC_CONTROL_SAMPLE_MODE_V0_V7,
	FOC_CONTROL_SAMPLE_MODE_V0_V7_INTERPOL
} mc_foc_control_sample_mode;

typedef enum {
	FOC_SPEED_SRC_CORRECTED = 0,
	FOC_SPEED_SRC_OBSERVER,
} FOC_SPEED_SRC;


typedef struct
{ 
    s16 phase_sin;       /* Q15 */
    s16 phase_cos;       /* Q15 */
    
    s16 iq;              /* current unit, presently mA */
    s16 id;              /* current unit, presently mA */
    s16 id_filter;
    s16 iq_filter;
    
    s16 vd;              /* Q15 normalized modulation */
    s16 vq;              /* Q15 normalized modulation */
    s32 vd_int;          /* Q15 normalized modulation */
    s32 vq_int;          /* Q15 normalized modulation */
    s32 vd_int_residual; /* fractional Q15 current-integrator remainder */
    s32 vq_int_residual; /* fractional Q15 current-integrator remainder */
    s32 id_error;
    s32 iq_error;
    u16 pwm_a;
    u16 pwm_b;
    u16 pwm_c;
    u16 svm_sector;    
    s16 max_duty;        /* Q15 */
    s32 v_bus;            /* filtered DC bus voltage in mV */
    s32 v_alpha;          /* alpha-axis voltage in mV */
    s32 v_beta;           /* beta-axis voltage in mV */
    s16 phase;
    
    s16 mod_alpha_raw;   /* 实际送入 SVM 的 alpha 轴 Q15 调制度 */
    s16 mod_beta_raw;    /* 实际送入 SVM 的 beta 轴 Q15 调制度 */
    s16 mod_d;
    s16 mod_q;
    
    s16 i_alpha;         /* current unit, presently mA */
    s16 i_beta;          /* current unit, presently mA */
    
    s16 duty_now;
    s16 i_abs_filter;
    s16 mod_q_filter;
    s16 id_target;       /* current unit, presently mA */
    s16 iq_target;       /* current unit, presently mA */
    bool id_override_hfi;
} motor_state_t;

/*
 * VESC observer_state 的定点版本。
 * x1/x2 是估算的转子磁链，单位为 uWb；电流历史值单位为 mA。
 */
typedef struct {
    s32 x1;
    s32 x2;
    s32 i_alpha_last;
    s32 i_beta_last;
    u32 pll_phase_q16;
    s32 pll_speed_step_q16;
    bool pll_initialized;
} observer_state;

typedef enum {
   MC_STATE_OFF = 0,
   MC_STATE_DETECTING,
   MC_STATE_RUNNING,
   MC_STATE_FULL_BRAKE,
} mc_state;

typedef enum {
	CONTROL_MODE_DUTY = 0,
	CONTROL_MODE_SPEED,
	CONTROL_MODE_CURRENT,
	CONTROL_MODE_CURRENT_BRAKE,
	CONTROL_MODE_POS,
	CONTROL_MODE_HANDBRAKE,
	CONTROL_MODE_OPENLOOP,
	CONTROL_MODE_OPENLOOP_PHASE,
	CONTROL_MODE_OPENLOOP_DUTY,
	CONTROL_MODE_OPENLOOP_DUTY_PHASE,
	CONTROL_MODE_NONE
} mc_control_mode;

typedef enum {
	MTPA_MODE_OFF = 0,
	MTPA_MODE_IQ_TARGET,
	MTPA_MODE_IQ_MEASURED
} MTPA_MODE;

typedef struct {
    mc_foc_control_sample_mode foc_control_sample_mode;
    MTPA_MODE foc_mtpa_mode;
    FOC_SPEED_SRC foc_speed_soure;
    mc_foc_sensor_mode foc_sensor_mode;
    
    
    s16 foc_temp_comp;
    s16 foc_current_ki;           /* Q15 modulation/current-unit/tick */
    s16 foc_current_filter_const; /* Q15 */
    
    s16 foc_current_kp;           /* Q15 modulation/current-unit */
    mc_foc_cc_decoupling_mode foc_cc_decoupling;
    s32 foc_motor_r;              /* phase resistance in mOhm */
    s32 foc_motor_l;              /* phase inductance in uH */
    s32 foc_motor_flux_linkage;   /* flux linkage in uWb */
    s16 foc_overmod_factor;       /* Q15 */
    
    s16 foc_offsets_current[3];//电流零偏
    
    s16 l_max_duty;
    s16 lo_current_min;
    s16 s_pid_min_erpm;
    s16 lo_current_max;
    s16 foc_duty_dowmramp_kp;
    s16 foc_duty_dowmramp_ki;
    s16 foc_observer_offset;
    s16 foc_motor_ld_lq_diff;
    s16 foc_fw_q_current_factor;
    s16 lo_in_current_min;
    s16 lo_in_current_max;
} mc_configuration;

typedef struct
{
    mc_configuration *m_conf;
    mc_state m_state;
    mc_control_mode m_control_mode;
	motor_state_t m_motor_state;
    
    bool m_phase_control_initialized;    
    bool m_i_alpha_beta_has_offset;
    bool m_was_control_duty;
    bool duty_was_pi;
    bool m_phase_override;
    bool m_cc_was_hfi;
    bool m_duty_next_set;
    bool m_observer_initial;
    s16 m_current_ki_temp_comp;
    s16 p_lq;
    s16 p_ld;
    s16 m_speed_est_fast;
    
    u16 m_duty1_next, m_duty2_next, m_duty3_next;
    s16 m_i_alpha_sample_next;
    s16 m_i_beta_sample_next;
    u16 p_fs;
    u16 p_dt;            /* elapsed current-loop ticks */
    s16 p_max_v_mag;     /* 初始化后缓存的最大电压矢量调制度，Q15 */
    
    s16 m_i_alpha_sample_with_offset;
    s16 m_i_beta_sample_with_offset;
    
    s16 m_pll_speed;
    s16 m_id_set;
    s16 m_iq_set;
    s16 m_duty_abs_filtered;
    s16 m_duty_cycle_set;
    s16 m_br_speed_before;
    s16 m_br_vq_before;
    s16 m_duty_filtered;
    u16 m_br_no_duty_samples;
    
    s16 m_speed_pid_set_rpm;
    s16 m_duty_i_term;
    s16 duty_pi_duty_last;
    
    s16 m_phase_now_observer;
    observer_state m_observer_state;
    s16 m_i_fw_override;
    s16 m_i_fw_set;
    s16 p_duty_norm;
    s16 m_pll_phase;
    u32 m_phase_control_q16;       /* 每个 PWM 周期外推的 Q16 控制角 */

} motor_all_state_t;
    


typedef struct
{
    s16 sin;
    s16 cos;
} MCS_TRIG_Q15;

#endif

