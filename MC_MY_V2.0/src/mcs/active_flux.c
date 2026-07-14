#include "main.h"


active_flux_t Active_Flux;



void active_flux_observer(active_flux_t * active_flux,FOC_DEF * foc_t)
{
	float cos_Theta;
	float sin_Theta;
	
	if(active_flux->We>-50.0f && active_flux->We<50.0f)
	{
		active_flux->Kp = 50.0f;
		active_flux->Ki = 314.0f;
	}
	else 
	{
		active_flux->Kp = active_flux->We*sign(active_flux->We);
		active_flux->Ki = 2.0f*PI*active_flux->We*sign(active_flux->We);
	}
	
	active_flux->Ialpha = foc_t->Ialpha;
	active_flux->Ibeta = foc_t->Ibeta;
	active_flux->Valpha = foc_t->Valpha;
	active_flux->Vbeta = foc_t->Vbeta;
	active_flux->Id = foc_t->Id;
	active_flux->Iq = foc_t->Iq;
	
	active_flux->RIa = active_flux->Rs * active_flux->Ialpha;
	active_flux->RIb = active_flux->Rs * active_flux->Ibeta;
	active_flux->LqIa = active_flux->Lq * active_flux->Ialpha;
	active_flux->LqIb = active_flux->Lq * active_flux->Ibeta;
	
	active_flux->Phi_Sd = active_flux->Id * active_flux->Ld + active_flux->Phi;
	active_flux->Phi_Sq = active_flux->Iq * active_flux->Lq;
	
	cos_Theta = arm_cos_f32(active_flux->Theta);
	sin_Theta = arm_sin_f32(active_flux->Theta);
	active_flux->Phi_Salpha = active_flux->Phi_Sd * cos_Theta - active_flux->Phi_Sq * sin_Theta;
	active_flux->Phi_Sbeta = active_flux->Phi_Sd * sin_Theta + active_flux->Phi_Sq * cos_Theta;
	
	active_flux->Phi_err_a = active_flux->Phi_Salpha_hat - active_flux->Phi_Salpha;
	active_flux->Phi_err_b = active_flux->Phi_Sbeta_hat - active_flux->Phi_Sbeta;
	active_flux->Phi_integral_a += active_flux->Ki*active_flux->Phi_err_a*active_flux->Ts;
	active_flux->Phi_integral_b += active_flux->Ki*active_flux->Phi_err_b*active_flux->Ts;
	active_flux->PI_out_a = active_flux->Phi_err_a*active_flux->Kp + active_flux->Phi_integral_a;
	active_flux->PI_out_b = active_flux->Phi_err_b*active_flux->Kp + active_flux->Phi_integral_b;
	
	active_flux->Err_alpha = active_flux->Valpha - active_flux->RIa - active_flux->PI_out_a;
	active_flux->Err_beta = active_flux->Vbeta - active_flux->RIb - active_flux->PI_out_b;
	
	active_flux->Phi_Salpha_hat += active_flux->Err_alpha*active_flux->Ts;
	active_flux->Phi_Sbeta_hat += active_flux->Err_beta*active_flux->Ts;
	
	active_flux->Phi_alpha = active_flux->Phi_Salpha_hat - active_flux->LqIa;
	active_flux->Phi_beta = active_flux->Phi_Sbeta_hat - active_flux->LqIb;
	
	active_flux->Theta = utils_fast_atan2(active_flux->Phi_beta,active_flux->Phi_alpha);
	if(active_flux->Theta<0.0f)
	{
		active_flux->Theta += _2_PI;
	}
	else if(active_flux->Theta>_2_PI)
	{
		active_flux->Theta -= _2_PI;
	}
	
	active_flux->Delta_Theta = active_flux->Theta - active_flux->Last_Theta;
	if(active_flux->Delta_Theta>PI)
	{
		active_flux->Delta_Theta = active_flux->Theta - active_flux->Last_Theta - _2_PI;
	}
	else if(active_flux->Delta_Theta<-PI)
	{
		active_flux->Delta_Theta = active_flux->Theta - active_flux->Last_Theta + _2_PI;
	}
	active_flux->We = active_flux->Delta_Theta/active_flux->Ts;
	iir(active_flux->We,&active_flux->We,&Active_Flux_We_Lpf_Par); 
	active_flux->RPM = active_flux->We*9.54930f/active_flux->Pole_Pair;
	active_flux->Last_Theta = active_flux->Theta;
}

void active_flux_observer_init(active_flux_t * active_flux)
{
	active_flux->Ld = foc_motor_ld;
	active_flux->Lq = foc_motor_lq;
	active_flux->Rs = foc_motor_r;
	active_flux->Phi = foc_motor_flux_linkage;
	active_flux->Pole_Pair = foc_motor_pole;
	active_flux->Ts = FOC_Ts;
}


