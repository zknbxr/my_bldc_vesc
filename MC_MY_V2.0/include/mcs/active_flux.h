#ifndef ACTIVE_FLUX_H
#define ACTIVE_FLUX_H



typedef struct
{
	float Ld;
	float Lq;
	float Phi;
	float Rs;
	
	float Ts;
	
	float RIa;
	float RIb;
	float LqIa;
	float LqIb;
	
	float Ialpha;
	float Ibeta;
	float Valpha;
	float Vbeta;
	float Id;
	float Iq;
	
	float Phi_Sd;
	float Phi_Sq;
	float Phi_Salpha;
	float Phi_Sbeta;
	float Phi_Salpha_hat;
	float Phi_Sbeta_hat;
	float Err_alpha;
	float Err_beta;
	float Phi_alpha;
	float Phi_beta;
	
	float Kp;
	float Ki;
	float Phi_err_a;
	float Phi_err_b;
	float Phi_integral_a;
	float Phi_integral_b;
	float PI_out_a;
	float PI_out_b;
	
	float Theta;
	float Last_Theta;
	float Delta_Theta;
	float We;
	float Pole_Pair;
	float RPM;
}active_flux_t;

extern active_flux_t Active_Flux;

void active_flux_observer(active_flux_t * active_flux,FOC_DEF * foc_t);
void active_flux_observer_init(active_flux_t * active_flux);

#endif