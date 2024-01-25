#include "main.h"
#include "stdint.h"





//=================================
// Parametry do płytki HW
// gain Op Amp = 5.81
// Rsh = 0.01 ohms, DC_offset = 1.65 V
// prad max zmierz --- +/- 28 A
// obliczenia prad w pliku motor_readme projektu
// w pliku read też skalowanie wartosci do 1000 jako 100% każdej wielkosci


#define PRAD_MAX 						(int16_t)28
#define PRAD_MAX_MOTOR					(int16_t)10
#define PRAD_OVC_LIMIT					(int16_t)12
#define PRAD_MAX_MOTOR_SQR				(int16_t)*((int16_t)*PRAD_MAX_MOTOR(int16_t)*PRAD_MAX_MOTOR)
#define PRAD_ADC_SCALE					(uint16_t)PRAD_MAX/2048
#define DC_BUS_ADC_SCALE				10

#define PRAD_MAX_PU 					2048
#define MOTOR_SPEED_MAX 				3000


#define I_OP_SCALE						72
#define I_RMS							3
#define I_RMS_PU						I_RMS * I_OP_SCALE
#define	I_MAX							I_RMS * 1.5
#define I_MAX_PU						I_MAX * I_OP_SCALE
#define TORQ_MAX						I_MAX

#define PI_SPEED_TIME					5 // 5ms, 	200 Hz pętla speed bandtwidth
#define PI_LOW_SPEED_TIME 				10 // 10ms, 100 Hz pętla speed bandtwidth
#define SPEED_SCALE 					(float)0.72f // dla 5ms
#define	SPEED_SCALE_LOW					(float)0.36f // dla 10ms

//=======MOTOR PARAMETRY============

#define MOTOR_PHASE_RESISTANCE 			(float)0.82  // ohms
#define MOTOR_PHASE_INDUCTANCE			(float)0.0035 // H
#define MOTOR_BACK_EMF					(float)72.2   // vpeak/krpm
#define MOTOR_POLE_PAIR					3

//======SOFTWARE PARAMETRY==========

#define PWM_FREQUENCY					10000U    // 10 kHz, centre aligned
#define PWM_SAMPLE_TIME					(float)(1/PWM_FREQUENCY) // 0.000100s == 100 us
#define T_PWM							(float)(PWM_SAMPLE_TIME)

//=====ENKODER PARAMETRY============

#define ENK_ABS_RES				        16384U
#define ENK_ABS_COUNT_PER_ELEC	        (uint16_t)(ENKODER_RES/MOTOR_POLE_PAIR)
#define ENK_INK_RES						1024U



extern volatile uint8_t PI_regul_on, FOC_ON;  // if FOC=1-> torq loop, reg speed loop
extern volatile uint16_t POMIAR_PRADU[4], count;     // pomiar pradu
extern volatile int32_t Kp,
				 Ki;        // Kp, Ki speed loop gain
extern volatile uint16_t pozycja_walu;     // z enkodera absolut
extern volatile uint16_t pozycja_walu_deg;	// przelicznie na deg
extern volatile unsigned char ETAP;
extern volatile int16_t obroty_pom;
extern volatile int32_t pozycja_zad;

extern const int16_t rotor_offset;

//==============================

extern volatile int32_t 	PI_VD_out,
					        PI_VQ_out,
					        prad_q_zad,
					        prad_d_zad;

extern volatile int16_t    prad_q,
                           prad_d,
					       prad_alpha,
					       prad_beta;

extern volatile int32_t    napiecie_Ualpha,
                    	   napiecie_Ubeta,
					       napiecie_U_U,
					       napiecie_U_V,
					       napiecie_U_W;

extern volatile int16_t 	rpm_speed_zad,
				 			rpm_speed,
							pozycja_poprz,
							pozycja_aktu,
							pozycja_x;

extern volatile uint16_t   I_a_ADC,
		                   I_b_ADC,
				           DC_bus_volt_ADC;




extern int16_t U_SVPWM, V_SVPWM, W_SVPWM;
extern volatile uint16_t PWM_U, PWM_V, PWM_W;
extern uint16_t prad[4], ADC_CAL[4];


typedef struct
{
	int32_t PI_error;
	int32_t PI_error_sum;
	int32_t PI_diff;
	int32_t PI_out;
	float KP;
	float KI;
	float KD;

}PID_reg;

typedef struct {
	int16_t out;
	int16_t alp_gain;
}LowPassFilter;





void angle_theta_calc();
void clark_transf(int16_t prad_a, int16_t prad_b, int16_t *alpha, int16_t *beta);
void park_transf(int16_t alpha, int16_t beta, int16_t rotor_pos, volatile int16_t *q, volatile int16_t *d);
void park_rev_transf(int32_t Vd, int32_t Vq, int16_t rotor_pos,  int32_t *u_alpha,  int32_t *u_beta);


void Generacja_Sinusa();
void SPWM_modulacja();
void Voltage_reg();
int16_t Pozycja(uint16_t pozycja_ak,uint16_t pozycja_pop, int32_t *poz_calk, uint16_t kier);
uint16_t Speed_Inc_Encoder(uint16_t enkoder_cnt, uint16_t enkoder_prev, uint16_t dir);
void SVPWM_modulacja(int32_t u_alpha, int32_t u_beta, int16_t *U_SVM,int16_t *V_SVM, int16_t *W_SVM);
void PID_REG(PID_reg *Reg, int32_t act_value,int32_t ref_value, int32_t *iq_out);

void lpf_init(LowPassFilter *fil, int16_t input, float alp);
int16_t lpf_update(LowPassFilter *fil, int32_t input);
