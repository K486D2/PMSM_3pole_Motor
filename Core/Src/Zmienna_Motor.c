/*
 * Zmienna_Motor.c
 *
 *  Created on: 23.10.2019
 *      Author: Bartek
 */
#include "main.h"
#include "stdlib.h"
#include "math.h"
#include "Zmienna_Motor.h"


volatile uint8_t PI_regul_on = 0, FOC_ON = 1;  // if FOC=1-> torq loop, reg speed loop
volatile uint16_t POMIAR_PRADU[4], count;     // pomiar pradu
volatile int32_t Kp=300000,
				 Ki=200000;        // Kp, Ki speed loop gain
volatile uint16_t pozycja_walu;     // z enkodera absolut
volatile uint16_t pozycja_walu_deg;	// przelicznie na deg
volatile unsigned char ETAP=1;
volatile int16_t obroty_pom;
volatile int32_t pozycja_zad;

const int16_t rotor_offset = -630;

//==============================

volatile int32_t 	PI_VD_out,
					PI_VQ_out,
					prad_q_zad = 0,
					prad_d_zad = 0;

volatile int16_t    prad_q,
                    prad_d,
					prad_alpha,
					prad_beta;

volatile int32_t    napiecie_Ualpha,
                    napiecie_Ubeta,
					napiecie_U_U,
					napiecie_U_V,
					napiecie_U_W;

volatile int16_t 	rpm_speed_zad = 20,
				 	rpm_speed,
					pozycja_poprz,
					pozycja_aktu,
					pozycja_x;

volatile uint16_t   I_a_ADC,
		            I_b_ADC,
				    DC_bus_volt_ADC;




int16_t U_SVPWM, V_SVPWM, W_SVPWM;
volatile uint16_t PWM_U, PWM_V, PWM_W;
uint16_t prad[4], ADC_CAL[4];

extern TIM_HandleTypeDef htim17;



const float sin_tab[200] = { 0.031, 0.063, 0.094, 0.125, 0.156, 0.187, 0.218, 0.249, 0.279, 0.309,
	    0.339, 0.368, 0.397, 0.426, 0.454, 0.482, 0.509, 0.536, 0.562, 0.588,
	    0.613, 0.637, 0.661, 0.685, 0.707, 0.729, 0.75, 0.771, 0.79, 0.809,
	    0.827, 0.844, 0.861, 0.876, 0.891, 0.905, 0.918, 0.93, 0.941, 0.951,
	    0.96, 0.969, 0.976, 0.982, 0.988, 0.992, 0.996, 0.998, 1.0, 1.0,
	    1.0, 0.998, 0.996, 0.992, 0.988, 0.982, 0.976, 0.969, 0.96, 0.951,
	    0.941, 0.93, 0.918, 0.905, 0.891, 0.876, 0.861, 0.844, 0.827, 0.809,
	    0.79, 0.771, 0.75, 0.729, 0.707, 0.685, 0.661, 0.637, 0.613, 0.588,
	    0.562, 0.536, 0.509, 0.482, 0.454, 0.426, 0.397, 0.368, 0.339, 0.309,
	    0.279, 0.249, 0.218, 0.187, 0.156, 0.125, 0.094, 0.063, 0.031, 0.0,
	    -0.031, -0.063, -0.094, -0.125, -0.156, -0.187, -0.218, -0.249, -0.279, -0.309,
	    -0.339, -0.368, -0.397, -0.426, -0.454, -0.482, -0.509, -0.536, -0.562, -0.588,
	    -0.613, -0.637, -0.661, -0.685, -0.707, -0.729, -0.75, -0.771, -0.79, -0.809,
	    -0.827, -0.844, -0.861, -0.876, -0.891, -0.905, -0.918, -0.93, -0.941, -0.951,
	    -0.96, -0.969, -0.976, -0.982, -0.988, -0.992, -0.996, -0.998, -1.0, -1.0,
	    -1.0, -0.998, -0.996, -0.992, -0.988, -0.982, -0.976, -0.969, -0.96, -0.951,
	    -0.941, -0.93, -0.918, -0.905, -0.891, -0.876, -0.861, -0.844, -0.827, -0.809,
	    -0.79, -0.771, -0.75, -0.729, -0.707, -0.685, -0.661, -0.637, -0.613, -0.588,
	    -0.562, -0.536, -0.509, -0.482, -0.454, -0.426, -0.397, -0.368, -0.339, -0.309,
	    -0.279, -0.249, -0.218, -0.187, -0.156, -0.125, -0.094, -0.063, -0.031, 0.0};

const float cos_tab[200] = {  1.000, 0.998, 0.996, 0.992, 0.988, 0.982, 0.976, 0.969, 0.960, 0.951,
	    0.941, 0.930, 0.918, 0.905, 0.891, 0.876, 0.861, 0.844, 0.827, 0.809,
	    0.790, 0.771, 0.750, 0.729, 0.707, 0.685, 0.661, 0.637, 0.613, 0.588,
	    0.562, 0.536, 0.509, 0.482, 0.454, 0.426, 0.397, 0.368, 0.339, 0.309,
	    0.279, 0.249, 0.218, 0.187, 0.156, 0.125, 0.094, 0.063, 0.031, 0.000,
	    -0.031, -0.063, -0.094, -0.125, -0.156, -0.187, -0.218, -0.249, -0.279, -0.309,
	    -0.339, -0.368, -0.397, -0.426, -0.454, -0.482, -0.509, -0.536, -0.562, -0.588,
	    -0.613, -0.637, -0.661, -0.685, -0.707, -0.729, -0.750, -0.771, -0.790, -0.809,
	    -0.827, -0.844, -0.861, -0.876, -0.891, -0.905, -0.918, -0.930, -0.941, -0.951,
	    -0.960, -0.969, -0.976, -0.982, -0.988, -0.992, -0.996, -0.998, -1.000, -1.000,
	    -1.000, -0.998, -0.996, -0.992, -0.988, -0.982, -0.976, -0.969, -0.960, -0.951,
	    -0.941, -0.930, -0.918, -0.905, -0.891, -0.876, -0.861, -0.844, -0.827, -0.809,
	    -0.790, -0.771, -0.750, -0.729, -0.707, -0.685, -0.661, -0.637, -0.613, -0.588,
	    -0.562, -0.536, -0.509, -0.482, -0.454, -0.426, -0.397, -0.368, -0.339, -0.309,
	    -0.279, -0.249, -0.218, -0.187, -0.156, -0.125, -0.094, -0.063, -0.031, 0.000,
	    0.031, 0.063, 0.094, 0.125, 0.156, 0.187, 0.218, 0.249, 0.279, 0.309,
	    0.339, 0.368, 0.397, 0.426, 0.454, 0.482, 0.509, 0.536, 0.562, 0.588,
	    0.613, 0.637, 0.661, 0.685, 0.707, 0.729, 0.750, 0.771, 0.790, 0.809,
	    0.827, 0.844, 0.861, 0.876, 0.891, 0.905, 0.918, 0.930, 0.941, 0.951,
	    0.960, 0.969, 0.976, 0.982, 0.988, 0.992, 0.996, 0.998, 1.000};



		/**OBLICZENIA KĄTA THETA**/
void angle_theta_calc()
{
int16_t tmp_poz_walu;
tmp_poz_walu = pozycja_walu + rotor_offset; //0,044 bo dwie pary biegunów

    if(tmp_poz_walu < 0){
	pozycja_walu_deg = (ENK_ABS_RES +  tmp_poz_walu) * 0.037; // bo 3 pp, 200 elementow tablicy = 16384/600
    }
    else {
    pozycja_walu_deg = (pozycja_walu + rotor_offset) * 0.037;
    }
}

		/**TRANSFORMACJA CLARK**/
void clark_transf(int16_t prad_a, int16_t prad_b, int16_t *alpha, int16_t *beta)
{
	*alpha = prad_a;
	*beta  = (prad_a + 2 * prad_b) * 0.577;
}

	/**TRANSFORMACJA PARK**/
void park_transf(int16_t alpha, int16_t beta, int16_t rotor_pos, volatile int16_t *q, volatile int16_t *d)
{
	uint16_t idx = 0;
	idx = (rotor_pos) % 200;
	*q = (beta  * cos_tab[idx] - alpha * sin_tab[idx]);
	*d = (alpha * cos_tab[idx] + beta  * sin_tab[idx]);

}

	/**TRANSFORMATA ODWROTNA PARK'a**/
void park_rev_transf(int32_t Vd, int32_t Vq, int16_t rotor_pos,  int32_t *u_alpha,   int32_t *u_beta)
{
	uint16_t idx = 0;
	idx = (rotor_pos) % 200;
	*u_alpha = (Vd * cos_tab[idx] - Vq * sin_tab[idx]) * 1; 	//0.000002 // skalowanie do max +/- 1000 dec
	*u_beta  = (Vq * cos_tab[idx] + Vd * sin_tab[idx]) * 1; 	// 0.000002
}





void SPWM_modulacja()
{

	if(FOC_ON==1)
	{
		TIM1->CCR1 = PWM_U;
		TIM1->CCR2 = PWM_V;
		TIM1->CCR3 = PWM_W;
	}

}




int16_t Pozycja(uint16_t pozycja_ak,uint16_t pozycja_pop, int32_t *poz_calk, uint16_t kier)
{
	int16_t  delta_poz, delta_2;
delta_2=pozycja_pop-pozycja_ak;
	if(abs(pozycja_pop-pozycja_ak)>20)
	{
		if(pozycja_pop>pozycja_ak&&kier==16)
			{
				delta_poz = -(pozycja_pop-pozycja_ak);
			}
		else if(pozycja_pop<pozycja_ak&&kier==16)
			{
				delta_poz = -((16385-pozycja_ak)+pozycja_pop);
			}
		else if(pozycja_pop>pozycja_ak&&kier==0)
			{
				delta_poz = (16385-pozycja_pop)+pozycja_ak;
			}
		else if(pozycja_pop<pozycja_ak&&kier==0)
			{
				delta_poz = pozycja_ak-pozycja_pop;
			}


		*poz_calk+=delta_poz;

	}
	return delta_poz;
}
uint16_t Speed_Inc_Encoder(uint16_t enkoder_cnt, uint16_t enkoder_prev, uint16_t dir)
{
	 static int16_t enc_prev, enc, delta_enc;
	enc=enkoder_cnt;
	enc_prev=enkoder_prev;

	if(dir==0)
	{
		if(enc>enc_prev&&enc-enc_prev>5)
		{
			delta_enc=enc-enc_prev;
		}
		else if(enc<enc_prev&&(2000-enc_prev)+enc>5)
		{
			delta_enc=(2000-enc_prev)+enc;
		}
		else if(enc==enc_prev||enc-enc_prev<5)
		{
			delta_enc=0;
		}
	}
	 if(dir==16)
	{
		if(enc>enc_prev&&(2000-enc)+enc_prev>5)
			{
			 delta_enc=-((2000-enc)+enc_prev);
			}
			else if(enc<enc_prev&&enc_prev-enc>5)
			{
			delta_enc=-(enc_prev-enc);
			}
			else if(enc==enc_prev||enc_prev-enc>5)
			{
				delta_enc=0;
			}
	}

	return delta_enc;


}
void SVPWM_modulacja(int32_t u_alpha, int32_t u_beta, int16_t *U_SVM, int16_t *V_SVM, int16_t *W_SVM)
{

	// U_alpha i beta to zmienne in z transformacji, *x_SVM to pointery na wrzucenie zmiennych out
int32_t u, v, w, T1, T2, T0;
uint8_t sektor;

		u = (u_alpha * 1.73 - u_beta) * 0.5;
		v = u_beta;
		w = (-u_alpha * 1.73 - u_beta) * 0.5;

		if(u > 0 && v > 0 && w < 0)
			{
			sektor = 1;
			}
		else if(v >0 && w < 0 && u < 0)
			{
			sektor = 2;
			}
		else if(v > 0 && w >0 && u < 0)
			{
			sektor = 3;
			}
		else if(v <0 && w > 0 && u < 0)
			{
			sektor = 4;
			}
		else if(v < 0 && w > 0 && u > 0)
			{
			sektor = 5;
			}
		else if(v < 0 && w < 0 && u > 0)
			{
			sektor = 6;
			}

		switch(sektor)
		{
		case 1:	T1 = u;
				T2 = v;
				T0 = 1 - T1 - T2;
				*U_SVM = T1 + T2 + 0.5 * T0;
				*V_SVM = T2 +0.5 * T0;
				*W_SVM = 0.5 * T0;
				break;
		case 2:	T1 = -w;
				T2 = -u;
				T0 = 1 - T1 - T2;
				*U_SVM = T1 + 0.5 * T0;
				*V_SVM = T1 + T2 + 0.5 * T0;
				*W_SVM = 0.5 * T0;
				break;
		case 3:	T1 = v;
				T2 = w;
				T0 = 1 - T1 - T2;
				*U_SVM = 0.5 * T0;
				*V_SVM = T1 + T2 + 0.5 * T0;
				*W_SVM = T2 + 0.5 * T0;
				break;
		case 4:	T1 = -u;
				T2 = -v;
				T0 = 1 - T1 - T2;
				*U_SVM = 0.5 * T0;
				*V_SVM = T1 + 0.5 * T0;
				*W_SVM = T1 + T2 + 0.5 * T0;
				break;
		case 5:	T1 = w;
				T2 = u;
				T0 = 1 - T1 - T2;
				*U_SVM = T2 + 0.5 * T0;
				*V_SVM = 0.5 * T0;
				*W_SVM = T1 + T2 + 0.5 * T0;
				break;
		case 6:	T1 = -v;
				T2 = -w;
				T0 = 1 - T1 - T2;
				*U_SVM = T1 + T2 + 0.5 * T0;
				*V_SVM = 0.5 * T0;
				*W_SVM = T1 + 0.5 * T0;
				break;
		}

}



void PID_REG(PID_reg *Reg, int32_t act_value,int32_t ref_value, int32_t *iq_out)

{
	static int32_t last_error, tmp_out;
	Reg->PI_error = (ref_value) - act_value; // dopisac antiwindup oraz limity wartosci aby nie doszlo do przeregulowan
	Reg->PI_error_sum= Reg->PI_error_sum + Reg->PI_error;
	Reg->PI_diff = Reg->PI_error-last_error; //error-last_erroe

	if (Reg->PI_error_sum > 10000000) Reg->PI_error_sum  =  10000000;
	if (Reg->PI_error_sum <-10000000) Reg->PI_error_sum  = -10000000;

	Reg->PI_out = Reg->KP * Reg->PI_error + Reg->KI * Reg->PI_error_sum + Reg->KD*Reg->PI_diff;

	if(Reg->PI_out > 50000000)Reg->PI_out= 50000000;
	if(Reg->PI_out <-50000000)Reg->PI_out=-50000000;


	last_error=Reg->PI_error;
	tmp_out=(Reg->PI_out/100000);
	*iq_out = tmp_out; // dla PMSM vectro control method

}

//======FIRST ORDER LOW PASS FILTER======

void lpf_init(LowPassFilter *fil, int16_t input, float alp)
{

	fil->alp_gain = alp;
}

int16_t lpf_update(LowPassFilter *fil, int32_t input)
{

    fil->out = fil->out - ( fil->alp_gain * (fil->out - input));
    return fil->out ;
}


// dokumentacja vector control PMSM by Bartosz D






