/*
 * pll_sensorless.c
 *
 *  Created on: Jan 25, 2024
 *      Author: Bartosz
 */
#include "Zmienna_Motor.h"
#include "pll_sensorless.h"

volatile int16_t est_rotor_angle;


extern pll_estymator_t	    pll_est;
extern pll_bemf_alp_bet_t     V_alphabeta;
extern pll_theta_t			theta_est;
extern const float sin_tab[200];
extern const float cos_tab[200];


void pll_theta_estimator(pll_theta_t *plltht)
	{

	uint32_t y_index;
	uint32_t y_indexNext;
	float x0, x1, y0, y1, temp;

	//sprawdzenie kąta czy 0<= kąt < 2PI
	if(plltht->Angle < 0){
	   plltht->Angle = plltht->Angle + PLL_ANGLE_2P; // zrolowanie do pełnego obrotu
	}
	if(plltht->Angle >= 6.28){
	   plltht->Angle = plltht->Angle - PLL_ANGLE_2P;
	}
	y_index = (uint32_t)(plltht->Angle/PLL_ANGLE_STEP);

	if(y_index >= 200)
	{
		y_index = 0;
		y_indexNext = 1;
		x0 = PLL_ANGLE_2P;
		x1 = PLL_ANGLE_STEP;
		temp = 0;
	}
	else {
		y_indexNext = y_index + 1;
		if(y_indexNext >= 200)
		{
			y_indexNext = 0;
			x1 = PLL_ANGLE_2P;
		}
		else{
			x1 = y_index * PLL_ANGLE_STEP;
		}
		x0 = y_index * PLL_ANGLE_STEP;
		temp = (plltht->Angle - x0)/(x1 - x0);

	}

	// Olbiczenia sin
	y0 = sin_tab[y_index];
	y1 = sin_tab[y_indexNext];
	plltht->sinus = y0 + ((y1 - y0)*temp);

	// Olbiczenia cos
	y0 = cos_tab[y_index];
	y1 = sin_tab[y_indexNext];
	plltht->cosinus = y0 + ((y1 - y0)*temp);


}



void pll_sensorless_estimator(pll_estymator_t *pll, pll_bemf_alp_bet_t *V_alphabetaparam)
   {

	pll_bemf_alp_bet_t bemf_ab; //lokalna struktura
    int16_t *l_bemfq, *l_bemfd;

		//===== PLL ESTYMATOR INIT =====//
	pll->qDIalpha 	= prad_alpha - pll->qLastIalpha;
	pll->qVIndalpha = pll->qLsDt * pll->qDIalpha;
	pll->qDIbeta    = prad_beta  - pll->qLastIbeta;
	pll->qVIndbeta  = pll->qLsDt * pll->qDIbeta;

	// update wartosci z poprzedniego kroku pradow w osi alpha i beta
	pll->qLastIalpha = prad_alpha;
	pll->qLastIbeta  = prad_beta;

	//równania napięciowe stojana
	pll->qEsa = bemf_ab.alpha = pll->qLastValpha - (pll->qRs * prad_alpha) -
				pll->qVIndalpha;
	pll->qEsb = bemf_ab.beta  = pll->qLastVbeta  - (pll->qRs * prad_beta) -
				pll->qVIndbeta;

	// update wartosci z poprzedniego kroku napięć w osi alpha i beta
	pll->qLastValpha = 1600 * V_alphabetaparam->alpha;  // max voltage adc 1600
	pll->qLastVbeta  = 1600 * V_alphabetaparam->beta;

	// obliczenia sin(Rho) i Cos(Rho)
	est_rotor_angle = pll->qRho + pll->RhoOffset;

	pll_theta_estimator(&theta_est);
	float tmp_theta = theta_est.Angle;

	//sprawdzić czy jest sens zmieniać zmienne lokalne na globalne i sprawdzić
	// przejście z float na int i odwrotnie bo tu moga się nie zgadzać typy zmiennych

	// BACK EMF (Alpha, Beta) ESA, ESB to Back EMF(D,Q) ESD, ESQ do park transform

	park_transf(bemf_ab.alpha, bemf_ab.beta, tmp_theta, &l_bemfq, &l_bemfd);

	pll->qEsd = l_bemfd;
	pll->qEsq = l_bemfq;

	// filtrowanie LOW PASS FILTER ==
	pll->qEsdf = pll->qEsdf + (pll->qEsd - pll->qEsdf) * pll->qKFi;
	pll->qEsqf = pll->qEsqf + (pll->qEsq - pll->qEsqf) * pll->qKFi;


}







