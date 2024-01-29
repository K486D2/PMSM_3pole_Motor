/*
 * pll_sensorless.h
 *
 *  Created on: Jan 25, 2024
 *      Author: Bartosz
 */

#ifndef INC_PLL_SENSORLESS_H_
#define INC_PLL_SENSORLESS_H_


#define PLL_PI						(float)3.14
#define	PLL_ANGLE_2P                (float)6.28
#define PLL_ANGLE_STEP				(float)PLL_ANGLE_2P/200 // 200 to rozmiar tablicy sin,cos

typedef struct
{
	int32_t qdeltaT;			// interwał integratora
	int32_t qRho;				// esytmowany kąt wirnika
	int32_t qOmegaMr;			// pierwotna estymacja predkosci
	int32_t qLastIalpha;		// poprzednia wartość prad i aplha
	int32_t qLastIbeta;			// poprzednia wartość prad i beta
	int32_t qDIalpha;			// różnica Ialpha
	int32_t qDIbeta;			// różnica Ibeta
	int32_t qEsa;				// BEMF alpha
	int32_t qEsb;				// BEMF beta
	int32_t qEsd;				// BEMF d
	int32_t qEsq;				// BEMF q
	int32_t qVIndalpha;			// Ls * di/dt alhpa
	int32_t qVIndbeta;			// Ls * di/dt beta
	int32_t qEsdf;          	// BEMF d filtr
	int32_t qEsqf;          	// BEMF q filtr
	int32_t qKfilter;       	// wzmocnienie filtru
	int32_t qVelEstim;      	// estymowana predkość
	int32_t qVelEstimFilterK;	// wzmocnenie filtru dla estymowania predkosci
	int32_t qLastValpha;		// wartosc z ostatniego kroku Ialpha
	int32_t qLastVbeta;		    // wartosc z ostatniego kroku Ibeta
	int32_t RhoOffset;			// estymowany kąt wirnika
	int32_t qRs;				// rezystancja silnika
	int32_t qLs;				// indukcyjność silnika
	int32_t qLsDt;				// indukcyjność stojana /dt - zmienna z predkoscia
	int32_t qInvKi;				// omega/bemf
	int32_t qKFi;				// back emf constant v-sec/rad
	int32_t qInvKFi_below;      // back emf const poniże
	int32_t	qLs_DIV_2_PI;		// indukcyjność fazy
	int32_t qNominal_Speed;		// predkość nominalna silnika rad/sec
	int32_t qDecim_Nom_speed;	// predkosć dec/10

}pll_estymator_t;

typedef struct
{
	int32_t alpha;
	int32_t beta;
}pll_bemf_alp_bet_t;

typedef struct {

	float Angle;
	float sinus;
	float cosinus;
}pll_theta_t;

#endif /* INC_PLL_SENSORLESS_H_ */
