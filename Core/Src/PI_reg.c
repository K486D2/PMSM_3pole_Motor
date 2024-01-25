/*
 * PI_reg.c
 *
 *  Created on: 25 maj 2023
 *      Author: Bartosz
 */
#include "main.h"
#include "PI_reg.h"
#include "Zmienna_Motor.h"



volatile uint16_t faza_programu = 100;
volatile uint8_t send_time;
volatile int16_t tq;
volatile float speed_scale;




motor_state prog_phase;



  /* Regulatory PI*/

void PI_REG(PI_reg *Reg, int16_t fdb_value, int16_t ref_value, int32_t *pi_out)
 {
	/*uchyb regulacji*/
	Reg->error = (ref_value) - fdb_value; // dopisac antiwindup oraz limity wartosci aby nie doszlo do przeregulowan
	Reg->error_sum = Reg->error_sum + Reg->error; // kierunek nie gra tu roli, regulator dzia�a tak samo

	if (Reg->error_sum > 1000000) Reg->error_sum  =  1000000;
	if (Reg->error_sum <-1000000) Reg->error_sum  = -1000000;
	Reg->out = (Reg->KP * Reg->error) + (Reg->KI * Reg->error_sum);
	if(Reg->out > 10000000)Reg->out= 10000000;
	if(Reg->out <-10000000)Reg->out=-10000000;

	*pi_out = (int16_t)(Reg->out/10000); //max output in decimal scale 1000

 }


void PI_REG_SER(PI_reg *Reg, int16_t fdb_value, int16_t ref_value, int16_t *pi_out)
 {
		/*uchyb regulacji*/
	Reg->error = (ref_value) - fdb_value;
	/*uchyb regulacji*/
	Reg->p_term = Reg->KP * Reg->error;
		/*człon całkujący*/
	Reg->i_term = Reg->anti_wind * (Reg->p_term * Reg->KI + Reg->i_term_prev);
	//Reg->i_term =  (Reg->error * Reg->KI) + Reg->i_term_prev;
		/*wyjście*/
	Reg->out_a = Reg->p_term + Reg->i_term;

	if(Reg->out_a > 10000000){Reg->out_b = 10000000;}
	else if(Reg->out_a <-10000000){Reg->out_b =-10000000;}
	else {Reg->out_b = Reg->out_a;}

	if(Reg->out_a != Reg->out_b) Reg->anti_wind = 0;
	else if(Reg->out_a == Reg->out_b) Reg->anti_wind = 1;

	Reg->i_term_prev = Reg->i_term;

	*pi_out = (int16_t)(Reg->out_b/10000);


 }

	/* Tryby pracy sterowania napędem*/

void state_speed_control_on()
{
	PI_regul_on = ON;

}
void state_currentspeed_control_on()
{
	PI_regul_on = ON;

}
void state_current_control_on()
{
	PI_regul_on = OFF;

}

void state_openloop_control()
{
	PI_regul_on = OFF;


}
void state_tuning_PI_speed()
{
	static uint16_t count;
	count++;
	state_speed_control_on();

	if(count < 3000)
	{
		rpm_speed_zad = 100;
	}
	if (count > 3000 && count < 6000){
		rpm_speed_zad = 300;
	}
	if(count == 6000){
		count = 0;
	}

}

void state_speed_control_low()
{
	//PI_regul_on = ON;
	Ki = 2000;
	Kp = 3000;
}
void state_speed_control_high()
{
	//PI_regul_on = ON;
	Ki = 200000;
	Kp = 300000;
}


void pi_reg_clc(PI_reg *Reg)
{
	Reg->error = 0;
	Reg->error_sum = 0;
	Reg->out = 0;

}


void motor_state_machine(){

switch(prog_phase)
{
case motor_start: break;
case motor_ph1:	state_openloop_control();break;
case motor_ph2: state_speed_control_on();break;
case motor_speed_tune: state_tuning_PI_speed();break;
}
}


