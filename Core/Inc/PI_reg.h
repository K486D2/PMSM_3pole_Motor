#include "main.h"
#include "stdio.h"
#include "stdint.h"



typedef struct
{
	int16_t error;
	int32_t error_sum;
	int32_t out;
	uint16_t KP;
	uint16_t KI;
	/*series PI*/
	int32_t out_a;
	int32_t out_b;
	int32_t p_term;
	int32_t i_term;
	int32_t i_term_prev;
	uint8_t anti_wind;

}PI_reg;



typedef enum
{
  CCW = 1,
  CW  = 0,
  ON = 1,
  OFF = 0,

}motor;

typedef enum{
  motor_start = 1,
  motor_ph1 = 2,
  motor_ph2 = 3,
  motor_reset = 4,
  motor_stop  = 5,
  motor_fault = 6,
  motro_fwd = 7,
  motor_rwd = 8,
  motor_speed_tune = 9,

}motor_state;



extern volatile uint8_t send_time;
extern volatile int16_t tq;
extern volatile uint16_t faza_programu;
extern volatile float speed_scale;


void motor_state_machine();
void PI_REG(PI_reg *Reg, int16_t fdb_value, int16_t ref_value, int32_t *pi_out);
void pi_reg_clc(PI_reg *Reg);



