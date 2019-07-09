/******************************************************************************
* File Name          : adcparams.c
* Date First Issued  : 03/09/2019
* Board              : DiscoveryF4
* Description        : Parameters for ADC app configuration
*******************************************************************************/
/*
Not thread safe.
*/
#include "adcparams.h"
#include "adcparamsinit.h"
#include "ADCTask.h"

#include "DTW_counter.h"

/*

Temp(degree) = (V_sense - V_25)/Avg_slope + 25

                 Min  Typ  Max
Average slope    4.0  4.3  4.6  mV/°C
Voltage at 25 °C 1.34 1.43 1.52 V

ADC sampling time when reading the
temperature - - 17.1 μs

V 25 = V SENSE value for 25° C and
Avg_Slope = Average Slope for curve between Temperature vs. V SENSE (given in
   mV/° C or μV/ °C).
*/

/* Everything for ADC1. */
struct ADCFUNCTION adc1;

/* *************************************************************************
 * void adcparams_init(void);
 *	@brief	: Copy parameters into structs
 * NOTE: => ASSUMES ADC1 ONLY <==
 * *************************************************************************/
void adcparams_init(void)
{
	/* Load parameters, either hard coded, (or later implement from high flash). */
	adc_idx_v_struct_hardcode_params(&adc1.lc);

	/* Init working struct for ADC function. */
	adcparamsinit_init(&adc1);

	return;
}

/* *************************************************************************
 * static void internal(struct ADCFUNCTION* p, struct ADCINTERNAL* pi,uin8_t idx);
 *	@brief	: Update values used for compensation from Vref and Temperature
 * @param	: p = Pointer to array of ADC reading sums plus other stuff
 * @param	: idx = index into ADC sum for sensor
 * *************************************************************************/
uint32_t adcdbg1;
uint32_t adcdbg2;
static void internal(struct ADCFUNCTION* p, struct ADCINTERNAL* pi,uin8_t idx)
{
/* 

Obtain the temperature using the following formula:
  Temperature (in °C) = {(V 25 - V SENSE ) / Avg_Slope} + 25.
Where,
   V 25 = V SENSE value for 25° C and
   Avg_Slope = Average Slope for curve between Temperature vs. V SENSE (given in
       mV/° C or μV/ °C).
Refer to the Electrical characteristics section for the actual values of V 25 and Avg_Slope

Average slope     4.0  4.3  4.6  mV/°C
Voltage at 25 °C  1.34 1.43 1.52 V

*/

adcdbg1 = DTWTIME;
adcdbg2 = DTWTIME - adcdbg1;

	return;
}
/* *************************************************************************
 * static void absolute(struct ADCFUNCTION* p, struct ADCABSOLUTE* pa,uin8_t idx);
 *	@brief	: Calibrate and filter absolute voltage readings
 * @param	: p = Pointer to array of ADC reading sums plus other stuff
 * @param	: pa = Pointer to absolute parameters
 * @param	: idx = index into ADC sum for sensor
 * *************************************************************************/
static void absolute(struct ADCFUNCTION* p, struct ADCABSOLUTE* pa,uin8_t idx)
{
	



}

/* *************************************************************************
 * static void ratiometric5v(struct ADCFUNCTION* p, struct ADCRATIOMETRIC* pr,uin8_t idx);
 *	@brief	: Calibrate and filter 5v ratiometric (e.g. Hall-effect sensor)
 * @param	: p = Pointer to array of ADC reading sums plus other stuff
 * @param	: pr = Pointer to ratiometric working vars
 * @param	: idx = index into ADC sum for sensor
 * *************************************************************************/
static void ratiometric5v(struct ADCFUNCTION* p, struct ADCRATIOMETRIC* pr,uin8_t idx)
{
	/* Compute ratio of sensor reading to 5v supply reading. */
	uint32_t adcke = (p->chan[idx].sum << ADCSCALE1b);
	uint32_t adcratio = adcke / adcp->chan[ADC1IDX_5VOLTSUPPLY].sum;

	/* Subtract offset */
	int32_t tmp = adcratio - pr->iko; 

	/* Apply adjustment for unequal resistor dividers. */
	int64_t tmp64 = pr->irk5ke * tmp;
	pr->iI = (tmp64 >> ADCSCALE1b);	// Normalize

	/* Filter */

	return;
}

/* *************************************************************************
 * void adcparams_cal(void);
 *	@brief	: calibrate and filter ADC readings
 * *************************************************************************/
void adcparams_cal(void)
{
	struct ADCFUNCTION* p = &adc1; // Convenience pointer

	ratiometric5v(p, p->cur1, ADC1IDX_CURRENTTOTAL);

	ratiometric5v(p, p->cur1, ADC1IDX_CURRENTMOTOR);


	return;
}
