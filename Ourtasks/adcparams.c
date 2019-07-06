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
AN3964
https://www.st.com/resource/en/application_note/dm00035957.pdf
V DDA = 3 × Val_V REFINT_CAL ⁄ Val_V REFINT

Temp = 80 ⁄ ( TS_CAL2 – TS_CAL1 ) × ( ValTS – TS_CAL1 ) + 30

The accurate embedded internal reference voltage (V REFINT ) is individually sampled by the
ADC, and the converted value for each device (Val_V REFINT_CAL ) is stored during the
manufacturing process in the protected memory area at address VREFINT_CAL specified
in the product datasheet. The internal reference voltage calibration data is a 12-bit unsigned
number (right-aligned bits, stored in 2 bytes) acquired by the STM32L1x ADC referenced to
V VREF_MEAS = V REF+ = 3V ± 0.01V
The total accuracy of the factory measured calibration data is then provided with an
accuracy of ± 5 mV (refer to the datasheet for more details).
We can determine the actual V DDA voltage by using the formula above as follows:
V DDA = 3 × Val_V REFINT_CAL ⁄ Val_V REFINT
The temperature sensor data, ValTS_bat, are sampled with the ADC scale referenced to the
actual V DDA value determined at the previous steps. Since the temperature sensor factory
calibration data are acquired with the ADC scale set to 3 V, we need to normalize ValTS_bat
to get the temperature sensor data (ValTS) as it would be acquired with ADC scale set to
3 V. ValTS_bat can be normalized by using the formula below:
ValTS = 3 × ValTS_bat ⁄ V DDA
If the ADC is referenced to the 3 V power supply (which is the case of the STM32L1
Discovery) such a normalization is not needed and the sampled temperature data can be
directly used to determine the temperature as described in Section 2.2.1: Temperature
sensor calibration.


Vdd = 3300*(*VREFINT_CAL_ADDR)/ADC_raw;

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

#define VREFINT_CAL_ADDR 

/* Calibration values common to all ADC modules. */
struct ADCCALCOMMON adcommon;

/* Raw and calibrated ADC1 readings. */
struct ADCCHANNEL adc1chan[ADC1IDX_ADCSCANSIZE];
uint32_t adc1ctr;  // Running count of updates.

/* *************************************************************************
 * void adcparams_init(void);
 *	@brief	: Copy parameters into structs
 * NOTE: => ASSUMES ADC1 ONLY <==
 * *************************************************************************/
void adcparams_init(void)
{
	/* Load parameter values for ADC channels. */
	adcparamsinit_init(&adc1channelstuff[0]);

	/* Common to board, plus some pre-computed values. */
	adcparamsinit_init_common(&adcommon,&adc1channelstuff[0]);

	return;
}

/* *************************************************************************
 * void adcparams_internal(struct ADCCALCOMMON* pacom, struct ADC1DATA* padc1);
 *	@brief	: Update values used for compensation from Vref and Temperature
 * @param	: pacom = Pointer calibration parameters for Temperature and Vref
 * @param	: padc1 = Pointer to array of ADC reading sums plus other stuff
 * *************************************************************************/
uint32_t adcdbg1;
uint32_t adcdbg2;
void adcparams_internal(struct ADCCALCOMMON* pacom, struct ADC1DATA* padc1)
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
/* The following two computaions with ints uses 119 machines cycles. */
//	pacom->ivdd = (3300 * ADC1DMANUMSEQ) * (*PVREFINT_CAL) / (padc1->adcs1sum[ADC1IDX_INTERNALVREF]);

	pacom->ui_tmp = (pacom->ivdd * padc1->adcs1sum[ADC1IDX_INTERNALTEMP] ) / 3300; // Adjust for Vdd not at 3.3v calibration
	pacom->degC  = pacom->ll_80caldiff * (pacom->ui_tmp - pacom->ui_cal1) + (30 * SCALE1 * ADC1DMANUMSEQ);
	pacom->degC *= ((float)1.0/(SCALE1*ADC1DMANUMSEQ)); 
	pacom->degCfilt = iir_f1_f(&adc1channelstuff[ADC1IDX_INTERNALTEMP].fpw.iir_f1, pacom->degC);

	pacom->fvdd = pacom->ivdd;
	pacom->fvdd = pacom->fvdd + pacom->tcoef * (pacom->degC - (float)30);

	pacom->fvddfilt = iir_f1_f(&adc1channelstuff[ADC1IDX_INTERNALVREF].fpw.iir_f1, pacom->fvdd);

	pacom->fvddcomp = pacom->fvddfilt * pacom->sensor5vcalVdd; // Pre-compute for multple uses later

	pacom->fvddrecip = (float)1.0/pacom->fvddfilt; // Pre-compute for multple uses later

	/* Scale up for fixed division, then convert to float and descale. */
	pacom->f5_Vddratio = ( (adc1data.adcs1sum[ADC1IDX_INTERNALVREF] * (float)(1<<12)) / adc1data.adcs1sum[ADC1IDX_5VOLTSUPPLY]);
	pacom->f5_Vddratio *= ((float)1.0/(float)(1<<12));

	/* 5v supply voltage. */
	pacom->f5vsupply = adc1data.adcs1sum[ADC1IDX_5VOLTSUPPLY] * pacom->fvddfilt * pacom->f5vsupplyprecal + pacom->f5vsupplyprecal_offset;
	pacom->f5vsupplyfilt = iir_f1_f(&adc1channelstuff[ADC1IDX_5VOLTSUPPLY].fpw.iir_f1, pacom->f5vsupply);

adcdbg2 = DTWTIME - adcdbg1;

	return;
}
/* *************************************************************************
 * void adcparams_chan(uint8_t adcidx);
 *	@brief	: calibration, compensation, filtering for channels
 * @param	: adcidx = index into ADC1 array
 * *************************************************************************/
void adcparams_chan(uint8_t adcidx)
{
	struct ADCCHANNELSTUFF* pstuff = &adc1channelstuff[adcidx];
	struct ADC1DATA* padc1         = &adc1data;
	union ADCCALREADING* pread     = &adc1data.adc1calreading[adcidx];
	union ADCCALREADING* preadfilt = &adc1data.adc1calreadingfilt[adcidx];
	struct ADCCALCOMMON* pacom     = &adcommon;

float ftmp[2];

	/* Compensation type */
/* Assumes 5v sensor supply is measured with an ADC channel.
#define ADC1PARAM_COMPTYPE_NONE      0     // No supply or temp compensation applied
#define ADC1PARAM_COMPTYPE_RATIOVDD  1     // Vdd (3.3v nominal) ratiometric
#define ADC1PARAM_COMPTYPE_RATIO5V   2     // 5v ratiometric with 5->Vdd measurement
#define ADC1PARAM_COMPTYPE_RATIO5VNO 3     // 5v ratiometric without 5->Vdd measurement
#define ADC1PARAM_COMPTYPE_VOLTVDD   4     // Vdd (absolute), Vref compensation applied
#define ADC1PARAM_COMPTYPE_VOLTVDDNO 5     // Vdd (absolute), no Vref compensation applied
#define ADC1PARAM_COMPTYPE_VOLTV5    6     // 5v (absolute), with 5->Vdd measurement applied
#define ADC1PARAM_COMPTYPE_VOLTV5NO  7     // 5v (absolute), without 5->Vdd measurement applied
*/
	if (pstuff->xprms.filttype == ADC1PARAM_CALIBTYPE_RAW_UI)
	{
		pread->ui = padc1->adcs1sum[adcidx]; // adc sum as unsigned int
	}
	else
	{
		pread->f = padc1->adcs1sum[adcidx]; // Convert adc sum to float
	}

	/* Apply compensation to reading. */
	switch(pstuff->xprms.comptype)
	{
	case ADC1PARAM_COMPTYPE_NONE:      // 0 No supply or temp compensation applied
		break;

	case ADC1PARAM_COMPTYPE_RATIOVDD:  // 1 Vdd (~3.3v nominal) ratiometric
		pread->f *= pacom->fvddratio;   // ratio: 0 - 100
		break;

	case ADC1PARAM_COMPTYPE_RATIO5V:   // 2 5v ratiometric with 5->Vdd measurement	
		pread->f *= pacom->fvddfilt * pacom->f5_Vddratio;
		break;

	case ADC1PARAM_COMPTYPE_RATIO5VNO: // 3 5v ratiometric without 5->Vdd measurement
		pread->f *= pacom->fvddfilt * (float)(1.0/(4095.0 * ADCSEQNUM));
		break;

	case ADC1PARAM_COMPTYPE_VOLTVDD:   // 4 Vdd (absolute), Vref compensation applied
		pread->f *= pacom->fvddfilt * (float)(1.0/(4095.0 * ADCSEQNUM)); // 
		break;

	case ADC1PARAM_COMPTYPE_VOLTVDDNO: // 5 Vdd (absolute), no Vref compensation applied
		pread->f *= pacom->fvddfilt * (float)(1.0/3.3);
		break;

	case ADC1PARAM_COMPTYPE_VOLTV5:    // 6 5v (absolute), with 5->Vdd measurement applied	
		pread->f *= 1; // TODO
		break;	
	case ADC1PARAM_COMPTYPE_VOLTV5NO:  // 7 5v (absolute), without 5->Vdd measurement applied
		pread->f *= pacom->sensor5vcal ;
		break;

	default:
		return;
	}
/* Reproduced for convenience 
#define ADC1PARAM_CALIBTYPE_RAW_F  0    // No calibration applied: FLOAT
#define ADC1PARAM_CALIBTYPE_OFSC   1    // Offset & scale (poly ord 0 & 1): FLOAT
#define ADC1PARAM_CALIBTYPE_POLY2  2    // Polynomial 2nd ord: FLOAT
#define ADC1PARAM_CALIBTYPE_POLY3  3    // Polynomial 3nd ord: FLOAT
#define ADC1PARAM_CALIBTYPE_RAW_UI 4    // No calibration applied: UNSIGNED INT
*/
	/* Apply calibration to reading. */
	switch(pstuff->xprms.calibtype)
	{
	case ADC1PARAM_CALIBTYPE_RAW_F: // 0 No calibration applied: FLOAT
		break;
	case ADC1PARAM_CALIBTYPE_OFSC:  // 1 Offset & scale (poly ord 0 & 1): FLOAT
		pread->f = pread->f * pstuff->cal.f[1] + pstuff->cal.f[0];
		break;
	case ADC1PARAM_CALIBTYPE_POLY2: // 2 Polynomial 2nd ord: FLOAT
		pread->f = pstuff->cal.f[0] +
                 pstuff->cal.f[1] * pread->f +
                 pstuff->cal.f[2] * pread->f * pread->f;
		break;
	case ADC1PARAM_CALIBTYPE_POLY3: // 3 Polynomial 3nd ord: FLOAT
		ftmp[0] = pread->f * pread->f; // (this approach saves 4 cycles)
		ftmp[1] = ftmp[0]  * pread->f;
		pread->f = pstuff->cal.f[0] +
                 pstuff->cal.f[1] * pread->f +
                 pstuff->cal.f[2] * ftmp[0] +
                 pstuff->cal.f[3] * ftmp[1];
		break;
	case ADC1PARAM_CALIBTYPE_RAW_UI:// 4 No calibration applied: UNSIGNED INT */
		break;
	default:
		return;
	}

	/* Apply filtering */
	switch(pstuff->xprms.filttype)
	{
	case ADCFILTERTYPE_NONE:	//	0 Skip filtering
		preadfilt->f = pread->f;
		break;
	case ADCFILTERTYPE_IIR1:	//	1 IIR single pole
		preadfilt->f = iir_f1_f(&pstuff->fpw.iir_f1, pread->f);
		break;
	case ADCFILTERTYPE_IIR2:	//	2 IIR second order
		// TODO
		break;
	default:
		break;
	}
	return;
}
