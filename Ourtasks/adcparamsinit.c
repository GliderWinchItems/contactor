/******************************************************************************
* File Name          : adcparamsinit.c
* Date First Issued  : 03/09/2019
* Board              : DiscoveryF4
* Description        : Initialization of parameters for ADC app configuration
*******************************************************************************/

/* 
This is where hard-coded parameters for the ADC are entered.

Later, this may be replaced with a "copy" of the flat file in high flash, generated
by the java program from the sql database.
*/

/*

                    Min  Typ  Max 
Internal reference: 1.16 1.20 1.24 V

*/

#include "adcparamsinit.h"
#include "adcparams.h"
#include "ADCTask.h"

/* *************************************************************************
 * void adcparamsinit_init_common(struct ADCCALCOMMON* padccommon, struct ADCCHANNELSTUFF* pacsx);
 *	@brief	: Initialize struct with parameters common to all ADC for this =>board<=
 * @param	: padccommon = pointer to struct holding parameters
 * @param	: pacsx = Pointer to struct "everything" for this ADC module
 * *************************************************************************/
void adcparamsinit_init_common(struct ADCCALCOMMON* padccommon, struct ADCCHANNELSTUFF* pacsx)
{
	padccommon->sensor5vcal    = 0.54 / ADCSEQNUM;	// 5v->Vdd divide ratio
	padccommon->sensor5vcalVdd = padccommon->sensor5vcal / 3.3; // Precompute: adjust for Vdd
	padccommon->fvddratio      = (100.0/(4095.0 * ADCSEQNUM));  // Ratiometric: Percent
	padccommon->f5vsupplyprecal = 1.0/( (pacsx + ADC1IDX_5VOLTSUPPLY)->cal.f[1] * 4095 * ADCSEQNUM);
	padccommon->f5vsupplyprecal_offset = (pacsx + ADC1IDX_5VOLTSUPPLY)->cal.f[0];

	padccommon->tcoef        = 30E-6; // 30 typ, 50 max, (ppm/deg C)

//	padccommon->ts_cal1      = (float)(*PTS_CAL1) * (float)ADC1DMANUMSEQ; // Factory calibration
//	padccommon->ts_cal2      = *PTS_CAL2; // Factory calibration
//	padccommon->ts_caldiff   = *PTS_CAL2 - *PTS_CAL1; // Pre-compute
	padccommon->ts_80caldiff = (80.0 / (padccommon->ts_caldiff *(float)ADC1DMANUMSEQ)); // Pre-compute

//	padccommon->uicaldiff    = *PTS_CAL2 - *PTS_CAL1; // Pre-compute
	padccommon->ll_80caldiff = (80 * SCALE1) /(padccommon->uicaldiff);
//	padccommon->ui_cal1      =	(*PTS_CAL1) * ADC1DMANUMSEQ;

	/* Data sheet gave these values.  May not need them. */
	padccommon->v25     = 0.76; // Voltage at 25 °C, typ
	padccommon->slope   = 2.0;  // Average slope (mv/deg C), typ

	return;
}



/* *************************************************************************
 * void adcparamsinit_init(struct ADCCHANNELSTUFF* pacsx);
 *	@brief	: Load structs for compensation, calibration and filtering for ADC channels
 * @param	: pacsx = Pointer to struct "everything" for this ADC module
 * *************************************************************************/

/* Reproduced for convenience

#define ADC1IDX_5VOLTSUPPLY   0   // PA0 IN0  - 5V sensor supply
#define ADC1IDX_CURRENTTOTAL  1   // PA5 IN5  - Current sensor: total battery current
#define ADC1IDX_CURRENTMOTOR1 2   // PA6 IN6  - Current sensor: motor #1
#define ADC1IDX_12VRAWSUPPLY  3   // PA7 IN7  - +12 Raw power to board
#define ADC1IDX_INTERNALTEMP  4   // IN17     - Internal temperature sensor
#define ADC1IDX_INTERNALVREF  5   // IN18     - Internal voltage reference

#define ADC1PARAM_COMPTYPE_NONE      0     // No supply or temp compensation applied
#define ADC1PARAM_COMPTYPE_RATIOVDD  1     // Vdd (3.3v nominal) ratiometric
#define ADC1PARAM_COMPTYPE_RATIO5V   2     // 5v ratiometric with 5->Vdd measurement
#define ADC1PARAM_COMPTYPE_RATIO5VNO 3     // 5v ratiometric without 5->Vdd measurement
#define ADC1PARAM_COMPTYPE_VOLTVDD   4     // Vdd (absolute), Vref compensation applied
#define ADC1PARAM_COMPTYPE_VOLTVDDNO 5     // Vdd (absolute), no Vref compensation applied
#define ADC1PARAM_COMPTYPE_VOLTV5    6     // 5v (absolute), with 5->Vdd measurement applied
#define ADC1PARAM_COMPTYPE_VOLTV5NO  7     // 5v (absolute), without 5->Vdd measurement applied

#define ADC1PARAM_CALIBTYPE_RAW_F  0    // No calibration applied: FLOAT
#define ADC1PARAM_CALIBTYPE_OFSC   1    // Offset & scale (poly ord 0 & 1): FLOAT
#define ADC1PARAM_CALIBTYPE_POLY2  2    // Polynomial 2nd ord: FLOAT
#define ADC1PARAM_CALIBTYPE_POLY3  3    // Polynomial 3nd ord: FLOAT
#define ADC1PARAM_CALIBTYPE_RAW_UI 4    // No calibration applied: UNSIGNED INT
*/

void adcparamsinit_init(struct ADCCHANNELSTUFF* pacsx)
{
	struct ADCCHANNELSTUFF* pacs; // Use pointer for convenience

/* IN18 - Internal voltage reference */
	pacs = pacsx + ADC1IDX_INTERNALVREF; // Point to "stuff" for this ADC channel

	// Filter type, calibration option, compensation option. */
	pacs->xprms.filttype  = ADCFILTERTYPE_IIR1;      // Single pole IIR
	pacs->xprms.calibtype = ADC1PARAM_CALIBTYPE_RAW_F; // Raw; no calibration applied
	pacs->xprms.comptype  = ADC1PARAM_COMPTYPE_NONE; // No temperature compensation

	// Calibration coefficients.
	pacs->cal.f[0] = 0.0;  // Offset
	pacs->cal.f[1] = 1.0;  // Scale (jic calibration not skipped)

	// Filter initialize, coefficients, and pre-computed value. */
	pacs->fpw.iir_f1.skipctr  = 12;    // Initial readings skip count
	pacs->fpw.iir_f1.coef     = 0.9999;  // Filter coefficient (< 1.0)
	pacs->fpw.iir_f1.onemcoef = (1 - pacs->fpw.iir_f1.coef);

/* IN17 - Internal temperature sensor */
	pacs = pacsx + ADC1IDX_INTERNALTEMP; // Point to "stuff" for this ADC channel

	// Filter type, calibration option, compensation option. */
	pacs->xprms.filttype  = ADCFILTERTYPE_IIR1;      // Single pole IIR
	pacs->xprms.calibtype = ADC1PARAM_CALIBTYPE_RAW_F; // Raw; no calibration applied
	pacs->xprms.comptype  = ADC1PARAM_COMPTYPE_NONE; // No temperature compensation

	// Calibration coefficients.
	pacs->cal.f[0] = 0.0;  // Offset
	pacs->cal.f[1] = 1.0;  // Scale (jic calibration not skipped)

	// Filter initialize, coefficients, and pre-computed value. */
	pacs->fpw.iir_f1.skipctr  = 4; 	  // Initial readings skip count
	pacs->fpw.iir_f1.coef     = 0.99;  // Filter coefficient (< 1.0)
	pacs->fpw.iir_f1.onemcoef = (1 - pacs->fpw.iir_f1.coef);


/* Total battery current sensor. */
	pacs = pacsx + ADC1IDX_CURRENTTOTAL; // Point to "stuff" for this ADC channel

	// Filter type, calibration option, compensation option. */
	pacs->xprms.filttype  = ADCFILTERTYPE_IIR1;        // Single pole IIR
	pacs->xprms.calibtype = ADC1PARAM_CALIBTYPE_OFSC;  // Offset & scale (poly ord 0 & 1)
	pacs->xprms.comptype  = ADC1PARAM_COMPTYPE_RATIO5V; // 5v w Vref abs w temp

	// Calibration coefficients.
	pacs->cal.f[0] = 2047.5; // Offset
	pacs->cal.f[1] = 0.1086; // Scale (200a @saturation)

	// Filter initialize, coefficients, and pre-computed value. */
	pacs->fpw.iir_f1.skipctr  = 4; 	 // Initial readings skip count
	pacs->fpw.iir_f1.coef     = 0.9;   // Filter coefficient (< 1.0)
	pacs->fpw.iir_f1.onemcoef = (1 - pacs->fpw.iir_f1.coef);

/* Current sensor: motor #1 */
	pacs = pacsx + ADC1IDX_CURRENTMOTOR1; // Point to "stuff" for this ADC channel

	// Filter type, calibration option, compensation option. */
	pacs->xprms.filttype  = ADCFILTERTYPE_IIR1;        // Single pole IIR
	pacs->xprms.calibtype = ADC1PARAM_CALIBTYPE_OFSC;  // Offset & scale (poly ord 0 & 1)
	pacs->xprms.comptype  = ADC1PARAM_COMPTYPE_VOLTV5; // 5v w Vref abs w temp

	// Calibration coefficients.
	pacs->cal.f[0] = 2047.5;  // Offset
	pacs->cal.f[1] = 0.3257;  // Scale (600a @saturation)

	// Filter initialize, coefficients, and pre-computed value. */
	pacs->fpw.iir_f1.skipctr  = 4; 	 // Initial readings skip count
	pacs->fpw.iir_f1.coef     = 0.9;   // Filter coefficient (< 1.0)
	pacs->fpw.iir_f1.onemcoef = (1 - pacs->fpw.iir_f1.coef);

/* +12v supply voltage */
	pacs = pacsx + ADC1IDX_12VRAWSUPPLY; // Point to "stuff" for this ADC channel

	// Filter type, calibration option, compensation option. */
	pacs->xprms.filttype  = ADCFILTERTYPE_IIR1;        // Single pole IIR
	pacs->xprms.calibtype = ADC1PARAM_CALIBTYPE_OFSC;  // Offset & scale (poly ord 0 & 1)
	pacs->xprms.comptype  = ADC1PARAM_COMPTYPE_VOLTVDD; // 5v w Vref abs w temp

	// Calibration coefficients.
	pacs->cal.f[0] = 0.0;     // Offset
	pacs->cal.f[1] = 0.1525; // Scale (volts) (1.8K-10K)

	// Filter initialize, coefficients, and pre-computed value. */
	pacs->fpw.iir_f1.skipctr  = 4; 	 // Initial readings skip count
	pacs->fpw.iir_f1.coef     = 0.9;   // Filter coefficient (< 1.0)
	pacs->fpw.iir_f1.onemcoef = (1 - pacs->fpw.iir_f1.coef);

/* 5v supply. */
	pacs = pacsx + ADC1IDX_5VOLTSUPPLY; // Point to "stuff" for this ADC channel

	// Filter type, calibration option, compensation option. */
	pacs->xprms.filttype  = ADCFILTERTYPE_IIR1;        // Single pole IIR
	pacs->xprms.calibtype = ADC1PARAM_CALIBTYPE_OFSC;  // Offset & scale (poly ord 0 & 1)
	pacs->xprms.comptype  = ADC1PARAM_COMPTYPE_VOLTVDD; // 5v sensor; Vref w 5v supply reading compensation

	// Calibration coefficients.
	pacs->cal.f[0] = -4.3;    // Offset(millivolts)
	pacs->cal.f[1] = 0.49987; // Scale (millivolts) (divider: 10K-10K)

	// Filter initialize, coefficients, and pre-computed value. */
	pacs->fpw.iir_f1.skipctr  = 4; 	 // Initial readings skip count
	pacs->fpw.iir_f1.coef     = 0.99;   // Filter coefficient (< 1.0)
	pacs->fpw.iir_f1.onemcoef = (1 - pacs->fpw.iir_f1.coef);

	return;
};
