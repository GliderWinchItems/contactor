/******************************************************************************
* File Name          : adcparams.h
* Date First Issued  : 06/15/2019
* Board              : DiscoveryF4
* Description        : Parameters for ADC app configuration
*******************************************************************************/
/* CALIBRATION NOTES:

5 volt supply calibration:
[Do this first: This calibration is applied to all 5v sensors]
- Measure 5v supply
- Measure Vdd
- Display ADCsum[5v supply]
Compute--
Ratio 'sensor5vcal' = (Vdd/V5volt) * (ADCsum[5v supply]/(4095*numdmaseq)
 where numdmaseq = number of ADC scans per 1/2 dma (i.e. number readings in sum)

Ratiometric 5v sensor calibration:
- With 5v supply calibration compiled-in:
- Measure Sensor voltage: Vx
- Display:
  . ADCsum[sensor]
  . Va = (V5 * ratio)/ADCsum[5v]
Compute
  Calibration ratio = ADCsum[sensor]/Va

                    Min  Typ  Max 
Internal reference: 1.16 1.20 1.24 V
*/

#ifndef __ADCPARAMS
#define __ADCPARAMS

#include "iir_f1.h"
#include "iir_f2.h"

#define ADC1DMANUMSEQ        16 // Number of DMA scan sequences in 1/2 DMA buffer
#define ADC1IDX_ADCSCANSIZE   6 // Number ADC channels read
#define ADCSCALE          65536 // ADC integer scaling

/* ADC reading sequence/array indices                         */
/* These indices -=>MUST<= match the hardware ADC scan sequence    */
#define ADC1IDX_5VOLTSUPPLY   0   // PA0 IN0  - 5V sensor supply
#define ADC1IDX_CURRENTTOTAL  1   // PA5 IN5  - Current sensor: total battery current
#define ADC1IDX_CURRENTMOTOR  2   // PA6 IN6  - Current sensor: motor
#define ADC1IDX_12VRAWSUPPLY  3   // PA7 IN7  - +12 Raw power to board
#define ADC1IDX_INTERNALTEMP  4   // IN17     - Internal temperature sensor
#define ADC1IDX_INTERNALVREF  5   // IN18     - Internal voltage reference

/* Calibration option.                                    */
/* Calibration is applied after compensation adjustments. */
#define ADC1PARAM_CALIBTYPE_RAW_F  0    // No calibration applied: FLOAT
#define ADC1PARAM_CALIBTYPE_OFSC   1    // Offset & scale (poly ord 0 & 1): FLOAT
#define ADC1PARAM_CALIBTYPE_POLY2  2    // Polynomial 2nd ord: FLOAT
#define ADC1PARAM_CALIBTYPE_POLY3  3    // Polynomial 3nd ord: FLOAT
#define ADC1PARAM_CALIBTYPE_RAW_UI 4    // No calibration applied: UNSIGNED INT

/* Compensation type                                         */
/* Assumes 5v sensor supply is measured with an ADC channel. */
#define ADC1PARAM_COMPTYPE_NONE      0     // No supply or temp compensation applied
#define ADC1PARAM_COMPTYPE_RATIOVDD  1     // Vdd (3.3v nominal) ratiometric
#define ADC1PARAM_COMPTYPE_RATIO5V   2     // 5v ratiometric with 5->Vdd measurement
#define ADC1PARAM_COMPTYPE_RATIO5VNO 3     // 5v ratiometric without 5->Vdd measurement
#define ADC1PARAM_COMPTYPE_VOLTVDD   4     // Vdd (absolute), Vref compensation applied
#define ADC1PARAM_COMPTYPE_VOLTVDDNO 5     // Vdd (absolute), no Vref compensation applied
#define ADC1PARAM_COMPTYPE_VOLTV5    6     // 5v (absolute), with 5->Vdd measurement applied
#define ADC1PARAM_COMPTYPE_VOLTV5NO  7     // 5v (absolute), without 5->Vdd measurement applied

/* Filter type codes */
#define ADCFILTERTYPE_NONE		0  // Skip filtering
#define ADCFILTERTYPE_IIR1		1  // IIR single pole
#define ADCFILTERTYPE_IIR2		2  // IIR second order

/* Calibrated ADC reading. */
union ADCCALREADING
{
	uint32_t ui;
	 int32_t  n;
	float     f;
};

/*
// IIR filter (int) parameters
struct IIR_L_PARAM
{
	int32_t k;		// Filter parameter for setting time constant
	int32_t scale;		// Scaling improve spare bits with integer math
};

struct IIRFILTERL
{
	double d_out;		// output as double
	int32_t z;		// Z^(-1)
	float f_out;		// output as float
	struct IIR_L_PARAM* pprm; // Pointer to k and scale for this filter
	uint8_t sw;		// Init switch
};
*/


/* ADC parameters (for one channel): initialized either 
     from 'adcparamsinit.c' or high flash. */
struct ADCCHANNEL	
{
	union ADCCALREADING readcal; // Calibrated, not filtered
	union ADCCALREADING readfilt;// Calibrated and filtered
	struct IIR_L_PARAM iirks; // iir: k and scale
	struct IIRFILTERL iir;    // Intermediate filter params
	float	   foffset;         // Offset: float
	float	   fscale;          // Scale:  float
	int32_t  ioffset;         // Offset: scaled int
   int32_t  iscale;          // Scale:  scaled int
	uint16_t sum;             // Sum of 1/2 DMA buffer
	uint8_t  filttype;        // Type of result filtering
	uint8_t  calibtype;       // Calibration type
	uint8_t  comptype;        // Compensation type
};

/* struct allows pointer to access raw and calibrated ADC1 data. */
struct ADCDATA
{
  struct ADCCHANNEL	chan; // Everything for the channel
  uint32_t           ctr;  // Running count of updates.
};

/* *************************************************************************/
void adcparams_init(void);
/*	@brief	: Copy parameters into structs
 * NOTE: => ASSUMES ADC1 ONLY <==
 * *************************************************************************/
void adcparams_internal(struct ADCCALCOMMON* pacom, struct ADC1DATA* padc1);
/*	@brief	: Update values used for compensation from Vref and Temperature
 * @param	: pacom = Pointer calibration parameters for Temperature and Vref
 * @param	: padc1 = Pointer to array of ADC reading sums plus other stuff
 * *************************************************************************/
void adcparams_chan(uint8_t adcidx);
/*	@brief	: calibration, compensation, filtering for channels
 * @param	: adcidx = index into ADC1 array
 * *************************************************************************/

/* Raw and calibrated ADC1 readings. */
extern struct ADC1DATA adc1data;

/* Calibration values common to all ADC modules. */
extern struct ADCCALCOMMON adcommon;

#endif
