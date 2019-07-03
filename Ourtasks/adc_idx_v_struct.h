/******************************************************************************
* File Name          : adc_idx_v_struct.h
* Date First Issued  : 06/17/2019
* Board              :
* Description        : Translate parameter index into pointer into struct
*******************************************************************************/

#include <stdint.h>
#include "common_can.h"
#include "iir_filter_lx.h"

#ifndef __ADC_IDX_V_STRUCT
#define __ADC_IDX_V_STRUCT

/* Parameters for ADC reading */
// Flointing pt params are converted to scaled integers during initialization
struct ADCCALCONTACTOR
{
	double	offset;	//	Offset
	double	scale;	//	Scale
	struct IIR_L_PARAM iir; // Filter: Time constant, integer scaling
};

// ADC 
// Naming convention--"cid" - CAN ID
// LC = Local (sram) Copy of parameters
 struct ADCCONTACTORLC
 {
	uint32_t size;			// Number of items in struct
 	uint32_t crc;			// crc-32 placed by loader
	uint32_t version;		// struct version number
	uint32_t hbct;			// Heartbeat ct: ticks between sending msgs
	float vdd;           // Vrefint calibration: Vdd voltage for vrefave reading
	float vrefave;       // Vrefint calibration: Average vref ADC readings at Vdd
	struct ADCCALCONTACTOR cal_cur1; // Current calibration (offset & scale)
	struct ADCCALCONTACTOR cal_cur2; // Current calibration (offset & scale)
	struct ADCCALCONTACTOR cal_5v;   // 5v regulated voltage 
	struct ADCCALCONTACTOR cal_12v;  // 12v raw CAN voltage
 };

/* **************************************************************************************/
int adc_idx_v_struct_hardcode_params(struct CONTACTORLC* p);
/* @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * ************************************************************************************** */
 
#endif

