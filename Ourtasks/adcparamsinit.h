/******************************************************************************
* File Name          : adcparamsinit.h
* Date First Issued  : 03/09/2019
* Board              : DiscoveryF4
* Description        : Initialization of parameters for ADC app configuration
*******************************************************************************/

#ifndef __ADCPARAMSINIT
#define __ADCPARAMSINIT

#include <stdint.h>
#include "adcparams.h"

#define SCALE1 (1 << 16)

/* *************************************************************************/
void adcparamsinit_init_common(struct ADCCALCOMMON* padccommon, struct ADCCHANNELSTUFF* pacsx);
/*	@brief	: Initialize struct with parameters common to all ADC for this =>board<=
 * @param	: padccommon = pointer to struct holding parameters
 * @param	: pacsx = Pointer to struct "everything" for this ADC module
 * *************************************************************************/
void adcparamsinit_init(struct ADCCHANNELSTUFF* pacsx);
/*	@brief	: Load structs for compensation, calibration and filtering all ADC channels
 * @param	: pacsx = Pointer to struct "everything" for this ADC module
 * *************************************************************************/

#endif

