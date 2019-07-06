/******************************************************************************
* File Name          : contactor_cmd_msg.c
* Date First Issued  : 07/03/2019
* Description        : cid_cmd_msg_i: Function command
*******************************************************************************/

#include "adcparams.h"

static void loadhv(struct CONTACTORFUNCTION* pcf, uint8_t idx);
static void loadhv(struct CONTACTORFUNCTION* pcf, enum HVIDX idx);
static void load4(uint8_t *po, uint32_t n);

/* *************************************************************************
 * void contactor_cmd_msg_i(struct CONTACTORFUNCTION* pcf);
 *	@brief	: Given the Mailbox pointer (within CONTACTORFUNCTION) handle request
 * *************************************************************************/
void contactor_cmd_msg_i(struct CONTACTORFUNCTION* pcf)
{

/*
NOTE: The CAN msg that is loaded with data is reused for all the command msg
responses.  If for some reason the loading of the msg into the CAN hw registers
is delayed, and another command is received, the CAN msg would be overwritten.
This is highly unlikely, and if the incoming command CAN id is lower priority
than the response, then it may not be possible for the overwrite situation.
*/

/*
enum contactor_cmd_codes
{
	ADCRAW5V,         // PA0 IN0  - 5V sensor supply
	ADCRAWCUR1,       // PA5 IN5  - Current sensor: total battery current
	ADCRAWCUR2,       // PA6 IN6  - Current sensor: motor
	ADCRAW12V,        // PA7 IN7  - +12 Raw power to board
	ADCINTERNALTEMP,  // IN17     - Internal temperature sensor
	ADCINTERNALVREF,  // IN18     - Internal voltage reference
	UARTWHV1,
	UARTWHV2,
	UARTWHV3,
	CAL5V,
	CAL12V,
};

*/
static void loadadc(struct CONTACTORFUNCTION* pcf, uint8_t idx);

	uint8_t pay0 = pcf->pmbx_cid_cmd_i.ncan.cd.uc[0];
	uint16_t tmp16;

	/* Switch on first payload byte response code */
	switch (pay0)
	{
	/* ADC readings */
	case ADCRAW5V:         // PA0 IN0  - 5V sensor supply
	case ADCRAWCUR1:       // PA5 IN5  - Current sensor: total battery current
	case ADCRAWCUR2:       // PA6 IN6  - Current sensor: motor
	case ADCRAW12V:        // PA7 IN7  - +12 Raw power to board
	case ADCINTERNALTEMP:  // IN17     - Internal temperature sensor
	case ADCINTERNALVREF:  // IN18     - Internal voltage reference

			loadadc(pcf,pay0); 
			break;

	/* External uart high voltage sensor readings. */
	case UARTWHV1: loadhv(pcf,HV1); break;
	case UARTWHV2: loadhv(pcf,HV2); break;
	case UARTWHV3: loadhv(pcf,HV3); break;

	}
	// Queue CAN msg
	xQueueSendToBack(CanTxQHandle,&pcf->canmsg[CID_CMD_R].ncan,portMAX_DELAY);

}
/* *************************************************************************
 * static void load4(uint8_t *po, uint32_t n);
 *	@brief	: Copy uint32 into byte array (not aligned)
 * *************************************************************************/
static void load4(uint8_t *po, uint32_t n)
{
	*(po + 0) = (n >>  0);
	*(po + 1) = (n >>  8);
	*(po + 2) = (n >> 16);
	*(po + 3) = (n >> 24);
	return;
}
/* *************************************************************************
 * static void loadadc(struct CONTACTORFUNCTION* pcf, uint8_t idx);
 *	@brief	: Load high voltage readings and send CAN msg
 * *************************************************************************/
static void loadadc(struct CONTACTORFUNCTION* pcf, uint8_t idx)
{
	// Calibrated as a float
	union ADCCALREADING tmp;

	// Return payload request code	
	pcf->canmsg[CID_CMD_R].ncan.can.cd.uc[0] = pay0;	

	// Raw integer readings (sum of 1/2 DMA buffer)
	tmp16 = adc1chan[pay0].sum; // Get raw sum reading
	pcf->canmsg[CID_CMD_R].ncan.can.cd.uc[1] = (tmp16 >> 0);
	pcf->canmsg[CID_CMD_R].ncan.can.cd.uc[2] = (tmp16 >> 8);

	// Calibrated 
	// Load reading as a float into payload
	tmp.f = adc1chan[pay0].readcal.ui; // Convert int to float
	tmp.f += pcf->lc.fcaladc[idx].offset;
	tmp.f *= pcf->lc.fcaladc[idx].scale;
	load4(&pcf->canmsg[CID_CMD_R].ncan.can.cd.uc[3],tmp.ui);

	pcf->canmsg[CID_CMD_R].ncan.can.dlc = 7; // Number of payload bytes
	return;
}
/* *************************************************************************
 * static void loadhv(struct CONTACTORFUNCTION* pcf, uint8_t idx);
 *	@brief	: Load high voltage readings and send CAN msg
 * *************************************************************************/
static void loadhv(struct CONTACTORFUNCTION* pcf, enum HVIDX idx)
{
	// Calibrated as a float
	union ADCCALREADING tmp;

	// Return payload request code
	pcf->canmsg[CID_CMD_R].ncan.can.cd.uc[0] = pcf->pmbx_cid_cmd_i.ncan.cd.uc[0];	

	// Raw integer readings
	pcf->canmsg[CID_CMD_R].ncan.can.cd.uc[1] = (pcf->hv[idx] >> 0);
	pcf->canmsg[CID_CMD_R].ncan.can.cd.uc[2] = (pcf->hv[idx] >> 8);

	// Load reading as a float into payload
	tmp.f = pcf->hv[idx].hv; // Convert uint16_t to float
	tmp.f += pcf->lc.fcalhv[idx].offset;
	tmp.f *= pcf->lc.fcalhv[idx].scale;
	load4(&pcf->canmsg[CID_CMD_R].ncan.can.cd.uc[3],tmp.ui);

	pcf->canmsg[CID_CMD_R].ncan.can.dlc = 7; // Number of payload bytes
	return;
}

