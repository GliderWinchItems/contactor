/******************************************************************************
* File Name          : contactor_cmd_msg.c
* Date First Issued  : 07/03/2019
* Description        : cid_cmd_msg_i: Function command
*******************************************************************************/
/*
SENT by contactor function:
 function command "cid_cmd_r"(response to "cid_cmd_i")
 (4)  conditional on payload[0], for example(!)--
      - ADC ct for calibration purposes hv1
      - ADC ct for calibration purposes hv2
      - ADC ct for calibration purposes hv3
      - ADC ct for calibration purposes current1
      - ADC ct for calibration purposes current2
      - Duration: (Energize coil 1 - aux 1)
      - Duration: (Energize coil 2 - aux 2)
      - Duration: (Drop coil 1 - aux 1)
      - Duration: (Drop coil 2 - aux 2)
      - volts: 12v CAN supply
      - volts: 5v regulated supply
      ... (many and sundry)
==================================
enum CONTACTOR_CMD_CODES
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
#include "adcparams.h"
#include "CanTask.h"
#include "can_iface.h"
#include "MailboxTask.h"
#include "can_iface.h"
#include "CanTask.h"

static void loadadc(struct CONTACTORFUNCTION* pcf, uint8_t idx);
static void loadhv(struct CONTACTORFUNCTION* pcf, uint8_t idx);
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

/* Reproduced for convenience--
enum CONTACTOR_CMD_CODES
{
	ADCRAW5V,         // PA0 IN0  - 5V sensor supply
	ADCRAWCUR1,       // PA5 IN5  - Current sensor: total battery current
	ADCRAWCUR2,       // PA6 IN6  - Current sensor: motor
	ADCRAW12V,        // PA7 IN7  - +12 Raw power to board
	ADCINTERNALTEMP,  // IN17     - Internal temperature sensor
	ADCINTERNALVREF,  // IN18     - Internal voltage reference
	UARTWHV1,  // Battery voltage
	UARTWHV2,  // DMOC +
	UARTWHV3,  // DMOC -
	CAL5V,     // 5V supply
	CAL12V,    // CAN raw 12v supply
};

*/
	int i;
	uint8_t pay0 = pcf->pmbx_cid_cmd_i->ncan.can.cd.uc[0];

	// Return payload request code
	pcf->canmsg[CID_CMD_R].can.cd.uc[0] = pcf->pmbx_cid_cmd_i->ncan.can.cd.uc[0];	

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
	case UARTWHV1: loadhv(pcf,IDXHV1); break;
	case UARTWHV2: loadhv(pcf,IDXHV2); break;
	case UARTWHV3: loadhv(pcf,IDXHV3); break;

	/* Bogus code */
	default:
		for (i = 1; i < 7; i++) pcf->canmsg[CID_CMD_R].can.cd.uc[i] = 0;
		pcf->canmsg[CID_CMD_R].can.dlc = 7; // Number of payload bytes
		
	}
	// Queue CAN msg
	xQueueSendToBack(CanTxQHandle,&pcf->canmsg[CID_CMD_R],portMAX_DELAY);

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
	// For loading float into payload
	union UIF
	{
		uint32_t ui;
		float    f;
	}tmp;

	// Raw integer readings (sum of 1/2 DMA buffer)
	uint16_t tmp16 = pcf->padc->chan[idx].sum; // Get raw sum reading
	pcf->canmsg[CID_CMD_R].can.cd.uc[1] = (tmp16 >> 0);
	pcf->canmsg[CID_CMD_R].can.cd.uc[2] = (tmp16 >> 8);

	// Calibrated 
	// Load reading as a float into payload
	double dtmp  = pcf->padc->chan[idx].ival;  // Convert int to float
	       dtmp *= pcf->padc->chan[idx].dscale; // Final scaling
	tmp.f = dtmp;  // Convert double to float
	load4(&pcf->canmsg[CID_CMD_R].can.cd.uc[3],tmp.ui); // Load payload

	pcf->canmsg[CID_CMD_R].can.dlc = 7; // Number of payload bytes
	return;
}
/* *************************************************************************
 * static void loadhv(struct CONTACTORFUNCTION* pcf, uint8_t idx);
 *	@brief	: Load high voltage readings and send CAN msg
 * *************************************************************************/
static void loadhv(struct CONTACTORFUNCTION* pcf, uint8_t idx)
{
	// Calibrated as a float
	union UIF
	{
		uint32_t ui;
		float    f;
	}tmp;

	// Raw integer reading
	pcf->canmsg[CID_CMD_R].can.cd.uc[1] = (pcf->hv[idx].hv >> 0);
	pcf->canmsg[CID_CMD_R].can.cd.uc[2] = (pcf->hv[idx].hv >> 8);

	// Load reading as a float into payload
	double dtmp = pcf->hv[idx].hv; // Convert uint16_t to float
	       dtmp *= pcf->hv[idx].dscale;
   tmp.f = dtmp;
	load4(&pcf->canmsg[CID_CMD_R].can.cd.uc[3],tmp.ui);

	pcf->canmsg[CID_CMD_R].can.dlc = 7; // Number of payload bytes
	return;
}

