/******************************************************************************
* File Name          : contactor_msgs.c
* Date First Issued  : 07/05/2019
* Description        : Setup and send non-command function CAN msgs
*******************************************************************************/
/*
SENT by contactor function:
 (1) contactor command "cid_keepalive_r" (response to "cid_keepalive_i")
     payload[0]
       bit 7 - faulted (code in payload[2])
       bit 6 - warning: minimum pre-chg immediate connect.
              (warning bit only resets with power cycle)
		 bit[0]-[3]: Current main state code

     payload[1] = critical error state error code
         0 = No fault
         1 = battery string voltage (hv1) too low
         2 = contactor 1 de-energized, aux1 closed
         3 = contactor 2 de-energized, aux2 closed
         4 = contactor #1 energized, after closure delay aux1 open
         5 = contactor #2 energized, after closure delay aux2 open
         6 = contactor #1 does not appear closed
         7 = Timeout before pre-charge voltage reached cutoff
         8 = Contactor #1 closed but voltage across it too big
         9 = Contactor #2 closed but voltage across it too big

		payload[2]
         bit[0]-[3] - current substate CONNECTING code
         bit[4]-[7] - current substate (spare) code

 poll  (response to "cid_gps_sync") & heartbeat
 (2)  "cid_msg1" hv #1 : current #1  battery string voltage:current
 (3)	"cid_msg2" hv #2 : hv #3       DMOC+:DMOC- voltages

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

 heartbeat (sent in absence of keep-alive msgs)
 (5)  "cid_hb1" Same as (2) above
 (6)  "cid_hb2" Same as (3) above
*/

#include "contactor_msgs.h"
#include "MailboxTask.h"

//#define ADDFLOATTOPAYLOAD

static void hvpayload(struct CONTACTORFUNCTION* pcf, uint8_t idx1,uint8_t idx2,uint8_t idx3);
static void load4(uint8_t *po, uint32_t n);

/* *************************************************************************
 * void contactor_msg1(struct CONTACTORFUNCTION* pcf, uint8_t w);
 *	@brief	: Setup and send responses: battery string voltage & battery current
 * @param	: pcf = Pointer to working struct for Contactor function
 * @param	: w = switch for CID_HB1 (0) or CID_MSG1 CAN ids (1)
 * *************************************************************************/
uint32_t dbgmsg1ctr;

void contactor_msg1(struct CONTACTORFUNCTION* pcf, uint8_t w)
{
	// For loading float into payload
	union UIF
	{
		uint32_t ui;
		float    f;
	}tmp;

	uint8_t idx2;

	/* Use heartbeat or polled msg CAN id */
	if (w == 0) 
		idx2 = CID_MSG1;
	else
		idx2 = CID_HB1;
#ifdef ADDFLOATTOPAYLOAD
	// Load Battery string voltage (IDXHV1) as float into payload
	pcf->hv[IDXHV1].dhvc = (double)pcf->hv[IDXHV1].dscale * (double)pcf->hv[IDXHV1].hv;
	tmp.f = pcf->hv[IDXHV1].dhvc; // Convert to float
	load4(&pcf->canmsg[idx2].can.cd.uc[0],tmp.ui); // Load float
	// Load high voltage 1 as a float into payload
	hvpayload(pcf, IDXHV1, idx2, 0);
#endif

	pcf->canmsg[idx2].can.dlc = 8;

dbgmsg1ctr += 1;

	// Queue CAN msg
	xQueueSendToBack(CanTxQHandle,&pcf->canmsg[idx2],portMAX_DELAY);
	return;

}
/* *************************************************************************
 * void contactor_msg2(struct CONTACTORFUNCTION* pcf, uint8_t w);
 *	@brief	: Setup and send responses: voltages: DMOC+, DMOC-
 * @param	: pcf = Pointer to working struct for Contactor function
 * @param	: w = switch for CID_HB1 (0) or CID_MSG1 CAN ids (1)
 * *************************************************************************/
void contactor_msg2(struct CONTACTORFUNCTION* pcf, uint8_t w)
{
	uint8_t idx2;
dbgmsg1ctr += 1;

	if (w == 0) 
		idx2 = CID_MSG2;
	else
		idx2 = CID_HB2;
#ifdef ADDFLOATTOPAYLOAD
	// Load high voltage 2 as a float into payload
	hvpayload(pcf, IDXHV2, idx2, 0);

	// Load high voltage 3 as a float into payload
	hvpayload(pcf, IDXHV3, idx2, 4);
#endif
	// Queue CAN msg
	xQueueSendToBack(CanTxQHandle,&pcf->canmsg[idx2],portMAX_DELAY);
	return;
}
/* *************************************************************************
 * static void hvpayload(struct CONTACTORFUNCTION* pcf, uint8_t idx1,uint8_t idx2,uint8_t idx3);
 *	@brief	: Setup and send responses: voltages: DMOC+, DMOC-
 * @param	: pcf = Pointer to working struct for Contactor function
 * @param	: idx1 = index into hv array (0-(NUMHV-1))
 * @param	: idx2 = index into ncan msg (0-(NUMCANMSGS-1))
 * @param	: idx3 = index into payload byte array
 * *************************************************************************/
static void hvpayload(struct CONTACTORFUNCTION* pcf, uint8_t idx1,uint8_t idx2,uint8_t idx3)
{
	union UIF
	{
		uint32_t ui;
		float    f;
	}tmp;

	// Load high voltage [idx1] as a float into payload msg [idx2] payload byte [idx3]
	pcf->hv[idx1].dhvc = (double)pcf->hv[idx1].dscale * (double)pcf->hv[idx1].hv;
	tmp.f = pcf->hv[idx1].dhvc;                       // Convert to float
	load4(&pcf->canmsg[idx2].can.cd.uc[idx3],tmp.ui); // Load payload
	return;
}
/* *************************************************************************
 * void contactor_msg_ka(struct CONTACTORFUNCTION* pcf);
 *	@brief	: Setup and send Keep-alive response
 * @param	: pcf = Pointer to working struct for Contactor function
 * *************************************************************************/
uint32_t dbgkactr;
void contactor_msg_ka(struct CONTACTORFUNCTION* pcf)
{
dbgkactr += 1;
	/* Return command byte w primary state code */
	pcf->canmsg[CID_KA_R].can.cd.uc[0]  = 
     (pcf->pmbx_cid_keepalive_i->ncan.can.cd.uc[0] & 0xf0) |
     (pcf->state & 0xf);

	/* Fault code */
	pcf->canmsg[CID_KA_R].can.cd.uc[1] = pcf->faultcode;

	/* substate codes */
	pcf->canmsg[CID_KA_R].can.cd.uc[2]  = (pcf->substateC << 0) & 0xf;
	pcf->canmsg[CID_KA_R].can.cd.uc[2] |= (pcf->substateX << 4);

	pcf->canmsg[CID_KA_R].can.dlc = 3; // Payload size

	// Queue CAN msg
	xQueueSendToBack(CanTxQHandle,&pcf->canmsg[CID_KA_R],portMAX_DELAY);
	return;
}
/* *************************************************************************
 * static void load4(uint8_t *po, uint32_t n);
 *	@brief	: Copy uint32_t into byte array (not aligned)
 * *************************************************************************/
static void load4(uint8_t *po, uint32_t n)
{
	*(po + 0) = (n >>  0);
	*(po + 1) = (n >>  8);
	*(po + 2) = (n >> 16);
	*(po + 3) = (n >> 24);
	return;
}
