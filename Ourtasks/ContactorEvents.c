/******************************************************************************
* File Name          : ContactorEvents.c
* Date First Issued  : 07/01/2019
* Description        : Events in Contactor function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "ADCTask.h"
#include "adctask.h"

#include "ContactorTask.h"
#include "contactor_idx_v_struct.h"
#include "morse.h"

#include "SerialTaskReceive.h"
#include "ContactorTask.h"
#include "can_iface.h"


/* Declarations */
static void uartline(struct CONTACTORFUNCTION* pcf);

/* *************************************************************************
 * void ContactorEvents_00(struct CONTACTORFUNCTION* pcf);
 * @brief	: ADC readings available
 * *************************************************************************/
void ContactorEvents_00(struct CONTACTORFUNCTION* pcf)
{
}

/* *************************************************************************
 * void ContactorEvents_01(struct CONTACTORFUNCTION* pcf);
 * @brief	: HV sensors usart RX line ready
 * *************************************************************************/
void ContactorEvents_01(struct CONTACTORFUNCTION* pcf)
{
	uartline(pcf);  // Extract readings from received line
	xTimerReset(pcf->swtimer3,1); // Reset keep-alive timer
	pcf->evstat &= ~CNCTEVTIMER3;	// Clear timeout bit 
	pcf->evstat |= CNCTEVHV;      // Show new HV readings available
	return;
}
/* *************************************************************************
 * void ContactorEvents_02(struct CONTACTORFUNCTION* pcf);
 * @brief	: (spare)
 * *************************************************************************/
void ContactorEvents_02(struct CONTACTORFUNCTION* pcf)
{
}
/* *************************************************************************
 * void ContactorEvents_03(struct CONTACTORFUNCTION* pcf);
 * @brief	: TIMER3: uart RX keep alive failed
 * *************************************************************************/
void ContactorEvents_03(struct CONTACTORFUNCTION* pcf)
{  // Readings failed to come in before timer timed out.
	pcf->evstat |= CNCTEVTIMER3;	// Set timeout bit 
	return;
}
/* *************************************************************************
 * void ContactorEvents_04(struct CONTACTORFUNCTION* pcf);
 * @brief	: TIMER1: Command Keep Alive failed
 * *************************************************************************/
void ContactorEvents_04(struct CONTACTORFUNCTION* pcf)
{
	pcf->evstat |= CNCTEVTIMER1;	// Set timeout bit 	
	return;
}
/* *************************************************************************
 * void ContactorEvents_05(struct CONTACTORFUNCTION* pcf);
 * @brief	: TIMER2: delay ended
 * *************************************************************************/
void ContactorEvents_05(struct CONTACTORFUNCTION* pcf)
{
	pcf->evstat |= CNCTEVTIMER2;	// Set timeout bit 	
	return;
}
/* *************************************************************************
 * void ContactorEvents_06(struct CONTACTORFUNCTION* pcf);
 * @brief	: CAN: cid_cmd_i 
 * *************************************************************************/
void ContactorEvents_06(struct CONTACTORFUNCTION* pcf)
{
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
	/* Switch on first payload byte response code */
	switch (pcf->pmbx_cid_cmd_i.ncan.cd.uc[0])
	{
	case 0: // ADCRAW5V
		pcf->pmbx_cid_cmd_i.ncan.dlc = 3;
		pcf->pmbx_cid_cmd_i.ncan.cd.uc[0] = pcf->pmbx_cid_cmd_i.ncan.cd.uc[0];
// TODO		pcf->pmbx_cid_cmd_i.ncan.cd.uc[1] = pcf->
		break;


	}
}
/* *************************************************************************
 * void ContactorEvents_07(struct CONTACTORFUNCTION* pcf);
 * @brief	: CAN: cid_keepalive_i 
 * *************************************************************************/
void ContactorEvents_07(struct CONTACTORFUNCTION* pcf)
{
	/* Incoming command byte with command bits */
	uint8_t cmd = pmbx_cid_keepalive_i->ncan.can.cd.uc[0];

	if ( (cmd & CMDCONNECT) != 0) // Command to connect
	{ // Here, request to connect
		pcf->evstat |= CNCTEVCMDCN;		
	}
	else
	{
		pcf->evstat &= !CNCTEVCMDCN;		
	}
	if ( (cmd & CMDRESET ) != 0) // Command to reset
	{ // Here, request to reset
		pcf->evstat |= CMDRESET;		
	}
	else
	{
		pcf->evstat &= !CMDRESET;		
	}
	return;
}	

/* *************************************************************************
 * void ContactorEvents_08(struct CONTACTORFUNCTION* pcf);
 * @brief	: CAN: cid_gps_sync 
 * *************************************************************************/
void ContactorEvents_08(struct CONTACTORFUNCTION* pcf)
{
}




/* *************************************************************************
 * static uint8_t aschex(char* p);
 * @brief	: 
 * @return	: nibble
 * *************************************************************************/
static int8_t aschex(char* p)
{
	switch (*p)
	{
		case '0':case '1': case '2': case '3': case '4':
		case '5':case '6': case '7': case '8': case '9':
			return (*p - '0');

		case 'a':case 'b': case 'c': case 'd': case 'e': case 'f':
			return (*p - 'a' + 10);

		case 'A':case 'B': case 'C': case 'D':	case 'E': case 'F':
			return (*p - 'A' + 10);
				
		default:
			return 0;
	}
}
/* *************************************************************************
 * static void uartline(struct CONTACTORFUNCTION* pcf);
 * @brief	: Get & convert ascii line to binary readings
 * @return	: readings stored in contactor function struct
 * *************************************************************************/
/*
Expect 6 hex chars: AABBCC<CR>
*/
static void uartline(struct CONTACTORFUNCTION* pcf)
{
	char* pline;	// Pointer to line buffer
	do
	{
		/* Get pointer of next completed line. */
		pline = xSerialTaskReceiveGetline(pcf->prbcb3);
		if (pline != NULL)
		{ // Here, a line is ready.
			if (*(pline+4) != 0x0D) return 0;
			pcf->hv1  =                   aschex(pline+0);
			pcf->hv1 |= (pcf->hv1 << 4) + aschex(pline+1);
			pcf->hv2  =                   aschex(pline+2);
			pcf->hv2 |= (pcf->hv2 << 4) + aschex(pline+3);			
			pcf->hv3  =                   aschex(pline+4);
			pcf->hv3 |= (pcf->hv3 << 4) + aschex(pline+5);			
		}
	} while (pline != NULL); // Catchup jic we got behind
	return;
}
