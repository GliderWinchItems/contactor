/******************************************************************************
* File Name          : contactor_hv.c
* Date First Issued  : 07/04/2019
* Description        : hv uart conversion and recalibration
*******************************************************************************/

#include "contactor_hv.h"
#include "SerialTaskReceive.h"
#include "hexbin.h"

#include <string.h>
#include "morse.h"


/* *************************************************************************
 * void contactor_hv_uartline(struct CONTACTORFUNCTION* pcf);
 * @brief	: Get & convert ascii line to binary readings
 * @return	: readings stored in contactor function struct
 * *************************************************************************/
/*
Expect 13 hex chars: AAaaBBbbCCcc<CR> and,
place binary values in uint16_t array.
*/
/* From ContactorTask.h for convenience 
struct CNCNTHV
{
	struct CNTCTCALSI ical; // Scale integer calibration values
   float fhv;     // Calibrated: Float
	uint32_t ihv;  // Calibrated: Scaled
	uint16_t hv;   // Raw reading as received from uart
};
*/
char dbgline[32];

void contactor_hv_uartline(struct CONTACTORFUNCTION* pcf)
{
	int i;
	uint8_t* pline;	// Pointer to line buffer
	do
	{
		/* Get pointer of next completed line. */
		pline = (uint8_t*)xSerialTaskReceiveGetline(pcf->prbcb3);
		if (pline != NULL)
		{ // Here, a line is ready.
strncpy(dbgline,(char*)pline,31);
			if (*(pline+12) != '\n') return; // Not correct line

			for (i = 0; i < NUMHV; i++)
			{ 
				/* Table lookup ASCII to binary: 4 asci -> uint16_t */
				pcf->hv[i].hv  = \
                (hxbn[*(pline+0)] <<  4) | \
                (hxbn[*(pline+1)] <<  0) | \
				    (hxbn[*(pline+2)] << 12) | \
                (hxbn[*(pline+3)] <<  8);
				pline += 4;
			}
		}
	} while (pline != NULL); // Catchup jic we got behind
	return;
}
/* *************************************************************************
 * void contactor_hv_calibrate(struct CONTACTORFUNCTION* pcf);
 * @brief	: Apply calibration to raw readings
 * @return	: readings stored in contactor function struct
 * *************************************************************************/
void contactor_hv_calibrate(struct CONTACTORFUNCTION* pcf)
{
	int i;
	for (i = 0; i < NUMHV; i++)
	{
		/* Volts = Volts per ADC ct * Raw ADC ticks (16b) from uart */
		pcf->hv[i].hvc = pcf->hv[i].hvcal * pcf->hv[i].hv;
	}
	return;
}
