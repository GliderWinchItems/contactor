/******************************************************************************
* File Name          : ADCTask.c
* Date First Issued  : 02/01/2019
* Description        : Processing ADC readings after ADC/DMA issues interrupt
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "ADCTask.h"
#include "adctask.h"
#include "morse.h"
#include "adcfastsum16.h"
#include "adcparams.h"

void StartADCTask(void const * argument);

uint32_t adcsumdb[6];
uint32_t adcdbctr = 0;

osThreadId ADCTaskHandle;

extern ADC_HandleTypeDef hadc1;

/* *************************************************************************
 * osThreadId xADCTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: ADCTaskHandle
 * *************************************************************************/
osThreadId xADCTaskCreate(uint32_t taskpriority)
{
 	osThreadDef(ADCTask, StartADCTask, osPriorityNormal, 0, 384);
	ADCTaskHandle = osThreadCreate(osThread(ADCTask), NULL);
	vTaskPrioritySet( ADCTaskHandle, taskpriority );
	return ADCTaskHandle;

}
/* *************************************************************************
 * void StartADCTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartADCTask(void const * argument)
{
	#define TSK02BIT02	(1 << 0)  // Task notification bit for ADC dma 1st 1/2 (adctask.c)
	#define TSK02BIT03	(1 << 1)  // Task notification bit for ADC dma end (adctask.c)

	uint16_t* pdma;

	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify

	/* notification bits processed after a 'Wait. */
	uint32_t noteused = 0;

	/* Get buffers, "our" control block, and ==>START<== ADC/DMA running. */
	struct ADCDMATSKBLK* pblk = adctask_init(&hadc1,TSK02BIT02,TSK02BIT03,&noteval);
	if (pblk == NULL) {morse_trap(15);}

  /* Infinite loop */
  for(;;)
  {
		/* Wait for DMA interrupt notification from adctask.c */
		xTaskNotifyWait(noteused, 0, &noteval, portMAX_DELAY);
		noteused = 0;	// Accumulate bits in 'noteval' processed.

		/* We handled one, or both, noteval bits */
		noteused |= (pblk->notebit1 | pblk->notebit2);

		if (noteval & TSK02BIT02)
		{
			pdma = adc1dmatskblk[0].pdma1;
		}
		else
		{
			pdma = adc1dmatskblk[0].pdma2;
		}

		/* Sum the readings 1/2 of DMA buffer to an array. */
		adcfastsum16(&adc1data.adcs1sum[0], pdma); // Fast in-line addition

		/* Save sum for defaultTask printout for debugging */
		adcsumdb[0] = adc1data.adcs1sum[0];
		adcsumdb[1] = adc1data.adcs1sum[1];
		adcsumdb[2] = adc1data.adcs1sum[2];
		adcsumdb[3] = adc1data.adcs1sum[3];
		adcsumdb[4] = adc1data.adcs1sum[4];
		adcsumdb[5] = adc1data.adcs1sum[5];
		adcdbctr += 1;

		/* Calibrate and filter ADC readings. */
		adcparams_cal();

  }
}
/* *************************************************************************
 * static void init(void);
 *	@brief	: Contactor init 
 * *************************************************************************/
static void fptoi(struct ADCCALCONTACTORINT* pi, struct ADCCALCONTACTOR* pf)
{
	pi->contactorlc.cal_hv.offset * 
}
static void init(void)
{

	struct CONTACTORFUNCTION* pfun = &contactorfunction; // Convenience pointer

	/* Load parameters into struct.*/
	adc_idx_v_struct_hardcode_params(&contactorfunction.contactorlc);

	/* Convert float parameters to scaled integers. */
		struct ADCCALCONTACTORINT ical_hv;  // High voltage calibration (offset & scale)
	struct ADCCALCONTACTORINT ical_cur; // Current calibration (offset & scale)
	struct ADCCALCONTACTORINT ical_5v;  // 5v regulated voltage 
	struct ADCCALCONTACTORINT ical_12v; // 12v raw CAN voltage

}


