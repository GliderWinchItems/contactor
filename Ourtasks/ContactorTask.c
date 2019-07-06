/******************************************************************************
* File Name          : ContactorTask.C
* Date First Issued  : 06/18/2019
* Description        : Contactor function w STM32CubeMX w FreeRTOS
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

#define SWTIM1 500

static void contactor_idx_v_struct_hardcode_params(struct struct CONTACTORLC* p);
static void contactor_init_params(void);

uint32_t adcsumdb[6];
uint32_t adcdbctr = 0;

osThreadId ContactorTaskHandle;

struct CONTACTORFUNCTION contactorfunction;

static TimerHandle_t swtim1;

/* Task notification bit assignments. */
#define CNCTBIT00	(1 << 0)  // ADCTask has new readings
#define CNCTBIT01	(1 << 1)  // HV sensors usart RX line ready
#define CNCTBIT02	(1 << 2)  // spare
#define CNCTBIT03	(1 << 3)  // TIMER 3: uart RX keep-alive
#define CNCTBIT04	(1 << 4)  // TIMER 1: Command Keep Alive
#define CNCTBIT05	(1 << 5)  // TIMER 2: Multiple use delays
/* MailboxTask notification bits for CAN msg mailboxes */
#define CNCTBIT06	(1 << 6)  // CANID_CMD: incoming command:        cid_cmd_i 
#define CNCTBIT07	(1 << 7)  // CANID-keepalive connect command:    cid_keepalive_i
#define CNCTBIT08	(1 << 8)  // CANID-GPS time sync msg (poll msg): cid_gps_sync
/* *************************************************************************
 * void swtim1_callback(void);
 * @brief	: Software timer 1 timeout callback
 * *************************************************************************/
static void swtim1_callback(void)
{
	xTaskNotify(ContactorTaskHandle, CNCTBIT04, eSetBits);
	return;
}
/* *************************************************************************
 * void swtim2_callback(void);
 * @brief	: Software timer 2 timeout callback
 * *************************************************************************/
static void swtim2_callback(void)
{
	xTaskNotify(ContactorTaskHandle, CNCTBIT05, eSetBits);
	return;
}
/* *************************************************************************
 * void swtim3_callback(void);
 * @brief	: Software timer 3 timeout callback
 * *************************************************************************/
static void swtim3_callback(void)
{
	xTaskNotify(ContactorTaskHandle, CNCTBIT03, eSetBits);
	return;
}
/* *************************************************************************
 * osThreadId xContactorTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: ContactorTaskHandle
 * *************************************************************************/
osThreadId xContactorTaskCreate(uint32_t taskpriority)
{
 	osThreadDef(ContactorTask, StartContactorTask, osPriorityNormal, 0, 384);
	ContactorTaskHandle = osThreadCreate(osThread(ContactorTask), NULL);
	vTaskPrioritySet( ContactorTaskHandle, taskpriority );
	return ContactorTaskHandle;
}
/* *************************************************************************
 * void StartContactorTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartContactorTask(void const * argument)
{
	/* Convenience pointer. */
	struct CONTACTORFUNCTION* pcf = &contactorfunction;

	uint16_t* pdma;

	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify

	/* notification bits processed after a 'Wait. */
	uint32_t noteused = 0;

	/* Setup serial receive for uart (HV sensing) */
	struct SERIALRCVBCB* prbcb3;	// usart3

	/* Incoming ascii lines. */
	pcf->prbcb3  = xSerialTaskRxAdduart(&huart3,1,CNCTBIT01,\
		&noteval,8,8,48,0);	// 8 line buffers of 8 chars, char-by-char line mode
	if (pcf->prbcb3 == NULL) morse_trap(15);

	/* Init struct with working params */
	contactor_idx_v_struct_hardcode_params(&contactorfunction);

	/* Create timer for keep-alive.  Auto-reload/periodic */
	pcf->swtimer1 = xTimerCreate("swtim1",pdMS_TO_TICKS(pcf->keepalive_k ),pdTRUE,\
		(void *) 0, swtim1_callback);
	if (pcf->swtimer1 == NULL) {morse_trap(41);}

	/* Create timer for other delays. One-shot */
	pcf->swtimer2 = xTimerCreate("swtim2",1,pdFALSE,\
		(void *) 0, swtim2_callback);
	if (pcf->swtimer2 == NULL) {morse_trap(42);}

	/* Create timer uart RX keep-alive. One-shot */
	pcf->swtimer3 = xTimerCreate("swtim3",1,pdFALSE,\
		(void *) 0, swtim3_callback);
	if (pcf->swtimer3 == NULL) {morse_trap(43);}

	/* Start command/keep-alive timer */
	BaseType_t bret = xTimerReset(pcf->swtimer1, 10);
	if (bret != pdPass) {morse_trap(44);}

  /* Infinite loop */
  for(;;)
  {
		/* Wait for DMA interrupt notification from adctask.c */
		xTaskNotifyWait(noteused, 0, &noteval, portMAX_DELAY);
		noteused = 0;	// Accumulate bits in 'noteval' processed.
  /* ========= Events =============================== */
// NOTE: this could be made into a loop that shifts 'noteval' bits
// and calls from table of addresses.  This would have an advantage
// if the high rate bits are shifted out first since a test for
// no bits left could end the testing early.
		// Check notification and deal with it if set.
		if (noteval & CNCTBIT00)
		{ // ADC readings ready
			noteused |= CNCTBIT00; // We handled the bit
			ContactorEvents_00(pcf);
		}
		else if (noteval & CNCTBIT01)
		{ // uart RX line ready
			noteused |= CNCTBIT01; // We handled the bit
			ContactorEvents_01(pcf);
		}
		else if ((noteval & CNCTBIT02)
		{ // (spare)
			noteused |= CNCTBIT02; // We handled the bit
		}
		else if (noteval & CNCTBIT03)
		{ // TIMER3: uart RX keep alive timed out
			noteused |= CNCTBIT03; // We handled the bit
			ContactorEvents_03(pcf);			
		}
		else if (noteval & CNCTBIT04)
		{ // TIMER1: Command Keep Alive duration tick
			noteused |= CNCTBIT04; // We handled the bit
			ContactorEvents_04(pcf);
		}
		else if (noteval & CNCTBIT05)
		{ // TIMER2: Multiple use Delay timed out
			noteused |= CNCTBIT05; // We handled the bit
		}
		else if (noteval & CNCTBIT06) 
		{ // CAN: cid_cmd_i 
			noteused |= CNCTBIT06; // We handled the bit
			ContactorEvents_06(pcf);
		}
		else if (noteval & CNCTBIT07) 
		{ // CAN: cid_keepalive_i 
			noteused |= CNCTBIT07; // We handled the bit
			ContactorEvents_07(pcf);
		}
		else if (noteval & CNCTBIT08) 
		{ // CAN: cid_gps_sync 
			noteused |= CNCTBIT08; // We handled the bit
			ContactorEvents_08(pcf);
		}
  /* ========= States =============================== */
		if (pcf->evstat == CNCTEVTIMER1)	// Show that TIMER1 timed out
		{
			transition_faulting(pcf,KEEP_ALIVE_TIMER_TIMEOUT);
		}

		switch (pcf->state)
		{
		case DISCONNECTED:
			ContactorStates_disconnected(pcf);
			break;
		case CONNECTING:
			ContactorStates_connecting(pcf);
			break;
		case CONNECTED:
			ContactorStates_connected(pcf);
			break;
		case FAULTING:
			ContactorStates_faulting(pcf);
			break;
		case FAULTED:
			ContactorStates_faulted(pcf);
			break;ContactorUpdates(struct CONTACTORFUNCTION* pcf)
		case RESETTING:
			ContactorStates_resetting(pcf);
			break;
		case DISCONNECTING:
			ContactorStates_disconnecting(pcf);
			break;
		}
  /* ========= Update outputs ======================= */
		ContactorUpdates(pcf);
  }
}
/* *************************************************************************
 * static void contactor_init_params(void);
 * @brief	: init working struct.
 * *************************************************************************/
static void contactor_init_params(void)
{
	// Convenience pointers 
	struct CONTACTORFUNCTION* pcf = &contactorfunction;
	struct CONTACTORLC* plc &pcf->lc;

	/* Load parameters into working struct. */
	contactor_idx_v_struct_hardcode_params(plc);

	/* Convert float into scaled int */
	pcf->sivdiff = (plc->fvdiff * (1<<SCALE16) );
	pcf->sivlo   = (plc->fvlo   * (1<<SCALE16) );

	/* Convert ms (_t) into timer ticks (_k). */
	pcf->prechgmax_k= pdMS_TO_TICKS(plc>prechgmax_t);// allowable delay for diffafter to reach closure point (timeout delay ms)
	pcf->close1_k   = pdMS_TO_TICKS(plc>close1_t);   // contactor #1 coil energize-closure (timeout delay ms)
	pcf->close2_k   = pdMS_TO_TICKS(plc>close2_t);   // contactor #2 coil energize-closure (timeout delay ms)
	pcf->open1_k    = pdMS_TO_TICKS(plc>open1_t);    // contactor #1 coil de-energize-open (timeout delay ms)
	pcf->open2_k    = pdMS_TO_TICKS(plc>open2_t);    // contactor #2 coil de-energize-open (timeout delay ms)
	pcf->auxoc1_k	 = pdMS_TO_TICKS(plc>auxoc1_t);   // aux open/close diff from contactor coil #1 (duration ms)
	pcf->auxoc2_k	 = pdMS_TO_TICKS(plc>auxoc2_t);   // aux open/close diff from contactor coil #2 (duration ms)
	pcf->hv2stable_k= pdMS_TO_TICKS(plc>hv2stable_t);// hv 2 reading stable after closer (duration ms)
	pcf->keepalive_k= pdMS_TO_TICKS(plc>keepalive_t);// keep-alive timeout (timeout delay ms)
	pcf->hbct1_k    = pdMS_TO_TICKS(plc>hbct1_t);    // Heartbeat ct: ticks between sending msgs hv1:cur1
	pcf->hbct2_k    = pdMS_TO_TICKS(plc>hbct2_t);    // Heartbeat ct: ticks between sending msgs hv2:cur2

	/* Status bits for state machine. */
	pcf->statusbits = 0;   
	pcf->statusbits_prev = pcf->statusbits;

	/* Add CAN Mailboxes                         CAN           CAN ID              Notify bit   Paytype */
	pcf->pmbx_cid_cmd_i       =  MailboxTask_add(pctl0,plc->lc.cid_cmd_i,      NULL,CNCTBIT06,0,36);
	pcf->pmbx_cid_keepalive_i =  MailboxTask_add(pctl0,plc->lc.cid_keepalive_i,NULL,CNCTBIT07,0,23);
	pcf->pmbx_cid_gps_sync    =  MailboxTask_add(pctl0,plc->lc.cid_gps_sync,   NULL,CNCTBIT08,0,23);

	/* PWM working struct for switching PWM values */
	sConfigOCn.
	sConfigOCn.OCMode = TIM_OCMODE_PWM1;
	sConfigOCn.Pulse = 0;	// New PWM value inserted here during execution
	sConfigOCn.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOCn.OCFastMode = TIM_OCFAST_DISABLE;

	// Convert PWM as percent to timer count
   pcf->ipwmpct1 = pcf->lc.fpwmpct1 * 0.01 * (htim4.Init.Period + 1) - 1;
   pcf->ipwmpct2 = pcf->lc.fpwmpct2 * 0.01 * (htim4.Init.Period + 1) - 1;

	/* Pre-load fixed data in CAN msgs */
	for (i = 0; i < NUMCANMSGS; i++)
	{
		pcf->canmsg[i].pctl = pctl0;   // Control block for CAN module (CAN 1)
		pcf->canmsg[i].maxretryct = 8; //
		pcf->canmsg[i].bits = 0;       //
		pcf->canmsg[i].can.dlc = 8;    // Default payload size (modified when loaded and sent)
	}
	// Pre-load CAN ids
	pcf->canmsg[CID_KA_R ].can.id  = pcf->lc.cid_keepalive_r;
	pcf->canmsg[CID_MSG1 ].can.id  = pcf->lc.cid_msg1;
	pcf->canmsg[CID_MSG2 ].can.id  = pcf->lc.cid_msg2;
	pcf->canmsg[CID_CMD_R].can.id  = pcf->lc.cid_cmd_r;
	pcf->canmsg[CID_HB1  ].can.id  = pcf->lc.cid_hb1;
	pcf->canmsg[CID_HB2  ].can.id  = pcf->lc.cid_hb2;

	pcf->canmsg[0].can.dlc = 4;


	
	return 0;
}

