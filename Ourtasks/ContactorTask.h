/******************************************************************************
* File Name          : ContactorTask.h
* Date First Issued  : 06/25/2019
* Description        : Contactor function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#ifndef __CONTACTORTASK
#define __CONTACTORTASK

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "adc_idx_v_struct.h"

/* 
=========================================      
CAN msgs: 
suffix: "_i" = incoming msg; "_r" response msg

Received CAN msgs directed into contactor function:
 (1) contactor command (also keep-alive) "cid_keepalive_i"
     payload[0]
       bit 7 - connect request
       bit 6 - reset critical error
 (2) poll (time sync) "cid_gps_sync"
 (3) function command (diagnostic poll) "cid_cmd_i"
    
Sent by contactor function:
 (1) contactor command "cid_keepalive_r" (response to "cid_keepalive_i")
     payload[0]
       bit 7 - faulted (code in payload[2])
       bit 6 - warning: minimum pre-chg immediate connect.
              (warning bit only resets with power cycle)
		 bit[0]-[3]: Current main state code

     payload[2] = critical error state error code
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

		payload[3]
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

=========================================    
NOTES:
1. The command/keep-alive msgs are sent
   - As a response to every command/keep-alive
   - Immediately when the program state changes
   - Every keep-alive timer timeout when incoming keep-alive
     msgs are not being receive, i.e. becomes a status heartbeat.

2. hv3 cannot measure negative voltages which might occur during
   regeneration with contactor #2 closed.  Likewise, hv2 can 
   exceed hv1 so that the difference becomes negative.  In both
   case the negative values would be small.
*/

/* Task notification bit assignments. */
#define CNCTBIT00	(1 << 0)  // ADCTask has new readings
#define CNCTBIT01	(1 << 1)  // HV sensors usart RX line ready
#define CNCTBIT02	(1 << 2)  // spare
#define CNCTBIT03	(1 << 3)  // TIMER 3: uart RX keep-alive
#define CNCTBIT04	(1 << 4)  // TIMER 1: Command Keep Alive
#define CNCTBIT05	(1 << 5)  // TIMER 2: Multiple use delays
// MailboxTask notification bits for CAN msg mailboxes
#define CNCTBIT06	(1 << 6)  // CANID_CMD: incoming command:        cid_cmd_i 
#define CNCTBIT07	(1 << 7)  // CANID-keepalive connect command:    cid_keepalive_i
#define CNCTBIT08	(1 << 8)  // CANID-GPS time sync msg (poll msg): cid_gps_sync

/* Event status bit assignments (CoNtaCTor EVent ....) */
#define CNCTEVTIMER1 (1 << 0) // 1 = timer1 timed out: command/keep-alive
#define CNCTEVTIMER2 (1 << 1) // 1 = timer2 timed out: delay
#define CNCTEVTIMER3 (1 << 2) // 1 = timer3 timed out: uart RX/keep-alive
#define CNCTEVCACMD  (1 << 3) // 1 = CAN rcv: general purpose command
#define CNCTEVCANKA  (1 << 4) // 1 = CAN rcv: Keep-alive/command
#define CNCTEVCAPOL  (1 << 5) // 1 = CAN rcv: Poll
#define CNCTEVCMDRS  (1 << 6) // 1 = Command to reset
#define CNCTEVCMDCN  (1 << 7) // 1 = Command to connect
#define CNCTEVHV     (1 << 8) // 1 = New HV reading

/* Output status bit assignments */
#define CNCTOUT00K1  (1 << 0) // 1 = contactor #1 energized
#define CNCTOUT01K2  (1 << 1) // 1 = contactor #2 energized
#define CNCTOUT02X1  (1 << 2) // 1 = aux #1 closed
#define CNCTOUT03X2  (1 << 3) // 1 = aux #2 closed
#define CNCTOUT04EN  (1 << 4) // 1 = DMOC enable FET
#define CNCTOUT05KA  (1 << 5) // 1 = CAN msg queue: KA status
#define CNCTOUT06KAw (1 << 6) // 1 = contactor #1 energized & pwm'ed
#define CNCTOUT07KAw (1 << 7) // 1 = contactor #2 energized & pwm'ed

/* AUX contact pins. */
#define AUX1_GPIO_REG GPIOA
#define AUX1_GPIO_IN  GPIO_PIN_1
#define AUX2_GPIO_REG GPIOA
#define AUX2_GPIO_IN  GPIO_PIN_2

/* Command request bits assignments. */
#define CMDCONNECT (1 << 7) // 1 = Connect requested; 0 = Disconnect requested
#define CMDRESET   (1 << 6) // 1 = Reset fault requested; 0 = no command

/* Number of different CAN id msgs this function sends. */
# define NUMCANMSGS 6

/* Fault codes */
enum contactor_faultcode
{
	NOFAULT = 0,
	BATTERYLOW,
	CONTACTOR1_OFF_AUX1_ON,
	CONTACTOR2_OFF_AUX2_ON,
	CONTACTOR1_ON_AUX1_OFF,
	CONTACTOR2_ON_AUX2_OFF,
	CONTACTOR1_DOES_NOT_APPEAR_CLOSED,
   PRECHGVOLT_NOTREACHED,
	CONTACTOR1_CLOSED_VOLTSTOOBIG,
	CONTACTOR2_CLOSED_VOLTSTOOBIG,
}

enum contactor_state
{
	DISCONNECTED,
	CONNECTING,
	CONNECTED,
	FAULTING,
	FAULTED,
	RESETTING,
	DISCONNECTING,
};

enum contactor_substateC
{
	CONNECTING1,
	CONNECTING2,
	CONNECTING3,
	CONNECTING4,
};

/* Function command response payload codes. */
enum contactor_cmd_codes
{
	ADCRAW5V,
	ADCRAW12V,
	ADCRAWHV1,
	ADCRAWHV2,
	ADCRAWHV3,
	ADCRAWCUR1,
	ADCRAWCUR2,
	SUPPLY5V,
	SUPPLY12V,
};


/* Working struct for Contactor function/task. */
// Prefixes: i = scaled integer, f = float
// Suffixes: k = timer ticks, t = milliseconds
struct CONTACTORFUNCTION
{
   // Parameter loaded either by high-flash copy, or hard-coded subroutine
	struct CONTACTORLC lc; // Parameters 

	/* Events status */
	uint32_t evstat;

	/* Output status */
	uint32_t outstat;
	uint32_t outstat_prev;

	/* Current fault code */
	enum contactor_faultcode faultcode;

/* In the disconnect state the battery string voltage must be above the following. */
	uint32_t ibattlow;   // Minimum battery volts required to connect

/* With two contactor config, (hv1-hv2) max when contactor #1 closes */
/* In one contactor config, (hv1-hv2) max when contactor #2 closes */
	uint32_t ihv1mhv2max;


	// Parameters converted to scaled integer or timer ticks
	uint32_t iprechgendv;// Prep-charge end volts threshold

/* Mininum pre-charge delay (befor monitoring voltage) */
   uint32_t prechgmin_k; // Minimum pre-charge duration

	uint32_t idiffafter; //  Scaled int: lc.diffafter
	uint32_t prechgmax_k;// allowable delay for diffafter to reach closure point (timeout delay ticks)
	uint32_t close1_k;   // contactor #1 coil energize-closure (timeout delay ticks)
	uint32_t close2_k;   // contactor #2 coil energize-closure (timeout delay ticks)
	uint32_t open1_k;    // contactor #1 coil de-energize-open (timeout delay ticks)
	uint32_t open2_k;    // contactor #2 coil de-energize-open (timeout delay ticks)
	uint32_t keepalive_k;// keep-alive timeout (timeout delay ticks)
	uint32_t hbct1_k;		// Heartbeat ct: ticks between sending msgs hv1:cur1
	uint32_t hbct2_k;		// Heartbeat ct: ticks between sending msgs hv2:cur2

	uint32_t ihv1;       // Latest reading: battery string at contactor #1
	uint32_t ihv2;       // Latest reading: DMOC side of contactor #1
	uint32_t ihv3;       // Latest reading: Pre-charge R, (if two contactor config)
	int32_t hv1mhv2;     // Voltage across contactor #1 from latest readings

	uint32_t statusbits;
	uint32_t statusbits_prev;

	/* Setup serial receive for uart (HV sensing) */
	struct SERIALRCVBCB* prbcb3;	// usart3

	TimerHandle_t swtimer1; // Software timer1: command/keep-alive
	TimerHandle_t swtimer2; // Software timer2: multiple purpose delay
	TimerHandle_t swtimer3; // Software timer3: uart RX/keep-alive

	/* High Voltage sensors communicating via uart. */
	uint16_t hv1; // High voltage reading: battery string side of contactor
	uint16_t hv2; // High voltage reading: DMOC+ side of contactor
	uint16_t hv3; // High voltage reading: across pre-charge resistor (two contactors)

	/* Pointers to incoming CAN msg mailboxes. */
	struct MAILBOXCAN* pmbx_cid_cmd_i;      //
	struct MAILBOXCAN* pmbx_cid_keepalive_i; //
	struct MAILBOXCAN* pmbx_cid_gps_sync;   //

	// Parameters converted from floats to scaled integers
	struct CNTCTCALSI icalcur1; // Motor current
	struct CNTCTCALSI icalcur2; // spare
	struct CNTCTCALSI icalhv1;  // Battery_minus-to-contactor #1 Battery_plus
	struct CNTCTCALSI icalhv2;  // Battery_minus-to-contactor #1 DMOC_plus
	struct CNTCTCALSI icalhv2;  // Battery_minus-to-contactor #2 DMOC_minus

	uint32_t ipwmpct1;     // Period ct PWM after closure delay at 100% coil #1
	uint32_t ipwmpct2;     // Period ct PWM after closure delay at 100% coil #2

	/* PWM struct */
	TIM_OC_InitTypeDef sConfigOCn; // 'n' - serves ch3 and ch4
	
	enum contactor_state state;         // Contactor main state
	enum contactor_substateC substateC; // State within CONNECTING

	/* CAN msgs */
	struct CANTXQMSG canmsg[5];


};

extern osThreadId ContactorTaskHandle;

#endif

