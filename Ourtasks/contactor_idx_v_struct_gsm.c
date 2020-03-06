/******************************************************************************
* File Name          : contactor_idx_v_struct.c
* Date First Issued  : 06/26/2019
* Board              :
* Description        : Load parameter struct 
*******************************************************************************/

#include "contactor_idx_v_struct.h"
#include "SerialTaskReceive.h"

/* *************************************************************************
 * void contactor_idx_v_struct_hardcode_params(struct struct CONTACTORLC* p);
 * @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * *************************************************************************/
void contactor_idx_v_struct_hardcode_params(struct CONTACTORLC* p)
{
	p->size       = 47;
	p->crc        = 0;   // TBD
        p->version    = 1;   // 

	/* Bits that define the hw configuration and features. */
	p->hwconfig   = 0; // Default configuration
//	p->hwconfig  |= ONECONTACTOR;   // One contactor w Pre-chg relay
	p->hwconfig  |= PWMCONTACTOR1;  // PWM coil #1
	p->hwconfig  |= PWMCONTACTOR2;  // PWM coil #2

	/* Threshold for minimum battery voltage. */
	p->fbattlow   = 9.0;  // Battery string low voltage (volts)
	// p->fbattlow   = 270.0;  // Battery string low voltage (volts)

/* Battery string current above which disconnecting is prevented. */
	p->dcurrentdisconnect = 5.5; // Disconnect threshold (amps)

	/* Timings in milliseconds. Converted later to timer ticks. */
	p->ka_t       = 1500; // Command/Keep-alive CAN msg timeout duration.
	p->ddiffb4    = 2.0;  // hv3, or (hv1-hv2) voltage across pre-charge resistor before allowing clousure of #2 contactor
	p->fdiffafter = 1.5;  // allowable (hv1-hv2) voltage difference after closure (volts)
	p->prechgmin_t= 4000; // always allow this amount of time after closing contactor #1 (timeout delay ms)
	p->prechgmax_t= 4000; // allowable delay for diffafter to reach closure point (timeout delay ms)
	p->close1_t   = 100;  // contactor #1 coil energize-closure (timeout delay ms)
	p->close2_t   = 100;  // contactor #2 coil energize-closure (timeout delay ms)
	p->open1_t    = 50;   // contactor #1 coil de-energize-open (timeout delay ms)
	p->open2_t    = 50;   // contactor #2 coil de-energize-open (timeout delay ms)
	//p->hv2stable_t= 30;   // hv 2 reading stable after closure (duration ms); possibly not used and could be removed
	//p->keepalive_t= 2555; // keep-alive timeout (timeout delay ms); possibly not used and could be removed
	p->hbct1_t    = 1000; // Heartbeat ct: ticks between sending msgs hv1:cur1
	p->hbct2_t    = 1000; // Heartbeat ct: ticks between sending msgs hv2:cur2

/* PWM durations as percent (0.0- 100.0) */
	p->fpwmpct1  = 100.0;  // Percent PWM after closure delay at 100% coil #1
	p->fpwmpct2  = 100.0;  // Percent PWM after closure delay at 100% coil #2
/*	present system uses k to divide IIR difference into integrator. The IIR sample rate is 499 Hz. Consider computing k to give set bandwidth
	where the IIR sample is computed based on the processor and its setup.	*/ 
	// Battery_minus-to-contactor #1
	p->calhv[IDXHV1].iir.k     = 3;
	p->calhv[IDXHV1].iir.scale = 2;
 	p->calhv[IDXHV1].dvcal  = 13.93; // Applied voltage
	// p->calhv[IDXHV1].adchv  = 1396;  // ADC reading (received from uart)
        p->calhv[IDXHV1].adchv  = 1700;  // ratiometric update; GSM 191214
	p->calhv[IDXHV1].offset =  3;    // ADC reading zero volts
        
	// Battery_minus-to-contactor #1 DMOC_plus
	p->calhv[IDXHV2].iir.k     = 3;
	p->calhv[IDXHV2].iir.scale = 2;
 	p->calhv[IDXHV2].dvcal  = 13.93; // Applied voltage
	// p->calhv[IDXHV2].adchv  = 1442;  // ADC reading (received from uart)
        p->calhv[IDXHV2].adchv  = 1710;  // ratiometric update; GSM 191214
	p->calhv[IDXHV2].offset =  7;    // ADC reading zero volts
        
	// Battery_minus-to-contactor #1 DMOC_minus
	p->calhv[IDXHV3].iir.k     = 3;
	p->calhv[IDXHV3].iir.scale = 2;
 	p->calhv[IDXHV3].dvcal  = 13.93; // Applied voltage
	// p->calhv[IDXHV3].adchv  = 1436;  // ADC reading (received from uart)
        p->calhv[IDXHV3].adchv  = 1700;  // GSM update to set nominal gain 191214
	p->calhv[IDXHV3].offset =  3;    // ADC reading zero volts

   //                 CANID_HEX      CANID_NAME       CAN_MSG_FMT     DESCRIPTION
	p->cid_hb1        = 0xFF800000; // CANID_HB_CNTCTR1V  : FF_FF : Contactor1: Heartbeat: High voltage1:Current sensor1
	p->cid_hb2        = 0xFF000000; // CANID_HB_CNTCTR1A  : FF_FF : Contactor1: Heartbeat: High voltage2:Current sensor2
        p->cid_msg1       = 0x50400000; // CANID_MSG_CNTCTR1V : FF_FF : Contactor1: poll response: High voltage1:Current sensor1
        p->cid_msg2       = 0x50600000; // CANID_MSG_CNTCTR1A : FF_FF : Contactor1: poll response: battery gnd to: DMOC+, DMOC-
	p->cid_cmd_r      = 0xE3600000; // CANID_CMD_CNTCTR1R : U8_VAR: Contactor1: R: Command response
	p->cid_keepalive_r= 0xE3C00000; // CANID_CMD_CNTCTRKAR: U8_U8 : Contactor1: R KeepAlive response

	// List of CAN ID's for setting up hw filter for incoming msgs
	p->cid_cmd_i        = 0xE360000C; // CANID_CMD_CNTCTR1I: U8_VAR: Contactor1: I: Command CANID incoming
	p->cid_keepalive_i  = 0xE3800000; // CANID_CMD_CNTCTRKAI:U8',    Contactor1: I KeepAlive and connect command
	p->cid_gps_sync     = 0x00400000; // CANID_HB_TIMESYNC:  U8 : GPS_1: U8 GPS time sync distribution msg-GPS time sync msg
	p->code_CAN_filt[0] = 0xFFFFFFFC; // CANID_DUMMY: UNDEF: Dummy ID: Lowest priority possible (Not Used)
	p->code_CAN_filt[1] = 0xFFFFFFFC; // CANID_DUMMY: UNDEF: Dummy ID: Lowest priority possible (Not Used)
	p->code_CAN_filt[2] = 0xFFFFFFFC; // CANID_DUMMY: UNDEF: Dummy ID: Lowest priority possible (Not Used)
	p->code_CAN_filt[3] = 0xFFFFFFFC; // CANID_DUMMY: UNDEF: Dummy ID: Lowest priority possible (Not Used)
	p->code_CAN_filt[4] = 0xFFFFFFFC; // CANID_DUMMY: UNDEF: Dummy ID: Lowest priority possible (Not Used)

	return;
}
