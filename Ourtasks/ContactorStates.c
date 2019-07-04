/******************************************************************************
* File Name          : ContactorStates.c
* Date First Issued  : 07/01/2019
* Description        : States in Contactor function w STM32CubeMX w FreeRTOS
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

/* TIM4 CH3, CH4 drive Conatactor #1, #2 coils. */
extern TIM_HandleTypeDef htim4; // Needs this for autoreload period


/*
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
			break;
		case RESETTING:
			ContactorStates_resetting(pcf);
			break;
		case DISCONNECTING:
			ContactorStates_disconnecting(pcf);
			break;
*/
/* *************************************************************************
 * static void new_state(struct CONTACTORFUNCTION* pcf);
 * @brief	: When there is a state change, do common things
 * *************************************************************************/
static void new_state(struct CONTACTORFUNCTION* pcf, uint32_t newstate)
{
	pcf->outstat |= CNCTOUT05KA;	// Queue keep-alive status CAN msg
	pcf->state = newstate;
	return;
}
/* *************************************************************************
 * void ContactorStates_disconnected(struct CONTACTORFUNCTION* pcf);
 * @brief	: DISCONNECTED state
 * *************************************************************************/
void static transition_disconnected(struct CONTACTORFUNCTION* pcf)
{ // Intialize disconnected state
	new_state(pcf,DISCONNECTED);
	return;
}
void ContactorStates_disconnected(struct CONTACTORFUNCTION* pcf)
{
	uint32_t tmp;

	/* Check for battery string below threshold. */
	if (pcf->hv1 < pcf->ibattlow)
	{ // Here, battery voltage is too low (or missing!)
		transition_fault(pcf,BATTERYLOW);
		return;
	}
	/* Check if aux contacts match, if aux contacts present. */
	if ((pcf->lc.hwconfig & AUX1PRESENT) != 0)
	{ // Aux contacts are present
		tmp = HAL_GPIO_ReadPin(AUX1_GPIO_REG,AUX1_GPIO_IN);// read i/o pin
		if (pcf->AUX1SENSE)
		{ // Reverse sense of bit
			tmp ^= 0x1;
		}
		if (tmp & bit defined) != GPIO_PIN_RESET)
		{ // Transition to fault state; set fault code
			transition_fault(pcf,CONTACTOR1_OFF_AUX1_ON);
			return;			
		}
	}
	/* Check if aux contacts match, if aux contacts present. */
	if ((pcf->lc.hwconfig & AUX2PRESENT) != 0)
	{ // Aux contacts are present
		tmp = HAL_GPIO_ReadPin(AUX2_GPIO_REG,AUX2_GPIO_IN);// read i/o pin
		if (pcf->AUX2SENSE)
		{ // Reverse sense of bit
			tmp ^= 0x1;
		}
		if (tmp & bit defined) != GPIO_PIN_RESET)
		{ // Transition to fault state; set fault code
			transition_fault(pcf,CONTACTOR2_OFF_AUX2_ON); 
			return;			
		}
	}
	/* Command/Keep-alive CAN msg received. */
	if ((pcf->evstat & CMDCONNECT) != 0)
	{ // Here, request to connect
		transition_connecting();
		return;
	}

	/* JIC.  Be sure Updates have both coils de-energized. */
	pcf->outstat &= ~(CNCTOUT00K1 | CNCTOUT01K2 | CNCTOUT00K1w | CNCTOUT00K2w);
}
/* *************************************************************************
 * void ContactorStates_connecting(struct CONTACTORFUNCTION* pcf);
 * @brief	: CONNECTING state
 * *************************************************************************/
}/* ====================================================================== */
void static transition_connecting(struct CONTACTORFUNCTION* pcf)
{ // Intialize disconnected state

	/* Reset sub-states for connecting */
	pcf->substateC = CONNECT_C1;

	/* Set one-shot timer for contactor #1 closure delay */
	xTimerChangePeriod(pcf->swtimer2,pcf->close1_k, 2); 

	/* Energize coil #1 */
	// TIM4 CH3 PWM set to 100%, i.e. full ON
	pcf->sConfigOCn.Pulse = htim4.Init.Period+2;
	HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_3);

	pcf->outstat |= CNCTOUT00K1; // Save state of energization

	/* Update main state */
	new_state(pcf,CONNECTING);
	return;
}
/* ====================================================================== */
void ContactorStates_connecting(struct CONTACTORFUNCTION* pcf)
{
	switch(pcf->substateC)
	{
	case CONNECT_C1:  // Contactor #1 closure delay
		if ((pcf->evstat & CNCTEVTIMER2) == 0) break;
		/* Timer timed out, so contactor #1 should be closed. */

		/* Check if aux contacts match, if aux contacts present. */
		if ((pcf->lc.hwconfig & AUX1PRESENT) != 0)
		{ // Aux contacts are present
			tmp = HAL_GPIO_ReadPin(AUX1_GPIO_REG,AUX1_GPIO_IN);// read i/o pin
			if (!pcf->AUX1SENSE)
			{ // Reverse sense of bit
				tmp ^= 0x1;
			}
			if (tmp & bit defined) != GPIO_PIN_RESET)
			{ // Transition to fault state; set fault code
				/* Aux contact says it did not close. */
				transition_fault(pcf,CONTACTOR1_ON_AUX1_OFF);
				return;			
			}
		}

		/* For two contactor config, we can check if it looks closed. */
		if ((pcf->lc.hwconfig & ONECONTACTOR) != 0)
		{ // Here, two contactor config, so voltage should jump up
			if ((pcf->hv1 - pcf->hv2) < pcf->ihv1mhv2max) // 
			{
				transition_fault(pcf,CONTACTOR1_DOES_NOT_APPEAR_CLOSED); 
				return;
			}
		}	

		/* Here, looks good, so start a minimum pre-charge delay. */

		/* If this contactor is to be PWM'ed drop down from 100%. */
		if ((pcf->hwconfig & PWMCONTACTOR1) != 0)
		{ // TIM4 CH3 Lower PWM from 100%
			pcf->sConfigOCn.Pulse = pcf->ipwmpct1;
			HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_3);
			pcf->outstat |= CNCTOUT00K1w; // Save state of energization
		}

		/* Set one-shot timer for a minimum pre-charge duration. */
		xTimerChangePeriod(pcf->swtimer2,pcf->prechgmin_k, 2); 

		pcf->substateC = CONNECT_C2;
		break;
/* ...................................................................... */
	case CONNECT_C2:  // Minimum pre-charge duration delay
		if ((pcf->evstat & CNCTEVTIMER2) != 0)
		{ // Timer2 timed out before cutoff voltage reached
			pcf->evstat &= ~CNCTEVTIMER2;	// Clear timeout bit 
			transition_faulting(pcf,PRECHGVOLT_NOTREACHED);
			return;
		}
		/* Check if voltage has reached cutoff. */
		if ((pcf->evstat & CNCTEVHV) != 0)
		{ // Here, new readings available
			pcf->evstat &= ~CNCTEVHV; // Clear new reading bit

			if ((pcf->lc.hwconfig & ONECONTACTOR) == 0)
			{ // Here, one contactor
				if ((pcf->hv1 - pcf->hv2) < pcf->iprechgendv)
				{ // Here, end of pre-charge.  Energize contactor 2
					pcf->sConfigOCn.Pulse = htim4.Init.Period+2;
					HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_4);

					pcf->outstat |= CNCTOUT00K2; // Save state of energization

					/* Set one-shot timer for contactor (relay) 2 closure duration. */
					xTimerChangePeriod(pcf->swtimer2,pcf->close2_k, 2); 
					pcf->substateC = CONNECT_C3;
					return;			
				}
			}
			else
			{ // Here, two contactors
				if (pcf->hv3 < pcf->iprechgendv)
				{ // Here, end of pre-charge. Energize contactor 2
					pcf->sConfigOCn.Pulse = htim4.Init.Period+2;
					HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_4);
					pcf->outstat |= CNCTOUT00K2; // Save state of energization

					/* Set one-shot timer for contactor 2 closure duration. */
					xTimerChangePeriod(pcf->swtimer2,pcf->close2_k, 2); 

					pcf->substateC = CONNECT_C3;
					return;
				}
			}
		}
		// Here, event was not relevant
		break;
/* ...................................................................... */
	case CONNECT_C3:  // Contactor #2 close

		if ((pcf->evstat & CNCTEVTIMER2) != 0)
		{ // Timer2 timed out: Contactor #2 should be closed
			if (((pcf->hv1-pcf->hv2) > pcf->ihv1mhv2max))
			{ // Here, something not right with contactor closing
				// TODO faulting
			}
					
			/* If this contactor is to be PWM'ed drop down from 100%. */
			if ((pcf->hwconfig & PWMCONTACTOR1) != 0)
			{ // TIM4 CH3 Lower PWM from 100%
				pcf->sConfigOCn.Pulse = pcf->ipwmpct1;
				HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_3);
				pcf->outstat |= CNCTOUT00K2w; // Save state of energization
			}
			new_state(pcf,CONNECTED);
		}
		/* event not relevant. Continue waiting for timer2 */
		break;
		
/* ====================================================================== */


	}	
	return;
}
