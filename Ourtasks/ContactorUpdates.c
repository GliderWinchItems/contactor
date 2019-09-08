/******************************************************************************
* File Name          : ContactorUpdates.c
* Date First Issued  : 07/02/2019
* Description        : Update outputs in Contactor function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "ADCTask.h"
#include "adctask.h"

#include "ContactorTask.h"
#include "contactor_idx_v_struct.h"
#include "CanTask.h"
#include "contactor_msgs.h"

#include "morse.h"

/* From 'main.c' */
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* *************************************************************************
 * void ContactorUpdates(struct CONTACTORFUNCTION* pcf);
 * @brief	: Update outputs based on bits set
 * *************************************************************************/
void ContactorUpdates(struct CONTACTORFUNCTION* pcf)
{
	/* Queue keep-alive status CAN msg */
	if ((pcf->outstat & CNCTOUT05KA) != 0)
	{
		pcf->outstat &= ~CNCTOUT05KA;	
		contactor_msg_ka(pcf);
	}

	/* Contactor #1 energization */
	if ( ((pcf->outstat & (CNCTOUT00K1 | CNCTOUT06KAw)) ^ (pcf->outstat_prev & (CNCTOUT00K1 | CNCTOUT06KAw))) != 0)
	{ // Either/both energize or pwm coil requested
		if ((pcf->outstat & CNCTOUT00K1) != 0)
		{ // Here, contactor off-->on
			if ((pcf->outstat & CNCTOUT06KAw) == 0)
			{ // Contactor ON, PWM OFF
				pcf->sConfigOCn.Pulse = (htim4.Init.Period+2); // Max+1 PWM period
				pcf->outstat_prev |= CNCTOUT00K1;
			}
			else
			{ // Contactor was on, switch to pwm
				pcf->sConfigOCn.Pulse = pcf->ipwmpct1; // PWM period for #1
				pcf->outstat_prev |= CNCTOUT06KAw;
			}
			HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);  
		}
		else
		{ // Contactor OFF; 
			pcf->sConfigOCn.Pulse = 0; // Period = 0 is OFF
			HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);  
			pcf->outstat_prev &= ~(CNCTOUT00K1 | CNCTOUT06KAw); // Reset requests
		}
	}

	/* Contactor #2 energization */
	if ( ((pcf->outstat & (CNCTOUT01K2 | CNCTOUT07KAw)) ^ (pcf->outstat_prev & (CNCTOUT01K2 | CNCTOUT07KAw))) != 0)
	{ // Either/both energize or pwm coil requested
		if ((pcf->outstat & CNCTOUT01K2) != 0)
		{  // Here, contactor off-->on
			if ((pcf->outstat & CNCTOUT07KAw) == 0)
			{ // Contactor ON, PWM OFF
				pcf->sConfigOCn.Pulse = (htim3.Init.Period+2); // Max+1 PWM period
				pcf->outstat_prev |= CNCTOUT01K2;
			}
			else
			{ // Contactor was on, switch to pwm
				pcf->sConfigOCn.Pulse = pcf->ipwmpct2; // PWM period for #2
				pcf->outstat_prev |= CNCTOUT07KAw;
			}
			HAL_TIM_PWM_ConfigChannel(&htim3, &pcf->sConfigOCn, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  
		}
		else
		{ // Contactor OFF; 
			pcf->sConfigOCn.Pulse = 0; // Period = 0
			HAL_TIM_PWM_ConfigChannel(&htim3, &pcf->sConfigOCn, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  
			pcf->outstat_prev &= ~(CNCTOUT01K2 | CNCTOUT07KAw);
		}
	}

	return;
}

