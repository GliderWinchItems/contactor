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
#include "contactor_idx_v_struct.h"

#include "morse.h"

/* *************************************************************************
 * void ContactorUpdates_00(struct CONTACTORFUNCTION* pcf);
 * @brief	: ADC readings available
 * *************************************************************************/
void ContactorUpates_00(struct CONTACTORFUNCTION* pcf)
{
}

