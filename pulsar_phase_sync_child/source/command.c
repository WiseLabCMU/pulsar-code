/*
 * commander.c
 *
 *  Created on: Oct 29, 2016
 *      Author: adwait
 */

#include "pulsar_board.h"

#include "fsl_debug_console.h"

#include "shared_task_objects.h"
#include "task_info.h"


/******************************************************************************
 * COMMAND TASK section
 *****************************************************************************/

/*!
 * @brief Command task. Handles inputs to/from user over UART console.
 */
void command_task(void *pvParameters) {
//	uint32_t count = 0;

	vTaskSuspend(NULL);
}

/******************************************************************************
 * end of COMMAND TASK section
 *****************************************************************************/
