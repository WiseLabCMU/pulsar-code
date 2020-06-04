/*
 * discipline.c
 *
 *  Created on: Oct 29, 2016
 *      Author: adwait
 */

#include <stdio.h>

#include "pulsar_board.h"

#include "fsl_debug_console.h"

#include "shared_task_objects.h"
#include "task_info.h"

/******************************************************************************
 * DISCIPLINE section
 *****************************************************************************/

struct time_sample_struct {
	volatile uint32_t msg_no;
	volatile uint64_t beacon_tx;
	volatile uint64_t beacon_rx;
	volatile uint64_t last_resp_rx;
	volatile uint64_t next_resp_tx;
};

struct time_sample_struct sample[2];
struct time_sample_struct correction_factor;


//static const uint8_t csacTestSteer[11] = "!FD100000\r\n";
static const uint8_t csacSteerQuery[5] = "!F?\r\n";

static void apply_time_correction(volatile uint64_t *new_event, volatile uint64_t *previous_event, volatile uint64_t *correction) {
	if(*new_event + *correction < *previous_event) {
		// correction factor should be updated
		*correction = (*previous_event - *new_event) & 0xFFFFFF0000000000ULL;
		if(*new_event + *correction < *previous_event) {
			*correction += (1ULL<<40);
		}
	}
	*new_event += *correction;
}





struct SPid
{
	double dState;      	// Last position input
	double iState;      	// Integrator state
	double iMax, iMin;
	// Maximum and minimum allowable integrator state
	double	iGain,    	// integral gain
			pGain,    	// proportional gain
			dGain;     	// derivative gain
};

struct SPid csacPID = {
//		.dState = -20000000.0,
		.dState = 0,
		.iState = 0,
		.iMax = 10000000.0,
		.iMin = -10000000.0,
		.iGain = 0.1,
		.pGain = 0.1,
		.dGain = 25,
};

double UpdatePID(struct SPid *pid, double error, double position) {
	double pTerm, dTerm, iTerm;
	pTerm = pid->pGain * error;
	// calculate the proportional term
	// calculate the integral state with appropriate limiting
	pid->iState += error;
	if (pid->iState > pid->iMax) {
		pid->iState = pid->iMax;
	} else if (pid->iState < pid->iMin) {
		pid->iState = pid->iMin;
	}
	iTerm = pid->iGain * pid->iState;  // calculate the integral term
	dTerm = pid->dGain * (position - pid->dState);
	pid->dState = position;
	return (pTerm + iTerm - dTerm);
}

void discipline_task(void *pvParameters) {

	int samples_in_mem = 0;
	memset((void*) sample, 0, 2*sizeof(struct time_sample_struct)); // clear memory for samples
	memset((void*) &correction_factor, 0 , sizeof(struct time_sample_struct));	// set all
	int64_t relative_freq_err;
	uint64_t tof;
	int64_t phase_offset;
	uint64_t phase_offset_unsigned;
	long phase_offset_nsec;
	long applied_f_correction;
	bool controller_enabled = false;	// do not start the controller at the beginning
//	uint64_t diff;

	GPIO_PinInit(BOARD_PLL_IO_GPIO, BOARD_PLL_IO_TRCTL_PIN, &(gpio_pin_config_t){kGPIO_DigitalOutput, (0U)});	// initialize TRCTL pin to low as experiment marker

	while(true) {
		// wait for new values

//		PRINTF("[%u] waiting\r\n", xTaskGetTickCount());
		xSemaphoreTake(semaphore[MSG_DISC_OK_SEM], portMAX_DELAY);
//		PRINTF("[%u] getting sample\r\n", xTaskGetTickCount());

		// update our values ie take a sample
		sample[1].msg_no = rx_msg_no;
		sample[1].beacon_tx = reference_tx_timestamp;
		sample[1].beacon_rx = local_rx_timestamp;
		sample[1].last_resp_rx = previous_rx_timestamp;
		sample[1].next_resp_tx = local_tx_timestamp;

		if(samples_in_mem < 2) {
			samples_in_mem++;
			applied_f_correction = csac_get_freq_steer();	// bootstrap
		}

		apply_time_correction(&(sample[1].beacon_rx), &(sample[0].beacon_rx), &(correction_factor.beacon_rx));
		apply_time_correction(&(sample[1].last_resp_rx), &(sample[0].last_resp_rx), &(correction_factor.last_resp_rx));
		apply_time_correction(&(sample[1].next_resp_tx), &(sample[0].next_resp_tx), &(correction_factor.next_resp_tx));

//		vTaskDelay(200);

		if(samples_in_mem > 1) {

			/*
			 * Only process things if we have enough samples
			 */


			/*
			 * Frequency estimate section. In parts per 10^15
			 */
			relative_freq_err = (((int64_t)(sample[1].beacon_tx - sample[0].beacon_tx) - (int64_t)(sample[1].beacon_rx - sample[0].beacon_rx))*(1000000000000000LL))/((int64_t)(sample[1].beacon_rx - sample[0].beacon_rx));

			/*
			 * This is our control loop.
			 * currently just a simple exponential sum at alpha = 0.1
			 */
			if(!controller_enabled && (xTaskGetTickCount() > 10000)) {
				GPIO_SetPinsOutput(BOARD_PLL_IO_GPIO, 1U << BOARD_PLL_IO_TRCTL_PIN);
				controller_enabled =true;
			}

			if(controller_enabled) {
				applied_f_correction = UpdatePID(&csacPID, (double) relative_freq_err, 0.0);
			} else {
//				applied_f_correction = 20000000;
			}

			// clip correction values
			if(applied_f_correction > 20000000) {
				applied_f_correction = 20000000;
			} else if(applied_f_correction < -20000000) {
				applied_f_correction = -20000000;
			}

			/*
			 * TOF estimation section
			 */
			tof = (sample[1].beacon_rx + sample[1].last_resp_rx - sample[0].next_resp_tx - sample[1].beacon_tx)/2;
			// TODO: show 3 message sampling as well

			/*
			 * Phase estimation section
			 */
			phase_offset = ((int64_t) sample[1].beacon_tx - (int64_t) sample[1].beacon_rx) % (63897600000LL);
			phase_offset_unsigned = sample[1].beacon_tx - sample[1].beacon_rx;

			phase_offset_nsec = (long)((phase_offset*40000LL/39LL)>>16);

//			// try applying a fake correction once
//			if(!test_correction && (xTaskGetTickCount() > (120000))) {
//				csac_send(&csac_uart_handle, csacTestSteer, 11);
//
//				PRINTF("[%u] applying correction 100pp12\r\n", xTaskGetTickCount());
//				test_correction = true;
//
//			}

//			csac_get_freq_steer();

			csac_apply_absolute_steer(applied_f_correction);


//			PRINTF("[%u] size = %d: %s", xTaskGetTickCount(), query_size, csacFrequencyRequest);

			PRINTF("%u, %u, %d, %d, %d, " TS64_XSTR ", %d\r\n", xTaskGetTickCount(), sample[1].msg_no, (long) relative_freq_err, (long) tof, phase_offset_nsec, TS64_TO_TS32(phase_offset_unsigned,1), TS64_TO_TS32(phase_offset_unsigned,0) ,applied_f_correction);

//			PRINTF("[%u] we got here\r\n", xTaskGetTickCount());
		}

		/*
		 * estimations complete. updates last few values
		 */
		sample[0] = sample[1];
	}
}
