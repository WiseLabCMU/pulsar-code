/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Fresh start.
 * Re-implement all previous systems using the correct interrupt configurations
 **/

/******************************************************************************
 * INCLUDES section
 *****************************************************************************/

#include <string.h>

#include "pulsar_board.h"
#include "pin_mux.h"
#include "clock_config.h"

/**
 * FreeRTOS includes
 */

#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "fsl_debug_console.h"
#include "fsl_uart_freertos.h"

#include "lmx2571_registers.h"
#include "deca_regs.h"
#include "csac_driver.h"
#include "pll_driver.h"
#include "deca_spi.h"
#include "deca_device_api.h"

/******************************************************************************
 * end of INCLUDES section
 *****************************************************************************/


struct TaskSharedInfo {
	volatile SemaphoreHandle_t 	*semaphores;
	volatile SemaphoreHandle_t 	*mutex;
	volatile QueueHandle_t 		*queues;
	volatile TaskHandle_t  		*tasks;
};

/******************************************************************************
 * LISTS section
 *****************************************************************************/

/*
 * List of all binary semaphores we expect to have
 */
enum SEM_ID {
	CSAC_PLL_OK_SEM 	= 0,	// indicates CSAC lock to PLL
	CSAC_PLL_FAIL_SEM 	= 1,	// indicates CSAC fail to PLL
	PLL_DW_OK_SEM 		= 2,	// indicates PLL lock to DW
	PLL_DW_FAIL_SEM 	= 3,	// indicates PLL fail to DW
	DW_MSG_OK_SEM 		= 4,	// indicates DW init to messenger
	DW_MSG_FAIL_SEM 	= 5,	// indicates DW failure to messenger
	MSG_DISC_OK_SEM 	= 6,	// indicates discipline values available
	MSG_DISC_FAIL_SEM 	= 7,	// informs discipline thread of DW failure
	DW_IRQ_SEM 			= 8,	// indicates IRQ event
	DW_PPS_SEM 			= 9,	// indicates PPS event
	SEM_N,
};

volatile SemaphoreHandle_t semaphore[SEM_N];

/*
 * List of all the tasks we expect to have
 */
enum TASK_ID {
	COMMAND_TASK 		= 0,
	HEARTBEAT_TASK 		= 1,
	CSAC_WATCHER_TASK 	= 2,
	PLL_WATCHER_TASK 	= 3,
	DW_WATCHER_TASK 	= 4,
	MESSENGER_TASK 		= 5,
	DISCIPLINE_TASK 	= 6,
	TASK_N,           //
};

volatile TaskHandle_t tasks[TASK_N];

/*
 * List of all the queues we expect to have
 */

enum QUEUE_ID {
	HEARTBEAT_Q			= 0,
	QUEUE_N,
};

volatile QueueHandle_t queues[QUEUE_N];

/******************************************************************************
 * end of LISTS section
 *****************************************************************************/



/******************************************************************************
 * INTERRUPTS section
 *****************************************************************************/

static volatile uint32_t portA_irq_reg;

void PORTA_IRQHandler(void) {
	portA_irq_reg = PORT_GetPinsInterruptFlags(PORTA);

	if(portA_irq_reg & (1U<<BOARD_CSAC_PPSOUT_PIN)) {
		xSemaphoreGiveFromISR(semaphore[DW_PPS_SEM], NULL);
		GPIO_ClearPinsInterruptFlags(GPIOA, (1U<<BOARD_CSAC_PPSOUT_PIN));
	}

}

static volatile uint32_t portC_irq_reg;

void PORTC_IRQHandler(void) {
	portC_irq_reg = PORT_GetPinsInterruptFlags(PORTC);

	if(portC_irq_reg & (1U<<BOARD_DW1000_GPIO_IRQ_PIN)) {
		xSemaphoreGiveFromISR(semaphore[DW_IRQ_SEM], NULL);
		GPIO_ClearPinsInterruptFlags(GPIOC, (1U<<BOARD_DW1000_GPIO_IRQ_PIN));
	}

}

/******************************************************************************
 * end of INTERRUPTS section
 *****************************************************************************/

/******************************************************************************
 * MAIN FUNCTION section
 *****************************************************************************/

/* Task priorities. */
#define heartbeat_PRIORITY 				(tskIDLE_PRIORITY)		// lowest priority possible = 0
#define watcher_PRIORITY				(configMAX_PRIORITIES -2) // 3
#define worker_PRIORITY					(configMAX_PRIORITIES -3) 	// 2
#define application_PRIORITY			(configMAX_PRIORITIES -2) 	// 3
#define commander_PRIORITY				(configMAX_PRIORITIES -1)	// highest priority possible = 4

/*
 * All the task definitions we have
 */
static void command_task(void *pvParameters);
static void heartbeat_task(void *pvParameters);
static void csac_watcher_task(void *pvParameters);
static void pll_watcher_task(void * pvParameters);
static void dw_watcher_task(void *pvParameters);
static void messenger_task(void *pvParameters);

/*
 * All the device data structures
 * TODO: these may be better if not static
 */
static uart_rtos_handle_t csac_uart_handle;	// CSAC UART
static dspi_rtos_handle_t pll_spi_handle;	// PLL SPI
static dspi_rtos_handle_t dw_rtos_handle;	// Deacwave SPI


/**
 * main function
 * Application entry point
 * @return 0 to comply with GCC requirements
 */
int main(void) {
  /* Init board hardware. */
	BOARD_ConfigPinmux();
	BOARD_InitPins();
	BOARD_BootClockHSRUN();
	BOARD_InitDebugConsole();

	// create semaphores
	for(int i=0; i<SEM_N;i++) {
		semaphore[i] = xSemaphoreCreateBinary();
	}

	NVIC_SetPriority(PORTA_IRQn, 5); // 5 is the max allowed interrupt priority
	NVIC_SetPriority(PORTC_IRQn, 5); // 5 is the max allowed interrupt priority
	NVIC_SetPriority(UART1_RX_TX_IRQn, 5);	// for CSAC UART interrupt
	NVIC_SetPriority(SPI1_IRQn, 5);	// for PLL SPI interrupt


	/* Create RTOS task */
	xTaskCreate(command_task, "command", configMINIMAL_STACK_SIZE, NULL, watcher_PRIORITY, tasks[COMMAND_TASK]);

	// low priority task for heartbeat LED control
	// there can only be one of these
	xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, NULL, heartbeat_PRIORITY, tasks[HEARTBEAT_TASK]);


	xTaskCreate(csac_watcher_task, "CSAC", configMINIMAL_STACK_SIZE, NULL, watcher_PRIORITY, tasks[CSAC_WATCHER_TASK]);


	xTaskCreate(pll_watcher_task, "PLL", configMINIMAL_STACK_SIZE, NULL, watcher_PRIORITY, tasks[PLL_WATCHER_TASK]);


	xTaskCreate(dw_watcher_task, "DW", configMINIMAL_STACK_SIZE, NULL, watcher_PRIORITY, tasks[DW_WATCHER_TASK]);


	xTaskCreate(messenger_task, "messenger", configMINIMAL_STACK_SIZE, NULL, application_PRIORITY, tasks[MESSENGER_TASK]);

	vTaskStartScheduler();

	for(;;) { /* Infinite loop to avoid leaving the main function */
	__asm("NOP"); /* something to use as a breakpoint stop while looping */
	}

	return 0;
}

/******************************************************************************
 * end of MAIN FUNCTION section
 *****************************************************************************/

/******************************************************************************
 * COMMAND TASK section
 *****************************************************************************/

/*!
 * @brief Task responsible for counting number of PPS pulses
 */
static void command_task(void *pvParameters) {
//	uint32_t count = 0;

	vTaskSuspend(NULL);
}

/******************************************************************************
 * end of COMMAND TASK section
 *****************************************************************************/

/******************************************************************************
 * HEARTBEAT TASK section
 *****************************************************************************/

// LED settings
#ifndef HEARTBEAT_LED_ON
#define HEARTBEAT_LED_ON() 			LED_RED_ON()
#endif	// HEARTBEAT_LED_ON

#ifndef HEARTBEAT_LED_OFF
#define HEARTBEAT_LED_OFF() 		LED_RED_OFF()
#endif // HEARTBEAT_LED_OFF

#ifndef HEARTBEAT_LED_TOGGLE
#define HEARTBEAT_LED_TOGGLE() 		LED_RED_TOGGLE()
#endif //HEARTBEAT_LED_TOGGLE

#ifndef HEARTBEAT_LED_INIT
#define HEARTBEAT_LED_INIT(output) 	LED_RED_INIT(output)
#endif // HEARTBEAT_LED_INIT

#ifndef HEARTBEAT_DEFAULT_RATE_MS
#define HEARTBEAT_DEFAULT_RATE_MS 1000
#endif // HEARTBEAT_DEFAULT_RATE_MS

/*!
 * @brief This implements a heartbeat
 */
static void heartbeat_task(void *pvParameters) {
	TickType_t xHeartbeat_period = HEARTBEAT_DEFAULT_RATE_MS/portTICK_PERIOD_MS;
	uint32_t beat_count = 0;

	HEARTBEAT_LED_INIT(LOGIC_LED_OFF);	// initialize heartbeat (red) LED as output in OFF state

	while(true) {
		beat_count++;
		HEARTBEAT_LED_ON();
		vTaskDelay((xHeartbeat_period * 10) / 100);
		HEARTBEAT_LED_OFF();
		vTaskDelay((xHeartbeat_period * 13) / 100);
		HEARTBEAT_LED_ON();
		vTaskDelay((xHeartbeat_period * 12) / 100);
		HEARTBEAT_LED_OFF();
		vTaskDelay((xHeartbeat_period * 65) / 100);	// wait for next cycle
	}
}

/******************************************************************************
 * end of HEARTBEAT TASK section
 *****************************************************************************/

/******************************************************************************
 * CSAC WATCHER section
 *****************************************************************************/
/*
 * NOTE: currently the watcher task is directly checking the csac state.
 * This function may later want to be moved to a worker task
 */

#define CSAC_LOCKED 0

static const uint8_t csacStatusRequest[4] = "!^\r\n";

int csac_get_state(uart_rtos_handle_t *rtos_uart_handle) {
	int csac_state = -1;
	bool first = true;
	size_t nRecv = 0;
	uint8_t csacRecvBuffer[1];
	size_t totalBytes = 0;
	TickType_t timeout = 20;

	// flush UART buffers
	// TODO: there should be a better way of handling this
	while(true) {
		nRecv = 0;
		if(csac_receive_timeout(rtos_uart_handle, csacRecvBuffer, 1, &nRecv, 0) != kStatus_Success) {
			break;
		}
	}

	// send telemetry request
	csac_send(rtos_uart_handle, csacStatusRequest, 4);

	// listen to UART and parse response
	while(totalBytes < 128) {	// the 128 character limit is to make sure string runoffs dont mess up communications
		nRecv = 0;

		if(csac_receive_timeout(rtos_uart_handle, csacRecvBuffer, 1, &nRecv, timeout) == kStatus_Success) {
			if(first == true) {
				csac_state = (int)csacRecvBuffer[0] - (int)'0';
				timeout = 2;
				first = false;
			}

			if(csacRecvBuffer[0] == (uint8_t) '\n') {
				break;
			}
			totalBytes++;
		} else {
			break;
		}
	}
	return csac_state;
}

static void csac_watcher_task(void *pvParameters) {
	uint8_t buf[8];

	// initialize lock pin for later use
//	GPIO_PinInit(BOARD_CSAC_GPIO, BOARD_CSAC_LOCK_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});

	// initialize UART device for communications
	csac_communication_init(&csac_uart_handle, BOARD_CSAC_UART_CLK_FREQ, buf, sizeof(buf));

	while(true) {

		if(csac_get_state(&csac_uart_handle) == CSAC_LOCKED) {
			xSemaphoreGive(semaphore[CSAC_PLL_OK_SEM]);
			PRINTF("[%u] CSAC lock detected\r\n", xTaskGetTickCount());
			vTaskSuspend(NULL);		// replace this with something nicer once failure handling is dealt with
		}

	}

	vTaskSuspend(NULL);
}

/******************************************************************************
 * end of CSAC WATCHER section
 *****************************************************************************/

/******************************************************************************
 * PLL WATCHER section
 *****************************************************************************/

/*
 * Constants
 */
static const union lmx2571_register lmx2571_reg_val[LMX2571_NREG] = {
	{.datamap = {.data = 0x4000, .ADDRESS = R60_addr, .RW = CMD_WRITE}}, // R60
	{.datamap = {.data = 0x0C00, .ADDRESS = R58_addr, .RW = CMD_WRITE}}, // R58
	{.datamap = {.data = 0x2802, .ADDRESS = R53_addr, .RW = CMD_WRITE}}, // R53
	{.datamap = {.data = 0x0000, .ADDRESS = R47_addr, .RW = CMD_WRITE}}, // R47
	{.datamap = {.data = 0x0210, .ADDRESS = R42_addr, .RW = CMD_WRITE}}, // R42
	{.datamap = {.data = 0x0810, .ADDRESS = R41_addr, .RW = CMD_WRITE}}, // R41
	{.datamap = {.data = 0x101C, .ADDRESS = R40_addr, .RW = CMD_WRITE}}, // R40
	{.datamap = {.data = 0x11F8, .ADDRESS = R39_addr, .RW = CMD_WRITE}}, // R39
	{.datamap = {.data = 0x0647, .ADDRESS = R35_addr, .RW = CMD_WRITE}}, // R35
	{.datamap = {.data = 0x1000, .ADDRESS = R34_addr, .RW = CMD_WRITE}}, // R34
	{.datamap = {.data = 0x0000, .ADDRESS = R33_addr, .RW = CMD_WRITE}}, // R33
	{.datamap = {.data = 0x0000, .ADDRESS = R32_addr, .RW = CMD_WRITE}}, // R32
	{.datamap = {.data = 0x0000, .ADDRESS = R31_addr, .RW = CMD_WRITE}}, // R31
	{.datamap = {.data = 0x0000, .ADDRESS = R30_addr, .RW = CMD_WRITE}}, // R30
	{.datamap = {.data = 0x0000, .ADDRESS = R29_addr, .RW = CMD_WRITE}}, // R29
	{.datamap = {.data = 0x0000, .ADDRESS = R28_addr, .RW = CMD_WRITE}}, // R28
	{.datamap = {.data = 0x0000, .ADDRESS = R27_addr, .RW = CMD_WRITE}}, // R27
	{.datamap = {.data = 0x0000, .ADDRESS = R26_addr, .RW = CMD_WRITE}}, // R26
	{.datamap = {.data = 0x0000, .ADDRESS = R25_addr, .RW = CMD_WRITE}}, // R25
	{.datamap = {.data = 0x0010, .ADDRESS = R24_addr, .RW = CMD_WRITE}}, // R24
	{.datamap = {.data = 0x1024, .ADDRESS = R23_addr, .RW = CMD_WRITE}}, // R23
	{.datamap = {.data = 0x8584, .ADDRESS = R22_addr, .RW = CMD_WRITE}}, // R22
	{.datamap = {.data = 0x0101, .ADDRESS = R21_addr, .RW = CMD_WRITE}}, // R21
	{.datamap = {.data = 0x1028, .ADDRESS = R20_addr, .RW = CMD_WRITE}}, // R20
	{.datamap = {.data = 0x0000, .ADDRESS = R19_addr, .RW = CMD_WRITE}}, // R19
	{.datamap = {.data = 0x0000, .ADDRESS = R18_addr, .RW = CMD_WRITE}}, // R18
	{.datamap = {.data = 0x0000, .ADDRESS = R17_addr, .RW = CMD_WRITE}}, // R17
	{.datamap = {.data = 0x0000, .ADDRESS = R16_addr, .RW = CMD_WRITE}}, // R16
	{.datamap = {.data = 0x0000, .ADDRESS = R15_addr, .RW = CMD_WRITE}}, // R15
	{.datamap = {.data = 0x0000, .ADDRESS = R14_addr, .RW = CMD_WRITE}}, // R14
	{.datamap = {.data = 0x0000, .ADDRESS = R13_addr, .RW = CMD_WRITE}}, // R13
	{.datamap = {.data = 0x0000, .ADDRESS = R12_addr, .RW = CMD_WRITE}}, // R12
	{.datamap = {.data = 0x0000, .ADDRESS = R11_addr, .RW = CMD_WRITE}}, // R11
	{.datamap = {.data = 0x0000, .ADDRESS = R10_addr, .RW = CMD_WRITE}}, // R10
	{.datamap = {.data = 0x0000, .ADDRESS = R9_addr,  .RW = CMD_WRITE}}, // R9
	{.datamap = {.data = 0x0000, .ADDRESS = R8_addr,  .RW = CMD_WRITE}}, // R8
	{.datamap = {.data = 0x0C64, .ADDRESS = R7_addr,  .RW = CMD_WRITE}}, // R7
	{.datamap = {.data = 0x1302, .ADDRESS = R6_addr,  .RW = CMD_WRITE}}, // R6
	{.datamap = {.data = 0x0201, .ADDRESS = R5_addr,  .RW = CMD_WRITE}}, // R5
	{.datamap = {.data = 0x10D7, .ADDRESS = R4_addr,  .RW = CMD_WRITE}}, // R4
	{.datamap = {.data = 0x1200, .ADDRESS = R3_addr,  .RW = CMD_WRITE}}, // R3
	{.datamap = {.data = 0xE200, .ADDRESS = R2_addr,  .RW = CMD_WRITE}}, // R2
	{.datamap = {.data = 0x7A04, .ADDRESS = R1_addr,  .RW = CMD_WRITE}}, // R1
	{.datamap = {.data = 0x0003, .ADDRESS = R0_addr,  .RW = CMD_WRITE}}, // R0
};



static void pll_watcher_task(void * pvParameters) {
	union lmx2571_register reg_local;

	while(true) {

		/*
		 * DISABLE PHASE: Hold PLL in reset.
		 * This is done by pulling the CE line to ground
		 */
		PLL_CE_INIT(PLL_CE_DISABLE);
		pll_communication_init(&pll_spi_handle, CLOCK_GetFreq(BOARD_PLL_SPI_MASTER_CLK_SRC));

		/*
		 * WAIT PHASE: Wait for CSAC lock (notification from CSAC watcher)
		 */
		xSemaphoreTake(semaphore[CSAC_PLL_OK_SEM], portMAX_DELAY);	// allow the task to block forever

		/*
		 * PROGRAM PHASE: Program the PLL with default values
		 */
		PLL_ENABLE();
		memset((void*) &reg_local, 0, PLL_REG_BYTESIZE);	// clear local register
		reg_local.R0.RESET = 1;								// set reset bit

		// soft reset the chip
		pll_register_write(&pll_spi_handle, R0_addr, reg_local.datamap.data);
		vTaskDelay(10);	// wait for stabilization

		/*
		 * PLL is reset
		 * Program all registers to desired values
		 */

		for(int i = 0; i < LMX2571_NREG; i++) {
			pll_register_write(&pll_spi_handle, lmx2571_reg_val[i].datamap.ADDRESS, lmx2571_reg_val[i].datamap.data);
			vTaskDelay(5);
		}

		/*
		 * OPERATIONAL PHASE: PLL should now be operational
		 */

		pll_communication_deinit(&pll_spi_handle);
		PORT_SetPinMux(BOARD_PLL_SPI_PORT, BOARD_PLL_SPI_MISO_PIN, kPORT_MuxAsGpio);	// PLL: MISO as GPIO

		// TODO: replace with IRQ based system
		while(!GPIO_ReadPinInput(BOARD_PLL_IO_GPIO, BOARD_PLL_SPI_MISO_PIN)) {
			vTaskDelay(100);
		}

		// PLL locked
		// inform decawave watcher immediately of lock
		xSemaphoreGive(semaphore[PLL_DW_OK_SEM]);
		PRINTF("[%u] PLL lock detected\r\n", xTaskGetTickCount());

		// suspend watcher
		vTaskSuspend(NULL);

		// if the watcher is resumed for any reason, set MISO pin back to SPI mode
		PORT_SetPinMux(BOARD_PLL_SPI_PORT, BOARD_PLL_SPI_MISO_PIN, BOARD_PLL_SPI_ALT); 	// PLL: SPI1 MISO
	}
}
/******************************************************************************
 * end of PLL WATCHER section
 *****************************************************************************/

/******************************************************************************
 * DECAWAVE WATCHER section
 *****************************************************************************/

int resync_dw_to_pps() {
	uint32_t ec_ctrl_val = 0;	// local storage for external control register

	// clear semaphore in case it was already given by a previous instance of the IRQ
	xSemaphoreTake(semaphore[DW_PPS_SEM], 0);

	// enable interrupts on pin
	PORT_SetPinInterruptConfig(BOARD_CSAC_GPIO_PORT, BOARD_CSAC_PPSOUT_PIN, kPORT_InterruptRisingEdge);

	// wait for PPS edge
	if(xSemaphoreTake(semaphore[DW_PPS_SEM], 1100) != pdTRUE) {
		PRINTF("[%u] err: unable to find 1st PPS\r\n", xTaskGetTickCount());
		return -1;
	}

	vTaskDelay(10);



	// program reset on next PPS edge
	ec_ctrl_val = EC_CTRL_OSTRM | EC_CTRL_PLLLCK | (1U << 3);
	dwt_write32bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET, ec_ctrl_val);

	// wait for next PPS edge
	if(xSemaphoreTake(semaphore[DW_PPS_SEM], 1100) != pdTRUE) {
		PRINTF("[%u] err: unable to find 2nd PPS\r\n", xTaskGetTickCount());
		return -2;
	}

	vTaskDelay(10);

	ec_ctrl_val = 0;
	dwt_write32bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET, ec_ctrl_val);

	// clear ext sync bit in DW
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ESYNCR);

	// disable PPS pin interrupt
	PORT_SetPinInterruptConfig(BOARD_CSAC_GPIO_PORT, BOARD_CSAC_PPSOUT_PIN, kPORT_InterruptOrDMADisabled);

	while(!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_CPLOCK));
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_CPLOCK);	// reset PLL lock bit in decawave

	return 0;
}

static void dw_watcher_task(void *pvParameters) {
	int sync_status;

	while(true) {
		/*
		 * Initialize SPI device
		 */
		dw_communication_slow_init(&dw_rtos_handle, CLOCK_GetFreq(BOARD_DW1000_SPI_CLKSRC));

		/*
		 * Reset DW chip by asserting the reset pin to ground
		 */
		DW1000_RST_INIT_OUT(LOGIC_DW1000_RST_ASSERT);

		/*
		 * Wait for PLL lock. The PLL watcher thread will give a semaphore once this happens
		 */
		xSemaphoreTake(semaphore[PLL_DW_OK_SEM], portMAX_DELAY);	// wait for PLL lock

		/*
		 * Start the decawave chip by de-asserting the reset line.
		 * It will go high once the device is on
		 */
		DW1000_RST_INIT_IN();	// Change the DW reset pin to input mode

		/*
		 * Wait for stabilization
		 */
		// TODO: replace with interrupt stabilization later
		while(!GPIO_ReadPinInput(BOARD_DW1000_GPIO, BOARD_DW1000_GPIO_RST_PIN));

		PRINTF("[%u] begin DW startup\r\n", xTaskGetTickCount());

		// load microcode instruction to DW
		dwt_initialise(DWT_LOADUCODE);

		/*
		 * Initialize SPI device in fast mode since microcode is now in place
		 */
		dw_communication_fast_init(&dw_rtos_handle, CLOCK_GetFreq(BOARD_DW1000_SPI_CLKSRC));

		EnableIRQ(PORTA_IRQn);		// enable IRQ on PORTA
		GPIO_PinInit(BOARD_CSAC_GPIO, BOARD_CSAC_PPSOUT_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});

		// TODO: add a loop around this for repeat requests
		while(true) {
			sync_status = resync_dw_to_pps();
			if(sync_status == 0) {
				// all good, pass control to next task
				break;
			} else {
				// we couldn't lock. try locking again
				PRINTF("[%u] unable to lock DW to PPS: %d", xTaskGetTickCount(), sync_status);
			}
		}
		PRINTF("[%u] DW clock locked to local PPS\r\n", xTaskGetTickCount());

		DisableIRQ(PORTA_IRQn);		// disable IRQ on PORTA

		xSemaphoreGive(semaphore[DW_MSG_OK_SEM]);
		vTaskSuspend(NULL);
	}
}

/******************************************************************************
 * end of DECAWAVE WATCHER section
 *****************************************************************************/

/******************************************************************************
 * MESSENGER section
 *****************************************************************************/

// ID number for node
#define NODE_ID 2
#define REFERENCE_ID	1

// Type of node
#define REFERENCE_NODE 	0
#define RELAY_NODE 		0
#define CHILD_NODE 		1

#define BEACON_SLOT 	1
#define RESPONSE_SLOT 	10

/*
 * TDMA parameters
 * Number of slots in a cycle and width of each slot determine the total cycle
 * period. Here we use 1 sec long cycles with 25 slots, each of size 40msec.
 * This may be extended all the way to 5 msec and 200 slots if desired.
 *
 */
#define N_TDMA_SLOTS	25		// 25 slots per second at gap of 40 msec
#define TDMA_WIDTH_UUS	39000	// each TDMA slot is 39000 UUS or 40000 microsec long

enum msg_type {
	BEACON_MSG_ID	= 1,
	RESPONSE_MSG_ID	= 2,
};

/*
 * Time conversions:
 * 1 UWB us (UUS)	= 1.0256 real microsec
 * 65536 DW TICKS 	= 1 UUS
 * 63897.6 DW TICKS = 1 real microsec
 */
#define UUS2DWT(uus) 	(uint64_t)((uint64_t)uus*65536) 			// UUS to DW ticks
#define SEC2DWT(sec) 	(uint64_t)((uint64_t)sec*63897600000ULL) 	// second to DW ticks
#define USEC2DWT(usec) 	(uint64_t)((uint64_t)usec*638976/10) 	// microsec to DW ticks. Has residual error
#define SEC_2_DWT 		(63897600000ULL)	// conversion factor: sec to DW ticks
#define UUS_TO_DWT_TIME 65536
#define US_TO_DWT_TIME 	63897.6 			// Warning: use this only if using floats
#define LIGHT_US 299.702547		// meter distance covered by light in 1 real microsec
#define SEC_TO_UUS 		975000


// Default antenna delay values for 64 MHz PRF
#define TX_ANT_DLY	16436		// in DW time units
#define RX_ANT_DLY	16436		// in DW time units
#define RX_TIMEOUT	8000 		// in UUS

#define RX_WINDOW_KLUDGE_UUS 	10	// kludge factor to start recieve window earlier
#define RX_TIMEOUT_KLUDGE	(8000 + RX_WINDOW_KLUDGE_UUS) // in UUS with added kludge factor

// Decawave radio configuration
static dwt_config_t channel_config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* Use non-standard SFD (Boolean) */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};


#define BEACON_SIZE 31
/*
 * beacon_message structure
 * # preamble section
 * 1: 			message type
 * 2: 			origin node ID
 * 3: 			beacon slot
 * 4,5,6,7: 	message number
 * 8,9,10,11,12,13,14,15: 	TX timestamp (in 64 bit values)
 * 16,17:		Child IDs
 * 18,19: 		Child response slots
 * 20,21,22,23,24: Child 1 RX timestamp
 * 25,26,27.28.29: Child 2 RX timestamp
 * 30,31: 		Checksum
 */

// Responses are only sent out if you have parent nodes
#define RESPONSE_SIZE 15
/*
 * response_message structure
 * # preamble section
 * 1: 			message type
 * 2: 			origin node ID
 * 3: 			response slot
 * 4:			parent node ID
 * 5,6,7,8: 	message number
 * 9,10,11,12,13: 	TX timestamp
 * 14,15: 		Checksum
 */

// these are to be used as tx and rx buffers
static volatile char beacon_message[BEACON_SIZE];
static volatile char response_message[RESPONSE_SIZE];

// converts 64 bit timestamp into 2 32 bit timestamps
// this is useful while printing
#define TS64_TO_TS32(ts64,index) ( ((uint32_t *)&ts64)[index] )

// shortcut string modifiers to print 64 bit and 40 bit timestamps easily
#define TS40_XSTR "0x%02X%08X"	// used to print 40 bit timestamps in hex
#define TS64_XSTR "0x%08X%08X" // used to print 64 bit timestamps in hex

/*
 * useful functions
 */

/**
 * Find the next acceptable DW time
 * @param last_send_time
 * @param interval_uus
 * @return
 */
static inline uint32_t compute_next_dwtime_uus(uint32_t last_send_time, uint32_t interval_uus) {
	return((last_send_time + (interval_uus<<8))&0xFFFFFFFE);
}

/**
 * Find the next send time based on value of last send time and microsecond delay.
 * This function will have a residual value due to integer division
 * @param last_send_time
 * @param usec
 * @return The next send time (in 32-bit). Note that this will have residual errors
 */
static inline uint32_t compute_next_dwtime_usec(uint32_t last_send_time, uint32_t interval_usec) {
	return ((last_send_time + ((interval_usec*1248UL)/5UL))&0xFFFFFFFE);
}

//static inline uint64_t add_corrrection_factor_ts64(uint64_t ts64, uint64_t correction) {
//	return (ts64 + correction);
//}

/**
 * Use this function while embedding timestamps in beacon messages
 * @param send_time
 * @param overflow_correction
 * @return
 */
static inline uint64_t sendtime_to_expected_timestamp(uint32_t send_time, uint64_t overflow_correction) {
	return ((( (uint64_t) (send_time & 0xFFFFFFFE) ) << 8) + TX_ANT_DLY + overflow_correction);
}

/**
 * Gets the next TDMA transmit slot approximately assuming the beacon message was sent at start of slot.
 *
 * @param rx_ts64
 * @param rx_slot		Slot number of last RX timestamp. The function assumes this lies in 1 to N_TDMA_SLOTS
 * @param desired_slot	Desired slot number. The function assumes this lies in 1 to N_TDMA_SLOTS
 * @return
 */
static inline uint32_t get_tdma_slot_time_from_rxts(uint64_t rx_ts64, int rx_slot, int desired_slot) {
	uint64_t deltaT;	// time between slots in DWT
	if(desired_slot > rx_slot) {
		deltaT = ((uint64_t)(desired_slot - rx_slot + 1)*TDMA_WIDTH_UUS)<<16;
	} else {
		// the desired slot wraps around
		deltaT = ((uint64_t)(N_TDMA_SLOTS - rx_slot + desired_slot + 1)*TDMA_WIDTH_UUS)<<16;
	}
	return ((uint32_t)((rx_ts64 + deltaT)>>8) & 0xFFFFFFFEUL);
}

static inline uint32_t get_tdma_slot_time_from_txts(uint32_t programmed_send_time, int tx_slot, int desired_slot) {
	if(desired_slot > tx_slot) {
		return ((programmed_send_time + (((uint32_t)(desired_slot - tx_slot)*TDMA_WIDTH_UUS)<<8)) & 0xFFFFFFFE);
	} else {
		return ((programmed_send_time + (((uint32_t)(N_TDMA_SLOTS - tx_slot + desired_slot)*TDMA_WIDTH_UUS)<<8)) & 0xFFFFFFFE);
	}
}

/*
 * Think of the radio as a state machine. A successful operation leads
 * to a transition to the next state
 *
 * A REFERENCE NODE has 2 states:
 * 1. Transmitting
 * 2. Receiving from RELAY
 *
 * A RELAY NODE with 2 children has 5 states:
 * 1. Listening to REFERENCE
 * 2. Transmit beacon
 * 3. Respond to REFERENCE
 * 4. Listen to 1st CHILD
 * 5. Listen to 2nd CHILD
 *
 * A CHILD NODE has 2 states:
 * 1. Listening to RELAY
 * 2. Respond to RELAY
 */

enum reference_states {
	REF_TRANSMIT_BEACON,
	REF_RECIEVE_RESP,
	REF_INVALID_MESSAGE,
	N_REF_STATES,
};

enum relay_states {
	RLY_RECIEVE_BEACON,
	RLY_TRANSMIT_BEACON,
	RLY_TRANSMIT_RESP,
	RLY_LISTEN_CHILD1,
	RLY_LISTEN_CHILD2,
	RLY_INVALID_BEACON,
	N_RELAY_STATES,
};

enum child_states {
	CH_SEARCH_BEACON,
	CH_RECEIVE_BEACON,
	CH_TRANSMIT_RESP,
	CH_INVALID_MESSAGE,
	N_CHILD_STATES,
};





/*
 * Variables that are better off being global for various reasons
 */
volatile uint32_t rx_msg_no = 0;
volatile bool rx_message_good = false;

volatile uint64_t local_rx_timestamp = 0;
volatile uint64_t local_tx_timestamp = 0;
volatile uint64_t previous_rx_timestamp = 0;

volatile uint64_t local_time_correction = 0;	// correction factor to apply
volatile uint64_t local_time;				// used to keep dw time value
volatile uint64_t last_local_time;			// used to keep dw time value

uint64_t tx_timestamp;

uint32_t programmed_send_time;		// used for storing 31 bit DW delayed TX time
uint32_t programmed_receive_time;	// used to store 31 bit DW delayed RX time

#if REFERENCE_NODE > 0
enum reference_states system_state;
#endif // REFERENCE_NODE > 0

#if RELAY_NODE > 0
enum relay_states system_state;
volatile uint64_t parent_tx_timestamp;
#endif // RELAY_NODE > 0

#if CHILD_NODE > 0
enum child_states system_state;
volatile uint64_t parent_tx_timestamp = 0;
#endif // CHILD_NODE > 0

/*
 * Callback functions
 */

// callback type for all events
typedef void (*dwt_cb_t)(const dwt_cb_data_t *);

/**
 * Sent TX frame callback function.
 * @param data
 */
void tx_good_callback(const dwt_cb_data_t *data) {
	local_tx_timestamp = 0;
	dwt_readtxtimestamp((uint8 *) &local_tx_timestamp);
}

/**
 * RX good frame received callback function. This is automatically run on
 * calling dwt_isr() after callbacks are registered
 * It will read the RX timestamp and place it in the local_rx_timestamp variable
 * @param data
 */
void rx_good_callback(const dwt_cb_data_t *data) {
	// try parsing the message

	rx_message_good = false;
//	PRINTF("[%u] RX good. length %u \r\n", xTaskGetTickCount(), data->datalength);
	if (data->datalength <= BEACON_SIZE) {

		// get timestamp
		local_rx_timestamp = 0;
		dwt_readrxtimestamp((uint8 *) &local_rx_timestamp);

		// copy data to buffer
		dwt_readrxdata((uint8 *) beacon_message, data->datalength, 0);
		// check the message belongs to parent and is of beacon type
		if((beacon_message[0] == BEACON_MSG_ID)) {	// TODO: add reference node ID based filtering here
			rx_message_good = true;
		}
	}
}

void rx_timeout_callback(const dwt_cb_data_t *data) {
	rx_message_good = false;
	PRINTF("[%u] RX timeout\r\n", xTaskGetTickCount());
}

void rx_error_callback(const dwt_cb_data_t *data) {
	rx_message_good = false;
	PRINTF("[%u] RX error\r\n", xTaskGetTickCount());
}

static void parse_good_beacon() {

}

/**
 * Simple case where we just have a REFERENCE and CHILD node.
 *
 * @param pvParamaeters
 */
static void messenger_task(void *pvParameters) {



	// wait for chip start
	xSemaphoreTake(semaphore[DW_MSG_OK_SEM], portMAX_DELAY);

	// all previous systems have now been stabilized
	// begin messaging scheme

	/*
	 * This section is for COMMON configurations that are valid regardless of node type
	 * and must be EXECUTED ONCE per DW power-on
	 */

	// Set callbacks. This applies to all cases
	dwt_setcallbacks((dwt_cb_t) tx_good_callback, (dwt_cb_t) rx_good_callback, (dwt_cb_t) rx_timeout_callback, (dwt_cb_t) rx_error_callback);
	dwt_configure(&channel_config);		// configure decawave channel parameters
	dwt_setrxantennadelay(RX_ANT_DLY);	// antenna delay value for 64 MHz PRF in DW ticks
	dwt_settxantennadelay(TX_ANT_DLY);	// antenna delay value for 64 MHz PRF in DW ticks

	/*
	 * This section contains NODE SPECIFIC initialization code that runs once per DW power-on
	 */

	// set init values for beacon message

#if REFERENCE_NODE > 0
	beacon_message[0] = BEACON_MSG_ID; 	// type of message (unchanged)
	beacon_message[1] = NODE_ID; 		// ID of beacon originator (unchanged)
	beacon_message[2] = BEACON_SLOT; 	// Which slot is this transmitting in (unchanged)
	memset((void *)&beacon_message[3], 0, 4 + 8);	// clear message number and TX timestamp (volatile)
	beacon_message[15] = 2;				// 1st child ID (unchanged)
	beacon_message[16] = 0; 			// 2nd child ID (unchanged)
	beacon_message[17] = 50; 			// 1st child response slot (unchanged)
	beacon_message[18] = 0; 			// 2nd child response slot (unchanged)
	memset((void *)&beacon_message[19], 0, 5 + 5 + 2);	// clear timestamp and checksum bits (volatile)
#endif

	// configure IRQ pin interrupt
	PORT_SetPinInterruptConfig(BOARD_DW1000_GPIO_PORT, BOARD_DW1000_GPIO_IRQ_PIN, kPORT_InterruptRisingEdge);
	EnableIRQ(PORTC_IRQn);
	GPIO_PinInit(BOARD_DW1000_GPIO, BOARD_DW1000_GPIO_IRQ_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});

#if REFERENCE_NODE > 0
	// test messaging loop
	dwt_readsystime((uint8 *) &tx_timestamp);
	programmed_send_time = (uint32_t)(tx_timestamp>>8);	// bootstrap the process with current decawave time
#endif

	/*
	 * Radio state machine starts here
	 */
	system_state = CH_SEARCH_BEACON;	// starting state

	while(true) {

		// update time at each state change
		dwt_readsystime((uint8 *) local_time);
		while(local_time + local_time_correction < last_local_time) {
			local_time_correction += (1ULL<<40);
		}
		last_local_time = local_time + local_time_correction;

#if CHILD_NODE > 0

		// begin child state machine
		switch(system_state) {

		case CH_SEARCH_BEACON:

			// continuously listen till something is heard

			// first clear old states
			// enable the right interrupts
			dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_RFTO | DWT_INT_SFDT | DWT_INT_RXPTO, (uint8_t) 1);
			xSemaphoreTake(semaphore[DW_IRQ_SEM], 0); // clear previous IRQ semaphore events
			dwt_isr(); 	// clear flags

			dwt_rxenable(DWT_START_RX_IMMEDIATE);	// start listening

			while(true) {
				if(xSemaphoreTake(semaphore[DW_IRQ_SEM], 2000) != pdTRUE) {
					rx_message_good = false;
					PRINTF("[%u] DW no reception\r\n", xTaskGetTickCount());
				} else {

					break;
				}
			}

			dwt_isr(); // process state and read timestamps etc

			// disable the used interrupts
			dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_RFTO | DWT_INT_SFDT | DWT_INT_RXPTO, (uint8_t) 0);


			if(rx_message_good) {

				// need to recover message number, parent_tx_timestamp, our previous RX timestamp from beacon message
				parent_tx_timestamp = 0;
				previous_rx_timestamp = 0;
				memcpy((void *) &rx_msg_no, (void *) &beacon_message[3], sizeof(uint32_t));
				memcpy((void *) &parent_tx_timestamp, (void *) &beacon_message[7], sizeof(uint64_t));

				// I assume we know we are child #1
				memcpy((void *) &previous_rx_timestamp, (void *) &beacon_message[19], 5);	// RX timestamp is sent back by the reference node

				PRINTF("[%u] received beacon %u: " TS64_XSTR "->" TS64_XSTR "\r\n", xTaskGetTickCount(), rx_msg_no, TS64_TO_TS32(parent_tx_timestamp,1), TS64_TO_TS32(parent_tx_timestamp,0), TS64_TO_TS32(local_rx_timestamp,1), TS64_TO_TS32(local_rx_timestamp,0));

				// TODO: pass data to discipline clock task

				// TODO: go to next state - TRANSMIT RESPONSE MESSAGE
				system_state = CH_TRANSMIT_RESP;
			} else {
				system_state = CH_SEARCH_BEACON;
			}



			break;	// end state machine SEARCH_BEACON processing

		case CH_RECEIVE_BEACON:

			// open up reception at specified time

			// first clear old states
			// enable the right interrupts
			dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_RFTO | DWT_INT_SFDT | DWT_INT_RXPTO, (uint8_t) 1);
			xSemaphoreTake(semaphore[DW_IRQ_SEM], 0); // clear previous IRQ semaphore events
			dwt_isr(); 	// clear flags

			// TODO: replace this with actual working code later
			system_state = CH_SEARCH_BEACON;
			break;

		case CH_TRANSMIT_RESP:

			// compute tx timestamp


			programmed_send_time = get_tdma_slot_time_from_rxts(local_rx_timestamp, 1, 10);
			tx_timestamp = sendtime_to_expected_timestamp(programmed_send_time, local_time_correction);

			response_message[0] = RESPONSE_MSG_ID;	// type of message
			response_message[1] = NODE_ID; 			// our node ID
			response_message[2] = 10;				// response slot
			response_message[3] = REFERENCE_ID;		// ID of parent
			memcpy((void *) &response_message[4], (void *) &rx_msg_no, sizeof(uint32));
			memcpy((void *) &response_message[8], (void *) &tx_timestamp, 5);			// copy only 5 bytes out of 8
			memset((void *) &response_message[13], 0, 2);



			dwt_writetxdata(RESPONSE_SIZE, (uint8 *) response_message, 0);
			dwt_writetxfctrl(RESPONSE_SIZE, 0, 0);

			dwt_setinterrupt(DWT_INT_TFRS, (uint8_t) 1);
			xSemaphoreTake(semaphore[DW_IRQ_SEM], 0); // clear previous IRQ semaphore events
			dwt_isr(); 	// clear flags

			dwt_setdelayedtrxtime(programmed_send_time);

			dwt_starttx(DWT_START_RX_DELAYED);

			if(xSemaphoreTake(semaphore[DW_IRQ_SEM], 1200) != pdTRUE) {
				PRINTF("[%u] Failed to send resp\r\n", xTaskGetTickCount());
				dwt_isr();		// run ISR function to complete callbacks etc
				dwt_setinterrupt(DWT_INT_TFRS, (uint8_t) 0);	// disable frame sent interrupt
				system_state = CH_SEARCH_BEACON;
			} else {

				// message sent successfully
				dwt_isr();
				dwt_setinterrupt(DWT_INT_TFRS, (uint8_t) 0);
				PRINTF("[%u] sent response %u: " TS64_XSTR"\r\n", xTaskGetTickCount(), rx_msg_no, TS64_TO_TS32(tx_timestamp,1), TS64_TO_TS32(tx_timestamp,0));
				system_state = CH_RECEIVE_BEACON;		// listen for next beacon
			}

			break;

		case CH_INVALID_MESSAGE:

			// this implies the wrong message was received
			// go back to beacon searching mode

			system_state = CH_SEARCH_BEACON;
			break;

		default:
			system_state = CH_SEARCH_BEACON;
		}

#endif // CHILD_NODE > 0

#if REFERENCE_NODE > 0
		// if node has children, it needs to send beacons and open receive window at appropriate times
		// TODO: listening to children nodes

		// construct beacon message
		beacon_msg_no++;

		programmed_send_time = compute_next_dwtime_uus(programmed_send_time, 975000); // schedule next message for 1 sec from now
		tx_timestamp = sendtime_to_expected_timestamp(programmed_send_time, tx_time_correction); // compute expected timestamp
		while(tx_timestamp < last_tx_timestamp) {
			tx_time_correction += (1ULL<<40);
			tx_timestamp = sendtime_to_expected_timestamp(programmed_send_time, tx_time_correction); // compute expected timestamp
		}

		memcpy((void *) &beacon_message[3], (void *) &beacon_msg_no, 4);
		memcpy((void *) &beacon_message[7], (void *) &tx_timestamp, 8);

		// write data to DW1000
		dwt_writetxdata(sizeof(beacon_message), (uint8_t *) beacon_message, 0);
		dwt_writetxfctrl(sizeof(beacon_message), 0, 1);
		dwt_setinterrupt(DWT_INT_TFRS, (uint8_t) 0);	// disable frame sent interrupt
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);	// clear transmit status flag
		dwt_setdelayedtrxtime(programmed_send_time);

		xSemaphoreTake(semaphore[DW_IRQ_SEM], 0); // clear previous IRQ flags

		// begin and wait for transmission
		dwt_starttx(DWT_START_TX_DELAYED);

		// wait for message or timeout
		if(xSemaphoreTake(semaphore[DW_IRQ_SEM], 1200) != pdTRUE) {
			PRINTF("[%u] Sending failed\r\n", xTaskGetTickCount());
			dwt_isr();		// run ISR function to complete callbacks etc
			tx_timestamp = 0;
			dwt_readsystime((uint8 *) &tx_timestamp);
			dwt_setinterrupt(DWT_INT_TFRS, (uint8_t) 0);	// disable frame sent interrupt

			continue;
		}
		dwt_isr();		// run ISR function to complete callbacks etc
		// TODO: check if interrupt was caused by transmit frame event
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);	// clear transmit status flag
		last_tx_timestamp = tx_timestamp;
		dwt_setinterrupt(DWT_INT_TFRS, (uint8_t) 0);	// disable frame sent interrupt
		PRINTF("[%u] Sent message %d @" TS64_XSTR "\r\n", xTaskGetTickCount(), beacon_msg_no, TS64_TO_TS32(tx_timestamp, 1), TS64_TO_TS32(tx_timestamp, 0));

#endif // REFERENCE_NODE > 0

	} // end of while loop

	// If a discipline task exists, it would stop the state machine here
}
