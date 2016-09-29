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

/* This is a template file for pins configuration created by New Kinetis SDK 2.x Project Wizard. Enjoy! */

#include "fsl_device_registers.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "pulsar_board.h"


/* CSAC UART configuration */
static const uart_config_t csac_uart_conf = {
		.baudRate_Bps = 57600U,
		.parityMode = kUART_ParityDisabled,
#if defined(FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT) && FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT
		.stopBitCount = kUART_OneStopBit,
#endif
#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
		.txFifoWatermark = 0,
		.rxFifoWatermark = 1,
#endif
		.enableTx = true,
		.enableRx = true,
};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Initialize all pins used in this example
 */
void BOARD_ConfigPinmux(void) {

	// enable port clocks for required peripherals
	CLOCK_EnableClock(kCLOCK_PortA); // Debug Serial: UART0
	CLOCK_EnableClock(kCLOCK_PortC); // DW1000: SPI0, DW1000: GPIO, Switches: GPIO, LED: GPIO
	CLOCK_EnableClock(kCLOCK_PortD); // PLL: SPI1, PLL: GPIO
	CLOCK_EnableClock(kCLOCK_PortE); // CSAC: UART1

	// LEDs
	PORT_SetPinMux(BOARD_LED_GREEN_GPIO_PORT, BOARD_LED_GREEN_GPIO_PIN, kPORT_MuxAsGpio);	// GREEN LED: GPIO
	PORT_SetPinMux(BOARD_LED_RED_GPIO_PORT, BOARD_LED_RED_GPIO_PIN, kPORT_MuxAsGpio);		// RED LED: GPIO

	// Switches
	PORT_SetPinMux(BOARD_SW502_PORT, BOARD_SW502_GPIO_PIN, kPORT_MuxAsGpio);	// SW502: GPIO
	PORT_SetPinMux(BOARD_SW503_PORT, BOARD_SW503_GPIO_PIN, kPORT_MuxAsGpio);	// SW503: GPIO

	// DW1000 SPI0
	PORT_SetPinMux(BOARD_DW1000_SPI_PORT, BOARD_DW1000_SPI_CS_PIN, BOARD_DW1000_SPI_ALT); 		// DW1000: SPI0 CS0
	PORT_SetPinMux(BOARD_DW1000_SPI_PORT, BOARD_DW1000_SPI_SCK_PIN, BOARD_DW1000_SPI_ALT); 		// DW1000: SPI0 SCK
	PORT_SetPinMux(BOARD_DW1000_SPI_PORT, BOARD_DW1000_SPI_MOSI_PIN, BOARD_DW1000_SPI_ALT); 	// DW1000: SPI0 MOSI
	PORT_SetPinMux(BOARD_DW1000_SPI_PORT, BOARD_DW1000_SPI_MISO_PIN, BOARD_DW1000_SPI_ALT); 	// DW1000: SPI0 MISO
	PORT_SetPinMux(BOARD_DW1000_GPIO_PORT, BOARD_DW1000_GPIO_IRQ_PIN, BOARD_DW1000_GPIO_ALT); 	// DW1000: IRQ
	PORT_SetPinMux(BOARD_DW1000_GPIO_PORT, BOARD_DW1000_GPIO_SYNC_PIN, BOARD_DW1000_GPIO_ALT); 	// DW1000: SYNC
//	PORT_SetPinMux(BOARD_DW1000_GPIO_PORT, BOARD_DW1000_GPIO_WAKEUP_PIN, BOARD_DW1000_GPIO_ALT); // DW1000: WAKEUP
	PORT_SetPinMux(BOARD_DW1000_GPIO_PORT, BOARD_DW1000_GPIO_RST_PIN, BOARD_DW1000_GPIO_ALT); 	// DW1000: RESET

	// CSAC Serial
	PORT_SetPinMux(BOARD_CSAC_UART_PORT, BOARD_CSAC_UART_TX_PIN, BOARD_CSAC_UART_ALT); // CSAC Serial: UART1 TX
	PORT_SetPinMux(BOARD_CSAC_UART_PORT, BOARD_CSAC_UART_RX_PIN, BOARD_CSAC_UART_ALT); // CSAC Serial: UART1 RX

	// CSAC IO
	PORT_SetPinMux(BOARD_CSAC_GPIO_PORT, BOARD_CSAC_LOCK_PIN, kPORT_MuxAsGpio);		// CSAC LOCK: GPIO
	PORT_SetPinMux(BOARD_CSAC_GPIO_PORT, BOARD_CSAC_PPSIN_PIN, kPORT_MuxAsGpio);	// CSAC PPSIN: GPIO
	PORT_SetPinMux(BOARD_CSAC_GPIO_PORT, BOARD_CSAC_PPSOUT_PIN, kPORT_MuxAsGpio);	// CSAC PPSOUT: GPIO

	// PLL SPI
	PORT_SetPinMux(BOARD_PLL_SPI_PORT, BOARD_PLL_SPI_CS_PIN, BOARD_PLL_SPI_ALT); 	// PLL: SPI1 CS0
	PORT_SetPinMux(BOARD_PLL_SPI_PORT, BOARD_PLL_SPI_SCK_PIN, BOARD_PLL_SPI_ALT); 	// PLL: SPI1 SCK
	PORT_SetPinMux(BOARD_PLL_SPI_PORT, BOARD_PLL_SPI_MOSI_PIN, BOARD_PLL_SPI_ALT); 	// PLL: SPI1 MOSI
	PORT_SetPinMux(BOARD_PLL_SPI_PORT, BOARD_PLL_SPI_MISO_PIN, BOARD_PLL_SPI_ALT); 	// PLL: SPI1 MISO
	PORT_SetPinMux(BOARD_PLL_IO_PORT, BOARD_PLL_IO_CE_PIN, BOARD_PLL_IO_ALT); 		// PLL: CE
//	PORT_SetPinMux(BOARD_PLL_IO_PORT, BOARD_PLL_IO_TRCTL_PIN, BOARD_PLL_IO_ALT); 	// PLL: TRCTL

	// Debug Serial
	PORT_SetPinMux(BOARD_DEBUG_UART_PORT, BOARD_DEBUG_UART_TX_PIN, BOARD_DEBUG_UART_ALT); // Debug Serial: UART0 TX
	PORT_SetPinMux(BOARD_DEBUG_UART_PORT, BOARD_DEBUG_UART_RX_PIN, BOARD_DEBUG_UART_ALT); // Debug Serial: UART0 RX
}

void BOARD_InitPins(void) {
	// initialize all board LEDs as OFF
	LED_RED_INIT(LOGIC_LED_OFF);
	LED_GREEN_INIT(LOGIC_LED_OFF);

	// initialize CSAC LOCK pin as input
	GPIO_PinInit(BOARD_CSAC_GPIO, BOARD_CSAC_LOCK_PIN, &(gpio_pin_config_t){ kGPIO_DigitalInput, 0});

	// initialize UART peripheral for CSAC communication
//	UART_Init(BOARD_CSAC_UART, &csac_uart_conf, (uint32_t) BOARD_CSAC_UART_CLK_FREQ);

	// Note: debug console is initialized in pulsar_board.c

}
