/*
 * pulsar_board.h
 * Adwait Dongare, CMU
 * Header file defining the pins and ports on the pulsar v2 board
 */

#ifndef _PULSAR_BOARD_H_
#define _PULSAR_BOARD_H_

#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_dspi.h"
#include "fsl_port.h"
#include "fsl_uart.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The board name */
#define BOARD_NAME 					"PulsarV2"

/*! @brief The UART to use for debug messages. */
#define BOARD_USE_UART
// TODO: start using generic UART functions
#define BOARD_DEBUG_UART_TYPE 		DEBUG_CONSOLE_DEVICE_TYPE_UART 	// Console type is UART
#define BOARD_DEBUG_UART_BASEADDR 	(uint32_t) UART0 				// UART0 interface
#define BOARD_DEBUG_UART_CLKSRC 	SYS_CLK 						// Clock source: system clock
#define BOARD_DEBUG_UART_CLK_FREQ 	CLOCK_GetCoreSysClkFreq() 		// Function to get source clock frequency
#define BOARD_UART0_IRQ 			UART0_RX_TX_IRQn 				// UART0 interface interrupt vector
#define BOARD_UART0_IRQ_HANDLER 	UART0_RX_TX_IRQHandler 			// UART0 interface interrupt handler function
#define BOARD_DEBUG_UART_PORT		PORTA
#define BOARD_DEBUG_UART_ALT		kPORT_MuxAlt2
#define BOARD_DEBUG_UART_TX_PIN		1U
#define BOARD_DEBUG_UART_RX_PIN		2U

#ifndef BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_UART_BAUDRATE 	115200
#endif /* BOARD_DEBUG_UART_BAUDRATE */

// ???? define later
// /*! @brief The Flextimer instance/channel used for board */
// #define BOARD_FTM_BASEADDR FTM0
// #define BOARD_FTM_CHANNEL 5U
// #define BOARD_FTM_X_CHANNEL 0U
// #define BOARD_FTM_Y_CHANNEL 1U
// #define BOARD_FTM_PERIOD_HZ 100U
// #define BOARD_FTM_IRQ_HANDLER FTM0_IRQHandler
// #define BOARD_FTM_IRQ_VECTOR FTM0_IRQn

// ???? define later
// /*! @brief The CMP instance/channel used for board. */
// #define BOARD_CMP_BASEADDR CMP0
// #define BOARD_CMP_CHANNEL 0U

// ???? define later
// // ! @brief The rtc instance used for board. 
// #define BOARD_RTC_FUNC_BASEADDR RTC

/*! @brief Define the port interrupt number for the board switches
 *
 */

#define BOARD_DW1000_SPI_PORT 		PORTC
#define BOARD_DW1000_SPI 			SPI0
#define BOARD_DW1000_SPI_CS_PIN 	4U
#define BOARD_DW1000_SPI_SCK_PIN 	5U
#define BOARD_DW1000_SPI_MOSI_PIN 	6U
#define BOARD_DW1000_SPI_MISO_PIN 	7U
#define BOARD_DW1000_SPI_ALT 		kPORT_MuxAlt2

#define LOGIC_DW1000_RST_ASSERT 	0U
#define LOGIC_DW1000_RST_RELEASE 	1U

#define BOARD_DW1000_GPIO_PORT 			PORTC
#define BOARD_DW1000_GPIO 				GPIOC
#define BOARD_DW1000_GPIO_IRQ_PIN 		0U
#define BOARD_DW1000_GPIO_SYNC_PIN 		1U
#define BOARD_DW1000_GPIO_WAKEUP_PIN 	2U 
#define BOARD_DW1000_GPIO_RST_PIN 		3U
#define BOARD_DW1000_GPIO_ALT 			kPORT_MuxAsGpio

#define BOARD_CSAC_UART			 	UART1						// UART used for CSAC serial
#define BOARD_CSAC_UART_BASEADDR 	(uint32_t) BOARD_CSAC_UART 	//
#define BOARD_CSAC_UART_CLKSRC	 	SYS_CLK 					// clock source for CSAC UART
#define BOARD_CSAC_UART_CLK_FREQ 	CLOCK_GetCoreSysClkFreq() 	//???
#define BOARD_UART1_IRQ 			UART1_RX_TX_IRQn //???
#define BOARD_UART1_IRQ_HANDLER 	UART1_RX_TX_IRQHandler //???
#define BOARD_CSAC_UART_PORT		PORTE
#define BOARD_CSAC_UART_ALT			kPORT_MuxAlt3
#define BOARD_CSAC_UART_TX_PIN		0U
#define BOARD_CSAC_UART_RX_PIN		1U
#define BOARD_CSAC_UART_BAUD_RATE	57600
#define BOARD_CSAC_GPIO_PORT 		PORTA
#define BOARD_CSAC_GPIO 			GPIOA
#define BOARD_CSAC_LOCK_PIN 		5U
#define BOARD_CSAC_PPSIN_PIN 		12U
#define BOARD_CSAC_PPSOUT_PIN 		13U


#define DW1000_RST_INIT(output) \
	GPIO_PinInit(BOARD_DW1000_GPIO, BOARD_DW1000_GPIO_RST_PIN, \
				 &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)}) /*!< enable DW reset pin */
#define DW1000_RST_ASSERT() \
	GPIO_ClearPinsOutput(BOARD_DW1000_GPIO, 1U << BOARD_DW1000_GPIO_RST_PIN) /*!< pull DW reset pin down */
#define DW1000_RST_RELEASE() \
	GPIO_SetPinsOutput(BOARD_DW1000_GPIO, 1U << BOARD_DW1000_GPIO_RST_PIN) /*!< pull DW reset pin high */
#define DW1000_RST_TOGGLE() \
	GPIO_TogglePinsOutput(BOARD_DW1000_GPIO, 1U << BOARD_DW1000_GPIO_RST_PIN) /*!< Toggle DW reset pin */

#define BOARD_PLL_SPI_PORT 			PORTD
#define BOARD_PLL_SPI_GPIO 			GPIOD
#define BOARD_PLL_SPI_CS_PIN 		4U
#define BOARD_PLL_SPI_SCK_PIN 		5U
#define BOARD_PLL_SPI_MOSI_PIN 		6U
#define BOARD_PLL_SPI_MISO_PIN 		7U
#define BOARD_PLL_SPI_ALT 			kPORT_MuxAlt7

#define BOARD_PLL_IO_PORT 			PORTD
#define BOARD_PLL_IO_GPIO 			GPIOD
#define BOARD_PLL_IO_CE_PIN 		2U
#define BOARD_PLL_IO_TRCTL_PIN 		3U
#define BOARD_PLL_IO_ALT 			kPORT_MuxAsGpio 

#define PLL_CE_INIT(output) \
	GPIO_PinInit(BOARD_PLL_IO_GPIO, BOARD_PLL_IO_CE_PIN, \
				 &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)}) /*!< enable DW reset pin */
#define PLL_DISABLE() \
	GPIO_ClearPinsOutput(BOARD_PLL_IO_GPIO, 1U << BOARD_PLL_IO_CE_PIN) /*!< pull DW reset pin down */
#define PLL_ENABLE() \
	GPIO_SetPinsOutput(BOARD_PLL_IO_GPIO, 1U << BOARD_PLL_IO_CE_PIN) /*!< pull DW reset pin high */
#define PLL_EN_TOGGLE() \
	GPIO_TogglePinsOutput(BOARD_PLL_IO_GPIO, 1U << BOARD_PLL_IO_CE_PIN) /*!< Toggle DW reset pin */

#define BOARD_SW502_GPIO 			GPIOC
#define BOARD_SW502_PORT 			PORTC
#define BOARD_SW502_GPIO_PIN 		9U
#define BOARD_SW502_IRQ 			PORTC_IRQn
#define BOARD_SW502_IRQ_HANDLER 	PORTC_IRQHandler
#define BOARD_SW502_NAME 			"SW502"

#define BOARD_SW503_GPIO 			GPIOC
#define BOARD_SW503_PORT 			PORTC
#define BOARD_SW503_GPIO_PIN 		8U
#define BOARD_SW503_IRQ 			PORTC_IRQn
#define BOARD_SW503_IRQ_HANDLER 	PORTC_IRQHandler
#define BOARD_SW503_NAME 			"SW503"

/* @brief Board LED color mapping on Pulsar v2
 * DS501 (PTC10): Red LED
 * DS502 (PTC11): Green LED
 */
#define LOGIC_LED_ON 	1U
#define LOGIC_LED_OFF 	0U

#define BOARD_LED_RED_GPIO 			GPIOC
#define BOARD_LED_RED_GPIO_PORT 	PORTC
#define BOARD_LED_RED_GPIO_PIN 		10U

#define BOARD_LED_GREEN_GPIO 		GPIOC
#define BOARD_LED_GREEN_GPIO_PORT 	PORTC
#define BOARD_LED_GREEN_GPIO_PIN 	11U


#define LED_RED_INIT(output)                                 \
	GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, \
				 &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)}) /*!< Enable target LED_RED */
#define LED_RED_OFF() \
	GPIO_ClearPinsOutput(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN) /*!< Turn off target LED_RED */
#define LED_RED_ON() \
	GPIO_SetPinsOutput(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN) /*!< Turn on target LED_RED */
#define LED_RED_TOGGLE() \
	GPIO_TogglePinsOutput(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN) /*!< Toggle on target LED_RED */

#define LED_GREEN_INIT(output)                                   \
	GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PIN, \
				 &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)}) /*!< Enable target LED_GREEN */
#define LED_GREEN_OFF() \
	GPIO_ClearPinsOutput(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN) /*!< Turn off target LED_GREEN */
#define LED_GREEN_ON() \
	GPIO_SetPinsOutput(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN) /*!< Turn on target LED_GREEN */
#define LED_GREEN_TOGGLE() \
	GPIO_TogglePinsOutput(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN) /*!< Toggle on target LED_GREEN */

/* @brief FreeRTOS tickless timer configuration. */
#define vPortLptmrIsr 				LPTMR0_IRQHandler /*!< Timer IRQ handler. */
#define TICKLESS_LPTMR_BASE_PTR 	LPTMR0  /*!< Tickless timer base address. */
#define TICKLESS_LPTMR_IRQn 		LPTMR0_IRQn /*!< Tickless timer IRQ number. */

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/

void BOARD_InitDebugConsole(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _PULSAR_BOARD_H_ */
