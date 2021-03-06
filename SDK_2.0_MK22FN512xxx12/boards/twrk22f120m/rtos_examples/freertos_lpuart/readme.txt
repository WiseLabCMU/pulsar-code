Overview
========
The LPUART example for FreeRTOS demonstrates the possibility to use the LPUART driver in the RTOS.
The example uses single instance of LPUART IP and writes string into, then reads back chars.
After every 4B received, these are sent back on LPUART.

Toolchain supported
===================
- IAR embedded Workbench 7.50.1
- Keil MDK 5.17
- GCC ARM Embedded 2015-4.9-q3
- Kinetis Development Studio IDE 3.0.0
- Atollic TrueSTUDIO 5.4.0

Hardware requirements
=====================
- Micro USB cable
- USB-to-Serial connector
- TWR-K22F120 board
- Elevator Tower
- Personal Computer

Board settings
==============
The freertos_lpuart example is configured to use LPUART0 with PTE4 and PTE5 pins. 
Connect pin:
- RX of USB2COM to J1-7
- TX of USB2COM to B56(Elevator)
- GND of USB2COM to B2(Elevator)

Prepare the Demo
================
1.  Connect a USB-to-Serial connector between the PC host and the LPUART0 pins on the board.
2.  Open a serial terminal on PC with these settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.
4.  Reset the SoC and run the project.

Running the demo
================
You will see the welcome string printed out on the console.
You can send characters to the console back and they will be printed out onto console in a group of 4 characters.

Customization options
=====================

