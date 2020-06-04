/*
 * lmx2571_registers.h
 *
 *  Defines all the registers of the TI LMX2571 RF Synthesizer
 *  All registers are 24 bit
 *
 *  Created on: Sep 10, 2016
 *      Author: Adwait Dongare
 */

#include <stdint.h>

#ifndef DRIVERS_LMX2571_REGISTERS_H_
#define DRIVERS_LMX2571_REGISTERS_H_

// number of registers to program
#define LMX2571_NREG 44

// Read and write commands bit values
#define CMD_WRITE 0U
#define CMD_READ 1U

enum lmx2571_reg_addr {
	R0_addr = 0,
	R1_addr = 1,
	R2_addr = 2,
	R3_addr = 3,
	R4_addr = 4,
	R5_addr = 5,
	R6_addr = 6,
	R7_addr = 7,
	R8_addr = 8,
	R9_addr = 9,
	R10_addr = 10,
	R11_addr = 11,
	R12_addr = 12,
	R13_addr = 13,
	R14_addr = 14,
	R15_addr = 15,
	R16_addr = 16,
	R17_addr = 17,
	R18_addr = 18,
	R19_addr = 19,
	R20_addr = 20,
	R21_addr = 21,
	R22_addr = 22,
	R23_addr = 23,
	R24_addr = 24,
	R25_addr = 25,
	R26_addr = 26,
	R27_addr = 27,
	R28_addr = 28,
	R29_addr = 29,
	R30_addr = 30,
	R31_addr = 31,
	R32_addr = 32,
	R33_addr = 33,
	R34_addr = 34,
	R35_addr = 35,
	R39_addr = 39,
	R40_addr = 40,
	R41_addr = 41,
	R42_addr = 42,
	R47_addr = 47,
	R53_addr = 53,
	R58_addr = 58,
	R60_addr = 60,
};

struct __attribute__( ( packed ) ) _R0 {
	unsigned FCAL_EN :1;
	unsigned BIT1 :1;
	unsigned BIT2 :1;
	unsigned BIT3 :1;
	unsigned BIT4 :1;
	unsigned BIT5 :1;
	unsigned F1F2_SEL :1;
	unsigned F1F2_MODE :1;
	unsigned F1F2_CTRL :1;
	unsigned F1F2_INIT :1;
	unsigned RXTX_POL :1;
	unsigned RXTX_CTRL :1;
	unsigned POWERDOWN : 1;
	unsigned RESET : 1;
	unsigned BIT14 : 1;
	unsigned BIT15 : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R1 {
	uint8_t PLL_NUM_F1;
	uint8_t PLL_DEN_F1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R2 {
	uint16_t PLL_NUM_F1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R3 {
	uint16_t PLL_DEN_F1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R4 {
	unsigned PLL_N_F1 : 12;
	unsigned FRAC_ORDER_F1 : 3;
	unsigned PLL_N_PRE_F1 : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R5 {
	uint8_t PLL_NUM_F1;
	uint8_t PLL_DEN_F1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R6 {
	unsigned MULT_F1 : 5;
	unsigned PFD_DELAY_F1 : 3;
	unsigned CHDIV1_F1 : 2;
	unsigned CHDIV2_F1 : 3;
	unsigned LF_R3_F1 : 3;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R7 {
	unsigned LF_R4_F1 : 3;
	unsigned BIT3 : 1;
	unsigned BIT4 : 1;
	unsigned BIT5 : 1;
	unsigned OUTBUF_RX_EN_F1 : 1;
	unsigned OUTBUF_TX_EN_F1 : 1;
	unsigned OUTBUF_RX_PWR_F1 : 5;
	unsigned BIT13 : 1;
	unsigned BIT14 : 1;
	unsigned BIT15 : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R8 {
	unsigned OUTBUF_TX_PWR_F1 : 5;
	unsigned EXTVCO_SEL_F1 : 4;
	unsigned FSK_EN_F1 :  1;
	unsigned BIT11 : 1;
	unsigned BIT12 : 1;
	unsigned BIT13 : 1;
	unsigned BIT14 : 1;
	unsigned BIT15 : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R9 {
	uint16_t FSK_DEV0_F1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R10 {
	uint16_t FSK_DEV1_F1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R11 {
	uint16_t FSK_DEV2_F1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R12 {
	uint16_t FSK_DEV3_F1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R13 {
	uint16_t FSK_DEV4_F1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R14 {
	uint16_t FSK_DEV5_F1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R15 {
	uint16_t FSK_DEV6_F1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R16 {
	uint16_t FSK_DEV7_F1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R17 {
	uint8_t PLL_NUM_F2;
	uint8_t PLL_DEN_F2;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R18 {
	uint16_t PLL_NUM_F2;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R19 {
	uint16_t PLL_NUM_F2;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R20 {
	unsigned PLL_N_F2 : 12;
	unsigned FRAC_ORDER_F2 : 3;
	unsigned PLL_N_PRE_F2 : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R21 {
	uint8_t PLL_R_PRE_F2;
	uint8_t PLL_R_F2;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R22 {
	unsigned MULT_F2 : 5;
	unsigned PFD_DELAY_F2 : 3;
	unsigned CHDIV1_F2 : 2;
	unsigned CHDIV2_F2 : 3;
	unsigned LF_R3_F2 : 3;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R23 {
	unsigned LF_R4_F2 : 3;
	unsigned BIT3 : 1;
	unsigned BIT4 : 1;
	unsigned BIT5 : 1;
	unsigned OUTBUF_RX_EN_F2 : 1;
	unsigned OUTBUF_TX_EN_F2 : 1;
	unsigned OUTBUF_RX_PWR_F2 : 5;
	unsigned BIT13 : 1;
	unsigned BIT14 : 1;
	unsigned BIT15 : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R24 {
	unsigned OUTBUF_TX_PWR_F2 : 5;
	unsigned EXTVCO_SEL_F2 : 1;
	unsigned EXTVCO_CHDIV_F2 : 4;
	unsigned FSK_EN_F2 : 1;
	unsigned BIT11 : 1;
	unsigned BIT12 : 1;
	unsigned BIT13 : 1;
	unsigned BIT14 : 1;
	unsigned BIT15 : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R25 {
	uint16_t FSK_DEV0_F2;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R26 {
	uint16_t FSK_DEV1_F2;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R27 {
	uint16_t FSK_DEV2_F2;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R28 {
	uint16_t FSK_DEV3_F2;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R29 {
	uint16_t FSK_DEV4_F2;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R30 {
	uint16_t FSK_DEV5_F2;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R31 {
	uint16_t FSK_DEV6_F2;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R32 {
	uint16_t FSK_DEV7_F2;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R33 {
	uint16_t FSK_DEV_SPI_FAST;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R34 {
	unsigned FSK_MODE_SEL1 : 1;
	unsigned FSK_MODE_SEL0 : 1;
	unsigned FSK_DEV_SEL : 3;
	unsigned FSK_LEVEL : 2;
	unsigned FSK_I2S_CLK_POL : 1;
	unsigned FSK_I2S_FS_POL : 1;
	unsigned BIT9 : 1;
	unsigned XTAL_EN : 1;
	unsigned XTAL_PWRCTRL : 3;
	unsigned IPBUF_SE_DIFF_SEL : 1;
	unsigned IPBUF_DIFF_TERM : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R35 {
	unsigned OUTBUF_RX_TYPE : 1;
	unsigned OUTBUF_TX_TYPE : 1;
	unsigned OUTBUF_AUTOMUTE : 1;
	unsigned MULT_WAIT : 11;
	unsigned BIT14 : 1;
	unsigned BIT15 : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R39 {
	unsigned LD_EN : 1;
	unsigned BIT1 : 1;
	unsigned BIT2 : 1;
	unsigned SDO_LD_SEL : 1;
	unsigned BIT4 : 1;
	unsigned BIT5 : 1;
	unsigned BIT6 : 1;
	unsigned BIT7 : 1;
	unsigned BIT8 : 1;
	unsigned BIT9 : 1;
	unsigned BIT10 : 1;
	unsigned BIT11 : 1;
	unsigned BIT12 : 1;
	unsigned BIT13 : 1;
	unsigned BIT14 : 1;
	unsigned BIT15 : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R40 {
	unsigned BIT0 : 1;
	unsigned BIT1 : 1;
	unsigned BIT2 : 1;
	unsigned BIT3 : 1;
	unsigned BIT4 : 1;
	unsigned BIT5 : 1;
	unsigned CP_GAIN : 2;
	unsigned CP_IUP : 5;
	unsigned BIT13 : 1;
	unsigned BIT14 : 1;
	unsigned BIT15 : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R41 {
	unsigned BIT0 : 1;
	unsigned BIT1 : 1;
	unsigned BIT2 : 1;
	unsigned BIT3 : 1;
	unsigned BIT4 : 1;
	unsigned BIT5 : 1;
	unsigned BIT6 : 1;
	unsigned BIT7 : 1;
	unsigned BIT8 : 1;
	unsigned BIT9 : 1;
	unsigned BIT10 : 1;
	unsigned BIT11 : 1;
	unsigned BIT12 : 1;
	unsigned BIT13 : 1;
	unsigned BIT14 : 1;
	unsigned BIT15 : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R42 {
	unsigned EXTVCO_CP_IDN : 5;
	unsigned EXTVCO_CP_POL : 1;
	unsigned BIT6 : 1;
	unsigned BIT7 : 1;
	unsigned BIT8 : 1;
	unsigned BIT9 : 1;
	unsigned BIT10 : 1;
	unsigned BIT11 : 1;
	unsigned BIT12 : 1;
	unsigned BIT13 : 1;
	unsigned BIT14 : 1;
	unsigned BIT15 : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

struct __attribute__( ( packed ) ) _R47 {
	unsigned BIT0 : 1;
	unsigned BIT1 : 1;
	unsigned BIT2 : 1;
	unsigned BIT3 : 1;
	unsigned BIT4 : 1;
	unsigned BIT5 : 1;
	unsigned BIT6 : 1;
	unsigned BIT7 : 1;
	unsigned BIT8 : 1;
	unsigned BIT9 : 1;
	unsigned BIT10 : 1;
	unsigned BIT11 : 1;
	unsigned BIT12 : 1;
	unsigned DITHERING : 2;
	unsigned BIT15 : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};



struct __attribute__( ( packed ) ) _R53 {
	unsigned BIT0 : 1;
	unsigned BIT1 : 1;
	unsigned BIT2 : 1;
	unsigned BIT3 : 1;
	unsigned BIT4 : 1;
	unsigned BIT5 : 1;
	unsigned BIT6 : 1;
	unsigned BIT7 : 1;
	unsigned BIT8 : 1;
	unsigned BIT9 : 1;
	unsigned BIT10 : 1;
	unsigned BIT11 : 1;
	unsigned BIT12 : 1;
	unsigned BIT13 : 1;
	unsigned BIT14 : 1;
	unsigned BIT15 : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};



struct __attribute__( ( packed ) ) _R58 {
	unsigned BIT0 : 1;
	unsigned BIT1 : 1;
	unsigned BIT2 : 1;
	unsigned BIT3 : 1;
	unsigned BIT4 : 1;
	unsigned BIT5 : 1;
	unsigned BIT6 : 1;
	unsigned BIT7 : 1;
	unsigned BIT8 : 1;
	unsigned BIT9 : 1;
	unsigned BIT10 : 1;
	unsigned BIT11 : 1;
	unsigned BIT12 : 1;
	unsigned BIT13 : 1;
	unsigned BIT14 : 1;
	unsigned BIT15 : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};


struct __attribute__( ( packed ) ) _R60 {
	unsigned BIT0 : 1;
	unsigned BIT1 : 1;
	unsigned BIT2 : 1;
	unsigned BIT3 : 1;
	unsigned BIT4 : 1;
	unsigned BIT5 : 1;
	unsigned BIT6 : 1;
	unsigned BIT7 : 1;
	unsigned BIT8 : 1;
	unsigned BIT9 : 1;
	unsigned BIT10 : 1;
	unsigned BIT11 : 1;
	unsigned BIT12 : 1;
	unsigned BIT13 : 1;
	unsigned BIT14 : 1;
	unsigned BIT15 : 1;
	unsigned ADDRESS : 7;
	unsigned RW : 1;
};

// generic 24 bit register format for LMX2571
// use for generic memory allocations

struct __attribute__ ( ( packed ) ) _bits {
	unsigned BIT0 : 1;
	unsigned BIT1 : 1;
	unsigned BIT2 : 1;
	unsigned BIT3 : 1;
	unsigned BIT4 : 1;
	unsigned BIT5 : 1;
	unsigned BIT6 : 1;
	unsigned BIT7 : 1;
	unsigned BIT8 : 1;
	unsigned BIT9 : 1;
	unsigned BIT10 : 1;
	unsigned BIT11 : 1;
	unsigned BIT12 : 1;
	unsigned BIT13 : 1;
	unsigned BIT14 : 1;
	unsigned BIT15 : 1;
	unsigned BIT16 : 1;
	unsigned BIT17 : 1;
	unsigned BIT18 : 1;
	unsigned BIT19 : 1;
	unsigned BIT20 : 1;
	unsigned BIT21 : 1;
	unsigned BIT22 : 1;
	unsigned BIT23 : 1;
};

struct __attribute__( ( packed ) ) _datamap {
	uint16_t data;
	unsigned ADDRESS: 7;
	unsigned RW : 1;
};

// generic 24 bit register format for LMX2571
// use for generic memory allocations
// can assume the form of any available register

union lmx2571_register {
	uint8_t bytes[3];
	struct _bits bits;
	struct _datamap datamap;
	struct _R0 R0;
	struct _R1 R1;
	struct _R2 R2;
	struct _R3 R3;
	struct _R4 R4;
	struct _R5 R5;
	struct _R6 R6;
	struct _R7 R7;
	struct _R8 R8;
	struct _R9 R9;
	struct _R10 R10;
	struct _R11 R11;
	struct _R12 R12;
	struct _R13 R13;
	struct _R14 R14;
	struct _R15 R15;
	struct _R16 R16;
	struct _R17 R17;
	struct _R18 R18;
	struct _R19 R19;
	struct _R20 R20;
	struct _R21 R21;
	struct _R22 R22;
	struct _R23 R23;
	struct _R24 R24;
	struct _R25 R25;
	struct _R26 R26;
	struct _R27 R27;
	struct _R28 R28;
	struct _R29 R29;
	struct _R30 R30;
	struct _R31 R31;
	struct _R32 R32;
	struct _R33 R33;
	struct _R34 R34;
	struct _R35 R35;
	struct _R39 R39;
	struct _R40 R40;
	struct _R41 R41;
	struct _R42 R42;
	struct _R47 R47;
	struct _R53 R53;
	struct _R58 R58;
	struct _R60 R60;
};

#endif /* DRIVERS_LMX2571_REGISTERS_H_ */
