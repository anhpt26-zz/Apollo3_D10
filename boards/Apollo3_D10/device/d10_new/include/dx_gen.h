/**
 * dx_gen.h  --  general (system wide) defines and externals.
 *
 * Copyright (C) 2016 DSP Group
 * All rights reserved.
 *
 * Unauthorized copying of this file is strictly prohibited.
 */

#ifndef _DX_GEN_H
#define _DX_GEN_H

#include <stdint.h>
#include <stdbool.h>

typedef enum  {
    DBM_D2 = 2,
    x,
    DBM_D4,
    DBM_D5,
    y,
    DBM_D7,
    DBM_D8,
	z,
    DBM_D10
} ChipNames;

#define MODE_OPT 1		// FW optimized
#define MODE_RECORD 2
#define MODE_DEBUG 3

#define FW_NO_LOG 1
#define FW_AUDIO_LOG 2
#define FW_DEBUG_LOG 3

#define TRANSPORT_NOT_YET 0
#define TRANSPORT_SPI 1
#define TRANSPORT_I2C 2
#define TRANSPORT_UART 3

#define MASTER_CLOCK_32768 	32768
#define MASTER_CLOCK_13824 	13824
#define MASTER_CLOCK_19200 	19200
#define MASTER_CLOCK_24576 	24576
#define MASTER_CLOCK_12288 	12288

//BEN: I skip this
//typedef enum {FALSE = 0, TRUE = 1} bool;

#define ON 1
#define OFF 0

#define uint unsigned int

#define i2s_clks_chip_slave   0
#define i2s_clks_chip_master  1

typedef enum  {
	MIC_NONE,
	MIC_DIGITAL,
    MIC_ANALOG,
	MIC_VESPER
}mic_types;

typedef enum  {
	TDM0 = 0,
	TDM1,
	TDM2,
	TDM3,
}tdm_number;

typedef enum  {
	PROTOCOL_I2S = 0,
	PROTOCOL_PCM,
}tdm_protocol;

typedef enum  {
	TRIGGER,
    COMMAND
}trigger_type;

typedef enum  {
	WWE_NONE = 0,
	WWE_SENSORY,
	WWE_GOOGLE,
	WWE_AMAZON,
	WWE_DUAL,
	WWE_SAMSUNG,
	WWE_SENSING_SED,
	WWE_T3T4,
	WWE_DOS
}wake_engine;

/* #define DEBUG 1 */
#ifdef DEBUG
	#define DEBUG_PRINT printf
#else
	#define DEBUG_PRINT(format, args...) ((void)0)
#endif

#endif /* _DX_GEN_H */
