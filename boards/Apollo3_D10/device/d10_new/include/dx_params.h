/**
 * dx_params.h  --  product parameters etc.
 *
 * Copyright (C) 2016 DSP Group
 * All rights reserved.
 *
 * Unauthorized copying of this file is strictly prohibited.
 */

#ifndef _DX_PARAMS_H
#define _DX_PARAMS_H

#include <stdint.h>

#define CUSTOMER_HOST_VERSION "0.1"

#define DSPG_C_REFERENCE_HOST_VERSION "0.1"

#define DSPG_BASE_VERSION "Kiwi_D10_ver_4633_RC2_OKG_210124_LDEVT_100_NNL"

#define I2C_DEVICE_ADDRESS 	0x3e  			// According to chip
//#define I2C_DEVICE_ADDRESS 	0x3f  		// According to chip

#define FW_MODE				1	 // 1 = fw_fast // 2 = asrp/fw record // 3 = fw-debug

#define REG_5_FW_RECORD		0x10	// set hex!
#define REG_6_FW_RECORD		0x0
#define REG_7_FW_RECORD		0x0

/* File name of Firmware and Acousric model */
#define FIRMWARE_IMAGE 		"host/fw/Kiwi_D10_ver_4633_RC2_OKG_210124_LDEVT_100_NNL.bin"
//#define FIRMWARE_IMAGE 		"host/fw/Dos_D8_ver_4601.bin"

#define WAKE_ENGINE			6	// 6 = SED // 7 = T3T4  // 4 = DUAL

/* VT ACOUSTIC MODELS */
#define A_MODEL_SMARTWATCH_DSP  	"host/Amodels/LDE/SmartWatch_dsp_model.bin"
#define A_MODEL_SMARTWATCH_NN  		"host/Amodels/LDE/SmartWatch_nn_model.bin"

#define A_MODEL_OKG					"host/Amodels/Google/210124/en_all.mmap"

#define	SED_MODEL_FILENAME	A_MODEL_SED_SW_GB
#define T3T4_MODEL_FILENAME	"no_loaded_file"

//inject register table files:
#define USE_CASE_1_HOOK1_FILENAME "host/scripts/use_case_1_hook1.txt"
#define USE_CASE_4_HOOK1_FILENAME "host/scripts/use_case_4_hook1.txt"
#define CONFIG_FILENAME 		  "host/scripts/config_ini.txt"
#define MENU_REG_TABLE			  "host/scripts/menu_reg_table.txt"		// for debug
#define USE_CASE_REG_TABLE_ENTER_FILENAME			  "host/scripts/uc_enter_reg_table.txt"		// for debug
#define USE_CASE_REG_TABLE_EXIT_FILENAME			  "host/scripts/uc_exit_reg_table.txt"		// for debug

#define DUMP_FILENAME 			"Dump_audio_file.raw"

#define WAKE_UP		 0    // 0 (no wakeup) // 1 (with wakeup)

#define BARGE_IN_TDM_BYPASS 0		// 0 = no bypass // 1 = bypass

#define TRASPORT_CHAN 1		// 1 = SPI // 2 = I2C  // 3 = UART

//#define transport type (SPI / I2C / UART) by comment 2 of next lines, and do full make:
#define SPI_TRANSPORT
//#define I2C_TRANSPORT
//#define UART_TRANSPORT

//[BEN] I undef this
//#define READ_WRITE_LOCK

//[BEN] I disable READY check for now
//Currently, A4 serve interrupt handlers of several pins which belong to the same GPIO***_IRQn in a while() loop
// so that during D10's INT handler, it cannot execute ISR handler of READY pin to update s_ready_flag 
// and hence it waits very long time in wait_ready_or_delay() call 
#define READY_ACK		0	// 1 = FW ready GPIO is operative (for now not per chip) // 0 = not operative
#define EVENTS_TO_HOST	1	// 1 = FW sends events to host  0 = no events to host

#define CHIP_1		10	// 2 = DBM_D2  // 4 = DBM_D4  // 5 = DBM_D5 // 7 = DBM_D7 // 8 = DBM_D8 // 10 = DBM_D10 (defined in dx_gen.h)

#define MASTER_CLOCK	3	// 1 = 32.768KHz	// 2 = 19.2MHz // 3 = 24.576MHz // 4 = 12.288MHz

// Mic configuration:
#define MIC1		1					// 1 (digital) // 2 (analog)
#define MIC2		0	// 0 (not mic) 	// 1 (digital) // 2 (analog)
#define MIC3		0	// 0 (no mic)	// 1 (digital) // 2 (analog)
#define MIC4		0	// 0 (no mic) 	// 1 (digital) // 2 (analog)

// TDM COnfiguration
#define TDM_SAMPLE_RATE 	16000 	// 16000 = 16 KHz	// 48000 = 48 KHz
#define TDM_BIT_RATE 		16 	// 16 = 16 bit	// 32 = 32 bit
#define NO_OF_SPEAKERS		1	// 1 = one speaker		2 = 2 speakers

#define I2S_CHIP_MASTER		0	// 0 = Chip Slave	// 1 = Chip Master

// Specific usecases options
#define SUPPORT_RX_PROCESS		0	// during voice call:  	0 = disable // 1 = enable
#define GOGGLE_ASSISTANT_MODE	0	// during Barge in:		0 = disable // 1 =  enable
#define USE_SPI_IN_GA_MODE 		0	// 0 = False  //  1 = True  // dump spi during google assist

// dump session:
#define DUMP_SECONDS	5	
		
#define DUMP_MAX_CHUNK				4096		// for NON SPI
#define DUMP_HOST_BUFFER_SIZE		16000

#define AUDIO_SAMPLE_RATE			16000
#define NO_OF_STREAM_CHANNELS			1	// how many channels to stream during dump
#define DUMP_FRAME_LENGTH			960
#define INFINITE_TIME 		0

#define ASRP_DELAY 1

#define OPTIMIZED	1				// 0 = not optimized // 1 = optimized

//////////////////////////////////////////////////////////////////////////////

// Parameters for RPI / EVBL (not for customer)

#define SPI_DEVICE_PATH "/dev/spidev0.1"
#define I2C_DEVICE_PATH "/dev/i2c-1"
#define UART_DEVICE_PATH "/dev/ttyAMA0"

#define GPIO_PLATFORM_PREFIX    "gpio" // "PB"

/* GPIO pins on RPI board */
#define RESET_VT_GPIO			17
#define INTERRRUPT_TRIGGER_GPIO	23
#define INTERRRUPT_READY_GPIO	5
#define WAKEUP_VT_GPIO 			14

#define EXTERNAL_MICS	// commented if internal mics (real product)

#define HOST_COLLECT_FW_LOGS		// to comment in real product

#define CPLD_RST_GPIO 			26
#define DEBUG_GPIO	 			13	// for tests only

//#define USE_HDMI_MICS		1	// 0 = false	// 1 = true
#define USE_HDMI_MICS		0	// 0 = false	// 1 = true

#endif /* _DX_PARAMS_H */
