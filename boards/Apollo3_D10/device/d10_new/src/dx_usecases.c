/*
 * dx_usecases.c -- functions per supported use case
 *
 * Copyright (C) 2016 DSP Group
 * All rights reserved.
 *
 * Unauthorized copying of this file is strictly prohibited.
 */

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "dx_fw_defs.h"
#include "dx_params.h"
#include "dx_driver.h"
#include "dx_utils.h"
#include "dx_gen.h"
#include "dx_usecases.h"

#include "ext_dsp_manager.h"
#include "debug.h"

extern int g_transport;
extern int g_use_wakeup;
extern int g_mic1_type;
extern int g_mic2_type;
extern int g_chip1;
extern int g_chip2;
extern int g_mode_flag;
extern int g_master_clock;
extern int g_i2s_chip_master;
extern int g_asrp_delay;
extern int g_no_of_speakers;
extern int g_tdm_sample_rate;
extern int g_tdm_bit_rate;
extern int g_support_rx_process;
extern int g_google_assist_mode;
extern int g_use_spi_in_ga_mode;

typedef enum  {
	UC_LOW_POWER,
	UC_NR_2_MICS,
    UC_BARGE_IN,
    UC_VOICE_CALL,
	UC_PRODUCTION
}usecase_id;

static void config_mics(int chip, int mic_1_type, int mic_2_type);
static void config_mics_alt(int chip, int mic_1_type, int mic_2_type, int dm_clk_freq, int gain);
static void close_mics(int chip, int mic_1_type, int mic_2_type);
static void set_dc_value_for_180k_res(int chip);

/***********************************************************************
* FUNCTION NAME: init_config_chip()
*
* DESCRIPTION:
*	Config chip till initial idle state
*
* PARAMETERS:
*
* RETURNS:
* 	0 on sucsses
***********************************************************************/

int init_config_chip(int chip)
{
	int host_interface;
	int fw_ready_enable;

	Debug_Printf(" init_config_chip D%d -->\n", chip);

	if (g_transport == TRANSPORT_SPI)
		host_interface = SPI_D2_TDM1_D4_GPIO_4_5_6_7_D10_GPIO_2_3_4_5;
	else
	if (g_transport == TRANSPORT_I2C)
		host_interface = I2C_D2_ON_I2C_SDA_SCK_D4_D6_GPIO_1_2_D10_GPIO_3_4;
	else	// UART
	if (g_transport == TRANSPORT_UART)
		host_interface = UART0_ENABLE_FOR_HOST_D2_TDM2_D4_D6_GPIO_17_18_D10_GPIO_2_5;

	// always enable FW UART HW for debug (not supported in TRANSPORT_UART)
	if (g_transport != TRANSPORT_UART)
	{	host_interface |= UART_DEBUG_MODE_0; }

	// enable host_interface and debug UART
	write_register(chip, HOST_INTERFACE_SUPPORT_29, host_interface | UART_DEBUG_MODE_0 | UART_D2_0_UART_TDI_RTCK_D4_1_GPIO_14_15_D10_GPIO_12_13);

	init_fw_log(chip);		// if needed, should be called after reg 29

	write_register(chip, EVENTS_INDICATIONS_EN_CFG_12,	VT1_DET			|
														VT2_DET			|
														VT3_DET			|
														AEP_DET			|
														WARNING_EVENT	|
														ERROR_EVENT);

	fw_ready_enable = READY_ACK ? EN_FW_READY_GPIO | FW_READY_GPIO_POL_HIGH | FW_READY_GPIO_NUM_10 : 0;

	write_register(chip, EVENTS_INDICATION_GPIO_15,	EN_EVENTS_GPIO					|
													EVENTS_GPIO_POL_HIGH			|
													EVENTS_GPIO_NUM_14 				|
													fw_ready_enable);

	if (g_master_clock == MASTER_CLOCK_32768)
		{write_register(chip, MASTER_CLOCK_FREQUENCY_1B, MCLK_32768Hz);}
	else if (g_master_clock == MASTER_CLOCK_24576)
		{write_register(chip, MASTER_CLOCK_FREQUENCY_1B, MCLK_24576000Hz);}
	else
		{write_register(chip, MASTER_CLOCK_FREQUENCY_1B, g_master_clock);}
	write_register(chip, AUDIO_BUFFER_SIZE_09, AUDIO_BUFFER_3_SEC);
	write_register(chip, GENERAL_CONFIGURATION_1_22, 	WAKEUP_POLARITY_ACTIVE_LOW	|
														MAX_NUMBER_OF_MIC_IS_0_OR_1);
	if (OPTIMIZED)
		{write_register(chip, MEMORY_CONFIG_33, AUDIO_BUF_LOC_DTCM_USING_MEM_ALLOCATION  |
												AHB_ON_SIZE_128KW_D4_192KW_D8_208KW_D10 |
												DTCM_SIZE_96KW_D4_192KW_D8_192KW_D10);}
	else
		{ write_register(chip, MEMORY_CONFIG_33, AUDIO_BUF_LOC_AHB_USING_MEM_ALLOCATION |
									             AHB_ON_SIZE_128KW_D4_192KW_D8_256KW_D10|
           										 DTCM_SIZE_96KW_D4_192KW_D8_192KW_D10);}

	write_register(chip, GENERAL_CONFIGURATION_2_23, 	MIC_SAMPLE_RATE_16K		|
														FW_VAD_TYPE_NO_VAD);

	if ((g_mic1_type == MIC_ANALOG) || (g_mic2_type == MIC_ANALOG)){
		write_register(chip, HIGH_PASS_FILTER_1A,	0x0010 |
													IIR_HPF_EN);
		set_dc_value_for_180k_res(chip);
	}else{
		write_register(chip, HIGH_PASS_FILTER_1A, 	0x0010 |
													IIR_HPF_EN	|
													DC_REMOVE_COARSE_EN |
                                                    IIR_HPF_CUTOFF_FREQ_60HZ);
	}

	configure_clocks (chip, NO_SWITCH_TO_TDM, NO_SWITCH_TO_MCLK,
						g_master_clock, g_tdm_sample_rate, g_tdm_bit_rate,
						102000, 102000,
						NO_WANTED_AHB_CLK, NO_WANTED_APB_CLK,
						NO_USE_PLL_POST_DIV,
						1024, DEFAULT_AMIC_FREQ, g_i2s_chip_master);

	//config_uart_speed(chip, CHANGE_UART1_BAUDRATE | UART1_UART_BAUD_RATE_1_Mhz, 1000000);

	config_mics(chip, g_mic1_type, g_mic2_type);

	write_register(chip, DIGITAL_GAIN_04,	GAIN_AFFECTS_MIC1 | DIGITAL_GAIN_3_DB);

#if OPTIMIZE_LEVEL > 1
  ms_delay(20); 
#else 
	ms_delay(100); // TBD
#endif

	close_mics(chip, g_mic1_type, g_mic2_type);

    write_register(chip, LDO_CONFIGURATION_2B, LDO_DISABLE);

	return 0;
}

/***********************************************************************/

void set_dc_value_for_180k_res(int chip)
{
	write_io_port(chip, 0x92800002, 0x1ff80fe3);
}

/***********************************************************************/

void config_wwe (int chip, int wwe, int cp)
{
	// config loaded engine
	if (wwe == WWE_AMAZON){
		write_register(chip, VT_GENERAL_CFG_304,  		VT_NUM_1				|
														VT_EN_EN				|
														VT_ACTIVE				|
														SET_IN_CP				|
														cp);

		write_register(chip, VT_AUDIO_HISTORY_CFG_305,  VT_NUM_1				|
														EXT_HIST_TIME_500MS		|
														HIST_POINT_IS_WW_START);

		write_register(chip, VT_GENERAL_CFG_304,  		VT_NUM_2				|
														VT_EN_DIS);
	}else
	if (wwe == WWE_GOOGLE){
		write_register(chip, VT_GENERAL_CFG_304,  		VT_NUM_2			|
														VT_EN_EN			|
														VT_ACTIVE			|
														SET_IN_CP			|
														cp);

		write_register(chip, VT_AUDIO_HISTORY_CFG_305,  VT_NUM_2			|
														HIST_POINT_IS_WW_START);

		write_register(chip, VT_GENERAL_CFG_304,  		VT_NUM_1			|
														VT_EN_DIS);

		write_register(chip, (VT_REG_PHRASE_LENGTH_27 + VT2_REGS_OFFSET),  USER_CFG_WWE_LENGTH_750_MS);
	}else
	if (wwe == WWE_DUAL){
		write_register(chip, VT_GENERAL_CFG_304,  		VT_NUM_GLOBAL		|
														VT_EN_EN			|
														VT_ACTIVE			|
														SET_IN_CP			|
														cp);

		write_register(chip, VT_AUDIO_HISTORY_CFG_305,  VT_NUM_GLOBAL		|
														EXT_HIST_TIME_500MS	|
														HIST_POINT_IS_WW_START);

		write_register(chip, (VT_REG_PHRASE_LENGTH_27 + VT2_REGS_OFFSET),  USER_CFG_WWE_LENGTH_750_MS);

	}else
	if (wwe == WWE_NONE){
		write_register(chip, VT_GENERAL_CFG_304,  		VT_NUM_GLOBAL		|
														VT_EN_DIS);
	}
}

static void reset_events(int chip)
{
	int interrupt_events = read_register(chip, DETECTION_AND_SYSTEM_EVENTS_STATUS_14);
	write_register(chip, DETECTION_AND_SYSTEM_EVENTS_STATUS_14, interrupt_events);
}

int use_case_wakeup_detection_Enter(int chip, int wwe)
{
	int amic_freq = 1;
	int dmic_freq = 1024;

	Debug_Printf("[%s]: Start.\n", __func__);
	write_register(chip, AUDIO_PROCESSING_CONFIGURATION_34, POST_DETECTION_MODE_REMAIN_IN_DETECTION);

	configure_clocks (chip, NO_SWITCH_TO_TDM, NO_SWITCH_TO_MCLK,
						g_master_clock, g_tdm_sample_rate, g_tdm_bit_rate,
						NO_WANTED_PLL, 63728, 8000, 8000, NO_USE_PLL_POST_DIV,
						dmic_freq, amic_freq, g_i2s_chip_master);
	write_register(chip, 0x18, 0x4);

	//LDE
    write_register(chip, VT_GENERAL_CFG_304,	VT_NUM_1    |
                                          	VT_EN_EN    |
                                            VT_ACTIVE   |
                                            SET_IN_CP   |
                                            IN_CP_NUM_0);
	write_register(chip, VT_AUDIO_HISTORY_CFG_305,  VT_NUM_1	|
											HIST_POINT_IS_SWITCH_TO_BUFFER);
#if 1
	//OKG
    write_register(chip, VT_GENERAL_CFG_304,	VT_NUM_2    |
                                           	VT_EN_EN    |
                                           	VT_ACTIVE   |
                                            SET_IN_CP   |
                                            IN_CP_NUM_0);
	write_register(chip, VT_AUDIO_HISTORY_CFG_305,  VT_NUM_2	|
											HIST_POINT_IS_WW_START);

	write_register(chip, (VT_REG_PHRASE_LENGTH_27 + VT2_REGS_OFFSET),
			USER_CFG_WWE_LENGTH_500_MS);
#else
	write_register(chip, VT_GENERAL_CFG_304,  		VT_NUM_2			|
													VT_EN_DIS);
#endif

#define DIGITAL_GAIN_6_DB (6<<4)
#define DIGITAL_GAIN_8_DB (8<<4)
	config_mics_alt(chip, MIC_DIGITAL, MIC_NONE,
			DM_CLK_FREQ_1024_1040_SR_16KHz_32KHz_48KHz, DIGITAL_GAIN_8_DB);

	ms_delay(10);

	/* reset detection trigger registers */
	write_register(chip, VT1_REGS_OFFSET | VT_REG_WORD_ID, 0x0);
	write_register(chip, VT2_REGS_OFFSET | VT_REG_WORD_ID, 0x0);

	write_register(chip, 0xd, 0x3);
	write_register(chip, 0xd, 0x4);
	ms_delay(5);
	write_register(chip, 0x18, 0x4);
	ms_delay(5);

	reset_events(chip);
	write_register(chip, OPERATION_MODE_01, DETECTION_MODE);
	Debug_Printf("[%s]: Finish.\n", __func__);
	return 0;
}

int use_case_wakeup_detection_Exit(int chip)
{
	Debug_Printf("[%s]: Start.\n", __func__);

	write_register(chip, OPERATION_MODE_01, IDLE_MODE);

	ms_delay (10);

	if (g_mode_flag == MODE_RECORD)
	{
		fw_record_turn_off(chip);
	}

	close_mics(chip, g_mic1_type, g_mic2_type);

	configure_clocks (chip, NO_SWITCH_TO_TDM, NO_SWITCH_TO_MCLK,
						g_master_clock, g_tdm_sample_rate, g_tdm_bit_rate,
						NO_WANTED_PLL, 73000,
						0, 0,
						NO_USE_PLL_POST_DIV,
						DEFAULT_DMIC_FREQ, DEFAULT_AMIC_FREQ, g_i2s_chip_master);

	ms_delay(10);

	write_register(chip, AUDIO_PROCESSING_CONFIGURATION_34, 0x0000);

	write_register(chip, GENERAL_CONFIGURATION_2_23, 0x0000);

	Debug_Printf("[%s]: Finish.\n", __func__);

	close_mics(chip, g_mic1_type, MIC_NONE);

	return 0;
}

static const char *use_case_wakeup_cmd[] = {
	"Start Workout",
	"Start My Workout",
	"Stop Workout",
	"Play Music",
	"Play My Music",
	"Stop Music",
	"Stop My Music",
	"Show Weather",
	"Show The Weather",
	"Show The Weather",
	"Weather",
	"Show Last Text",
	"Show Last Text",
	"Show Last Text",
	"Show Last Call",
	"Show Last Call",
	"Show Last Call",
	"Show My Last Call",
	"Show My Last Call",
	"Show My Last Call",
	"See Wellness",
	"Show Wellness",
	"Show My Wellness",
	"Show Commute",
	"Launch Stopwatch",
	"Start Stopwatch",
	"Stop Stopwatch",
	"Launch Timer",
	"Start Timer",
	"Start My Timer",
	"Stop Timer",
	"Stop My Timer",
	"Daily Mode",
	"Night Mode",
	"Challenge Mode",
	NULL
};

#define _OUT_RED_ "\033[0;31m"
#define _OUT_GREEN_ "\033[0;32m"
#define _OUT_DEFAULT_ "\033[0m"

void use_case_wakeup_print_commands(void)
{
	const char *prev = NULL;
	const char **cmd = &use_case_wakeup_cmd[0];

	Debug_Printf(_OUT_GREEN_"Wakeup use case commands:\n");
	Debug_Printf("\tOK Watch\n");
	do {
		if (prev && strcmp(*cmd, prev) == 0)
			continue;

		Debug_Printf("\t\t%s\n", *cmd);
		prev = *cmd;
	} while (*++cmd);
	Debug_Printf(_OUT_DEFAULT_);
}

static void process_lde(int chip, int word_id)
{
	if (word_id > 1000 && word_id < 2000)
		Debug_Printf(_OUT_GREEN_"Trigger ! - OK Watch  -> \n"_OUT_DEFAULT_);
	else if (word_id > 2000) {
		int ix = word_id - 2001;

		if (ix < 0 || ix > 34) {
			Debug_Printf(_OUT_RED_"Unknown word ID: %d\n"_OUT_DEFAULT_, word_id);
			return;
		}

		Debug_Printf(_OUT_GREEN_"Command ! - %s  -> \n"_OUT_DEFAULT_, use_case_wakeup_cmd[ix]);
	} else {
		Debug_Printf(_OUT_RED_"Unexpected word ID: %d\n"_OUT_DEFAULT_, word_id);
	}
}

extern void launch_record_thread();

static void process_okg(int chip, int word_id)
{
	static int okg_count = 0;
	Debug_Printf("Got on OK GOOGLE trigger #%d!\n", ++okg_count);
  ExtDspMgr_SignalEvent(EXT_DSP_START_STREAM_VOICE_EVT);
}

void use_case_wakeup_vt_vc_trigger(int chip)
{
	enum {nvts = 2};
	static const int vt_offset[nvts] = {VT1_REGS_OFFSET, VT2_REGS_OFFSET};
	static void (*vt_handle[nvts])(int, int) = {process_lde, process_okg};
	int word_id, i;

	for (i = 0; i < nvts; i++) {
		int regnum;

		regnum = vt_offset[i] | VT_REG_WORD_ID;
		word_id = read_register(chip, regnum);
		printf("vt%d: word id: %d\n", i, word_id);

		if (!word_id) continue;

		vt_handle[i](chip, word_id);
		write_register(chip, regnum, 0x0);
		check_fw_errors(chip);
	}
}

//*****************************************************************************/

// Activate a uc which is written in a predefined reg table file.
int use_case_reg_table_only_enter(int chip)
{
	// reg table file returns the state number, if user wants
	return (hook_reg_setting(USE_CASE_REG_TABLE_ENTER_FILENAME));
}

// Full exit uc setting in reg table file.  for enter and for exit this usecase
// make sure exit brings the system to idle usecase.

int use_case_reg_table_only_exit(int chip)
{
	return (hook_reg_setting(USE_CASE_REG_TABLE_EXIT_FILENAME));
}

/*****************************************************************************/

//mic_1_type =  actual mic 1 type (for current uc) (not necessary identical to g_mic1_type)
//mic_2_type =  actual mic 2 type (for current uc) (not necessary ldentical to g_mic2_type)
void config_mics(int chip, int mic_1_type, int mic_2_type)
{
	int sleep = 10;
	int mic_synch_flag;

	mic_synch_flag = (mic_2_type != MIC_NONE)? SYNCED_START : NO_SYNCED_START;		// set sync flag if 2 mics

	if (mic_1_type == MIC_DIGITAL){
		if (USE_HDMI_MICS) {
			write_register(chip,FIRST_MICROPHONE_CONFIG_24,	DDF_AUDIO_ATTENUATION_0dB							|
															mic_synch_flag										|
															DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz			|
															CLOCK_POLARITY_RISING_EDGE							|
															D10L_DDF_AND_DM_CONFIG_DDF2_DM1);
		}else{
			write_register(chip,FIRST_MICROPHONE_CONFIG_24,	DDF_AUDIO_ATTENUATION_0dB							|
															mic_synch_flag										|
															DM_CLK_FREQ_1024_1040_SR_16KHz_32KHz_48KHz			|
															CLOCK_POLARITY_RISING_EDGE							|
															D10L_DDF_AND_DM_CONFIG_DDF2_DM1);
		}
	} else if (mic_1_type == MIC_ANALOG){
		write_register(chip, FIRST_MICROPHONE_CONFIG_24,	DDF_AUDIO_ATTENUATION_MINUS_6dB	|
															mic_synch_flag				|
															SAR_IIR_FILTER_128			|
															D10L_DDF_AND_DM_CONFIG_SAR0_D10L_DDF_SAR_ADC);
		delay_if_not_ready(chip, sleep);
	}

	if (mic_2_type == MIC_DIGITAL){
		if (mic_1_type == MIC_DIGITAL){
			if (USE_HDMI_MICS) {
				write_register(chip,SECOND_MICROPHONE_CONFIG_25,DDF_AUDIO_ATTENUATION_0dB							|
																DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz			|
																CLOCK_POLARITY_RISING_EDGE							|
																D10L_DDF_AND_DM_CONFIG_DDF2_DM1);
			}else{
				write_register(chip,SECOND_MICROPHONE_CONFIG_25,DDF_AUDIO_ATTENUATION_0dB							|
																DM_CLK_FREQ_1024_1040_SR_16KHz_32KHz_48KHz			|
																CLOCK_POLARITY_RISING_EDGE							|
																D10L_DDF_AND_DM_CONFIG_DDF1_DM0);
			}

		}else if (mic_1_type == MIC_ANALOG){

			if (USE_HDMI_MICS){
				write_register(chip,SECOND_MICROPHONE_CONFIG_25,	DDF_AUDIO_ATTENUATION_0dB							|
																	DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz			|
																	CLOCK_POLARITY_RISING_EDGE							|
																	D10L_DDF_AND_DM_CONFIG_DDF0_DM0);
			}else{
				write_register(chip,SECOND_MICROPHONE_CONFIG_25,	DDF_AUDIO_ATTENUATION_0dB							|
																	DM_CLK_FREQ_1024_1040_SR_16KHz_32KHz_48KHz			|
																	CLOCK_POLARITY_RISING_EDGE							|
																	D10L_DDF_AND_DM_CONFIG_DDF2_DM1);
			}
		}

	}else if (mic_2_type == MIC_ANALOG){
		if (mic_1_type == MIC_ANALOG){
			write_register(chip, SECOND_MICROPHONE_CONFIG_25,	DDF_AUDIO_ATTENUATION_MINUS_6dB	|
																SAR_IIR_FILTER_128			|
																D10L_DDF_AND_DM_CONFIG_SAR1_D10L_DDF_SAR_ADC);

		}else {
			write_register(chip, SECOND_MICROPHONE_CONFIG_25,	DDF_AUDIO_ATTENUATION_0dB	|
																SAR_IIR_FILTER_128			|
																D10L_DDF_AND_DM_CONFIG_SAR0_D10L_DDF_SAR_ADC);

		}

		delay_if_not_ready(chip, sleep);
	}
}

static void config_mics_alt(int chip, int mic_1_type, int mic_2_type, int dm_clk_freq, int gain)
{
	int sleep = 10;
	int mic_synch_flag;

	mic_synch_flag = (mic_2_type != MIC_NONE)? SYNCED_START : NO_SYNCED_START;		// set sync flag if 2 mics

	if (mic_1_type != MIC_NONE){
		write_register(chip,FIRST_MICROPHONE_CONFIG_24,	DDF_AUDIO_ATTENUATION_0dB							|
														mic_synch_flag										|
														dm_clk_freq											|
														CLOCK_POLARITY_RISING_EDGE							|
														D10L_DDF_AND_DM_CONFIG_DDF2_DM1);
		delay_if_not_ready(chip, sleep);
		write_register(chip, DIGITAL_GAIN_04, GAIN_AFFECTS_MIC1 | gain);
		delay_if_not_ready(chip, sleep);
	}

	if (mic_2_type != MIC_NONE) {
		write_register(chip,SECOND_MICROPHONE_CONFIG_25,DDF_AUDIO_ATTENUATION_0dB			|
														dm_clk_freq							|
														CLOCK_POLARITY_RISING_EDGE			|
														D10L_DDF_AND_DM_CONFIG_DDF2_DM1);
		delay_if_not_ready(chip, sleep);
		write_register(chip, DIGITAL_GAIN_04, GAIN_AFFECTS_MIC2 | gain);
		delay_if_not_ready(chip, sleep);
	}
}

void close_mics(int chip, int mic_1_type, int mic_2_type)
{
	// close 2nd mic before 1st mic
	if (mic_2_type != MIC_NONE){
		if (mic_2_type == MIC_ANALOG){
			write_register(chip,SECOND_MICROPHONE_CONFIG_25, CLOSING_MICS_SAR_LOW_AMP);
		}else{
			write_register(chip,SECOND_MICROPHONE_CONFIG_25, CLOSING_MICS_NO_DM_CLOCK);
		}
	}

	if (mic_1_type == MIC_ANALOG){
			write_register(chip,FIRST_MICROPHONE_CONFIG_24, CLOSING_MICS_SAR_LOW_AMP);
		}else{
			write_register(chip,FIRST_MICROPHONE_CONFIG_24, CLOSING_MICS_NO_DM_CLOCK);
		}
}

//*****************************************************************************/
int d10l_enter_hibernation(int chip, int mic_1_type, int mic_2_type)
{
	write_register(chip, OPERATION_MODE_01, IDLE_MODE);
	close_mics(chip, mic_1_type, mic_2_type);
	write_register(chip, OPERATION_MODE_01, HIBERNATE_MODE);
	return 0;
}
//*****************************************************************************/
int d10l_exit_hibernation(int chip, int mic_1_type, int mic_2_type)
{
	int value = 1;

	wakeup(chip);
	value = read_register(chip, OPERATION_MODE_01);
	if (value != IDLE_MODE)
	{
		Debug_Printf("D10L didnt wakeup!\n");
		return -1;
	}
	write_register(chip, OPERATION_MODE_01, IDLE_MODE);
	config_mics(chip, mic_1_type, mic_2_type);
	return 0;
}
//*****************************************************************************/

// Parse and activate a reg table file
// typical line in a register table (hook) file:  "D8  18   5"  or "D8  delay  20"
int hook_reg_setting(char* filename)
{
	FILE *file;
	char* str;
	char buffer[128];
	char chip_str[32];
	char command_str[5];
	char reg_str[32];
	char val_str[32];
	int chip;
	int reg;
	int val;
	int state = 100;

	int rc;

	/* Open the file */
	file = fopen(filename, "rb");

	if(!file)
	{
		Debug_Printf("Regs File %s does not exist - continue\n", filename);
		return -1 ;
	}

    while (fgets(buffer, 128, file) != NULL )
	{
		//Debug_Printf("line %s", buffer);
   		if(buffer[0] == ';')
   		{
			memset(buffer,0,128);
			continue ;
   		}

		memset(chip_str,0,32);
		memset(reg_str,0,32);
		memset(val_str,0,32);

		rc = sscanf(buffer, "%s	%s %s %s", chip_str, command_str, reg_str,val_str);
		if((rc == 4) || (rc == 3))
		{
			Debug_Printf("chip %s command %s %s %s \n", chip_str, command_str, reg_str, val_str);

			if((strcmp(chip_str,"D4") == 0) || (strcmp(chip_str,"D8") == 0) || (strcmp(chip_str,"D10") == 0))
				chip = g_chip1;
			else
				chip = g_chip2;

			if(strcmp(command_str,"w") == 0)
			{
				val = strtol(val_str,&str,16);		// convert value
				reg = strtol(reg_str,&str,16);		// convert register
				write_register(chip, reg, val);
			}
			else
			if(strcmp(command_str,"r") == 0)
			{
				reg = strtol(reg_str,&str,16);		// convert register
				val = read_register(chip, reg);
			}
			else
			if(strcmp(command_str,"delay") == 0)
			{
				ms_delay(val);
			}
			else
			if(strcmp(command_str,"state") == 0)
			{
				state = val;
			}
			else{
				Debug_Printf("failed to parse line %s \n", buffer);
			}
		}

		memset(buffer,0,128);
       }

	fclose(file);

	return (state);
}
