/*
 * dx_driver.c
 *
 * Functions to communicate with chip
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
#include <math.h>

#include "dx_fw_defs.h"
#include "dx_params.h"
#include "dx_gen.h"
#include "dx_driver.h"
#include "dx_utils.h"

#include "ext_dsp_manager.h"
#include "debug.h"

int segmentId;

unsigned int *sared_stop_streaming_flag;

int chip_io[15];

int skip_prints = 0;

//BEN: To be supported
#define host_collect_fw_log_start(x)
#define host_collect_fw_log_stop(x)

/* list of internal functions */

void wakeup(int chip);
int read_chunk (int chip, uint8_t * header, int header_len, char * buf, int bytes_to_read);
int request_and_read_chunk (int chip, int16_t request_reg, int16_t request_val, uint8_t * header, int header_len, char * buf, int bytes_to_read);
int Run_chip(int chip);
void start_fw_log(int chip, int which_log);
void stop_fw_log(int chip, int which_log);
int vt_reg_address (int vt_num);
void powerUpReadyRequest(int chip);
void dbmdx_interface_sync(int chip);

/////////////////////////////////////////////////////////////////////////////////
/* Global variable definitions */

//extern int chip;
extern int g_transport;           //to choose SPI/I2C/UART interface
extern int g_use_wakeup;          //if we want to toggle WAKEUP pin @ every SPI transfer
extern char * g_FirmWare_filename;
extern int g_reg5_fw_record;
extern int g_reg6_fw_record;
extern int g_reg7_fw_record;
extern int g_mode_flag;           //if we want to enable D10 to write its log so that later we can readout via UART
extern int g_asrp_lib_version;
extern int g_asrp_parameter_version;
extern int g_asrp_delay;
extern bool g_disable_asrp_record;
extern char * g_trigger_filename;

//int asrp_delay = ASRP_DELAY;

/* default delay periods not during debug */
int delay_io_write = 5;
int delay_after_reset = 70;	// for D10L
int delay_preboot = 2;
int delay_after_load = 1;
int delay_after_run = 15;
int delay_before_asrp_load1 = 10;
int delay_before_asrp_load2 = 10;
int delay_after_asrp_load = 20;
int delay_after_change_clock = 50;
int delay_after_wakeup = 5;

void set_long_delays()

{
	// Set long delays for debug, ASRP recording and I2C modes
	delay_io_write = 15;
	delay_after_reset = 70;	// for D10L
	delay_preboot = 10;
	delay_after_load = 10;
	delay_after_run = 50;
	delay_before_asrp_load1 = 10;
	delay_before_asrp_load2 = 20;
	delay_after_asrp_load = 20;
	delay_after_change_clock = 50;
	delay_after_wakeup = 10;
}


static volatile int keepRunning = 1;

// Include actual protocol (interface) related functions
#define SPI_TRANSPORT

/***********************************************************************
* FUNCTION NAME: init_IO_ports()
*
* DESCRIPTION:
*	init all io ports towards chip: gpios and io (SPI / I2C / UART)
*
* PARAMETERS:
*
* RETURNS:
*	0 on sucsses
*	error otherwise
*
***********************************************************************/

int init_IO_ports(int chip)
{
  //All HW management is done by ext_dsp_manager.c already
  return 0;
#if 0 
	// "io" is the transport interface to chip (either SPI / I2C / UART)

	chip_io[chip] =init_IO_to_chip();

	rw_lock_create();		// if required, define lock for R/W transactions with chip

	// GPIOs

	gpio_init(GPIO_PLATFORM_PREFIX);

	if(gpio_request(RESET_VT_GPIO) < 0)
		goto err1;
	ms_delay(60);

	if(gpio_direction_output(RESET_VT_GPIO, LOW) < 0)
		goto err1;

	if(gpio_request(INTERRRUPT_TRIGGER_GPIO) < 0)
		goto err1;
	ms_delay(60);

	if(gpio_direction_input(INTERRRUPT_TRIGGER_GPIO) < 0)
		goto err1;

	if (READY_ACK) {
		if(gpio_request(INTERRRUPT_READY_GPIO) < 0)
			goto err1;
		ms_delay(60);

		if(gpio_direction_input(INTERRRUPT_READY_GPIO) < 0)
			goto err1;
	}

	// in RPI  - WAKEUP_D4_GPIO is used for UART, see wakeup implementation
	if (g_transport != TRANSPORT_UART){
		if(gpio_request(WAKEUP_VT_GPIO) < 0)
			goto err1;
		ms_delay(60);

		if(gpio_direction_output(WAKEUP_VT_GPIO, HIGH) < 0)
			goto err1;
	}

	return 0;

err1:
	return -1;
#endif 
}

/***********************************************************************/
// release IO ports
// To Add here GPIO release

void chip_release(int chip)
{
  //All HW management is done by ext_dsp_manager.c already
#if 0  
	if(chip_io[chip] > 0) {

		IO_Release(chip);  // in Protocol file
		chip_io[chip] = 0;
	}

	rw_lock_destroy();
#endif
}

/***********************************************************************
* FUNCTION NAME: init_chip()
*
* DESCRIPTION:
*	HW init, boot FW, run FW.
*
* PARAMETERS:
*
* RETURNS:
*	0 on sucsses
*	error otherwise
*
***********************************************************************/

int init_chip(int chip)
{
  /* reset */
  if ((chip == DBM_D4) || (chip == DBM_D8) || (chip == DBM_D10))
  {
    //gpio_set_value(RESET_VT_GPIO, LOW);
    //ms_delay(10);
    //gpio_set_value(RESET_VT_GPIO, HIGH);
    ExtDsp_ResetChip();
  }
#if OPTIMIZE_LEVEL > 1
  ms_delay(5);     //reduce from default 70ms to 5ms
#else
	ms_delay(delay_after_reset);			// time for PLL to stable (D10)
#endif 


	dbmdx_interface_sync(chip);

	if (EVENTS_TO_HOST){
		powerUpReadyRequest(chip);
	}

	{
		/* TBD */
		const unsigned char buf1[] = {0x5a, 0x04, 0x90, 0x00, 0x00, 0x03, 0xa5, 0x52, 0x55, 0x5};
		const unsigned char buf2[] = {0x5a, 0x04, 0x94, 0x00, 0x00, 0x03, 0xa5, 0x52, 0x55, 0x15};
		const unsigned char buf3[] = {0x5a, 0x04, 0x20, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00};
		const unsigned char clr_crc_chip[]= {BOOT_PRE_OP_CODE, BOOT_RESET_CHECKSUM};
		io_write_delay(chip, buf1, sizeof(buf1));
		io_write_delay(chip, buf2, sizeof(buf2));
		io_write_delay(chip, buf3, sizeof(buf3));
		io_write_delay(chip, clr_crc_chip, sizeof(clr_crc_chip));
#if OPTIMIZE_LEVEL > 1
    ms_delay(2);
#else
		ms_delay(10);
#endif
	}
  if(boot_chip(chip) < 0) {
    Debug_Printf("Failed to boot_chip\n");
    return -1;
  }

	ms_delay(delay_after_load);

	/* START chip */
  Debug_Printf("~~~~~~~~~~~~~~~~~run chip~~~~~~~~~~~~~\n");
	Run_chip(chip);

	set_ready_flag(chip, FLAG_ENABLE);			// enable ready listening
	set_powerup_flag(chip, FLAG_ENABLE);		// enable event listening

  Debug_Printf("~~~~~~~~~~~~~~~~~wait power up~~~~~~~~~~~~~\n");
	wait_powerup_or_delay(chip, delay_after_run);

	if (g_transport == TRANSPORT_UART){
		// send first reg 29 here, to eliminate FW logs towards host
		write_register(chip, HOST_INTERFACE_SUPPORT_29, HOST_INTERFACE_UART);
	}
#if OPTIMIZE_LEVEL == 1
  Debug_Printf("~~~~~~~~~~~~~~~~~fw validate~~~~~~~~~~~~~\n");
	fw_validate(chip);
#endif

	return 0;
}
/**********************************************************************/

// Toggle wakeup GPIO b efore speaking with chip
void wakeup(int chip)
{
	if (!g_use_wakeup)
	return;

	if ((chip == DBM_D4) || (chip == DBM_D8) || (chip == DBM_D10) )
	{
		if (g_transport != TRANSPORT_UART){
          //gpio_set_value(WAKEUP_VT_GPIO, LOW);
          //ms_delay(1);
          //gpio_set_value(WAKEUP_VT_GPIO, HIGH);
          //ms_delay(delay_after_wakeup);
          ExtDsp_WakeChip();
		}
		else{
			// write '0' to FW on UART for wakeup
			// to prepare 60 chard of 0 and to write chunk.
		}
	}
}

// Long wakeup is used to exit hibernate state
void wakeup_long_delay(int chip, int clk)		// wakeup + additional delay
{
	wakeup(chip);
	if (clk == 32768)
		{ms_delay(60);}
	else
		{ms_delay(( 1/(clk*1000))*2);}
}

/***********************************************************************/
// Needed for wakeup from Vesper (15ms) and other cases

void set_wakeup_delay(int chip, int ms)
{
	delay_after_wakeup = ms;
}

/***********************************************************************
* FUNCTION NAME: ASRP_read_register()
*
* DESCRIPTION:
*	read dbmdx ASRP register value using IO.
*	using indirect commands since the address of registers are above 0x100
* PARAMETERS:
*	int fd, int16_t reg
* RETURNS:
*	register value
***********************************************************************/
int16_t ASRP_read_register(int chip, int16_t reg)
{
	int16_t val;
	write_register(chip, INDIRECT_ACCESS_REGISTER_NUMBER_3D, reg);
	val = read_register(chip, INDIRECT_ACCESS_REGISTER_READ_3F);
	Debug_Printf("Register 0x%d = 0x%X = %d\n",reg, (val&0xffff), (val&0xffff));
	return val;
}

/***********************************************************************
* FUNCTION NAME: ASRP_change_param_indirect()
*
* DESCRIPTION:
*	write dbmdx ASRP register value using IO.
*	using indirect commands since the address of registers are above 0x100
* PARAMETERS:
*	int fd, int16_t reg, int16_t val
* RETURNS:
***********************************************************************/
void ASRP_change_param_indirect(int chip, int16_t blockid, int16_t offset, int16_t val)
{
	write_register(chip, ASRP_BLOCKID_LOW_PART_124, blockid & 0x00ff);
	write_register(chip, ASRP_BLOCKID_HIGH_PART_125, blockid & 0xff00);
	write_register(chip, ASRP_ENABLE_OFFSET_126, offset);
	write_register(chip, ASRP_VALUE_127, val);
}

/***********************************************************************/

void change_asrp_start_delay(int chip,int asrp_start_delay)
{
	int calc_delay;

	calc_delay = asrp_start_delay * 125 / 4;

	if (calc_delay != 0){
		ASRP_change_param_indirect(chip, 0x21, 1, calc_delay);
	}
}

/***********************************************************************/
// set ASRP gains
void set_asrp_output_scale_gain(int chip, int tx_output_gain, int vcpf_output_gain, int rx_output_gain)
{
	int tx1_out_gain, tx2_out_gain, tx3_out_gain;

	if (tx_output_gain != 0){
		if (tx_output_gain > 0){
			/////tx1_out_gain = (10**(tx_output_gain/20.0) * 2**8);  //???
			tx1_out_gain = 0x059f;  //???
		} else
		if (tx_output_gain < 0){
			Debug_Printf("ASRP Scaling Value cannot be bellow 0!\n");
		} else{
			Debug_Printf("ASRP Tx Output is Muted!\n");
			tx1_out_gain = tx_output_gain;
		}
		write_register(chip, ASRP_OUTPUT_GAIN_DEST_SEL_10A, ASRP_OUTPUT_DEST_TX_1);
		write_register(chip, ASRP_OUTPUT_GAIN_DEST_VALUE_10B, tx1_out_gain);
		Debug_Printf("ASRP Tx Output increased by %d dB\n", tx_output_gain);
	}

	if (vcpf_output_gain != 0){
		if (vcpf_output_gain > 0){
			//////tx2_out_gain = (10**(vcpf_output_gain/20.0) * 2**8);  //???
			tx2_out_gain = 0x059f;  //???
		} else if (vcpf_output_gain < 0){
			Debug_Printf("ASRP Scaling Value cannot be bellow 0!\n");
		} else{
			Debug_Printf("ASRP VCPF Output is Muted!\n");
			tx2_out_gain = vcpf_output_gain;
		}
		write_register(chip, ASRP_OUTPUT_GAIN_DEST_SEL_10A, ASRP_OUTPUT_DEST_TX_2);
		write_register(chip, ASRP_OUTPUT_GAIN_DEST_VALUE_10B, tx2_out_gain);
		Debug_Printf("ASRP VCPF Output increased by %d dB\n", vcpf_output_gain);
	}

	if (rx_output_gain != 0){
		if (rx_output_gain > 0){
			/////tx3_out_gain = (10**(rx_output_gain/20.0) * 2**8);  //???
			tx3_out_gain = 0x0169;  //???
		} else if (rx_output_gain < 0){
			Debug_Printf("ASRP Scaling Value cannot be bellow 0!\n");
		} else{
			Debug_Printf("ASRP RX Output is Muted!\n");
			tx3_out_gain = rx_output_gain;
		}
		write_register(chip, ASRP_OUTPUT_GAIN_DEST_SEL_10A, ASRP_OUTPUT_DEST_TX_3);
		write_register(chip, ASRP_OUTPUT_GAIN_DEST_VALUE_10B, tx3_out_gain);
		Debug_Printf("ASRP Rx Output increased by %d dB\n", rx_output_gain);
	}
}
/***********************************************************************/

// In FW Record mode, activate both FW & ASRP recording.
void asrp_record_turn_on(int chip, int asrp_record_channels1, int asrp_record_channels2, int record_command)
{
	Debug_Printf("asrp_record_turn_on. record command=%d ->\n", record_command);

	if (!g_disable_asrp_record){
		/* turn on ASRP recording */
		write_register(chip, ASRP_RECORDS_CHANNELS_128, asrp_record_channels1);
		write_register(chip, ASRP_RECORDS_CHANNELS_2_129, asrp_record_channels2);
	}

	/* add also fw record if requested */
	write_register(chip, REG_FW_RECORD_A_05, g_reg5_fw_record);
	write_register(chip, REG_FW_RECORD_B_06, g_reg6_fw_record);
	write_register(chip, REG_FW_RECORD_C_07, g_reg7_fw_record);
	write_register(chip, UART_DEBUG_RECORDING_30, record_command);

	set_fw_log_status(chip, FW_AUDIO_LOG);
}

/***********************************************************************/

void asrp_record_turn_off(int chip)
{
	Debug_Printf("asrp_record_turn_off->\n");
	/* tuning off ASRP recording */
	write_register(chip, INDIRECT_ACCESS_REGISTER_NUMBER_3D, 0x0128);
	write_register(chip, INDIRECT_ACCESS_REGISTER_WRITE_3E, 0x0000);

	/* tuning off FW recording */
	write_register(chip, REG_FW_RECORD_A_05, 0x0000);
	write_register(chip, REG_FW_RECORD_B_06, 0x0000);
	write_register(chip, REG_FW_RECORD_C_07, 0x0000);
	write_register(chip, UART_DEBUG_RECORDING_30, 0x0000);

	set_fw_log_status(chip, FW_NO_LOG);
}

/***********************************************************************/

// this function is called only for use cases without ASRP
void fw_record_turn_on(int chip, int record_command)
{
	Debug_Printf("fw_record_turn_on->\n");

	write_register(chip, REG_FW_RECORD_A_05, g_reg5_fw_record);
	write_register(chip, REG_FW_RECORD_B_06, g_reg6_fw_record);
	write_register(chip, REG_FW_RECORD_C_07, g_reg7_fw_record);
	write_register(chip, UART_DEBUG_RECORDING_30, record_command);

	set_fw_log_status(chip, FW_AUDIO_LOG);
}

/***********************************************************************/

// this function is called only for use cases without ASRP
void fw_record_turn_off(int chip)
{
	Debug_Printf("fw_record_turn_off->\n");
	/* tuning off mic recording */
	write_register(chip, UART_DEBUG_RECORDING_30, 0x0000);

	set_fw_log_status(chip, FW_NO_LOG);
}

/***********************************************************************
* FUNCTION NAME: configure_tdm_hw()
*
* DESCRIPTION:
*	send command to tdm hw
* PARAMETERS:
*	int chip, int tdm_number, int tdm_addr, int tdm_dir, int bit_depth, int mode
*
* RETURNS:
*	0 on sucsses elese error
***********************************************************************/
void configure_tdm_hw(int chip, int tdm_number, int tdm_addr, int tdm_dir, int bit_depth, int mode, int num_of_channels)
{
	if (chip == DBM_D10){

		if (tdm_dir == TDM_RX){
			if (bit_depth == 16){
				write_io_port(chip, tdm_addr, 0x1b800015);
				write_io_port(chip,(tdm_addr + 4), 0x7);
				write_io_port(chip,(tdm_addr + 6), 0x100f001f);
			}else{
				//  Config TDM input to 32bit
				if (num_of_channels == 2)
				{
					write_io_port(chip, tdm_addr, 0x1b804055);
					write_io_port(chip, (tdm_addr + 4), 0x2064);
					write_io_port(chip, (tdm_addr + 6), 0x103f003f);
					write_io_port(chip, (tdm_addr + 10), 0x5);
				//4 16 bit channels
				}else{
					write_io_port(chip, tdm_addr, 0x1b800015);
					write_io_port(chip, (tdm_addr + 4), 0x73007);
					write_io_port(chip, (tdm_addr + 6), 0x103f003f);
				}
			}
		}else{
			if (bit_depth == 16){
				if (mode == TDM_SLAVE){
					write_io_port(chip, tdm_addr, 0x1b800217);
					write_io_port(chip, (tdm_addr + 4), 0x7);
					write_io_port(chip, (tdm_addr + 6), 0x100f001f);
				}else{
					write_io_port(chip, tdm_addr, 0x1b800012);
					write_io_port(chip, (tdm_addr + 4), 0x241024);
					write_io_port(chip, (tdm_addr + 6), 0x100f001f);
				}
			}else{
				if (mode == TDM_SLAVE){
					write_io_port(chip, tdm_addr, 0x1b804257);
					write_io_port(chip, (tdm_addr + 4), 0x64);
					write_io_port(chip, (tdm_addr + 6), 0x101f003f);
					write_io_port(chip, (tdm_addr + 10), 0x5);
				}else{
					write_io_port(chip, tdm_addr, 0x1b804012);
					write_io_port(chip, (tdm_addr + 4), 0x641064);
					write_io_port(chip, (tdm_addr + 6), 0x101f003f);
					write_io_port(chip, (tdm_addr + 10), 0x55);
				}
			}
		}
	}else{   //D8 or D2
		if (tdm_dir == TDM_RX){
			if (bit_depth == 16){
				write_io_port(chip, tdm_addr, 0x800015);
				write_io_port(chip,(tdm_addr + 4), 0x7);
				write_io_port(chip,(tdm_addr + 6), 0x100f001f);
			}else{
				//  Config TDM input to 32bit
				write_io_port(chip, tdm_addr, 0x80405d);
				write_io_port(chip, (tdm_addr + 4), 0x2064);
				write_io_port(chip, (tdm_addr + 6), 0x103f003f);
				write_io_port(chip, (tdm_addr + 10), 0x5);
			}
		}else{
			if (bit_depth == 16){
				if (mode == TDM_SLAVE){
					write_io_port(chip, tdm_addr, 0x804053);
				}else{
					write_io_port(chip, tdm_addr, 0x804052);
				}
				write_io_port(chip, (tdm_addr + 4), 0x241024);
				write_io_port(chip, (tdm_addr + 6), 0x100f001f);
				write_io_port(chip, (tdm_addr + 10), 0xf);

			}else{
				if (mode == TDM_SLAVE){
					write_io_port(chip, tdm_addr, 0x804053);
					write_io_port(chip, (tdm_addr + 4), 0x64);
					write_io_port(chip, (tdm_addr + 6), 0x101f003f);
					write_io_port(chip, (tdm_addr + 10), 0x5);
				}else{
					write_io_port(chip, tdm_addr, 0x804052);
					write_io_port(chip, (tdm_addr + 4), 0x641064);
					write_io_port(chip, (tdm_addr + 6), 0x101f003f);
					write_io_port(chip, (tdm_addr + 10), 0x55);
				}
			}
		}
	}

	Debug_Printf("Configured chip=%d tdm=%d %s to bits %d mode %s\n", chip, tdm_number, tdm_dir?"tx":"rx", bit_depth, (mode == TDM_SLAVE)? "slave" : "master");
}

/***********************************************************************/

// reset tdm activity (tx and tx)
void reset_tdm(int chip, int tdm_no)
{
	write_register(chip, TDM_ACTIVATION_CONTROL_31, tdm_no);
	write_register(chip, TDM_TX_CONFIG_37, 0);
	write_register(chip, TDM_RX_CONFIG_36, 0);
}

/***********************************************************************
* FUNCTION NAME: write_HW_port_32()
*
* DESCRIPTION:
*	using indirect commands
* PARAMETERS:
*	int chip, int32_t address, int32_t value
* RETURNS:
*	none.
***********************************************************************/
void write_HW_port_32(int chip, int32_t address, int32_t val)
{
	write_register(chip, REG_IO_PORT_ADDR_LO_05, address);
	write_register(chip, REG_IO_PORT_ADDR_HI_06, address >> 16);
	write_register(chip, REG_IO_PORT_VALUE_LO_07, val);
	write_register(chip, REG_IO_PORT_VALUE_HI_08, val >> 16);
}

/***********************************************************************
* FUNCTION NAME: read_HW_port_32()
*
* DESCRIPTION:
*	using indirect commands
* PARAMETERS:
*	int chip, int32_t address
* RETURNS: 	int32_t value
*	none.
***********************************************************************/
int32_t read_HW_port_32(int chip, int32_t address)
{
	int32_t val;
	int16_t address_high, address_low;
	int16_t val_high, val_low;
	address_high = address >>16;
	address_low = address;

	//Debug_Printf ("address high = %x, address low = %x\n", address_high, address_low);

	write_register(chip, REG_IO_PORT_ADDR_LO_05, address_low);
	write_register(chip, REG_IO_PORT_ADDR_HI_06, address_high);
	val_low = read_register(chip, REG_IO_PORT_VALUE_LO_07);
	val_high = read_register(chip, REG_IO_PORT_VALUE_HI_08);

	val = val_high <<16 | val_low;

	//Debug_Printf ("val_high = %x val_low = %x  val= %x\n", val_high, val_low, val);
	return val;
}

/***********************************************************************
* FUNCTION NAME: boot_chip()
*
* DESCRIPTION:
*	Boot chip
*
* PARAMETERS:
*
* RETURNS:
*	0 on sucsses elese error
***********************************************************************/
int boot_chip(int chip)
{
	int fail_cnt = 0;

	while(fail_cnt < 5) {
		/* release D2 from reset */
		Debug_Printf("[%s]: Start uploading FirmWare to chip.\n", __func__);
		/* Load Firmware */
		if(load_file(chip, g_FirmWare_filename, 4) < 0) {
			fail_cnt++;
			ms_delay(20);
			if(fail_cnt == 5) {
				Debug_Printf("[%s]: Failed uploading FirmWare to chip.\n", __func__);
				goto err;
			}
		}
		else
			break;
	}

	return 0;
err:
	return -1;
}

/***********************************************************************/
// Command is sent to booter before FW init (preboot protocol)

char powerup_and_ready_request[] = {0x5A,0x02,0x04,0x00,0x00,0x00,0x02,0x02,0x00,0x00,0xCD,0xAB,0x03,0x00,0x01,0x00,0x8e,0x8a};
/*
		self.WriteBytes([0x00])

		HEADER_BYTE = [0x5a]
		OP_CODE = [0x02]
		NUM_OF_WORDS = [0x04, 0x00, 0x00, 0x00]
		ADDR = [0x02, 0x02, 0x00, 0x00]
		MAGIC_NUM = [0xCD, 0xAB]
		VALID_MASK = [0x03, 0x00]
		EN_EVENTS = [0x01,0x00] #value of register 0x12 - only for power up event
		GPIO_CFG = [0x8e,0x8a]  #value of register 0x15

		self.WriteBytes(HEADER_BYTE + OP_CODE + NUM_OF_WORDS + ADDR + MAGIC_NUM + VALID_MASK + EN_EVENTS + GPIO_CFG)
*/

void powerUpReadyRequest(int chip)
{
	io_write_delay(chip,(unsigned char *)powerup_and_ready_request, sizeof(powerup_and_ready_request));
}

/***********************************************************************
* FUNCTION NAME: Run_chip()
*
* DESCRIPTION:
*	Rum chip
*
* PARAMETERS:
*
* RETURNS:
* 	0 on sucsses
***********************************************************************/
int Run_chip(int chip)

{
	unsigned char buf_run[] = {BOOT_PRE_OP_CODE, BOOT_RUN_FW};

	io_write_delay(chip, buf_run, 2);

	return 0;
}

/***********************************************************************/

// For a given vt, calculates the FW register (its address), where from host can read the fw-status (loaded, not loaded) of that vt
int vt_reg_address (int vt_num)
{
	int vt_reg;

	if (vt_num == LOAD_ENGINE_TYPE_VT1){
		vt_reg = VT_REG_INIT + VT1_REGS_OFFSET;
	} else if (vt_num == LOAD_ENGINE_TYPE_VT2){
		vt_reg = VT_REG_INIT + VT2_REGS_OFFSET;
	} else if (vt_num == LOAD_ENGINE_TYPE_VT3){
		vt_reg = VT_REG_INIT + VT3_REGS_OFFSET;
	}
	return vt_reg;
}

/***********************************************************************/

// Preparations before VT loading, and call Load_A_Model
void model_loading(int chip, int wwe)
{
#define KIWI_MODEL_TYPE_SEC_MODEL (0x11)
#define KIWI_MODEL_TYPE_PRIM_MODEL (0x0)
	Load_A_Model(chip, A_MODEL_SMARTWATCH_NN, KIWI_MODEL_TYPE_SEC_MODEL, MEM_TYPE_AHB, LOAD_ENGINE_TYPE_VT1);
	Load_A_Model(chip, A_MODEL_SMARTWATCH_DSP, KIWI_MODEL_TYPE_PRIM_MODEL, MEM_TYPE_DTCM, LOAD_ENGINE_TYPE_VT1);
	//Load_A_Model(chip, A_MODEL_OKG, KIWI_MODEL_TYPE_PRIM_MODEL, MEM_TYPE_AHB, LOAD_ENGINE_TYPE_VT2);
	Load_A_Model(chip, A_MODEL_OKG, KIWI_MODEL_TYPE_PRIM_MODEL, MEM_TYPE_DTCM, LOAD_ENGINE_TYPE_VT2);
}

/*****************************************************************************/
//  FUNCTION NAME: Load_A_Model()
//
//  DESCRIPTION:
//		Load VT acoustic model to memory (not ASRP)
//  PARAMETERS:
//    char *filename
//  RETURNS:
//        0 on sucsses else error
//*****************************************************************************/
void Load_A_Model(int chip, char * filename, int model_type, int mem_loc, int vt_num)
{
	int value;
	write_register(chip, LOAD_BINARY_FILE_0F, 	vt_num					|
														OP_TYPE_LOAD_FILE		|
														FILE_LOAD_INIT_OR_KILL_REL			|
														BLK_START_NUM_1			|
														mem_loc					|
														model_type	);
	delay_if_not_ready(chip, 2);

	Debug_Printf("[%s]: Start uploading Acoustic Model %s to chip\n", __func__, filename);


	set_ready_flag(chip, FLAG_DISABLE);

	load_amodel_file(chip, filename, 0);

	Debug_Printf("[%s]: Done uploading Acoustic model.\n", __func__);

	Run_chip(chip);

	set_ready_flag(chip, FLAG_ENABLE);
#if OPTIMIZE_LEVEL > 1
  ms_delay(2);
#else
	ms_delay(10);
#endif

	// Verify model acceptance
	value = read_register(chip, vt_reg_address(vt_num));
	if (value == 0)	{Debug_Printf("[%s]: Acoustic model was not successfully uploaded %d !\n", __func__, value);}
	else 			{Debug_Printf("[%s]: Acoustic model was successfully uploaded!\n", __func__);}

	return;
}

/***********************************************************************
* FUNCTION NAME: Load_ASRP_param()
*
* DESCRIPTION: Load ASRP parameter file to FW
*
* PARAMETERS:
*	int chip, int param
* RETURNS:
* 	0 on sucsses
***********************************************************************/
int Load_ASRP_param(int chip, char *filename)
{
	int value, lib_version;
	long err = 1;

	//??? This function should be compared to python!!
	// Load_A_Model(chip, filename, model_type: LOAD_ASRP_PARAM_FILE, LOAD_MODEL_TO_DTCM, vt_num: LOAD_ENGINE_TYPE_ASRP);

	write_register(chip, LOAD_BINARY_FILE_0F, 	LOAD_ENGINE_TYPE_ASRP	|
												OP_TYPE_LOAD_FILE		|
												FILE_LOAD_INIT_OR_KILL_REL			|
												BLK_START_NUM_0			|
												MEM_TYPE_DTCM		|
												FILE_TYPE_ASRP_PARAMS_PRIM_INIT	);

	Debug_Printf("[%s]: Start uploading ASRP param.\n", __func__);

	delay_if_not_ready(chip, delay_before_asrp_load2);

	set_ready_flag(chip, FLAG_DISABLE);

	if(!load_file(chip, filename, 2)) {				// skip 0x5A 0x0B at tail of the file
		perror ("fail uploading ASRP MODEL.\n");
		err = -1;
		goto out;
	}

	Run_chip(chip);

	set_ready_flag(chip, FLAG_ENABLE);

	Debug_Printf("[%s]: Done uploading ASRP model.\n", __func__);

	delay_if_not_ready(chip, delay_after_asrp_load);

	// Verify asrp acceptance
	lib_version = read_register(chip, ASRP_LIB_VER_100);
	value = read_register(chip, ASRP_TUNING_VER_101);

	if (lib_version == 0){
		Debug_Printf ("ASRP Params file failed to load\n");
	} else {	// check for errors in ASRP

		g_asrp_parameter_version = value;

		value = read_register(chip, ASRP_NUM_OF_ERRS_104);
		if (value > 0){
			Debug_Printf ("ASRP Params have errors !\n");
		} else{
			Debug_Printf("ASRP Loaded! lib Version: %x, Tuning Read from ASRP: %s\n", lib_version, filename);
			g_asrp_lib_version = lib_version;
		}
	}

	delay_if_not_ready(chip, 10);

out:
	return err;
}

/***********************************************************************
* FUNCTION NAME: configure_clocks
*
* DESCRIPTION:
*	configure clocks and switch to TDM / MCLK whenever needed
*
* PARAMETERS:
*
*
* RETURNS:
*	0 on sucsses elese abort
***********************************************************************/

int configure_clocks (int chip, bool switch_to_tdm_clk, bool switch_to_mclk,
						int input_clk, int freq, int bit_depth,
						int wanted_pll, int wanted_tl3_clk,
						int wanted_ahb_clk, int wanted_apb_clk,
						bool use_pll_post_div,
						int digital_mic_freq, int analog_mic_freq, bool i2s_chip_master)
{

	int post_pll_div;
	int ref_clk, i2s_sclk=0, max_clk, low_clk_steps, switch_clk, clk_src, output_clk, perph_clk;
	int div_ratio,multiplier, pll_step, tl3_div;
	int ahb_div, apb_div, tl3_out, global_out, ahb_out, apb_out;
	int reg_23, reg_23_orig, sleep_ms, value, clk_config;
	int x,ref_clk_c;
	int ret = 0;


	Debug_Printf("\n================= configure clocks input parameters:=================>> \n");
	Debug_Printf("   switch_to_tdm_clk=%d	switch_to_mclk=%d \n", switch_to_tdm_clk, switch_to_mclk);
	Debug_Printf("   input_clk=%d	freq=%d bit_depth=%d \n", input_clk, freq, bit_depth);
	Debug_Printf("   wanted_pll=%d	wanted_tl3_clk=%d \n", wanted_pll, wanted_tl3_clk);
	Debug_Printf("   wanted_ahb_clk=%d	wanted_apb_clk=%d \n", wanted_ahb_clk, wanted_apb_clk);
	Debug_Printf("   use_pll_post_div=%d  \n", use_pll_post_div);
	Debug_Printf("   digital_mic_freq=%d	analog_mic_freq=%d i2s_chip_master = %d\n", digital_mic_freq, analog_mic_freq, i2s_chip_master);

	post_pll_div = 1;

	ref_clk = input_clk;

	if (i2s_chip_master){
		i2s_sclk = (freq * bit_depth * 2)/1000;
	}else{
		i2s_sclk = 1;
	}

	if (chip == DBM_D2){			//D2 has different clock config
		max_clk = 340000;
		if ((input_clk == 32768) || (input_clk == 19200)){
			ref_clk = 49152;}
		low_clk_steps = 6144;
	}else
	if (chip == DBM_D10){
			max_clk = 200000;
			if (input_clk != 32768){
				ref_clk = 12288;}
			low_clk_steps = 4096;
	}else{
		max_clk = 150000;
		if (input_clk == 19200){
			ref_clk = 12288;
			low_clk_steps = 6144;
		}else{
			low_clk_steps = 4096;
		}
	}

	Debug_Printf("a: i2s_sclk=%d  max_clk=%d  ref_clk=%d  low_clk_steps=%d\n", i2s_sclk, max_clk, ref_clk, low_clk_steps);

	///////////////

	if (digital_mic_freq != 1){
		digital_mic_freq *= 1000;
	}

	/*if (analog_mic_freq != 1){
		analog_mic_freq *= 1000;
	}*/

	if (switch_to_tdm_clk) {
		ref_clk = (freq * bit_depth * 2)/1000;
		switch_clk = SRC_SET_CONFIG_CLK_ACCORDING_TO_CLK_SEL;
		clk_src = CLK_SEL_TDM0_SCLK;
		write_register(chip, TDM_SCLK_CLOCK_FREQUENCY_1D, ref_clk);
		Debug_Printf("b: I2S SCLK = %d ", ref_clk);

		if (chip == DBM_D10){
			ref_clk = 12288;
		}
	}

	else if (switch_to_mclk){
		switch_clk = SRC_SET_CONFIG_CLK_ACCORDING_TO_CLK_SEL;
		clk_src = CLK_SEL_MCLK_IN;
		if (ref_clk == 32768){
			write_register(chip,MASTER_CLOCK_FREQUENCY_1B, 0x20);
		}else{
			write_register(chip,MASTER_CLOCK_FREQUENCY_1B, (input_clk));
		}

	}else{
		switch_clk = SRC_SET_KEEP_CUR_CLK_SRC;
		clk_src = CLK_SEL_MCLK_IN;
	}

	///////////////

	// calc_clk_div

	if (wanted_pll == 0){
		wanted_pll = wanted_tl3_clk;
	}else if (wanted_pll < ref_clk){
		Debug_Printf("!!! Wanted PLL frequency cannot be lower than input CLK\n");
		ret = -1;
	}

	div_ratio = wanted_pll/ref_clk;

	ref_clk_c = ref_clk;

	if (wanted_pll < ref_clk_c){
		multiplier = 2;
	}else{
		multiplier = 1;
	}
	output_clk = 0;

	Debug_Printf("c: div ratio=%d multiplier=%d \n",div_ratio, multiplier);

	///////////////

	if ((ref_clk == 32768) || (ref_clk == 49152) || ((ref_clk == 12288) && (input_clk == 19200)) || (chip == DBM_D10) ){

		if (multiplier*wanted_pll < ref_clk_c){
			pll_step = 0;
		}else{
			pll_step = div_and_ceil((multiplier*wanted_pll - ref_clk), low_clk_steps);
		}

		Debug_Printf("pll_step = %d\n", pll_step);

		while (output_clk/1000 < max_clk){
			if (pll_step < 0) {pll_step = 0;}   //making sure a non negative number is used
			output_clk = ref_clk_c*1000 + ((pll_step) * low_clk_steps*1000);
			if (chip == DBM_D10) (perph_clk = output_clk/2);
			else(perph_clk = output_clk);

			// Search for output_clk that is a multiplier of the below 3 frequencies (i.e. modulo operatopr % returns 0)
			if (!(perph_clk % i2s_sclk) && !(perph_clk % digital_mic_freq) && !(perph_clk % analog_mic_freq)){
				break;		// found!
			}else{
				Debug_Printf ("no common divider - PLL step goes up from %d to %d\n", pll_step, pll_step+1);
				pll_step = pll_step+1;
			}
		}

	}else{
		pll_step = div_and_ceil((multiplier*wanted_pll),(ref_clk_c));
		while (output_clk/1000 < max_clk){
			output_clk = pll_step * ref_clk_c*1000;
				// Search for output_clk that is a multiplier of the below 3 frequencies (i.e. modulo operatopr % returns 0)
			if (chip == DBM_D10) (perph_clk = output_clk/2);
			else(perph_clk = output_clk);

			if (!(perph_clk % i2s_sclk) && !(perph_clk % digital_mic_freq) && !(perph_clk % analog_mic_freq)){
				break;		// found!

			}else{
				Debug_Printf ("no common divider - PLL step goes up from %d to %d\n", pll_step, pll_step+1);
				pll_step = pll_step+1;
			}
		}
	}

	if (use_pll_post_div){
		post_pll_div = ((output_clk/1000)/wanted_tl3_clk);	// simple division = floor
		tl3_div = 1;
	}else{
		tl3_div = (output_clk/1000) / wanted_tl3_clk;		// simple division = floor
	}

	if (output_clk/1000 > (max_clk-40000)){
			Debug_Printf("!\n\n!!! MIPS ARE HIGH!!\n\n!\n");
			ret = -1;
	}
	Debug_Printf ("d: pll step = %d\npost_pll_div=%d  tl3_div=%d\n", pll_step, post_pll_div, tl3_div);

	///////////////

	tl3_div = tl3_div-1;

	if (wanted_ahb_clk == 0){
		wanted_ahb_clk = wanted_tl3_clk;
	}

	if (wanted_apb_clk == 0){
		wanted_apb_clk = wanted_ahb_clk;
	}

	ahb_div = (wanted_tl3_clk/wanted_ahb_clk)-1;
	apb_div = (wanted_ahb_clk/wanted_apb_clk)-1;
	global_out =  output_clk/(post_pll_div);
	tl3_out = global_out/(tl3_div+1);
	ahb_out = tl3_out/(ahb_div+1);
	apb_out = ahb_out/(apb_div+1);

	Debug_Printf("e: Selected Clock Frequencies are:\n   PLL = %d\n   GLB = %d\n   TL3 = %d\n   AHB = %d\n   APB = %d\n",
									output_clk, global_out, tl3_out, ahb_out, apb_out);
	Debug_Printf("f: TL3 frequency = %d MHz\n", tl3_out/1000000);

	reg_23_orig = reg_23 = read_register(chip, GENERAL_CONFIGURATION_2_23);

	if (use_pll_post_div){
		reg_23 &= (0xFFE0 | (post_pll_div-1));
	}else{
		reg_23 &= 0xFFE0;
	}

	if (reg_23_orig != reg_23) {
		write_register(chip, GENERAL_CONFIGURATION_2_23, reg_23);
	}

	///////////////

	if (ref_clk == 32768){
		sleep_ms = 40;
	}else{
		if ((input_clk != 24756) && (g_mode_flag == MODE_DEBUG)){
			sleep_ms =50;
		}else{
			sleep_ms = 15;
		}
	}

	//sleep_ms =100;		// to be tuned for D10L

	if (((pll_step > 14) || (switch_to_mclk)) && !switch_to_tdm_clk){

		if(g_transport != TRANSPORT_I2C){	// send 32 bit register
			write_register_32(chip, DSP_CLOCK_CONFIG_10,	switch_clk <<  12			|
															clk_src << 12 				|
															PLL_OSC_SEL_USE_PLL	<< 12	|
															pll_step << 12				|
															tl3_div << 8				|
															apb_div	<< 5				|
															ahb_div);
		}else{
			write_register(chip, DSP_CLOCK_CONFIG_10, USE_PLL_STEP_FROM_REG_0x1E	|
																tl3_div << 8		|
																apb_div	<< 5		|
																ahb_div);
			delay_if_not_ready(chip, sleep_ms);	// additional interim delay

			write_register(chip, DSP_CLOCK_CONFIG_EXTENSION_1E, switch_clk			|
																clk_src				|
																PLL_OSC_SEL_USE_PLL	|
																pll_step);

		}

		delay_if_not_ready(chip, sleep_ms);

	}else if (!switch_to_tdm_clk){
		if(g_transport != TRANSPORT_I2C){	// send 32 bit register
			write_register_32(chip, DSP_CLOCK_CONFIG_10,	switch_clk <<  12			|
															clk_src << 12 				|
															PLL_OSC_SEL_USE_PLL	<< 12	|
															pll_step << 12				|
															tl3_div << 8				|
															apb_div	<< 5				|
															ahb_div);
		}else{
			write_register(chip, DSP_CLOCK_CONFIG_10, pll_step << 12	|
														tl3_div << 8	|
														apb_div << 5	|
														ahb_div);
		}

		delay_if_not_ready(chip, sleep_ms);

	}

	// should be "else" //??
	if (switch_to_tdm_clk){

		if ((input_clk != 24756) && (g_mode_flag == MODE_DEBUG)){
			sleep_ms =100;
		} else {
			sleep_ms =100;   // for D10L switch to TDM
		}

		for (x = 0; x < 100; x++){

			if(g_transport != TRANSPORT_I2C){	// send 32 bit register
				write_register_32(chip, DSP_CLOCK_CONFIG_10,	switch_clk <<  12			|
																clk_src << 12 				|
																PLL_OSC_SEL_USE_PLL	<< 12	|
																pll_step << 12				|
																tl3_div << 8				|
																apb_div	<< 5				|
																ahb_div);
			}else{

				write_register(chip, DSP_CLOCK_CONFIG_10, USE_PLL_STEP_FROM_REG_0x1E	|
																tl3_div << 8			|
																apb_div	<< 5			|
																ahb_div);

				delay_if_not_ready(chip, 10);

				write_register(chip, DSP_CLOCK_CONFIG_EXTENSION_1E, switch_clk			|
																	clk_src				|
																	PLL_OSC_SEL_USE_PLL	|
																	pll_step);
			}

			delay_if_not_ready(chip, sleep_ms);

		// verify actual clock switch
		value = read_register(chip, DSP_CLOCK_CONFIG_EXTENSION_1E);
			clk_config = (CLK_SEL_TDM0_SCLK|PLL_OSC_SEL_USE_PLL) + pll_step;

			if (value == (clk_config & 0x7fff)){
				Debug_Printf("Successfull switch to TDM0 Clock\n");
				break;
			}else{
				//ERR_CNT = x;
				if (x == 99){
					Debug_Printf("Cannot switch to TDM0 CLOCK\n");
					ret = -1;
				}else{
					Debug_Printf("No Clock in TDM0 pin - Try Number %d\n", (x+1));
					ms_delay(50);		//??? to verify
					ret = -1;
				}
			}
		}

	}else if (switch_to_mclk){

		if (chip == DBM_D10) delay_if_not_ready(chip, 20);		//??? to verify if required

		if ((read_register(chip, CHIP_ID_NUMBER_19) & 0xff00) == 0xdb00){
			Debug_Printf("Successfull switch to MCLK Clock\n");
		}else{
			Debug_Printf("Cannot switch to MCLK CLOCK\n");
			ret = -1;
		}
	}
		Debug_Printf("================= finish configure clocks  <<============== \n\n");

	return(ret);
 }

/**********************************************************************/

int config_uart_speed(int chip, int fw_rate, int host_rate)
{
//BEN: Disabled for now since we do not use UART yet
#if 0
	write_register(chip, UART_BAUD_RATE_0C, fw_rate);

	if (g_transport == TRANSPORT_UART){
		// change also host UART baudrate
		config_speed(chip, host_rate);
	}
	else
	{
		update_host_debug_device_speed(host_rate);
	}
#endif

	return (0);
}

/**********************************************************************/

int fw_validate(int chip)
{
	int major,minor,rc;
	major = read_register(chip, FW_VERSION_NUMBER_00);
	minor = read_register(chip, FW_MINOR_VERSION_NUMBER_05);
	rc = read_register(chip, FW_RC_VERSION_NUMBER_06);
	if(major != 0) {
		if (rc != 0){Debug_Printf("[%s]: Chip Firmware version 0x%X.%x_RC%x\n", __func__, major & 0xffff, minor, rc);}
		else{Debug_Printf("[%s]: Chip Firmware version 0x%X.%x\n", __func__, major & 0xffff, minor);}
	} else {
		Debug_Printf("[%s]: Failed read Chip Firmware version \n", __func__);
	}
	return major;
}

/***********************************************************************
* FUNCTION NAME: record_buffer(int chip) into a Data consumer
*
* DESCRIPTION:
*	read the streamimg from chip and collect them into file
* PARAMETERS:
*
* RETURNS:
*	0 on sucsses else -1.
***********************************************************************/

#if 1   //Fossil's record_buffer()
int record_buffer(int chip, int sample_rate, int number_of_channels,  char *sample_buf, uint32_t sample_num) {
  int fw_buffer_size, fw_sample_size, bytes_to_read;
  uint8_t header[2];
  uint8_t delta = 2;    //we use SPI transport, so delta is 2

  //NOTE: We already wait for 93ms to D10 to collect 1500 mono-samples @ 16KHz
  // Therefore, we do not need to check avail samples in D10 but read them out immediately
  fw_buffer_size = read_register(chip, NUMBER_OF_SAMPLES_IN_BUFFER_0A);  //sample_num = fw_buffer_size/8 
  if (fw_buffer_size == 0xffff){
    Debug_Printf("FW Buffer got full and, now there are no samples in audio buffer!\n");
    return 0;
  }
  fw_sample_size = fw_buffer_size*8;         // number of 16-bit samples in D10 AUDIO BUFFER

  //If D10 backs to IDLE mode, return 0 here to abort audio recording task
  if (get_operation_mode(chip) == IDLE_MODE)
  {
    Debug_Printf("D10 backs to IDLE unexpected!\n");
    return 0;
  }

  if (fw_sample_size > sample_num){
    bytes_to_read = sample_num*2;

  }else{	// 1 frame is ready, read it:
    bytes_to_read = fw_sample_size*2;
  }

  request_and_read_chunk(chip, READ_AUDIO_BUFFER_20, bytes_to_read/16, header, 0, sample_buf, bytes_to_read + delta);

  return (bytes_to_read/2);
}


#else 
// buf size has to be tuned according to SPI speed, max frame size and max channels to be recorded --
// Mandatory to ensure that record speed is not slower than speed of FW audio buf get filled.
char buf[DUMP_HOST_BUFFER_SIZE] = {0};

int s_streaming_in_progress = FALSE;
int s_stop_streaming_flag = FALSE;

// This function runs sometimes from interrupt context

// Data consumer can be a file or a socket, it is opened and closed in upper layer, and used here to get data chunks.
int record_buffer(int chip, HDC DataConsumer, int time_in_sec, int sample_rate, int frame_length, int number_of_channels)
{
	int wanted_fifo_size,current_fifo_size;
	int	max_bytes_to_read,
		frame_length_in_bytes,
		retry_counter,
		fw_buffer_size, fw_buffer_size_bytes, delta;
	int bytes_to_read,data_len;

	uint8_t header[2];

	current_fifo_size= 0;		// size in Bytes
	retry_counter = 0;
	max_bytes_to_read = DUMP_HOST_BUFFER_SIZE-2;		// for SPI delta
	frame_length_in_bytes = frame_length*2;

	if (s_streaming_in_progress){
		Debug_Printf("streaming already in progress!\n");
		return -1;
	}

	s_streaming_in_progress = TRUE;

  Debug_Printf ("\nStart downloading %d seconds of audio buffer.\n", time_in_sec);
  Debug_Printf ("audio_sample_rate=%d number_of_stream_channels=%d, frame_length=%d.\n", sample_rate, number_of_channels, frame_length);
  Debug_Printf ("Please wait ...\n\n");

// Calculate total amount to record:
	if (time_in_sec != 0){
		wanted_fifo_size = 16*time_in_sec*sample_rate*number_of_channels/8; // size in Bytes
	}else{
		wanted_fifo_size = 1;	// infinite mode
	}


	if (g_transport != TRANSPORT_SPI){
		max_bytes_to_read = DUMP_MAX_CHUNK;	// spi is not limmited, other transports may be limmited
		delta=0;
	}
	else
	{
		delta=2;		// additional 2 byte for SPI (1 HW and 1 that FW adds)
	}

	skip_prints = 1;

	while ((current_fifo_size < wanted_fifo_size) /*|| (!time_in_sec)*/){

		if (s_stop_streaming_flag){
			break;
		}

// check FW readiness: read from fw how many words exist
		fw_buffer_size = read_register(chip, NUMBER_OF_SAMPLES_IN_BUFFER_0A);

		// check speacial cases, 0000 or 0xffff:
		if ((fw_buffer_size == 0x0000) && (retry_counter < 10) ){

			// wait more for buffer to get filled
			retry_counter += 1;
			ms_delay(frame_length/16);
			continue;
		}
		else
		if (retry_counter >= 10){
			Debug_Printf("Buffer is full of Zero !\n");
			break;
		}
		else{
			retry_counter = 0;
		}

		if (fw_buffer_size == 0xffff){
			Debug_Printf("FW Buffer got full and, now there are no samples in audio buffer!\n");
			goto out;
		}

		fw_buffer_size_bytes = 16*fw_buffer_size ;     		// this is data length in bytes (size*8*2)

		// if less than 1 frame, wait more
		if (fw_buffer_size_bytes < frame_length_in_bytes){
			//if FW switched to idle
			if (get_operation_mode(chip) == IDLE_MODE)
			{
				goto out;
			}
			else
			{
				ms_delay(frame_length/16);
				continue;
			}

		}

// Decide how much to read now from FW:
		// if at least 2 frames are ready - read now:
		if (fw_buffer_size_bytes > 2*frame_length_in_bytes){
			bytes_to_read = fw_buffer_size_bytes;

		}else{	// 1 frame is ready, read it:
				bytes_to_read = frame_length*2;
		}

		// anyhow - do not read more than "max_bytes_to_read":
		if (bytes_to_read > max_bytes_to_read){
				bytes_to_read = max_bytes_to_read;
		}

// Read:
		//printf("===== bytes_to_read: %d 0x%x\n",bytes_to_read, bytes_to_read);

		// request (red 20) samples, and read into "buf" (single lock transaction)
		data_len = request_and_read_chunk(chip, READ_AUDIO_BUFFER_20, bytes_to_read/16, header, 0, buf, bytes_to_read + delta);

		data_len -= delta;	// calculate real data len

		//printf("+++++ data_len: %d\n", data_len);

		if (data_len != bytes_to_read){
			Debug_Printf("Error 1! missing %d bytes\n", bytes_to_read - data_len);
		}

// Store:
   		DC_ConsumeData(DataConsumer, (uint8_t *)&buf[delta], data_len);

		current_fifo_size += data_len;

		if (time_in_sec == 0){
			wanted_fifo_size += current_fifo_size;	// we always expect 1 + what we already got
		}

	}	// while

	skip_prints = 0;

out:
	Debug_Printf ("End of pulling audio data!\n\n");

	s_streaming_in_progress = FALSE;
	s_stop_streaming_flag = FALSE;

	return 0;
}

/**********************************************************************/

// This function runs from main task or from interrupt, and is blocking till record is finished
int stop_record_buffer(int chip)
{
	if ((s_stop_streaming_flag) || (!s_streaming_in_progress)) return 0;	// no actual streaming or already during stop

	s_stop_streaming_flag = TRUE;

	while (s_streaming_in_progress)		// wait for other thread to stop streaming
		ms_delay(20);

	// s_stop_streaming_flag = FALSE - inside Dump_buffer;

	return 0;
}
#endif    


/**********************************************************************/

// This function is activated from interrupt context
// It identifies the engine number (vt_event) of the event and the word id
trigger_type get_trigger_data (int chip, int * vt_event, int * word_id, int * phrase_length )
{
	int event, VT_offset;
	trigger_type type;

	event = read_register(chip, DETECTION_AND_SYSTEM_EVENTS_STATUS_14);

	write_register(chip, DETECTION_AND_SYSTEM_EVENTS_STATUS_14, 0xe);

	// Identify offset to read the word_id
	if ((event == VT1_DET) && (WAKE_ENGINE == WWE_DOS)){
		VT_offset = VT1_REGS_OFFSET;
	}else if ((event == VT1_DET) || (event == 0x6)){
		VT_offset = VT1_REGS_OFFSET;
	}else if (event == VT2_DET){
		VT_offset = VT2_REGS_OFFSET;
	}

	*word_id = read_register(chip, VT_offset | VT_REG_WORD_ID);

	*vt_event = event;
	type = TRIGGER;

	*phrase_length = 0;

	// phrase_length is not always actual
	*phrase_length = read_register(chip, VT_offset | VT_REG_PHRASE_LENGTH_27);  ///??? to verify
	/// python: phrase_length = str(round(int(Driver.ReadRegisterShort(DBMD4,VT | VT_REG_PHRASE_LENGTH),16)/float(AUDIO_SAMPLE_RATE),3)) + " Seconds"

	Debug_Printf("vt_event = %d word=%d phrase_l = %d\n", *vt_event, *word_id, *phrase_length);

	return type;
}

/**********************************************************************/

// This function is activated from interrupt context
void move_to_bufferring(int chip)
{
	write_register(chip, OPERATION_MODE_01, BUFFERING_MODE);
}

/**********************************************************************/

void simulate_trigger(int chip)
{
	write_register(chip, FW_DEBUG_REGISTER_2_17, FORCE_DETECTION_INTERRUPT);
}

/**********************************************************************/
// This function is activated from interrupt context
void back_to_detection(int chip, trigger_type t_type)
{
	if (t_type == TRIGGER) {
		write_register(chip, OPERATION_MODE_01, DETECTION_MODE);
	} else	// commands
	{ /* do nothing, fw goes back to detection by itself */}
}

/**********************************************************************/
//checking chip operation mode
int get_operation_mode(int chip)
{
	int value = 0;

	value = read_register(chip, OPERATION_MODE_01);
	return(value);
}
/***********************************************************************/

int check_fw_errors (int chip)
{
	int ret;
	write_register(chip, FW_STATUS_INDEX_0D, READ_FW_ERROR_COUNTER);
	ret = read_register(chip, REG_IO_PORT_ADDR_LO_05);
	Debug_Printf ("fw_errors: %x\n", ret);
	if (ret != 0){
		Debug_Printf("  reg 6 = %x  reg 8 = %x \n",
			read_register(chip, REG_IO_PORT_ADDR_HI_06),
			read_register(chip, REG_IO_PORT_VALUE_HI_08));
	}

	return (ret);
}

/**********************************************************************/

//  Next 2 functions are called by BSP, when (in I2C mode) it needs to close/reopen temporarily I2C fd for CPLD / Codec

int open_IO_port(int chip)
{
	chip_io[chip] =init_IO_to_chip();

	return(0);
}

int close_IO_port(int chip)
{
	IO_Release(chip);
	return(0);
}

/**********************************************************************/

/*   Next 4 functions synchonize:
   		1. project/menu FW Mode request (g_mode_flag)
   		2. Fw log status (18 5 toggle)
   		3. Host collecting FW logs (from UART)
   		4. Actual running log type
*/

// "current_log == FW_DEBUG_LOG" tells us if 18 5 toggle is enabled in FW or not.
int current_log = FW_NO_LOG;

// Called after FW init only
void init_fw_log(int chip)
{
  Debug_Printf("[%s]\n", __func__);

	if (g_mode_flag == MODE_DEBUG) {
		current_log = FW_DEBUG_LOG;		// update current log before calling start_fw_log!
		start_fw_log(chip, FW_DEBUG_LOG);
	}else{
		// for all other cases, close FW logs (open by default)
		write_register(chip, FW_DEBUG_REGISTER_18, TOGGLE_UART_DEBUG_PRINTS);
		current_log = FW_NO_LOG;
	}
}

/**********************************************************************/

// Control FW log in term of 18 5 and host collecing logs
void set_fw_log_status(int chip, int which_log)
{
	Debug_Printf("[%s] which_log = %d (=%s)\n", __func__, which_log, which_log==2 ? "audio record" : (which_log==3 ? "debug log" : "no log"));

	if (current_log == which_log){
		Debug_Printf("fw log status already updated: %d\n",which_log);
		return;
	}else{
		if (which_log == FW_NO_LOG){
			stop_fw_log(chip, current_log);
		}else{
			if (current_log != FW_NO_LOG){
				stop_fw_log(chip, current_log);
			}
			start_fw_log(chip, which_log);
		}
	}
	current_log = which_log;
}

/**********************************************************************/

// called internally only when no log is active before
void start_fw_log(int chip, int which_log)
{
	Debug_Printf("[%s] which_log = %d\n", __func__, which_log);

	// send 18 5 for debug only, and only if not already enabled.
	if ((which_log == FW_DEBUG_LOG) && (current_log != FW_DEBUG_LOG)){
		write_register(chip, FW_DEBUG_REGISTER_18, TOGGLE_UART_DEBUG_PRINTS);
	}
	set_long_delays();		// if not set already earlier

	host_collect_fw_log_start(which_log);
}

// called internally only when a log is active before
void stop_fw_log(int chip, int which_log )
{
	Debug_Printf("[%s] which_log = %d\n", __func__, which_log);

	host_collect_fw_log_stop(which_log);

	if (which_log == FW_DEBUG_LOG){
		write_register(chip, FW_DEBUG_REGISTER_18, TOGGLE_UART_DEBUG_PRINTS);
	}
}

/**********************************************************************/
