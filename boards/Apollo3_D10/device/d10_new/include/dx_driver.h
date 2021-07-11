/**
 * dx_driver.h  --  DRIVER interface functions
 *
 * Copyright (C) 2016 DSP Group
 * All rights reserved.
 *
 * Unauthorized copying of this file is strictly prohibited.
 */

#ifndef _DX_RIVER_H
#define _DX_RIVER_H

#include <stdint.h>
#include "dx_gen.h"
//#include "osl_defs.h"
//#include "data_consumer.h"

//BEN: I added this
#define FALSE     (0)
#define TRUE      (1)
typedef struct {
  uint8_t *buf;
  uint32_t len;
}HDC;


// init
int init_IO_ports(int chip);
int config_speed(int chip, int new_speed);

int init_chip(int chip);
void chip_release(int fd);
int boot_chip(int chip);

int open_IO_port(int chip);
int close_IO_port(int chip);

// config
void set_long_delays();
int check_fw_errors (int chip);
void set_wakeup_delay(int chip, int ms);
int fw_validate(int chip);

void init_fw_log(int chip);
void set_fw_log_status(int chip, int which_log);

// load files
int load_file(int chip, char *filename, int skip_bytes);
int load_amodel_file(int chip, char *filename, int skip_bytes);

void Load_A_Model(int chip, char * filename, int load_type, int mem_loc, int vt_num);
int Load_ASRP_param(int chip, char *filename);
void model_loading(int chip, int wwe);

// registers and IO ports
int16_t read_register(int chip, int16_t reg);
void write_register(int chip, int16_t reg, int16_t val);
int32_t read_HW_port_32(int chip, int32_t address);
void write_HW_port_32(int chip, int32_t address, int32_t val);
int32_t read_io_port(int chip, int32_t addr);
void write_io_port(int chip, int32_t addr, int32_t val);

// asrp
void ASRP_change_param_indirect(int chip, int16_t blockid, int16_t offset, int16_t val);
void set_asrp_output_scale_gain(int chip, int tx_output_gain, int vcpf_output_gain, int rx_output_gain);

// defines for configure_clocks function:
#define SWITCH_TO_TDM 		1
#define NO_SWITCH_TO_TDM 	0
#define SWITCH_TO_MCLK 		1
#define NO_SWITCH_TO_MCLK 	0
#define NO_WANTED_PLL	 	0
#define NO_WANTED_AHB_CLK 	0
#define NO_WANTED_APB_CLK 	0
#define NO_USE_PLL_POST_DIV	0
#define USE_PLL_POST_DIV	1
#define DEFAULT_DMIC_FREQ	1
#define DEFAULT_AMIC_FREQ	1

int configure_clocks (int chip, bool switch_to_tdm_clk, bool switch_to_mclk, 
						int input_clk, int freq, int bit_depth, 
						int wanted_pll, int wanted_tl3_clk, 
						int wanted_ahb_clk, int wanted_apb_clk, 
						bool use_pll_post_div,
						int digital_mic_freq, int analog_mic_freq, bool i2s_chip_master);


// audio records
void asrp_record_turn_on(int chip, int asrp_record_channels1, int asrp_record_channels2, int record_command);
void asrp_record_turn_off(int chip);
void fw_record_turn_on(int chip, int record_command);
void fw_record_turn_off(int chip);

// tdm
void reset_tdm(int chip, int tdm_no);

typedef enum  {
	TDM_RX,
    TDM_TX
}tdm_direction;

typedef enum  {
	TDM_SLAVE,
    TDM_MASTER
}tdm_mode;

void configure_tdm_hw(int chip, int tdm_number, int tdm_addr, int tdm_dir, int bit_depth, int mode, int num_of_channels);

// uart
int config_uart_speed(int chip, int fw_rate, int host_rate);

// Trigger and dump functions -  may be called from interrupt context
trigger_type get_trigger_data (int chip, int * vt_event, int * word_id, int * phrase_length );
void back_to_detection(int chip, trigger_type t_type);
void simulate_trigger(int chip);
void move_to_bufferring(int chip);
int stop_dump_buffer(int chip);
int dump_buffer(int chip, HDC DataConsumer, int time_in_sec, int sample_rate, int frame_length, int number_of_channels);
int get_operation_mode(int chip);
int record_buffer(int chip, int sample_rate, int number_of_channels,  char *sample_buf, uint32_t sample_num);
void wakeup(int chip);
#endif /* _DX_RIVER_H */
