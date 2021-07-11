/**
 * dx_bsp.h  --  BSP defines.
 *
 * Copyright (C) 2016 DSP Group
 * All rights reserved.
 *
 * Unauthorized copying of this file is strictly prohibited.
 */

#ifndef _DX_BSP_H
#define _DX_BSP_H

#include <stdint.h>
#include "dx_gen.h"
#include "osl_defs.h"

/* CPLD address & registers */
#define CPLD_I2C_ADDR 	0x33
#define CPLD_NUV_ADDR 	0x1a

#define CPLD_REG0		0X00
#define CPLD_REG1		0X01
#define CPLD_REG2		0X02
#define CPLD_REG3		0X03
#define CPLD_REG4		0X04
#define CPLD_REG5		0X05
#define CPLD_REG6		0X06
#define CPLD_REG7		0X07
#define CPLD_REG8		0X08
#define CPLD_REG9		0X09
#define CPLD_REG10 		0X0a

#define CPLD_OVERWRITE_PROJECT_DEFAULTS   0x80

//#define CPLD_REG0   0x0
#define MANGO_PROJECT   	0x5
#define MELON_PROJECT_D4P   0x2
#define MELON_PROJECT_D4   	0x05
#define KIWI_PROJECT   		0x4
#define D10L_KIWI_PROJECT 	0x1
#define D10L_FPGA_PROJECT	0xF
#define D10L_PNS_PROJECT	0x2

#define ORANGE_PROJECT   0x1
#define UART_TO_FTDI   	0x40
#define CPLD_USE_EXTERNAL_MICS_EVB_L_DBMDX  0x20
#define CPLD_USE_EXTERNAL_MICS_EVB_L   		0x10
#define CPLD_USE_INTERNAL_MICS   			0x0

//#define CPLD_REG1   0x1
#define CPLD_DIFF_MCLK_CONFIG   0x40
#define CPLD_SAME_MCLK_CONFIG   0x0
#define CPLD_APR_MCLK_32K   	0x0
#define CPLD_APR_MCLK_24M   	0x8
#define CPLD_APR_MCLK_EXT   	0x10
#define CPLD_APR_MCLK_NO_CLK   	0x18
#define CPLD_APR_MCLK_HOST   	0x20
#define CPLD_APR_MCLK_12M   	0x28
#define CPLD_VT_MCLK_OR_UNI_MCLK_32K   0x0
#define CPLD_VT_MCLK_OR_UNI_MCLK_24M   0x1
#define CPLD_VT_MCLK_OR_UNI_MCLK_EXT   0x2
#define CPLD_VT_MCLK_OR_UNI_MCLK_NO_CLK   0x3
#define CPLD_VT_MCLK_OR_UNI_MCLK_HOST  0x4
#define CPLD_VT_MCLK_OR_UNI_MCLK_12M   0x5

//#define CPLD_REG2   0x2
#define CPLD_D2_TDM3_SRC_HOST_PB   			0x0
#define CPLD_D2_TDM3_SRC_MINI_STREAMER_PB   0x10
#define CPLD_D2_TDM3_SRC_HOST_FULL_DPLX   	0x20
#define CPLD_D2_TDM3_SRC_MINI_STREAMER_FULL_DPLX   0x30
#define CPLD_D2_TDM3_SRC_EXT_VIA_LED   		0x40
#define CPLD_D2_TDM3_SRC_GPIO_FLOAT   		0x50
#define CPLD_D2_TDM3_SRC_DAC_PB_ONLY   		0x70
#define CPLD_TDM0_CTL_DBMDX_MASTER   		0x8
#define CPLD_TDM0_CTL_DBMDX_SLAVE   		0x0
#define CPLD_D4_TDM0_SRC_HOST_PB  			0x0
#define CPLD_D4_TDM0_SRC_MINI_STREAMER_PB   0x0
#define CPLD_D4_TDM0_SRC_HOST_FULL_DPLX   	0x2
#define CPLD_D4_TDM0_SRC_MINI_STREAMER_FULL_DPLX   0x2
#define CPLD_D4_TDM0_SRC_EXT_VIA_LED   		0x4
#define CPLD_D4_TDM0_SRC_GPIO_FLOAT   		0x5
#define CPLD_D2_TDM3_SRC_DAC_CONNECTED_ONLY_DSP  0x60
#define CPLD_D2_TDM3_SRC_CONNECTED_RPI_AND_DSP  0x70
#define CPLD_TDM_HOST_CTL_DBMDX_MASTER  0x8
#define CPLD_TDM_HOST_CTL_DBMDX_SLAVE  0x0
#define CPLD_D4_D2_TDM0_SRC_HOST_PB  0x0
#define CPLD_D4_D2_TDM0_SRC_MINI_STREAMER_PB  0x0
#define CPLD_D4_D2_TDM0_SRC_HOST_FULL_DPLX  0x2
#define CPLD_D4_D2_TDM0_SRC_MINI_STREAMER_FULL_DPLX  0x2
#define CPLD_D4_D2_TDM0_SRC_EXT_VIA_LED  0x4
#define CPLD_D4_D2_TDM0_SRC_GPIO_FLOAT  0x5
#define CPLD_D4_D2_TDM0_SRC_DAC_CONNECTED_ONLY_DSP  0x6
#define CPLD_D4_D2_TDM0_SRC_DAC_CONNECTED_RPI_AND_DSP  0x7

//#define CPLD_REG3   0x3
#define CPLD_D5_UART_TX_SRC_D4   0x0
#define CPLD_D5_UART_TX_SRC_D2   0x1
#define CPLD_D5_I2C_D2_EN   0x2
#define CPLD_D5_I2C_D4_EN   0x4
//#define CPLD_REG4   0x4
#define CPLD_D2_D5_TDM_2_FLOAT 0x0
#define CPLD_D2_D5_TDM_2_TDM_0_SHORT 0x4
#define CPLD_D2_D5_TDM_2_DEFAULT 0xc
#define CPLD_D2_D5_D10_TDM_1_FLOAT 0x0
#define CPLD_D2_D5_D10_TDM_1_TDM_0_SHORT 0x1
#define CPLD_D2_D5_TDM_1_SPI 0x2
#define CPLD_D2_D5_TDM_1_DEFAULT 0x3

//#define CPLD_REG5   0x5
//#define CPLD_REG6   0x6
#define CPLD_MANGO_VT_TDM_1_IS_LINK_MASTER   0x0
#define CPLD_MANGO_APR_TDM_0_IS_LINK_MASTER   0x8
#define CPLD_MELON_KIWI_TDM1_CLK_FSYNC_FLOAT   0x0
#define CPLD_MELON_KIWI_TDM1_CLK_FSYNC_CONNECTED_TDM0   0x4
#define CPLD_MANGO_TDM2_CLK_FSYNC_FLOAT   0x0
#define CPLD_MANGO_TDM2_CLK_FSYNC_CONNECTED_TDM3   0x4
#define CPLD_MANGO_APR_TDM3_TX_TO_RPI   0x0
#define CPLD_MANGO_VT_TDM0_TX_TO_RPI   0x2
#define CPLD_DBMDX_IS_DRIVING_CODEC   0x0
#define CPLD_HOST_IS_DRIVING_CODEC   0x1

//#define CPLD_REG7   0x7
#define CPLD_SPI_CONNECTED_TO_HOST   0x0
#define CPLD_SPI_CONNECTED_TO_FLASH   0x1
#define CPLD_SPI_CONNECTED_TO_FTDI 0x2
#define CPLD_SPI_FLOAT 0x3
#define CPLD_UART_CONNECTED_TO_HOST 0x4
#define CPLD_UART_FLOAT 0x0
#define CPLD_I2C_CONNECTED_TO_HOST 0x8
#define CPLD_I2C_FLOAT 0x0

//#define CPLD_REG8   0x8
#define CPLD_D4_D8_STRAP_0_HIGH   0x1
#define CPLD_D4_D8_STRAP_1_HIGH   0x2
#define CPLD_D4_D8_STRAP_2_HIGH   0x4
#define CPLD_D2_STRAP_0_HIGH   0x8
#define CPLD_D2_STRAP_1_HIGH   0x10
#define CPLD_D2_STRAP_2_HIGH   0x20
#define CPLD_D2_STRAP_3_HIGH   0x40

//#define CPLD_REG9   0x9
#define CPLD_VESPER_TH_84DB_OLD_72DB   0x0
#define CPLD_VESPER_TH_78DB_OLD_65DB   0x20
#define CPLD_VESPER_TH_72DB   0x40
#define CPLD_VESPER_TH_65DB   0x30
#define CPLD_VESPER_DOUT_FLOAT 0x1F

// CPLD_REG10 0xa
#define CPLD_VESPER_MODE_FLOAT 0x1F

// CPLD_REG12 0xC
#define CPLD_DSP_GPIO_0 0x0
#define CPLD_DSP_GPIO_1 0x1
#define CPLD_DSP_GPIO_2 0x2
#define CPLD_DSP_GPIO_3 0x3
#define CPLD_DSP_GPIO_4 0x4
#define CPLD_DSP_GPIO_5 0x5
#define CPLD_DSP_GPIO_6 0x6
#define CPLD_DSP_GPIO_7 0x7
#define CPLD_DSP_GPIO_8 0x8
#define CPLD_DSP_GPIO_9 0x9
#define CPLD_DSP_GPIO_10 0xa
#define CPLD_DSP_GPIO_11 0xb
#define CPLD_DSP_GPIO_12 0xc
#define CPLD_DSP_GPIO_13 0xe
#define CPLD_DSP_GPIO_14 0xf
#define CPLD_DSP_GPIO_15 0x10
#define CPLD_DSP_GPIO_16 0x11
#define CPLD_DSP_GPIO_17 0x12
#define CPLD_DSP_GPIO_18 0x13
#define CPLD_DSP_GPIO_19 0x14
#define CPLD_DSP_GPIO_20 0x15
#define CPLD_DSP_GPIO_21 0x16
#define CPLD_DSP_GPIO_22 0x17
#define CPLD_DSP_GPIO_23 0x18
#define CPLD_DSP_GPIO_24 0x19

int  bsp_init(int chip);

void bsp_release(int fd);

void kill_mplayer(void);
void mplayer_mute(void);
void write_to_CPLD(int reg, int param);
int  reset_cpld();
void set_cpld_for_init();

int toggle_debug_gpio();

void play_to_i2s(const char *path_to_song, int rate, int bit_depth, bool if_list, bool loop, bool async_play, int channels, bool change_pulse_server,int i2s_clks);
int  record_from_i2s(const char *path_to_file, int rate, int duration, int channels, int bits, int i2s_clks);
int  user_space_playback_16b(const char *path_to_song); // not used
int  user_space_playback_32b(const char *path_to_song); //not used
void exit_play_and_record(void);
void change_pavucontrol_params(int sample_rate, int bit_depth, int out_channels, int kill, int i2s_clks);
void mute_unmute_music();
void set_play_volume(int volume);

unsigned long get_file_size(char *filename);
int  get_wav_file_duration(char *filename);

void update_host_debug_device_speed(int speed);

void host_collect_fw_log_start();
void host_collect_fw_log_stop();

#endif /* _DX_BSP_H */
