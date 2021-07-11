/*
 * dx_bsp.c -- BSP functions (RPI and EVBL board peripherals, all NOT related to communication with dspg chip
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
#include <ctype.h>
#include <signal.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <sys/eventfd.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/shm.h>
#include "dx_params.h"
#include "dx_bsp.h"
#include "dx_utils.h"
#include "dx_gen.h"
#include "dx_driver.h"
#include "dbmx_gpio.h"

#define DEBUG 1

int music_pid = 0;
int record_pid = 0;
pid_t play_pid;

extern int g_transport;
extern int g_chip1;		// chip
extern int g_tdm_sample_rate;
extern int g_tdm_bit_rate;
extern int g_i2s_chip_master;
extern int g_master_clock;

// I2C defines for CPLD, Codec, not real transport
int i2c_address = I2C_DEVICE_ADDRESS;			// I2C address of chip
int i2c_addressCPLD = CPLD_I2C_ADDR;		// I2C address of CPLD
int i2c_nuv_addr = CPLD_NUV_ADDR; 		// I2C address of nuvoton */
char *i2c_dev = I2C_DEVICE_PATH ;

void bsp_release(int fd);

void init_debug_device(int chip);
int	init_pavucontrol_params();
int reset_debug_gpio();

/***********************************************************************
* FUNCTION NAME: bsp_init()
*
* DESCRIPTION:
*	init all BSP (RPI + EVBL) components (not related to chip)

* PARAMETERS:
*
* RETURNS:
*	0 on sucsses
*	error otherwise
*
***********************************************************************/
int bsp_init(int chip)
{
	reset_cpld();
	reset_debug_gpio();
	
	set_cpld_for_init();

	// in the past we called here to configure Nuvoton

	system("killall -9 mplayer");
	system("killall -9 arecord");

	system("mkdir Log/");

	init_pavucontrol_params();

	init_debug_device(chip);

	return 0;
}

/*******************************************************************/
// Write to CPLD.
// If I2C transport case, close and re-open chip I2C fd

void write_to_CPLD(int reg, int param)
{
	int err;
	char buf[2];
	int i2c;

	//printf("[%s]: Start.\n", __func__);

	// If required, close temporarily the I2C device
	if (g_transport == TRANSPORT_I2C) 
	{
		close_IO_port(g_chip1);	
	}

	// Open the I2C device 
	i2c = open(i2c_dev, O_RDWR);
	if (i2c < 0) {
		printf("i2c < 0\n");
		exit(1);
	}

	tcdrain(i2c);
	// CPLD 
	if((ioctl(i2c, I2C_SLAVE, i2c_addressCPLD)) < 0) {
		printf("ERROR Selecting I2C device\n");
		exit(2);
	}

	ms_delay(10);

	buf[0] = reg;
	buf[1] = param;
	printf("[%s]: Write address 0x%X, param 0x%X.\n", __func__, reg, param);
	write(i2c, buf, 2);
	tcdrain(i2c);

	err = close(i2c);
	if (err)
		printf("ERROR writing I2C device %x",err);

	// Re-Open the I2C device if required
	if (g_transport == TRANSPORT_I2C) 
	{
		open_IO_port(g_chip1);
	}
}

/******************************************************************/

int reset_cpld()
{

	if(gpio_request(CPLD_RST_GPIO) < 0)
		goto err1;
	ms_delay(60);
	if(gpio_direction_output(CPLD_RST_GPIO, HIGH) < 0)
		goto err1;

	gpio_set_value(CPLD_RST_GPIO, HIGH);
	ms_delay(10);				
	gpio_set_value(CPLD_RST_GPIO, LOW);
	ms_delay(100);// 60 + 40				

	return 0;

err1:
	return -1;
}

/******************************************************************/

int reset_debug_gpio()
{
	if(gpio_request(DEBUG_GPIO) < 0)
		goto err1;
	ms_delay(60);
	if(gpio_direction_output(DEBUG_GPIO, HIGH) < 0)
		goto err1;

	return 0;

err1:
	return -1;
}

int toggle_debug_gpio()
{
	gpio_set_value(DEBUG_GPIO, HIGH);
	ms_delay(1);				
	gpio_set_value(DEBUG_GPIO, LOW);
	ms_delay(1);				
	return 0;
}


/******************************************************************/

void set_cpld_for_init()
{
	//??? should be updated

	int if_external_mics;
	int cpld_spi, cpld_ice, cpld_uart, cpld_mclock;
	
	// CPLD configuration (for EVBL)
#ifdef EXTERNAL_MICS
	if_external_mics = CPLD_USE_EXTERNAL_MICS_EVB_L_DBMDX;
#else
	if_external_mics = 0;
#endif

	write_to_CPLD(CPLD_REG0, CPLD_OVERWRITE_PROJECT_DEFAULTS	|
							if_external_mics					|
							UART_TO_FTDI						|
							D10L_KIWI_PROJECT);																			

	cpld_spi = CPLD_SPI_FLOAT;
	cpld_ice = CPLD_I2C_FLOAT;
	cpld_uart = CPLD_UART_FLOAT;
	// correct actual transport channel:
	if (g_transport == TRANSPORT_SPI) {cpld_spi = CPLD_SPI_CONNECTED_TO_HOST;}
	else if (g_transport == TRANSPORT_I2C) {cpld_spi = CPLD_I2C_CONNECTED_TO_HOST;}
	else if (g_transport == TRANSPORT_UART) {cpld_spi = CPLD_UART_CONNECTED_TO_HOST;}

	write_to_CPLD(CPLD_REG7, CPLD_OVERWRITE_PROJECT_DEFAULTS	|
									cpld_ice					|
									cpld_uart					|
									cpld_spi);

	write_to_CPLD(CPLD_REG6, CPLD_OVERWRITE_PROJECT_DEFAULTS     |
                         CPLD_MELON_KIWI_TDM1_CLK_FSYNC_FLOAT    |
                         CPLD_HOST_IS_DRIVING_CODEC);

	write_to_CPLD(CPLD_REG9, CPLD_OVERWRITE_PROJECT_DEFAULTS     |
							CPLD_VESPER_DOUT_FLOAT);
	
	write_to_CPLD(CPLD_REG10, CPLD_OVERWRITE_PROJECT_DEFAULTS     |
							CPLD_VESPER_DOUT_FLOAT);

	if (g_master_clock == 32768) {cpld_mclock=CPLD_VT_MCLK_OR_UNI_MCLK_32K;}
	else 
	if (g_master_clock == 12288) {cpld_mclock=CPLD_VT_MCLK_OR_UNI_MCLK_12M;}
	else 
	if (g_master_clock == 19200) {cpld_mclock=CPLD_VT_MCLK_OR_UNI_MCLK_HOST;}
	else 
	if (g_master_clock == 24576) {cpld_mclock=CPLD_VT_MCLK_OR_UNI_MCLK_24M;}

	write_to_CPLD(CPLD_REG1, CPLD_OVERWRITE_PROJECT_DEFAULTS | 
								cpld_mclock);
}

/******************************************************************/
// Not used function, as BSP always closes the fd after any configuration of CPLD or Nuvotone
void bsp_release(int fd)
{
	if(fd > 0) {
		close(fd);
		fd = -1;
	}
}

/******************************************************************/

// WAV file Utilities

unsigned long get_file_size(char *filename)
{
	FILE *file;
	unsigned long fileLen;

	// Open the file
	file = fopen(filename, "rb");
	if(!file) {
		fprintf(stderr, "Unable to open file %s\n", filename);
		return -1;
	}

	//Get file length
	fseek(file, 0, SEEK_END); 	// seek to the end of file
	fileLen = ftell(file); 		// get current file pointer
	fseek(file, 0, SEEK_SET); 	// seek back to beggining of file
	return fileLen;
}

/******************************************************************/

// Used only after recording of TDM in RPI host 
/* WAVE file header format */
typedef struct WAV_HEADER {
    char riff[4];				/* RIFF string */
    int overall_size;		    /* overall size of file in bytes */
    char wave[4];               /* WAVE string */
    char fmt_chunk_marker[4];   /* fmt string with trailing null char */
    int length_of_fmt;          /* length of the format data */
    short format_type;          /* format type. 1-PCM, 3- IEEE float, 6 - 8bit A law, 7 - 8bit mu law */
    short channels;             /* no.of channels */
    int sample_rate;            /* sampling rate (blocks per second) */
    int byterate;               /* SampleRate * NumChannels * BitsPerSample/8 */
    short block_align;          /* NumChannels * BitsPerSample/8 */
    short bits_per_sample;      /* bits per sample, 8- 8bits, 16- 16 bits etc */
    char data_chunk_header [4];	/* DATA string or FLLR string */
    int data_size;              /* NumSamples * NumChannels * BitsPerSample/8 - size of the next chunk that will be read */
} wav_hdr;

/******************************************************************/

int get_wav_file_duration(char *filename)
{
	FILE *file;
	wav_hdr wavHeader;
	int headerSize = sizeof(wav_hdr);
	int duration_in_seconds;
	int numSamples;

	/* Open the file */
	file = fopen(filename, "r");
	if(!file) {
		fprintf(stderr, "Unable to open file %s\n", filename);
		return -1;
	}

	fread(&wavHeader, 1, headerSize, file);

	printf("wavHeader.data_size = %d\n", wavHeader.data_size);
	printf("wavHeader.bits_per_sample = %d\n", wavHeader.bits_per_sample);
	printf("wavHeader.channels = %d\n", wavHeader.channels);

	/* calculate duration of file */
	numSamples = wavHeader.data_size / (wavHeader.channels * (wavHeader.bits_per_sample/8));

	printf("numSamples = %d\n", numSamples);
	printf("wavHeader.sample_rate = %d\n", wavHeader.sample_rate);
	duration_in_seconds =  numSamples / wavHeader.sample_rate;
	printf("Approx.Duration in seconds=%d\n", duration_in_seconds);
	fclose(file);
	return duration_in_seconds;
}

// Mplayer functions

/***********************************************************************
* FUNCTION NAME: user_space_playback_16b
*
* DESCRIPTION:
*	Fork & call aplay from user space
* PARAMETERS:
*	char * path to song
* RETURNS:
*	0 on sucsses else error
***********************************************************************/
// Not used today
int user_space_playback_16b(const char *path_to_song)
{
	char system_str[64] = {0};
	if((music_pid = fork()) == 0) {
		setgid(1000);
		setuid(1000);
		putenv("HOME=/home/pi/");
		sprintf(system_str, "aplay -v %s", path_to_song);
		system(system_str);
		DEBUG_PRINT("[%s]: I'm in the child. music_pid = %d.\n", __func__, music_pid);
		exit(0);
	} else {
		DEBUG_PRINT("[%s]: I'm in the parrent. music_pid = %d.\n", __func__, music_pid);
	}
	return 0;
}

/***********************************************************************
* FUNCTION NAME: user_space_playback_32b
*
* DESCRIPTION:
*	Fork & call aplay from user space
* PARAMETERS:
*	char * path to song
* RETURNS:
*	0 on sucsses else error
***********************************************************************/
// Not used today
int user_space_playback_32b(const char *path_to_song)
{
	char system_str[64] = {0};
	if((music_pid = fork()) == 0) {
		setgid(1000);
		setuid(1000);
		putenv("HOME=/home/pi/");
		sprintf(system_str, "aplay -vD dspg %s", path_to_song);
		system(system_str);
		DEBUG_PRINT("[%s]: I'm in the child. music_pid = %d.\n", __func__, music_pid);
		exit(0);
	} else {
		DEBUG_PRINT("[%s]: I'm in the parrent. music_pid = %d.\n", __func__, music_pid);
	}

	return 0;
}

/***********************************************************************
* FUNCTION NAME: record_from_i2s
*
* DESCRIPTION:
*	Fork & call arecord from user space
* PARAMETERS:
*	char * path to song
* 	int rate
* 	int duration (0 infinity)
* 	int bits
* RETURNS:
*	0 on sucsses else error

record_audio_via_i2s(16000, "s16le", channels, "Mic_To_TDM_Mic_1_%s_Mic_2_%s_%shz_%s_Bits.wav" %(mic_1_type, mic_2_type, 16000, "s16le"))

def record_audio_via_i2s (sample_rate,bit_depth,channels,file_name = "record.wav",record_time = 0):		

	if ((len(str(sample_rate)) < 3) or (sample_rate == 44.1)):			
		sample_rate *= 1000	bit_depth = bit_depth.upper()		
	if len(bit_depth) < 6:
		bit_depth = bit_depth[:3] + "_" + bit_depth[3:]

shell_command("arecord -D plughw 
						-r %s 
						-f %s 
						-c %s 
						-d %d %s" 
						%(str
							(sample_rate),
							str(bit_depth), 
							str(channels), 
							int(record_time), 
							str(file_name)) 
						,True)	

shell_command("arecord -D plughw 
						-r %s 	(sample_rate)	16000
						-f %s 	str(bit_depth)	"s16le"
						-c %s 	str(channels)	channels
						-d %d  	int(record_time) 
						%s" 	str(file_name)

***********************************************************************/
int record_from_i2s(const char *path_to_file, int rate, int duration, int channels, int bits, int i2s_clks)
{
	char system_str[100] = {0};
						
	sprintf(system_str, "arecord -D %s  -r %d -f S%d_LE -d %d -c %d %s &", 
										i2s_clks ? "plughw:CARD=dspgDual,DEV=1": "plughw",
										rate, bits, duration, channels, path_to_file);								

	printf("%s\n",system_str);
	system(system_str);

	return 0;
}

/**********************************************************************/
typedef enum  {
	OVERLAY_NO_CARD,
	OVERLAY_CHIP_MASTER,
    OVERLAY_CHIP_SLAVE
}t_overlay;

int s_overlay = OVERLAY_NO_CARD;	// at init: set actual overlay to no card

int	init_pavucontrol_params()
{
	// for RPI Image 5:
	// before killing pulseaudio, verify that autospawn = no (otherwise it will pop up again)
	DEBUG_PRINT("system:  sudo rm /etc/pulse/client.conf \n");
	system("sudo rm /etc/pulse/client.conf");

	DEBUG_PRINT("system: sudo echo 'autospawn = no' > /etc/pulse/client.conf \n");
	system("sudo echo 'autospawn = no' > /etc/pulse/client.conf");

	DEBUG_PRINT("system: pulseaudio -k  \n");
	system("pulseaudio -k");

	// clear any overlay if exists
	DEBUG_PRINT("3 times system: sudo dtoverlay -r \n");
	system("sudo dtoverlay -r");
	system("sudo dtoverlay -r");
	system("sudo dtoverlay -r");

	return 0;
}

int s_pavu_active = 0;	// flag keeps if pavu is not closed.

void change_pavucontrol_params(int sample_rate, int bit_depth, int out_channels, int kill, int i2s_clks)
{
	char system_str[256] = {0};

	bool change_overlay = FALSE;

	// if already closed and request is to close - exit
	if ((!s_pavu_active) && kill) return;
	s_pavu_active = !kill;
	
	printf ("change_pavucontrol_params: sample_rate= %d bit_depth= %d out_channels= %d kill=%d i2s_clks= %d\n", sample_rate, bit_depth, out_channels, kill, i2s_clks);

	// change overlay (audio card) at init or if different from already loaded:
	change_overlay = 
		((s_overlay == OVERLAY_NO_CARD) /*init*/ 		  || 
		((s_overlay == OVERLAY_CHIP_MASTER) && !i2s_clks) ||
		((s_overlay == OVERLAY_CHIP_SLAVE) && i2s_clks));

	// kill maplayer
	DEBUG_PRINT("system: sudo pkill mplayer \n");
	system("sudo pkill mplayer");

//////////////-->
	// for RPI Image 5:
	// before killing pulseaudio, verify that autospawn = no (otherwise it will pop up again)
	// rm daemon conf file
	DEBUG_PRINT("system:  sudo rm /etc/pulse/client.conf \n");
	system("sudo rm /etc/pulse/client.conf");
	
	DEBUG_PRINT("system: sudo echo 'autospawn = no\n' > /etc/pulse/client.conf \n");
	system("sudo echo 'autospawn = no\n' > /etc/pulse/client.conf");

///////////////<--

// kill pulsaudio 
	DEBUG_PRINT("system: pulseaudio -k  \n");
	system("pulseaudio -k");

	if (!kill){		// revive pulseaudio to work (afterkilling)

//////////////-->
		// renew daemon.conf file:
		// rm daemon conf file
		DEBUG_PRINT("system:  sudo rm /etc/pulse/daemon.conf \n");
		system("sudo rm /etc/pulse/daemon.conf");
		
		DEBUG_PRINT("sudo echo 'default-sample-format = s%dle\ndefault-sample-rate = %d' > /etc/pulse/daemon.conf \n",bit_depth,sample_rate);
		sprintf(system_str,"sudo echo 'default-sample-format = s%dle\ndefault-sample-rate = %d' > /etc/pulse/daemon.conf",bit_depth,sample_rate);
		system(system_str);

///////////////<--

///////////////-->
		// rm config files of pulse and pavucontrol.ini
		DEBUG_PRINT("system: sudo rm -R /home/pi/.config/pulse  \n");
		system("sudo rm -R /home/pi/.config/pulse");

		DEBUG_PRINT("system:  sudo rm -r /home/pi/.config/pavucontrol.ini \n");
		system("sudo rm -r /home/pi/.config/pavucontrol.ini");
//////////////<--

////////--->
// load proper overlay only if required
		if(change_overlay){
			DEBUG_PRINT("system: sudo dtoverlay -r  \n");
			
			system("sudo dtoverlay -r");

			DEBUG_PRINT("system: sudo dtoverlay ... \n");
			sprintf(system_str, "sudo dtoverlay %s", (i2s_clks == i2s_clks_chip_slave)? "hifiberry-dac" : "dspg-dual");
			system(system_str);	

			s_overlay = (i2s_clks == i2s_clks_chip_slave)? OVERLAY_CHIP_SLAVE : OVERLAY_CHIP_MASTER;
		}
//////<-----

		// activate pulseaudio
		DEBUG_PRINT("system: pulseaudio -D \n");
		system("pulseaudio -D");

		ms_delay(100);

		// config pulseaudio 1
		DEBUG_PRINT("system: pacmd set-card-profile 0 output:analog-stereo  \n");
		system("pacmd set-card-profile 0 output:analog-stereo");

		if (i2s_clks == i2s_clks_chip_slave){
			printf("RPI I2S Clocks are set to %d .1fKhz %d sbit %s - RPI is Master\n", sample_rate, bit_depth, (out_channels != 2)? "mono" : "stereo");

		}else{
			printf("RPI is Slave\n");

			// config pulseaudio 2
			DEBUG_PRINT("system: pacmd set-sink-volume 0 0x9500  \n");
			system("pacmd set-sink-volume 0 0x9500");
		}

		if (out_channels == 2){
			// config pulseaudio 3.1
			DEBUG_PRINT("system: pacmd unload-module module-remap-sink  \n");
			system("pacmd unload-module module-remap-sink");

			// config pulseaudio 4.1
			DEBUG_PRINT("system: pacmd set-default-sink 0  \n");
			system("pacmd set-default-sink 0");
		}
		else{
			// config pulseaudio 3.2
			DEBUG_PRINT("system: pacmd load-module module-remap-sink sink_name=mono master= channels=2 channel_map  \n");
			sprintf(system_str, "pacmd load-module module-remap-sink sink_name=mono master=%s channels=2 channel_map=mono,mono", (i2s_clks == i2s_clks_chip_slave)? "alsa_output.platform-soc_sound.analog-stereo" : "alsa_output.platform-dual-soundcard.analog-stereo" );
			system(system_str);

			// config pulseaudio 4.2
			DEBUG_PRINT("system: spacmd set-default-sink 1  \n");
			system("pacmd set-default-sink 1");
		}
	}
	else{
		printf("PulseAudio Server is down, RPI I2S Clocks are closed");
	}
}
/**********************************************************************/
void kill_mplayer()
{
    system("killall mplayer");
}

/**********************************************************************/

void play_to_i2s(const char *path_to_song, int rate, int bit_depth, bool if_list, bool loop, bool async_play, int channels, bool change_pulse_server, int i2s_clks)
{
/*
	if (change_pulse_server){
		change_pavucontrol_params(rate,bit_depth,channels, FALSE, i2s_clks);
	}	
*/
	char system_str[256] = {0};

	//sprintf(system_str, "mplayer -ao %s -quiet softvol %s -slave -novideo -input file=/tmp/mplayer-control %s %s </dev/null >/dev/null 2>&1 &", 
	sprintf(system_str, "mplayer -ao %s -really-quiet softvol %s -slave -novideo -input file=/tmp/mplayer-control %s %s </dev/null >/dev/null 2>&1 &", 
													//!i2s_clks? "pulse" : "alsa:device=dspg",
													"pulse",				/*always pulse*/
													loop? " -loop 0" : "",   
													if_list? " -playlist" : "",   
													path_to_song);

	printf("%s\n", system_str);
	system(system_str);
}

/****************************************************************/

void exit_play_and_record(void)
{
	if(music_pid > 0)
		system("killall -9 aplay");

	if(record_pid > 0)
		system("killall -9 arecord");

	if(play_pid > 0) {
		system("killall -9 mplayer");
	}
}

/******************************************************************************************/

/**********		HOST DEBUG UART	connected between RPI and FW Uart port	*******************/

#include "dbmx_uart.h"

// these defines should be moved to dbmx_uart.h
#define UART_D2_BOOT_PARITY		1
#define UART_D2_NORMAL_PARITY	0
#define UART_BOOT_STOP_BITS		2
#define UART_NORMAL_STOP_BITS	1

#define UART_LOG_DEVICE_PATH1 "/dev/ttyUSB1"
#define UART_LOG_DEVICE_PATH2 "/dev/ttyUSB2"
#define UART_LOG_DEVICE_PATH3 "/dev/ttyUSB3"

int uart_fd;
char * s_uart_log_device_path;

int device_in_use = FALSE;
int stop_device_activity = FALSE;

pthread_t log_polling_pid = 0;
pthread_attr_t attr;

/**********************************************************************/

int uart_platform_configure_tty(int fd, int bps, int stop, int parity, int flow);

void init_debug_device(int chip)
{
	int initial_baudrate;
	
	if (chip == DBM_D10 ){
			s_uart_log_device_path = UART_LOG_DEVICE_PATH3;
			initial_baudrate = UART_SPEED_1024KHz;
	}else
	if (chip == DBM_D2 ){
			s_uart_log_device_path = UART_LOG_DEVICE_PATH1;
			initial_baudrate = UART_SPEED_921600;
	}else
	 {
			s_uart_log_device_path = UART_LOG_DEVICE_PATH2;
			initial_baudrate = UART_SPEED_921600;
	 }

	pthread_attr_init(&attr);

    uart_fd = open(s_uart_log_device_path, O_RDWR | O_NONBLOCK | O_NOCTTY);
    if (uart_fd == -1) {
        printf("[%s]: UART device node open failed errno %d\n", __func__, errno);
        return; 
    }

	uart_platform_configure_tty(uart_fd, initial_baudrate, UART_NORMAL_STOP_BITS, UART_D2_NORMAL_PARITY,0);
}


void update_host_debug_device_speed(int speed)
{
#ifdef HOST_COLLECT_FW_LOGS
	uart_platform_configure_tty(uart_fd, speed, UART_NORMAL_STOP_BITS, UART_D2_NORMAL_PARITY,0);
#endif
}

/**********************************************************************/

#define DEBUG_LOG_NAME	"Logic_out.txt"
#define RECORD_LOG_NAME	"Logic_out.bin"

int debug_count  =0;	// counter of debug logs since last reset
int record_count =0;	// counter of record logs since last reset

int actual_log = FW_NO_LOG;

void * uart_log_polling_thread(void *ignore)
{
	FILE * log_fd;
	char buf[256];
	int bytes, actual_count;
	char name_str[100];
	char sys_command[100];
	char * actual_log_name;

	if (actual_log == FW_DEBUG_LOG){
		actual_log_name = DEBUG_LOG_NAME;
		debug_count++;
		actual_count = debug_count;
	}else{
		actual_log_name = RECORD_LOG_NAME;
		record_count++;
		actual_count = record_count;
	}

	ioctl(uart_fd, TCFLSH, 2);

	sprintf(name_str, "Log/%d_%s",actual_count, actual_log_name );

	log_fd = fopen(name_str, "wb");

	if (!log_fd){
		printf ("No UART connection between host and target!\n" );
		return (0);
	}

	printf("Start collecting to FW Log File: %s \n", name_str);

	device_in_use = TRUE;

	while(!stop_device_activity) {

        bytes = read(uart_fd, buf, 255);
        if (bytes == -1) {
           // error - get out
        }
        else if (bytes == 0) {

            ms_delay(20);
        }
        else {
			fwrite(buf, bytes, 1, log_fd);
			fflush(log_fd);
			//printf(" %d bytes read \n", bytes);
        }
    }

	stop_device_activity = FALSE;

	if (!log_fd){
		printf ("No UART connection between host and target!\n" );
		return(0);
	}

	fclose(log_fd);

	printf("FW Log File: %s is closed and ready! \n", name_str);

	sprintf(sys_command, "chmod 777 %s",name_str);
	system(sys_command);
	
	device_in_use = FALSE;

	return (0);
}

/**********************************************************************/

void host_collect_fw_log_start(int which_log)
{
	printf("[%s] which_log = %d\n", __func__, which_log);

#ifdef HOST_COLLECT_FW_LOGS

	actual_log = which_log;

	if (device_in_use) {
		printf("debug device is already in use!\n");
	}
	else{
		pthread_create(&log_polling_pid, &attr, uart_log_polling_thread, NULL);
		printf("pthread_create\n");
	}

#endif
	return;

}

/**********************************************************************/

void host_collect_fw_log_stop(int which_log)
{
	printf("[%s] which_log = %d\n", __func__, which_log);

#ifdef HOST_COLLECT_FW_LOGS

	stop_device_activity = TRUE;
	while (device_in_use)
		ms_delay(10);

	actual_log = FW_NO_LOG;
#endif

	return; 

}

/**********************************************************************/

void set_play_volume(int volume)
{
	char system_str[256] = {0};
	
	sprintf(system_str, "pactl set-sink-volume 0 %d%%", volume);
	system(system_str);
	printf("Volume is set to %d%%\n", volume);
	return;
}
