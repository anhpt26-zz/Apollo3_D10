/*
 * dx_host_refapp.c
 *
 * main host application of project
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
#include "dx_driver.h"
#include "dx_utils.h"
#include "dx_usecases.h"
#include "dbmx_gpio.h"
#include "dbmx_spi.h"
#include "dbmx_uart.h"
#include "dx_gen.h"
#include "dx_fw_defs.h"
#include "osl_task.h"

//*****************************************************************************/
//						GUI and Menu

#define RESET		0
#define BRIGHT 		1
#define DIM 		2
#define UNDERLINE 	3
#define BLINK 		4
#define REVERSE 	6
#define HIDDEN 		8

#define BLACK		0
#define RED 		1
#define GREEN 		2
#define YELLOW 		3
#define BLUE 		4
#define MAGENTA 	5
#define CYAN 		6
#define WHITE 		7


int segmentId;

unsigned int *sared_stop_streaming_flag;

/* Local functions */

void print_dspg_versions();
void print_actual_states();

void event_interrupt(int chip);
void vt_vc_trigger(int chip);
void prepare_project_configuration();
int  parse_config_file(char* filename);
int  use_case_exit(int use_case);
void launch_record_thread();
void *record_thread(void *ignore);

void menu(void);
void testcolor(int attr, int fg, int bg);


extern int use_case_wakeup_detection_Enter(int chip, int wwe);
extern int use_case_wakeup_detection_Exit(int chip);
extern void use_case_wakeup_vt_vc_trigger(int chip);
extern void use_case_wakeup_print_commands(void);
/////////////////////////////////////////////////////////////////////////////////

// Project Global Variables

// Parameters that can be re-configured in config.ini file - should be initialized inside "prepare_project_configuration" function.
int g_chip1;
int g_chip2;
int g_use_wakeup;
int g_buffer_time_in_sec;
int g_mic1_type;
int g_mic2_type;

int g_mode_flag = FW_MODE;

int g_reg5_fw_record;
int g_reg6_fw_record;
int g_reg7_fw_record;
bool g_disable_asrp_record = FALSE; 	// option to enable FW audio records but no asrp

int g_spi_speed;
int g_uart_speed;

int g_master_clock;

int g_i2s_chip_master;
int g_tdm_sample_rate;
int g_tdm_bit_rate;
int g_no_of_speakers;

int g_transport = TRASPORT_CHAN;

int g_wake_word_engine;
char * g_dspg_host_ver = DSPG_C_REFERENCE_HOST_VERSION;

char * g_FirmWare_filename;
char * g_trigger_filename;

int g_asrp_delay;
int g_asrp_lib_version;
int g_asrp_parameter_version;

int g_support_rx_process;
int g_google_assist_mode;
int g_use_spi_in_ga_mode;

/* Global variable default setting (according to parameters.h) */
/* Some of then can be overrive by config_init.txt file */

void prepare_project_configuration()
{
	g_chip1 = CHIP_1;

	switch (MASTER_CLOCK)
	{
		case 1:
		g_master_clock = MASTER_CLOCK_32768;
		break;

		case 2:
		g_master_clock = MASTER_CLOCK_19200;
		break;

		case 3:
		g_master_clock = MASTER_CLOCK_24576;
		break;

		case 5:
		g_master_clock = MASTER_CLOCK_12288;
		break;
	}

	g_use_wakeup = WAKE_UP;
	g_buffer_time_in_sec = DUMP_SECONDS;
	g_mic1_type = MIC1;
	g_mic2_type = MIC2;

	g_wake_word_engine = WAKE_ENGINE;
    g_asrp_delay = ASRP_DELAY;

	// g_mode_flag = MODE_OPT;		// Do not change here: It could already be changed by a command line parameter
	// g_transport = TRANSPORT_NOT_YET;		// ! do not clear here, as value is saved before "init" option in menu

	g_spi_speed = SPI_SPEED_BOOT_MAX;
	g_uart_speed = UART_SPEED_1MHz;

	g_reg5_fw_record = REG_5_FW_RECORD;
	g_reg6_fw_record = REG_6_FW_RECORD;
	g_reg7_fw_record = REG_7_FW_RECORD;

	g_i2s_chip_master = I2S_CHIP_MASTER;
	g_tdm_sample_rate = TDM_SAMPLE_RATE;
	g_tdm_bit_rate = TDM_BIT_RATE;
	g_no_of_speakers = NO_OF_SPEAKERS;

	g_support_rx_process = SUPPORT_RX_PROCESS;
	g_google_assist_mode = GOGGLE_ASSISTANT_MODE;
	g_use_spi_in_ga_mode = USE_SPI_IN_GA_MODE;

	// correct some parameter values according to the configuration INI file:
	parse_config_file(CONFIG_FILENAME);

	g_FirmWare_filename = FIRMWARE_IMAGE;
#if 0
	if (g_wake_word_engine == WWE_SENSING_SED){
		g_trigger_filename = SED_MODEL_FILENAME;	
	}else{
		g_trigger_filename = T3T4_MODEL_FILENAME;
	}
#else
	g_trigger_filename = "none";
#endif
}

/////////////////////////////////////////////////////////////////////////////////

/* Static variable definitions */

pid_t play_pid;

static volatile int keepRunning = 1;
int chip2;

//int s_trigger_model_loaded = FALSE;
int s_command_model_loaded = FALSE;

int s_state = 0;

int s_count;			// counter of triggers

int s_music_on = FALSE;

/////////////////////////////////////////////////////////////////////////////////

/* Signal handling Ctrl+C to exit program correctly */
void intHandler(int dummy)
{
	printf("[intHandler]: Got Ctrl-C.\n");
	keepRunning = 0;

	use_case_exit(s_state);

	exit_play_and_record();

	chip_release(g_chip1);

	shmdt(sared_stop_streaming_flag);

	shmctl(segmentId, IPC_RMID, NULL);

	exit(0);
}

/***********************************************************************/

void pabort(const char *s)
{
	perror(s);
	chip_release(g_chip1);
	abort();
}

/***********************************************************************/

// Print versions of all components of DSPG deliverables
void print_dspg_versions()
{
	uint16_t dspg_fw_version;
	//uint16_t dspg_asrp_engine_version; // ASRP is not loaded yet


	printf("customer host version = %s\n", CUSTOMER_HOST_VERSION);

	printf("dspg .c reference file version = %s\n\n", g_dspg_host_ver);

	printf("dspg Base Version = %s\n", DSPG_BASE_VERSION);

	dspg_fw_version = read_register(g_chip1, 0);
	//dspg_asrp_engine_version = read_register(g_chip1, 0x100);
	printf("dspg_fw_version = 0x%x\n", dspg_fw_version);

	//printf("dspg_asrp_engine_version = 0x%x\n", dspg_asrp_engine_version);

	printf("dspg FW file name = %s\n", g_FirmWare_filename);
	printf("Model file name = %s\n\n", g_trigger_filename);

}

void print_actual_states()
{
	printf("current main state = uc%d\n",s_state);
	//printf("fw mode = %d    1=no-log; 2=fw-record;  3=fw-debug \n", g_mode_flag);
	printf("fw mode = %s \n", (g_mode_flag == 1) ? "fw-fast" : ((g_mode_flag == 2) ? "fw-record" : "fw-debug")) ;
	printf("fw state = %d\n", read_register(g_chip1, OPERATION_MODE_01));

}
/**************************************************************************/

// Print actual project configuration (parameters)
void print_project_configuration()
{
// add here more parameters from parameters.h

	printf("Project Configuration: \n") ;
	printf("     chip = D%d \n", g_chip1) ;
	printf("     mode = %s \n", (g_mode_flag == 1) ? "fw-fast" : ((g_mode_flag == 2) ? "fw-record" : "fw-debug")) ;
	printf("     reg fw records:  5=%x; 6=%x; 7=%x  \n", g_reg5_fw_record, g_reg6_fw_record, g_reg7_fw_record ) ;
	printf("     transport = %d    %s \n", g_transport, "0=not yet; 1=SPI; 2=I2C 3=UART") ;
	printf("     wakeup = %d \n", g_use_wakeup) ;
	printf("     mic1 type = %d    %s \n", g_mic1_type, "0=none; 1=digital; 2=analog; 3=vesper") ;
	printf("     mic2 type = %d    %s \n", g_mic2_type, "0=none; 1=digital; 2=analog; 3=vesper") ;
	printf("     buffer_time_in_sec = %d \n", g_buffer_time_in_sec) ;
	printf("     wake word engine = %d    %s\n", g_wake_word_engine,  "0=none; 1=Sensory; 2=Google;");
 	printf("     i2s master = %s \n", (g_i2s_chip_master)? "Chip" : "Host");
	printf("     host tdm sample rate = %d \n", g_tdm_sample_rate) ;
	printf("     host tdm bit rate = %d \n", g_tdm_bit_rate) ;
}
/**************************************************************************/

// Exit from any active usecase into idle
int use_case_exit(int use_case)
{
	//printf ("[%s]: s_state = %d.\n", __func__, use_case);

	s_state = 0;

	switch(use_case) {
		case 1:
			use_case_wakeup_detection_Exit(g_chip1);
			break;
		case 100:
			use_case_reg_table_only_exit(g_chip1);
			break;
		default :
			printf ("[%s]: s_state = %d, do nothing.\n", __func__, use_case);
			break;
	}
	check_fw_errors(g_chip1);	// for debug only
	printf ("[%s]: Finish!\n", __func__);
	return 0;
}

/**********************************************************************/
void *ready_polling_thread(void *ignore)
{
	printf("!!! starting READY thread !!!\n");
	while(1) {
		int ret;

		ret = gpio_poll(INTERRRUPT_READY_GPIO);
		//printf("!!! READY !!!\n");
		if((ret != 1) && (ret != 0)) {
			printf ("[%s]: ready_gpio_poll fails ret = %d\n", __func__, ret);
			return NULL;
		}
		set_ready_flag (g_chip1, FLAG_SET);
	}
}
/**********************************************************************/

void *trigger_polling_thread(void *ignore)
{

	while(1) {
		int ret;

		ret = gpio_poll(INTERRRUPT_TRIGGER_GPIO);

		printf("Event happened!\n");

		if(ret != 1) {
			printf ("[%s]: Event_gpio_poll return \n", __func__);
			return NULL;
		}

		// handle the voice trigger
		event_interrupt(g_chip1);

	}

	return NULL;
}

void event_interrupt(int chip)
{
	int interrupt_events, current_events_status;

	//printf("GOT EVENT INTERRUPT\n");
	if (get_powerup_flag(chip) == FLAG_DISABLE)		// too early event
	return;

	ms_delay(10);
	interrupt_events = read_register(chip, DETECTION_AND_SYSTEM_EVENTS_STATUS_14);
	current_events_status = interrupt_events;

	if ((interrupt_events == 0) && (s_state == 5)){
		current_events_status = 2;
	}
	while (current_events_status != 0){
		if (current_events_status & (1 << 15)){
			check_fw_errors (chip);
			current_events_status = current_events_status & (~ERROR_EVENT);
			break;
		}else if (current_events_status & (1 << 14)){
			check_fw_errors (chip);
			current_events_status = current_events_status & (~WARNING_EVENT);
		}else if (current_events_status & (1 << 0)){
			set_powerup_flag(chip, FLAG_SET);
			printf("FW POWER UP COMPLETED\n");
			current_events_status = current_events_status & (~PWRUP_COMP);
		}else if (current_events_status & (1 << 4)){
			printf("AEP EVENT\n");
			current_events_status = current_events_status & (~AEP_DET);
		}else{
			use_case_wakeup_vt_vc_trigger(chip);
			current_events_status = current_events_status & (~current_events_status);
		}
	}

	// clean events in FW
	write_register(chip, DETECTION_AND_SYSTEM_EVENTS_STATUS_14, interrupt_events);

	delay_if_not_ready(chip, 20);
}

/**********************************************************************/
// This function is activated from interrupt context
#define _OUT_RED_ "\033[0;31m"
#define _OUT_GREEN_ "\033[0;32m"
#define _OUT_DEFAULT_ "\033[0m"
/**********************************************************************/
void launch_record_thread()
{
	HTASK task_id;

	task_id = OSL_TASK_Create((TaskProc)record_thread, NULL,0,NULL,0);

	printf("record thread %d created \n", (int)task_id);
}

/*********************************************************************/

int record_file_count =1;

// Example of record into a file.
void *record_thread(void *ignore)
{
	HDC DataConsumer;				// Consumer to collect the data chunks, can be a file or a socket
	DCParams f_dc;
	int frame_length = DUMP_FRAME_LENGTH;
	int num_of_channels = 1;
	char name_str[100];

#if 1
	sprintf(name_str, "Log/%d_%s",record_file_count, DUMP_FILENAME);
	record_file_count++;
#else	// old (for QA)
	sprintf(name_str, "%s",DUMP_FILENAME);
#endif

	f_dc.fdc.fname = name_str;

	if (s_state == 0) return NULL;

	DataConsumer = DC_Create(DC_TYPE_FILE, &f_dc);

	record_buffer(g_chip1, DataConsumer, g_buffer_time_in_sec, AUDIO_SAMPLE_RATE, frame_length, num_of_channels);

	// Save and close file, and destroy DataConsumer
	DC_Destroy(DataConsumer);

	printf("---file %s was created\n\n", name_str);

	back_to_detection(g_chip1, TRIGGER);

	if ((s_music_on) && (s_state != 5)) {set_play_volume(100);}// if during music - unmute it

	check_fw_errors(g_chip1);	// for debug only

	return NULL;

}

/**********************************************************************/
/*
		#########################################################
		########################## MAIN #########################
		#########################################################

		collect command line parameter
		init bsp, IO ports, chip, enter idle state
		run customer menu loop.
*/

int main(int argc, char **argv)
{
	int value = 0;

	HTASK trigger_polling_pid =0;
	HTASK ready_polling_pid = 0;

	const int shareSize = sizeof(int) * 4;

	prepare_project_configuration();

	// collect argument and set debug g_mode_flag (if exist)
	char buf[16] = {0};

	if(argv[1] != 0) {
		sprintf(buf, "%s", argv[1]);
		if(strcmp(buf, "record") == 0) {
			g_mode_flag = MODE_RECORD;
		} else if(strcmp(buf, "debug") == 0) {
			g_mode_flag = MODE_DEBUG;
		} else if(strcmp(buf, "fast") == 0) {
			g_mode_flag = MODE_OPT;
		}
	}

	signal(SIGINT, intHandler);

	segmentId = shmget(IPC_PRIVATE, shareSize, S_IRUSR | S_IWUSR);
	sared_stop_streaming_flag = (unsigned int*)shmat(segmentId, NULL, 0);
	(*sared_stop_streaming_flag) = 0;

	printf("dspg_host_ver = %s \n", g_dspg_host_ver);		// Customer: Please keep this print in customer terminal

	// check if host has to run slowlier for FW debug messages
	if ((g_mode_flag != MODE_OPT) || (g_transport == TRANSPORT_I2C))
	{
		set_long_delays();

		// For debug reduce spi speed before define the spi device
		// not sure this is required.TBD
		// for D10L: meanwhile do not reduce the SPI rate, fast speed is required for production use case
		if (g_transport == TRANSPORT_UART) {g_uart_speed = UART_SPEED_1MHz;}
	}

	print_project_configuration();

//----------------------------------------------

	if(init_IO_ports(g_chip1) < 0)
		goto err;

//---------------------------------------------

	// Reset CPLD, open, use I2C for CPLD, and close it back.
	// should be called after gpio init of init_IO_ports
	if(bsp_init(g_chip1) < 0)
		goto err;

//---------------------------------------------

	if (EVENTS_TO_HOST){
		if (trigger_polling_pid == 0){	// first time only
			// enable gpio for trigger and polling thread
			gpio_edge(INTERRRUPT_TRIGGER_GPIO, RISING);

			trigger_polling_pid = OSL_TASK_Create((TaskProc)trigger_polling_thread, NULL,0,NULL,0);
			printf("[%s]: trigger_polling_pid = %d \n", __func__, (int)trigger_polling_thread);
		}
	}

	if (READY_ACK){
		if (ready_polling_pid == 0){	// first time only
			// enable gpio for ready and polling thread
			gpio_edge(INTERRRUPT_READY_GPIO, RISING);

			ready_polling_pid = OSL_TASK_Create((TaskProc)ready_polling_thread, NULL,0,NULL,0);
			printf("[%s]: ready_polling_pid = %d \n", __func__, (int)ready_polling_thread);
		}
	}

//----------------------------------------------

init_chip:
	if(init_chip(g_chip1) < 0)
		goto err;

	// Move to Idle state and THEN load trigger model

	init_config_chip(g_chip1);

	//value = fw_validate(g_chip1);

	model_loading(g_chip1, g_wake_word_engine);

	// verify again chip1 is alive by read version
	value = fw_validate(g_chip1);
	if(value != 0) {

		printf ("*********************************\n");
		printf ("*********************************\n");
		printf ("******   D%d IS RUNNING       ****\n", g_chip1);
		printf ("*********************************\n");
		printf ("*********************************\n");
		printf ("Chip version = 0x%04X\n", value & 0xffff);

	} else {
		// return -1;
	}

	check_fw_errors(g_chip1);

	use_case_wakeup_detection_Enter(g_chip1, g_wake_word_engine);
	s_state = 1;
	check_fw_errors(g_chip1);

	menu();
	char line[1024] = {0};

	while(keepRunning) {
		fgets(line, 1024, stdin);

		if(strncmp(line, "q", 1) == 0) {
			use_case_exit(s_state);
			break;
		} else if (strncmp(line, "h", 1) == 0) {
			menu();
		} else if (strncmp(line, "c", 1) == 0) {
			use_case_wakeup_print_commands();
		} else if (strncmp(line, "w", 1) == 0) {
			uint16_t reg;
			uint16_t val;
			const char delimeter[] = " ";
			char *token;

			token = strtok(line, delimeter);
			token = strtok(NULL, delimeter);
			if(token == NULL) {
				printf ("[%s]: Invalid command: %s\n", __func__, line);
				continue;
			}
			reg = atoh__(token);
			printf ("[%s]: Write to register 0x%X\n", __func__, reg);

			token = strtok(NULL, delimeter);
			if(token == NULL) {
				printf ("[%s]: Invalid command: %s\n", __func__, line);
				continue;
			}
			val = atoh__(token);
			printf ("[%s]: value 0x%X\n", __func__, val);
			write_register(g_chip1, reg, val);
		} else if (strncmp(line, "s", 1) == 0) {
			g_buffer_time_in_sec = 10;
			s_state = 1;
			move_to_bufferring(g_chip1);
			launch_record_thread();
		} else if (strncmp(line, "r", 1) == 0) {
			uint16_t reg;
			reg = atoh__((char*)&line[2]);
			value = read_register(g_chip1, reg);
			value = value & 0xffff;
			if(value < 0) {
				printf ("[%s]: FAIL read register 0x%X\n", __func__, reg);
			}
			printf("[%s]: register = 0x%X, val = 0x%X.\n", __func__, reg, value);
		} else {
			menu();
		}
	}
	chip_release(g_chip1);

	shmdt(sared_stop_streaming_flag);
	shmctl(segmentId, IPC_RMID, NULL);

	return 0;

err:
	shmdt(sared_stop_streaming_flag);
	shmctl(segmentId, IPC_RMID, NULL);
	chip_release(g_chip1);
	exit(5);
}


//*****************************************************************************/

void testcolor(int attr, int fg, int bg)
{
	char command[13];
	sprintf(command, "%c[%d;%d;%dm", 0x1B, attr, fg+30, bg+40);
	printf("%s", command);
}

//*****************************************************************************/

void menu()
{
#if 0
	testcolor(BRIGHT, RED, BLACK);
	printf("\n-------------- User Menu ------------\n");

	printf("r [reg]         Read register.\n");
	printf("w [reg] [val]   Write register.\n");
	printf("uc0             Exit to Idle use case.\n");
	printf("uc1             SED Sensing \n");
	printf("ucx             Run a use case from script %s.\n\n", USE_CASE_REG_TABLE_ENTER_FILENAME);

	printf("uc_mode         modify use case parameters.\n");
	printf("state           display current state\n\n");

	printf("ver             display dspg versions.\n");
	printf("conf            display project configuration\n");
	printf("err             check fw errors.\n");
	printf("init            re-init target.\n");
	printf("regs            ad hoc reg-table script %s.\n\n", MENU_REG_TABLE);
	printf("567  [val] [val] [val]   set fw-record-regs for next usecase.\n");
	printf("asrp [val]      asrp records: 0=disabled 1=enabled (=default in mode 2).\n\n");

	printf("mode [val]      set fw mode: 1 = no-log; 2 = fw-record; 3 = fw-debug.\n");

	printf("h               display menu.\n");
	printf("q               exit the program.\n\n");
	printf("Enter command:\n");
	testcolor(RESET, WHITE, BLACK);
#else
	printf(_OUT_RED_);
	printf("\n-------------- User Menu ------------\n");

	printf("r [reg]         Read register.\n");
	printf("w [reg] [val]   Write register.\n");
	printf("c               Supported voice commands\n");
	printf("s               Start recording audio (10 sec)\n");
	printf("h               display menu.\n");
	printf("q               exit the program.\n\n");
	printf("Enter command:\n");
	printf(_OUT_DEFAULT_);

#endif
}

//*****************************************************************************/

// typical line in a run time configuration file:  "#define MIC1		2	"
int parse_config_file(char* filename)
{
	FILE *file;
	char* str;
	char buffer[128];
	char define_str[32];
	char param_str[32];
	char val_str[100];
	int val_decimal, val_hex;

	int rc;

	/* Open the file */
	file = fopen(filename, "rb");

	if(!file)
	{
		printf("Config File %s does not exist - Continue\n", filename);
		return -1 ;
	}

    while (fgets(buffer, 128, file) != NULL )
	{
		//printf("line %s", buffer);
   		if(buffer[0] == ';')
   		{
			memset(buffer,0,128);
			continue ;
   		}

		memset(define_str,0,sizeof(define_str));
		memset(param_str,0,sizeof(param_str));
		memset(val_str,0,sizeof(val_str));

		rc = sscanf(buffer, "%s	%s	%s", define_str, param_str, val_str);
		if(rc == 3)
		{
			//printf("define_str %s param_str %s val_str %s \n", define_str, param_str, val_str);

			val_hex = strtol(val_str,&str,16);
			val_decimal = strtol(val_str,&str,10);

			if(strcmp(param_str,"MIC1") == 0) {g_mic1_type = val_decimal;}
			else
			if(strcmp(param_str,"MIC2") == 0) {g_mic2_type = val_decimal;}
			else
			if(strcmp(param_str,"WAKE_UP") == 0) {g_use_wakeup = val_decimal;}
			else
			if(strcmp(param_str,"DUMP_SECONDS") == 0) {g_buffer_time_in_sec = val_decimal;}
			else
			if(strcmp(param_str,"ENGINE") == 0) {g_wake_word_engine = val_decimal;}
			else
			if(strcmp(param_str,"FW_FILENAME") == 0) {g_FirmWare_filename = val_str;}
			else
			if(strcmp(param_str,"FIRMWARE_GOOGLE") == 0) {g_FirmWare_filename = val_str;}
			else
			if(strcmp(param_str,"I2S_CHIP_MASTER") == 0) {g_i2s_chip_master = val_decimal;}
			else
			if(strcmp(param_str,"TDM_SAMPLE_RATE") == 0) {g_tdm_sample_rate = val_decimal;}
			else
			if(strcmp(param_str,"TDM_BIT_RATE") == 0) {g_tdm_bit_rate = val_decimal;}
			else
			if(strcmp(param_str,"NO_OF_SPEAKERS") == 0) {g_no_of_speakers = val_hex;}
			else
			if(strcmp(param_str,"SUPPORT_RX_PROCESS") == 0) {g_support_rx_process = val_decimal;}
			else
			if(strcmp(param_str,"GOGGLE_ASSISTANT_MODE") == 0) {g_google_assist_mode = val_decimal;}
			else
			if(strcmp(param_str,"USE_SPI_IN_GA_MODE") == 0) {g_use_spi_in_ga_mode = val_decimal;}

			printf("param: %s = %d (decimal)\n", param_str, val_decimal);

		}else{
			printf("failed to parse line %s \n", buffer);
		}

		memset(buffer,0,128);
       }

	fclose(file);

	return 0;
}
