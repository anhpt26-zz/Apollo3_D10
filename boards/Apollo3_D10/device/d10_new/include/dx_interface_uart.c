/*
 * dx_interface_uart.c
 *
 * Copyright (C) 2016 DSP Group
 * All rights reserved.
 *
 * Unauthorized copying of this file is strictly prohibited.
 */

#include "dbmx_uart.h"

extern int g_uart_speed;
int dbmdx_uart_sync(int chip);

// these defines should be moved to dbmx_uart.h
#define UART_D2_BOOT_PARITY		1
#define UART_D2_NORMAL_PARITY	0
#define UART_BOOT_STOP_BITS		2
#define UART_NORMAL_STOP_BITS	1

/***********************************************************************
* FUNCTION NAME: init_IO_to_chip()
*
* DESCRIPTION:
*	init all io ports towards chip (gpios i2c channel and SPI)
* PARAMETERS:
*
* RETURNS:
*	0 on sucsses
*	error otherwise
*
***********************************************************************/   

ioInterface init_IO_to_chip()
{
	UART* uart_io;
	UartConfig cfg;
	
	uart_io = uart_open(UART_DEVICE_PATH, 1);

	cfg.speed = UART_SPEED_1MHz;
	cfg.parity = UART_D2_NORMAL_PARITY; 
	cfg.stop_bit = UART_NORMAL_STOP_BITS;
	cfg.flow_ctl = 0;
	
	uart_config(uart_io,  &cfg);

	return ((void *)uart_io);
}

int config_speed(int chip, int new_speed)
{
	UartConfig cfg;
	int ret;
	
	cfg.speed = UART_SPEED_1MHz;
	cfg.parity = UART_D2_NORMAL_PARITY; 
	cfg.stop_bit = UART_NORMAL_STOP_BITS;
	cfg.flow_ctl = 0;
	
	ret = uart_config((UART *)chip_io[chip],  &cfg);

	g_uart_speed = new_speed;
	
	return (ret);
}

/***********************************************************************
*  FUNCTION NAME: io_write_delay()
*
* DESCRIPTION:
*	Write len bytes to SPI file descriptor
*
* PARAMETERS:
* 	int fd, unsigned char *buf, int len
*
* RETURNS:
*
***********************************************************************/
int io_write_delay(int chip, unsigned char *buf, int len)		//uart_implementation

{
	int rc = 0;

	rw_lock_aquire();

	wakeup(chip);

	rc  = uart_write((UART *)chip_io[chip], (uint8_t *) buf, len);

	rw_lock_release();

	ms_delay(delay_io_write);
	
	return rc;
}

/***********************************************************************
*  FUNCTION NAME: write_io_port()
*
* DESCRIPTION:
*	Use Writing of 32 bits commands ("W" instead of "w")
*	Indirect write: 
*		write address to registers 5 (and implicitly also to 6)
*		write value    to register 7 (and implicitly also to 8)
*
* PARAMETERS:
* 	int chip, uint32_t addr, int32_t val
*
* RETURNS:
*
***********************************************************************/
void write_io_port(int chip, int32_t addr, int32_t val)		//uart_implementation

{
	char str[20];
	int len;
	
	/* Write Format:<RRR>W<VVVVVVVV> */
	len = sprintf(str, "%03xW%08x", REG_IO_PORT_ADDR_LO_05, addr);
	str[len]=0;


	printf("[%s]: %s\n",__func__, str);

	rw_lock_aquire();

	wakeup(chip);

	uart_write((UART *)chip_io[chip], (uint8_t *) str, strlen(str));

	len = sprintf(str, "%03xW%08x", REG_IO_PORT_VALUE_LO_07, val);
	str[len]=0;

	ms_delay(1);

	uart_write((UART *)chip_io[chip], (uint8_t *) str, strlen(str));

	rw_lock_release();

	printf("[%s]: %s\n",__func__, str);

	ms_delay(delay_io_write);


}

/***********************************************************************
*  FUNCTION NAME: write_register()
*
* DESCRIPTION:
*	Send via IO ascii buffer of 7 bytes, format is:
* 	2 bytes - register address in ascii format
* 	1 byte contains the character 'w' for write command
* 	4 bytes contains the value to be written to register in ascii format.
*
* PARAMETERS:
* 	int chip, uint8_t reg, int16_t val
*
* RETURNS:
*
***********************************************************************/
void write_register(int chip, int16_t reg, int16_t val)			//uart_implementation

{
	int ret, len;
	char str[16];
	len = sprintf(str, "%03xw%04x", reg, (val)&0xffff);

	str[len]=0;
	
	if (!skip_prints) printf("[%s]: %s\n",__func__, str);

	ret = io_write_delay(chip, (unsigned char*)str, len);
	if(ret != strlen(str)) {
		printf("[%s]: io_write error %d.\n", __func__, ret);
		pabort("[write_register]: io_write error.");
	}
	///tcdrain(chip_io[chip]);			// to clean
}

/***********************************************************************/

void write_register_32(int chip, int16_t reg, int32_t val)			//uart_implementation

{
	int ret, len;
	char str[24];

	len = sprintf(str, "%03xW%08x", reg, (val)&0xffffffff);
	str[len]=0;
	
	printf("[%s]: %s\n",__func__, str);

	ret = io_write_delay(chip, (unsigned char*)str, len);
	if(ret != strlen(str)) {
		printf("[%s]: io_write error %d.\n", __func__, ret);
		pabort("[write_register]: io_write error.");
	}
	///tcdrain(chip_io[chip]);			// to clean
}


/***********************************************************************
*  FUNCTION NAME: read_register()
*
* DESCRIPTION:
*	Send via IO ascii buffer of 3 bytes, format is:
* 	2 bytes - register address in ascii format.
* 	1 byte contains the character 'r' for read command
*
* 	Receive ascii buffer
* 	Read via IO 5 (SPI) or 4 (I2C) bytes
* 	byte[0] - not relevant
* 	bytes[1-4] - ascii string
*
* PARAMETERS:
* 	int fd_spi int16_t reg
*
* RETURNS:
*
***********************************************************************/
int16_t read_register(int chip, int16_t reg)		//uart_implementation

{
	uint16_t val = 0;
	int ret, len;
	unsigned char Buff[10] = {0};
	char RegSet[18] = {0};

/*????????????????????????????????????//
	while(1) {	//clean UART Rx from FW logs
		ret = uart_read((UART *)chip_io[chip], (void *)buff_tmp, 16);
		if(ret == 0) break;
	};
//????????????????????????????????????*/
	rw_lock_aquire();

	wakeup(chip);

	len = sprintf(RegSet, "%03xr", reg);
	RegSet[len]=0;
	

	ret = uart_write((UART *)chip_io[chip], (uint8_t *) RegSet, len);
	
	if(ret != strlen(RegSet)) {
		rw_lock_release();
		printf("[%s]: io_write error %d.\n", __func__, ret);
		pabort("[read_register]: io_write error.");
	}

	// usleep(10);    
	ms_delay(5);		// may be changed to 0.5 ms

	// why do we read 5 and not 4???
 	ret = uart_read((UART *)chip_io[chip], (uint8_t *)Buff, 5);

	rw_lock_release();

	if(ret != 5) {
		printf("[%s]: io_read error.\n", __func__);
		pabort("[read_register]: io_read error.");
	}

	Buff[4] = 0;

	val = atoh__((char*)&Buff[0]);
	if(val < 0) {
		printf("[%s]: io_read error.\n", __func__);
	}

	if (!skip_prints) printf("[%s]: register = 0x%X, val = 0x%X.\n", __func__, reg, val);
	return val;
}

/***********************************************************************/

int32_t read_register_32(int chip, int16_t reg)		//uart_implementation

{
	uint32_t val = 0;
	int ret, len;
	unsigned char Buff[15] = {0};
	char RegSet[18] = {0};

	len = sprintf(RegSet, "%03xR", reg);
	RegSet[len]=0;
	
	rw_lock_aquire();

	wakeup(chip);

	ret = uart_write((UART *)chip_io[chip], (uint8_t *) RegSet, len);

	ms_delay(5);		// 	usleep(10);    

	if(ret != strlen(RegSet)) {
		rw_lock_release();
		printf("[%s]: io_write error %d.\n", __func__, ret);
		pabort("[read_register]: io_write error.");
	}
	
 	ret = uart_read((UART *)chip_io[chip], (uint8_t *)Buff, 8);

	rw_lock_release();

	if(ret != 8) {
		printf("[%s]: io_read error.\n", __func__);
		pabort("[read_register]: io_read error.");
	}

	Buff[8] = 0;
	/* First byte read by SPI must be ignored. */
	val = atoh__((char*)&Buff[0]);
	if(val < 0) {
		printf("[%s]: io_read error.\n", __func__);
	}

	printf("[%s]: register = 0x%X, val = 0x%X.\n", __func__, reg, val);
	return val;
}

/***********************************************************************
* FUNCTION NAME: read_checksum()
*
* DESCRIPTION:
*	Send check sum command to DBMDX using io and comprae the results
*	to the expeceted checksum.
*
* PARAMETERS:
*	char *checksum - array of 4 char the expected checksum
* RETURNS:
*	0 - checksum is OK
*	else error
***********************************************************************/
int read_checksum(int chip, char *checksum)			//uart_implementation

{
	char buf[] = {BOOT_PRE_OP_CODE, BOOT_READ_CHECKSUM};
	char c[7];
	int rc ;

	uart_write((UART *)chip_io[chip], (uint8_t *)buf, 2);

	ms_delay(10);

 	rc = uart_read((UART *)chip_io[chip], (uint8_t *)c, 6);

	ms_delay(1);
	printf("[%s]: checksum: read return %d\n", __func__, rc);
		printf ("%02x %02x %02x %02x %02x %02x\n",c[0],c[1],c[2],c[3],c[4],c[5]);
	printf ("read checksum got: = %02x %02x %02x %02x expected : %02x %02x %02x %02x\n",
			c[2],c[3],c[4],c[5],checksum[0],checksum[1],checksum[2],checksum[3]);
        
	rc = strncmp(&c[2],checksum, 4);
	if (rc)
		printf ("checksum error : got: = %2x %2x %2x %2x expected : %2x %2x %2x %2x\n",
		c[2],c[3],c[4],c[5],checksum[0],checksum[1],checksum[2],checksum[3]);
	else
		printf ("checksum pass\n");

    return rc ;
}

/***********************************************************************/

// read a chunk from transport, header (optional) + buf
int read_chunk (int chip, uint8_t * header, int header_len, char * buf, int bytes_to_read)
{
	printf("header_len = %d bytes_to_read = %d\n", header_len, bytes_to_read);

	rw_lock_aquire();

 	if (header_len){ 
		uart_read((UART *)chip_io[chip], (uint8_t *)header, header_len);
	}

 	uart_read((UART *)chip_io[chip], (uint8_t *)buf, bytes_to_read);

	rw_lock_release();

	ms_delay(8);		// not sure this wait is required.

	return bytes_to_read;

}

/***********************************************************************
* FUNCTION NAME: load_file()
*
* DESCRIPTION:
*	load file into buffer and transmit it to DBMDX over the SPI.
*   the load succssed only if the checksum is correct.
*   it also sends clear crc command before sending the file so
* 	the checksum can be checked correctly.
*   usally the firmware files containt the correct checksum
* 	at the end of the firmware file.
*	4 last charcters of the firmware, the function reads the checksum
* 	value from the chip and compare it ).
* PARAMETERS:
*	char *filename
* RETURNS:
*	0 on sucsses (passed checksum).
*	error otherwise
***********************************************************************/
int load_file(int chip, char *filename, int skip_bytes)		//uart_implementation

{
	int fileLen;
	char *buffer;
	char *p_buffer;
	char *checksum;
	int rc;
	long err = 1;
	const char clr_crc_chip[]= {BOOT_PRE_OP_CODE, BOOT_RESET_CHECKSUM};
	ImgFile * fw_image;
	int n_written = 0;
	int n_total = 0;

	fw_image = IMG_fopen(filename);		// Open source file

	fileLen = fw_image->size;

	printf("fileLen = %d skip_bytes = %d\n",fileLen,skip_bytes );

	/* Allocate memory */
	buffer = (char *)malloc(fileLen+1);
	if(!buffer) {
		fprintf(stderr, "Memory error!");
		err = -1;
		goto exit_1;
	}
	p_buffer = buffer;
	IMG_fread(fw_image, (uint8_t*)buffer, fileLen, 0);

	if(skip_bytes >= CHECKSUM_SIZE) {		// checksum to be verified
		checksum = &buffer[fileLen - skip_bytes];
		if(1) {

		// init chip checksum
		rc  = uart_write((UART *)chip_io[chip], (uint8_t *)clr_crc_chip, sizeof(clr_crc_chip));
			printf("[%s]: clr_crc_chip write ret = %d.\n", __func__, rc);
		} else {
			perror("[load_file]: wrong file name.");
			err = -1;
			goto exit_1;
		}
		if(rc <= 0) {
			perror("[load_file]: fail to write clr_crc command");
			err = -1;
			goto exit_1;
		}
	}

	do {
		n_written = rc = uart_write((UART *)chip_io[chip], (uint8_t *)p_buffer, (fileLen - skip_bytes) - n_total);
		if(n_written > 0) {
			p_buffer += n_written;
			n_total += n_written;
		} else {
			continue;
		}
	} while (n_total < (fileLen - skip_bytes));


	printf("[%s]: write firmware n_total = %d.\n", __func__, n_total);
	ms_delay(10);			// To be tuned

	if(skip_bytes >= CHECKSUM_SIZE){			// checksum to be verified
		rc = read_checksum(chip, checksum);
		if(rc < 0) {
			printf("[%s]: fail read checksum.\n", __func__);
			err = -1;
			goto exit_1;
		}
	}
	
exit_1:
	IMG_fclose(fw_image);
	free(buffer);
	return err;
}

/***********************************************************************/

// load model file (built as: (header + data) pairs + tail of 2 bytes)
int load_amodel_file(int chip, char *filename, int skip_bytes)		//spi_implementation
{
	unsigned long fileLen;
	unsigned char *buffer;
	unsigned char header_buffer[10];
	uint32_t num_of_words;
	int i, length;

	int n_total = 0;
	int n_written;
	int source_read_bytes=0;

	ImgFile * fw_image;

	// Open the file
	fw_image = IMG_fopen(filename);		// Open source file

	// Get file length
	fileLen = fw_image->size;

	printf("[load_amodel_file]: %s, fileLen:%d \n", filename, (unsigned int)fileLen);

	// read header+data pairs as much as exist in source file (before its tail)  

	while(source_read_bytes < (fileLen - BINARY_FILE_TAIL_LEN)) {		

		// printf("source_read_bytes:  %d, fileLen:  %d \n",source_read_bytes,(unsigned int)fileLen );
		// read header of file
		IMG_fread(fw_image, (uint8_t *)header_buffer, sizeof(header_buffer), IMG_CUR_POS);
		source_read_bytes += sizeof(header_buffer);
		if(header_buffer[0] != BOOT_PRE_OP_CODE && (header_buffer[1] != 0x01 || header_buffer[1] != 0x02)) {
			printf("[load_amodel_file]: header error.\n");
			err = -1;
			goto error_1;
		}

		wakeup(chip);

		// Send header to FW
		if(io_write_delay(chip, &header_buffer[0], sizeof(header_buffer)) < 0){
			goto error_1;
			err = -1;
		}			
			
		ms_delay(100);	// to be tuned

		// Read data from file 

		num_of_words = ((uint32_t)header_buffer[5] << 24)
				| ((uint32_t)header_buffer[4] << 16)
				| ((uint32_t)header_buffer[3] << 8)
				| ((uint32_t)header_buffer[2] << 0);

		DEBUG_PRINT("[load_amodel_file]: num_of_words:%d \n", num_of_words);

		length = num_of_words * 2;

		/* Allocate memory */
		buffer = (unsigned char*)malloc(length);
		if (!buffer) {
			fprintf(stderr, "Memory error!");
			err = -1;
			goto error_1;
		}

		// read into buffer 
		IMG_fread(fw_image, (uint8_t *)buffer, length, IMG_CUR_POS);
		source_read_bytes += length;

		// Send to FW

		n_total = 0;
		do {
			n_written = io_write_delay(chip, &buffer[n_total], num_of_words * 2 - n_total);
			if(n_written > 0) {
				n_total += n_written;
			} else {
				continue;
			}
		} while (n_total < num_of_words * 2);

		ms_delay(100);	// to be tuned
		free(buffer);
	} /* while another (header+data) pair exists in source */

	IMG_fclose(fw_image);
	return 1;
	
error_2:
	free(buffer);
error_1:
	IMG_fclose(fw_image);
	return err;
}

void IO_Release(int chip)
{
	uart_close((UART*)chip_io[chip]);
}


////int get_uart_fd(UART *u);

int dbmdx_interface_sync(int chip)
{
	int rc;
	char buf_sync[16] = {0};
	int i;
	char c[4] = {0};
	int size = 4;

	////int size = 16;

	unsigned char buff_tmp[64] = {0};

	///int uart_fd;

	///uart_fd = get_uart_fd((UART *)chip_io[chip]);
	
	//printf("[%s]: start boot sync  uart_fd = %d\n", __func__, uart_fd);

	///tcdrain(uart_fd);

	while(1) {
		rc = uart_read((UART *)chip_io[chip], (uint8_t *) buff_tmp, 1);
		if(rc == 0) break;

	};

	for (i = 0; i < size; i++) {
		rc = uart_write((UART *)chip_io[chip], (uint8_t *) buf_sync, 16);

		if (rc != 16)
			printf("[%s]: sync buffer not sent correctly rc = %d\n", __func__, rc);
		///tcdrain(uart_fd);
		rc = uart_read((UART *)chip_io[chip], (uint8_t *) c, 4);
		printf("[%s]: sync read buffer rc = %d, \n", __func__, rc);

		printf("[%s]: read sync: = %2x %2x %2x %2x (%s)\n",__func__, c[0],c[1],c[2],c[3], c);

		if(!strcmp("OK\n\r", c)) {
			printf("[%s]: UART Got Sync.\n", __func__);
			break;
		} else {
			printf("[%s]: Failed Sync UART.\n", __func__);
		}
	}

	if(i == size) {
		printf("[%s]: Failed Sync UART.\n", __func__);
		exit(0);
	}

	///tcdrain(uart_fd);
	printf("[%s]: boot sync successfully\n", __func__);

	return rc;
}

