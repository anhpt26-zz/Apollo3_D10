/*
 * dx_interface_i2c.c
 *
 * Copyright (C) 2016 DSP Group
 * All rights reserved.
 *
 * Unauthorized copying of this file is strictly prohibited.
 */

#include "dbmx_i2c.h"

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
	I2C* i2c_io;
	I2cConfig cfg;
	i2c_io = i2c_open(I2C_DEVICE_PATH, 1);

	cfg.address = I2C_DEVICE_ADDRESS;
	cfg.speed = I2C_SPEED_100KHz;
	i2c_config(i2c_io, &cfg);

	return ((void *)i2c_io);
}

int config_speed(int chip, int new_speed)
{
	// no option to change I2C speed in RPI
	return (-1);
}

void dbmdx_interface_sync(int chip)
{
	// empty today
	return;
}

/***********************************************************************
*  FUNCTION NAME: io_write_delay()
*
* DESCRIPTION:
*	Write len bytes to I2C file descriptor
*
* PARAMETERS:
* 	int fd, unsigned char *buf, int len
*
* RETURNS:
*
***********************************************************************/

int io_write_delay(int chip, unsigned char *buf, int len)		//i2c_implementation
{
	int rc = 0;

	rw_lock_aquire();
	
	rc  = i2c_write((I2C*)chip_io[chip], (uint8_t *) buf, len);

	ms_delay(delay_io_write);

	rw_lock_release();

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
// not implemented yet
void write_io_port(int chip, int32_t addr, int32_t val)		//i2c_implementation

{
/*
	char str[20];
	int len;
	
	// Write Format:<RRR>W<VVVVVVVV>
	len = sprintf(str, "%03xW%08x", REG_IO_PORT_ADDR_LO_05, addr);
	str[len]=0;


	printf("[%s]: %s\n",__func__, str);

	rw_lock_aquire();

	i2c_write((I2C*)chip_io[chip], (uint8_t *) str, strlen(str));

	len = sprintf(str, "%03xW%08x", REG_IO_PORT_VALUE_LO_07, val);
	str[len]=0;

	ms_delay(1);

	i2c_write((I2C*))chip_io[chip], (uint8_t *) str, strlen(str));

	rw_lock_release();

	printf("[%s]: %s\n",__func__, str);

	ms_delay(delay_io_write);
*/
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

void write_register(int chip, int16_t reg, int16_t val)			//i2c_implementation
{
	int ret;
	char str[16];
	uint8_t data[4];
	data[0] = (reg >>  8) & 0xff;;
	data[1] = (reg) & 0xff;
	data[2] = (val >> 8) & 0xff;
	data[3] = val & 0xff;

	sprintf(str, "%04xw%04x", reg, (val)&0xffff);

	ret = i2c_write((I2C *)chip_io[chip], data, 4);

	if(ret != 4) {
		printf("[%s]: i2c write error.\n", __func__);
		pabort("[write_register_i2c]: i2c write error.");
	}

	//printf("[%s]: %s\n",__func__, str);

	ms_delay(delay_io_write); 	/* to be optimized - was 50*/
}
/***********************************************************************/
// Not implemented yet
/*
void write_register_32(int chip, int16_t reg, int32_t val)			//spi_implementation

{
	int ret;
	char str[24];
	sprintf(str, "%03xW%08x", reg, (val)&0xffffffff);	//???

	printf("[%s]: %s\n",__func__, str);

	ret = io_write_delay(chip, (unsigned char*)str, strlen(str));
	if(ret != strlen(str)) {
		printf("[%s]: io_write error %d.\n", __func__, ret);
		pabort("[write_register]: io_write error.");
	}
	///tcdrain(chip_io[chip]);			// to clean
}
*/

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
* 	int fd_i2c int16_t reg
*
* RETURNS:
*
***********************************************************************/
int16_t read_register(int chip, int16_t reg)		//i2c_implementation
{
	int16_t val;
	int ret;
	uint8_t data[2] = {0};
	uint8_t recv[2] = {0};
	data[1] = reg & 0xFF;
	data[0] = (reg >> 8) & 0xFF;

	rw_lock_aquire();

	ret = i2c_write((I2C*)chip_io[chip], data, 2);

	ms_delay(5);
	if (ret !=2){
		rw_lock_release();
		printf("[%s]: i2c write error.\n", __func__);
		pabort("[read_register_i2c]: i2c write error.");
	}

 	ret = i2c_read((I2C*)chip_io[chip], recv, 2);

	rw_lock_release();
	
	if (ret !=2){
		printf("[%s]: i2c read error.\n", __func__);
		pabort("[read_register_i2c]: i2c read error.");
	}

	val = (recv[0] << 8) | recv[1];
	//printf ("[%s]: Register 0x%x = 0x%x\n", __func__, reg, val & 0xffff);
	ms_delay(10);  		/* to be optimized*/
	return val;
}

/***********************************************************************/
/*
int32_t read_register_32(int chip, int16_t reg)		//i2c_implementation

{
	uint32_t val = 0;
	int ret;
	unsigned char Buff[15] = {0};
	char RegSet[18] = {0};

	sprintf(RegSet, "%03xR", reg);

	rw_lock_aquire();

	wakeup(chip);

	ret = spi_write((SPI *)chip_io[chip], (uint8_t *) RegSet, strlen(RegSet));

	ms_delay(5);		// 	usleep(10);    

	if(ret != strlen(RegSet)) {
		printf("[%s]: io_write error %d.\n", __func__, ret);
		pabort("[read_register]: io_write error.");
	}
	
 	ret = spi_read((SPI *)chip_io[chip], (uint8_t *)Buff, 9);

	rw_lock_release();

	if(ret != 9) {
		printf("[%s]: io_read error.\n", __func__);
		pabort("[read_register]: io_read error.");
	}

	Buff[9] = 0;
	// First byte read by SPI must be ignored.
	val = atoh__((char*)&Buff[1]);
	if(val < 0) {
		printf("[%s]: io_read error.\n", __func__);
	}

	printf("[%s]: register = 0x%X, val = 0x%X.\n", __func__, reg, val);
	return val;
}
*/
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

int read_checksum(int chip, char *checksum)			//i2c_implementation
{
	char buf[] = {BOOT_PRE_OP_CODE, BOOT_READ_CHECKSUM};
	char c[8];
	int rc ;

	i2c_write((I2C*)chip_io[chip], (uint8_t *)buf, 2);

	//tcdrain(fd);
	ms_delay(10);

 	i2c_read((I2C*)chip_io[chip], (uint8_t *)c, 6);

	ms_delay(1);

	printf ("read checksum got: = %2x %2x %2x %2x %2x %2x\n", c[0],c[1],c[2],c[3],c[4],c[5]);
	rc = strncmp (&c[2], checksum, 4);
	if (rc)
		printf ("2. checksum error : got: = %2x %2x %2x %2x expected : %2x %2x %2x %2x\n",
			c[4],c[5],c[6],c[7],checksum[0],checksum[1],checksum[2],checksum[3]);
	else
		printf ("checksum pass\n");

    return rc ;
}

/***********************************************************************/

// read a chunk from transport, header (optional) + buf
int read_chunk (int chip, uint8_t * header, int header_len, char * buf, int bytes_to_read)
{

	rw_lock_aquire();
	
 	if (header_len){ 
		i2c_read((I2C *)chip_io[chip], (uint8_t *)header, header_len);

		//tcdrain(chip);
	}

 	i2c_read((I2C *)chip_io[chip], (uint8_t *)buf, bytes_to_read);

	rw_lock_release();
	
	// tcdrain(chip);
	ms_delay(8);		// not sure this wait is required.

	return bytes_to_read;

}

/***********************************************************************
* FUNCTION NAME: load_file()
*
* DESCRIPTION:
*	load file into buffer and transmit it to DBMDX over the I2C.
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
int load_file(int chip, char *filename, int skip_bytes)		//i2c_implementation
{
	unsigned long fileLen;
	char checksum[CHECKSUM_SIZE];
	ImgFile * fw_image;

	char *buffer;
	char *p_buffer;
	int rc;
	long err = 1;
	int n_written = 0;
	int n_total = 0;
	const char clr_crc_chip[]= {BOOT_PRE_OP_CODE, BOOT_RESET_CHECKSUM};

	/* Open the file */
	fw_image = IMG_fopen(filename);		// Open source file

	fileLen = fw_image->size;

	// Upload File to FW

	if(skip_bytes >= CHECKSUM_SIZE) {		// checksum to be verified

		IMG_fread(fw_image, (uint8_t *)checksum, CHECKSUM_SIZE, fileLen - skip_bytes);

		// init chip checksum
		rc  = i2c_write((I2C *)chip_io[chip], (uint8_t *)clr_crc_chip, sizeof(clr_crc_chip));

		if(rc < 0) {
			perror("[load_file]: fail to write clr_crc command");
			err = -1;
			goto error_2;
		}
	}


	/* Allocate memory */
	buffer = (char *)malloc(fileLen+1);
	if(!buffer) {
		fprintf(stderr, "Memory error!");
		err = -1;
		goto error_1;
	}


	p_buffer = buffer;

	IMG_fread(fw_image, (uint8_t *)buffer, fileLen - skip_bytes, 0 );

	printf("[%s]: load_file:%s, filelen:%d bytes.\n", __func__, filename,(int)fileLen);

	// Send file to FW

	do {
		n_written = rc = i2c_write((I2C*)chip_io[chip], (uint8_t *)p_buffer, (fileLen - skip_bytes) - n_total);

		if(n_written > 0) {
			p_buffer += n_written;
			n_total += n_written;
		} else {
			continue;
		}
	} while (n_total < (fileLen - skip_bytes));

	printf("[%s]: write firmware n_total = %d.\n", __func__, n_total);

	ms_delay(100);

	if(skip_bytes >= CHECKSUM_SIZE){

		rc = read_checksum(chip, checksum);
		if(rc < 0) {
			printf("[%s]: fail read checksum.", __func__);
			err = -1;
			goto error_2;
		}
	}

	IMG_fclose(fw_image);
	free(buffer);
	return err;

error_2:
	free(buffer);
error_1:
	IMG_fclose(fw_image);
	return err;
}

/*******************************************************************************/

int load_amodel_file(int chip, char *filename, int skip_bytes)		//i2c_implementation
{
	unsigned long fileLen;
	unsigned char *buffer;
	unsigned char header_buffer[10];
	uint32_t num_of_words;
	int i, rc, length;
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

		// Send header to FW
		rc = i2c_write((I2C*)chip_io[chip], (uint8_t *)&header_buffer[0], sizeof(header_buffer));

		if(rc != sizeof(header_buffer)) {
			printf("[%s]: uart write %d bytes, sizeof(header_buffer) = %d\n", __func__, rc, sizeof(header_buffer));
			goto error_1;
			err = -1;
		}

		ms_delay(100);

		num_of_words = ((uint32_t)header_buffer[5] << 24)
				| ((uint32_t)header_buffer[4] << 16)
				| ((uint32_t)header_buffer[3] << 8)
				| ((uint32_t)header_buffer[2] << 0);

		DEBUG_PRINT("[load_amodel_file]: num_of_words:%d \n", num_of_words);
		length = num_of_words * 2;

		DEBUG_PRINT("[load_amodel_file]: length:%d \n", length);

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
		n_total = 0;
		do {
			n_written = i2c_write((I2C*)chip_io[chip], (uint8_t *)&buffer[n_total], length - n_total);

			if(n_written > 0) {
				n_total += n_written;
			} else {
				continue;
			}
		} while (n_total < length);

		ms_delay(100);
		free(buffer);
	} /* while another (header+data) pair exists in source */

	IMG_fclose(fw_image);

	return 1;

error_1:
	IMG_fclose(fw_image);
	return err;
}

void IO_Release(int chip)
{
	i2c_close((I2C*)chip_io[chip]);
	chip_io[chip] = 0;
}

