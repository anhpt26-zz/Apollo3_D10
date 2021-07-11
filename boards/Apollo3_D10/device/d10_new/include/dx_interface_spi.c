/*
 * dx_interface_spi.c
 *
 *	Some communication functions with chip - SPI implementation
 * 
 *	Copyright (C) 2016 DSP Group
 * All rights reserved.
 *
 * Unauthorized copying of this file is strictly prohibited.
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "dx_gen.h"
#include "dx_params.h"
#include "dx_fw_defs.h"
#include "dx_utils.h"
#include "debug.h"
#include "system.h"
#include "ext_dsp_manager.h"

extern int g_spi_speed;

#define SPI   void
extern int chip_io[15];
extern int delay_io_write;
extern int skip_prints;

#define rw_lock_aquire()  
#define rw_lock_release()
#define pabort(x)
int io_write_delay(int chip, unsigned char *buf, int len);
static int spi_write(SPI *spi, const uint8_t *buf, int len) {
  return ExtDsp_SpiTransfer((uint8_t *)buf, len, true);
}
static int spi_read(SPI *spi, uint8_t *buf, int len) {
  return ExtDsp_SpiTransfer(buf, len, false);
}

char sync_spi_type[] = {0x0};

void dbmdx_interface_sync(int chip)
{
	if (chip == DBM_D10){
		io_write_delay(chip,(unsigned char *)sync_spi_type, sizeof(sync_spi_type));
	}
}

/***********************************************************************
*  FUNCTION NAME: io_write_delay()
*
* DESCRIPTION:
*	Write len bytes to SPI, with some delay
*
* PARAMETERS:
* 	int fd, unsigned char *buf, int len
*
* RETURNS:
*
***********************************************************************/
int io_write_delay(int chip, unsigned char *buf, int len)		//spi_implementation

{
	int rc = 0;

	rw_lock_aquire();

	wakeup(chip);

	set_ready_flag(chip, FLAG_CLEAR);
	
	rc  = spi_write((SPI *)chip_io[chip], (uint8_t *) buf, len);

	wait_ready_or_delay(chip, delay_io_write);

	rw_lock_release();	// release only after delay, when FW is ready!
	
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
void write_io_port(int chip, int32_t addr, int32_t val)		//spi_implementation

{
	char str[20];
	int len;
	
	/* Write Format:<RRR>W<VVVVVVVV> */
	len = sprintf(str, "%03xW%08x", REG_IO_PORT_ADDR_LO_05, addr);
	str[len]=0;

	Debug_Printf("[%s]: %s\n",__func__, str);

	rw_lock_aquire();

	wakeup(chip);

	set_ready_flag(chip, FLAG_CLEAR);

	spi_write((SPI *)chip_io[chip], (uint8_t *) str, len);

	len = sprintf(str, "%03xW%08x", REG_IO_PORT_VALUE_LO_07, val);
	str[len]=0;

	wait_ready_or_delay(chip, delay_io_write );

	set_ready_flag(chip, FLAG_CLEAR);

	spi_write((SPI *)chip_io[chip], (uint8_t *) str, len);

	wait_ready_or_delay(chip, delay_io_write );

	rw_lock_release();	// release lock after delay

	Debug_Printf("[%s]: %s\n",__func__, str);
}

/***********************************************************************
*  FUNCTION NAME: read_io_port()
*
* DESCRIPTION:
*	Use reading of 32 bits commands ("W/R" instead of "w/r")
*	Indirect write: 
*		write address to registers 5 (and implicitly also to 6)
*		write "007R"  (by register 7) 
*		read 8 + 1 bytes
*
* PARAMETERS:
* 	int chip, uint32_t addr, int32_t val
*
* RETURNS:
*
***********************************************************************/
int32_t read_io_port(int chip, int32_t addr)		//spi_implementation

{
	char str[20];
	int len, ret;
	int32_t val;
	
	/* Write Format:<RRR>W<VVVVVVVV> */
	len = sprintf(str, "%03xW%08x", REG_IO_PORT_ADDR_LO_05, addr);
	str[len]=0;


	Debug_Printf("[%s]: %s\n",__func__, str);

	rw_lock_aquire();

	wakeup(chip);

	set_ready_flag(chip, FLAG_CLEAR);

	spi_write((SPI *)chip_io[chip], (uint8_t *) str, strlen(str));

	wait_ready_or_delay(chip, 5);

	/* Write Format:<007>R   -- command to read */
	len = sprintf(str, "%03xR", REG_IO_PORT_VALUE_LO_07);
	str[len]=0;

	set_ready_flag(chip, FLAG_CLEAR);

	spi_write((SPI *)chip_io[chip], (uint8_t *) str, strlen(str));

	wait_ready_or_delay(chip, 5);

	/* read value from spi: */
	ret = spi_read((SPI *)chip_io[chip], (uint8_t *)str, 9);

	ms_delay(2);		// no ready after read
	
	rw_lock_release();
//BEN: Apollo4 SPI does not return transferred bytes but SUCCESS(0)/FAIL(-1) status
//  So we do not compare size here
	if(ret != 0) {  //9) {
		Debug_Printf("[%s]: io_read error.\n", __func__);
		pabort("[read_register]: io_read error.");
	}

	str[9] = 0;

	if ((str[8] == 0) && (str[7] == 0) && (str[6] == 0) && (str[5] == 0) && (str[4] == 0) && (str[3] == 0) && (str[2] == 0) && (str[1] == 0)){
		Debug_Printf("*************************************************************\n\n");
		Debug_Printf("*************************************************************\n\n");
		Debug_Printf("[%s]: reg 0x%X - Probably IO READ ERROR!!.\n\n", __func__, addr);
		Debug_Printf("*************************************************************\n\n");
		Debug_Printf("*************************************************************\n\n");
	}
	/* First read byte by SPI must be ignored. */
	val = atoh__((char*)&str[1]);

	Debug_Printf("[%s]: read address = 0x%X, val = 0x%X.\n", __func__, addr, val);
	return val;

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
void write_register(int chip, int16_t reg, int16_t val)			//spi_implementation

{
	int ret, len;
	char str[16];
	len = sprintf(str, "%03xw%04x", reg, (val)&0xffff);

	str[len]=0;
#if OPTIMIZE_LEVEL == 0
	if (!skip_prints) Debug_Printf("[%s]: %s\n",__func__, str);
#endif
	ret = io_write_delay(chip, (unsigned char*)str, len);
//[BEN] Adapt Apollo SPI implementation
	//if(ret != strlen(str)) {
  if(ret) {
		Debug_Printf("[%s]: io_write error %d.\n", __func__, ret);
		pabort("[write_register]: io_write error.");
	}
}

/***********************************************************************/

void write_register_32(int chip, int16_t reg, int32_t val)			//spi_implementation

{
	int ret, len;
	char str[24];

	len = sprintf(str, "%03xW%08x", reg, (val)&0xffffffff);
	str[len]=0;
#if OPTIMIZE_LEVEL == 0
	Debug_Printf("[%s]: %s\n",__func__, str);
#endif
	ret = io_write_delay(chip, (unsigned char*)str, len);
//[BEN] Adapt Apollo SPI implementation
	//if(ret != strlen(str)) {
  if(ret) {
		Debug_Printf("[%s]: io_write error %d.\n", __func__, ret);
		pabort("[write_register]: io_write error.");
	}
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
int16_t read_register(int chip, int16_t reg)		//spi_implementation

{
	uint16_t val = 0;
	int ret, len;
	unsigned char Buff[10] = {0};
	char RegSet[18] = {0};

	len = sprintf(RegSet, "%03xr", reg);
	RegSet[len]=0;
	
	rw_lock_aquire();

	wakeup(chip);

	set_ready_flag(chip, FLAG_CLEAR);

	ret = spi_write((SPI *)chip_io[chip], (uint8_t *) RegSet, len);

	wait_ready_or_delay(chip, 5);
	
//BEN: Apollo SPI does not return transferred bytes but SUCCESS(0)/FAIL(-1) status
//  So we do not compare size here
	if(ret != 0) {  //strlen(RegSet)) {
		rw_lock_release();
		Debug_Printf("[%s]: io_write error %d.\n", __func__, ret);
		pabort("[read_register]: io_write error.");
	}

 	ret = spi_read((SPI *)chip_io[chip], (uint8_t *)Buff, 5);

	ms_delay(2);	// no ready after read

	rw_lock_release();

//BEN: Apollo SPI does not return transferred bytes but SUCCESS(0)/FAIL(-1) status
//  So we do not compare size here
	if(ret != 0) {  //5) {
		Debug_Printf("[%s]: io_read error.\n", __func__);
		pabort("[read_register]: io_read error.");
	}

	Buff[5] = 0;
	/* First byte read by SPI must be ignored. */
	/* in spi ascii protocol - bytes should not be real zeros (only ascii of zero 0x30) */
	if ((Buff[4] == 0) && (Buff[3] == 0) && (Buff[2] == 0) && (Buff[1] == 0)){ 
	//toggle_debug_gpio();		// for debug, when require.
		Debug_Printf("*************************************************************\n\n");
		Debug_Printf("*************************************************************\n\n");
		Debug_Printf("[%s]: reg 0x%X - Probably IO READ ERROR!!.\n\n", __func__, reg);
		Debug_Printf("*************************************************************\n\n");
		Debug_Printf("*************************************************************\n\n");
	}

	val = atoh__((char*)&Buff[1]);
#if OPTIMIZE_LEVEL == 0
	if (!skip_prints) Debug_Printf("[%s]: register = 0x%X, val = 0x%X.\n", __func__, reg, val);
#endif
	return val;
}

/***********************************************************************/

int32_t read_register_32(int chip, int16_t reg)		//spi_implementation

{
	uint32_t val = 0;
	int ret, len;
	unsigned char Buff[20] = {0};
	char RegSet[18] = {0};

	len = sprintf(RegSet, "%03xR", reg);
	RegSet[len]=0;
	
	rw_lock_aquire();

	wakeup(chip);

	set_ready_flag(chip, FLAG_CLEAR);

	ret = spi_write((SPI *)chip_io[chip], (uint8_t *) RegSet, len);

	wait_ready_or_delay(chip, 5);

//BEN: Apollo SPI does not return transferred bytes but SUCCESS(0)/FAIL(-1) status
//  So we do not compare size here
	if(ret != 0) {  //strlen(RegSet)) {
		rw_lock_release();
		Debug_Printf("[%s]: io_write error %d.\n", __func__, ret);
		pabort("[read_register]: io_write error.");
	}

 	ret = spi_read((SPI *)chip_io[chip], (uint8_t *)Buff, 9);

	ms_delay(2);		// no ready after read

	rw_lock_release();

//BEN: Apollo SPI does not return transferred bytes but SUCCESS(0)/FAIL(-1) status
//  So we do not compare size here
	if(ret != 0) {  //9) {
		Debug_Printf("[%s]: io_read error.\n", __func__);
		pabort("[read_register]: io_read error.");
	}

	Buff[9] = 0;

	if ((Buff[8] == 0) && (Buff[7] == 0) && (Buff[6] == 0) && (Buff[5] == 0) && (Buff[4] == 0) && (Buff[3] == 0) && (Buff[2] == 0) && (Buff[1] == 0)){
		Debug_Printf("*************************************************************\n\n");
		Debug_Printf("*************************************************************\n\n");
		Debug_Printf("[%s]: reg 0x%X - Probably IO READ ERROR!!.\n\n", __func__, reg);
		Debug_Printf("*************************************************************\n\n");
		Debug_Printf("*************************************************************\n\n");
	}
	/* First read byte by SPI must be ignored. */
	val = atoh__((char*)&Buff[1]);

	Debug_Printf("[%s]: register = 0x%X, val = 0x%X.\n", __func__, reg, val);
	return val;
}

/***********************************************************************
* FUNCTION NAME: read_checksum()
*
* DESCRIPTION:
*	Send check sum command to DBMDX using io and comprae the results
*	to the expeceted given checksum.
*
* PARAMETERS:
*	char *checksum - array of 4 char the expected checksum
* RETURNS:
*	0 - checksum is OK
*	else error
***********************************************************************/
int read_checksum(int chip, char *checksum)			//spi_implementation

{
	char buf[] = {BOOT_PRE_OP_CODE, BOOT_READ_CHECKSUM};
	char c[7];
	int rc ;

	spi_write((SPI *)chip_io[chip], (uint8_t *)buf, 2);
#if OPTIMIZE_LEVEL > 1
  ms_delay(2);
#else 
	ms_delay(10);	//?? too long 
#endif


 	spi_read((SPI *)chip_io[chip], (uint8_t *)c, 7);

	ms_delay(2);
        
	/* First read byte by SPI must be ignored. */
	rc = strncmp(&c[3],checksum, 4);
	
	if (rc) {
		Debug_Printf ("checksum error : got: = %2x %2x %2x %2x expected : %2x %2x %2x %2x\n",
		c[3],c[4],c[5],c[6],checksum[0],checksum[1],checksum[2],checksum[3]);
		rc = -1;
	}else{
		Debug_Printf ("checksum pass\n");
	}
    return rc ;
}

/***********************************************************************/

// read a chunk from transport, header (optional) + buf
int read_chunk (int chip, uint8_t * header, int header_len, char * buf, int bytes_to_read)
{

	rw_lock_aquire();
	
 	if (header_len){ 
		spi_read((SPI *)chip_io[chip], (uint8_t *)header, header_len);
	}

 	spi_read((SPI *)chip_io[chip], (uint8_t *)buf, bytes_to_read);

	ms_delay(2);		// no ready after read

	rw_lock_release();
	
	return bytes_to_read;

}

/***********************************************************************/

// request and read chunk (single lock transaction) from transport, header (optional) + buf
int request_and_read_chunk (int chip, int16_t request_reg, int16_t request_val, uint8_t * header, int header_len, char * buf, int bytes_to_read)
{
	int rc, len;
	char str[16];

	// prepare request:
	len = sprintf(str, "%03xw%04x", request_reg, (request_val)&0xffff);

	str[len]=0;
#if OPTIMIZE_LEVEL == 0
	if (!skip_prints) Debug_Printf("[%s]: %s\n",__func__, str);
#endif
	rw_lock_aquire();

	wakeup(chip);

	set_ready_flag(chip, FLAG_CLEAR);

	// send request:
	rc  = spi_write((SPI *)chip_io[chip], (uint8_t *) str, len);

//BEN: Apollo SPI does not return transferred bytes but SUCCESS(0)/FAIL(-1) status
//  So we do not compare size here
	if(rc != 0) { //strlen(str)) {
		rw_lock_release();
		Debug_Printf("[%s]: io_write error %d.\n", __func__, rc);
	}

	wait_ready_or_delay(chip, delay_io_write);

	// read header (optional) and chunk:
 	if (header_len){ 
		spi_read((SPI *)chip_io[chip], (uint8_t *)header, header_len);
	}

 	spi_read((SPI *)chip_io[chip], (uint8_t *)buf, bytes_to_read);

	ms_delay(2);		// no ready after read

	rw_lock_release();

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
int load_file(int chip, char *filename, int skip_bytes)		//spi_implementation

{
	int fileLen;

	char checksum[CHECKSUM_SIZE];
	int rc;
	long err = 1;
	int i;
	const char clr_crc_chip[]= {BOOT_PRE_OP_CODE, BOOT_RESET_CHECKSUM};
	ImgFile * fw_image;

#if 0 //[BEN]Should use dynamic allocation for big size buffer
  char chunk_buff[SPI_CHUNK];
#else 
  char *chunk_buff = fsl_malloc(SPI_CHUNK);
  if(chunk_buff == NULL) {
    Debug_Printf("Failed to allocate buf for SPI transfer\n");
    return -1;
  }
#endif
		
	fw_image = IMG_fopen(filename);		// Open source file

	if (!fw_image) goto exit_1;

	fileLen = fw_image->size;

	Debug_Printf("fileLen = %d skip_bytes = %d\n",fileLen,skip_bytes );

	// Upload File to FW

	//wakeup(chip);		// too early

	if(skip_bytes >= CHECKSUM_SIZE) {		// checksum to be verified

		IMG_fread(fw_image, (uint8_t *)checksum, CHECKSUM_SIZE, fileLen - skip_bytes);

		// init chip checksum (if not d2)
		if (chip != DBM_D2){
			rc  = spi_write((SPI *)chip_io[chip], (uint8_t *)clr_crc_chip, sizeof(clr_crc_chip));

			if(rc < 0) {
				Debug_Printf("[load_file]: fail to write clr_crc command");
				err = -1;
				goto exit_1;
			}
		}
	}
#if ENABLE_D10_BURST_SPI
  ExtDsp_SetBurstSpi();   //instruct SPI to keep CS active 
#endif
	for(i = 0 ; i < (fileLen - skip_bytes); i += SPI_CHUNK) {
		int chunk ;

		chunk = (SPI_CHUNK < ((fileLen - skip_bytes) - i) ? SPI_CHUNK : (fileLen - skip_bytes) - i);

		IMG_fread(fw_image, (uint8_t *)chunk_buff, chunk , i);
#if ENABLE_D10_BURST_SPI
    if(chunk < SPI_CHUNK) {
      ExtDsp_ClearBurstSpi(); //imstruct SPI to deassert CS after this last transfer
    }
#endif	
//BEN: Apollo SPI does not return transferred bytes but SUCCESS(0)/FAIL(-1) status
//  So we do not compare size here
    //if(chunk != spi_write((SPI *)chip_io[chip], (uint8_t *)chunk_buff, chunk)) {
    if(spi_write((SPI *)chip_io[chip], (uint8_t *)chunk_buff, chunk)) {
      Debug_Printf("error loading file\n");
      err = -1;
      goto exit_1;
    }
  }
#if OPTIMIZE_LEVEL > 1
  ms_delay(2);
#else
	ms_delay(10);			//?? To be tuned 
#endif

	if(skip_bytes >= CHECKSUM_SIZE){			// checksum to be verified
		rc = read_checksum(chip, checksum);
		if(rc < 0) {
			Debug_Printf("[%s]: fail read checksum.\n", __func__);
			err = -1;
			goto exit_1;
		}
	}
	
exit_1:
  if(chunk_buff != NULL) {
    fsl_free(chunk_buff);
  }
	IMG_fclose(fw_image);
    Debug_Printf("%s: ret %d\n", __func__, err);
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
	int j, length, err;
	int source_read_bytes=0;
		
	ImgFile * fw_image;

  char *chunk_buff = fsl_malloc(SPI_CHUNK);
  if(chunk_buff == NULL) {
    Debug_Printf("Failed to allocate buf for SPI transfer\n");
    return -1;
  }

	// Open the file
	fw_image = IMG_fopen(filename);		// Open source file

	// Get file length
	fileLen = fw_image->size;
	Debug_Printf("[load_amodel_file]: %s, fileLen:%d \n", filename, (unsigned int)fileLen - skip_bytes);

	// read header+data pairs as much as exist in source file (before its tail)  

	while(source_read_bytes < (fileLen - BINARY_FILE_TAIL_LEN)) {		

		// Debug_Printf("source_read_bytes:  %d, fileLen:  %d \n",source_read_bytes,(unsigned int)fileLen );

		// read header of file
		IMG_fread(fw_image, (uint8_t *)header_buffer, sizeof(header_buffer), IMG_CUR_POS);
		source_read_bytes += sizeof(header_buffer);
		
		if(header_buffer[0] != BOOT_PRE_OP_CODE && (header_buffer[1] != 0x01 || header_buffer[1] != 0x02)) {
			Debug_Printf("[load_amodel_file]: header error.\n");
			err = -1;
			goto error_1;
		}

		wakeup(chip);

		// Debug_Printf("header size = %d\n", sizeof(header_buffer));
		// Send header to FW
		if(io_write_delay(chip, &header_buffer[0], sizeof(header_buffer)) < 0){
			goto error_1;
			err = -1;
		}			
#if OPTIMIZE_LEVEL < 1
		ms_delay(100);	//?? to be tuned 
#endif

		// Read data from file 

		num_of_words = ((uint32_t)header_buffer[5] << 24)
				| ((uint32_t)header_buffer[4] << 16)
				| ((uint32_t)header_buffer[3] << 8)
				| ((uint32_t)header_buffer[2] << 0);

		Debug_Printf("[load_amodel_file]: num_of_words:%d \n", num_of_words);

		length = num_of_words * 2;
//[BEN] I disabled this code as we will read in by chunks
#if 0
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
#endif

		// Send to FW
#if ENABLE_D10_BURST_SPI
    ExtDsp_SetBurstSpi(); //instruct SPI to keep CS asserted for Burst transfers
#endif
		for(j = 0 ; j < (length); j += SPI_CHUNK) {
			int chunk;
			chunk = (SPI_CHUNK < ((length) - j) ? SPI_CHUNK : (length) - j);

      IMG_fread(fw_image, (uint8_t *)chunk_buff, chunk , IMG_CUR_POS);
      source_read_bytes += chunk;
#if ENABLE_D10_BURST_SPI
      if(chunk < SPI_CHUNK) {
        ExtDsp_ClearBurstSpi(); //instruct SPI to de-assert CS after this last transfer
      }
#endif
//BEN: Apollo SPI does not return transferred bytes but SUCCESS(0)/FAIL(-1) status
//  So we do not compare size here
    if(spi_write((SPI *)chip_io[chip], (uint8_t *)chunk_buff, chunk)) {
		//if(chunk != io_write_delay(chip, &buffer[j], chunk)) {
				Debug_Printf("%s: error loading file\n", filename);
				err = -1;
				goto error_2;
			}
		}
#if OPTIMIZE_LEVEL > 1
    ms_delay(5);
#else
		ms_delay(100);	// to be tuned //??
#endif

		//free(buffer);
	} /* while another (header+data) pair exists in source */

	//IMG_fclose(fw_image);
	//return 1;
  err = 1;
  goto exit;
	
error_2:
	//free(buffer);
error_1:
  Debug_Printf("[%s:%d]: NOK\n", __func__, __LINE__);
exit:
  if(chunk_buff != NULL) {
    fsl_free(chunk_buff);
  }
	IMG_fclose(fw_image);
	return err;
}
/***********************************************************************/