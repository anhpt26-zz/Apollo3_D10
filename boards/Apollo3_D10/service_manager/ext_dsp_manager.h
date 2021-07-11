#ifndef EXT_DSP_MANAGER_H
#define EXT_DSP_MANAGER_H

#ifdef __cplusplus
extern "C"
{
#endif

//Level 1: Disable all printing, skip some delay in INT handler
//Level 2: Reduce some delays between steps
//  - Reset pulse reduces from 10 to 1ms
//  - Delay after reset reduces from 70ms to 5ms
//  - Delay to read checksum from 10ms to 2ms
//  - Delay loop when waiting power up flag reduces from 10ms to 2ms
//  - Skip fw_validate in init_chip() as we will do after that
//  - Delay in init_config_chip() reduces from 100ms to 20ms
//  - Blocking delay in wait_ready_or_delay() reduces from 5ms to 1ms
//  - Delay after sending header of model file reduces from 100ms to 5ms

#define OPTIMIZE_LEVEL      2

enum {
  EXT_DSP_INT_EVT,
  EXT_DSP_VT_EVT,
  EXT_DSP_START_STREAM_VOICE_EVT,
  EXT_DSP_STOP_STREAM_VOICE_EVT,
  EXT_DSP_READ_AUDIO_FRAME_EVT,
  EXT_DSP_MAX_EVT
};

#define VOICE_STREAMING_SECONDS       (5)

//[BEN]TODO: Replace this with real file system operation
#define IMG_CUR_POS     (-1)
typedef struct {
  uint8_t *file_buf;
  uint32_t size;
  uint32_t read_pos;
}ImgFile;
ImgFile *IMG_fopen(const char *name);
void IMG_fclose(ImgFile *fimg);
int IMG_fread(ImgFile *fimg, uint8_t *buf, uint32_t size, uint32_t pos);


void ExtDspMgr_Init(void);
void ExtDspMgr_Task(void *pvParameter);
void ExtDsp_ResetChip();
void ExtDsp_WakeChip();
void ExtDspMgr_EnterLowPower();
void ExtDspMgr_ExitLowPower();
int ExtDsp_SpiTransfer(uint8_t *buf, int len, bool is_tx);
void ExtDsp_SetBurstSpi();
void ExtDsp_ClearBurstSpi(); 
void ExtDspMgr_SignalEvent(uint8_t event);


#ifdef __cplusplus
}
#endif
#endif    //EXT_DSP_MANAGER_H
