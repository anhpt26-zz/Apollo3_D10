/************************************************************************************************************
 Module:       xxxx.c
 
 Description:  This is the main file that will initialize all the modules
 
 Notes:        ---
 
 History:
 Date          Name      Changes
 -----------   ----      -------------------------------------------------------------------------------------
 3/18/2018     XXX       Began coding

 ************************************************************************************************************/
#include <string.h>
#include <stdio.h>
#include "am_mcu_apollo.h"
#include "system.h"
#include "board_def.h"
#include "debug.h"
#include "am_util.h"

#include "dx_params.h"
#include "dx_fw_defs.h"
#include "dx_driver.h"
#include "dx_utils.h"
#include "dx_usecases.h"
#include "dx_gen.h"

#include "ext_dsp_manager.h"

//###########################################################################################################
//      CONSTANT DEFINITION
//###########################################################################################################
//NOTE: EVK setup cannot work with SCK > 12MHz
#define DSP_SPI_FREQ_HZ              AM_HAL_IOM_12MHZ 
#if SKMBv1_BOARD
#define DSP_SPI_MODULE               3
#else 
#define DSP_SPI_MODULE               1
#endif
//###########################################################################################################
//      TYPEDEF DEFINITION
//###########################################################################################################



//###########################################################################################################
//      Module Level Variables
//###########################################################################################################
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

int s_state = 0;
int s_count;      /* trigger counter */

void *Dsp_Spi_Handler = NULL;
QueueHandle_t Dsp_Signal_Handler;

uint8_t Dsp_Voice_Streaming_Timer;
uint8_t Dsp_Audio_Samples_Timer;
bool Dsp_Stop_Voice_Streaming;
char Dsp_Audio_Samples[3000];
uint32_t Dsp_Audio_Sample_Num;
//###########################################################################################################
//      PRIVATE FUNCTION DECLARATION
//###########################################################################################################
static int extDspMgr_InitD10();
static void d10_InterruptIsr(void *pArg);
static void d10_InterruptReadyIsr(void *pArg);
void vt_vc_trigger(int chip);
static void extDspMgr_StartVoiceStreaming(void);
static void extDspMgr_StopVoiceStreaming(void);
static void extDspMgr_VoiceStreamTimeout(void);
static void extDspMgr_VoiceSamplesTimeout(void);
//###########################################################################################################
//      PUBLIC FUNCTION DEFINITION
//###########################################################################################################
void ExtDspMgr_Init(void) {
  am_hal_gpio_pincfg_t gpio_int_cfg =
  {
    .GP.cfg_b.uFuncSel = 3,
    .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI, //AM_HAL_GPIO_PIN_INTDIR_HI2LO,
    .GP.cfg_b.eGPInput = AM_HAL_GPIO_PIN_INPUT_ENABLE,
    .GP.cfg_b.eGPOutCfg = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE,  //AM_HAL_GPIO_PIN_PULLUP_50K,
  };


  uint32_t    irq_status;
  uint32_t  irq_pin = ABQ_D10_INT_PIN;
  IRQn_Type irq_num = GPIO0_203F_IRQn; 

  //configure input interrupts for INT and RDY INT signals from D10
  am_hal_gpio_pinconfig(ABQ_D10_INT_PIN, gpio_int_cfg);
  am_hal_gpio_pinconfig(ABQ_D10_RDY_PIN, gpio_int_cfg);

  am_hal_gpio_interrupt_irq_status_get(irq_num, false, &irq_status);
  am_hal_gpio_interrupt_irq_clear(irq_num, irq_status);
  am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, ABQ_D10_INT_PIN, d10_InterruptIsr, NULL);
  am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,(void *)&irq_pin);
  NVIC_SetPriority((IRQn_Type)irq_num, 4);
  NVIC_EnableIRQ((IRQn_Type)irq_num);


  irq_pin = ABQ_D10_RDY_PIN;
  irq_num = GPIO0_203F_IRQn;  
  am_hal_gpio_interrupt_irq_status_get(irq_num, false, &irq_status);
  am_hal_gpio_interrupt_irq_clear(irq_num, irq_status);
  am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, ABQ_D10_RDY_PIN, d10_InterruptReadyIsr, NULL);
  am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,(void *)&irq_pin);
  NVIC_SetPriority((IRQn_Type)irq_num, 4);
  NVIC_EnableIRQ((IRQn_Type)irq_num);

  //configure outputs to drive WAKEUP and RST signals of D10
  am_hal_gpio_pinconfig(ABQ_D10_RST_PIN, am_hal_gpio_pincfg_output);
  am_hal_gpio_pinconfig(ABQ_D10_WAKEUP_PIN, am_hal_gpio_pincfg_output);
 
  //initialize SPI interface
  am_hal_iom_config_t iom_spi_settings =
  {
      .eInterfaceMode = AM_HAL_IOM_SPI_MODE,
      .ui32ClockFreq = DSP_SPI_FREQ_HZ,
      .eSpiMode = AM_HAL_IOM_SPI_MODE_0,
      .ui32NBTxnBufLength = 0,
      .pNBTxnBuf = NULL,
  };
  am_hal_iom_initialize(DSP_SPI_MODULE, &Dsp_Spi_Handler);
  am_hal_iom_power_ctrl(Dsp_Spi_Handler, AM_HAL_SYSCTRL_WAKE, false);
  am_hal_iom_configure(Dsp_Spi_Handler, &iom_spi_settings);
  am_hal_iom_enable(Dsp_Spi_Handler);

  //Init SPI pins
#if  SKMBv1_BOARD
    am_hal_gpio_pincfg_t pin_cfg =
    {
        .GP.cfg_b.uFuncSel             = AM_HAL_PIN_31_M3SCK,
        .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
        .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
        .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
        .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
        .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,
        .GP.cfg_b.uSlewRate            = 0,
        .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_100K,
        .GP.cfg_b.uNCE                 = 0,
        .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
        .GP.cfg_b.uRsvd_0              = 0,
        .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
        .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.uRsvd_1              = 0,
    };

    //Because base on , SPI peripheral. Idle status of MOSI and MISO pin is High
    pin_cfg.GP.cfg_b.uFuncSel = AM_HAL_PIN_33_M3MISO;
    am_hal_gpio_pinconfig(ABQ_D10_MISO_PIN, pin_cfg);

    pin_cfg.GP.cfg_b.uFuncSel = AM_HAL_PIN_32_M3MOSI;
    am_hal_gpio_pinconfig(ABQ_D10_MOSI_PIN, pin_cfg);
    
    //Because base on , SPI peripheral. Idle status of SCK and CS pin is High
    //pin_cfg.GP.cfg_b.ePullup  = AM_HAL_GPIO_PIN_PULLUP_100K;

    pin_cfg.GP.cfg_b.uFuncSel = AM_HAL_PIN_31_M3SCK;
    am_hal_gpio_pinconfig(ABQ_D10_SCK_PIN,  pin_cfg);

    //pin_cfg.GP.cfg_b.uFuncSel = AM_HAL_PIN_25_NCE25;
    //pin_cfg.GP.cfg_b.uNCE     = AM_HAL_GPIO_NCE_IOM3CE0;
    //am_hal_gpio_pinconfig(ABQ_D10_CS_PIN,   pin_cfg);

    pin_cfg.GP.cfg_b.uFuncSel = AM_HAL_PIN_11_NCE11 ;
    pin_cfg.GP.cfg_b.uNCE     = AM_HAL_GPIO_NCE_IOM3CE0;
    am_hal_gpio_pinconfig(ABQ_D10_CS_PIN,   pin_cfg);  

#else 
  am_hal_gpio_pincfg_t pin_cfg =
  {
      .GP.cfg_b.uFuncSel             = AM_HAL_PIN_10_M1MISO,
      .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
      .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
      .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
      .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
      .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,
      .GP.cfg_b.uSlewRate            = 0,
      .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_100K,
      .GP.cfg_b.uNCE                 = 0,
      .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
      .GP.cfg_b.uRsvd_0              = 0,
      .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
      .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
      .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
      .GP.cfg_b.uRsvd_1              = 0,
  };

  //Because base on , SPI peripheral. Idle status of MOSI and MISO pin is High
  am_hal_gpio_pinconfig(ABQ_D10_MISO_PIN, pin_cfg);
  pin_cfg.GP.cfg_b.uFuncSel = AM_HAL_PIN_9_M1MOSI;
  am_hal_gpio_pinconfig(ABQ_D10_MOSI_PIN, pin_cfg);
  //Because base on , SPI peripheral. Idle status of SCK and CS pin is High
  //pin_cfg.GP.cfg_b.ePullup  = AM_HAL_GPIO_PIN_PULLUP_100K;
  pin_cfg.GP.cfg_b.uFuncSel = AM_HAL_PIN_8_M1SCK;
  //NOTE: Need to increase drive strength to capture with Saleae
  //Can revert to lower drive strenght at the end to save power
  pin_cfg.GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X; 
  am_hal_gpio_pinconfig(ABQ_D10_SCK_PIN,  pin_cfg);

  pin_cfg.GP.cfg_b.uFuncSel = AM_HAL_PIN_11_NCE11 ;
  pin_cfg.GP.cfg_b.uNCE     = AM_HAL_GPIO_NCE_IOM1CE0;
  am_hal_gpio_pinconfig(ABQ_D10_CS_PIN,   pin_cfg);  
#endif


  //32KHz 
  /* Generate 32KHz clock source to input to CYW43012 module  */
  am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_RTC_SEL_XTAL, 0);   //Using LF XTAL(32KHz)
  am_hal_clkgen_clkout_enable(true, AM_HAL_CLKGEN_CLKOUT_XTAL_32768);
  am_hal_gpio_pincfg_t pin_cfg_32 = AM_HAL_GPIO_PINCFG_OUTPUT;
  pin_cfg_32.GP.cfg_b.uFuncSel = AM_HAL_PIN_33_CLKOUT;
  am_hal_gpio_pinconfig(ABQ_D10_MCLK_PIN, pin_cfg_32);

  am_hal_gpio_output_clear(ABQ_D10_RST_PIN);
  am_hal_gpio_output_set(ABQ_D10_WAKEUP_PIN);


  //uint8_t test_buf[10] = {1,2,3,4,5,6,7,8,9,10};
  //ExtDsp_SpiTransfer(test_buf, 10, true);
}

void ExtDsp_ResetChip() {
  am_hal_gpio_output_clear(ABQ_D10_RST_PIN);
  am_util_delay_ms(10);
  am_hal_gpio_output_set(ABQ_D10_RST_PIN);
}

void ExtDsp_WakeChip() {
  am_hal_gpio_output_clear(ABQ_D10_WAKEUP_PIN);
  am_util_delay_ms(1);
  am_hal_gpio_output_set(ABQ_D10_WAKEUP_PIN);
  am_util_delay_ms(10);
}

static bool ExtDsp_Spi_Burst = false;
void ExtDsp_SetBurstSpi() {
  ExtDsp_Spi_Burst = true;
}
void ExtDsp_ClearBurstSpi() {
  ExtDsp_Spi_Burst = false;
}

int ExtDsp_SpiTransfer(uint8_t *buf, int len, bool is_tx) {
  am_hal_iom_transfer_t Transaction;
  Transaction.ui8RepeatCount = 0;
  Transaction.ui32PauseCondition = 0;
  Transaction.ui32StatusSetClr = 0;
  Transaction.ui32NumBytes = len;
  Transaction.uPeerInfo.ui32SpiChipSelect = 0;
  Transaction.ui64Instr = 0x00;
  Transaction.ui32InstrLen = 0;
  if(ExtDsp_Spi_Burst) {
    Transaction.bContinue = true;
  } else {
    Transaction.bContinue = false;
  }

  if(is_tx) {
    Transaction.eDirection = AM_HAL_IOM_TX; //AM_HAL_IOM_TX;
    Transaction.pui32RxBuffer = NULL;
    Transaction.pui32TxBuffer = (uint32_t *)buf;
    #if 0
    Debug_Printf("\nSPI_TX:\n");
    for(int i=0; i<len; i++){
      Debug_Printf("0x%x,", buf[i]);
    }
    #endif
  } else {
    Transaction.eDirection = AM_HAL_IOM_RX;    
    Transaction.pui32RxBuffer = (uint32_t *)buf;
    Transaction.pui32TxBuffer = (uint32_t *)NULL;
  }

  int ret = am_hal_iom_blocking_transfer(Dsp_Spi_Handler, &Transaction);
  if(ret) {
    Debug_Printf("Failed to write SPI to D10\n");
    return -1;
  }
  return 0;
}


void ExtDspMgr_SignalEvent(uint8_t event) {
  if(Dsp_Signal_Handler == NULL || System_SignalEvt(Dsp_Signal_Handler, event)) 
  {
      Debug_Printf("Failed to signal event to ExtDspMgr\n");
  }
}

void ExtDspMgr_EnterLowPower() {
  use_case_wakeup_detection_Exit(g_chip1);
  ms_delay(5);
  d10l_enter_hibernation(g_chip1, g_mic1_type, g_mic2_type);
}

void ExtDspMgr_ExitLowPower() {
  ExtDsp_WakeChip();
  d10l_exit_hibernation(g_chip1, g_mic1_type, g_mic2_type);
  ms_delay(5);
  use_case_wakeup_detection_Enter(g_chip1, g_wake_word_engine);
}


void ExtDspMgr_Task(void *pvParameter) {
  uint8_t event;
  int sample_num, recorded_sample_num;
  Dsp_Signal_Handler = xQueueCreate(EXT_DSP_MAX_EVT, 1);
  if (Dsp_Signal_Handler == NULL)
  {  
    Debug_Printf("FAILED to create Ext DSP signal handler\r\n");  
    return;
  }
  if(extDspMgr_InitD10()) {
    Debug_Printf("Failed to init D10\n");
    return;
  }
  Dsp_Stop_Voice_Streaming = true;
  while (1) {
    xQueueReceive(Dsp_Signal_Handler, &event, portMAX_DELAY);
    switch(event) {
      case EXT_DSP_VT_EVT:
        use_case_wakeup_vt_vc_trigger(0);   //check which WWE is triggered
        break;
      case EXT_DSP_START_STREAM_VOICE_EVT:
        extDspMgr_StartVoiceStreaming();
        SoundMgr_StartLoopback();
        break;
      case EXT_DSP_STOP_STREAM_VOICE_EVT:
        extDspMgr_StopVoiceStreaming();
        SoundMgr_StopLoopback();
        break;
      case EXT_DSP_READ_AUDIO_FRAME_EVT:
        if(Dsp_Stop_Voice_Streaming) break;
#define CHANNEL_NUM         1
        //Read sample number in D10 AUDIO BUFFER
        recorded_sample_num = record_buffer(g_chip1, AUDIO_SAMPLE_RATE, CHANNEL_NUM, Dsp_Audio_Samples, Dsp_Audio_Sample_Num);
        Debug_Printf("Recorded size = %d\n", recorded_sample_num);
        if(recorded_sample_num) {
          // Request Sound Manager to play with self ping-pong  
          SoundMgr_PlayAudioSamples(Dsp_Audio_Samples, recorded_sample_num); 
          //Start next 93ms timeout to wait for next batch of audio samples
          System_StartHwTimer(Dsp_Audio_Samples_Timer);
        } else {
          extDspMgr_StopVoiceStreaming();
          SoundMgr_StopLoopback();
        }
        break;
      default: break;
    }
  }
}

void ExtDspMgr_HandleTestCommand(char cmd){
  switch(cmd) {
    case '1':
      ExtDspMgr_EnterLowPower();
      break;
    case '2':
      ExtDspMgr_ExitLowPower();
      break;
    case '3':
      Debug_Printf("fw state = %d\n", read_register(g_chip1, OPERATION_MODE_01));
      break;
    default: break;
  }
}

//[BEN]TODO: Replace with real file system operation
#include "Amodels/LDE/SmartWatch_dsp_model.c"
#include "Amodels/LDE/SmartWatch_nn_model.c"
#include "Amodels/Google/210124/en_all.c"
#include "fw/Kiwi_D10_ver_4633_RC2_OKG_210124_LDEVT_100_NNL.c"
ImgFile watch_dsp_model_file = {.file_buf = (uint8_t *)_acSmartWatch_dsp_model,
                          .size = 4576UL, .read_pos = 0};
ImgFile watch_nn_model_file = {.file_buf = (uint8_t *)_acSmartWatch_nn_model,
                              .size = 265848UL, .read_pos=0};
ImgFile google_model = {.file_buf = (uint8_t *)_acen_all,
                              .size = 51242UL, .read_pos=0};                
ImgFile fw_file = {.file_buf = (uint8_t *)_acKiwi_D10_ver_4633_RC2_OKG_210124_LDEVT_100_NNL,
                   .size = 186818UL, .read_pos = 0}; 
ImgFile *IMG_fopen(const char *name) {
  if(strcmp(name, A_MODEL_SMARTWATCH_DSP) == 0) {
    return &watch_dsp_model_file;
  } else if(strcmp(name, A_MODEL_SMARTWATCH_NN) == 0) {
    return &watch_nn_model_file;
  } else if(strcmp(name, A_MODEL_OKG) == 0) {
    return &google_model;
  } else if(strcmp(name, FIRMWARE_IMAGE) == 0) {
    return &fw_file;
  } else return NULL;
}                       
                                       
void IMG_fclose(ImgFile *fimg) {
  fimg->read_pos = 0;
  return;
}
int IMG_fread(ImgFile *fimg, uint8_t *buf, uint32_t size, uint32_t pos) {
  int read_size = 0;
  uint32_t start_addr = (uint32_t)fimg->file_buf;

  if(fimg == NULL) return 0;

  if(pos != IMG_CUR_POS) {
    fimg->read_pos = pos;
  }

  if(fimg->size > fimg->read_pos) {
    if(fimg->size - fimg->read_pos > size) {
      read_size = size;
    } else {
      read_size = fimg->size - fimg->read_pos;
    }

    memcpy(buf,fimg->file_buf + fimg->read_pos, read_size);
    fimg->read_pos += read_size;
  } else {
    Debug_Printf("Invalid IMG_fread\n");
  }

  return read_size;
}
//###########################################################################################################
//      PRIVATE FUNCTION DEFINITION
//###########################################################################################################
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

  //g_spi_speed = SPI_SPEED_3MHz;
  //g_uart_speed = UART_SPEED_1MHz;

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
  //parse_config_file(CONFIG_FILENAME);

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
void print_project_configuration()
{
// add here more parameters from parameters.h
  Debug_Printf("Project Configuration: \n") ;
  Debug_Printf("     chip = D%d \n", g_chip1) ;
  Debug_Printf("     mode = %s \n", (g_mode_flag == 1) ? "fw-fast" : ((g_mode_flag == 2) ? "fw-record" : "fw-debug")) ;
  Debug_Printf("     reg fw records:  5=%x; 6=%x; 7=%x  \n", g_reg5_fw_record, g_reg6_fw_record, g_reg7_fw_record ) ;
  Debug_Printf("     transport = %d    %s \n", g_transport, "0=not yet; 1=SPI; 2=I2C 3=UART") ;
  Debug_Printf("     wakeup = %d \n", g_use_wakeup) ;
  Debug_Printf("     mic1 type = %d    %s \n", g_mic1_type, "0=none; 1=digital; 2=analog; 3=vesper") ;
  Debug_Printf("     mic2 type = %d    %s \n", g_mic2_type, "0=none; 1=digital; 2=analog; 3=vesper") ;
  Debug_Printf("     buffer_time_in_sec = %d \n", g_buffer_time_in_sec) ;
  Debug_Printf("     wake word engine = %d    %s\n", g_wake_word_engine,  "0=none; 1=Sensory; 2=Google;");
  Debug_Printf("     i2s master = %s \n", (g_i2s_chip_master)? "Chip" : "Host");
  Debug_Printf("     host tdm sample rate = %d \n", g_tdm_sample_rate) ;
  Debug_Printf("     host tdm bit rate = %d \n", g_tdm_bit_rate) ;
}
void print_dspg_versions()
{
  uint16_t dspg_fw_version;
  //uint16_t dspg_asrp_engine_version; // ASRP is not loaded yet
  Debug_Printf("customer host version = %s\n", CUSTOMER_HOST_VERSION);
  Debug_Printf("dspg .c reference file version = %s\n\n", g_dspg_host_ver);
  Debug_Printf("dspg Base Version = %s\n", DSPG_BASE_VERSION);

  dspg_fw_version = read_register(g_chip1, 0);
  //dspg_asrp_engine_version = read_register(g_chip1, 0x100);
  Debug_Printf("dspg_fw_version = 0x%x\n", dspg_fw_version);

  //printf("dspg_asrp_engine_version = 0x%x\n", dspg_asrp_engine_version);
  Debug_Printf("dspg FW file name = %s\n", g_FirmWare_filename);
  Debug_Printf("Model file name = %s\n\n", g_trigger_filename);

}

void print_actual_states()
{
  Debug_Printf("current main state = uc%d\n",s_state);
  //printf("fw mode = %d    1=no-log; 2=fw-record;  3=fw-debug \n", g_mode_flag);
  Debug_Printf("fw mode = %s \n", (g_mode_flag == 1) ? "fw-fast" : ((g_mode_flag == 2) ? "fw-record" : "fw-debug")) ;
  Debug_Printf("fw state = %d\n", read_register(g_chip1, OPERATION_MODE_01));

}

static int extDspMgr_InitD10() {
  Debug_SetDebugPin1(1);
  prepare_project_configuration();
#if OPTIMIZE_LEVEL == 0
  print_project_configuration();
  print_dspg_versions();
  print_actual_states();
#endif
  Debug_Printf("**** init_chip ******\n");
  if(init_chip(g_chip1) < 0) {
    Debug_Printf("Failed to init chip\n");
    return -1;
  }
  Debug_SetDebugPin1(0);
  Debug_Printf("**** init_config_chip ******\n");
  init_config_chip(g_chip1);
  Debug_Printf("**** model_loading ******\n");
  Debug_SetDebugPin1(1);
  model_loading(g_chip1, g_wake_word_engine);
  Debug_SetDebugPin1(0);
  int value = fw_validate(g_chip1);
  Debug_SetDebugPin1(1);
  if(value != 0) {
    Debug_Printf ("*********************************\n");
    Debug_Printf ("*********************************\n");
    Debug_Printf ("******   D%d IS RUNNING       ****\n", g_chip1);
    Debug_Printf ("*********************************\n");
    Debug_Printf ("*********************************\n");
    Debug_Printf ("Chip version = 0x%04X\n", value & 0xffff);
  } else {
    Debug_Printf("D10 not running\n");
    return -1;
  }
  //check_fw_errors(g_chip1);
  //Debug_SetDebugPin1(0);
  Debug_Printf("Enter Wakeup Detection\n");
  use_case_wakeup_detection_Enter(g_chip1, g_wake_word_engine);
  Debug_SetDebugPin1(0);
  s_state = 1;
  check_fw_errors(g_chip1);
  Debug_SetDebugPin1(1);

  return 0;
}

static void extDspMgr_StartVoiceStreaming() {
  if(!Dsp_Stop_Voice_Streaming) return;   //skip any start event while streaming is in progress
  //Get the expected sample num for a ping pong buffer in sound_manager
  Dsp_Audio_Sample_Num = SoundMgr_GetAudioSampleBufSize();
  //NOTE: For unknown reasons, if we allocate/free multiple times, the system gets hardfault
  // Therefore, I use a global buffer and skip using dynamic allocation for now  
  //Dsp_Audio_Samples = fsl_malloc(Dsp_Audio_Sample_Num*2);   //we collect 16-bit audio samples from D10
  //if(Dsp_Audio_Samples == NULL) {
  //  Debug_Printf("Failed to allocate buf to receive audio samples\n");
  //} else {
  {
    //D10 to streaming mode
    move_to_bufferring(g_chip1);
    //Start timer to signal streaming ending event
    Dsp_Voice_Streaming_Timer = System_RegisterHWTimer(VOICE_STREAMING_SECONDS*1000, extDspMgr_VoiceStreamTimeout);
    System_StartHwTimer(Dsp_Voice_Streaming_Timer);
    Dsp_Stop_Voice_Streaming = false;
    //Start 93ms timeout to wait for audio samples in D10 FIFO
    //NOTE: In sound_manager.c we have ping pong buffers to play back MIC to SPEAKER
    // We can reuse that ping pong buffer to play back voice command here.
    // That each ping pong buffer contains 1500samples, each sample is max 32bit
    // D10 MIC settings: 16bit samples @ 16KHz @ Mono => it takes 1500/16K = 93ms to collect 1500samples
    // Therefore we can schedule 93ms timeout to read D10 audio samples and foward to I2S playback
    Dsp_Audio_Samples_Timer = System_RegisterHWTimer(93, extDspMgr_VoiceSamplesTimeout);
    System_StartHwTimer(Dsp_Audio_Samples_Timer);
  }
}

static void extDspMgr_StopVoiceStreaming() {
  if(Dsp_Stop_Voice_Streaming) return;

  Dsp_Stop_Voice_Streaming = true;
  //Free any SW resources
  if(INVALID_TIMER_ID != Dsp_Voice_Streaming_Timer) System_UnRegisterHwTimer(Dsp_Voice_Streaming_Timer);
  if(INVALID_TIMER_ID != Dsp_Audio_Samples_Timer) System_UnRegisterHwTimer(Dsp_Audio_Samples_Timer);
  //NOTE: For unknown reasons, if we allocate/free multiple times, the system gets hardfault
  // Therefore, I use a global buffer and skip using dynamic allocation for now
  //if(NULL != Dsp_Audio_Samples) fsl_free(Dsp_Audio_Samples);
  //Dsp_Audio_Samples = NULL;
  Dsp_Voice_Streaming_Timer = Dsp_Audio_Samples_Timer = INVALID_TIMER_ID;

  //Get D10 back to detection mode
  back_to_detection(g_chip1, TRIGGER);

}



static void d10_InterruptIsr(void *pArg) {
  int chip = 0;
  int interrupt_events, current_events_status;
#if OPTIMIZE_LEVEL == 0
  Debug_Printf("GOT EVENT INTERRUPT\n");
#endif
  if (get_powerup_flag(chip) == FLAG_DISABLE) { // too early event
    Debug_Printf("Interrupt too early\n");
    return;
  }
#if OPTIMIZE_LEVEL == 0
  ms_delay(10);
#endif
  interrupt_events = read_register(chip, DETECTION_AND_SYSTEM_EVENTS_STATUS_14);
  current_events_status = interrupt_events;
  if ((interrupt_events == 0) && (s_state == 5)){
    current_events_status = 2;
  }
  while (current_events_status != 0){
    if (current_events_status & (1 << 15)){
      check_fw_errors (chip);
      current_events_status = current_events_status & (~ERROR_EVENT);
      Debug_Printf("ERROR EVENT\n");
      break;
    }else if (current_events_status & (1 << 14)){
      check_fw_errors (chip);
      current_events_status = current_events_status & (~WARNING_EVENT);
      Debug_Printf("WARNING EVENT\n");
    }else if (current_events_status & (1 << 0)){
      set_powerup_flag(chip, FLAG_SET);
      Debug_Printf("FW POWER UP COMPLETED\n");
      current_events_status = current_events_status & (~PWRUP_COMP);
    }else if (current_events_status & (1 << 4)){
      Debug_Printf("AEP EVENT\n");
      current_events_status = current_events_status & (~AEP_DET);
    }else{
      //use_case_wakeup_vt_vc_trigger(chip);
      ExtDspMgr_SignalEvent(EXT_DSP_VT_EVT);
      current_events_status = current_events_status & (~current_events_status);
    }
  }
  // clean events in FW
  write_register(chip, DETECTION_AND_SYSTEM_EVENTS_STATUS_14, interrupt_events);
#if OPTIMIZE_LEVEL == 0
  delay_if_not_ready(chip, 20);
#endif
}

static void d10_InterruptReadyIsr(void *pArg) {
  set_ready_flag(0, FLAG_SET);
}


static void extDspMgr_VoiceStreamTimeout(void) {
  ExtDspMgr_SignalEvent(EXT_DSP_STOP_STREAM_VOICE_EVT);
}

static void extDspMgr_VoiceSamplesTimeout(void) {
  System_StopHwTimer(Dsp_Audio_Samples_Timer);
  ExtDspMgr_SignalEvent(EXT_DSP_READ_AUDIO_FRAME_EVT);
}