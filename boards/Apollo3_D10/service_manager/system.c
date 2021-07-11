/************************************************************************************************************
 Module:       rtos.c
 
 Description:  Contains RTOS utility functions
 
 Notes:        ---
 
 History:
 Date          Name      Changes
 -----------   ----      -------------------------------------------------------------------------------------
 1/3/2021     XX       Began coding

 ************************************************************************************************************/

#include "debug.h"
#include "main.h"
#include "board_def.h"
#include "system.h"
#include "am_mcu_apollo.h"
//###########################################################################################################
//      CONSTANT DEFINITION
//###########################################################################################################

//###########################################################################################################
//      TYPEDEF DEFINITION
//###########################################################################################################
typedef struct
{
  bool is_used;
  sys_timer_cb_t cb;
} sys_timer_t;

//###########################################################################################################
//      Module Level Variables
//###########################################################################################################
sys_timer_cb_t Timer_Cb_List[MAX_TIMER_COUNT];
uint8_t Apollo4_Timer_List[MAX_TIMER_COUNT] = {0};
sys_timer_t Sys_Hw_Timer_List[MAX_TIMER_COUNT];
static bool System_Allow_Enable_GPIO_IRQ = false;
static bool System_Allow_Enable_Timer_IRQ = false;
QueueHandle_t System_Evt_Handle = NULL;
//###########################################################################################################
//      PRIVATE FUNCTION DECLARATION
//###########################################################################################################
static void System_button0ISR(void);
//###########################################################################################################
//      PUBLIC FUNCTION DEFINITION
//###########################################################################################################
int System_Init(void)
{
  System_Allow_Enable_GPIO_IRQ = false;
  System_Allow_Enable_Timer_IRQ = false;
  for (int i = 0; i < MAX_TIMER_COUNT; i++)
  {
    Sys_Hw_Timer_List[i].is_used = false;
  }

  return 0;
}


void System_Task(void *pvParameter)
{
  uint8_t event;
  uint8_t twm_timer_id;

  System_Evt_Handle = xQueueCreate(SYS_SIGNAL_EVT_MAX, 1);
  if (System_Evt_Handle == NULL)
  {
    Debug_Printf("Failed to create System Event Handle\n");
    return;
  }

  do
  {
    xQueueReceive(System_Evt_Handle, &event, portMAX_DELAY);
    switch (event)
    {
      case SYS_SIGNAL_EVT_EXT_DSP_RDY:
      break;
    default:
      break;
    }
  } while (1);
}


int System_RegIRQPin(uint64_t pin_no, 
                     uint32_t drv_strength, 
                     uint32_t int_dir,
                     void (*irs_cb)(void))
{
  am_hal_gpio_pincfg_t gpio_int_cfg  =
  {
    .uFuncSel       = 3,
    .eDriveStrength = drv_strength,
    .eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE,
    .eIntDir        = int_dir,
    .eGPOutcfg      = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .ePullup        = AM_HAL_GPIO_PIN_PULLUP_NONE,
  };

  System_Allow_Enable_GPIO_IRQ = true;

  am_hal_gpio_pinconfig(pin_no, gpio_int_cfg);

  AM_HAL_GPIO_MASKCREATE(GpioIntMask);
  // Set up the host IO interrupt
  am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, pin_no));
  // Register handler for IOS => IOM interrupt
  am_hal_gpio_interrupt_register(pin_no, irs_cb);
  am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, pin_no));
}

void System_EnableNVICIRQPin(void)
{
  if(System_Allow_Enable_GPIO_IRQ) 
  {
    //Set Priority for GPIO NVIC IRQ
    NVIC_SetPriority(GPIO_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);    
    //Enable NVIC IRQ for GPIO
    NVIC_EnableIRQ(GPIO_IRQn);
  }
}

uint8_t System_RegisterHWTimer(uint32_t timeout_ms, sys_timer_cb_t isr_cb)
{
  uint8_t timer_id = INVALID_TIMER_ID;
  uint32_t ret;
  int i;
  //Look for a free HW timer
  for (timer_id = 0; timer_id < MAX_TIMER_COUNT; timer_id++)
  {
    if (Sys_Hw_Timer_List[timer_id].is_used == false)
      break;
  }
  if (timer_id < MAX_TIMER_COUNT)
  {
    System_Allow_Enable_Timer_IRQ = true;
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);
    am_hal_ctimer_config_t timer_cfg = 
    {
      .ui32Link = 0,
      .ui32TimerAConfig = AM_HAL_CTIMER_FN_ONCE |     //Single Count
                          AM_HAL_CTIMER_INT_ENABLE |  //Enable Interrupt when timer count
                          AM_HAL_CTIMER_XT_2_048KHZ,  //Set input XT clock source is 48kHz
      .ui32TimerBConfig = 0,
    };
    am_hal_ctimer_clear(timer_id, AM_HAL_CTIMER_TIMERA);
    am_hal_ctimer_config(timer_id, &timer_cfg);

    timeout_ms = timeout_ms;// * 48;
    am_hal_ctimer_period_set(timer_id, AM_HAL_CTIMER_TIMERA, timeout_ms,
                             (timeout_ms >> 1));
    am_hal_ctimer_int_enable(((uint32_t)0x01) << ((uint32_t)(timer_id * 2)));
    Sys_Hw_Timer_List[timer_id].cb = isr_cb; //record the callback
    Sys_Hw_Timer_List[timer_id].is_used = true;
  }
  else 
  {
    timer_id = INVALID_TIMER_ID;
  }

  return timer_id;
}

void System_EnableNVICTimer(void)
{
  if(System_Allow_Enable_Timer_IRQ) 
  {
    //Set Priority for CTIMER NVIC IRQ
    NVIC_SetPriority(CTIMER_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);
    //Enable CTIMER NVIC IRQ
    NVIC_EnableIRQ(CTIMER_IRQn);
  }
}

void System_UnRegisterHwTimer(uint8_t timer_id)
{
  if (timer_id < MAX_TIMER_COUNT && Sys_Hw_Timer_List[timer_id].is_used)
  {
    System_StopHwTimer(timer_id);
    Sys_Hw_Timer_List[timer_id].is_used = false;
    Sys_Hw_Timer_List[timer_id].cb = NULL;
  }
}

int System_SetHWTimerTimeout(uint8_t timer_id, uint32_t timeout_ms)
{
  if (timer_id < MAX_TIMER_COUNT && Sys_Hw_Timer_List[timer_id].is_used)
  {
    //Force timer stops
    am_hal_ctimer_stop(timer_id, AM_HAL_CTIMER_TIMERA);
    //Re-Configure corresponding HW timer
    am_hal_ctimer_config_t TimerConfig = 
    {
      .ui32Link = 0,
      //Only use Timer config A
      .ui32TimerAConfig = AM_HAL_CTIMER_FN_ONCE |     //Single Count
                          AM_HAL_CTIMER_INT_ENABLE |  //Enable Interrupt when timer count
                          AM_HAL_CTIMER_XT_2_048KHZ,  //Set input XT clock source is 48kHz
      .ui32TimerBConfig = 0,
    };
    am_hal_ctimer_clear(timer_id, AM_HAL_CTIMER_TIMERA);
    am_hal_ctimer_config(timer_id, &TimerConfig);

    timeout_ms = timeout_ms * 48;
    am_hal_ctimer_period_set(timer_id, AM_HAL_CTIMER_TIMERA, timeout_ms,
                             (timeout_ms >> 1));
    return 0;
  }
  else
  {
    return -1;
  }
}

int System_StartHwTimer(uint8_t timer_id)
{
  if ((timer_id < MAX_TIMER_COUNT) && (Sys_Hw_Timer_List[timer_id].is_used))
  {
    am_hal_ctimer_clear(timer_id, AM_HAL_CTIMER_TIMERA);
    am_hal_ctimer_start(timer_id, AM_HAL_CTIMER_TIMERA);
    return 0;
  }
  else
  {
    return -1;
  }
}

int System_ResumeHwTimer(uint8_t timer_id)
{
  if ((timer_id < MAX_TIMER_COUNT) && (Sys_Hw_Timer_List[timer_id].is_used))
  {
    //Just retart timer
    am_hal_ctimer_start(timer_id, AM_HAL_CTIMER_TIMERA);    
    return 0;
  }
  else
  {
    return -1;
  }
}

int System_StopHwTimer(uint8_t timer_id)
{
  if ((timer_id < MAX_TIMER_COUNT) && (Sys_Hw_Timer_List[timer_id].is_used))
  {
    am_hal_ctimer_stop(timer_id, AM_HAL_CTIMER_TIMERA);
    return 0;
  }
  else
  {
    return -1;
  }
}

int System_SignalEvt(QueueHandle_t queue_handle, uint8_t event)
{
  BaseType_t higher_priority_task_woken;

  BaseType_t ret = xQueueSendFromISR(queue_handle, (void *)&event, &higher_priority_task_woken);
  if (ret != pdTRUE)
  {
    //Debug_Printf("Failed to signal event \n");
    return -1;
  }

  //a higher priority task need to wake up, should give yield now
  if (ret == pdTRUE && higher_priority_task_woken == pdTRUE) 
  {
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
  return 0;
}

//###########################################################################################################
//      PRIVATE FUNCTION DEFINITION
//###########################################################################################################

void am_gpio_isr(void)
{
  AM_HAL_GPIO_MASKCREATE(GpioIntStatusMask);
  am_hal_gpio_interrupt_status_get(false, pGpioIntStatusMask);
  am_hal_gpio_interrupt_clear(pGpioIntStatusMask);
  am_hal_gpio_interrupt_service(pGpioIntStatusMask);
}

void am_ctimer_isr(void)
{
  uint32_t ctimer_status;
  uint8_t idx;
  ctimer_status = am_hal_ctimer_int_status_get(true);
  for(idx = 0; idx < MAX_TIMER_COUNT; idx++)
  {
    if(Sys_Hw_Timer_List[idx].is_used)
    {
      if((ctimer_status >> (idx * 2)))
      {
        Sys_Hw_Timer_List[idx].cb();
      }
    }
  }
  am_hal_ctimer_int_clear(ctimer_status);
} // am_ctimer_isr()