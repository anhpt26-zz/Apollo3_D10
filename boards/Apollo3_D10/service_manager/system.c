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
//###########################################################################################################
//      PRIVATE FUNCTION DECLARATION
//###########################################################################################################
static void System_button0ISR(void);
//###########################################################################################################
//      PUBLIC FUNCTION DEFINITION
//###########################################################################################################
int System_Init(void)
{
  for (int i = 0; i < MAX_TIMER_COUNT; i++)
  {
    Sys_Hw_Timer_List[i].is_used = false;
  }

  return 0;
}

uint8_t System_RegisterHWTimer(uint32_t timeout_ms, sys_timer_cb_t isr_cb)
{
  uint8_t timer_id = INVALID_TIMER_ID;
  uint32_t ret;
  int i;
  //Look for a free HW timer
  for (i = 0; i < MAX_TIMER_COUNT; i++)
  {
#if APOLLO4_REVB1
    //These HW timers are broken in RevB1, check internal_timer_config() to confirm
    if(i == 1 || i == 3 || i > 9) continue;   
#endif
    if (Sys_Hw_Timer_List[i].is_used == false)
      break;
  }
  if (i < MAX_TIMER_COUNT)
  {
    timer_id = i;
    am_hal_clkgen_clkout_enable(true, AM_HAL_CLKGEN_CLKOUT_LFRC);
    //Configure corresponding HW timer
    am_hal_timer_config_t TimerConfig;
    am_hal_timer_default_config_set(&TimerConfig);
    TimerConfig.eFunction = AM_HAL_TIMER_FN_UPCOUNT;
    TimerConfig.eInputClock = AM_HAL_TIMER_CLOCK_LFRC; //LRFC freq = 1024Hz
    TimerConfig.ui32PatternLimit = 0;
    TimerConfig.ui32Compare0 = timeout_ms;
    if(am_hal_timer_config(timer_id, &TimerConfig) != 0) {
      Debug_Printf("Failed to configure HW Timer %d\n", timer_id);
      return INVALID_TIMER_ID;
    } else {
      //Configure Timer Interrupt
      //am_hal_timer_clear(timer_id);   //It auto re-enable timer
      am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(timer_id, AM_HAL_TIMER_COMPARE0));
      am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(timer_id, AM_HAL_TIMER_COMPARE0));
      NVIC_EnableIRQ(TIMER0_IRQn + timer_id);
      NVIC_SetPriority(TIMER0_IRQn + timer_id, 0x4);
      Sys_Hw_Timer_List[i].cb = isr_cb; //record the callback
      Sys_Hw_Timer_List[i].is_used = true;
    }
  }

  return timer_id;
}


uint8_t System_RegisterHWTimerUs(uint32_t timeout_us, sys_timer_cb_t isr_cb)
{
  uint8_t timer_id = INVALID_TIMER_ID;
  int i;
  //Look for a free HW timer
  for (i = 0; i < MAX_TIMER_COUNT; i++)
  {
#if APOLLO4_REVB1
    //These HW timers are broken in RevB1, check internal_timer_config() to confirm
    if(i == 1 || i == 3 || i > 9) continue;   
#endif    
    if (Sys_Hw_Timer_List[i].is_used == false)
      break;
  }
  if (i < MAX_TIMER_COUNT)
  {
    timer_id = i;
    //Configure corresponding HW timer
    am_hal_timer_config_t TimerConfig;
    am_hal_timer_default_config_set(&TimerConfig);
    TimerConfig.eFunction = AM_HAL_TIMER_FN_UPCOUNT;
    TimerConfig.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV16; //6MHz freq
    TimerConfig.ui32PatternLimit = 0;
    TimerConfig.ui32Compare0 = timeout_us*6;
    if(am_hal_timer_config(timer_id, &TimerConfig) != 0) {
      Debug_Printf("Failed to configure HW Timer %d\n", timer_id);
      return INVALID_TIMER_ID;
    } else {    
      //Configure Timer Interrupt
      //am_hal_timer_clear(timer_id);   //It auto re-enable timer
      am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(timer_id, AM_HAL_TIMER_COMPARE0));
      am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(timer_id, AM_HAL_TIMER_COMPARE0));
      NVIC_SetPriority((IRQn_Type)((uint32_t)TIMER0_IRQn + timer_id), 0x04);
      NVIC_EnableIRQ((IRQn_Type)((uint32_t)TIMER0_IRQn + timer_id));
      Sys_Hw_Timer_List[i].cb = isr_cb; //record the callback
      Sys_Hw_Timer_List[i].is_used = true;
    }
  }

  return timer_id;
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
    am_hal_timer_stop(timer_id);
    //Re-Configure corresponding HW timer
    am_hal_timer_config_t TimerConfig;
    am_hal_timer_default_config_set(&TimerConfig);
    TimerConfig.eFunction = AM_HAL_TIMER_FN_UPCOUNT;
    TimerConfig.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV16;
    TimerConfig.ui32PatternLimit = 0;
    TimerConfig.ui32Compare0 = timeout_ms * 6000; // Default Clock is HFRC/16 or 6MHz.  100 msec.
    am_hal_timer_config(timer_id, &TimerConfig);
    return 0;
  }
  else
  {
    return -1;
  }
}

int System_StartHwTimer(uint8_t timer_id)
{
  if (timer_id < MAX_TIMER_COUNT && Sys_Hw_Timer_List[timer_id].is_used)
  {
    //Clear the counter and retstart the timer
    am_hal_timer_clear(timer_id);
    am_hal_timer_start(timer_id);
    return 0;
  }
  else
  {
    return -1;
  }
}

int System_ResumeHwTimer(uint8_t timer_id)
{
  if (timer_id < MAX_TIMER_COUNT && Sys_Hw_Timer_List[timer_id].is_used)
  {
    //Just retart timer
    am_hal_timer_start(timer_id);
    return 0;
  }
  else
  {
    return -1;
  }
}

int System_StopHwTimer(uint8_t timer_id)
{
  if (timer_id < MAX_TIMER_COUNT && Sys_Hw_Timer_List[timer_id].is_used)
  {
    am_hal_timer_stop(timer_id);
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
  if (ret == pdTRUE && higher_priority_task_woken == pdTRUE) //a higher priority task need to wake up, should give yield now
  {
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
  return 0;
}

//###########################################################################################################
//      PRIVATE FUNCTION DEFINITION
//###########################################################################################################

void sys_handle_gpio_isr(int irq_num)
{
  uint32_t ui32IntStatus;

  am_hal_gpio_interrupt_irq_status_get(irq_num, false, &ui32IntStatus);
  am_hal_gpio_interrupt_irq_clear(irq_num, ui32IntStatus);
  am_hal_gpio_interrupt_service(irq_num, ui32IntStatus);
}

void sys_handle_hw_timer_isr(int timer_num)
{
  am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(timer_num, AM_HAL_TIMER_COMPARE0));
  if (Sys_Hw_Timer_List[timer_num].is_used)
  {
    am_hal_timer_clear(timer_num);
    Sys_Hw_Timer_List[timer_num].cb();
  }
  else
  {
    am_hal_timer_stop(timer_num);
  }
}

void am_gpio0_001f_isr(void)
{
  sys_handle_gpio_isr(GPIO0_001F_IRQn);
}

void am_gpio0_203f_isr(void)
{
  sys_handle_gpio_isr(GPIO0_203F_IRQn);
}
void am_gpio0_405f_isr(void)
{
  sys_handle_gpio_isr(GPIO0_405F_IRQn);
}
void am_gpio0_607f_isr(void)
{
  sys_handle_gpio_isr(GPIO0_607F_IRQn);
}
void am_gpio1_001f_isr(void)
{
  sys_handle_gpio_isr(GPIO1_001F_IRQn);
}
void am_gpio1_203f_isr(void)
{
  sys_handle_gpio_isr(GPIO1_203F_IRQn);
}
void am_gpio1_405f_isr(void)
{
  sys_handle_gpio_isr(GPIO1_405F_IRQn);
}
void am_gpio1_607f_isr(void)
{
  sys_handle_gpio_isr(GPIO1_607F_IRQn);
}

void am_timer00_isr(void)
{
  sys_handle_hw_timer_isr(0);
}
void am_timer01_isr(void)
{
  sys_handle_hw_timer_isr(1);
}
void am_timer02_isr(void)
{
  sys_handle_hw_timer_isr(2);
}
void am_timer03_isr(void)
{
  sys_handle_hw_timer_isr(3);
}
void am_timer04_isr(void)
{
  sys_handle_hw_timer_isr(4);
}
void am_timer05_isr(void)
{
  sys_handle_hw_timer_isr(5);
}
void am_timer06_isr(void)
{
  sys_handle_hw_timer_isr(6);
}
void am_timer07_isr(void)
{
  sys_handle_hw_timer_isr(7);
}
void am_timer08_isr(void)
{
  sys_handle_hw_timer_isr(8);
}
void am_timer09_isr(void)
{
  sys_handle_hw_timer_isr(9);
}
void am_timer10_isr(void)
{
  sys_handle_hw_timer_isr(10);
}
void am_timer11_isr(void)
{
  sys_handle_hw_timer_isr(11);
}
void am_timer12_isr(void)
{
  sys_handle_hw_timer_isr(12);
}
void am_timer13_isr(void)
{
  sys_handle_hw_timer_isr(13);
}
void am_timer14_isr(void)
{
  sys_handle_hw_timer_isr(14);
}
void am_timer15_isr(void)
{
  sys_handle_hw_timer_isr(15);
}
