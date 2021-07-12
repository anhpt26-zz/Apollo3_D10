/************************************************************************************************************
 Module:       xxxx.c
 
 Description:  This is the main file that will initialize all the modules
 
 Notes:        ---
 
 History:
 Date          Name      Changes
 -----------   ----      -------------------------------------------------------------------------------------
 3/18/2018     XXX       Began coding

 ************************************************************************************************************/
#include "debug.h"
#include "main.h"
#include "board_def.h"
#include "system.h"
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "test_task.h"
//###########################################################################################################
//      CONSTANT DEFINITION
//###########################################################################################################


//###########################################################################################################
//      TYPEDEF DEFINITION
//###########################################################################################################



//###########################################################################################################
//      Module Level Variables
//###########################################################################################################
QueueHandle_t TestTask_Evt_Handler = NULL;
uint32_t Hw_Timer_Id = INVALID_TIMER_ID;
//###########################################################################################################
//      PRIVATE FUNCTION DECLARATION
//###########################################################################################################
int TestTask_SignalEvt(testtask_signal_evt_t evt);
static void TestTask_IRQGpio(void);
static void TestTask_IRQHwTimer(void);

//###########################################################################################################
//      PUBLIC FUNCTION DEFINITION
//###########################################################################################################

void TestTask_Init(void)
{
  System_RegIRQPin(ABQ_TEST_TASK_PIN_1, 
                   AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA, 
                   AM_HAL_GPIO_PIN_INTDIR_BOTH, 
                   TestTask_IRQGpio);
  am_hal_gpio_pinconfig(ABQ_TEST_TASK_PIN_2, g_AM_HAL_GPIO_OUTPUT);

  Hw_Timer_Id = System_RegisterHWTimer(10, TestTask_IRQHwTimer);
  am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);
}

void TestTask_Task(void *pvParameter)
{
  uint8_t event;
  TestTask_Evt_Handler = xQueueCreate(TESTTASK_SIGNAL_EVT_MAX, 1);
  if(!TestTask_Evt_Handler)
  {
    Debug_Printf("%s: Failed to create Signal Event Handler\r\n", __func__);
    return;
  }

  if(Hw_Timer_Id == INVALID_TIMER_ID)
  {
    vTaskDelay(portMAX_DELAY);
  }
  System_StartHwTimer(Hw_Timer_Id);

  while(1)
  {
    xQueueReceive(TestTask_Evt_Handler, &event, portMAX_DELAY);
    switch (event)
    {
      case TESTTASK_SIGNAL_EVT_HW_TIMEOUT:
        {
          am_hal_gpio_output_toggle(ABQ_TEST_TASK_PIN_2);
        }
        break;

      case TESTTASK_SIGNAL_EVT_PIN_TOGGLE:
        { 
          am_devices_led_toggle(am_bsp_psLEDs, 2);
          am_devices_led_toggle(am_bsp_psLEDs, 3);
        }
        break;

      default:
        break;
    }
  }
}

//###########################################################################################################
//      PRIVATE FUNCTION DEFINITION
//###########################################################################################################
int TestTask_SignalEvt(testtask_signal_evt_t evt)
{
  if(TestTask_Evt_Handler)
  {
     return System_SignalEvt(TestTask_Evt_Handler, evt);
  }
  Debug_Printf("%s: Handler Is NULL\r\n", __func__);
  return -1;
}

static void TestTask_IRQGpio(void)
{
  TestTask_SignalEvt(TESTTASK_SIGNAL_EVT_PIN_TOGGLE);
}

static void TestTask_IRQHwTimer(void)
{
  System_StartHwTimer(Hw_Timer_Id);
  TestTask_SignalEvt(TESTTASK_SIGNAL_EVT_HW_TIMEOUT);
}