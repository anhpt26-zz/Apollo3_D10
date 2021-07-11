/************************************************************************************************************
 Module:       xxxx.c
 
 Description:  Contains debug utility functions
 
 Notes:        ---
 
 History:
 Date          Name      Changes
 -----------   ----      -------------------------------------------------------------------------------------
 3/18/2018     xxx       Began coding

 ************************************************************************************************************/

#include "am_util.h"
#include "board_def.h"
#include "debug.h"
#include "main.h"

//###########################################################################################################
//      CONSTANT DEFINITION
//###########################################################################################################


//###########################################################################################################
//      TYPEDEF DEFINITION
//###########################################################################################################



//###########################################################################################################
//      Module Level Variables
//###########################################################################################################


//###########################################################################################################
//      PUBLIC FUNCTION DEFINITION
//###########################################################################################################
void Debug_Init() {
    //Init IO pins for debugging
    am_hal_gpio_pinconfig(ABQ_DEBUG_1_PIN,g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_pinconfig(ABQ_DEBUG_2_PIN,g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_pinconfig(ABQ_DEBUG_3_PIN,g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_output_clear(ABQ_DEBUG_1_PIN);
    am_hal_gpio_output_clear(ABQ_DEBUG_2_PIN);
    am_hal_gpio_output_clear(ABQ_DEBUG_3_PIN);
}

void Debug_SetDebugPin1(uint8_t val) {
  if(val) am_hal_gpio_output_set(ABQ_DEBUG_1_PIN);
  else am_hal_gpio_output_clear(ABQ_DEBUG_1_PIN);
}
void Debug_SetDebugPin2(uint8_t val) {
  if(val) am_hal_gpio_output_set(ABQ_DEBUG_2_PIN);
  else am_hal_gpio_output_clear(ABQ_DEBUG_2_PIN);
}
void Debug_SetDebugPin3(uint8_t val) {
  if(val) am_hal_gpio_output_set(ABQ_DEBUG_3_PIN);
  else am_hal_gpio_output_clear(ABQ_DEBUG_3_PIN);
}
//###########################################################################################################
//      PRIVATE FUNCTION DEFINITION
//###########################################################################################################