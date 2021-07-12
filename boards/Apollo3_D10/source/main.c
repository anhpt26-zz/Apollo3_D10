//*****************************************************************************
//
//! @file freertos_lowpower.c
//!
//! @brief Example of the app running under FreeRTOS.
//!
//! This example implements LED task within the FreeRTOS framework. It monitors
//! three On-board buttons, and toggles respective on-board LEDs in response.
//! To save power, this application is compiled without print
//! statements by default. To enable them, add the following project-level
//! macro definitions.
//!
//! AM_DEBUG_PRINTF
//!
//! If enabled, debug messages will be sent over ITM.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.5.1 of the AmbiqSuite Development Package.
//
//*****************************************************************************

//*****************************************************************************
//
// This application has a large number of common include files. For
// convenience, we'll collect them all together in a single header and include
// that everywhere.
//
//*****************************************************************************
#include "main.h"
#include "system.h"

#if ENABLE_DEBUG_FEATURE
#include "debug.h"
#endif

#if ENABLE_EXTERNAL_DPS
#include "ext_dsp_manager.h"
#endif

#if ENABLE_TEST_TASK
#include "test_task.h"
#endif

//*****************************************************************************
//
// Enable printing to the console.
//
//*****************************************************************************
void
enable_print_interface(void)
{
    //
    // Initialize a debug printing interface.
    //
    am_bsp_itm_printf_enable();
}

//*****************************************************************************
//
// Disable printing to the console.
//
//*****************************************************************************
void
disable_print_interface(void)
{
    //
    // Deinitialize a debug printing interface.
    //
    am_bsp_itm_printf_disable();
}

//*****************************************************************************
//
// Sleep function called from FreeRTOS IDLE task.
// Do necessary application specific Power down operations here
// Return 0 if this function also incorporates the WFI, else return value same
// as idleTime
//
//*****************************************************************************
uint32_t am_freertos_sleep(uint32_t idleTime)
{
    //This function make CPU going to deep sleep
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    return 0;
}

//*****************************************************************************
//
// Recovery function called from FreeRTOS IDLE task, after waking up from Sleep
// Do necessary 'wakeup' operations here, e.g. to power up/enable peripherals etc.
//
//*****************************************************************************
void am_freertos_wakeup(uint32_t idleTime)
{
    return;
}




//*****************************************************************************
//
// FreeRTOS debugging functions.
//
//*****************************************************************************
void
vApplicationMallocFailedHook(void)
{
    //
    // Called if a call to pvPortMalloc() fails because there is insufficient
    // free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    // internally by FreeRTOS API functions that create tasks, queues, software
    // timers, and semaphores.  The size of the FreeRTOS heap is set by the
    // configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h.
    //
    while (1);
}

void
vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void) pcTaskName;
    (void) pxTask;

    //
    // Run time stack overflow checking is performed if
    // configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    // function is called if a stack overflow is detected.
    //
    while (1)
    {
        __asm("BKPT #0\n") ; // Break into the debugger
    }
}

//*****************************************************************************
//
// Main Function
//
//*****************************************************************************
int
main(void)
{
   
    //
    // Set the clock frequency.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    am_bsp_itm_printf_enable();
#if ENABLE_DEBUG_PRINTF
    am_util_stdio_terminal_clear();
#endif
    Debug_Printf("Start Program\r\n");
    
    //
    // Enable Interrupts.
    //
    am_hal_interrupt_master_enable();
    
    //
    // Initialize system.
    //
    System_Init();

#if ENABLE_DEBUG_FEATURE
    Debug_Init();
#endif

#if ENABLE_EXTERNAL_DPS
    ExtDspMgr_Init();
#endif

#if ENABLE_TEST_TASK
    TestTask_Init();
#endif

    System_EnableNVICIRQPin();
    System_EnableNVICTimer();

    xTaskCreate(System_Task,         //Task function
              "system_task",       //Task name
              SYS_TASK_STACK_SIZE, //Stack size
              NULL,                //pvParameter
              SYS_TASK_PRIORITY,   //Priority
              NULL);

#if ENABLE_EXTERNAL_DPS
    xTaskCreate(ExtDspMgr_Task,         //Task function
            "External_DSP_D10",              //Task name
            EXTERNAL_DSP_STACK_SIZE, //Stack size
            NULL,                   //pvParameter
            EXT_DSP_TASK_PRIORITY,   //Priority
            NULL);
#endif

#if ENABLE_TEST_TASK
    xTaskCreate(TestTask_Task,         //Task function
            "Test_Task",              //Task name
            TEST_TASK_STACK_SIZE, //Stack size
            NULL,                   //pvParameter
            TEST_TASK_TASK_PRIORITY,   //Priority
            NULL);
#endif

    vTaskStartScheduler();
    while(1);
}

