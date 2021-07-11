//*****************************************************************************
//
//! @file freertos_fit.h
//!
//! @brief Global includes for the freertos_fit app.
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

#ifndef _MAIN_H_
#define _MAIN_H_

//*****************************************************************************
//
// Required built-ins.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

//*****************************************************************************
//
// Standard AmbiqSuite includes.
//
//*****************************************************************************
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// FreeRTOS include files.
//
//*****************************************************************************
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
//*****************************************************************************
//
// Definitions
//
//*****************************************************************************
#define ENABLE_DEBUG_PRINTF     (0)
#define ENABLE_DEBUG_FEATURE    (1)
#define ENABLE_EXTERNAL_DPS     (0)
#define ENABLE_TEST_TASK        (1)

/*
  NOTICE: Max Priority in freeRtos config is 7
*/
//Define configuration for System Task
#define SYS_TASK_STACK_SIZE 256
#define SYS_TASK_PRIORITY 5

//Define configuration for External DSP Task
#if ENABLE_EXTERNAL_DPS
#define EXTERNAL_DSP_STACK_SIZE   (512)
#define EXT_DSP_TASK_PRIORITY     (4)
#endif

#if ENABLE_TEST_TASK
#define TEST_TASK_STACK_SIZE        (512)
#define TEST_TASK_TASK_PRIORITY     (5) 
#endif

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void disable_print_interface(void);


#endif // _MAIN_H_
