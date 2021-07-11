#ifndef DEBUG_H
#define DEBUG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "am_util_stdio.h"

#define Debug_Printf(format, ...)  am_util_stdio_printf(format, ##__VA_ARGS__)

void Debug_Init();
void Debug_SetDebugPin1(uint8_t val);
void Debug_SetDebugPin2(uint8_t val);
void Debug_SetDebugPin3(uint8_t val);



#ifdef __cplusplus
}
#endif
#endif    //DEBUG_H