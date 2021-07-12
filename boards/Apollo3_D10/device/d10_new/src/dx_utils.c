/*
 * dx_utils.c -- Some specific utils, some of them call osl functions
 *
 * Copyright (C) 2016 DSP Group
 * All rights reserved.
 *
 * Unauthorized copying of this file is strictly prohibited.
 */

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include "dx_params.h"
#include "dx_gen.h"
#include "dx_utils.h"

#include "ext_dsp_manager.h"
#include "debug.h"
#include "system.h"
#include "am_util.h"

void ms_delay(unsigned int msecond)
{
  // delay_ms(msecond);
  am_util_delay_ms(msecond);
}

//*****************************************************************************/

// each flag has 3 states: disabled / set / clear
// for now each flag is global and not per chip
// parameter "chip" of functions below may be used in multi processor system.

volatile int s_ready_flag   = FLAG_DISABLE;
volatile int s_powerup_flag = FLAG_DISABLE;



void set_ready_flag(int chip, int value)
{
	if (!READY_ACK) return;
	
	// ignore event if too early:
	if( s_ready_flag == FLAG_DISABLE){
		if (value == FLAG_ENABLE){
			s_ready_flag = FLAG_CLEAR;
			Debug_Printf("ready_flag_enabled!\n");
			//toggle_debug_gpio();				
		}	
		else {return;}	// ignore other values during disable
	}else{	// ready flag is enabled, it is CLEAR or SET
		if (value == FLAG_ENABLE) return; 	// ignore double enable
		s_ready_flag = value;

		if (value == FLAG_DISABLE){			
			Debug_Printf("ready_flag_disabled!\n");
			//toggle_debug_gpio();			
			//toggle_debug_gpio();			
		}	
	}
}

// The parameter "chip" may be used in multi processor system.
void wait_ready_or_delay(int chip, unsigned int msecond)
{
  int i = 0;
  int wait_max = 200;				// 10ms X 200, as sleep step is 10ms

  if ((READY_ACK) && (s_ready_flag != FLAG_DISABLE)){
    if (s_ready_flag == FLAG_SET){	// ready event already happened
      Debug_Printf("-------------------------ready already happened!\n");
      s_ready_flag = FLAG_CLEAR;
      return;
    } else {
      Debug_Printf("wait for ready !\n");
      while((s_ready_flag == FLAG_CLEAR) && (i<wait_max)){
        ms_delay(10);			// this wait may be implemented by a SW interrupt
        i++;
      }
      if (i == wait_max){
        Debug_Printf("!!!!!! NO READY ARRIVED !!!!!!! i=%d\n",i);
      }else{
        s_ready_flag = FLAG_CLEAR;
      }
    }
  }else{ // either no FW ready supported or ready not enabled
#if OPTIMIZE_LEVEL == 0
    Debug_Printf("Delay as no ready !\n");
#endif
    //[BEN]TODO: How to use vTaskDelay()??
#if OPTIMIZE_LEVEL > 1
    ms_delay(1);
#else
    ms_delay(msecond);
#endif
    //vTaskDelay(MS_TO_TICKS(msecond));
  }
}

// delay ONLY if not ready flag (if ready flag, delay is performed by a register setting)
void delay_if_not_ready(int chip, unsigned int msecond)
{
	if (!READY_ACK) {
		//Debug_Printf("delay_if_not_ready !!\n");
		ms_delay(msecond);
	}
}

//*****************************************************************************/

int get_powerup_flag(int chip)
{
	return (s_powerup_flag);
}

//*****************************************************************************/

void set_powerup_flag(int chip, int value)
{
	if (!EVENTS_TO_HOST) return;

	// ignore event if too early:
	if( s_powerup_flag == FLAG_DISABLE){
		if (value == FLAG_ENABLE){
			s_powerup_flag = FLAG_CLEAR;
			Debug_Printf("powerup_flag_enabled!\n");
		}
		else {return;}	// ignore other values during disable
	}else{
		if (value == FLAG_ENABLE) return; 	// ignore double enable
		s_powerup_flag = value;
	}
}

void wait_powerup_or_delay(int chip, unsigned int msecond)
{
	if ((EVENTS_TO_HOST) && (s_powerup_flag != FLAG_DISABLE)){
		if (s_powerup_flag == FLAG_SET){	// powerup event already happened
			Debug_Printf("-------------------------powerup already happened!\n");
			return;
		}
		//Debug_Printf("wait for power up !\n");
		while(s_powerup_flag == FLAG_CLEAR){
#if OPTIMIZE_LEVEL > 1
      ms_delay(2);
#else 
      ms_delay(10);				// this wait may be implemented by a SW interrupt
#endif
		}
	}else{ // either no FW events or not ready
		ms_delay(msecond);
	}
}


//*****************************************************************************/
//  FUNCTION NAME: rw_lock_aquire() rw_lock_release()
//
//
//  DESCRIPTION:
//		If configured in project parameters.h, lock transport (by a semaphore) during read / write registers (when 2 tasks may do it simultanously).
//
//*****************************************************************************/

#ifdef READ_WRITE_LOCK

#include "osl_cs.h"

HCS hCriticSec;

// lock the transport channel during a read/ write command.
// used in case different application threads can read/write in parallel.

void rw_lock_aquire()
{
	OSL_CS_Enter( hCriticSec );
}

void rw_lock_release()
{
	OSL_CS_Leave( hCriticSec );
}

void rw_lock_create()
{
	hCriticSec  = OSL_CS_Create( );
}

void rw_lock_destroy()
{
	OSL_CS_Close( hCriticSec );
}

//---------------------------
#else	// When no lock is configured

void rw_lock_aquire()  {}
void rw_lock_release() {}
void rw_lock_create()  {}
void rw_lock_destroy() {}

#endif
//*****************************************************************************/
//  FUNCTION NAME: atoh__(const char *String)
//
//  DESCRIPTION:
//		Convert Ascii to Hex.
//
//  PARAMETERS:
//      const char *String
//
//  RETURNS:
//      integer value
//*****************************************************************************/
unsigned int atoh__(const char *String)
{
    int Value = 0, Digit;
    char c;

    while ((c = *String++) != '\0')
    {
        if (c >= '0' && c <= '9')
            Digit = (uint) (c - '0');
        else if (c >= 'a' && c <= 'f')
            Digit = (uint) (c - 'a') + 10;
        else if (c >= 'A' && c <= 'F')
            Digit = (uint) (c - 'A') + 10;
        else  {
            break;
		}

        Value = (Value << 4) + Digit;
    }

    return Value;
}

//*****************************************************************************/
//  FUNCTION NAME: atoi__(const char *String)
//
//  DESCRIPTION:
//		Convert Ascci to integer.
//
//  PARAMETERS:
//      const char *String
//
//  RETURNS:
//      integer value
//*****************************************************************************/
unsigned int atoi__(const char *String)
{
    int Value = 0, Digit;
    char c;

    while ((c = *String++) != '\0')
    {
        if (c >= '0' && c <= '9')
            Digit = (uint) (c - '0');
        else  {
            break;
		}

        Value = Value *10 + Digit;
    }

    return Value;
}


/*****************************************************************************/

int isNumeric(const char *s)
{
	int i;
	char *p;

	if(s == NULL || *s == '\0' || isspace(*s))
		return 0;

	for(i = 0; i < strlen(s) - 1; i++) {
		if(isdigit(s[i]) == 0) {
			/* Debug_Printf("Not Digit!!!\n"); */
			return 0;
		}
	}

	return strtod(s, &p);
}

/*****************************************************************************/

long long current_timestamp()
{
  //TODO: Add support
  return 0;
#if 0
    struct timeval te;
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
    printf("milliseconds: %lld\n", milliseconds);
    return milliseconds;
#endif
}

/*****************************************************************************/

int div_and_ceil(int n, int d) 
{
    if(n%d == 0)
         return (n/d);
    else 
         return (n/d + 1);
}
