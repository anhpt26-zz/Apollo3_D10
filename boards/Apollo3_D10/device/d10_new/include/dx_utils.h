/**
 * dx_utils.h  --  DBMDX interface common functions
 *
 * Copyright (C) 2016 DSP Group
 * All rights reserved.
 *
 * Unauthorized copying of this file is strictly prohibited.
 */

#ifndef _DX_UTILS_H
#define _DX_UTILS_H

#include <stdint.h>

void pabort(const char *s);
unsigned int atoh__(const char *String);
unsigned int atoi__(const char *String);

void ms_delay(unsigned int msecond);

void wait_ready_or_delay(int chip, unsigned int msecond);
void delay_if_not_ready(int chip, unsigned int msecond);

void wait_powerup_or_delay(int chip, unsigned int msecond);

typedef enum  {
	FLAG_DISABLE,
	FLAG_ENABLE,
	FLAG_SET,
    FLAG_CLEAR
}flag_states;

void set_powerup_flag(int chip, int value);
int  get_powerup_flag(int chip);

void set_ready_flag(int chip, int value);

void rw_lock_aquire();
void rw_lock_release();
void rw_lock_create();
void rw_lock_destroy();

int div_and_ceil(int n, int d);


#endif /* _DX_UTILS_H */
