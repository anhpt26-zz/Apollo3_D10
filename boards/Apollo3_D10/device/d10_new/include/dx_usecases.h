/**
 * dx_usecases.h  --  usecases interface common functions
 *
 * Copyright (C) 2016 DSP Group
 * All rights reserved.
 *
 * Unauthorized copying of this file is strictly prohibited.
 */

#ifndef _DX_USECASES_H
#define _DX_USECASES_H

#include <stdint.h>

int init_config_chip(int chip);

int use_case_Sensing_Enter(int chip, int wwe);
int use_case_Sensing_Exit(int chip);
int use_case_Dos_Modem_Enter(int chip);
int use_case_Dos_Modem_Exit(int chip);

int use_case_reg_table_only_enter(int chip);
int use_case_reg_table_only_exit(int chip);

int hook_reg_setting(char* filename);

#endif /* _DX_USECASES_H */
