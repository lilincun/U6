/**
 ****************************************************************************************
 *
 * Copyright (C) 2015. Dialog Semiconductor, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#ifndef __UPDATA_H__
#define __UPDATA_H__
#include "stdint.h"

uint8_t updata_check_read(uint8_t section);

uint8_t  read_data_from_flash(uint8_t section,uint8_t *g_data_flash,uint16_t from_adder,uint16_t read_number);

uint8_t mark_the_section_has_read(uint8_t section);
#endif
