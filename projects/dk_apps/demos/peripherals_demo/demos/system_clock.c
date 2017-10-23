/**
 ****************************************************************************************
 *
 * @file system_clock.c
 *
 * @brief Clock settings menu functions
 *
 * Copyright (C) 2016. Dialog Semiconductor, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#include <sys_clock_mgr.h>
#include <stdio.h>
#include "common.h"

void menu_system_clock_func(const struct menu_item *m, bool checked)
{
        sys_clk_t clk = (sys_clk_t) m->param;

        if (!cm_sys_clk_set(clk)) {
                printf("Switching to this clock is not allowed\r\n");
        }
}

bool menu_system_clock_checked_cb_func(const struct menu_item *m)
{
        sys_clk_t clk = (sys_clk_t) m->param;

        return cm_sys_clk_get() == clk;
}
