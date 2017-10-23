/**
 ****************************************************************************************
 *
 * @file main_task.c
 *
 * @brief Main task of the peripherals demo application
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

#include <string.h>
#include <stdio.h>
#include <osal.h>
#include <resmgmt.h>
#include "app/menu.h"
#include "app/tasklist.h"
#include "platform_devices.h"
#include "serial_console.h"

static inline void readline(char *buf, size_t size)
{
        int c;

        do {
                c = getchar();
                *buf = c;
                buf++;
                size--;
        } while (c != '\n' && c != '\r' && size > 1); // wait for CR/LF or reserve 1 char for \0

        /* make sure it's null-terminated */
        *buf = '\0';
}

void main_task_func(void *param)
{
        char s[64];

        resource_init();

        console_init(SERIAL1, 256);

        app_tasklist_create();

        for(;;) {
                app_menu_draw();

                readline(s, sizeof(s));

                app_menu_parse_selection(s);
        }
}
