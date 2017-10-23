/**
 ****************************************************************************************
 *
 * @file demo_timer0.c
 *
 * @brief Timer0 demo (hw_timer0 driver)
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

#ifndef __TIMER0_H__
#define __TIMER0_H__

#include <stdbool.h>

/*
 * Use LED for low power indication, (short blink once in a while)
 */
void timer0_slow_blink_func(void);


void timer0_blink_led_dim_func(uint8_t brightness, bool checked);

void timer0_light_led_func(uint8_t brightness);


void timer0_turn_off_func(void);


#endif
