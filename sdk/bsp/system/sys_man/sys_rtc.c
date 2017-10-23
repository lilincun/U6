/**
\addtogroup BSP
\{
\addtogroup SYSTEM
\{
\addtogroup Real_Time_Clock
\{
*/

/**
 ****************************************************************************************
 *
 * @file sys_rtc.c
 *
 * @brief RTC Driver
 *
 * Copyright (C) 2015. Dialog Semiconductor, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor. All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#include <stdint.h>
#include "sys_rtc.h"
#include "hw_timer1.h"
#include "FreeRTOS.h"


/*
 * Global and / or retained variables
 */

PRIVILEGED_DATA static uint64_t rtc_time;               // counts in LP cycles
PRIVILEGED_DATA static uint32_t rtc_previous_time;      // counts in LP cycles


/*
 * Local variables
 */



/*
 * Forward declarations
 */



/*
 * Function definitions
 */

uint64_t rtc_get(void)
{
        uint32_t lp_prescaled_time;
        uint32_t lp_current_time;

        vPortEnterCritical();

        HW_TIMER1_GET_INSTANT(lp_prescaled_time, lp_current_time);
        rtc_time += (lp_current_time - rtc_previous_time) & LP_CNT_NATIVE_MASK;
        rtc_previous_time = lp_current_time;

        vPortExitCritical();

        return rtc_time;
}


uint64_t rtc_get_fromISR(void)
{
        uint32_t ulPreviousMask;
        uint32_t lp_prescaled_time;
        uint32_t lp_current_time;

        ulPreviousMask = portSET_INTERRUPT_MASK_FROM_ISR();

        HW_TIMER1_GET_INSTANT(lp_prescaled_time, lp_current_time);
        rtc_time += (lp_current_time - rtc_previous_time) & LP_CNT_NATIVE_MASK;
        rtc_previous_time = lp_current_time;

        portCLEAR_INTERRUPT_MASK_FROM_ISR( ulPreviousMask );

        return rtc_time;
}


__RETAINED_CODE uint64_t rtc_get_fromCPM(uint32_t *lp_prescaled_time, uint32_t *lp_current_time)
{
        HW_TIMER1_GET_INSTANT(*lp_prescaled_time, *lp_current_time);
        rtc_time += (*lp_current_time - rtc_previous_time) & LP_CNT_NATIVE_MASK;
        rtc_previous_time = *lp_current_time;

        return rtc_time;
}



/**
\}
\}
\}
*/
