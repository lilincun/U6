/**
 * \addtogroup BSP
 * \{
 * \addtogroup SYSTEM
 * \{
 * \addtogroup REAL_TIME_CLOCK
 * 
 * \brief Real Time Clock
 *
 * \{
 */

/**
 ****************************************************************************************
 *
 * @file sys_rtc.h
 *
 * @brief RTC header file.
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

#ifndef RTC_H_
#define RTC_H_

#include <stdint.h>
#include "sdk_defs.h"

/**
 * \brief Update Real Time Clock value.
 *
 * \return The current RTC time. This is expressed in ticks of the clock that clocks TIMER1
 *         (e.g. XTAL32K). For example, if XTAL32K drives TIMER1, each RTC tick will be
 *         1000000 / 32768 = 30.5uS
 *
 * \warning This function is called only from OS Tasks.
 *
 */
uint64_t rtc_get(void);

/**
 * \brief Update Real Time Clock value.
 *
 * \return The current RTC time. This is expressed in ticks of the clock that clocks TIMER1
 *         (e.g. XTAL32K). For, example, if XTAL32K drives TIMER1, each RTC tick will be
 *         1000000 / 32768 = 30.5uS
 *
 * \warning This function is called only from Interrupt Context.
 *
 */
uint64_t rtc_get_fromISR(void);

/**
 * \brief Update Real Time Clock value and get current time (in prescaled LP cycles).
 *
 * \return The current RTC time.
 *
 * \warning This function is used only by the Clock and Power Manager.
 *
 */
__RETAINED_CODE uint64_t rtc_get_fromCPM(uint32_t *, uint32_t *);


#endif /* RTC_H_ */

/**
\}
\}
\}
*/
