/**
 * \addtogroup BSP
 * \{
 * \addtogroup MIDDLEWARE
 * \{
 * \addtogroup CONSOLE
 *
 * \brief Serial console
 *
 * \{
 */

/**
 ****************************************************************************************
 *
 * @file console.h
 *
 * @brief Declarations for serial console service
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

#ifndef CONSOLE_H_
#define CONSOLE_H_

#if dg_configUSE_CONSOLE

#include "ad_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#if !dg_configUSE_CONSOLE_STUBS

/*
 * Serial console provides input and output through one of the UART's.
 * It uses UART adapter for reading and writing.
 * Unlike UART adapter functions, writes to serial port can be started in interrupt context, so
 * console can be used to allow prints from interrupts.
 * To allow writes from tasks and interrupts, console uses it's own task that does actual
 * UART hardware access through UART adapter.
 * Console does not use additional RAM for printing. RAM that is used is allocated during
 * initialization only.
 *
 * Passing data to print is done using circular buffer that is filled by console_write calls.
 * If data flow is too fast for UART, calls from task will wait, while calls from interrupt can
 * lose some data.
 *
 * If CONFIG_RETARGET is enabled, it will be automatically handled by console instead of standard
 * retarget implementation.
 */

/**
 * \brief Console task priority
 */
#ifndef CONSOLE_TASK_PRIORITY
#define CONSOLE_TASK_PRIORITY       (OS_TASK_PRIORITY_NORMAL)
#endif

/**
 * \brief Initialize console to use with specified serial device
 *
 * This function will allocate all necessary resources for serial console (RAM, task,
 * synchronization primitives).
 *
 */
void console_init(void);

/**
 * \brief Write to serial console
 *
 * This function can be called from normal task as well as interrupts.
 * When called from interrupts this function will not block. If buffer can't hold all requested
 * data some data will be dropped and will never leave UART.
 * When called from a task, this function can block if there is no space in buffer to hold data.
 *
 * \param [in] buf pointer to data to send to serial console
 * \param [in] len number of bytes to print
 *
 * \return number of bytes written
 */
int console_write(const char *buf, int len);

/**
 * \brief Read from serial console
 *
 * Call this function to read data from serial console.
 *
 * \param [out] buf pointer to buffer to fill with data from serial console
 * \param [in] len number of bytes to read
 *
 */
int console_read(char *buf, int len);

#else /* !dg_configUSE_CONSOLE_STUBS */

static inline void console_init(void)
{

}

static inline int console_write(const char *buf, int len)
{
        return len;
}

static inline int console_read(char *buf, int len)
{
        return 0;
}

#endif /* !dg_configUSE_CONSOLE_STUBS */

#ifdef __cplusplus
extern }
#endif

#endif /* dg_configUSE_CONSOLE */

#endif /* CONSOLE_H_ */

/**
 * \}
 * \}
 * \}
 */
