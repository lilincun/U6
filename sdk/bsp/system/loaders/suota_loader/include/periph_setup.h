 /**
 ****************************************************************************************
 *
 * @file periph_setup.h
 *
 * @brief Peripheral Setup file.
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
#ifndef _PERIPH_SETUP_H_
#define _PERIPH_SETUP_H_

#define glue(a, b) a##b
#define glue2(a, b) glue(a, b)
#define UART_ID glue2(HW_UART, LOADER_UART)

void periph_init(void);
void periph_deinit(void);

#endif /* _PERIPH_SETUP_H_ */
