/**
 ****************************************************************************************
 *
 * @file config.h
 *
 * @brief Common configuration for application
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

#ifndef CONFIG_H_
#define CONFIG_H_

#include "userconfig.h"

/* define CFG_UART_USE_CRLF when using terminal which does not handle \n alone properly */
#if CFG_UART_USE_CRLF
#define NEWLINE        "\r\n"
#define NEWLINE_SIZE   (2)
#else
#define NEWLINE        "\n"
#define NEWLINE_SIZE   (1)
#endif

#endif /* CONFIG_H_ */
