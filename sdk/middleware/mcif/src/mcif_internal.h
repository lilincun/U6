/**
 ****************************************************************************************
 * @file mcif_internal.h
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

#ifndef MCIF_INTERNAL_H
#define MCIF_INTERNAL_H

#include "osal.h"
#include "queue.h"

#include "mcif.h"

#define MCIF_ASCII_UNKNOWN_HEADER "\r\nERROR: Unknown command.\r\n  "
#define MCIF_ASCII_HELP "\r\nAvailable commands:\r\n\r\n  "
#define MCIF_ASCII_EINVAL "\r\nERROR: Invalid arguments. Usage:\r\n\r\n  "
#define MCIF_ASCII_DONE_MESSAGE "\r\nOK\r\n"
#define MCIF_ASCII_FLAGS_ARG1_MASK 0x3
#define MCIF_ASCII_FLAGS_ARG2_MASK 0xC

#define MCIF_HALF_DUPLEX_PROTO

struct mcif_client
{
        uint8_t msgid;
        OS_QUEUE txq;
        OS_QUEUE rxq;

};

int mcif_parse_frame(uint8_t rxbyte[], int len, struct mcif_message_s **rxmsg);

void mcif_framing_init(void);
#endif /* MCIF_INTERNAL_H */
