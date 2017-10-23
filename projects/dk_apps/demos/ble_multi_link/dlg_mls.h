/**
 ****************************************************************************************
 *
 * @file dlg_mls.h
 *
 * @brief Dialog Multi-Link Service sample implementation API
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

#ifndef DLG_MLS_H_
#define DLG_MLS_H_

#include "ble_service.h"

typedef void (* bd_addr_write_cb_t) (ble_service_t *svc, uint16_t conn_idx, const bd_address_t *addr);

ble_service_t *dlg_mls_init(bd_addr_write_cb_t addr_write_cb);

#endif /* DLG_MLS_H_ */
