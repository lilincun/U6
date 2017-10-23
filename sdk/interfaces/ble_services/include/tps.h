/**
 ****************************************************************************************
 *
 * @file tps.h
 *
 * @brief Tx Power Service implementation API
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

#ifndef TPS_H_
#define TPS_H_

#include <stdint.h>
#include "ble_service.h"

/**
 * Register Tx Power Service instance
 *
 * \param [in] level  TX power level
 *
 * \return service instance
 *
 */
ble_service_t *tps_init(int8_t level);

#endif /* TPS_H_ */
