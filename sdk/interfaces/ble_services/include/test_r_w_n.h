/**
 ****************************************************************************************
 *
 * @file lls.h
 *
 * @brief Link Loss Service implementation API
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

#ifndef LLS_H_
#define LLS_H_

#include <stdbool.h>
#include <stdint.h>
#include <ble_service.h>

/**
 * \brief LLS alert callback
 *
 * \param [in] conn_idx         connection index
 * \param [in] address          address of device
 * \param [in] level            alert level set by client
 *
 */
//typedef void (* lls_alert_cb_t) (uint16_t conn_idx, const bd_address_t *address, uint8_t level);

/**
 * \brief Create Link Loss Service instance
 *
 * Function registers instance of Link Loss Service.
 *
 * \p alert_cb is called when link loss occurs.
 *
 * \param [in] alert_cb         alert callback
 *
 * \return service instance
 *
 */
//ble_service_t *test_r_w_n_init(lls_alert_cb_t alert_cb);

ble_service_t *test_r_w_n_init(void);

#endif /* LLS_H_ */
