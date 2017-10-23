/**
 ****************************************************************************************
 *
 * @file wsp_weightscale_config.h
 *
 * @brief Application configuration
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

#ifndef WSP_WEIGHTSCALE_CONFIG_H_
#define WSP_WEIGHTSCALE_CONFIG_H_

#define CFG_MULTIPLE_CLIENTS    (0)     // allow couple of users to connect
#define CFG_UDS_MAX_USERS       (3)     // max number of users in database for UDS
#define CFG_MAX_MEAS_TO_STORE   (25)    // max number of measurements to store for each client who
                                        // is registered and have a proper consent, according to
                                        // specification server should store at least 25 data meas

/**
 * Port and pin configuration for UDS user select.
 */
#define CFG_HW_GPIO_PORT_SELECT_USER        (HW_GPIO_PORT_1)
#define CFG_HW_GPIO_START_PIN_SELECT_USER   (HW_GPIO_PIN_2)

#endif /* WSP_WEIGHTSCALE_CONFIG_H_ */
