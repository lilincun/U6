/**
 ****************************************************************************************
 *
 * @file bms_config.h
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

#ifndef BMS_CONFIG_H_
#define BMS_CONFIG_H_

// authentication code used for appropriate operations
#define CFG_AUTH_CODE           "bms_auth_code"

/**
 * Delete bond button pin configuration.
 */
#define CFG_TRIGGER_DELETE_BOND_GPIO_PORT       (HW_GPIO_PORT_1)
#define CFG_TRIGGER_DELETE_BOND_GPIO_PIN        (HW_GPIO_PIN_6)

#endif /* BMS_CONFIG_H_ */
