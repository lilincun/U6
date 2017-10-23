/**
 ****************************************************************************************
 *
 * @file ble_peripheral_config.h
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

#ifndef BLE_PERIPHERAL_CONFIG_H_
#define BLE_PERIPHERAL_CONFIG_H_

// enable debug service (see readme.txt for details)
#define CFG_DEBUG_SERVICE    (1)

#define CFG_BAS              (1)     // register BAS service
#define CFG_BAS_MULTIPLE     (1)     // add 2 instances of BAS service
#define CFG_CTS              (1)     // register CTS
#define CFG_DIS              (1)     // register DIS
#define CFG_DIS_FULL         (1)     // add all possible characteristics to DIS
#define CFG_SCPS             (1)     // register ScPS
#define CFG_USER_SERVICE     (1)     // register custom service (using 128-bit UUIDs)

#endif /* BLE_PERIPHERAL_CONFIG_H_ */
